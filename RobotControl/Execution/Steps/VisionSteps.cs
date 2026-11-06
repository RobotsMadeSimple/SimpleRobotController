using Controller.RobotControl.Vision;
using Controller.RobotControl.Vision.Calibration;

namespace Controller.RobotControl.Execution
{
    /// <summary>Cameras and vision: RunVision, CaptureImage, SaveImage.</summary>
    internal static class VisionSteps
    {
        /// <summary>RunVision default timeout when the step sets no WaitTimeoutMs.</summary>
        internal const int DefaultVisionTimeoutMs = 30_000;

        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.RunVision]    = new RunVisionStep();
            r[StepType.CaptureImage] = new CaptureImageStep();
            r[StepType.SaveImage]    = new SaveImageStep();
        }

        private static long NowMs() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        private static string CameraLabel(string cameraId) =>
            string.IsNullOrEmpty(cameraId) ? "default" : cameraId;

        private static byte[]? LatestFrame(ExecutionContext ctx, string cameraId)
        {
            var camera = string.IsNullOrEmpty(cameraId)
                ? ctx.Controller.CameraManager.GetFirstCamera()
                : ctx.Controller.CameraManager.GetCamera(cameraId);
            return camera?.GetLatestFrame();
        }

        // ── RunVision ─────────────────────────────────────────────────────────

        private sealed class RunVisionStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var programId = step.VisionProgramId;
                if (string.IsNullOrEmpty(programId))
                    return ctx.Finish(ProgramStatus.Error, "RunVision step has no vision program selected");

                var vision = ctx.Vision;
                var ctrl   = ctx.Controller;

                if (!vision.Awaiting)
                {
                    // First entry: start the processor and record the trigger.
                    ctrl.VisionManager.StartProgram(programId);
                    vision.ProgramId = programId;
                    vision.StartMs   = NowMs();
                    vision.Awaiting  = true;

                    ctx.Progress.Announce($"Vision → {step.VisionProgramName ?? programId}");
                    return StepOutcome.Yield;
                }

                // Subsequent entries: poll for a fresh result
                var proc = ctrl.VisionManager.GetProcessor(vision.ProgramId!);
                if (proc == null)
                {
                    // Processor was stopped externally — treat as error
                    vision.Awaiting  = false;
                    vision.ProgramId = null;
                    return ctx.Finish(ProgramStatus.Error, $"Vision processor lost for '{step.VisionProgramName}'");
                }

                var result = proc.GetLatestResult();
                if (result == null || result.Timestamp <= vision.StartMs)
                {
                    // No fresh result yet — keep waiting, up to the step's timeout (same
                    // WaitTimeoutMs field as a Wait step; a default applies when unset,
                    // 0 or less waits forever).
                    int timeout = step.WaitTimeoutMs ?? DefaultVisionTimeoutMs;
                    if (timeout > 0 && NowMs() - vision.StartMs >= timeout)
                        return ctx.Finish(ProgramStatus.Error,
                            $"Vision → {step.VisionProgramName ?? programId}: no result within {timeout} ms");
                    return StepOutcome.Yield;
                }

                // Got a fresh result — write output variables, stop processor, advance.
                var outputFrame = ResolveOutputFrame(step, result, ctx, out var frameError);
                if (outputFrame == null)
                {
                    ctrl.VisionManager.StopProgram(vision.ProgramId!);
                    vision.Awaiting  = false;
                    vision.ProgramId = null;
                    return ctx.Finish(ProgramStatus.Error, $"Vision → {step.VisionProgramName ?? programId}: {frameError}");
                }
                VisionStepOutputs.WriteOutputs(step, result, ctx.Vars, outputFrame);

                var snap = proc.GetLatestAnnotated();
                if (snap != null) ctrl.SetProgramVisionSnapshot(vision.ProgramId!, snap);
                ctrl.SetProgramVisionResult(vision.ProgramId!, result);

                ctrl.VisionManager.StopProgram(vision.ProgramId!);
                vision.Awaiting  = false;
                vision.ProgramId = null;
                return StepOutcome.Advance;
            }

            /// <summary>
            /// The step's output frame for this result. Null (with <paramref name="error"/>) when
            /// the frame name is unknown, or "robot" is asked for and the vision program's camera
            /// has no calibration, or one made at a different resolution.
            /// </summary>
            private static VisionOutputFrame? ResolveOutputFrame(ProgramStep step, VisionResult result,
                                                                 ExecutionContext ctx, out string error)
            {
                error = "";
                if (!VisionOutputFrame.TryParse(step.OutputFrame, out var kind))
                {
                    error = $"unknown outputFrame '{step.OutputFrame}' (use pixel, normalized or robot)";
                    return null;
                }
                if (kind != OutputFrameKind.Robot)
                    return new VisionOutputFrame(kind, result.ImageWidth, result.ImageHeight);

                var cameraId = ctx.Controller.VisionManager.GetProgram(step.VisionProgramId!)?.CameraId ?? "";
                var cal = ctx.Controller.CalibrationRepo.Get(cameraId);
                if (cal == null)
                {
                    error = $"camera '{CameraLabel(cameraId)}' is not calibrated, so robot coordinates are not available";
                    return null;
                }
                if (result.ImageWidth > 0 && (result.ImageWidth != cal.ImageWidth || result.ImageHeight != cal.ImageHeight))
                {
                    error = $"camera '{cameraId}' runs at {result.ImageWidth}×{result.ImageHeight} but was calibrated at " +
                            $"{cal.ImageWidth}×{cal.ImageHeight}; calibrate it again";
                    return null;
                }
                return new VisionOutputFrame(kind, cal.ImageWidth, cal.ImageHeight, cal);
            }
        }

        // ── CaptureImage ──────────────────────────────────────────────────────

        private sealed class CaptureImageStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var varName  = step.CaptureImageVariableName;
                var cameraId = step.CaptureImageCameraId ?? "";
                if (string.IsNullOrWhiteSpace(varName))
                    return ctx.Finish(ProgramStatus.Error, "CaptureImage step has no target variable set");
                if (!ctx.Vars.IsImage(varName))
                    return ctx.Finish(ProgramStatus.Error, $"CaptureImage: variable '{varName}' is not an image variable");

                var frameBytes = LatestFrame(ctx, cameraId);
                if (frameBytes == null || frameBytes.Length == 0)
                    return ctx.Finish(ProgramStatus.Error,
                        $"CaptureImage: no frame available from camera '{CameraLabel(cameraId)}'");

                ctx.Vars.SetImage(varName, Convert.ToBase64String(frameBytes));
                return StepOutcome.Advance;
            }
        }

        // ── SaveImage ─────────────────────────────────────────────────────────

        private sealed class SaveImageStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var pathTemplate = step.SaveImagePath ?? "";
                if (string.IsNullOrEmpty(pathTemplate))
                {
                    ctx.Progress.StepStarted(step);
                    return StepOutcome.Advance;
                }

                var resolvedPath = ctx.Vars.Interpolate(pathTemplate);
                if (!Path.IsPathRooted(resolvedPath))
                    resolvedPath = Path.Combine(AppContext.BaseDirectory, resolvedPath);

                var cameraId   = step.SaveImageCameraId ?? "";
                var frameBytes = LatestFrame(ctx, cameraId);
                if (frameBytes == null || frameBytes.Length == 0)
                    return ctx.Finish(ProgramStatus.Error,
                        $"SaveImage: no frame available from camera '{CameraLabel(cameraId)}'");

                try
                {
                    var dir = Path.GetDirectoryName(resolvedPath);
                    if (!string.IsNullOrEmpty(dir))
                        Directory.CreateDirectory(dir);
                    File.WriteAllBytes(resolvedPath, frameBytes);
                }
                catch (Exception ex)
                {
                    return ctx.Finish(ProgramStatus.Error, $"SaveImage failed writing '{resolvedPath}': {ex.Message}");
                }

                ctx.Progress.StepStarted(step);
                return StepOutcome.Advance;
            }
        }
    }

    /// <summary>RunVision's variable writes: passVariable and toolOutputs (docs/vision-tools.md, "RunVision step").</summary>
    internal static class VisionStepOutputs
    {
        internal static void WriteOutputs(ProgramStep step, VisionResult result, VariableScope vars,
                                          VisionOutputFrame? frame = null)
        {
            frame ??= new VisionOutputFrame(OutputFrameKind.Default, result.ImageWidth, result.ImageHeight);

            if (!string.IsNullOrWhiteSpace(step.PassVariable))
                vars.Set(step.PassVariable.Trim().TrimStart('$'), result.Passed ? 1 : 0);

            foreach (var o in step.ToolOutputs ?? [])
            {
                if (o == null || string.IsNullOrWhiteSpace(o.Variable) || string.IsNullOrWhiteSpace(o.Value)) continue;
                WriteToolOutput(result.Tool(o.ToolId), o.Value.Trim(), o.Variable.Trim().TrimStart('$'), vars, frame);
            }
        }

        /// <summary>
        /// One toolOutputs entry. A tool that has no result (disabled, or not in the program)
        /// writes empty lists, passed 0 and found 0, and leaves other values unchanged.
        /// Positions (points, item / pose x and y, values x and y) go through the output frame;
        /// other values (widths, radii, distances) stay in pixels.
        /// </summary>
        private static void WriteToolOutput(ToolResult? tr, string value, string name, VariableScope vars, VisionOutputFrame frame)
        {
            switch (value)
            {
                case "passed":
                    vars.Set(name, tr is { Passed: true } ? 1 : 0);
                    return;
                case "status":
                    vars.SetString(name, tr?.Status ?? ToolStatus.NotRun);
                    return;
                case "cells":
                    vars.SetList(name, ListVar.OfRecords((tr?.Cells ?? []).Select(c => new ObjectRecord
                    {
                        ["row"] = c.Row, ["col"] = c.Col, ["index"] = c.Index, ["value"] = c.Value, ["passed"] = c.Passed ? 1 : 0,
                    })));
                    return;
                case "items":
                    vars.SetList(name, ListVar.OfRecords((tr?.Items ?? []).Select(i => ItemRecord(i, frame))));
                    return;
                case "points":
                    vars.SetList(name, ListVar.OfPoints((tr?.Points ?? []).Select(p =>
                    {
                        var (x, y, z) = frame.FromPixel(p.X, p.Y);
                        return new Vector6Val { X = x, Y = y, Z = z };
                    })));
                    return;
                case "lines":
                    vars.SetList(name, ListVar.OfRecords((tr?.Lines ?? []).Select(l =>
                    {
                        var (x1, y1, _) = frame.FromPixel(l.X1, l.Y1);
                        var (x2, y2, _) = frame.FromPixel(l.X2, l.Y2);
                        return new ObjectRecord { ["x1"] = x1, ["y1"] = y1, ["x2"] = x2, ["y2"] = y2 };
                    })));
                    return;
                case "pose":
                    // A one-record list {x, y, z, angle}, read as $pose[0].x; empty when not found.
                    var poses = new List<ObjectRecord>();
                    if (tr?.Pose is { } pose)
                    {
                        var (x, y, z) = frame.FromPixel(pose.X, pose.Y);
                        poses.Add(new ObjectRecord { ["x"] = x, ["y"] = y, ["z"] = z, ["angle"] = pose.Angle });
                    }
                    vars.SetList(name, ListVar.OfRecords(poses));
                    return;
            }

            if (tr == null)
            {
                if (value == "found") vars.Set(name, 0);
                return;
            }
            if (!tr.Values.TryGetValue(value, out var raw))
            {
                if (value is "found" or "count") vars.Set(name, 0);
                return;
            }
            if (raw is string str) { vars.SetString(name, str); return; }
            if (!tr.TryGetNumber(value, out var num)) return;
            if ((value == "x" || value == "y") && tr.TryGetNumber("x", out var vx) && tr.TryGetNumber("y", out var vy))
            {
                var (cx, cy, _) = frame.FromPixel(vx, vy);
                num = value == "x" ? cx : cy;
            }
            vars.Set(name, num);
        }

        private static ObjectRecord ItemRecord(Dictionary<string, object> item, VisionOutputFrame frame)
        {
            // Numbers only (records are numeric); x/y converted together.
            var rec = new ObjectRecord();
            foreach (var (k, v) in item)
                if (v is double d) rec[k] = d;
                else if (v is int i) rec[k] = i;
                else if (v is float f) rec[k] = f;
                else if (v is long l) rec[k] = l;
            if (rec.TryGetValue("x", out var x) && rec.TryGetValue("y", out var y))
            {
                var (cx, cy, cz) = frame.FromPixel(x, y);
                rec["x"] = cx; rec["y"] = cy;
                if (frame.Kind == OutputFrameKind.Robot) rec["z"] = cz;
            }
            return rec;
        }
    }

    /// <summary>A RunVision step waiting for a fresh inspection result.</summary>
    internal sealed class VisionState
    {
        public bool    Awaiting;
        public long    StartMs;
        public string? ProgramId;

        /// <summary>Stops a processor the run left behind and returns to idle.</summary>
        public void Reset(RobotController controller)
        {
            // A processor left running would keep grabbing and analysing frames
            if (Awaiting && !string.IsNullOrEmpty(ProgramId))
                controller.VisionManager.StopProgram(ProgramId);
            Awaiting  = false;
            StartMs   = 0;
            ProgramId = null;
        }
    }
}
