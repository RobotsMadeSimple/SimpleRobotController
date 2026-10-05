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
                    // First entry: start the processor (applying any zone override so the
                    // analysis and debug frame run in the selected zone) and record the trigger.
                    ctrl.VisionManager.StartProgram(programId, ResolveZoneOverride(step, ctx));
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
                if (result == null || result.TimestampMs <= vision.StartMs)
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
                // The zone override (if any) was applied to the processor at start, so every
                // inspection ran in the selected zone and all of its outputs are relevant.
                var outputFrame = ResolveOutputFrame(step, result, ctx, out var frameError);
                if (outputFrame == null)
                {
                    ctrl.VisionManager.StopProgram(vision.ProgramId!);
                    vision.Awaiting  = false;
                    vision.ProgramId = null;
                    return ctx.Finish(ProgramStatus.Error, $"Vision → {step.VisionProgramName ?? programId}: {frameError}");
                }
                WriteOutputs(step, result, ctx.Vars, outputFrame);

                var snap = proc.GetLatestAnnotated();
                if (snap != null) ctrl.SetProgramVisionSnapshot(vision.ProgramId!, snap);
                ctrl.SetProgramVisionResult(vision.ProgramId!, result);

                ctrl.VisionManager.StopProgram(vision.ProgramId!);
                vision.Awaiting  = false;
                vision.ProgramId = null;
                return StepOutcome.Advance;
            }

            /// <summary>
            /// Resolves the runtime zone override for a RunVision step: a variable (1-based
            /// index into the program's zones) takes priority over a fixed zone id. Returns
            /// null when no override is set (each inspection uses its own configured zone).
            /// </summary>
            private static string? ResolveZoneOverride(ProgramStep step, ExecutionContext ctx)
            {
                string? zoneId = step.VisionZoneId;
                if (!string.IsNullOrEmpty(step.VisionZoneVar) &&
                    ctx.Vars.TryGetLocal(step.VisionZoneVar, out var zoneIdxVal))
                {
                    var vp = ctx.Controller.VisionManager.GetProgram(step.VisionProgramId!);
                    if (vp != null)
                    {
                        int zoneIdx = (int)zoneIdxVal - 1; // 1-based → 0-based
                        zoneId = (zoneIdx >= 0 && zoneIdx < vp.Zones.Count) ? vp.Zones[zoneIdx].Id : null;
                    }
                }
                return zoneId;
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

            private static void WriteOutputs(ProgramStep step, VisionResult result, VariableScope vars,
                                              VisionOutputFrame? frame = null)
            {
                frame ??= new VisionOutputFrame(OutputFrameKind.Default, result.ImageWidth, result.ImageHeight);

                foreach (var output in step.VisionOutputs ?? [])
                {
                    var ir = result.Inspections.Find(i => i.InspectionId == output.InspectionId);
                    if (ir == null) continue;

                    if (!string.IsNullOrEmpty(output.CountVar))
                        vars.Set(output.CountVar, ir.Blobs.Count);

                    if (!string.IsNullOrEmpty(output.PointsVar))
                    {
                        vars.SetList(output.PointsVar, ListVar.OfPoints(
                            ir.Blobs.Select(b =>
                            {
                                var (x, y, z) = frame.FromPixel(b.X, b.Y);
                                return new Vector6Val { X = x, Y = y, Z = z };
                            })));
                    }

                    if (!string.IsNullOrEmpty(output.DetectedVar))
                        vars.Set(output.DetectedVar, ir.Blobs.Count > 0 ? 1 : 0);
                }

                foreach (var output in step.ColorOutputs ?? [])
                {
                    var cr = result.ColorResults.Find(r => r.InspectionId == output.InspectionId);
                    if (cr == null) continue;

                    if (!string.IsNullOrEmpty(output.CoverageVar))
                        vars.Set(output.CoverageVar, cr.Coverage);

                    if (!string.IsNullOrEmpty(output.PassedVar))
                        vars.Set(output.PassedVar, cr.Passed ? 1 : 0);

                    // Grid cells. An ungridded zone has no cells, so the variable is emptied
                    // rather than left holding the previous run's grid.
                    if (!string.IsNullOrEmpty(output.CellsVar))
                    {
                        vars.SetList(output.CellsVar, ListVar.OfRecords(
                            (cr.Cells ?? []).Select(cell => new ObjectRecord
                            {
                                ["row"]      = cell.Row,
                                ["col"]      = cell.Col,
                                ["index"]    = cell.Index,
                                ["coverage"] = cell.Coverage,
                                ["passed"]   = cell.Passed ? 1 : 0,
                            })));
                    }

                    if (!string.IsNullOrEmpty(output.CellsPassedVar))
                        vars.Set(output.CellsPassedVar, cr.CellsPassed ?? 0);
                }

                foreach (var output in step.PolygonOutputs ?? [])
                {
                    var pr = result.PolygonResults.Find(r => r.InspectionId == output.InspectionId);
                    if (pr == null) continue;

                    if (!string.IsNullOrEmpty(output.CountVar))
                        vars.Set(output.CountVar, pr.Count);

                    if (!string.IsNullOrEmpty(output.FoundVar))
                        vars.Set(output.FoundVar, pr.Found ? 1 : 0);

                    if (!string.IsNullOrEmpty(output.AngleVar))
                        vars.Set(output.AngleVar, pr.Angle);

                    var (pcx, pcy, _) = frame.FromNormalized(pr.CenterX, pr.CenterY);
                    if (!string.IsNullOrEmpty(output.CenterXVar))
                        vars.Set(output.CenterXVar, pcx);

                    if (!string.IsNullOrEmpty(output.CenterYVar))
                        vars.Set(output.CenterYVar, pcy);
                }

                foreach (var output in step.ArucoOutputs ?? [])
                {
                    var ar = result.ArucoResults.Find(r => r.InspectionId == output.InspectionId);
                    if (ar == null) continue;

                    if (!string.IsNullOrEmpty(output.CountVar))
                        vars.Set(output.CountVar, ar.Count);

                    if (!string.IsNullOrEmpty(output.FoundVar))
                        vars.Set(output.FoundVar, ar.Found ? 1 : 0);

                    var first = ar.Markers.FirstOrDefault();
                    if (first != null)
                    {
                        if (!string.IsNullOrEmpty(output.FirstIdVar))
                            vars.Set(output.FirstIdVar, first.MarkerId);

                        var (acx, acy, _) = frame.FromNormalized(first.CenterX, first.CenterY);
                        if (!string.IsNullOrEmpty(output.FirstCenterXVar))
                            vars.Set(output.FirstCenterXVar, acx);

                        if (!string.IsNullOrEmpty(output.FirstCenterYVar))
                            vars.Set(output.FirstCenterYVar, acy);
                    }
                }
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
