using System.Globalization;

namespace Controller.RobotControl.Execution
{
    /// <summary>CNC blocks: CncProgram expands a toolpath spec into moves at run time.</summary>
    internal static class CncSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.CncProgram] = new CncProgramStep();
        }

        private sealed class CncProgramStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                // "current" origin mode: the robot's position right now becomes the
                // toolpath origin. Override targets get the active local frame applied
                // after resolution, so the anchor is expressed in the LOCAL frame —
                // the re-applied frame then cancels out and moves land relative to the
                // physical position, rotated with whatever local frame is active.
                Vector6? anchor = null;
                if (step.CncSpec?.OriginMode == "current")
                {
                    var pos = ctx.LivePosition();
                    var loc = ctx.ActiveLocal;
                    anchor = loc == null ? pos : LocalFrame.Inverse(loc, pos);
                }

                // Structural spec expressions (drill depth / peck) evaluate against
                // the program's CURRENT variables — the state when the block starts.
                double ResolveSpecExpr(string key, double fallback)
                {
                    var e = step.CncSpec?.Expressions;
                    if (e == null || !e.TryGetValue(key, out var expr) || string.IsNullOrWhiteSpace(expr))
                        return fallback;
                    try { return ctx.Eval.Evaluate(expr); }
                    catch (UnknownVariableException) { throw; }
                    catch { return fallback; }
                }

                // Prefer runtime generation from the spec; fall back to baked steps
                // from older app versions.
                var innerSteps = step.CncSpec != null
                    ? CncStepGenerator.Generate(step.CncSpec, anchor, ResolveSpecExpr)
                    : step.CncProgramSteps ?? new();
                if (innerSteps.Count == 0) return StepOutcome.Advance;

                // Publish the resolved XY toolpath — anchor AND active local frame
                // applied, so the monitor preview shows exactly where the robot will
                // move in world coordinates. Cleared when the block's frame pops.
                if (step.CncSpec is { } spec2)
                {
                    double ax = anchor?.X ?? 0, ay = anchor?.Y ?? 0;
                    var loc = ctx.ActiveLocal;
                    // Transform at the tool-down plane: with a tilted frame the world
                    // XY of a path point depends on its local Z.
                    double planeZ = ResolveSpecExpr("activeZ", spec2.ActiveZ ?? 0) + (anchor?.Z ?? 0);
                    (double wx, double wy) ToWorld(double x, double y)
                    {
                        double px = x + ax, py = y + ay;
                        if (loc == null) return (px, py);
                        var w = LocalFrame.Apply(loc, new Vector6(px, py, planeZ, 0, 0, 0));
                        return (w.X, w.Y);
                    }

                    var paths = new List<List<double>>();
                    foreach (var flat in spec2.Paths ?? [])
                    {
                        if (flat is not { Count: >= 4 }) continue;
                        var world = new List<double>(flat.Count);
                        for (int i = 0; i + 1 < flat.Count; i += 2)
                        {
                            var (wx, wy) = ToWorld(flat[i], flat[i + 1]);
                            world.Add(wx);
                            world.Add(wy);
                        }
                        paths.Add(world);
                    }
                    var holes = (spec2.Holes ?? []).Select(h =>
                    {
                        var (wx, wy) = ToWorld(h.X, h.Y);
                        return new CncHole { X = wx, Y = wy };
                    }).ToList();
                    ctx.Controller.ActiveCncToolpath = new RobotController.CncToolpathInfo(ctx.Program!.Name, paths, holes);
                }

                // The block's steps run in their own (non-loop) frame; the CncProgram step
                // itself completes on Advance and counts separately.
                ctx.Frames.Push(StepListFrame.Cnc(innerSteps));
                return StepOutcome.Advance;
            }
        }
    }

    /// <summary>Expands a CNC toolpath spec into executable steps.</summary>
    internal static class CncStepGenerator
    {
        /// <summary>
        /// Expand a CNC toolpath spec into executable steps: drill or thread each
        /// hole (per HoleOp), then follow each contour as travel → plunge →
        /// blended MoveL chain → retract. The retract is unblended so the
        /// continuous active run stops exactly on the contour's last point before
        /// the tool lifts. When <paramref name="anchor"/> is set (origin mode
        /// "current"), every X/Y/Z is shifted so the anchor position acts as the
        /// origin. <paramref name="resolve"/> evaluates $variable expressions for
        /// STRUCTURAL values (drill depth / peck depth decide how many steps are
        /// generated) against the program's variables at block start; when null,
        /// the numeric spec values are used.
        /// </summary>
        public static List<ProgramStep> Generate(CncSpec spec, Vector6? anchor = null,
            Func<string, double, double>? resolve = null)
        {
            var steps = new List<ProgramStep>();
            string NewId() => Guid.NewGuid().ToString("N");

            double ax = anchor?.X ?? 0, ay = anchor?.Y ?? 0, az = anchor?.Z ?? 0;
            double Resolve(string key, double fallback) => resolve?.Invoke(key, fallback) ?? fallback;

            // Spec-level $variable expressions map onto per-step expression keys,
            // so the existing EvalField machinery resolves them at run time. The
            // dictionaries are shared across generated steps (read-only there).
            // With an anchor, Z expressions are wrapped as "(expr) + anchorZ" so
            // they stay relative to the position the block started from.
            var ex = spec.Expressions ?? new Dictionary<string, string>();
            Dictionary<string, string>? MapExprs(params (string from, string to)[] pairs)
            {
                Dictionary<string, string>? d = null;
                foreach (var (from, to) in pairs)
                    if (ex.TryGetValue(from, out var e) && !string.IsNullOrWhiteSpace(e))
                    {
                        if (to == "overrideZ" && anchor != null)
                            e = $"({e}) + {az.ToString(CultureInfo.InvariantCulture)}";
                        (d ??= new())[to] = e;
                    }
                return d;
            }

            var travelExprs = MapExprs(("safeZ", "overrideZ"),
                ("travelSpeed", "speed"), ("travelAccel", "accel"), ("travelDecel", "decel"));
            var activeExprs = MapExprs(("activeZ", "overrideZ"),
                ("activeSpeed", "speed"), ("activeAccel", "accel"), ("activeDecel", "decel"));
            var activeBlendExprs = MapExprs(("activeZ", "overrideZ"),
                ("activeSpeed", "speed"), ("activeAccel", "accel"), ("activeDecel", "decel"),
                ("blendRadius", "blendRadius"));
            // Dynamics only — drill plunges set their own numeric Z targets
            var activeDynExprs = MapExprs(
                ("activeSpeed", "speed"), ("activeAccel", "accel"), ("activeDecel", "decel"));
            var threadExprs = MapExprs(("holeDepth", "threadDistance"), ("threadPitch", "threadPitch"),
                ("holePeckDepth", "threadPeckDepth"));

            bool drill = spec.HoleOp == "drill";
            foreach (var h in spec.Holes ?? [])
            {
                steps.Add(new ProgramStep
                {
                    Id = NewId(),
                    Type = StepType.MoveL,
                    Name = $"Approach ({h.X:0.0}, {h.Y:0.0})",
                    OverrideX = h.X + ax, OverrideY = h.Y + ay, OverrideZ = spec.SafeZ + az,
                    Speed = spec.TravelSpeed, Accel = spec.TravelAccel, Decel = spec.TravelDecel,
                    Expressions = travelExprs,
                });

                if (drill)
                {
                    // Straight plunge(s) to depth. Depth/peck are structural (they
                    // decide the pass layout), so expressions resolve at block start.
                    double depth = Resolve("holeDepth", spec.HoleDepth ?? 0);
                    double peckD = spec.HolePeck == true
                        ? Math.Abs(Resolve("holePeckDepth", spec.HolePeckDepth ?? 0))
                        : 0;
                    double sign  = depth >= 0 ? 1 : -1;
                    double total = Math.Abs(depth);

                    var passDepths = new List<double>();
                    if (peckD > 0.0001 && total > peckD)
                        for (double d = peckD; d < total; d += peckD) passDepths.Add(d);
                    passDepths.Add(total);

                    foreach (var d in passDepths)
                    {
                        steps.Add(new ProgramStep
                        {
                            Id = NewId(), Type = StepType.MoveL,
                            OverrideX = h.X + ax, OverrideY = h.Y + ay,
                            OverrideZ = spec.SafeZ + az + sign * d,
                            Speed = spec.ActiveSpeed, Accel = spec.ActiveAccel, Decel = spec.ActiveDecel,
                            Expressions = activeDynExprs,
                        });
                        steps.Add(new ProgramStep
                        {
                            Id = NewId(), Type = StepType.MoveL,
                            OverrideX = h.X + ax, OverrideY = h.Y + ay, OverrideZ = spec.SafeZ + az,
                            Speed = spec.TravelSpeed, Accel = spec.TravelAccel, Decel = spec.TravelDecel,
                            Expressions = travelExprs,
                        });
                    }
                }
                else
                {
                    steps.Add(new ProgramStep
                    {
                        Id = NewId(),
                        Type = StepType.ThreadMove,
                        ThreadDistance   = spec.HoleDepth,
                        ThreadPitch      = spec.ThreadPitch,
                        ThreadPeck       = spec.HolePeck,
                        ThreadPeckDepth  = spec.HolePeck == true ? spec.HolePeckDepth : null,
                        ThreadReverseOut = spec.ThreadReverseOut,
                        Expressions      = threadExprs,
                    });
                }
            }

            double activeZ  = (spec.ActiveZ ?? 0) + az;
            double safeZ  = spec.SafeZ + az;
            double blendR = spec.BlendRadius ?? 0;
            bool   blend  = blendR > 0 || ex.ContainsKey("blendRadius");
            int    cIdx   = 0;
            foreach (var flat in spec.Paths ?? [])
            {
                cIdx++;
                if (flat == null || flat.Count < 4) continue;
                int nPts = flat.Count / 2;
                double sx = flat[0] + ax, sy = flat[1] + ay;

                steps.Add(new ProgramStep
                {
                    Id = NewId(), Type = StepType.MoveL,
                    Name = $"Contour {cIdx} ({nPts} pts)",
                    OverrideX = sx, OverrideY = sy, OverrideZ = safeZ,
                    Speed = spec.TravelSpeed, Accel = spec.TravelAccel, Decel = spec.TravelDecel,
                    Expressions = travelExprs,
                });
                steps.Add(new ProgramStep
                {
                    Id = NewId(), Type = StepType.MoveL,
                    OverrideX = sx, OverrideY = sy, OverrideZ = activeZ,
                    Speed = spec.ActiveSpeed, Accel = spec.ActiveAccel, Decel = spec.ActiveDecel,
                    Expressions = activeExprs,
                });
                for (int i = 1; i < nPts; i++)
                {
                    steps.Add(new ProgramStep
                    {
                        Id = NewId(), Type = StepType.MoveL,
                        OverrideX = flat[2 * i] + ax, OverrideY = flat[2 * i + 1] + ay, OverrideZ = activeZ,
                        Speed = spec.ActiveSpeed, Accel = spec.ActiveAccel, Decel = spec.ActiveDecel,
                        Blend = blend ? true : null,
                        BlendRadius = blend ? blendR : null,
                        Expressions = blend ? activeBlendExprs : activeExprs,
                    });
                }
                steps.Add(new ProgramStep
                {
                    Id = NewId(), Type = StepType.MoveL,
                    OverrideX = flat[2 * (nPts - 1)] + ax, OverrideY = flat[2 * (nPts - 1) + 1] + ay, OverrideZ = safeZ,
                    Speed = spec.TravelSpeed, Accel = spec.TravelAccel, Decel = spec.TravelDecel,
                    Expressions = travelExprs,
                });
            }

            return steps;
        }
    }
}
