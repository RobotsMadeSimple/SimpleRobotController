namespace Controller.RobotControl.Execution
{
    /// <summary>Robot motion and motion settings: MoveL/J, JumpL/J, ThreadMove, SetSpeedL/J,
    /// SetBlendRadius, SetTool, SetLocal/ClearLocal, RunHoming.</summary>
    internal static class MotionSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.MoveL]          = MoveStep.Instance;
            r[StepType.MoveJ]          = MoveStep.Instance;
            r[StepType.JumpL]          = JumpStep.Instance;
            r[StepType.JumpJ]          = JumpStep.Instance;
            r[StepType.ThreadMove]     = ThreadMoveStep.Instance;
            r[StepType.SetSpeedL]      = new SetSpeedStep("SpeedS", "AccelS"); // raw; override applied in MoveL
            r[StepType.SetSpeedJ]      = new SetSpeedStep("SpeedJ", "AccelJ"); // raw; override applied in MoveJ
            r[StepType.SetBlendRadius] = new SetBlendRadiusStep();
            r[StepType.SetTool]        = new SetToolStep();
            r[StepType.SetLocal]       = new SetLocalStep();
            r[StepType.ClearLocal]     = new ClearLocalStep();
            r[StepType.RunHoming]      = new RunHomingStep();
        }

        // Effective blend radius for a move: its own override if set, else the program default.
        internal static double EffectiveBlendRadius(ProgramStep step, ExecutionContext ctx) =>
            ctx.Eval.OptionalField(step, "blendRadius", step.BlendRadius) is double r
                ? Math.Max(0, r)
                : ctx.Motion.DefaultBlendRadius;

        // A move only actually blends when blending is on AND it has a non-zero radius.
        // Blend-on with a zero radius would round nothing yet still not stop — a jolt — so
        // it's treated as a normal (stopping) move instead.
        internal static bool EffectivelyBlends(ProgramStep step, ExecutionContext ctx) =>
            (step.Blend ?? false) && EffectiveBlendRadius(step, ctx) > 0;
    }

    // ── MoveL / MoveJ ─────────────────────────────────────────────────────────

    internal sealed class MoveStep : IStepHandler
    {
        public static readonly MoveStep Instance = new();

        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            if (ctx.Motion.AwaitingMove) return StepOutcome.Yield;

            if (!ctx.ResolveMoveTarget(step, out Vector6 target)) return StepOutcome.Finished;

            bool hasToolOffset = MoveTargetResolver.HasToolOffset(step);

            // Blended MoveL run: gather consecutive blendable MoveL steps into one
            // continuous path so the robot rounds the corners instead of stopping.
            if (step.Type == StepType.MoveL && MotionSteps.EffectivelyBlends(step, ctx) && !hasToolOffset)
            {
                if (TryDispatchBlendedRun(step, target, frame, ctx) is { } outcome) return outcome;
            }

            var ev = ctx.Eval;
            var toolOffset = MoveTargetResolver.ResolveToolOffset(step, ev);
            var cmd = new RobotCommand
            {
                CommandType = step.Type == StepType.MoveL ? "MoveL" : "MoveJ",
                X  = target.X,
                Y  = target.Y,
                Z  = target.Z,
                RX = target.RX,
                RY = target.RY,
                RZ = target.RZ,
                // Optional local tool offset applied on top of the active tool
                TX  = toolOffset?.X,
                TY  = toolOffset?.Y,
                TZ  = toolOffset?.Z,
                TRX = toolOffset?.RX,
                TRY = toolOffset?.RY,
                TRZ = toolOffset?.RZ,
                // Raw speed — the global override is applied centrally in MoveL/MoveJ.
                Speed = ev.OptionalField(step, "speed", step.Speed),
                Accel = ev.OptionalField(step, "accel", step.Accel),
                Decel = ev.OptionalField(step, "decel", step.Decel),
                ApplySpeedOverride = true,   // program move — subject to the speed override
            };

            ctx.Motion.Enqueue(cmd);
            ctx.AwaitMove(step, frame);
            return StepOutcome.Yield;
        }

        // Gather a run of consecutive blendable MoveL steps and dispatch one continuous
        // (blended) path. Returns null when there is nothing to blend, so the caller
        // falls back to a normal single move.
        private static StepOutcome? TryDispatchBlendedRun(ProgramStep first, Vector6 firstTarget,
            StepListFrame frame, ExecutionContext ctx)
        {
            var steps     = frame.Steps;
            var run       = new List<ProgramStep> { first };
            var waypoints = new List<Vector6> { firstTarget };

            bool prevBlend = MotionSteps.EffectivelyBlends(first, ctx);
            int j = frame.Index + 1;
            while (prevBlend && j < steps.Count)
            {
                var nxt = steps[j];
                // Only plain MoveL steps without a per-step tool offset can join the path.
                if (nxt.Type != StepType.MoveL || MoveTargetResolver.HasToolOffset(nxt)) break;
                // Resolve relative to the previous waypoint so "current position" moves chain
                // off where the robot actually ends up, not where the blend started.
                if (!ctx.ResolveMoveTarget(nxt, out Vector6 t, waypoints[^1])) return StepOutcome.Finished;
                run.Add(nxt);
                waypoints.Add(t);
                // A blend-on move with a zero radius stops here (terminates the run).
                prevBlend = MotionSteps.EffectivelyBlends(nxt, ctx);
                j++;
            }

            if (run.Count < 2) return null; // only one move — nothing to blend

            // Corner radius at waypoint k comes from the move arriving there; the final
            // waypoint is always an exact stop.
            var radii = new List<double>(run.Count);
            for (int k = 0; k < run.Count; k++)
                radii.Add(k < run.Count - 1 ? MotionSteps.EffectiveBlendRadius(run[k], ctx) : 0);

            var ev = ctx.Eval;
            double? speed = ev.OptionalField(first, "speed", first.Speed);
            double? accel = ev.OptionalField(first, "accel", first.Accel);
            double? decel = ev.OptionalField(first, "decel", first.Decel);

            ctx.Motion.StartContinuous(waypoints, radii, speed, accel, decel);
            ctx.AwaitBlendedRun(run, frame);
            return StepOutcome.Yield;
        }
    }

    // ── JumpL / JumpJ ─────────────────────────────────────────────────────────

    /// <summary>A Jump's legs between ticks: lift, traverse, lower.</summary>
    internal sealed class JumpState
    {
        public int     SubStep;          // 0=idle, 1=lift dispatched, 2=transit dispatched, 3=lower dispatched
        public Vector6 Target   = new();
        public Vector6 StartPos = new();
        public double  ZStart;
        public double  ZEnd;
        public string  CmdType  = "MoveL";
        public double? Speed, Accel, Decel;

        public void Reset()
        {
            SubStep  = 0;
            Target   = new();
            StartPos = new();
            ZStart   = 0;
            ZEnd     = 0;
            CmdType  = "MoveL";
            Speed = Accel = Decel = null;
        }
    }

    internal sealed class JumpStep : IStepHandler
    {
        public static readonly JumpStep Instance = new();

        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            if (ctx.Motion.AwaitingMove) return StepOutcome.Yield;
            var jump = ctx.Jump;
            var ev   = ctx.Eval;

            if (jump.SubStep == 0)
            {
                // Resolve final target via the shared resolver — same as MoveL/MoveJ.
                // Supports PointName, GridPoint, StackPoint, variable points, offsets,
                // per-axis overrides, and the active local. Finish(Error) on failure.
                if (!ctx.ResolveMoveTarget(step, out Vector6 resolvedTarget)) return StepOutcome.Finished;

                double? jumpZStart = ev.OptionalField(step, "jumpZStart", step.JumpZStart);
                double? jumpZEnd   = ev.OptionalField(step, "jumpZEnd",   step.JumpZEnd);
                // The shared JumpZ is only a fallback — evaluated when a side needs it.
                double? jumpZ      = jumpZStart.HasValue && jumpZEnd.HasValue ? null : ev.OptionalField(step, "jumpZ", step.JumpZ);

                if (!jumpZ.HasValue && !jumpZStart.HasValue)
                    return ctx.Finish(ProgramStatus.Error, "Jump step: JumpZ must be set");

                var cur = ctx.LivePosition();
                jump.StartPos = cur;
                jump.Target   = resolvedTarget;
                jump.ZStart   = jumpZStart ?? jumpZ ?? 0;
                jump.ZEnd     = jumpZEnd   ?? jumpZ ?? 0;
                jump.CmdType  = step.Type == StepType.JumpJ ? "MoveJ" : "MoveL";
                jump.Speed    = ev.OptionalField(step, "speed", step.Speed); // override applied in MoveL/MoveJ
                jump.Accel    = ev.OptionalField(step, "accel", step.Accel);
                jump.Decel    = ev.OptionalField(step, "decel", step.Decel);

                // Blended JumpL: run lift → traverse → lower as one continuous path,
                // rounding the two apex corners instead of stopping at each leg. (JumpJ's
                // traverse is a joint move, so it keeps the stepped behaviour for now.)
                if (step.Type == StepType.JumpL && MotionSteps.EffectivelyBlends(step, ctx))
                {
                    double r = MotionSteps.EffectiveBlendRadius(step, ctx);
                    var s = jump.StartPos;
                    var t = jump.Target;
                    var apexUp   = new Vector6(s.X, s.Y, jump.ZStart, s.RX, s.RY, s.RZ);
                    var apexOver = new Vector6(t.X, t.Y, jump.ZEnd,   t.RX, t.RY, t.RZ);
                    ctx.Motion.StartContinuous(
                        new List<Vector6> { apexUp, apexOver, t },
                        new List<double> { r, r, 0 },   // round both apexes; land exactly on target
                        jump.Speed, jump.Accel, jump.Decel);
                    jump.SubStep = 0;
                    ctx.AwaitMove(step, frame);
                    return StepOutcome.Yield;
                }

                jump.SubStep = 1;

                ctx.Motion.Enqueue(new RobotCommand
                {
                    CommandType = "MoveL",
                    X = jump.StartPos.X, Y = jump.StartPos.Y, Z = jump.ZStart,
                    RX = jump.StartPos.RX, RY = jump.StartPos.RY, RZ = jump.StartPos.RZ,
                    Speed = jump.Speed, Accel = jump.Accel, Decel = jump.Decel,
                    ApplySpeedOverride = true,
                });
                ctx.Motion.AwaitingMove = true;
                ctx.Progress.StepStarted(step);
                return StepOutcome.Yield;
            }

            if (jump.SubStep == 1)
            {
                ctx.Motion.Enqueue(new RobotCommand
                {
                    CommandType = jump.CmdType,
                    X = jump.Target.X, Y = jump.Target.Y, Z = jump.ZEnd,
                    RX = jump.Target.RX, RY = jump.Target.RY, RZ = jump.Target.RZ,
                    Speed = jump.Speed, Accel = jump.Accel, Decel = jump.Decel,
                    ApplySpeedOverride = true,
                });
                jump.SubStep = 2;
                ctx.Motion.AwaitingMove = true;
                return StepOutcome.Yield;
            }

            if (jump.SubStep == 2)
            {
                ctx.Motion.Enqueue(new RobotCommand
                {
                    CommandType = "MoveL",
                    X = jump.Target.X, Y = jump.Target.Y, Z = jump.Target.Z,
                    RX = jump.Target.RX, RY = jump.Target.RY, RZ = jump.Target.RZ,
                    Speed = jump.Speed, Accel = jump.Accel, Decel = jump.Decel,
                    ApplySpeedOverride = true,
                });
                jump.SubStep = 3;
                ctx.Motion.AwaitingMove = true;
                return StepOutcome.Yield;
            }

            // SubStep == 3: all three legs complete
            jump.SubStep = 0;
            return StepOutcome.Advance;
        }
    }

    // ── ThreadMove ────────────────────────────────────────────────────────────

    /// <summary>A ThreadMove's queued strokes between ticks.</summary>
    internal sealed class ThreadMoveState
    {
        public int                  SubStep;
        public Queue<RobotCommand>? Queue;

        public void Reset()
        {
            SubStep = 0;
            Queue   = null;
        }
    }

    internal sealed class ThreadMoveStep : IStepHandler
    {
        public static readonly ThreadMoveStep Instance = new();

        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            if (ctx.Motion.AwaitingMove) return StepOutcome.Yield;
            var state = ctx.ThreadMove;

            if (state.SubStep == 0)
            {
                var ev       = ctx.Eval;
                var start    = ctx.LivePosition();
                double dist  = ev.EvalField(step, "threadDistance",  step.ThreadDistance  ?? 0);
                double pitch = ev.EvalField(step, "threadPitch",     step.ThreadPitch     ?? 1);
                bool   peck  = step.ThreadPeck ?? false;
                double peckD = ev.OptionalField(step, "threadPeckDepth", step.ThreadPeckDepth) ?? Math.Abs(dist);
                bool   rev   = step.ThreadReverseOut ?? true;

                if (Math.Abs(pitch) < 0.0001) pitch = 1.0;
                if (peckD <= 0) peckD = Math.Abs(dist);

                double? speed = step.Speed.HasValue ? ev.EvalField(step, "speed", step.Speed ?? 0) : null; // override applied in MoveL
                double? accel = step.Accel;
                double? decel = step.Decel;
                double  sign  = dist >= 0 ? 1.0 : -1.0;
                double  absDist = Math.Abs(dist);

                RobotCommand Move(double dZ, double dRZ) => new RobotCommand
                {
                    CommandType = "MoveL",
                    X  = start.X, Y  = start.Y, Z  = start.Z  + dZ,
                    RX = start.RX, RY = start.RY, RZ = start.RZ + dRZ,
                    Speed = speed, Accel = accel, Decel = decel,
                    ApplySpeedOverride = true,
                };

                var queue = new Queue<RobotCommand>();
                state.Queue = queue;

                if (peck && peckD > 0)
                {
                    // 2x down, 1x up: advance 2*peckD each cycle, retract 1*peckD between cycles
                    double accumulated = 0;
                    while (accumulated < absDist - 0.0001)
                    {
                        double nextDepth = Math.Min(accumulated + peckD * 2, absDist);
                        queue.Enqueue(Move(sign * nextDepth, (sign * nextDepth / pitch) * 360.0));
                        if (nextDepth >= absDist - 0.0001) break;
                        double retractTo = Math.Max(nextDepth - peckD, 0);
                        queue.Enqueue(Move(sign * retractTo, (sign * retractTo / pitch) * 360.0));
                        accumulated = retractTo;
                    }
                }
                else
                {
                    queue.Enqueue(Move(dist, (dist / pitch) * 360.0));
                }

                if (rev)
                    queue.Enqueue(Move(0, 0)); // reverse back to start

                state.SubStep = 1;
                ctx.Progress.StepStarted(step);
            }

            // Dispatch next queued move (or finish if queue empty)
            if (state.Queue == null || state.Queue.Count == 0)
            {
                state.Reset();
                return StepOutcome.Advance;
            }

            ctx.Motion.Enqueue(state.Queue.Dequeue());
            ctx.Motion.AwaitingMove = true;
            return StepOutcome.Yield;
        }
    }

    // ── Settings ──────────────────────────────────────────────────────────────

    internal sealed class SetSpeedStep(string speedCommand, string accelCommand) : IStepHandler
    {
        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            var ev = ctx.Eval;
            double? speed = ev.OptionalField(step, "speed", step.Speed);
            double? accel = ev.OptionalField(step, "accel", step.Accel);
            double? decel = ev.OptionalField(step, "decel", step.Decel);
            if (speed.HasValue)
                ctx.Motion.EnqueueSetting(new RobotCommand { CommandType = speedCommand, Speed = speed });
            if (accel.HasValue || decel.HasValue)
                ctx.Motion.EnqueueSetting(new RobotCommand { CommandType = accelCommand, Accel = accel, Decel = decel });
            return StepOutcome.Advance;
        }
    }

    internal sealed class SetBlendRadiusStep : IStepHandler
    {
        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            ctx.Motion.DefaultBlendRadius = Math.Max(0, ctx.Eval.EvalField(step, "blendRadius", step.BlendRadius ?? 0));
            return StepOutcome.Advance;
        }
    }

    internal sealed class SetToolStep : IStepHandler
    {
        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            ctx.Controller.ApplyTool(step.ToolName);
            return StepOutcome.Advance;
        }
    }

    internal sealed class SetLocalStep : IStepHandler
    {
        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            if (string.IsNullOrEmpty(step.LocalName))
            {
                ctx.ActiveLocal = null;
            }
            else
            {
                var local = ctx.Locals.Get(step.LocalName);
                if (local is null) return ctx.Finish(ProgramStatus.Error, $"Local not found: {step.LocalName}");
                ctx.ActiveLocal = new Vector6(local.X, local.Y, local.Z, local.RX, local.RY, local.RZ);
            }
            return StepOutcome.Advance;
        }
    }

    internal sealed class ClearLocalStep : IStepHandler
    {
        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            ctx.ActiveLocal = null;
            return StepOutcome.Advance;
        }
    }

    // ── RunHoming ─────────────────────────────────────────────────────────────

    internal sealed class HomingState
    {
        public bool Triggered;
        public bool StartedMoving;

        public void Reset()
        {
            Triggered     = false;
            StartedMoving = false;
        }
    }

    internal sealed class RunHomingStep : IStepHandler
    {
        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            var homing = ctx.Homing;
            if (!homing.Triggered)
            {
                ctx.Controller.TriggerHoming();
                homing.Triggered = true;
                return StepOutcome.Yield;
            }
            // Wait for homing to start, then wait for it to finish
            if (ctx.Controller.HomingState == "WaitingForStart")
            {
                // Either homing completed or hasn't started yet. Only advance after the
                // state has been seen leaving WaitingForStart at least once.
                if (homing.StartedMoving)
                {
                    homing.Reset();
                    return StepOutcome.Advance;
                }
                // else: still waiting for homing state machine to pick up the trigger
            }
            else
            {
                homing.StartedMoving = true;
            }
            return StepOutcome.Yield;
        }
    }
}
