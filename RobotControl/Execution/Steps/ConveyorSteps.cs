using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Execution
{
    /// <summary>Conveyor tracking (docs/conveyor-tracking.md): ConveyorTrack, ConveyorEndTracking,
    /// ConveyorAddPart, ConveyorClearQueue.</summary>
    internal static class ConveyorSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.ConveyorTrack]       = new TrackStep();
            r[StepType.ConveyorEndTracking] = new EndTrackingStep();
            r[StepType.ConveyorAddPart]     = new AddPartStep();
            r[StepType.ConveyorClearQueue]  = new ClearQueueStep();
        }

        private static long NowMs() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        /// <summary>The step's conveyor (by stable id), or null after ending the program with an error.</summary>
        private static Conveyors.Conveyor? Resolve(ProgramStep step, ExecutionContext ctx, out StepOutcome failed)
        {
            failed = StepOutcome.Finished;
            var svc = ctx.Controller.Conveyors;
            if (string.IsNullOrWhiteSpace(step.ConveyorId))
            {
                ctx.Finish(ProgramStatus.Error, $"{ProgressReporter.StepDescription(step)}: no conveyor selected");
                return null;
            }
            var c = svc?.Get(step.ConveyorId);
            if (c == null)
            {
                ctx.Finish(ProgramStatus.Error, $"{ProgressReporter.StepDescription(step)}: conveyor not found: {step.ConveyorId}");
                return null;
            }
            return c;
        }

        // ── ConveyorTrack ─────────────────────────────────────────────────────

        /// <summary>
        /// Picks the oldest queued part inside the window and starts tracking it. With trackWait
        /// (the default) it waits for one to enter the window — up to trackTimeoutMs, then the
        /// catchable conveyorTimeout. Without it, no part in the window = carry on untracked
        /// (<c>$conveyor.&lt;id&gt;.tracking</c> reads 0).
        /// </summary>
        private sealed class TrackStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var c = Resolve(step, ctx, out var failed);
                if (c == null) return failed;
                var svc   = ctx.Controller.Conveyors;
                var state = ctx.Conveyor;
                if (!state.Waiting)
                {
                    state.Waiting = true;
                    state.StartMs = NowMs();
                    ctx.Progress.StepStarted(step);
                }

                // The belt must be slower than the robot can move, or it could never catch the part.
                double speed = Math.Abs(svc.SpeedOf(c.Id));
                double limit = ctx.Controller.TrackingSpeedLimit;
                if (speed > limit + 1e-6)
                {
                    state.Reset();
                    return ctx.Fail(ErrorCodes.ConveyorSpeed,
                        $"{ProgressReporter.StepDescription(step)}: conveyor '{c.Name}' runs at {speed:0.#} mm/s — faster than the robot's linear speed ({limit:0.#} mm/s)");
                }

                var part = svc.OldestInWindow(c.Id);
                if (part == null)
                {
                    if (!(step.TrackWait ?? true))
                    {
                        state.Reset();
                        return StepOutcome.Advance;
                    }
                    int timeout = step.TrackTimeoutMs ?? 0;
                    if (timeout > 0 && NowMs() - state.StartMs >= timeout)
                    {
                        state.Reset();
                        return ctx.Fail(ErrorCodes.ConveyorTimeout,
                            $"{ProgressReporter.StepDescription(step)}: no part entered the window of conveyor '{c.Name}' within {timeout} ms");
                    }
                    return StepOutcome.Yield;
                }

                state.Reset();
                svc.StartTracking(c.Id, part.Id, ctx);
                return StepOutcome.Advance;
            }
        }

        // ── ConveyorEndTracking ───────────────────────────────────────────────

        /// <summary>Stops tracking (the robot stays where it is; later moves are normal). The part
        /// counts as picked, or stays on the belt with conveyorRelease. Nothing tracked = no-op.</summary>
        private sealed class EndTrackingStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var svc = ctx.Controller.Conveyors;
                if (svc?.Tracking is { } t && ReferenceEquals(t.Owner, ctx))
                    svc.EndTracking(step.ConveyorRelease ?? false, string.IsNullOrWhiteSpace(step.ConveyorId) ? null : step.ConveyorId);
                ctx.Motion.PlanTravel = null;
                return StepOutcome.Advance;
            }
        }

        // ── ConveyorAddPart ───────────────────────────────────────────────────

        /// <summary>A software trigger: queue a part at conveyor X = partOffsetMm now (e.g. from a
        /// vision result).</summary>
        private sealed class AddPartStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var c = Resolve(step, ctx, out var failed);
                if (c == null) return failed;
                double x = ctx.Eval.EvalField(step, "partOffsetMm", step.PartOffsetMm ?? 0);
                if (!double.IsFinite(x))
                    return ctx.Finish(ProgramStatus.Error, $"{ProgressReporter.StepDescription(step)}: part offset is not a number");
                ctx.Controller.Conveyors.AddPart(c.Id, x);
                return StepOutcome.Advance;
            }
        }

        // ── ConveyorClearQueue ────────────────────────────────────────────────

        private sealed class ClearQueueStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var c = Resolve(step, ctx, out var failed);
                if (c == null) return failed;
                ctx.Controller.Conveyors.ClearQueue(c.Id);
                return StepOutcome.Advance;
            }
        }
    }

    /// <summary>ConveyorTrack's wait between ticks.</summary>
    internal sealed class ConveyorStepState
    {
        public bool Waiting;
        public long StartMs;

        public void Reset()
        {
            Waiting = false;
            StartMs = 0;
        }
    }
}
