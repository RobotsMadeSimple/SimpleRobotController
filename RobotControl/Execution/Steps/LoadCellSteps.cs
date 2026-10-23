using Controller.RobotControl.Io;

namespace Controller.RobotControl.Execution
{
    /// <summary>Load cell (HX711) program steps: Tare (zero) and ReadLoadCell (into a variable).</summary>
    internal static class LoadCellSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.Tare]          = new TareStep();
            r[StepType.ReadLoadCell]  = new ReadLoadCellStep();
            r[StepType.MoveWithForce] = new MoveWithForceStep();
        }

        private static long NowMs() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        /// <summary>Zero the selected load cell (capture its current raw count as the tare offset).</summary>
        private sealed class TareStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (!string.IsNullOrEmpty(step.LoadCellDeviceId) && step.LoadCellPin is int pin)
                    ctx.Controller.NanoManager.TareLoadCell(step.LoadCellDeviceId, pin);
                return StepOutcome.Advance;
            }
        }

        /// <summary>Read the load cell's weight into a program variable, in the step's chosen unit (default grams).</summary>
        private sealed class ReadLoadCellStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (!string.IsNullOrEmpty(step.VariableName) &&
                    !string.IsNullOrEmpty(step.LoadCellDeviceId) && step.LoadCellPin is int pin)
                {
                    double grams = ctx.Controller.NanoManager.GetLoadCellGrams(step.LoadCellDeviceId, pin) ?? 0;
                    ctx.Vars.Set(step.VariableName, LoadCellUnits.GramsToUnit(grams, step.LoadCellUnit));
                }
                return StepOutcome.Advance;
            }
        }

        /// <summary>
        /// Move along an axis while continuously holding a target contact force (position-based
        /// admittance). Phase 1 approaches in the push direction until the force is reached; phase 2
        /// maintains it for a hold time, correcting position as the force drifts. Runs as a loop of
        /// small incremental MoveL moves in the program-executor context (NOT the STB4100 loop).
        /// Safety: a force ceiling triggers an immediate back-off, and max-travel / timeout stop it.
        /// </summary>
        private sealed class MoveWithForceStep : IStepHandler
        {
            // Proportional step: bigger when far from target, tiny near it. Clamped for safety.
            private static double StepMm(double absErrGrams, double targetGrams)
            {
                double frac = Math.Min(1.0, absErrGrams / Math.Max(1.0, Math.Abs(targetGrams)));
                return Math.Clamp(frac * 0.5, 0.02, 0.5);
            }

            private static Vector6 WithAxisDelta(Vector6 c, string axis, double d) => axis switch
            {
                "X"  => new Vector6(c.X + d, c.Y, c.Z, c.RX, c.RY, c.RZ),
                "Y"  => new Vector6(c.X, c.Y + d, c.Z, c.RX, c.RY, c.RZ),
                "RX" => new Vector6(c.X, c.Y, c.Z, c.RX + d, c.RY, c.RZ),
                "RY" => new Vector6(c.X, c.Y, c.Z, c.RX, c.RY + d, c.RZ),
                "RZ" => new Vector6(c.X, c.Y, c.Z, c.RX, c.RY, c.RZ + d),
                _    => new Vector6(c.X, c.Y, c.Z + d, c.RX, c.RY, c.RZ), // "Z" / default
            };

            private static double AxisVal(Vector6 c, string axis) => axis switch
            {
                "X" => c.X, "Y" => c.Y, "RX" => c.RX, "RY" => c.RY, "RZ" => c.RZ, _ => c.Z,
            };

            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                // Wait for any in-flight corrective move to finish before reading force + deciding again.
                if (ctx.Motion.AwaitingMove) return StepOutcome.Yield;

                var fs    = ctx.Force;
                var axis  = (step.ForceAxis ?? "Z").ToUpperInvariant();
                int dir   = Math.Sign(step.ForceDirection ?? -1); if (dir == 0) dir = -1;
                double target    = step.ForceTargetGrams ?? 0;
                double tol       = step.ForceToleranceGrams ?? Math.Max(5, Math.Abs(target) * 0.05);
                double maxTravel = step.ForceMaxTravelMm ?? 10;
                double speed     = step.ForceApproachSpeed ?? 5;
                int    holdMs    = step.ForceHoldMs ?? 0;
                int    timeoutMs = step.ForceTimeoutMs ?? 20000;
                double ceiling   = Math.Abs(target) + Math.Max(tol * 4, Math.Abs(target)); // ~2x target

                // Nothing to control without a sensor + a positive target.
                double? gOpt = string.IsNullOrEmpty(step.LoadCellDeviceId) || step.LoadCellPin is not int lcPin
                    ? null : ctx.Controller.NanoManager.GetLoadCellGrams(step.LoadCellDeviceId, lcPin);
                if (gOpt == null || target <= 0) { fs.Reset(); return StepOutcome.Advance; }
                double force = gOpt.Value;
                var cur = ctx.LivePosition();

                long now = NowMs();
                if (fs.Phase == 0)
                {
                    fs.StartAxis = AxisVal(cur, axis);
                    fs.StartMs   = now;
                    fs.Phase     = 1;
                    ctx.Progress.StepStarted(step);
                }

                if (now - fs.StartMs >= timeoutMs)
                {
                    fs.Reset();
                    return ctx.Finish(ProgramStatus.Error, "Move with force: timed out before completing");
                }

                double depth = Math.Abs(AxisVal(cur, axis) - fs.StartAxis); // how far we've pushed along the axis
                double error = target - force;                              // >0 need more force, <0 too much

                // Safety: over the force ceiling — relieve immediately by backing off, whatever the phase.
                if (force > ceiling)
                {
                    ctx.Motion.Enqueue(MoveCmd(WithAxisDelta(cur, axis, -dir * 0.1), speed));
                    ctx.Motion.AwaitingMove = true;
                    return StepOutcome.Yield;
                }

                if (fs.Phase == 1) // approach
                {
                    if (force >= target - tol) { fs.Phase = 2; fs.HoldStartMs = now; }
                    else
                    {
                        if (depth >= maxTravel)
                        {
                            fs.Reset();
                            return ctx.Finish(ProgramStatus.Error, "Move with force: target force not reached within max travel");
                        }
                        ctx.Motion.Enqueue(MoveCmd(WithAxisDelta(cur, axis, dir * StepMm(Math.Abs(error), target)), speed));
                        ctx.Motion.AwaitingMove = true;
                        return StepOutcome.Yield;
                    }
                }

                if (fs.Phase == 2) // hold
                {
                    if (holdMs <= 0 || now - fs.HoldStartMs >= holdMs) { fs.Reset(); return StepOutcome.Advance; }
                    if (Math.Abs(error) <= tol) return StepOutcome.Yield; // within tolerance — maintain

                    if (error > 0 && depth >= maxTravel)
                    {
                        fs.Reset();
                        return ctx.Finish(ProgramStatus.Error, "Move with force: lost contact beyond max travel");
                    }
                    int sign = error > 0 ? 1 : -1; // push further, or back off
                    ctx.Motion.Enqueue(MoveCmd(WithAxisDelta(cur, axis, dir * sign * StepMm(Math.Abs(error), target)), speed));
                    ctx.Motion.AwaitingMove = true;
                    return StepOutcome.Yield;
                }

                return StepOutcome.Yield;
            }

            private static RobotCommand MoveCmd(Vector6 t, double speed) => new()
            {
                CommandType = "MoveL",
                X = t.X, Y = t.Y, Z = t.Z, RX = t.RX, RY = t.RY, RZ = t.RZ,
                Speed = speed,
                ApplySpeedOverride = true,
            };
        }
    }

    /// <summary>Multi-tick state for a MoveWithForce step (one runs at a time; shared on the context).</summary>
    internal sealed class ForceMoveState
    {
        public int    Phase;       // 0 = not started, 1 = approaching, 2 = holding
        public double StartAxis;   // control-axis value at start (for the travel/depth limit)
        public long   StartMs;     // overall start (timeout)
        public long   HoldStartMs; // when the hold phase began
        public void Reset() { Phase = 0; }
    }
}
