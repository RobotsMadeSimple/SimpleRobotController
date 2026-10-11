namespace Controller.RobotControl.Execution
{
    using Controller.RobotControl.AuxAxis;

    /// <summary>Auxiliary axes: AuxMove, AuxContinuous, AuxStop, AuxEnable, plus the role-specific
    /// GripperAction, ConveyorAction, RotaryIndex (which reuse the axis's configured
    /// role action distance/velocity).</summary>
    internal static class AuxSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.AuxMove]        = new AuxMoveStep();
            r[StepType.AuxContinuous]  = new AuxContinuousStep();
            r[StepType.AuxStop]        = new AuxStopStep();
            r[StepType.AuxEnable]      = new AuxEnableStep();
            r[StepType.GripperAction]  = new GripperActionStep();
            r[StepType.ConveyorAction] = new ConveyorActionStep();
            r[StepType.RotaryIndex]    = new RotaryIndexStep();
        }

        private static string DeviceOrFirst(ProgramStep step, ExecutionContext ctx) =>
            step.AuxDeviceId ?? ctx.Controller.AuxAxisManager.GetFirstDevice()?.Id ?? "";

        // Role action distance (physical unit when the axis has a type, else raw steps) -> steps magnitude.
        private static long ActionStepsMag(AuxAxisChannelConfig? cfg)
        {
            double dist = cfg?.ActionDistance ?? 0;
            double spu  = cfg?.StepsPerUnit() ?? 0;
            return (long)Math.Abs(Math.Round(spu > 0 ? dist * spu : dist));
        }

        private static double ActionVel(AuxAxisChannelConfig? cfg) =>
            (cfg?.ActionVelocity ?? 0) > 0 ? cfg!.ActionVelocity : 1600;

        private sealed class AuxMoveStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var    ctrl      = ctx.Controller;
                var    ev        = ctx.Eval;
                string deviceId  = DeviceOrFirst(step, ctx);
                int    axisIndex = step.AuxAxisIndex ?? 0;

                var axisCfg = ctrl.AuxAxisManager.GetAxisConfig(deviceId, axisIndex);
                double spu   = axisCfg?.StepsPerUnit() ?? 0;

                long   steps;
                double velocity, accel, decel;

                if (!string.IsNullOrEmpty(step.AuxUnit) && step.AuxDistance.HasValue && spu > 0)
                {
                    // Physical-unit mode — convert distance and rates to steps
                    double dist = ev.EvalField(step, "auxDistance", step.AuxDistance.Value);
                    steps    = (long)Math.Round(dist * spu);
                    velocity = ev.EvalField(step, "auxVelocity", step.AuxVelocity ?? 10) * spu;
                    accel    = ev.EvalField(step, "auxAccel",    step.AuxAccel    ?? 50)  * spu;
                    decel    = ev.EvalField(step, "auxDecel",    step.AuxDecel    ?? accel / spu) * spu;
                }
                else
                {
                    steps    = (long)ev.EvalField(step, "auxSteps",    step.AuxSteps    ?? 0);
                    velocity = ev.EvalField(step, "auxVelocity", step.AuxVelocity ?? 1600);
                    accel    = ev.EvalField(step, "auxAccel",    step.AuxAccel    ?? 3200);
                    decel    = ev.EvalField(step, "auxDecel",    step.AuxDecel    ?? accel);
                }

                if (step.AuxAbsolute == true)
                {
                    long currentPos = ctrl.AuxAxisManager.GetPosition(deviceId, axisIndex);
                    steps = steps - currentPos;
                }

                if (steps == 0) return StepOutcome.Advance;

                // StartAuxMove derives direction from the sign of steps (and applies the
                // axis InvertDirection), then moves by the absolute amount. Pass the SIGNED
                // value — pre-abs'ing it here made every move go the same direction
                // regardless of a positive or negative distance.
                ctrl.StartAuxMove(deviceId, axisIndex, steps, velocity, accel, decel);

                bool wait = step.AuxWaitForDone ?? true;
                if (!wait) return StepOutcome.Advance;

                ctx.AwaitAuxMove(step, frame);
                if (Diag.Enabled) Diag.AuxDispatch(ctrl.IsAuxMoving);
                return StepOutcome.Yield;
            }
        }

        private sealed class AuxContinuousStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var    ctrl      = ctx.Controller;
                var    ev        = ctx.Eval;
                string deviceId  = DeviceOrFirst(step, ctx);
                int    axisIndex = step.AuxAxisIndex ?? 0;

                var axisCfg = ctrl.AuxAxisManager.GetAxisConfig(deviceId, axisIndex);
                double spu   = axisCfg?.StepsPerUnit() ?? 0;

                double velocity, accel;
                if (!string.IsNullOrEmpty(step.AuxUnit) && spu > 0)
                {
                    velocity = ev.EvalField(step, "auxVelocity", step.AuxVelocity ?? 10) * spu;
                    accel    = ev.EvalField(step, "auxAccel",    step.AuxAccel    ?? 50)  * spu;
                }
                else
                {
                    velocity = ev.EvalField(step, "auxVelocity", step.AuxVelocity ?? 1600);
                    accel    = ev.EvalField(step, "auxAccel",    step.AuxAccel    ?? 3200);
                }

                // StartAuxContinuous derives direction from the sign of velocity (and
                // applies InvertDirection), then ramps by the absolute value. Pass the
                // SIGNED velocity — abs'ing it here made CW and CCW go the same way.
                ctrl.StartAuxContinuous(deviceId, axisIndex, velocity, accel);
                return StepOutcome.Advance;
            }
        }

        private sealed class AuxStopStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var    ctrl      = ctx.Controller;
                double decel     = ctx.Eval.EvalField(step, "auxDecel", step.AuxDecel ?? 10000);
                bool   immediate = step.AuxImmediate ?? false;

                if (!string.IsNullOrEmpty(step.AuxDeviceId) && step.AuxAxisIndex.HasValue)
                {
                    // Stop a specific axis on a specific device
                    if (immediate)
                        ctrl.AuxAxisManager.StopAll(step.AuxDeviceId);
                    else
                        ctrl.AuxAxisManager.StopSmooth(step.AuxDeviceId, step.AuxAxisIndex.Value, (int)Math.Max(1, decel));
                }
                else
                {
                    // Stop all axes on all devices
                    if (immediate)
                        ctrl.AuxAxisManager.StopAllDevices();
                    else
                        ctrl.StopAux(decel, false);
                }
                return StepOutcome.Advance;
            }
        }

        private sealed class AuxEnableStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                string deviceId = DeviceOrFirst(step, ctx);
                bool   enable   = step.AuxEnable ?? true;
                if (!string.IsNullOrEmpty(deviceId))
                    ctx.Controller.AuxAxisManager.Enable(deviceId, enable);
                return StepOutcome.Advance;
            }
        }

        // Gripper per-direction speed (mm/s): derived from the full-stroke open/close time
        // (speed = stroke / time) so a time honours any gap target; falls back to ActionVelocity,
        // then a gentle default. opening = jaws moving apart (gap increasing).
        private static double GripperVel(AuxAxisChannelConfig cfg, bool opening)
        {
            double stroke = Math.Abs(cfg.GripperOpenMm - cfg.GripperClosedMm);
            double time   = opening ? cfg.GripperOpenTimeSec : cfg.GripperCloseTimeSec;
            if (stroke > 0 && time > 0) return stroke / time;
            if (cfg.ActionVelocity > 0) return cfg.ActionVelocity;
            return 20; // mm/s
        }

        // Gripper Open/Close drive to the configured gap limits; GripTo drives to an absolute
        // jaw gap (mm) clamped to [closed, open]. Speed comes from the open/close time (or the
        // auxVelocity override), accel from GripperAccel (or the auxAccel override), both in mm/s(²).
        private sealed class GripperActionStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var    ctrl      = ctx.Controller;
                var    ev        = ctx.Eval;
                string deviceId  = DeviceOrFirst(step, ctx);
                int    axisIndex = step.AuxAxisIndex ?? 0;
                var    cfg       = ctrl.AuxAxisManager.GetAxisConfig(deviceId, axisIndex);

                double spu = cfg?.StepsPerUnit() ?? 0; // steps per mm of jaw gap (Linear)
                if (cfg == null || spu <= 0) return StepOutcome.Advance;

                double openMm   = cfg.GripperOpenMm;
                double closedMm = cfg.GripperClosedMm;
                if (openMm == closedMm) return StepOutcome.Advance;

                string action   = step.GripperAction ?? "Open";
                bool   isClose  = string.Equals(action, "Close",  StringComparison.OrdinalIgnoreCase);
                bool   isGripTo = string.Equals(action, "GripTo", StringComparison.OrdinalIgnoreCase);

                double targetMm = isClose  ? closedMm
                                : isGripTo ? ev.EvalField(step, "gripTargetMm", step.GripTargetMm ?? closedMm)
                                           : openMm;

                double lo = Math.Min(openMm, closedMm), hi = Math.Max(openMm, closedMm);
                targetMm = Math.Clamp(targetMm, lo, hi);

                long   currentPos = ctrl.AuxAxisManager.GetPosition(deviceId, axisIndex);
                double curMm      = currentPos / spu;

                // Open/Close head for a hard stop (opening = toward the wider gap); GripTo heads
                // for an in-between gap. Direction picks the time-derived speed and the over-travel sign.
                bool   opening = isGripTo ? (targetMm - curMm) > 0 : !isClose;
                long   steps   = (long)Math.Round((targetMm - curMm) * spu);

                // Over-travel a full Open/Close into the stop so the jaws seat and the tracked
                // position re-references at the clamped limit. GripTo must stop exactly on its gap.
                if (!isGripTo && cfg.GripperOverTravelMm > 0)
                    steps += (opening ? 1 : -1) * (long)Math.Round(cfg.GripperOverTravelMm * spu);

                if (steps == 0) return StepOutcome.Advance;

                double velMm   = ev.EvalField(step, "auxVelocity", step.AuxVelocity ?? GripperVel(cfg, opening));
                double accelMm = ev.EvalField(step, "auxAccel",    step.AuxAccel    ?? (cfg.GripperAccel > 0 ? cfg.GripperAccel : velMm * 4));
                double velSteps = Math.Max(1, velMm   * spu);
                double accSteps = Math.Max(1, accelMm * spu);

                ctrl.StartAuxMove(deviceId, axisIndex, steps, velSteps, accSteps, accSteps);
                ctx.AwaitAuxMove(step, frame);
                return StepOutcome.Yield;
            }
        }

        // Conveyor Run forward/reverse / Stop — continuous motion at the configured actionVelocity.
        private sealed class ConveyorActionStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var    ctrl      = ctx.Controller;
                string deviceId  = DeviceOrFirst(step, ctx);
                int    axisIndex = step.AuxAxisIndex ?? 0;
                var    cfg       = ctrl.AuxAxisManager.GetAxisConfig(deviceId, axisIndex);
                string action    = step.ConveyorAction ?? "Stop";

                if (string.Equals(action, "Stop", StringComparison.OrdinalIgnoreCase))
                {
                    ctrl.AuxAxisManager.StopSmooth(deviceId, axisIndex, 10000);
                    return StepOutcome.Advance;
                }

                double vel     = ActionVel(cfg);
                bool   reverse = string.Equals(action, "RunReverse", StringComparison.OrdinalIgnoreCase);
                ctrl.StartAuxContinuous(deviceId, axisIndex, reverse ? -vel : vel, 3200);
                return StepOutcome.Advance;
            }
        }

        // Rotary table / indexer — advance by N increments of the configured actionDistance.
        private sealed class RotaryIndexStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var    ctrl      = ctx.Controller;
                string deviceId  = DeviceOrFirst(step, ctx);
                int    axisIndex = step.AuxAxisIndex ?? 0;
                var    cfg       = ctrl.AuxAxisManager.GetAxisConfig(deviceId, axisIndex);

                long mag   = ActionStepsMag(cfg);
                int  count = Math.Max(1, step.RotaryCount ?? 1);
                int  dir   = (step.RotaryDirection ?? 1) < 0 ? -1 : 1;
                long steps = mag * count * dir;
                if (steps == 0) return StepOutcome.Advance;

                ctrl.StartAuxMove(deviceId, axisIndex, steps, ActionVel(cfg), 3200, 3200);
                ctx.AwaitAuxMove(step, frame);
                return StepOutcome.Yield;
            }
        }
    }
}
