namespace Controller.RobotControl.Execution
{
    /// <summary>Auxiliary axes: AuxMove, AuxContinuous, AuxStop, AuxEnable.</summary>
    internal static class AuxSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.AuxMove]       = new AuxMoveStep();
            r[StepType.AuxContinuous] = new AuxContinuousStep();
            r[StepType.AuxStop]       = new AuxStopStep();
            r[StepType.AuxEnable]     = new AuxEnableStep();
        }

        private static string DeviceOrFirst(ProgramStep step, ExecutionContext ctx) =>
            step.AuxDeviceId ?? ctx.Controller.AuxAxisManager.GetFirstDevice()?.Id ?? "";

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
    }
}
