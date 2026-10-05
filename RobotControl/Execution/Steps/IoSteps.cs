namespace Controller.RobotControl.Execution
{
    /// <summary>Digital IO: SetOutput (with optional pulse) and Wait (time or condition).</summary>
    internal static class IoSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.SetOutput] = new SetOutputStep();
            r[StepType.Wait]      = new WaitStep();
        }

        /// <summary>Applies a single output state to the correct IO card.</summary>
        internal static void ApplyOutput(RobotController ctrl, string card, int number, bool value, string? nanoId)
        {
            switch (card)
            {
                case "relay":
                    ctrl.RelayManager.SetRelay(number, value);
                    break;
                case "nano":
                    if (!string.IsNullOrEmpty(nanoId))
                        ctrl.NanoManager.SetOutput(nanoId, number, value);
                    break;
                default: // "stb"
                    ctrl.stb.SetOutput(number, value);
                    break;
            }
        }

        private static long NowMs() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        private sealed class SetOutputStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var card   = step.OutputCard ?? "stb";
                var number = step.OutputNumber ?? 1;
                var value  = step.OutputValue ?? false;
                var pulse  = step.PulseMs ?? 0;

                if (!frame.WaitStarted)
                {
                    ApplyOutput(ctx.Controller, card, number, value, step.OutputNanoId);

                    if (pulse > 0 && step.PulseBlocking == true)
                    {
                        frame.WaitStartMs = NowMs();
                        frame.WaitStarted = true;
                        ctx.Progress.StepStarted(step);
                        return StepOutcome.Yield;
                    }

                    if (pulse > 0)
                    {
                        // Scheduled on the loop thread (see ProgramExecutor.ProcessOutputFlips)
                        // rather than a Task.Delay, so Stop/Reset/e-stop cancel the revert
                        // instead of it flipping the output some time after the program was halted.
                        var ctrl   = ctx.Controller;
                        var nanoId = step.OutputNanoId;
                        ctx.OutputFlips.Add((NowMs() + pulse,
                            () => ApplyOutput(ctrl, card, number, !value, nanoId)));
                    }

                    return StepOutcome.Advance;
                }

                // Blocking pulse — wait for pulse duration then flip
                var elapsed = NowMs() - frame.WaitStartMs;
                if (elapsed >= pulse)
                {
                    ApplyOutput(ctx.Controller, card, number, !value, step.OutputNanoId);
                    frame.WaitStarted = false;
                    return StepOutcome.Advance;
                }
                return StepOutcome.Yield;
            }
        }

        private sealed class WaitStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                if (!frame.WaitStarted)
                {
                    frame.WaitStartMs = NowMs();
                    frame.WaitStarted = true;
                    ctx.Progress.StepStarted(step);
                    return StepOutcome.Yield;
                }

                var elapsed = NowMs() - frame.WaitStartMs;

                if (step.WaitMode == "condition" && step.WaitCondition != null)
                {
                    // Program variables + IO, from the tick's snapshot
                    bool condMet  = ctx.Eval.EvaluateCondition(step.WaitCondition);
                    int  timeout  = step.WaitTimeoutMs ?? 0;
                    bool timedOut = timeout > 0 && elapsed >= timeout;

                    if (condMet || timedOut)
                    {
                        if (!string.IsNullOrEmpty(step.WaitTimeoutVariableName))
                            ctx.Vars.Set(step.WaitTimeoutVariableName, timedOut ? 1 : 0);
                        frame.WaitStarted = false;
                        return StepOutcome.Advance;
                    }
                }
                else if (elapsed >= ctx.Eval.EvalField(step, "waitMs", step.WaitMs ?? 0))
                {
                    frame.WaitStarted = false;
                    return StepOutcome.Advance;
                }
                return StepOutcome.Yield;
            }
        }
    }
}
