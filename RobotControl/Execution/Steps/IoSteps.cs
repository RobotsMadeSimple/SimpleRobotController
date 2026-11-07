using Controller.RobotControl.Programs;

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
        internal static void ApplyOutput(RobotController ctrl, string card, int number, bool value, string? deviceId)
        {
            switch (card)
            {
                case "relay":
                    ctrl.RelayManager.SetRelay(number, value);
                    break;
                case "io":
                    if (!string.IsNullOrEmpty(deviceId))
                        ctrl.NanoManager.SetOutput(deviceId, number, value);
                    break;
                default: // "stb"
                    ctrl.stb.SetOutput(number, value);
                    break;
            }
        }

        /// <summary>
        /// An output's current state, as the IO snapshot expressions read it (<c>$stb.out2</c>,
        /// <c>$relay.3</c>; an IO device pin by device id and pin number). False when unknown.
        /// </summary>
        internal static bool ReadOutput(RobotController ctrl, string card, int number, string? deviceId)
        {
            switch (card)
            {
                case "relay":
                    var relays = ctrl.RelayManager.GetRelayStates();
                    return relays != null && number >= 1 && number <= relays.Length && relays[number - 1];
                case "io":
                    if (string.IsNullOrEmpty(deviceId)) return false;
                    var dev = ctrl.NanoManager.GetAllStates().FirstOrDefault(d => d.Id == deviceId);
                    return dev?.Pins.FirstOrDefault(p => p.Pin == number)?.Value ?? false;
                default: // "stb"
                    return number switch
                    {
                        1 => ctrl.stb.Output1, 2 => ctrl.stb.Output2,
                        3 => ctrl.stb.Output3, 4 => ctrl.stb.Output4,
                        _ => false,
                    };
            }
        }

        private static long NowMs() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        /// <summary>A condition as one line of text for an error message.</summary>
        internal static string DescribeCondition(ConditionGroup? g)
        {
            if (g == null || g.Items.Count == 0) return "the condition";
            var sb = new System.Text.StringBuilder();
            string defaultJoin = g.Combinator == "ANY" ? "or" : "and";
            for (int i = 0; i < g.Items.Count; i++)
            {
                var it = g.Items[i];
                if (i > 0) sb.Append(' ').Append(string.IsNullOrEmpty(it.Join) ? defaultJoin : it.Join.ToLowerInvariant()).Append(' ');
                sb.Append(!string.IsNullOrWhiteSpace(it.Expr) ? it.Expr.Trim() : $"{it.Left} {it.Operator} {it.Right}");
            }
            return sb.ToString();
        }

        private sealed class SetOutputStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var card   = step.OutputCard ?? "stb";
                var number = step.OutputNumber ?? 1;
                var pulse  = step.PulseMs ?? 0;

                if (!frame.WaitStarted)
                {
                    // The state to set: toggle flips the output's current state; set takes
                    // expressions.outputValue (non-zero = on) when given, else the literal.
                    // Decided once, on entry — a blocking pulse reverts to its opposite.
                    bool value = step.OutputAction == "toggle"
                        ? !ReadOutput(ctx.Controller, card, number, step.OutputDeviceId)
                        : step.Expressions?.ContainsKey("outputValue") == true
                            ? Math.Abs(ctx.Eval.EvalField(step, "outputValue", step.OutputValue == true ? 1 : 0)) > 1e-9
                            : step.OutputValue ?? false;
                    frame.PulseValue = value;

                    ApplyOutput(ctx.Controller, card, number, value, step.OutputDeviceId);

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
                        var ctrl     = ctx.Controller;
                        var deviceId   = step.OutputDeviceId;
                        ctx.OutputFlips.Add((NowMs() + pulse,
                            () => ApplyOutput(ctrl, card, number, !value, deviceId)));
                    }

                    return StepOutcome.Advance;
                }

                // Blocking pulse — wait for pulse duration then flip
                var elapsed = NowMs() - frame.WaitStartMs;
                if (elapsed >= pulse)
                {
                    ApplyOutput(ctx.Controller, card, number, !frame.PulseValue, step.OutputDeviceId);
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
                        // The condition wins a tie: met on the tick the timeout falls due is met.
                        timedOut = timedOut && !condMet;
                        if (!string.IsNullOrEmpty(step.WaitTimeoutVariableName))
                            ctx.Vars.Set(step.WaitTimeoutVariableName, timedOut ? 1 : 0);
                        frame.WaitStarted = false;
                        if (timedOut && step.WaitTimeoutAction == "error")
                            return ctx.Fail(ErrorCodes.WaitTimeout,
                                $"{ProgressReporter.StepDescription(step)}: timed out after {timeout} ms waiting for {DescribeCondition(step.WaitCondition)}");
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
