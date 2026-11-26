using System.Collections.Generic;
using System.Text.RegularExpressions;
using Controller.RobotControl.Serial;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// SerialCommand program step (serial-device contract §4): send a named/raw command to a generic
    /// serial device and, when expecting a reply, wait (non-blocking, via the executor's yield/poll
    /// pattern) for the first reply line and store it into a variable. The raw payload supports
    /// {{expr}} interpolation against the step's eval context.
    /// </summary>
    internal static class SerialSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.SerialCommand] = new SerialCommandStep();
        }

        private static long NowMs() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

        private static readonly Regex Mustache = new(@"\{\{(.*?)\}\}", RegexOptions.Compiled);

        /// <summary>Interpolate {{expr}} in <paramref name="raw"/> using the program's variable scope.
        /// Each {{…}} is evaluated as a single-brace template expression ($vars, lists, numbers).</summary>
        internal static string Interpolate(string? raw, ExecutionContext ctx)
        {
            if (string.IsNullOrEmpty(raw) || !raw.Contains("{{")) return raw ?? "";
            return Mustache.Replace(raw, m => ctx.Vars.Interpolate("{" + m.Groups[1].Value.Trim() + "}"));
        }

        private sealed class SerialCommandStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                var dev = ctx.Controller.SerialManager.Resolve(step.SerialDeviceId, step.SerialDeviceName);

                if (!frame.WaitStarted)
                {
                    if (dev == null) return StepOutcome.Advance;   // unknown device — skip, like a missing load cell

                    // Resolve payload + reply expectation: a saved command, else the raw text field.
                    var cmd = dev.GetCommand(step.SerialCommandId);
                    string rawPayload = cmd != null ? cmd.Payload : (step.SerialText ?? "");
                    bool expectReply  = step.SerialExpectReply ?? cmd?.ExpectReply ?? false;

                    string payload = Interpolate(rawPayload, ctx);

                    frame.SerialMark = dev.RxSeq;     // only replies after this point count
                    dev.SendLine(payload);

                    if (!expectReply) return StepOutcome.Advance;

                    frame.WaitStartMs = NowMs();
                    frame.WaitStarted = true;
                    ctx.Progress.StepStarted(step);
                    return StepOutcome.Yield;
                }

                // Awaiting a reply — poll until a line arrives or the timeout elapses.
                var cmdDef   = dev?.GetCommand(step.SerialCommandId);
                int timeout  = step.SerialTimeoutMs ?? cmdDef?.ReplyTimeoutMs ?? 1000;
                var lines    = dev?.RxLinesSince(frame.SerialMark) ?? (IReadOnlyList<string>)[];
                bool timedOut = NowMs() - frame.WaitStartMs >= timeout;

                if (lines.Count > 0 || timedOut)
                {
                    frame.WaitStarted = false;
                    if (!string.IsNullOrEmpty(step.SerialResultVariable))
                        ctx.Vars.SetString(step.SerialResultVariable, lines.Count > 0 ? lines[0] : "");
                    return StepOutcome.Advance;
                }
                return StepOutcome.Yield;
            }
        }
    }
}
