using System.Collections.Concurrent;
using System.Globalization;
using System.Runtime.CompilerServices;
using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Program editor v2 operator blocks (docs/program-language.md §4.3): Prompt — a message with
    /// 1–4 buttons and an optional number or text input, published in the program status
    /// (<see cref="ProgramPromptInfo"/>) and answered with the <c>AnswerPrompt</c> command.
    /// </summary>
    internal static class OperatorSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.Prompt] = PromptStep.Instance;
        }
    }

    /// <summary>An operator's answer (or the timeout's stand-in for one).</summary>
    internal sealed record PromptAnswer(int Button, string? Value, bool TimedOut = false);

    /// <summary>
    /// One prompt waiting for an answer. Created on the program loop; answered from a WebSocket
    /// thread (first answer wins) and consumed on the loop.
    /// </summary>
    internal sealed class PendingPrompt
    {
        private PromptAnswer? _answer;
        private long _lastTickMs;

        public PendingPrompt(ProgramPromptInfo info, ExecutionContext ctx, long nowMs)
        {
            Info       = info;
            Context    = ctx;
            Generation = ctx.RunGeneration;
            _lastTickMs = nowMs;
        }

        public ProgramPromptInfo Info       { get; }
        public ExecutionContext  Context    { get; }
        /// <summary>The run it belongs to — a reset or a new run makes it stale.</summary>
        public int               Generation { get; }

        /// <summary>The run it belonged to has ended or been reset (an int read — safe off the loop thread).</summary>
        public bool IsStale => Generation != Context.RunGeneration;

        /// <summary>When the waiting step last ran (registry clock). A gap means the program is paused.</summary>
        public long LastTickMs
        {
            get => Interlocked.Read(ref _lastTickMs);
            set => Interlocked.Exchange(ref _lastTickMs, value);
        }

        public PromptAnswer? Answer => Volatile.Read(ref _answer);

        /// <summary>Records the answer; false when one was already recorded.</summary>
        public bool TryAnswer(PromptAnswer answer) => Interlocked.CompareExchange(ref _answer, answer, null) == null;
    }

    /// <summary>
    /// Every prompt waiting for an answer, by id, across all programs — what <c>AnswerPrompt</c>
    /// looks answers up in, and the rules it applies to them.
    /// </summary>
    internal static class PromptRegistry
    {
        /// <summary>A program whose waiting prompt step has not run for this long is paused (Stop):
        /// it cannot be answered until it continues, and is shown afresh then.</summary>
        public const long PausedAfterMs = 750;

        private static readonly ConcurrentDictionary<string, PendingPrompt> ById = new(StringComparer.Ordinal);

        /// <summary>Milliseconds clock; replaceable by tests.</summary>
        internal static Func<long> Clock = () => Environment.TickCount64;

        public static void Add(PendingPrompt p) => ById[p.Info.Id] = p;

        public static void Remove(PendingPrompt p) => ById.TryRemove(new KeyValuePair<string, PendingPrompt>(p.Info.Id, p));

        public static bool TryGet(string id, out PendingPrompt p) => ById.TryGetValue(id ?? "", out p!);

        /// <summary>
        /// Applies an answer. Null on success, else the error the command reports:
        /// <c>stalePrompt</c> (unknown id, already answered, or the run it belonged to is over),
        /// <c>programPaused</c>, <c>badButton</c> or <c>badValue</c>.
        /// </summary>
        public static string? Answer(string promptId, int button, string? value)
        {
            if (!TryGet(promptId, out var p))
                return $"stalePrompt: no prompt is waiting with id '{promptId}' (already answered, timed out or ended)";
            if (p.IsStale)
            {
                Remove(p);
                return "stalePrompt: the program run this prompt belonged to has ended";
            }
            if (p.Answer != null)
                return "stalePrompt: this prompt was already answered";
            if (Clock() - p.LastTickMs > PausedAfterMs)
                return "programPaused: the program is stopped — continue it, then answer the prompt";

            var info = p.Info;
            if (button < 0 || button >= info.Buttons.Count)
                return $"badButton: button {button} does not exist (0–{info.Buttons.Count - 1})";

            string? text = null;
            switch (info.Input)
            {
                case "number":
                {
                    var raw = string.IsNullOrWhiteSpace(value) ? info.Default : value;
                    if (string.IsNullOrWhiteSpace(raw))
                        return "badValue: a number is required";
                    if (!double.TryParse(raw.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out var n) || !double.IsFinite(n))
                        return $"badValue: '{raw}' is not a number";
                    if (info.Min is { } min && n < min) return $"badValue: {VariableScope.FormatNumber(n)} is below the minimum {VariableScope.FormatNumber(min)}";
                    if (info.Max is { } max && n > max) return $"badValue: {VariableScope.FormatNumber(n)} is above the maximum {VariableScope.FormatNumber(max)}";
                    text = raw.Trim();
                    break;
                }
                case "text":
                    text = value ?? info.Default ?? "";
                    break;
            }

            return p.TryAnswer(new PromptAnswer(button, text)) ? null : "stalePrompt: this prompt was already answered";
        }

        /// <summary>Test hook: forget every prompt.</summary>
        internal static void Clear() => ById.Clear();
    }

    /// <summary>
    /// The Prompt step. First call: build and publish the prompt, then wait. Each later call:
    /// an answer → write the result / input variables and continue; the timeout → press
    /// <c>promptTimeoutButton</c>, or fail <c>promptTimeout</c> (catchable) when none is set.
    /// After a Stop the step is shown again on Continue, under a fresh id, its timeout restarted.
    /// </summary>
    internal sealed class PromptStep : IStepHandler
    {
        public static readonly PromptStep Instance = new();

        /// <summary>Per-executor state of the prompt being waited on.</summary>
        private sealed class RunState
        {
            public PendingPrompt? Pending;
            public ProgramStep?   Step;
            public StepListFrame? Frame;
            public long           DeadlineMs; // 0 = no timeout
        }

        private static readonly ConditionalWeakTable<ExecutionContext, RunState> States = new();

        // The prompt is published through the program status, which the executor's progress
        // reporter owns.
        internal static ProgramCycleManager? Programs(ExecutionContext ctx) => ctx.Progress.Programs;

        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            var state = States.GetValue(ctx, _ => new RunState());
            long now = PromptRegistry.Clock();

            bool current = state.Pending is { } p0 && ReferenceEquals(state.Step, step) &&
                           ReferenceEquals(state.Frame, frame) && !p0.IsStale;
            if (!current)
            {
                if (state.Pending != null) Withdraw(ctx, state); // left over from an ended run
                Show(step, frame, ctx, state, now);
                ctx.Progress.StepStarted(step);
                return StepOutcome.Yield;
            }

            var pending = state.Pending!;
            if (pending.Answer is { } answer)
                return Complete(step, ctx, state, answer);

            // Not run for a while: the program was stopped and has just continued. Show it again
            // (fresh id, timeout from now) — an answer to the old id is refused as stale.
            if (now - pending.LastTickMs > PromptRegistry.PausedAfterMs)
            {
                Withdraw(ctx, state);
                Show(step, frame, ctx, state, now);
                return StepOutcome.Yield;
            }
            pending.LastTickMs = now;

            if (state.DeadlineMs > 0 && now >= state.DeadlineMs)
            {
                var info = pending.Info;
                int? button = step.PromptTimeoutButton;
                if (button is { } b && b >= 0 && b < info.Buttons.Count)
                {
                    var timeoutAnswer = new PromptAnswer(b, info.Input == "none" ? null : info.Default, TimedOut: true);
                    // An answer that arrived in the same instant wins.
                    if (!pending.TryAnswer(timeoutAnswer)) return Complete(step, ctx, state, pending.Answer!);
                    return Complete(step, ctx, state, timeoutAnswer);
                }
                if (!pending.TryAnswer(new PromptAnswer(-1, null, TimedOut: true)))
                    return Complete(step, ctx, state, pending.Answer!);
                Withdraw(ctx, state);
                return ctx.Fail(ErrorCodes.PromptTimeout,
                    $"{ProgressReporter.StepDescription(step)}: no answer within {VariableScope.FormatNumber(info.TimeoutMs)} ms");
            }
            return StepOutcome.Yield;
        }

        private static void Show(ProgramStep step, StepListFrame frame, ExecutionContext ctx, RunState state, long now)
        {
            var info = BuildInfo(step, ctx);
            var pending = new PendingPrompt(info, ctx, now);
            state.Pending    = pending;
            state.Step       = step;
            state.Frame      = frame;
            state.DeadlineMs = info.TimeoutMs > 0 ? now + info.TimeoutMs : 0;
            PromptRegistry.Add(pending);
            Programs(ctx)?.SetPrompt(info.ProgramName, info);
        }

        /// <summary>Unregisters the waiting prompt and takes it out of the status (if it is still the one shown).</summary>
        private static void Withdraw(ExecutionContext ctx, RunState state)
        {
            if (state.Pending is not { } p) return;
            PromptRegistry.Remove(p);
            var programs = Programs(ctx);
            if (programs?.GetPrompt(p.Info.ProgramName) is { } shown && shown.Id == p.Info.Id)
                programs.SetPrompt(p.Info.ProgramName, null);
            state.Pending = null;
            state.Step    = null;
            state.Frame   = null;
        }

        private static StepOutcome Complete(ProgramStep step, ExecutionContext ctx, RunState state, PromptAnswer answer)
        {
            var info = state.Pending!.Info;
            Withdraw(ctx, state);
            var vars = ctx.Vars;

            var resultVar = VarName(step.PromptResultVariableName);
            if (resultVar.Length > 0 && answer.Button >= 0)
            {
                if (vars.IsString(resultVar)) vars.SetString(resultVar, info.Buttons[answer.Button]);
                else vars.Set(resultVar, answer.Button);
            }

            var inputVar = VarName(step.PromptInputVariableName);
            if (inputVar.Length > 0 && info.Input != "none" && answer.Value != null)
            {
                if (vars.IsString(inputVar)) vars.SetString(inputVar, answer.Value);
                else vars.Set(inputVar, ExpressionEvaluator.TextToNumber(answer.Value));
            }

            ctx.Progress.StepProgress(answer.TimedOut
                ? $"Prompt timed out — pressed \"{info.Buttons[answer.Button]}\""
                : $"Prompt answered: \"{info.Buttons[answer.Button]}\"{(answer.Value != null && info.Input != "none" ? $" ({answer.Value})" : "")}");
            return StepOutcome.Advance;
        }

        /// <summary>The prompt as published: templates interpolated, buttons trimmed to 1–4, the default evaluated.</summary>
        internal static ProgramPromptInfo BuildInfo(ProgramStep step, ExecutionContext ctx)
        {
            var vars = ctx.Vars;
            var buttons = (step.PromptButtons ?? [])
                .Select(b => b?.Trim() ?? "")
                .Where(b => b.Length > 0)
                .Take(4)
                .ToList();
            if (buttons.Count == 0) buttons.Add("OK");

            string input = step.PromptInput is "number" or "text" ? step.PromptInput : "none";
            string? def = null;
            if (input != "none" && !string.IsNullOrWhiteSpace(step.PromptDefault))
                def = input == "number"
                    ? VariableScope.FormatNumber(ctx.Eval.Evaluate(step.PromptDefault))
                    : vars.Interpolate(step.PromptDefault);

            return new ProgramPromptInfo
            {
                Id          = Guid.NewGuid().ToString("N"),
                StepId      = step.Id ?? "",
                ProgramName = ctx.Progress.ProgramName,
                Title       = string.IsNullOrWhiteSpace(step.PromptTitle) ? null : vars.Interpolate(step.PromptTitle),
                Message     = vars.Interpolate(step.PromptMessage ?? ""),
                Buttons     = buttons,
                Input       = input,
                Default     = def,
                Min         = input == "number" ? step.PromptMin : null,
                Max         = input == "number" ? step.PromptMax : null,
                TimeoutMs   = Math.Max(0, step.PromptTimeoutMs ?? 0),
                ShownUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            };
        }

        private static string VarName(string? raw) => raw?.Trim().TrimStart('$') ?? "";
    }
}
