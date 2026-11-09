using Controller.RobotControl.Execution;

namespace Controller.RobotControl.Commands;

/// <summary>
/// Operator prompts (docs/program-language.md §4.3, Prompt):
/// <c>AnswerPrompt { promptId, button, value? }</c> (<see cref="AnswerPromptParams"/>) → {}.
/// The first answer wins. Failures (<c>{ ok: false, error }</c>, the error starting with its code):
/// <c>stalePrompt</c> — no prompt with that id is waiting (already answered, timed out, re-shown
/// under a new id, or the run ended); <c>programPaused</c> — the program is stopped, continue it
/// first; <c>badButton</c> — no such button; <c>badValue</c> — a number input that is missing,
/// not a number, or outside min–max.
/// </summary>
internal sealed class OperatorCommands
{
    private readonly ProgramCycleManager _programs;

    public OperatorCommands(ProgramCycleManager programs) => _programs = programs;

    public void Register(CommandDispatcher d)
    {
        d.Add("AnswerPrompt", AnswerPrompt);
    }

    private object? AnswerPrompt(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<AnswerPromptParams>(msg);
        if (string.IsNullOrWhiteSpace(p.PromptId))
            throw new InvalidOperationException("stalePrompt: no promptId was given");

        var error = PromptRegistry.Answer(p.PromptId.Trim(), p.Button, p.Value);
        if (error == null) return null;

        // A prompt nobody waits on any more must not stay on screen: if the status still shows
        // it (its run ended without clearing it — a stopped background program), clear it.
        if (error.StartsWith("stalePrompt", StringComparison.Ordinal))
            foreach (var shown in _programs.GetPrompts())
                if (shown.Id == p.PromptId.Trim() && !PromptRegistry.TryGet(shown.Id, out _))
                    _programs.SetPrompt(shown.ProgramName, null);

        throw new InvalidOperationException(error);
    }
}
