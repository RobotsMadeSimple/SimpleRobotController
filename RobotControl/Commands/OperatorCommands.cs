namespace Controller.RobotControl.Commands;

/// <summary>
/// Operator prompts (docs/program-language.md §4.3, Prompt). Package A registers the name;
/// package C fills it in:
/// <c>AnswerPrompt { promptId, button, value? }</c> (<see cref="AnswerPromptParams"/>) → {} — the
/// first answer wins, a later one (or an unknown id) fails with <c>stalePrompt</c>.
/// </summary>
internal sealed class OperatorCommands
{
    private readonly ProgramCycleManager _programs;

    public OperatorCommands(ProgramCycleManager programs) => _programs = programs;

    public void Register(CommandDispatcher d)
    {
        d.Add("AnswerPrompt", CommandNotImplementedException.Handler("AnswerPrompt"));
    }
}
