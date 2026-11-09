namespace Controller.RobotControl.Commands;

/// <summary>
/// Program debugging (docs/program-language.md §4.2, Debugging), for the foreground program:
/// <list type="bullet">
/// <item><c>StepBuiltProgram {}</c> — a paused program runs its next step, then pauses again before
/// the one after (pauseReason "step"); a running one pauses before its next step. Fails when no
/// program is running or paused.</item>
/// <item><c>SetBreakpoints { stepIds }</c> (<see cref="SetBreakpointsParams"/>) — replaces the running
/// program's breakpoints → <c>{ count }</c>.</item>
/// </list>
/// The run options (startAtStepId, breakpoints, stepMode) ride ExecuteBuiltProgram
/// (<see cref="ExecuteBuiltProgramParams"/>); Continue is the usual StartProgram / resume.
/// </summary>
internal sealed class DebugCommands
{
    private readonly ProgramExecutor? _executor;

    public DebugCommands(ProgramExecutor? executor) => _executor = executor;

    public void Register(CommandDispatcher d)
    {
        d.Add("StepBuiltProgram", StepBuiltProgram);
        d.Add("SetBreakpoints",   SetBreakpoints);
    }

    private object? StepBuiltProgram(CommandMessage msg)
    {
        if (_executor == null || !_executor.StepOnce())
            throw new InvalidOperationException("No program is running or paused");
        return new { programName = _executor.CurrentProgramName };
    }

    private object? SetBreakpoints(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetBreakpointsParams>(msg);
        int count = _executor?.SetBreakpoints(p.StepIds) ?? 0;
        return new { count };
    }
}
