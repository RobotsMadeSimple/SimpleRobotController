namespace Controller.RobotControl.Commands;

/// <summary>
/// Program debugging (docs/program-language.md §4.2, Debugging). Package A registers the names;
/// package B fills them in:
/// <list type="bullet">
/// <item><c>StepBuiltProgram {}</c> — run the paused foreground program's next step, then pause again (pauseReason "step").</item>
/// <item><c>SetBreakpoints { stepIds }</c> (<see cref="SetBreakpointsParams"/>) — replace the running program's breakpoints.</item>
/// </list>
/// The run options (startAtStepId, breakpoints, stepMode) ride ExecuteBuiltProgram
/// (<see cref="ExecuteBuiltProgramParams"/>), which refuses them until B lands.
/// </summary>
internal sealed class DebugCommands
{
    private readonly ProgramExecutor? _executor;

    public DebugCommands(ProgramExecutor? executor) => _executor = executor;

    public void Register(CommandDispatcher d)
    {
        d.Add("StepBuiltProgram", CommandNotImplementedException.Handler("StepBuiltProgram"));
        d.Add("SetBreakpoints",   CommandNotImplementedException.Handler("SetBreakpoints"));
    }
}
