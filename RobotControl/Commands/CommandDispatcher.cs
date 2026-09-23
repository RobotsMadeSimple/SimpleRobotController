namespace Controller.RobotControl.Commands;

/// <summary>
/// Maps a WebSocket command name to its handler. A handler returns the response
/// payload, or null for the empty <c>{}</c> acknowledgement. Every command is
/// registered once at construction by the per-domain command classes; a
/// duplicate name is a programming error and throws at startup.
/// </summary>
internal sealed class CommandDispatcher
{
    private readonly Dictionary<string, Func<CommandMessage, Task<object?>>> _handlers = new(StringComparer.Ordinal);

    public void AddAsync(string name, Func<CommandMessage, Task<object?>> handler) => _handlers.Add(name, handler);

    public void Add(string name, Func<CommandMessage, object?> handler) =>
        _handlers.Add(name, msg => Task.FromResult(handler(msg)));

    public void Add(string name, Action<CommandMessage> handler) =>
        _handlers.Add(name, msg => { handler(msg); return Task.FromResult<object?>(null); });

    public bool TryGet(string? name, out Func<CommandMessage, Task<object?>> handler)
    {
        if (name is not null && _handlers.TryGetValue(name, out var h))
        {
            handler = h;
            return true;
        }
        handler = null!;
        return false;
    }

    public IReadOnlyCollection<string> Names => _handlers.Keys;

    /// <summary>Builds the dispatcher with every command the controller serves.</summary>
    public static CommandDispatcher Create(RobotController robot, ProgramCycleManager programs,
        ProgramExecutor? executor, BackgroundProgramManager background)
    {
        var d = new CommandDispatcher();
        new SystemCommands(robot, programs, background).Register(d);
        new ConfigCommands(robot).Register(d);
        new MotionCommands(robot).Register(d);
        new PointCommands(robot).Register(d);
        new ToolCommands(robot).Register(d);
        new LocalCommands(robot).Register(d);
        new GridCommands(robot).Register(d);
        new StackCommands(robot).Register(d);
        new BuiltProgramCommands(robot, programs, executor, background).Register(d);
        new RevisionCommands(robot).Register(d);
        new ProgramCycleCommands(robot, programs, executor).Register(d);
        new IoCommands(robot).Register(d);
        new AuxCommands(robot).Register(d);
        new CameraCommands(robot).Register(d);
        new VisionCommands(robot).Register(d);
        return d;
    }
}
