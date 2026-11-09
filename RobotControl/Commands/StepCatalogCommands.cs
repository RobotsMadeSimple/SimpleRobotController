using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Commands;

/// <summary>
/// GetStepCatalog {} → { catalogVersion, languageVersion, categories, errorCodes, steps,
/// commonFields, legacyFields } (docs/program-language.md §5.2). Static per build, so clients
/// fetch it once per connection.
/// </summary>
internal sealed class StepCatalogCommands
{
    public void Register(CommandDispatcher d)
    {
        d.Add("GetStepCatalog", _ => StepCatalog.Info());
    }
}

/// <summary>Thrown by commands whose contract exists but whose behaviour has not landed yet. The
/// client sees <c>{ ok: false, error: "notImplemented: …" }</c>.</summary>
internal sealed class CommandNotImplementedException(string command)
    : InvalidOperationException($"notImplemented: {command} is not implemented on this controller yet")
{
    /// <summary>A command handler that always fails with this exception.</summary>
    public static Func<CommandMessage, object?> Handler(string command) =>
        _ => throw new CommandNotImplementedException(command);
}
