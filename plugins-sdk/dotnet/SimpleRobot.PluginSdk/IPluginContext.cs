using System.Text.Json;

namespace SimpleRobot.PluginSdk;

/// <summary>What a plugin can do while connected. A new context is created for every connection.</summary>
public interface IPluginContext
{
    /// <summary>The plugin's current config (schema defaults merged in by the controller). Updated on <c>config.changed</c>.</summary>
    JsonElement Config { get; }
    string ControllerVersion { get; }
    string PluginId { get; }
    string PluginDir { get; }
    string DataDir { get; }

    void Log(string message, LogLevel level = LogLevel.Info);

    /// <summary>Replaces these values in the plugin's property table. Accepts an anonymous object or a dictionary.</summary>
    void SetProperties(object values);
    void ClearProperties();
    /// <summary><paramref name="state"/> is "ok", "degraded" or "error".</summary>
    void SetStatus(string state, string? message = null);

    /// <summary>Reports step progress. Only valid inside a step handler (throws <see cref="InvalidOperationException"/> otherwise).</summary>
    void Progress(string? message, double? percent = null);

    /// <summary>Runs any WebSocket API command. Throws <see cref="CommandException"/> when the controller reports failure.</summary>
    Task<JsonElement> CommandAsync(string command, object? @params = null);
    Task<VariablesSnapshot> GetVariablesAsync(string? program = null);
    Task SetVariablesAsync(object values, string? program = null);

    /// <summary>
    /// Subscribes to events (globs like <c>program.*</c> allowed). Events already subscribed (including the automatic
    /// subscription for every <c>On(...)</c> handler) are kept; intervals that are null keep their previous value.
    /// </summary>
    Task SubscribeAsync(IEnumerable<string> events, int? positionIntervalMs = null, int? statusIntervalMs = null, int? ioIntervalMs = null);
}

/// <summary>The context handed to step handlers (cast the <see cref="IPluginContext"/> argument to this).</summary>
public interface IStepContext : IPluginContext
{
    string InvocationId { get; }
    string StepId { get; }
    string ProgramName { get; }
    string? StepName { get; }
    bool IsBackground { get; }
    bool IsCancelled { get; }
    /// <summary>Cancelled when the controller sends <c>step.cancel</c> or the connection ends.</summary>
    CancellationToken CancellationToken { get; }
    /// <summary>The reason from <c>step.cancel</c> ("stopped", "reset", "timeout", "shutdown"), once cancelled.</summary>
    string? CancelReason { get; }
}
