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

    /// <summary>
    /// Pushes view state to the plugin's custom GUI (docs/plugins.md §4.3 <c>view.set</c>). The controller
    /// shallow-merges the top-level keys of <paramref name="state"/> into the stored view state and streams
    /// it to every open GUI. Accepts an anonymous object or a dictionary; any JSON is allowed (unlike properties).
    /// </summary>
    void SetView(object state);
    /// <summary>Clears all view state (the GUI receives an empty state). Also happens automatically on disconnect.</summary>
    void ClearView();
    /// <summary><paramref name="state"/> is "ok", "degraded" or "error".</summary>
    void SetStatus(string state, string? message = null);

    /// <summary>Reports step progress. Only valid inside a step handler (throws <see cref="InvalidOperationException"/> otherwise).</summary>
    void Progress(string? message, double? percent = null);

    /// <summary>Runs any WebSocket API command. Throws <see cref="CommandException"/> when the controller reports failure.</summary>
    Task<JsonElement> CommandAsync(string command, object? @params = null);
    Task<VariablesSnapshot> GetVariablesAsync(string? program = null);
    Task SetVariablesAsync(object values, string? program = null);

    /// <summary>Convenience helpers for the most common robot commands (motion, points, live pose).
    /// A thin layer over <see cref="CommandAsync"/>; anything not covered is still reachable with it.</summary>
    IRobotApi Robot { get; }

    /// <summary>
    /// Subscribes to events (globs like <c>program.*</c> allowed). Events already subscribed (including the automatic
    /// subscription for every <c>On(...)</c> handler) are kept; intervals that are null keep their previous value.
    /// </summary>
    Task SubscribeAsync(IEnumerable<string> events, int? positionIntervalMs = null, int? statusIntervalMs = null, int? ioIntervalMs = null);
}

/// <summary>
/// Convenience wrappers over common controller commands, reached as <c>ctx.Robot</c>. Each method issues
/// exactly one WebSocket command and returns its result; anything not covered here (any command in
/// <c>docs/websocket-api.md</c>) is still reachable with <see cref="IPluginContext.CommandAsync"/>.
/// </summary>
public interface IRobotApi
{
    /// <summary>Linear (straight-line) move to a Cartesian pose. On the 4-axis ASTRO the only orientation
    /// DOF is <paramref name="rz"/> (J4); <paramref name="rx"/>/<paramref name="ry"/> stay 0. Queued on the
    /// motion thread — returns when the move is <i>accepted</i>, not when it finishes (await
    /// <see cref="IsMoving"/> / the <c>robot.position</c> event to know it has stopped).</summary>
    Task<JsonElement> MoveL(double x, double y, double z, double rx = 0, double ry = 0, double rz = 0, double? speed = null, double? accel = null);
    /// <summary>Joint-interpolated move to a Cartesian pose (see <see cref="MoveL"/> for the DOF/timing notes).</summary>
    Task<JsonElement> MoveJ(double x, double y, double z, double rx = 0, double ry = 0, double rz = 0, double? speed = null, double? accel = null);
    /// <summary>Start the homing sequence.</summary>
    Task<JsonElement> Home();
    /// <summary>Stop jogging / continuous motion (<c>StopJog</c>).</summary>
    Task<JsonElement> Stop();

    /// <summary>Taught points as a JSON array (<c>[{ name, x, y, z, rx, ry, rz }, …]</c>).</summary>
    Task<JsonElement> GetPoints();
    /// <summary>Save the robot's current pose as a taught point (stored in the base frame).</summary>
    Task<JsonElement> TeachPoint(string name);
    /// <summary>Saved grids (pallet patterns) as a JSON array.</summary>
    Task<JsonElement> GetGrids();
    /// <summary>Saved stacks (pick/place columns) as a JSON array.</summary>
    Task<JsonElement> GetStacks();

    /// <summary>Current tool pose. One <c>GetStatus</c> call — for high-rate reads subscribe to the
    /// <c>robot.position</c> event instead.</summary>
    Task<PluginPoint> Position();
    /// <summary>True while the robot is in motion.</summary>
    Task<bool> IsMoving();
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
