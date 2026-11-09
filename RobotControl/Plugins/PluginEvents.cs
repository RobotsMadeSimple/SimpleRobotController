namespace Controller.RobotControl.Plugins;

/// <summary>
/// Event names the controller publishes to plugins (docs/plugins.md §4.4) and the
/// subscription glob matching. Payload shapes are documented next to each constant.
/// </summary>
public static class PluginEvents
{
    // ── program lifecycle: { programName, isBackground, runCount, error? } ──
    public const string ProgramStarted  = "program.started";
    public const string ProgramResumed  = "program.resumed";
    public const string ProgramPaused   = "program.paused";
    public const string ProgramStopped  = "program.stopped";
    public const string ProgramFinished = "program.finished";
    public const string ProgramError    = "program.error";

    // ── steps: { programName, stepId, stepType, stepName?, stepIndex, description } ──
    public const string StepStarted   = "step.started";
    public const string StepCompleted = "step.completed";
    public const string StepSkipped   = "step.skipped";

    /// <summary><c>{ changes: { "stb.in1": 1, … } }</c> — only changed keys, polled at ioIntervalMs.</summary>
    public const string IoChanged = "io.changed";

    /// <summary><c>{ x,y,z,rx,ry,rz, moving }</c> at positionIntervalMs.</summary>
    public const string RobotPosition     = "robot.position";
    public const string RobotHomed        = "robot.homed";
    public const string RobotFault        = "robot.fault";
    public const string RobotFaultCleared = "robot.faultCleared";
    public const string RobotEstop        = "robot.estop";

    /// <summary>The <c>GetStatus</c> payload at statusIntervalMs.</summary>
    public const string Status = "status";

    /// <summary><c>{ pluginId }</c> for other plugins.</summary>
    public const string PluginStarted = "plugin.started";
    public const string PluginStopped = "plugin.stopped";

    /// <summary>
    /// Periodic/state events that may be dropped (oldest first) when a plugin's outbound
    /// queue is full. Every other event is never dropped: a full queue disconnects the plugin.
    /// </summary>
    public static bool IsDroppable(string name) =>
        name is RobotPosition or Status or IoChanged;

    /// <summary>
    /// Subscription glob match: <c>*</c> matches everything; a pattern ending in <c>*</c>
    /// matches by prefix (<c>program.*</c>); anything else must match exactly.
    /// </summary>
    public static bool Matches(string pattern, string name)
    {
        if (pattern == "*") return true;
        if (pattern.EndsWith('*')) return name.StartsWith(pattern[..^1], StringComparison.Ordinal);
        return string.Equals(pattern, name, StringComparison.Ordinal);
    }

    /// <summary>True when any of <paramref name="patterns"/> matches <paramref name="name"/>.</summary>
    public static bool MatchesAny(IEnumerable<string> patterns, string name)
    {
        foreach (var p in patterns) if (Matches(p, name)) return true;
        return false;
    }
}
