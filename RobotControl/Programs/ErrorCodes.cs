namespace Controller.RobotControl.Programs;

/// <summary>
/// Runtime error codes a step fails with (<c>ExecutionContext.Fail(code, message)</c>), and whether a
/// Try block can catch them (docs/program-language.md §4.2, "Catchable vs fatal errors"). A
/// RaiseError step may also use any identifier of its own; such codes are always catchable.
/// The list is published in GetStepCatalog's <c>errorCodes</c>.
/// </summary>
public static class ErrorCodes
{
    // ── Catchable: runtime failures ────────────────────────────────────────
    public const string WaitTimeout      = "waitTimeout";
    public const string VisionTimeout    = "visionTimeout";
    public const string VisionFailed     = "visionFailed";
    public const string HttpFailed       = "httpFailed";
    public const string HttpTimeout      = "httpTimeout";
    public const string ReceiveTimeout   = "receiveTimeout";
    public const string PluginFailed     = "pluginFailed";
    public const string PluginTimeout    = "pluginTimeout";
    public const string PluginNotRunning = "pluginNotRunning";
    public const string PromptTimeout    = "promptTimeout";
    /// <summary>A template move target that names no saved point.</summary>
    public const string PointNotFound    = "pointNotFound";
    /// <summary>MoveC through collinear (or coincident) start, via and end.</summary>
    public const string ArcDegenerate    = "arcDegenerate";
    /// <summary>A ListOp / SavePosition index outside the list.</summary>
    public const string IndexOutOfRange  = "indexOutOfRange";
    /// <summary>The default RaiseError code.</summary>
    public const string UserError        = "userError";

    // ── Fatal: never catchable ─────────────────────────────────────────────
    public const string MotionFault      = "motionFault";
    public const string JointLimit       = "jointLimit";
    public const string DriverDisconnected = "driverDisconnected";
    public const string EmergencyStop    = "emergencyStop";
    public const string UserStop         = "userStop";
    public const string UnknownVariable  = "unknownVariable";
    public const string ExpressionSyntax = "expressionSyntax";
    /// <summary>A write to a computed variable.</summary>
    public const string ComputedVariable = "computedVariable";
    /// <summary>A List / Save Position step that does not fit its list: fields on a Number list,
    /// a Number value for a Points list, a missing value, an unknown action. An authoring bug.</summary>
    public const string BadListOperation = "badListOperation";

    public static readonly IReadOnlyList<ErrorCodeSpec> All =
    [
        new() { Code = WaitTimeout,      Label = "Wait timed out",                    Catchable = true },
        new() { Code = VisionTimeout,    Label = "Vision timed out",                  Catchable = true },
        new() { Code = VisionFailed,     Label = "Vision failed",                     Catchable = true },
        new() { Code = HttpFailed,       Label = "HTTP request failed",               Catchable = true },
        new() { Code = HttpTimeout,      Label = "HTTP request timed out",            Catchable = true },
        new() { Code = ReceiveTimeout,   Label = "HTTP receive timed out",            Catchable = true },
        new() { Code = PluginFailed,     Label = "Plugin step failed",                Catchable = true },
        new() { Code = PluginTimeout,    Label = "Plugin step timed out",             Catchable = true },
        new() { Code = PluginNotRunning, Label = "Plugin not running",                Catchable = true },
        new() { Code = PromptTimeout,    Label = "Prompt timed out",                  Catchable = true },
        new() { Code = PointNotFound,    Label = "Point not found",                   Catchable = true },
        new() { Code = ArcDegenerate,    Label = "Arc points are in a line",          Catchable = true },
        new() { Code = IndexOutOfRange,  Label = "List index out of range",           Catchable = true },
        new() { Code = UserError,        Label = "Raised error",                      Catchable = true },
        new() { Code = MotionFault,      Label = "Motion fault",                      Catchable = false },
        new() { Code = JointLimit,       Label = "Joint limit",                       Catchable = false },
        new() { Code = DriverDisconnected, Label = "Driver disconnected",             Catchable = false },
        new() { Code = EmergencyStop,    Label = "Emergency stop",                    Catchable = false },
        new() { Code = UserStop,         Label = "Stopped by user",                   Catchable = false },
        new() { Code = UnknownVariable,  Label = "Unknown variable",                  Catchable = false },
        new() { Code = ExpressionSyntax, Label = "Expression error",                  Catchable = false },
        new() { Code = ComputedVariable, Label = "Write to a computed variable",      Catchable = false },
        new() { Code = BadListOperation, Label = "List step does not fit the list",  Catchable = false },
    ];

    private static readonly Dictionary<string, ErrorCodeSpec> ByCode =
        All.ToDictionary(e => e.Code, StringComparer.Ordinal);

    /// <summary>True when a Try can catch <paramref name="code"/>: a listed catchable code, or any
    /// code not in the list (a RaiseError step's own identifier).</summary>
    public static bool IsCatchable(string code) =>
        !ByCode.TryGetValue(code, out var spec) || spec.Catchable;

    public static bool IsKnown(string code) => ByCode.ContainsKey(code);
}
