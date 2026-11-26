using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Alarms;

/// <summary>Alarm severities, most severe first.</summary>
public static class AlarmSeverity
{
    /// <summary>Safety: the e-stop. Motion is blocked until reset.</summary>
    public const string Critical = "critical";
    /// <summary>Something stopped: a program error, a joint limit, the motor driver lost.</summary>
    public const string Error    = "error";
    /// <summary>Degraded but running: a device or plugin dropped out.</summary>
    public const string Warning  = "warning";
    public const string Info     = "info";

    public static int Rank(string severity) => severity switch
    {
        Critical => 0, Error => 1, Warning => 2, _ => 3,
    };
}

/// <summary>Help for one alarm code: what it is, why it happens, what to do.</summary>
public sealed record AlarmCatalogEntry(string Code, string Title, string Severity, string Cause, string Remedy, bool? Catchable);

/// <summary>
/// The one catalog of alarm help (title, cause, remedy) — every program error code of
/// <see cref="ErrorCodes"/> plus the alarm-only codes the controller raises itself. Served by
/// <c>GetAlarmCatalog</c>; the app reads it from there rather than keeping its own copy.
/// </summary>
public static class AlarmCatalog
{
    // ── Alarm-only codes (never a program's error code) ─────────────────────
    /// <summary>An edge device (IO board or aux stepper board) disconnected.</summary>
    public const string DeviceDisconnected = "deviceDisconnected";
    /// <summary>A plugin process crashed (exited or stopped answering).</summary>
    public const string PluginCrashed      = "pluginCrashed";
    /// <summary>A program error with no specific code (an unexpected step failure).</summary>
    public const string ProgramError       = "programError";

    private static AlarmCatalogEntry E(string code, string title, string severity, string cause, string remedy) =>
        new(code, title, severity, cause, remedy, ErrorCodes.IsKnown(code) ? ErrorCodes.IsCatchable(code) : null);

    public static readonly IReadOnlyList<AlarmCatalogEntry> All =
    [
        // ── Safety ─────────────────────────────────────────────────────────
        E(ErrorCodes.EmergencyStop, "Emergency stop", AlarmSeverity.Critical,
          "The e-stop circuit opened: the button was pressed, the circuit was broken, or the e-stop input's device went offline. The motor drivers lost power and every program was stopped.",
          "Find out why it was pressed and make the cell safe. Release (twist) the e-stop button, check the input reads healthy, press Reset, then home the robot before moving it."),

        // ── Motion and hardware ────────────────────────────────────────────
        E(ErrorCodes.JointLimit, "Joint limit", AlarmSeverity.Error,
          "A move would have taken a joint past its soft limit, so motion was halted.",
          "Jog the joint back into range (turn on Ignore limits if it is already past), then clear the fault. Check the program's points and the configured limits."),
        E(ErrorCodes.MotionFault, "Motion fault", AlarmSeverity.Error,
          "A move could not be executed: its target did not resolve, or the motion system rejected it.",
          "Check the step's target point exists and is reachable, then run the program again."),
        E(ErrorCodes.DriverDisconnected, "Motor driver disconnected", AlarmSeverity.Error,
          "The controller lost the USB connection to the motor driver board.",
          "Check the driver board's USB cable and power. The robot must be homed again once it reconnects."),
        E(DeviceDisconnected, "Device disconnected", AlarmSeverity.Warning,
          "An IO or aux stepper board stopped answering on its serial port.",
          "Check the board's USB cable and power. It reconnects on its own; programs that use it fail until it does."),
        E(PluginCrashed, "Plugin crashed", AlarmSeverity.Warning,
          "A plugin process exited unexpectedly or could not be restarted.",
          "Open the plugin's page and read its log. Restart it once the cause is fixed."),

        // ── Program: catchable runtime failures ────────────────────────────
        E(ErrorCodes.WaitTimeout, "Wait timed out", AlarmSeverity.Error,
          "A Wait step's condition did not become true within its timeout.",
          "Check the input or variable the wait is watching, or raise the timeout. Wrap the step in a Try to recover automatically."),
        E(ErrorCodes.VisionTimeout, "Vision timed out", AlarmSeverity.Error,
          "A vision step did not get a result from its camera within the timeout.",
          "Check the camera is connected and streaming, then run again. Raise the timeout for slow cameras."),
        E(ErrorCodes.VisionFailed, "Vision failed", AlarmSeverity.Error,
          "A vision inspection ran but did not pass, or its program could not run.",
          "Check the part is in view and lit, and review the vision program's tools and thresholds."),
        E(ErrorCodes.HttpFailed, "HTTP request failed", AlarmSeverity.Error,
          "An HTTP step got an error response or could not reach the server.",
          "Check the URL, the network and the server, then run again."),
        E(ErrorCodes.HttpTimeout, "HTTP request timed out", AlarmSeverity.Error,
          "An HTTP step's server did not answer within the timeout.",
          "Check the server is running and reachable, or raise the timeout."),
        E(ErrorCodes.ReceiveTimeout, "HTTP receive timed out", AlarmSeverity.Error,
          "A step waiting for an inbound HTTP message did not receive one in time.",
          "Check the sender is running and posting to the right address, or raise the timeout."),
        E(ErrorCodes.PluginFailed, "Plugin step failed", AlarmSeverity.Error,
          "A plugin step or function reported an error.",
          "Read the plugin's message and log on its page, fix the cause, then run again."),
        E(ErrorCodes.PluginTimeout, "Plugin step timed out", AlarmSeverity.Error,
          "A plugin step did not finish within its timeout.",
          "Check the plugin is healthy and not blocked, or raise the step's timeout."),
        E(ErrorCodes.PluginNotRunning, "Plugin not running", AlarmSeverity.Error,
          "A step used a plugin that is stopped, disabled or crashed.",
          "Start the plugin from its page (enable it first if needed), then run again."),
        E(ErrorCodes.PromptTimeout, "Prompt timed out", AlarmSeverity.Error,
          "An operator prompt was not answered within its timeout.",
          "Answer prompts in time, or raise or remove the prompt's timeout."),
        E(ErrorCodes.PointNotFound, "Point not found", AlarmSeverity.Error,
          "A move named a saved point that does not exist.",
          "Teach the missing point, or correct the point name or expression in the step."),
        E(ErrorCodes.ArcDegenerate, "Arc points are in a line", AlarmSeverity.Error,
          "A circular move's start, via and end points are in a straight line (or the same), so no arc fits.",
          "Move the via point off the line between start and end."),
        E(ErrorCodes.IndexOutOfRange, "List index out of range", AlarmSeverity.Error,
          "A list step or point reference used an index outside the list.",
          "Check the index expression and the list's length before using it."),
        E(ErrorCodes.UserError, "Raised error", AlarmSeverity.Error,
          "The program raised this error itself with a Raise Error step.",
          "Read the message the program gave; it describes the condition the program detected."),

        // ── Program: fatal ─────────────────────────────────────────────────
        E(ErrorCodes.UserStop, "Stopped by user", AlarmSeverity.Info,
          "The program was stopped by an operator.",
          "No action needed. Continue or run the program again when ready."),
        E(ErrorCodes.UnknownVariable, "Unknown variable", AlarmSeverity.Error,
          "A step or expression used a variable the program does not declare.",
          "Declare the variable, or correct its name in the step. Validate the program to find every use."),
        E(ErrorCodes.ExpressionSyntax, "Expression error", AlarmSeverity.Error,
          "An expression could not be parsed or evaluated.",
          "Open the step and fix the expression; the message gives the position of the problem."),
        E(ErrorCodes.ComputedVariable, "Write to a computed variable", AlarmSeverity.Error,
          "A step tried to set a computed variable, which is a formula and cannot be written.",
          "Write to a stored variable instead, or change the formula."),
        E(ErrorCodes.BadListOperation, "List step does not fit the list", AlarmSeverity.Error,
          "A list step's action or value does not match the list's type.",
          "Open the step and pick an action and value that suit the list."),
        E(ProgramError, "Program error", AlarmSeverity.Error,
          "A step failed unexpectedly.",
          "Read the message, check the failing step's settings and the devices it uses, then run again."),
    ];

    private static readonly Dictionary<string, AlarmCatalogEntry> ByCode =
        All.ToDictionary(e => e.Code, StringComparer.Ordinal);

    /// <summary>The help for <paramref name="code"/>. A code with no entry of its own (a RaiseError
    /// step's identifier, or a new error code) gets a generic entry under its label.</summary>
    public static AlarmCatalogEntry Get(string code)
    {
        if (ByCode.TryGetValue(code, out var e)) return e;
        var label = ErrorCodes.All.FirstOrDefault(c => c.Code == code)?.Label;
        return ErrorCodes.IsKnown(code)
            ? new AlarmCatalogEntry(code, label ?? code, AlarmSeverity.Error,
                "The program stopped with this error.", "Read the message and check the failing step.", ErrorCodes.IsCatchable(code))
            : new AlarmCatalogEntry(code, code, AlarmSeverity.Error,
                "A code the program raised itself (Raise Error).", "Read the message the program gave.", true);
    }

    public static bool Has(string code) => ByCode.ContainsKey(code);
}
