using K = Controller.RobotControl.Programs.FieldKinds;

namespace Controller.RobotControl.Programs;

/// <summary>
/// The one table that describes every step type: category, label, text keyword and syntax, and
/// typed fields (docs/program-language.md §5). Published by <c>GetStepCatalog</c>; drives the
/// app's generic step fields, the text language's printer/parser and its completion, and the
/// generated block reference. <c>StepCatalogTests</c> keeps it honest against the model and the
/// executor — a new step type or ProgramStep property that is not described here fails a test.
/// </summary>
public static class StepCatalog
{
    public const int CatalogVersion  = 1;
    public const int LanguageVersion = 1;

    /// <summary>Properties every step carries (not repeated per step).</summary>
    public static readonly IReadOnlyList<string> CommonFields =
        ["id", "name", "comment", "type", "enabled", "expressions", "unknownStepType"];

    /// <summary>Legacy properties still read but no longer written by an editor.</summary>
    public static readonly IReadOnlyList<string> LegacyFields =
        ["varPointName", "varPointIndex", "jsonImageOutbound", "cncProgramSteps"];

    public static readonly IReadOnlyList<StepCategorySpec> Categories =
    [
        new() { Key = "motion",         Label = "Motion",             Description = "Send the robot somewhere — points, jumps, arcs, threads and homing." },
        new() { Key = "motionSettings", Label = "Motion Settings",    Description = "Change how later moves behave: speed, blending, tool and frame." },
        new() { Key = "flow",           Label = "Logic & Flow",       Description = "Decide what runs next — branches, loops, jumps, errors and pauses." },
        new() { Key = "variables",      Label = "Variables & Timing", Description = "Store values, edit lists, count time, and wait before continuing." },
        new() { Key = "io",             Label = "I/O & Aux Axes",     Description = "Drive digital outputs, load cells and auxiliary stepper axes." },
        new() { Key = "vision",         Label = "Vision",             Description = "Run camera inspections and capture or save images." },
        new() { Key = "background",     Label = "Background",         Description = "Start, stop and wait on programs running in parallel." },
        new() { Key = "comms",          Label = "Communication",      Description = "Talk to the operator and to other machines over HTTP." },
        new() { Key = "plugins",        Label = "Plugins",            Description = "Steps provided by installed plugins." },
    ];

    private static readonly Lazy<IReadOnlyList<StepSpec>> _steps = new(BuildSteps);
    private static readonly Lazy<Dictionary<StepType, StepSpec>> _byType =
        new(() => _steps.Value.ToDictionary(s => Enum.Parse<StepType>(s.Type)));

    /// <summary>Every step type except Unknown, in picker order.</summary>
    public static IReadOnlyList<StepSpec> Steps => _steps.Value;

    public static StepSpec? Get(StepType type) => _byType.Value.GetValueOrDefault(type);

    /// <summary>The GetStepCatalog payload.</summary>
    public static StepCatalogInfo Info() => new()
    {
        CatalogVersion  = CatalogVersion,
        LanguageVersion = LanguageVersion,
        Categories      = Categories,
        ErrorCodes      = ErrorCodes.All,
        Steps           = Steps,
        CommonFields    = CommonFields,
        LegacyFields    = LegacyFields,
    };

    // ── Builders ───────────────────────────────────────────────────────────

    private static FieldOption O(object value, string label) => new() { Value = value, Label = label };

    private static IReadOnlyList<FieldOption> Opts(params (object Value, string Label)[] o) =>
        o.Select(x => O(x.Value, x.Label)).ToList();

    private static ShowIf When(string field, params string[] values) => new() { Field = field, In = values };

    /// <summary>One field. <paramref name="expr"/> = the literal may be replaced by
    /// <c>expressions[key]</c>; <paramref name="arg"/> defaults to the key.</summary>
    private static FieldSpec F(string key, string kind, string label, string group = "",
        string? arg = null, int? pos = null, string? unit = null, object? def = null,
        double? min = null, double? max = null, double? step = null,
        IReadOnlyList<FieldOption>? options = null, bool expr = false, bool required = false,
        ShowIf? showIf = null, string? help = null, IReadOnlyList<string>? props = null,
        IReadOnlyList<string>? exprKeys = null, IReadOnlyList<string>? varKinds = null, bool hidden = false) => new()
    {
        Key = key, Kind = kind, Label = label, Group = group, Arg = arg ?? key, Position = pos,
        Unit = unit, Default = def, Min = min, Max = max, Step = step, Options = options,
        ExprKey = expr ? key : null, Required = required, ShowIf = showIf, Help = help,
        Props = props, ExprKeys = exprKeys, VarKinds = varKinds, Hidden = hidden,
    };

    private static StepSpec S(StepType type, string category, string label, string description,
        string syntax, string summary, IEnumerable<FieldSpec> fields, string? body = null,
        string? keyword = null, IReadOnlyList<string>? aliases = null) => new()
    {
        Type        = type.ToString(),
        Category    = category,
        Label       = label,
        Description = description,
        Keyword     = keyword ?? type.ToString(),
        Aliases     = aliases ?? [],
        Background  = !ProgramExecutor.IsRestrictedInBackground(type),
        Body        = body,
        Syntax      = syntax,
        Summary     = summary,
        Fields      = fields.ToList(),
    };

    private static readonly string[] Axes = ["X", "Y", "Z", "RX", "RY", "RZ"];

    private static string[] AxisProps(string prefix) => Axes.Select(a => prefix + a).ToArray();

    // Shared field groups ---------------------------------------------------

    private static FieldSpec Target(int? pos = 0, string group = "Target") =>
        F("target", K.Target, "Target", group, arg: "", pos: pos,
          props: ["pointId", "pointName", "pointNameExpr", "gridPoint", "stackPoint"],
          exprKeys: ["gridRowIndex", "gridColIndex", "gridGridIndex", "stackIndex"],
          help: "A saved point, a $points[i] element, a template naming a point, a grid or stack cell — or nothing for the current position (use with offsets).");

    private static IEnumerable<FieldSpec> Dynamics(string speedUnit = "mm/s") =>
    [
        F("speed", K.Number, "Speed", "Dynamics", unit: speedUnit, min: 0, expr: true,
          help: "Overrides the program's speed for this move only."),
        F("accel", K.Number, "Acceleration", "Dynamics", unit: "mm/s²", min: 0, expr: true),
        F("decel", K.Number, "Deceleration", "Dynamics", unit: "mm/s²", min: 0, expr: true),
    ];

    private static IEnumerable<FieldSpec> Blending() =>
    [
        F("blend", K.Bool, "Blend into next move", "Blending", def: false,
          help: "Round the corner into the next move instead of stopping."),
        F("blendRadius", K.Number, "Blend radius", "Blending", unit: "mm", min: 0, expr: true,
          showIf: When("blend", "true"), help: "Empty = the radius set by Set Blend Radius."),
    ];

    private static FieldSpec Axes6(string key, string arg, string label, string prefix, string help) =>
        F(key, K.Axes6, label, "Offsets", arg: arg, unit: "mm / deg",
          props: AxisProps(prefix), exprKeys: AxisProps(prefix), help: help);

    private static IEnumerable<FieldSpec> Modifiers(bool toolOffset = true)
    {
        yield return Axes6("offset", "offset", "Offset", "offset", "Added to the target (relative move when there is no target).");
        yield return Axes6("override", "set", "Override", "override", "Replaces an axis of the target outright.");
        if (toolOffset)
            yield return Axes6("toolOffset", "toolOffset", "Tool offset", "toolOffset", "Applied in the tool frame at execution time.");
        yield return F("localId", K.LocalRef, "Local frame", "Frame", arg: "local", props: ["localId", "localName"],
                       help: "Run this move in a local frame instead of the active one.");
    }

    private static IEnumerable<FieldSpec> StopCondition(bool aux) =>
    [
        F("moveStopCondition", K.Condition, "Stop condition", "Stop condition", arg: "until",
          help: "Watched every tick while the move runs, e.g. $io.Nano.Load >= 2000."),
        F("moveStopMode", K.Enum, "When true", "Stop condition", arg: "untilMode", def: "stop",
          options: Opts(("stop", "Stop the move"), ("record", "Record the position, keep moving"))),
        F("moveStopPositionVariableName", K.VarWrite, "Position variable", "Stop condition", arg: "untilPos",
          varKinds: aux ? ["number"] : ["list:Point"],
          help: aux ? "Receives the axis position (mm / deg, or steps)." : "A Points list set to the one pose where the condition fired."),
        F("moveStopFlagVariableName", K.VarWrite, "Fired flag", "Stop condition", arg: "untilFlag",
          varKinds: ["boolean", "number"], help: "Set to 1 if the condition fired during the move, else 0."),
    ];

    private static IEnumerable<FieldSpec> AuxAxis() =>
    [
        F("auxDeviceId", K.DeviceRef, "Device", "Axis", arg: "device", help: "The aux IO card."),
        F("auxAxisIndex", K.Integer, "Axis", "Axis", arg: "axis", def: 0, min: 0, step: 1),
    ];

    private static IEnumerable<FieldSpec> LoadCell() =>
    [
        F("loadCellDeviceId", K.DeviceRef, "Device", "Load cell", arg: "device"),
        F("loadCellPin", K.Integer, "Pin", "Load cell", arg: "pin", min: 0, step: 1),
    ];

    private static FieldSpec BackgroundProgram() =>
        F("backgroundProgramId", K.ProgramRef, "Background program", "Program", arg: "", pos: 0, required: true,
          props: ["backgroundProgramId", "backgroundProgramName"]);

    private static FieldSpec Body(string key, string label) => F(key, K.Steps, label, "Body", arg: "");

    private static FieldSpec Ms(string key, string label, string group, string arg, object? def = null,
                                ShowIf? showIf = null, string? help = null, bool expr = false, int? pos = null) =>
        F(key, K.Integer, label, group, arg: arg, pos: pos, unit: "ms", def: def, min: 0, step: 1,
          showIf: showIf, help: help, expr: expr);

    // ── The table ──────────────────────────────────────────────────────────

    private static IReadOnlyList<StepSpec> BuildSteps() =>
    [
        // ── Motion ─────────────────────────────────────────────────────────
        S(StepType.MoveL, "motion", "Move Linear", "Move to a target in a straight line",
          "MoveL <target>[, speed=…][, blend=…][, offset=(…)][, until=…]",
          "→ {target} · {speed} · blend {blendRadius}",
          [Target(), ..Dynamics(), ..Blending(), ..Modifiers(), ..StopCondition(aux: false)]),
        S(StepType.MoveJ, "motion", "Move Joint", "Move to a target by joint interpolation",
          "MoveJ <target>[, speed=…][, offset=(…)][, until=…]",
          "→ {target} · {speed}",
          [Target(), ..Dynamics(), ..Modifiers(), ..StopCondition(aux: false)]),
        S(StepType.JumpL, "motion", "Jump Linear", "Lift, move linearly over the target, then lower",
          "JumpL <target>, z=…[, zStart=…][, zEnd=…][, speed=…]",
          "→ {target} · z {jumpZ}",
          [Target(),
           F("jumpZ", K.Number, "Lift height", "Jump", arg: "z", unit: "mm", expr: true, help: "Z used for the lift and lower legs."),
           F("jumpZStart", K.Number, "Lift height (start)", "Jump", arg: "zStart", unit: "mm", expr: true, help: "Overrides the lift leg only."),
           F("jumpZEnd", K.Number, "Lift height (end)", "Jump", arg: "zEnd", unit: "mm", expr: true, help: "Overrides the lower leg only."),
           ..Dynamics(), ..Blending(), ..Modifiers(), ..StopCondition(aux: false)]),
        S(StepType.JumpJ, "motion", "Jump Joint", "Lift, move by joint interpolation over the target, then lower",
          "JumpJ <target>, z=…[, zStart=…][, zEnd=…][, speed=…]",
          "→ {target} · z {jumpZ}",
          [Target(),
           F("jumpZ", K.Number, "Lift height", "Jump", arg: "z", unit: "mm", expr: true),
           F("jumpZStart", K.Number, "Lift height (start)", "Jump", arg: "zStart", unit: "mm", expr: true),
           F("jumpZEnd", K.Number, "Lift height (end)", "Jump", arg: "zEnd", unit: "mm", expr: true),
           ..Dynamics(), ..Modifiers(), ..StopCondition(aux: false)]),
        S(StepType.MoveC, "motion", "Move Arc", "Move along a circular arc through a via point to the end target",
          "MoveC <end>, via=<point>[, speed=…][, tolerance=…]",
          "→ {target} via {via} · {speed}",
          [Target(),
           F("via", K.Target, "Via point", "Target", arg: "via", required: true,
             props: ["viaPointId", "viaPointName", "viaPointNameExpr"],
             help: "A point the arc passes through (saved point, $points[i] element or template)."),
           ..Dynamics(),
           F("arcTolerance", K.Number, "Arc tolerance", "Dynamics", arg: "tolerance", unit: "mm", def: 0.1, min: 0.001,
             help: "Largest allowed gap between the true arc and its straight segments."),
           ..Modifiers(toolOffset: false)]),
        S(StepType.ThreadMove, "motion", "Thread Move", "Move the tool in Z while rotating RZ to follow a thread",
          "ThreadMove distance=…, pitch=…[, peck=on, peckDepth=…, peckRetract=…][, exitHeight=…][, reverseOut=on]",
          "{threadDistance} · pitch {threadPitch}",
          [F("threadDistance", K.Number, "Distance", "Thread", arg: "distance", unit: "mm", expr: true, required: true),
           F("threadPitch", K.Number, "Pitch", "Thread", arg: "pitch", unit: "mm", min: 0, expr: true, required: true),
           F("threadPeck", K.Bool, "Peck", "Thread", arg: "peck", def: false),
           F("threadPeckDepth", K.Number, "Peck depth", "Thread", arg: "peckDepth", unit: "mm", min: 0, expr: true, showIf: When("threadPeck", "true")),
           F("threadPeckRetract", K.Number, "Peck retract", "Thread", arg: "peckRetract", unit: "mm", min: 0, expr: true,
             showIf: When("threadPeck", "true"), help: "Empty = retract fully to the start each peck."),
           F("threadExitHeight", K.Number, "Exit height", "Thread", arg: "exitHeight", unit: "mm", expr: true),
           F("threadReverseOut", K.Bool, "Reverse out", "Thread", arg: "reverseOut", def: false),
           ..Dynamics()]),
        S(StepType.RunHoming, "motion", "Run Homing", "Run the full homing sequence and wait for it to finish",
          "RunHoming", "", []),
        S(StepType.SetJointPosition, "motion", "Set Joint Position", "Declare one joint's value with no motion (manual homing)",
          "SetJointPosition J<n>, <value>", "J{jointIndex} = {jointValue}",
          [F("jointIndex", K.Integer, "Joint", "Joint", arg: "", pos: 0, def: 0,
             options: Opts((0, "J1"), (1, "J2"), (2, "J3"), (3, "J4"))),
           F("jointValue", K.Number, "Value", "Joint", arg: "", pos: 1, unit: "deg / mm", expr: true, required: true)]),
        S(StepType.CncProgram, "motion", "CNC Program", "Follow a toolpath generated from a DXF or SVG file",
          "CncProgram \"<file>\"[, safeZ=…]", "{cncDxfFile}",
          [F("cncDxfFile", K.Text, "File", "Toolpath", arg: "", pos: 0),
           F("cncSafeZ", K.Number, "Safe Z", "Toolpath", arg: "safeZ", unit: "mm"),
           F("cncSpec", K.Json, "Toolpath", "Toolpath", hidden: true, help: "Edited in the CNC builder.")]),
        S(StepType.SavePosition, "motion", "Save Position", "Store the current position in a Points list or a saved point",
          "SavePosition $<list>[, mode=replace|append|insert|set][, index=…][, source=actual|target] | SavePosition point <name>",
          "→ {variableName}{pointId} · {savePositionMode}",
          [F("savePositionTo", K.Enum, "Save to", "Destination", arg: "to", def: "variable",
             options: Opts(("variable", "Points variable"), ("point", "Saved point"))),
           F("variableName", K.ListVar, "Points variable", "Destination", arg: "", pos: 0, varKinds: ["list:Point"],
             showIf: When("savePositionTo", "variable")),
           F("pointId", K.PointRef, "Saved point", "Destination", arg: "point", props: ["pointId", "pointName"],
             showIf: When("savePositionTo", "point"), help: "Overwritten (or created) with the current position; recorded in the points history."),
           F("savePositionMode", K.Enum, "Mode", "Destination", arg: "mode", def: "replace",
             options: Opts(("replace", "Replace the list"), ("append", "Append"), ("insert", "Insert at index"), ("set", "Set at index")),
             showIf: When("savePositionTo", "variable")),
           F("savePositionIndex", K.Integer, "Index", "Destination", arg: "index", min: 0, step: 1, expr: true,
             showIf: When("savePositionMode", "insert", "set")),
           F("savePositionSource", K.Enum, "Position", "Destination", arg: "source", def: "actual",
             options: Opts(("actual", "Actual position"), ("target", "Last commanded target")))]),

        // ── Motion settings ────────────────────────────────────────────────
        S(StepType.SetSpeedL, "motionSettings", "Set Speed (Linear)", "Set the linear move speed, accel and decel",
          "SetSpeedL <speed>[, accel=…][, decel=…]", "{speed} · accel {accel}",
          [F("speed", K.Number, "Speed", "Dynamics", arg: "", pos: 0, unit: "mm/s", min: 0, expr: true, required: true),
           F("accel", K.Number, "Acceleration", "Dynamics", unit: "mm/s²", min: 0, expr: true),
           F("decel", K.Number, "Deceleration", "Dynamics", unit: "mm/s²", min: 0, expr: true)]),
        S(StepType.SetSpeedJ, "motionSettings", "Set Speed (Joint)", "Set the joint move speed, accel and decel",
          "SetSpeedJ <speed>[, accel=…][, decel=…]", "{speed} · accel {accel}",
          [F("speed", K.Number, "Speed", "Dynamics", arg: "", pos: 0, unit: "mm/s", min: 0, expr: true, required: true),
           F("accel", K.Number, "Acceleration", "Dynamics", unit: "mm/s²", min: 0, expr: true),
           F("decel", K.Number, "Deceleration", "Dynamics", unit: "mm/s²", min: 0, expr: true)]),
        S(StepType.SetBlendRadius, "motionSettings", "Set Blend Radius", "Set the default corner radius for later blended moves",
          "SetBlendRadius <mm>", "{blendRadius}",
          [F("blendRadius", K.Number, "Blend radius", "Blending", arg: "", pos: 0, unit: "mm", min: 0, expr: true, required: true)]),
        S(StepType.SetSpeedFactor, "motionSettings", "Set Speed Factor", "Scale this program's move speeds by a percentage",
          "SetSpeedFactor <percent>", "{speedFactor}",
          [F("speedFactor", K.Number, "Speed factor", "Dynamics", arg: "", pos: 0, unit: "%", def: 100, min: 1, max: 100, step: 1,
             expr: true, required: true, help: "Never exceeds the operator's speed override; resets to 100 when the program ends.")]),
        S(StepType.SetTool, "motionSettings", "Set Tool", "Change the active tool used by later moves",
          "SetTool <tool>", "{toolId}",
          [F("toolId", K.ToolRef, "Tool", "Tool", arg: "", pos: 0, props: ["toolId", "toolName"], help: "None = no tool offset.")]),
        S(StepType.SetLocal, "motionSettings", "Set Local", "Activate a local coordinate frame for later moves",
          "SetLocal <local>", "{localId}",
          [F("localId", K.LocalRef, "Local frame", "Frame", arg: "", pos: 0, required: true, props: ["localId", "localName"])]),
        S(StepType.ClearLocal, "motionSettings", "Clear Local", "Return later moves to the world frame",
          "ClearLocal", "", []),

        // ── Logic & flow ───────────────────────────────────────────────────
        S(StepType.IfCondition, "flow", "If Condition", "Run steps only when a condition holds, with else-if and else branches",
          "if <condition> … [elif <condition> …] [else …] end", "{condition}",
          [F("condition", K.Condition, "Condition", "Condition", arg: "", pos: 0, required: true),
           Body("ifSteps", "Then"),
           F("elseIfBranches", K.Branches, "Else if", "Body", arg: ""),
           Body("elseSteps", "Else")],
          body: "branches", keyword: "if"),
        S(StepType.Switch, "flow", "Switch", "Run the case whose value equals an expression",
          "switch <expr> case <v>[, <v>…] … [default …] end", "{switchExpr}",
          [F("switchExpr", K.Expr, "Value", "Switch", arg: "", pos: 0, required: true, help: "Evaluated once; compared with each case value."),
           F("switchCases", K.Cases, "Cases", "Body", arg: ""),
           Body("switchDefault", "Default")],
          body: "cases", keyword: "switch"),
        S(StepType.Loop, "flow", "Loop", "Repeat steps a number of times, for each list element, or while a condition holds",
          "repeat <n>|forever [as $i] … end | for $v[, $i] in $list … end | while <condition> … end",
          "{loopMode} {loopCount}{forEachVariableName}{loopWhileCondition}",
          [F("loopMode", K.Enum, "Repeat", "Loop", arg: "mode", def: "count",
             options: Opts(("count", "A number of times"), ("forEach", "For each element"), ("while", "While a condition holds"))),
           F("loopCount", K.Integer, "Count", "Loop", arg: "", pos: 0, def: 1, min: 0, step: 1, expr: true,
             showIf: When("loopMode", "count"), help: "0 = forever."),
           F("forEachVariableName", K.ListVar, "List", "Loop", arg: "in", varKinds: ["list"], showIf: When("loopMode", "forEach")),
           F("forEachValueVariableName", K.VarWrite, "Value variable", "Loop", arg: "value", varKinds: ["number", "boolean"],
             showIf: When("loopMode", "forEach"), help: "Number/Boolean lists: the element. Other lists: the index."),
           F("forEachIndexVariableName", K.VarWrite, "Index variable", "Loop", arg: "as", varKinds: ["number"],
             showIf: When("loopMode", "count", "forEach")),
           F("loopWhileCondition", K.Condition, "While", "Loop", arg: "while", showIf: When("loopMode", "while")),
           Body("loopSteps", "Body")],
          body: "steps", keyword: "repeat", aliases: ["for", "while"]),
        S(StepType.LoopControl, "flow", "Break / Next", "Leave the enclosing loop, or skip to its next iteration",
          "break | continue", "{loopControl}",
          [F("loopControl", K.Enum, "Action", "Loop", arg: "", pos: 0, def: "break",
             options: Opts(("break", "Break loop"), ("continue", "Next iteration")))],
          keyword: "break", aliases: ["continue"]),
        S(StepType.Try, "flow", "Try", "Run steps; on a runtime error retry them or run the catch steps",
          "try [retry=…][, delay=…] … [catch [$msg][, code=$code][, only=(…)] …] end",
          "retry {tryRetries} · {catchCodes}",
          [F("tryRetries", K.Integer, "Retries", "Errors", arg: "retry", def: 0, min: 0, step: 1),
           F("tryRetryDelayMs", K.Integer, "Retry delay", "Errors", arg: "delay", unit: "ms", def: 0, min: 0, step: 1),
           F("catchCodes", K.TextList, "Catch only", "Errors", arg: "only",
             options: ErrorCodes.All.Where(e => e.Catchable).Select(e => O(e.Code, e.Label)).ToList(),
             help: "Empty = every catchable error. A Raise Error code may be typed in."),
           F("errorVariableName", K.VarWrite, "Message variable", "Errors", arg: "message", varKinds: ["string"]),
           F("errorCodeVariableName", K.VarWrite, "Code variable", "Errors", arg: "code", varKinds: ["string"]),
           Body("trySteps", "Try"),
           Body("catchSteps", "Catch")],
          body: "tryCatch", keyword: "try"),
        S(StepType.RaiseError, "flow", "Raise Error", "Fail with a message — caught by an enclosing Try, otherwise ends the program",
          "raise \"<message>\"[, code=<identifier>]", "{errorCode}: {errorMessage}",
          [F("errorMessage", K.Template, "Message", "Error", arg: "", pos: 0, required: true),
           F("errorCode", K.Text, "Code", "Error", arg: "code", def: ErrorCodes.UserError, help: "An identifier a Try can filter on.")],
          keyword: "raise"),
        S(StepType.CallRoutine, "flow", "Call Routine", "Run a routine, then continue",
          "call <routine>[(param=…, …)][, out param=$var]", "{routineId}",
          [F("routineId", K.RoutineRef, "Routine", "Routine", arg: "", pos: 0, required: true, props: ["routineId", "routineName"]),
           F("routineArgs", K.ExprMap, "Arguments", "Parameters", arg: "",
             help: "One expression per in / in-out parameter (a template for a text parameter), evaluated before the call."),
           F("routineOutputs", K.MappingOut, "Outputs", "Parameters", arg: "out",
             help: "Out / in-out parameters copied into your variables when the routine returns.")],
          keyword: "call"),
        S(StepType.Return, "flow", "Return", "Leave the routine (or end the program)", "return", "", [], keyword: "return"),
        S(StepType.EndProgram, "flow", "End Program", "Finish the program now, from anywhere",
          "exit [\"<message>\"]", "{statusMessage}",
          [F("statusMessage", K.Template, "Message", "Program", arg: "", pos: 0)],
          keyword: "exit"),
        S(StepType.Label, "flow", "Label", "Mark a place Go To Label can jump to",
          "label <name>", "{labelName}",
          [F("labelName", K.Text, "Name", "Label", arg: "", pos: 0, required: true),
           F("labelId", K.Text, "Id", "Label", arg: "", hidden: true)],
          keyword: "label"),
        S(StepType.GoToLabel, "flow", "Go To Label", "Jump to a Label",
          "goto <label>", "{labelId}",
          [F("labelId", K.LabelRef, "Label", "Label", arg: "", pos: 0, required: true, props: ["labelId", "labelName"])],
          keyword: "goto"),
        S(StepType.PauseProgram, "flow", "Pause Program", "Pause — the operator continues or stops from the monitor",
          "pause", "", [], keyword: "pause"),
        S(StepType.Note, "flow", "Note", "A note or section header — not executed",
          "# text | ## Section", "{comment}",
          [F("noteStyle", K.Enum, "Style", "Note", arg: "", def: "note",
             options: Opts(("note", "Note"), ("section", "Section header"))),
           F("comment", K.Text, "Text", "Note", arg: "", pos: 0)],
          keyword: "#"),

        // ── Variables & timing ─────────────────────────────────────────────
        S(StepType.SetVariable, "variables", "Set Variable", "Assign a value or expression to a variable",
          "$<name> = <expression | \"template\">", "${variableName} = {variableExpr}",
          [F("variableName", K.VarWrite, "Variable", "Variable", arg: "", pos: 0, required: true, varKinds: ["number", "boolean", "string"]),
           F("variableExpr", K.Expr, "Value", "Variable", arg: "", pos: 1, required: true, help: "An expression; a template for a text variable.")],
          keyword: "$"),
        S(StepType.ListOp, "variables", "List", "Append, insert, remove, set, sort or copy list elements",
          "$<list>.append(…) | .insert(i, …) | .removeAt(i) | .clear() | [i] = … | [i].field = … | .sort(by=…, desc) | .reverse() | .copyFrom($other)",
          "${variableName}.{listAction}",
          [F("variableName", K.ListVar, "List", "List", arg: "", pos: 0, required: true, varKinds: ["list"]),
           F("listAction", K.Enum, "Action", "List", arg: "action", def: "append", required: true,
             options: Opts(("append", "Append"), ("insert", "Insert"), ("removeAt", "Remove at"), ("clear", "Clear"),
                           ("set", "Set element"), ("setField", "Set field"), ("sort", "Sort"), ("reverse", "Reverse"),
                           ("copyFrom", "Copy from"))),
           F("listIndex", K.Integer, "Index", "List", arg: "index", min: 0, step: 1, expr: true,
             showIf: When("listAction", "insert", "removeAt", "set", "setField")),
           F("listValueExpr", K.Expr, "Value", "List", arg: "value",
             showIf: When("listAction", "append", "insert", "set"), help: "For Number and Boolean lists."),
           F("listFields", K.ExprMap, "Fields", "List", arg: "fields",
             showIf: When("listAction", "append", "insert", "set", "setField"),
             help: "For Point and Record lists. Append with no fields on a Points list appends the current position."),
           F("listSourceVariable", K.ListVar, "Copy from", "List", arg: "from", varKinds: ["list"], showIf: When("listAction", "copyFrom")),
           F("listSortField", K.Text, "Sort by field", "List", arg: "by", showIf: When("listAction", "sort"), help: "Empty = the element value."),
           F("listSortDescending", K.Bool, "Descending", "List", arg: "desc", def: false, showIf: When("listAction", "sort"))],
          keyword: "$"),
        S(StepType.StopwatchControl, "variables", "Stopwatch", "Start, stop or reset a stopwatch variable",
          "stopwatch start|stop|reset $<name>", "{stopwatchAction} ${stopwatchVariableName}",
          [F("stopwatchAction", K.Enum, "Action", "Stopwatch", arg: "", pos: 0, def: "Start", required: true,
             options: Opts(("Start", "Start"), ("Stop", "Stop"), ("Reset", "Reset"))),
           F("stopwatchVariableName", K.VarWrite, "Stopwatch", "Stopwatch", arg: "", pos: 1, required: true, varKinds: ["stopwatch"])],
          keyword: "stopwatch"),
        S(StepType.Wait, "variables", "Wait", "Wait for a time, or until a condition holds",
          "Wait <ms> | Wait until <condition>[, timeout=…][, timedOut=$v][, onTimeout=continue|error]",
          "{waitMs}{waitCondition} · timeout {waitTimeoutMs}",
          [F("waitMode", K.Enum, "Wait for", "Wait", arg: "mode", def: "duration",
             options: Opts(("duration", "A duration"), ("condition", "A condition"))),
           Ms("waitMs", "Duration", "Wait", arg: "", pos: 0, expr: true, showIf: When("waitMode", "duration")),
           F("waitCondition", K.Condition, "Until", "Wait", arg: "until", showIf: When("waitMode", "condition")),
           Ms("waitTimeoutMs", "Timeout", "Timeout", "timeout", showIf: When("waitMode", "condition"), help: "Empty or 0 = wait forever."),
           F("waitTimeoutVariableName", K.VarWrite, "Timed-out flag", "Timeout", arg: "timedOut", varKinds: ["boolean", "number"],
             showIf: When("waitMode", "condition")),
           F("waitTimeoutAction", K.Enum, "On timeout", "Timeout", arg: "onTimeout", def: "continue",
             options: Opts(("continue", "Continue"), ("error", "Fail (catchable)")), showIf: When("waitMode", "condition"))]),

        // ── I/O, load cells, aux axes ──────────────────────────────────────
        S(StepType.SetOutput, "io", "Set Output", "Turn a digital output on or off, toggle it or pulse it",
          "SetOutput <stb|relay|io> [<device>] <number> <on|off|toggle|(expr)>[, pulse=…][, blocking=on]",
          "{outputCard} {outputNumber} → {outputValue}{outputAction}",
          [F("outputCard", K.Enum, "Card", "Output", arg: "", pos: 0, def: "stb", required: true,
             options: Opts(("stb", "Driver board"), ("relay", "USB relay"), ("io", "IO device"))),
           F("outputDeviceId", K.DeviceRef, "Device", "Output", arg: "", pos: 1, showIf: When("outputCard", "io")),
           F("outputNumber", K.Integer, "Output", "Output", arg: "", pos: 2, min: 0, step: 1, required: true),
           F("outputAction", K.Enum, "Action", "Output", arg: "action", def: "set",
             options: Opts(("set", "Set"), ("toggle", "Toggle"))),
           F("outputValue", K.Bool, "On", "Output", arg: "", pos: 3, def: true, expr: true, showIf: When("outputAction", "set"),
             help: "An expression: non-zero = on."),
           Ms("pulseMs", "Pulse", "Pulse", "pulse", help: "Set for this long, then flip back. Empty = hold."),
           F("pulseBlocking", K.Bool, "Wait for pulse", "Pulse", arg: "blocking", def: false)]),
        S(StepType.Tare, "io", "Tare Load Cell", "Zero a load cell",
          "Tare device=…, pin=…", "{loadCellDeviceId} pin {loadCellPin}", [..LoadCell()]),
        S(StepType.ReadLoadCell, "io", "Read Load Cell", "Read a load cell into a variable",
          "ReadLoadCell $<var>, device=…, pin=…[, unit=g]", "${variableName} ← {loadCellDeviceId} pin {loadCellPin}",
          [F("variableName", K.VarWrite, "Variable", "Load cell", arg: "", pos: 0, required: true, varKinds: ["number"]),
           ..LoadCell(),
           F("loadCellUnit", K.Enum, "Unit", "Load cell", arg: "unit", def: "g",
             options: Opts(("g", "g"), ("kg", "kg"), ("N", "N"), ("lb", "lb"), ("oz", "oz")))]),
        S(StepType.StartRecording, "io", "Start Recording", "Record a load cell into a list variable",
          "StartRecording $<list>, device=…, pin=…", "${recordVariableName}",
          [F("recordVariableName", K.ListVar, "Into list", "Recording", arg: "", pos: 0, required: true, varKinds: ["list:Record"]),
           ..LoadCell()]),
        S(StepType.StopRecording, "io", "Stop Recording", "Stop a recording and store its samples",
          "StopRecording device=…, pin=…", "{loadCellDeviceId} pin {loadCellPin}", [..LoadCell()]),
        S(StepType.ExportData, "io", "Export Data", "Write variables to a CSV or JSON file",
          "ExportData csv|json \"<path>\" … end", "{exportFormat} → {exportPath}",
          [F("exportFormat", K.Enum, "Format", "Export", arg: "", pos: 0, def: "csv", options: Opts(("csv", "CSV"), ("json", "JSON"))),
           F("exportPath", K.Template, "Path", "Export", arg: "", pos: 1, required: true),
           F("exportColumns", K.Rows, "Columns", "Export", arg: "")],
          body: "mapping"),
        S(StepType.AuxMove, "io", "Aux Move", "Move an aux axis by a distance or a number of steps",
          "AuxMove device=…, axis=…, distance=…|steps=…[, unit=mm|deg][, velocity=…][, absolute=on][, wait=off][, until=…]",
          "axis {auxAxisIndex} · {auxDistance}{auxSteps}",
          [..AuxAxis(),
           F("auxUnit", K.Enum, "Unit", "Move", arg: "unit", options: Opts(("mm", "mm"), ("deg", "deg")),
             help: "Empty = raw steps."),
           F("auxDistance", K.Number, "Distance", "Move", arg: "distance", expr: true, help: "In the unit above."),
           F("auxSteps", K.Integer, "Steps", "Move", arg: "steps", expr: true, help: "Used when no unit is set; the sign is the direction."),
           F("auxVelocity", K.Number, "Velocity", "Move", arg: "velocity", min: 0, expr: true),
           F("auxAccel", K.Number, "Acceleration", "Move", arg: "accel", min: 0, expr: true),
           F("auxDecel", K.Number, "Deceleration", "Move", arg: "decel", min: 0, expr: true),
           F("auxAbsolute", K.Bool, "Absolute", "Move", arg: "absolute", def: false),
           F("auxWaitForDone", K.Bool, "Wait until done", "Move", arg: "wait", def: true),
           ..StopCondition(aux: true)]),
        S(StepType.AuxContinuous, "io", "Aux Continuous Run", "Run an aux axis continuously until an Aux Stop",
          "AuxContinuous device=…, axis=…, velocity=…[, accel=…]", "axis {auxAxisIndex} · {auxVelocity}",
          [..AuxAxis(),
           F("auxUnit", K.Enum, "Unit", "Move", arg: "unit", options: Opts(("mm", "mm"), ("deg", "deg"))),
           F("auxVelocity", K.Number, "Velocity", "Move", arg: "velocity", expr: true, help: "The sign is the direction."),
           F("auxAccel", K.Number, "Acceleration", "Move", arg: "accel", min: 0, expr: true)]),
        S(StepType.AuxStop, "io", "Aux Stop", "Stop aux axis motion — ramped or immediate",
          "AuxStop device=…, axis=…[, decel=…][, immediate=on]", "axis {auxAxisIndex}",
          [..AuxAxis(),
           F("auxDecel", K.Number, "Deceleration", "Stop", arg: "decel", min: 0, expr: true),
           F("auxImmediate", K.Bool, "Immediate", "Stop", arg: "immediate", def: false)]),
        S(StepType.AuxEnable, "io", "Aux Motor Enable", "Enable or disable the motor drivers of an aux card",
          "AuxEnable device=…, axis=…, on|off", "{auxEnable}",
          [..AuxAxis(),
           F("auxEnable", K.Bool, "Enabled", "Drivers", arg: "", pos: 0, def: true)]),
        S(StepType.GripperAction, "io", "Gripper", "Open, close or grip to a jaw gap",
          "GripperAction open|close|to <mm>[, device=…, axis=…][, velocity=…][, until=…]", "{gripperAction} {gripTargetMm}",
          [F("gripperAction", K.Enum, "Action", "Gripper", arg: "", pos: 0, def: "Open", required: true,
             options: Opts(("Open", "Open"), ("Close", "Close"), ("GripTo", "Grip to"))),
           F("gripTargetMm", K.Number, "Jaw gap", "Gripper", arg: "to", unit: "mm", min: 0, expr: true, showIf: When("gripperAction", "GripTo")),
           ..AuxAxis(),
           F("auxVelocity", K.Number, "Speed", "Gripper", arg: "velocity", unit: "mm/s", min: 0, expr: true),
           F("auxAccel", K.Number, "Acceleration", "Gripper", arg: "accel", unit: "mm/s²", min: 0, expr: true),
           ..StopCondition(aux: true)]),
        S(StepType.ConveyorAction, "io", "Conveyor", "Run a conveyor forward or reverse, or stop it",
          "ConveyorAction forward|reverse|stop[, device=…, axis=…]", "{conveyorAction}",
          [F("conveyorAction", K.Enum, "Action", "Conveyor", arg: "", pos: 0, def: "RunForward", required: true,
             options: Opts(("RunForward", "Forward"), ("RunReverse", "Reverse"), ("Stop", "Stop"))),
           ..AuxAxis()]),
        S(StepType.RotaryIndex, "io", "Rotary Index", "Index a rotary axis a number of positions",
          "RotaryIndex +1|-1[, count=…][, device=…, axis=…]", "{rotaryDirection} × {rotaryCount}",
          [F("rotaryDirection", K.Integer, "Direction", "Rotary", arg: "", pos: 0, def: 1, options: Opts((1, "+1"), (-1, "-1"))),
           F("rotaryCount", K.Integer, "Count", "Rotary", arg: "count", def: 1, min: 1, step: 1),
           ..AuxAxis()]),

        // ── Vision ─────────────────────────────────────────────────────────
        S(StepType.RunVision, "vision", "Run Vision", "Run a vision program and write its results into variables",
          "RunVision <program>[, frame=pixel|normalized|robot][, pass=$v] … end", "{visionProgramId}",
          [F("visionProgramId", K.VisionRef, "Vision program", "Vision", arg: "", pos: 0, required: true,
             props: ["visionProgramId", "visionProgramName"]),
           F("outputFrame", K.Enum, "Coordinates", "Vision", arg: "frame", def: "pixel",
             options: Opts(("pixel", "Pixels"), ("normalized", "Normalized 0–1"), ("robot", "Robot (mm)"))),
           F("passVariable", K.VarWrite, "Pass variable", "Vision", arg: "pass", varKinds: ["boolean", "number"]),
           F("toolOutputs", K.MappingOut, "Tool outputs", "Outputs", arg: "",
             help: "Rows of tool · value → variable.")],
          body: "mapping"),
        S(StepType.CaptureImage, "vision", "Capture Image", "Store the current camera frame in an image variable",
          "CaptureImage $<image>[, camera=…]", "${captureImageVariableName}",
          [F("captureImageVariableName", K.VarWrite, "Image variable", "Image", arg: "", pos: 0, required: true, varKinds: ["image"]),
           F("captureImageCameraId", K.CameraRef, "Camera", "Image", arg: "camera")]),
        S(StepType.SaveImage, "vision", "Save Image", "Save a camera snapshot to a file",
          "SaveImage \"<path>\"[, camera=…]", "{saveImagePath}",
          [F("saveImagePath", K.Template, "Path", "Image", arg: "", pos: 0, required: true),
           F("saveImageCameraId", K.CameraRef, "Camera", "Image", arg: "camera")]),

        // ── Background ─────────────────────────────────────────────────────
        S(StepType.StartBackground, "background", "Start Background", "Start a background program in parallel",
          "StartBackground <program>", "{backgroundProgramId}", [BackgroundProgram()]),
        S(StepType.StopBackground, "background", "Stop Background", "Stop a background program",
          "StopBackground <program>", "{backgroundProgramId}", [BackgroundProgram()]),
        S(StepType.WaitForBackground, "background", "Wait for Background", "Wait until a background program finishes",
          "WaitForBackground <program>", "{backgroundProgramId}", [BackgroundProgram()]),

        // ── Communication ──────────────────────────────────────────────────
        S(StepType.HttpRequest, "comms", "HTTP Request", "POST JSON to a server and read values from the reply",
          "HttpRequest \"<url>\"[, wait=on][, timeout=…] send … receive … end", "{jsonUrl}",
          [F("jsonUrl", K.Text, "URL", "Request", arg: "", pos: 0, required: true),
           F("jsonWaitForResponse", K.Bool, "Wait for the reply", "Request", arg: "wait", def: true),
           Ms("jsonTimeoutMs", "Timeout", "Request", "timeout"),
           F("jsonOutbound", K.Rows, "Send", "Body", arg: "send"),
           F("jsonInbound", K.MappingOut, "Receive", "Body", arg: "receive")],
          body: "mapping"),
        S(StepType.HttpReceive, "comms", "HTTP Receive", "Wait for an HTTP POST to a named webhook",
          "HttpReceive \"<name>\"[, timeout=…] receive … end", "{httpReceiveName}",
          [F("httpReceiveName", K.Text, "Name", "Webhook", arg: "", pos: 0, required: true, help: "Other machines POST to /webhook/<name>."),
           Ms("httpReceiveTimeoutMs", "Timeout", "Webhook", "timeout"),
           F("httpReceiveInbound", K.MappingOut, "Receive", "Body", arg: "receive")],
          body: "mapping"),
        S(StepType.StatusUpdate, "comms", "Status Update", "Publish a message, warning or error to the monitor",
          "status [info|warning|error] \"<message>\"", "{statusMessage}{statusWarning}{statusError}",
          [F("statusSeverity", K.Enum, "Severity", "Status", arg: "", def: "Info",
             options: Opts(("Info", "Info"), ("Warning", "Warning"), ("Error", "Error"))),
           F("statusMessage", K.Template, "Message", "Status", arg: "", pos: 0),
           F("statusWarning", K.Template, "Warning", "Status", arg: "warning"),
           F("statusError", K.Template, "Error", "Status", arg: "error")],
          keyword: "status"),
        S(StepType.Prompt, "comms", "Prompt", "Ask the operator: show a message with buttons and an optional input",
          "prompt \"<message>\"[, title=…][, buttons=(…)][, input=number|text][, result=$v][, value=$v][, timeout=…]",
          "{promptMessage}",
          [F("promptMessage", K.Template, "Message", "Prompt", arg: "", pos: 0, required: true),
           F("promptTitle", K.Template, "Title", "Prompt", arg: "title"),
           F("promptButtons", K.TextList, "Buttons", "Prompt", arg: "buttons", def: new[] { "OK" }, min: 1, max: 4),
           F("promptResultVariableName", K.VarWrite, "Button variable", "Prompt", arg: "result", varKinds: ["number", "string"],
             help: "A number variable gets the button index (from 0); a text variable gets its label."),
           F("promptInput", K.Enum, "Input", "Input", arg: "input", def: "none",
             options: Opts(("none", "None"), ("number", "Number"), ("text", "Text"))),
           F("promptDefault", K.Expr, "Default", "Input", arg: "default", showIf: When("promptInput", "number", "text")),
           F("promptMin", K.Number, "Minimum", "Input", arg: "min", showIf: When("promptInput", "number")),
           F("promptMax", K.Number, "Maximum", "Input", arg: "max", showIf: When("promptInput", "number")),
           F("promptInputVariableName", K.VarWrite, "Input variable", "Input", arg: "value", varKinds: ["number", "string"],
             showIf: When("promptInput", "number", "text")),
           Ms("promptTimeoutMs", "Timeout", "Timeout", "timeout", def: 0, help: "0 = wait forever."),
           F("promptTimeoutButton", K.Integer, "On timeout press", "Timeout", arg: "onTimeout", min: 0, max: 3, step: 1,
             help: "Button index chosen on timeout. Empty = fail (catchable).")],
          keyword: "prompt"),

        // ── Plugins ────────────────────────────────────────────────────────
        S(StepType.Plugin, "plugins", "Plugin", "A step provided by an installed plugin",
          "plugin <id>.<step>[, timeout=…] <param> = … <output> -> $v end", "{pluginId}",
          [F("pluginId", K.PluginRef, "Plugin step", "Plugin", arg: "", pos: 0, required: true, props: ["pluginId", "pluginStepId"]),
           F("pluginParams", K.ExprMap, "Parameters", "Parameters", arg: "", help: "Typed by the plugin manifest."),
           F("pluginOutputs", K.MappingOut, "Outputs", "Outputs", arg: ""),
           Ms("pluginTimeoutMs", "Timeout", "Plugin", "timeout", help: "Empty = the manifest default; 0 = none.")],
          body: "mapping", keyword: "plugin"),
    ];
}
