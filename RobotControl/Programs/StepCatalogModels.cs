using System.Text.Json.Serialization;

namespace Controller.RobotControl.Programs;

// The GetStepCatalog wire shape (docs/program-language.md §5.2). Frozen by package A: the app
// (src/models/robotModels.tsx StepCatalog / FieldSpec) and the text language read it.

/// <summary><c>GetStepCatalog {}</c> → this.</summary>
public sealed class StepCatalogInfo
{
    [JsonPropertyName("catalogVersion")]  public int                         CatalogVersion  { get; init; }
    [JsonPropertyName("languageVersion")] public int                         LanguageVersion { get; init; }
    [JsonPropertyName("categories")]      public IReadOnlyList<StepCategorySpec> Categories  { get; init; } = [];
    [JsonPropertyName("errorCodes")]      public IReadOnlyList<ErrorCodeSpec> ErrorCodes     { get; init; } = [];
    [JsonPropertyName("steps")]           public IReadOnlyList<StepSpec>     Steps           { get; init; } = [];
    /// <summary>ProgramStep properties every step has (<c>id name comment type enabled expressions</c>,
    /// plus <c>unknownStepType</c> of the Unknown placeholder) — not repeated in each step's fields.</summary>
    [JsonPropertyName("commonFields")]    public IReadOnlyList<string>       CommonFields    { get; init; } = [];
    /// <summary>Read-only legacy properties no editor writes any more; the text language carries
    /// them in <c>@with {…}</c>.</summary>
    [JsonPropertyName("legacyFields")]    public IReadOnlyList<string>       LegacyFields    { get; init; } = [];
}

public sealed class StepCategorySpec
{
    [JsonPropertyName("key")]         public string Key         { get; init; } = "";
    [JsonPropertyName("label")]       public string Label       { get; init; } = "";
    [JsonPropertyName("description")] public string Description { get; init; } = "";
}

public sealed class ErrorCodeSpec
{
    [JsonPropertyName("code")]      public string Code      { get; init; } = "";
    [JsonPropertyName("label")]     public string Label     { get; init; } = "";
    /// <summary>True when a Try block can catch it (runtime failures); false for authoring bugs,
    /// motion faults and a user Stop.</summary>
    [JsonPropertyName("catchable")] public bool   Catchable { get; init; }
}

public sealed class StepSpec
{
    /// <summary>The <see cref="StepType"/> name.</summary>
    [JsonPropertyName("type")]        public string Type        { get; init; } = "";
    /// <summary>A <see cref="StepCategorySpec.Key"/>.</summary>
    [JsonPropertyName("category")]    public string Category    { get; init; } = "";
    [JsonPropertyName("label")]       public string Label       { get; init; } = "";
    [JsonPropertyName("description")] public string Description { get; init; } = "";
    /// <summary>Canonical text keyword (matched case-insensitively).</summary>
    [JsonPropertyName("keyword")]     public string Keyword     { get; init; } = "";
    [JsonPropertyName("aliases")]     public IReadOnlyList<string> Aliases { get; init; } = [];
    /// <summary>Runs in a background program (false = a background program skips it).</summary>
    [JsonPropertyName("background")]  public bool   Background  { get; init; } = true;
    /// <summary>null | "steps" | "branches" (if) | "cases" (switch) | "tryCatch" | "mapping".</summary>
    [JsonPropertyName("body")]        public string? Body       { get; init; }
    /// <summary>Human-readable text syntax, for hover help.</summary>
    [JsonPropertyName("syntax")]      public string Syntax      { get; init; } = "";
    /// <summary>Row summary template: <c>{fieldKey}</c> placeholders; a part whose value is empty is dropped.</summary>
    [JsonPropertyName("summary")]     public string Summary     { get; init; } = "";
    [JsonPropertyName("fields")]      public IReadOnlyList<FieldSpec> Fields { get; init; } = [];
}

/// <summary>
/// One editable field of a step. <see cref="Key"/> is a ProgramStep JSON property, or a composite
/// (<c>target</c>, <c>via</c>, <c>offset</c>, <c>override</c>, <c>toolOffset</c>) whose stored
/// properties are listed in <see cref="Props"/>. Reference kinds (<c>pointRef</c>, <c>toolRef</c>, …)
/// also list both their id and their display-name property there.
/// </summary>
public sealed class FieldSpec
{
    [JsonPropertyName("key")]      public string  Key      { get; init; } = "";
    /// <summary>Named-argument name in text (defaults to key); "" = positional only.</summary>
    [JsonPropertyName("arg")]      public string  Arg      { get; init; } = "";
    /// <summary>0-based positional slot in text, or null.</summary>
    [JsonPropertyName("position")] public int?    Position { get; init; }
    [JsonPropertyName("label")]    public string  Label    { get; init; } = "";
    /// <summary>UI section (GroupCard).</summary>
    [JsonPropertyName("group")]    public string  Group    { get; init; } = "";
    /// <summary>See <see cref="FieldKinds"/>.</summary>
    [JsonPropertyName("kind")]     public string  Kind     { get; init; } = FieldKinds.Number;
    [JsonPropertyName("unit"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public string? Unit { get; init; }
    /// <summary>Shown as placeholder; null = "program default" / none.</summary>
    [JsonPropertyName("default")]  public object? Default  { get; init; }
    [JsonPropertyName("min"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public double? Min { get; init; }
    [JsonPropertyName("max"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public double? Max { get; init; }
    [JsonPropertyName("step"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public double? Step { get; init; }
    /// <summary>For enum (and option-limited integer/textList) fields.</summary>
    [JsonPropertyName("options"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public IReadOnlyList<FieldOption>? Options { get; init; }
    /// <summary>Key in <c>step.expressions</c> when an expression may replace the literal; null = literal only.</summary>
    [JsonPropertyName("exprKey")]  public string? ExprKey  { get; init; }
    /// <summary>Composite fields: every <c>expressions</c> key the field's parts may use.</summary>
    [JsonPropertyName("exprKeys"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public IReadOnlyList<string>? ExprKeys { get; init; }
    [JsonPropertyName("required")] public bool    Required { get; init; }
    /// <summary>Shown only when another field's value (as text; an absent field reads as its
    /// default; booleans as "true"/"false") is one of <see cref="ShowIf.In"/>.</summary>
    [JsonPropertyName("showIf"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public ShowIf? ShowIf { get; init; }
    [JsonPropertyName("help"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public string? Help { get; init; }
    /// <summary>The ProgramStep properties this field stores into; null = just <see cref="Key"/>.</summary>
    [JsonPropertyName("props"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public IReadOnlyList<string>? Props { get; init; }
    /// <summary>Variable pickers: the kinds offered — number, boolean, string, image, stopwatch, list,
    /// or list:&lt;Number|Boolean|Point|Record&gt; for one element type.</summary>
    [JsonPropertyName("varKinds"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public IReadOnlyList<string>? VarKinds { get; init; }
    /// <summary>Not rendered by the generic editor (an id the app manages, a field edited elsewhere).</summary>
    [JsonPropertyName("hidden"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingDefault)]
    public bool Hidden { get; init; }

    /// <summary><see cref="Props"/>, or just <see cref="Key"/>.</summary>
    [JsonIgnore] public IReadOnlyList<string> StoredProps => Props ?? [Key];
}

public sealed class FieldOption
{
    /// <summary>The stored value (a string, or a number for integer fields).</summary>
    [JsonPropertyName("value")] public object Value { get; init; } = "";
    [JsonPropertyName("label")] public string Label { get; init; } = "";
}

public sealed class ShowIf
{
    [JsonPropertyName("field")] public string                Field { get; init; } = "";
    [JsonPropertyName("in")]    public IReadOnlyList<string> In    { get; init; } = [];
}

/// <summary>The <see cref="FieldSpec.Kind"/> values (docs/program-language.md §5.2).</summary>
public static class FieldKinds
{
    public const string Number    = "number";
    public const string Integer   = "integer";
    public const string Bool      = "bool";
    public const string Enum      = "enum";
    public const string Text      = "text";
    public const string Template  = "template";
    public const string Expr      = "expr";
    public const string Condition = "condition";
    public const string VarWrite  = "varWrite";
    public const string VarRead   = "varRead";
    public const string ListVar   = "listVar";
    public const string PointRef   = "pointRef";
    public const string ToolRef    = "toolRef";
    public const string LocalRef   = "localRef";
    public const string GridRef    = "gridRef";
    public const string StackRef   = "stackRef";
    public const string RoutineRef = "routineRef";
    public const string ProgramRef = "programRef";
    public const string VisionRef  = "visionRef";
    public const string DeviceRef  = "deviceRef";
    public const string CameraRef  = "cameraRef";
    public const string LabelRef   = "labelRef";
    /// <summary>Plugin step identity: pluginId + pluginStepId.</summary>
    public const string PluginRef  = "pluginRef";
    /// <summary>Composite: a move target (point / expression / grid / stack).</summary>
    public const string Target     = "target";
    /// <summary>Composite: &lt;prefix&gt;X…RZ (rx/ry hidden on 4-axis robots).</summary>
    public const string Axes6      = "axes6";
    /// <summary>A list of {key, variableName} rows: a value read out of the step into a variable.</summary>
    public const string MappingOut = "mappingOut";
    /// <summary>A list of {key, expr | listVar | imageVar} rows sent out of the program.</summary>
    public const string Rows       = "rows";
    /// <summary>A {name → expression/template} dictionary (routine arguments, list fields, plugin params).</summary>
    public const string ExprMap    = "exprMap";
    /// <summary>A list of strings (prompt buttons, catch codes).</summary>
    public const string TextList   = "textList";
    /// <summary>A nested step list.</summary>
    public const string Steps      = "steps";
    /// <summary>If's else-if branches ({id, condition, steps}).</summary>
    public const string Branches   = "branches";
    /// <summary>Switch cases ({id, values, steps}).</summary>
    public const string Cases      = "cases";
    /// <summary>Opaque JSON edited elsewhere (the CNC builder).</summary>
    public const string Json       = "json";

    public static readonly IReadOnlyList<string> All =
    [
        Number, Integer, Bool, Enum, Text, Template, Expr, Condition, VarWrite, VarRead, ListVar,
        PointRef, ToolRef, LocalRef, GridRef, StackRef, RoutineRef, ProgramRef, VisionRef, DeviceRef,
        CameraRef, LabelRef, PluginRef, Target, Axes6, MappingOut, Rows, ExprMap, TextList, Steps,
        Branches, Cases, Json,
    ];
}
