using System.Text.Json.Serialization;

namespace Controller.RobotControl.Language;

// The text-language (SRS) wire contract (docs/program-language.md §6.9) — frozen by package A,
// produced by package D's FormatProgramText / ParseProgramText.

/// <summary>
/// One parse or validation problem located in the text. Lines and columns are 1-based; columns
/// count UTF-16 code units (JS string indexing). <see cref="EndLine"/>/<see cref="EndCol"/> are
/// inclusive-exclusive like an editor selection: the span is [col, endCol) on a one-line problem.
/// </summary>
public sealed class TextProblem
{
    [JsonPropertyName("line")]     public int     Line     { get; init; }
    [JsonPropertyName("col")]      public int     Col      { get; init; }
    [JsonPropertyName("endLine")]  public int     EndLine  { get; init; }
    [JsonPropertyName("endCol")]   public int     EndCol   { get; init; }
    /// <summary>"error" | "warning".</summary>
    [JsonPropertyName("severity")] public string  Severity { get; init; } = "error";
    /// <summary>A <see cref="TextProblemCodes"/> code (source "parse") or a ValidationCodes code (source "validate").</summary>
    [JsonPropertyName("code")]     public string  Code     { get; init; } = "";
    [JsonPropertyName("message")]  public string  Message  { get; init; } = "";
    [JsonPropertyName("stepId"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public string? StepId { get; init; }
    /// <summary>The step field (validator field path, or a catalog key) when the problem is about one.</summary>
    [JsonPropertyName("field"), JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)]
    public string? Field { get; init; }
    /// <summary>"parse" | "validate".</summary>
    [JsonPropertyName("source")]   public string  Source   { get; init; } = "parse";
}

/// <summary>Where a step sits in the text: lines [line, endLine], 1-based, inclusive (a block's
/// endLine is its <c>end</c>).</summary>
public sealed class ProgramTextMapEntry
{
    [JsonPropertyName("stepId")]   public string StepId   { get; init; } = "";
    /// <summary>The validator's stepPath ("steps[2].switchCases[0].steps[1]").</summary>
    [JsonPropertyName("stepPath")] public string StepPath { get; init; } = "";
    [JsonPropertyName("line")]     public int    Line     { get; init; }
    [JsonPropertyName("endLine")]  public int    EndLine  { get; init; }
}

/// <summary>FormatProgramText → this.</summary>
public sealed class FormatProgramTextResult
{
    [JsonPropertyName("text")]            public string                       Text            { get; init; } = "";
    [JsonPropertyName("languageVersion")] public int                          LanguageVersion { get; init; }
    [JsonPropertyName("map")]             public List<ProgramTextMapEntry>    Map             { get; init; } = new();
}

/// <summary>ParseProgramText → this.</summary>
public sealed class ParseProgramTextResult
{
    /// <summary>False when any error-severity parse problem exists (validation errors do not clear it).</summary>
    [JsonPropertyName("ok")]       public bool                      Ok       { get; init; }
    /// <summary>base's metadata + the parsed variables and steps, ids reconciled; null when not ok.</summary>
    [JsonPropertyName("program")]  public BuiltProgram?             Program  { get; init; }
    [JsonPropertyName("problems")] public List<TextProblem>         Problems { get; init; } = new();
    [JsonPropertyName("map")]      public List<ProgramTextMapEntry> Map      { get; init; } = new();
}

/// <summary>Parse problem codes (docs/program-language.md §6.11). Name-resolution codes
/// (unknownPoint, unknownTool, unknownRoutine, …) reuse the ValidationCodes names as warnings.</summary>
public static class TextProblemCodes
{
    public const string Syntax            = "syntax";
    public const string UnknownKeyword    = "unknownKeyword";
    public const string UnknownArgument   = "unknownArgument";
    public const string DuplicateArgument = "duplicateArgument";
    public const string MissingArgument   = "missingArgument";
    public const string BadValue          = "badValue";
    public const string ExpressionSyntax  = "expressionSyntax";
    public const string UnknownFunction   = "unknownFunction";
    public const string BadArity          = "badArity";
    public const string MissingEnd        = "missingEnd";
    public const string UnexpectedEnd     = "unexpectedEnd";
    public const string MisplacedClause   = "misplacedClause";
    public const string DeclarationKind   = "declarationKind";
    public const string StringRequired    = "stringRequired";
    public const string DuplicateId       = "duplicateId";
    public const string UnknownLabel      = "unknownLabel";
    /// <summary>Warning: a device name that matches no IO / aux device (the text is kept as the id).</summary>
    public const string UnknownDevice     = "unknownDevice";
    /// <summary>Warning: a camera name that matches no camera (the text is kept as the id).</summary>
    public const string UnknownCamera     = "unknownCamera";
    /// <summary>Warning: an <c>@with</c> key that is not a step / variable field (it is dropped).</summary>
    public const string UnknownField      = "unknownField";
}
