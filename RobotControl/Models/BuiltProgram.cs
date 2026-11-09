using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

public class BuiltProgram
{
    [JsonPropertyName("id")]
    public string Id { get; set; } = "";
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";
    [JsonPropertyName("description")]
    public string Description { get; set; } = "";
    [JsonPropertyName("steps")]
    public List<ProgramStep> Steps { get; set; } = new();
    [JsonPropertyName("variables")]
    public List<ProgramVariable>? Variables { get; set; }
    /// <summary>
    /// Optional user-authored HTML for a custom display page: a dashboard that shows this
    /// program's variables/state/images/graphs and offers inputs + a run button. Rendered in a
    /// sandboxed WebView in the app, which brokers data through the host bridge — see the app's
    /// DisplayPage. Null/empty means the program has no custom page.
    /// </summary>
    [JsonPropertyName("displayPageHtml")]
    public string? DisplayPageHtml { get; set; }
    /// <summary>
    /// Which editor authored the display page: "widgets" (a drag-and-drop grid, stored in
    /// <see cref="DisplayPageLayout"/> and compiled to HTML by the app) or "html" (the raw HTML
    /// in <see cref="DisplayPageHtml"/>). Null reads as "html" for pages authored before widgets.
    /// </summary>
    [JsonPropertyName("displayPageMode")]
    public string? DisplayPageMode { get; set; }
    /// <summary>
    /// The visual display-page layout (grid of configured widgets). Stored opaquely — the app
    /// owns the widget schema and compiles this to the bridge HTML the WebView runs. Null for a
    /// raw-HTML page or a program with no display page.
    /// </summary>
    [JsonPropertyName("displayPageLayout")]
    public JsonElement? DisplayPageLayout { get; set; }
    [JsonPropertyName("lastUpdatedUnixMs")]
    public long LastUpdatedUnixMs { get; set; }
    /// <summary>Routines are hidden from the program list and can only be called from a program step.</summary>
    [JsonPropertyName("isRoutine")]
    public bool IsRoutine { get; set; } = false;
    /// <summary>Background programs run in parallel with the main program; motion/tool/homing steps are skipped.</summary>
    [JsonPropertyName("isBackground")]
    public bool IsBackground { get; set; } = false;
    /// <summary>When true (default), all running background programs are stopped when the main program finishes.</summary>
    [JsonPropertyName("killBackgroundOnStop")]
    public bool KillBackgroundOnStop { get; set; } = true;
    /// <summary>
    /// The program's preferred editor view, owned by the app: "blocks" or "text"
    /// (docs/program-language.md §7.1). Null = the app's default. Not used by the controller.
    /// </summary>
    [JsonPropertyName("editorMode")]
    public string? EditorMode { get; set; }
}

public class SaveBuiltProgramParams
{
    [JsonPropertyName("id")]                   public string Id                   { get; set; } = "";
    [JsonPropertyName("name")]                 public string Name                 { get; set; } = "";
    [JsonPropertyName("description")]          public string Description          { get; set; } = "";
    [JsonPropertyName("steps")]                public List<ProgramStep> Steps     { get; set; } = new();
    [JsonPropertyName("variables")]            public List<ProgramVariable>? Variables { get; set; }
    [JsonPropertyName("displayPageHtml")]      public string? DisplayPageHtml      { get; set; }
    [JsonPropertyName("displayPageMode")]      public string? DisplayPageMode      { get; set; }
    [JsonPropertyName("displayPageLayout")]    public JsonElement? DisplayPageLayout { get; set; }
    [JsonPropertyName("isRoutine")]            public bool IsRoutine              { get; set; } = false;
    [JsonPropertyName("isBackground")]         public bool IsBackground           { get; set; } = false;
    [JsonPropertyName("killBackgroundOnStop")] public bool KillBackgroundOnStop   { get; set; } = true;
    /// <summary>"blocks" | "text"; null (absent) keeps the saved program's value.</summary>
    [JsonPropertyName("editorMode")]           public string? EditorMode          { get; set; }
}

public class BuiltProgramNameParams
{
    [JsonPropertyName("name")] public string Name { get; set; } = "";
}

/// <summary>
/// ExecuteBuiltProgram { name, startAtStepId?, breakpoints?, stepMode? } — the debug options are
/// docs/program-language.md §4.2 "Debugging". Older clients send only the name.
/// </summary>
public class ExecuteBuiltProgramParams : BuiltProgramNameParams
{
    /// <summary>Run from this step: the frame stack is built down its path (enclosing loops at iteration 0).</summary>
    [JsonPropertyName("startAtStepId")] public string?       StartAtStepId { get; set; }
    /// <summary>Step ids to pause BEFORE (pauseReason "breakpoint"). Ignored by background programs.</summary>
    [JsonPropertyName("breakpoints")]   public List<string>? Breakpoints   { get; set; }
    /// <summary>Pause before every step (pauseReason "step").</summary>
    [JsonPropertyName("stepMode")]      public bool?         StepMode      { get; set; }

    /// <summary>True when any debug option is set.</summary>
    [JsonIgnore] public bool HasDebugOptions =>
        !string.IsNullOrEmpty(StartAtStepId) || (Breakpoints?.Count ?? 0) > 0 || StepMode == true;
}

/// <summary>SetBreakpoints { stepIds } — replaces the running foreground program's breakpoints.</summary>
public class SetBreakpointsParams
{
    [JsonPropertyName("stepIds")] public List<string> StepIds { get; set; } = new();
}

/// <summary>AnswerPrompt { promptId, button, value? } — the first answer to a prompt wins.</summary>
public class AnswerPromptParams
{
    [JsonPropertyName("promptId")] public string  PromptId { get; set; } = "";
    /// <summary>Index of the pressed button (0-based).</summary>
    [JsonPropertyName("button")]   public int     Button   { get; set; }
    /// <summary>The typed input (number or text prompt), as text; null for a buttons-only prompt.</summary>
    [JsonPropertyName("value")]    public string? Value    { get; set; }
}

/// <summary>FormatProgramText { program, includeIds? } (docs/program-language.md §6.9).</summary>
public class FormatProgramTextParams
{
    [JsonPropertyName("program")]    public BuiltProgram? Program    { get; set; }
    [JsonPropertyName("includeIds")] public bool          IncludeIds { get; set; }
    /// <summary>Also print the <c>@program {…}</c> header line (name, kind, display page) — for
    /// exporting a .srs file; parsing without a base reads it back.</summary>
    [JsonPropertyName("includeHeader")] public bool       IncludeHeader { get; set; }
}

/// <summary>ParseProgramText { text, base?, validate? } (docs/program-language.md §6.9).</summary>
public class ParseProgramTextParams
{
    [JsonPropertyName("text")]     public string        Text     { get; set; } = "";
    [JsonPropertyName("base")]     public BuiltProgram? Base     { get; set; }
    [JsonPropertyName("validate")] public bool          Validate { get; set; } = true;
}

public class ProgramImageParams
{
    [JsonPropertyName("name")]     public string Name     { get; set; } = ""; // the program
    [JsonPropertyName("variable")] public string Variable { get; set; } = ""; // the image variable on it
}

public class SaveBuiltProgramImageParams
{
    [JsonPropertyName("name")]  public string Name  { get; set; } = "";
    [JsonPropertyName("image")] public string Image { get; set; } = ""; // base64-encoded JPEG bytes
}

/// <summary>Summary row for <c>GetBuiltProgramRevisions</c> — one saved snapshot of a program.</summary>
public class ProgramRevisionInfo
{
    [JsonPropertyName("id")]            public string  Id            { get; set; } = ""; // unix-ms file stem
    [JsonPropertyName("savedUnixMs")]   public long    SavedUnixMs   { get; set; }
    [JsonPropertyName("stepCount")]     public int     StepCount     { get; set; }
    [JsonPropertyName("variableCount")] public int     VariableCount { get; set; }
    [JsonPropertyName("note")]          public string? Note          { get; set; }
}

/// <summary>Params for <c>GetBuiltProgramRevision</c> and <c>RestoreBuiltProgramRevision</c>.</summary>
public class BuiltProgramRevisionParams
{
    [JsonPropertyName("name")] public string Name { get; set; } = "";
    [JsonPropertyName("id")]   public string Id   { get; set; } = "";
}


