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
}

public class BuiltProgramNameParams
{
    [JsonPropertyName("name")] public string Name { get; set; } = "";
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


