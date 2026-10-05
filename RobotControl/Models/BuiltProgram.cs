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


