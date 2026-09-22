using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── Program cycle ─────────────────────────────────────────────────────────────

[JsonConverter(typeof(JsonStringEnumConverter))]
public enum ProgramStatus
{
    Ready,
    Starting,
    Running,
    Finishing,
    Stopping,
    Stopped,
    Complete,
    Error
}

/// <summary>Full program state held inside the controller.</summary>
public class ProgramModel
{
    public string       Name                   { get; set; } = "";
    public string       Description            { get; set; } = "";
    public byte[]?      Image                  { get; set; }
    public List<string> StepLogs               { get; set; } = new();
    // Absolute index of StepLogs[0]. Increments each time an old entry is dropped
    // so clients can page by a stable absolute index even after the ring buffer
    // wraps past its cap.
    [JsonIgnore] public int LogBaseIndex        { get; set; } = 0;
    public ProgramStatus Status                { get; set; } = ProgramStatus.Ready;
    public string       CurrentStepDescription { get; set; } = "";
    public int          CurrentStepNumber      { get; set; } = 0;
    public int          MaxStepCount           { get; set; } = 0;
    public string       ErrorDescription       { get; set; } = "";
    public string       WarningDescription     { get; set; } = "";
    public string  CurrentPointName  { get; set; } = "";
    public double? CurrentOffsetX   { get; set; }
    public double? CurrentOffsetY   { get; set; }
    public double? CurrentOffsetZ   { get; set; }
    public double? CurrentOffsetRX  { get; set; }
    public double? CurrentOffsetRY  { get; set; }
    public double? CurrentOffsetRZ  { get; set; }
    public double? CurrentToolOffsetX  { get; set; }
    public double? CurrentToolOffsetY  { get; set; }
    public double? CurrentToolOffsetZ  { get; set; }
    public double? CurrentToolOffsetRX { get; set; }
    public double? CurrentToolOffsetRY { get; set; }
    public double? CurrentToolOffsetRZ { get; set; }

    // Action flags — set by the mobile app; consumed by the external program
    public bool Start { get; set; } = false;
    public bool Stop  { get; set; } = false;
    public bool Reset { get; set; } = false;
    public bool Abort { get; set; } = false;
}

/// <summary>Sparse status update — all fields optional except ProgramName.</summary>
public class ProgramCycleUpdate
{
    [JsonPropertyName("programName")]
    public string ProgramName { get; set; } = "";

    [JsonPropertyName("programStatus")]
    public ProgramStatus? ProgramStatus { get; set; }

    [JsonPropertyName("currentStepNumber")]
    public int? CurrentStepNumber { get; set; }

    [JsonPropertyName("maxStepCount")]
    public int? MaxStepCount { get; set; }

    [JsonPropertyName("stepDescription")]
    public string StepDescription { get; set; } = "";

    [JsonPropertyName("errorDescription")]
    public string? ErrorDescription { get; set; }

    [JsonPropertyName("warningDescription")]
    public string? WarningDescription { get; set; }

    [JsonPropertyName("currentPointName")]     public string? CurrentPointName    { get; set; }
    [JsonPropertyName("currentOffsetX")]       public double? CurrentOffsetX      { get; set; }
    [JsonPropertyName("currentOffsetY")]       public double? CurrentOffsetY      { get; set; }
    [JsonPropertyName("currentOffsetZ")]       public double? CurrentOffsetZ      { get; set; }
    [JsonPropertyName("currentOffsetRX")]      public double? CurrentOffsetRX     { get; set; }
    [JsonPropertyName("currentOffsetRY")]      public double? CurrentOffsetRY     { get; set; }
    [JsonPropertyName("currentOffsetRZ")]      public double? CurrentOffsetRZ     { get; set; }
    [JsonPropertyName("currentToolOffsetX")]   public double? CurrentToolOffsetX  { get; set; }
    [JsonPropertyName("currentToolOffsetY")]   public double? CurrentToolOffsetY  { get; set; }
    [JsonPropertyName("currentToolOffsetZ")]   public double? CurrentToolOffsetZ  { get; set; }
    [JsonPropertyName("currentToolOffsetRX")]  public double? CurrentToolOffsetRX { get; set; }
    [JsonPropertyName("currentToolOffsetRY")]  public double? CurrentToolOffsetRY { get; set; }
    [JsonPropertyName("currentToolOffsetRZ")]  public double? CurrentToolOffsetRZ { get; set; }

    /// <summary>
    /// When true, StepDescription is also appended to the program's persistent log.
    /// Set only on step completion — not on the "started" notification — to prevent double entries.
    /// </summary>
    public bool ShouldLog { get; set; } = false;
}

/// <summary>Params for SetAvailablePrograms — sends a list of program definitions.</summary>
public class SetAvailableProgramsParams
{
    [JsonPropertyName("programs")]
    public List<ProgramUpdateParams> Programs { get; set; } = new();
}

/// <summary>One program entry inside SetAvailablePrograms. Null fields are left unchanged on update.</summary>
public class ProgramUpdateParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";

    [JsonPropertyName("description")]
    public string? Description { get; set; }

    /// <summary>Base-64 encoded image bytes. Null = leave existing image unchanged.</summary>
    [JsonPropertyName("image")]
    public string? Image { get; set; }
}

/// <summary>Params for GetProgramLogs — supports a half-open [start, end) range for lazy loading.</summary>
public class GetProgramLogsParams
{
    [JsonPropertyName("programName")]
    public string ProgramName { get; set; } = "";

    /// <summary>First log index to return (inclusive, default 0).</summary>
    [JsonPropertyName("start")]
    public int? Start { get; set; }

    /// <summary>Last log index to return (exclusive, default = total count).</summary>
    [JsonPropertyName("end")]
    public int? End { get; set; }
}

/// <summary>Params for program action commands (StartProgram, StopProgram, ResetProgram, AbortProgram).</summary>
public class ProgramActionParams
{
    [JsonPropertyName("programName")]
    public string ProgramName { get; set; } = "";
}


