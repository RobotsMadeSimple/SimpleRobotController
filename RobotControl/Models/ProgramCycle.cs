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
    /// <summary>Number of times this program has been started since the controller booted.</summary>
    public int  RunCount          { get; set; }
    /// <summary>Unix ms of the most recent start. Lets a client notice a run that began and finished between two polls.</summary>
    public long LastStartedUnixMs { get; set; }
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

    /// <summary>Id of the step most recently started (built programs); null when idle or unknown.
    /// After an Error it is the step that failed.</summary>
    public string? CurrentStepId { get; set; }
    /// <summary>The error code of a run that ended in Error (<c>Programs.ErrorCodes</c>, or a Raise
    /// Error step's own code); null otherwise or when unknown. Cleared when a new run starts.</summary>
    public string? ErrorCode { get; set; }
    /// <summary>
    /// Why a Stopped program is paused: "breakpoint" | "step" (step mode) | "pauseStep" (a
    /// PauseProgram step) | "user" (Stop). Null while running or after it finishes.
    /// See <see cref="PauseReasons"/>.
    /// </summary>
    public string? PauseReason    { get; set; }
    /// <summary>The step the program will run next on Continue (breakpoint / step mode); null otherwise.</summary>
    public string? PausedAtStepId { get; set; }
    /// <summary>The operator prompt this program is waiting on (the head of its queue); null when none.</summary>
    public ProgramPromptInfo? Prompt { get; set; }

    // Action flags — set by the mobile app; consumed by the external program
    public bool Start { get; set; } = false;
    public bool Stop  { get; set; } = false;
    public bool Reset { get; set; } = false;
    public bool Abort { get; set; } = false;
}

/// <summary>The <see cref="ProgramModel.PauseReason"/> values.</summary>
public static class PauseReasons
{
    public const string Breakpoint = "breakpoint";
    public const string Step       = "step";
    public const string PauseStep  = "pauseStep";
    public const string User       = "user";
}

/// <summary>
/// An operator prompt published in the program status (docs/program-language.md §4.3, Prompt).
/// Answered with <c>AnswerPrompt { promptId, button, value? }</c>; the first answer wins.
/// </summary>
public class ProgramPromptInfo
{
    /// <summary>Unique per prompt instance (a fresh id each time the step runs or is re-shown).</summary>
    [JsonPropertyName("id")]          public string        Id          { get; set; } = "";
    [JsonPropertyName("stepId")]      public string        StepId      { get; set; } = "";
    [JsonPropertyName("programName")] public string        ProgramName { get; set; } = "";
    /// <summary>Interpolated title; null for none.</summary>
    [JsonPropertyName("title")]       public string?       Title       { get; set; }
    /// <summary>Interpolated message.</summary>
    [JsonPropertyName("message")]     public string        Message     { get; set; } = "";
    /// <summary>1–4 button labels, in order; the answer's <c>button</c> is an index into this.</summary>
    [JsonPropertyName("buttons")]     public List<string>  Buttons     { get; set; } = new() { "OK" };
    /// <summary>"none" | "number" | "text".</summary>
    [JsonPropertyName("input")]       public string        Input       { get; set; } = "none";
    /// <summary>The evaluated default input (number or text), as text; null for none.</summary>
    [JsonPropertyName("default")]     public string?       Default     { get; set; }
    [JsonPropertyName("min")]         public double?       Min         { get; set; }
    [JsonPropertyName("max")]         public double?       Max         { get; set; }
    /// <summary>0 = no timeout.</summary>
    [JsonPropertyName("timeoutMs")]   public int           TimeoutMs   { get; set; }
    /// <summary>Unix ms when the prompt was shown, so a client can draw the countdown.</summary>
    [JsonPropertyName("shownUnixMs")] public long          ShownUnixMs { get; set; }
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

    /// <summary>The run's error code (with an Error status); null leaves it unchanged.</summary>
    [JsonPropertyName("errorCode")]
    public string? ErrorCode { get; set; }

    [JsonPropertyName("warningDescription")]
    public string? WarningDescription { get; set; }

    /// <summary>The step this update is about (step started / completed); null leaves it unchanged.</summary>
    [JsonPropertyName("currentStepId")]
    public string? CurrentStepId { get; set; }

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

    /// <summary>
    /// Stable id of a built program. When set, the target is resolved by id first (rename-proof)
    /// and its current name used for the name-keyed runtime. Optional — external (non-built)
    /// programs have no id and are addressed by <see cref="ProgramName"/> only.
    /// </summary>
    [JsonPropertyName("programId")]
    public string? ProgramId { get; set; }

    /// <summary>
    /// Optional launch inputs for StartProgram: a map of variable name → value that seeds the
    /// program's declared input variables (<see cref="ProgramVariable.IsInput"/>) before the run,
    /// overriding their initial values. Non-input, computed, list and image variables are ignored.
    /// Numbers seed scalars, true/false seed booleans, strings seed string variables.
    /// </summary>
    [JsonPropertyName("variables")]
    public Dictionary<string, JsonElement>? Variables { get; set; }
}


