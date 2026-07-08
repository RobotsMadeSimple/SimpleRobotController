using System;
using System.Collections.Generic;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;

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

// ── Program builder ───────────────────────────────────────────────────────────

[JsonConverter(typeof(JsonStringEnumConverter))]
public enum StepType { MoveL, MoveJ, JumpL, JumpJ, SetOutput, Wait, Loop, StatusUpdate, CallRoutine, SetSpeedL, SetSpeedJ, SetVariable, PauseProgram, Label, GoToLabel, IfCondition, SetTool, RunHoming, AuxMove, AuxContinuous, AuxStop, AuxEnable, RunVision, SetLocal, ClearLocal, StartBackground, StopBackground, WaitForBackground, StopwatchControl, SaveImage, ThreadMove, CncProgram }

/// <summary>6-DOF value stored in a Points-type program variable or written by RunVision.</summary>
public class Vector6Val
{
    [JsonPropertyName("x")]  public double X  { get; set; }
    [JsonPropertyName("y")]  public double Y  { get; set; }
    [JsonPropertyName("z")]  public double Z  { get; set; }
    [JsonPropertyName("rx")] public double RX { get; set; }
    [JsonPropertyName("ry")] public double RY { get; set; }
    [JsonPropertyName("rz")] public double RZ { get; set; }

    public double GetComponent(string name) => name.ToLower() switch {
        "x" or "0" => X, "y" or "1" => Y, "z" or "2" => Z,
        "rx" or "3" => RX, "ry" or "4" => RY, "rz" or "5" => RZ,
        _ => 0
    };
    public double GetComponent(int idx) => idx switch {
        0 => X, 1 => Y, 2 => Z, 3 => RX, 4 => RY, 5 => RZ, _ => 0
    };
}

/// <summary>Maps one BlobInspection's outputs to program variable names.</summary>
public class VisionStepOutput
{
    [JsonPropertyName("inspectionId")] public string  InspectionId { get; set; } = "";
    [JsonPropertyName("countVar")]     public string? CountVar     { get; set; }
    [JsonPropertyName("pointsVar")]    public string? PointsVar    { get; set; }
    [JsonPropertyName("detectedVar")]  public string? DetectedVar  { get; set; }
}

/// <summary>Maps one ColorCoverageInspection's outputs to program variable names.</summary>
public class ColorVisionStepOutput
{
    [JsonPropertyName("inspectionId")] public string  InspectionId { get; set; } = "";
    [JsonPropertyName("coverageVar")]  public string? CoverageVar  { get; set; }
    [JsonPropertyName("passedVar")]    public string? PassedVar    { get; set; }
}

/// <summary>Maps one PolygonInspection's outputs to program variable names.</summary>
public class PolygonVisionStepOutput
{
    [JsonPropertyName("inspectionId")] public string  InspectionId { get; set; } = "";
    [JsonPropertyName("countVar")]     public string? CountVar     { get; set; }
    [JsonPropertyName("foundVar")]     public string? FoundVar     { get; set; }
    [JsonPropertyName("angleVar")]     public string? AngleVar     { get; set; }
    [JsonPropertyName("centerXVar")]   public string? CenterXVar   { get; set; }
    [JsonPropertyName("centerYVar")]   public string? CenterYVar   { get; set; }
}

/// <summary>Maps one ArucoInspection's outputs to program variable names.</summary>
public class ArucoVisionStepOutput
{
    [JsonPropertyName("inspectionId")]    public string  InspectionId    { get; set; } = "";
    [JsonPropertyName("countVar")]        public string? CountVar        { get; set; }
    [JsonPropertyName("foundVar")]        public string? FoundVar        { get; set; }
    [JsonPropertyName("firstIdVar")]      public string? FirstIdVar      { get; set; }
    [JsonPropertyName("firstCenterXVar")] public string? FirstCenterXVar { get; set; }
    [JsonPropertyName("firstCenterYVar")] public string? FirstCenterYVar { get; set; }
}

public class ConditionItem
{
    [JsonPropertyName("id")]       public string Id       { get; set; } = "";
    [JsonPropertyName("left")]     public string Left     { get; set; } = "";
    [JsonPropertyName("operator")] public string Operator { get; set; } = "==";
    [JsonPropertyName("right")]    public string Right    { get; set; } = "";
}

public class ConditionGroup
{
    [JsonPropertyName("combinator")] public string Combinator { get; set; } = "ALL";
    [JsonPropertyName("items")]      public List<ConditionItem> Items { get; set; } = new();
}

public class ElseIfBranch
{
    [JsonPropertyName("id")]        public string         Id        { get; set; } = "";
    [JsonPropertyName("condition")] public ConditionGroup Condition { get; set; } = new();
    [JsonPropertyName("steps")]     public List<ProgramStep> Steps  { get; set; } = new();
}

public class ProgramStep
{
    [JsonPropertyName("id")]
    public string Id { get; set; } = "";

    [JsonPropertyName("name")]
    public string? Name { get; set; }

    [JsonPropertyName("type")]
    public StepType Type { get; set; }

    // MoveL / MoveJ / SetSpeedL / SetSpeedJ
    [JsonPropertyName("pointName")]
    public string? PointName { get; set; }
    [JsonPropertyName("speed")]
    public double? Speed { get; set; }
    [JsonPropertyName("accel")]
    public double? Accel { get; set; }
    [JsonPropertyName("decel")]
    public double? Decel { get; set; }

    // Optional position offset added to the target point (mm / deg)
    [JsonPropertyName("offsetX")]  public double? OffsetX  { get; set; }
    [JsonPropertyName("offsetY")]  public double? OffsetY  { get; set; }
    [JsonPropertyName("offsetZ")]  public double? OffsetZ  { get; set; }
    [JsonPropertyName("offsetRX")] public double? OffsetRX { get; set; }
    [JsonPropertyName("offsetRY")] public double? OffsetRY { get; set; }
    [JsonPropertyName("offsetRZ")] public double? OffsetRZ { get; set; }

    // Local tool offset applied at execution time (mm / deg)
    [JsonPropertyName("toolOffsetX")]  public double? ToolOffsetX  { get; set; }
    [JsonPropertyName("toolOffsetY")]  public double? ToolOffsetY  { get; set; }
    [JsonPropertyName("toolOffsetZ")]  public double? ToolOffsetZ  { get; set; }
    [JsonPropertyName("toolOffsetRX")] public double? ToolOffsetRX { get; set; }
    [JsonPropertyName("toolOffsetRY")] public double? ToolOffsetRY { get; set; }
    [JsonPropertyName("toolOffsetRZ")] public double? ToolOffsetRZ { get; set; }

    // Per-axis absolute overrides — when set, replace the calculated axis value (base + offset) (mm / deg)
    [JsonPropertyName("overrideX")]  public double? OverrideX  { get; set; }
    [JsonPropertyName("overrideY")]  public double? OverrideY  { get; set; }
    [JsonPropertyName("overrideZ")]  public double? OverrideZ  { get; set; }
    [JsonPropertyName("overrideRX")] public double? OverrideRX { get; set; }
    [JsonPropertyName("overrideRY")] public double? OverrideRY { get; set; }
    [JsonPropertyName("overrideRZ")] public double? OverrideRZ { get; set; }

    // SetOutput
    [JsonPropertyName("outputNumber")]
    public int? OutputNumber { get; set; }
    [JsonPropertyName("outputValue")]
    public bool? OutputValue { get; set; }
    [JsonPropertyName("outputCard")]
    public string? OutputCard { get; set; }
    [JsonPropertyName("outputNanoId")]
    public string? OutputNanoId { get; set; }
    // Non-blocking pulse: set to OutputValue for PulseMs, then set to opposite. 0/null = hold.
    [JsonPropertyName("pulseMs")]
    public int? PulseMs { get; set; }
    // When true and PulseMs > 0, block program execution until the pulse completes before advancing.
    [JsonPropertyName("pulseBlocking")]
    public bool? PulseBlocking { get; set; }

    // Wait
    [JsonPropertyName("waitMs")]
    public int? WaitMs { get; set; }
    // Wait condition mode: "duration" (default) | "condition"
    [JsonPropertyName("waitMode")]
    public string? WaitMode { get; set; }
    [JsonPropertyName("waitCondition")]
    public ConditionGroup? WaitCondition { get; set; }
    [JsonPropertyName("waitTimeoutMs")]
    public int? WaitTimeoutMs { get; set; }
    [JsonPropertyName("waitTimeoutVariableName")]
    public string? WaitTimeoutVariableName { get; set; }

    // Loop
    [JsonPropertyName("loopCount")]
    public int? LoopCount { get; set; }          // 0 = infinite
    [JsonPropertyName("loopSteps")]
    public List<ProgramStep>? LoopSteps { get; set; }
    // Loop mode: "count" (default) | "forEach" | "while"
    [JsonPropertyName("loopMode")]
    public string? LoopMode { get; set; }
    [JsonPropertyName("forEachVariableName")]
    public string? ForEachVariableName { get; set; }
    [JsonPropertyName("forEachValueVariableName")]
    public string? ForEachValueVariableName { get; set; }
    [JsonPropertyName("forEachIndexVariableName")]
    public string? ForEachIndexVariableName { get; set; }
    [JsonPropertyName("loopWhileCondition")]
    public ConditionGroup? LoopWhileCondition { get; set; }

    // StatusUpdate
    [JsonPropertyName("statusMessage")]
    public string? StatusMessage { get; set; }
    [JsonPropertyName("statusWarning")]
    public string? StatusWarning { get; set; }
    [JsonPropertyName("statusError")]
    public string? StatusError { get; set; }

    // CallRoutine
    [JsonPropertyName("routineName")]
    public string? RoutineName { get; set; }

    // SetVariable
    [JsonPropertyName("variableName")]
    public string? VariableName { get; set; }
    [JsonPropertyName("variableExpr")]
    public string? VariableExpr { get; set; }

    // Variable expressions — overrides any literal numeric field with a math expression.
    // Keys match JSON property names (camelCase). Evaluated at execution time.
    [JsonPropertyName("expressions")]
    public Dictionary<string, string>? Expressions { get; set; }

    // Grid point reference — when set, overrides pointName with a calculated grid position
    [JsonPropertyName("gridPoint")]
    public GridPointRef? GridPoint { get; set; }

    // Stack point reference — when set, overrides pointName with a 1-D indexed position
    [JsonPropertyName("stackPoint")]
    public StackPointRef? StackPoint { get; set; }

    // Label / GoToLabel
    [JsonPropertyName("labelId")]
    public string? LabelId { get; set; }
    [JsonPropertyName("labelName")]
    public string? LabelName { get; set; }

    // IfCondition
    [JsonPropertyName("condition")]       public ConditionGroup?     Condition      { get; set; }
    [JsonPropertyName("ifSteps")]         public List<ProgramStep>?  IfSteps        { get; set; }
    [JsonPropertyName("elseIfBranches")]  public List<ElseIfBranch>? ElseIfBranches { get; set; }
    [JsonPropertyName("elseSteps")]       public List<ProgramStep>?  ElseSteps      { get; set; }

    // SetTool
    [JsonPropertyName("toolName")]        public string? ToolName { get; set; }

    // SetLocal / ClearLocal — also used as per-step local override on move steps
    [JsonPropertyName("localName")]       public string? LocalName { get; set; }

    // JumpL / JumpJ — Z height used for lift and lower legs (mm). JumpZStart/JumpZEnd override each leg independently.
    [JsonPropertyName("jumpZ")]      public double? JumpZ      { get; set; }
    [JsonPropertyName("jumpZStart")] public double? JumpZStart { get; set; }
    [JsonPropertyName("jumpZEnd")]   public double? JumpZEnd   { get; set; }

    // RunVision
    [JsonPropertyName("visionProgramId")]   public string? VisionProgramId   { get; set; }
    [JsonPropertyName("visionProgramName")] public string? VisionProgramName { get; set; }
    [JsonPropertyName("visionZoneId")]      public string? VisionZoneId      { get; set; }
    [JsonPropertyName("visionZoneVar")]     public string? VisionZoneVar     { get; set; }
    [JsonPropertyName("visionOutputs")]     public List<VisionStepOutput>?         VisionOutputs  { get; set; }
    [JsonPropertyName("colorOutputs")]      public List<ColorVisionStepOutput>?    ColorOutputs   { get; set; }
    [JsonPropertyName("polygonOutputs")]    public List<PolygonVisionStepOutput>?  PolygonOutputs { get; set; }
    [JsonPropertyName("arucoOutputs")]      public List<ArucoVisionStepOutput>?    ArucoOutputs   { get; set; }

    // Variable point target for move steps — overrides pointName when set
    [JsonPropertyName("varPointName")]  public string? VarPointName  { get; set; }
    [JsonPropertyName("varPointIndex")] public string? VarPointIndex { get; set; }

    // StartBackground / StopBackground / WaitForBackground
    [JsonPropertyName("backgroundProgramName")]
    public string? BackgroundProgramName { get; set; }
    [JsonPropertyName("backgroundProgramId")]
    public string? BackgroundProgramId { get; set; }

    // StopwatchControl — action: "Start" | "Stop" | "Reset"
    [JsonPropertyName("stopwatchAction")]
    public string? StopwatchAction { get; set; }
    [JsonPropertyName("stopwatchVariableName")]
    public string? StopwatchVariableName { get; set; }

    // SaveImage — path supports $variable interpolation (including built-in $time_ms)
    [JsonPropertyName("saveImagePath")]
    public string? SaveImagePath { get; set; }
    [JsonPropertyName("saveImageCameraId")]
    public string? SaveImageCameraId { get; set; }

    // AuxMove / AuxContinuous / AuxStop
    // auxSteps:    steps to move; sign determines direction (positive=CW, negative=CCW)
    // auxVelocity: peak velocity in steps/sec  (AuxMove + AuxContinuous)
    // auxAccel:    acceleration in steps/sec^2 (AuxMove + AuxContinuous)
    // auxDecel:    deceleration in steps/sec^2 (AuxMove only; AuxStop uses this for ramp-down)
    // auxWaitForDone: when true (default), program blocks until AuxMove finishes
    [JsonPropertyName("auxDeviceId")]    public string? AuxDeviceId    { get; set; }
    [JsonPropertyName("auxAxisIndex")]   public int?    AuxAxisIndex   { get; set; }
    [JsonPropertyName("auxSteps")]       public long?   AuxSteps       { get; set; }
    // Physical-unit move: distance in mm (Linear) or degrees (Rotary). Null = use auxSteps.
    [JsonPropertyName("auxDistance")]    public double? AuxDistance    { get; set; }
    // "mm" | "deg" — indicates auxDistance is in physical units; velocity/accel/decel also in those units.
    [JsonPropertyName("auxUnit")]        public string? AuxUnit        { get; set; }
    [JsonPropertyName("auxVelocity")]    public double? AuxVelocity    { get; set; }
    [JsonPropertyName("auxAccel")]       public double? AuxAccel       { get; set; }
    [JsonPropertyName("auxDecel")]       public double? AuxDecel       { get; set; }
    [JsonPropertyName("auxWaitForDone")] public bool?   AuxWaitForDone { get; set; }
    [JsonPropertyName("auxImmediate")]   public bool?   AuxImmediate   { get; set; }
    [JsonPropertyName("auxAbsolute")]    public bool?   AuxAbsolute    { get; set; }
    // AuxEnable — enable or disable motor drivers
    [JsonPropertyName("auxEnable")]      public bool?   AuxEnable      { get; set; }

    // ThreadMove
    [JsonPropertyName("threadDistance")]   public double? ThreadDistance   { get; set; }
    [JsonPropertyName("threadPitch")]      public double? ThreadPitch      { get; set; }
    [JsonPropertyName("threadPeck")]       public bool?   ThreadPeck       { get; set; }
    [JsonPropertyName("threadPeckDepth")]  public double? ThreadPeckDepth  { get; set; }
    [JsonPropertyName("threadReverseOut")] public bool?   ThreadReverseOut { get; set; }

    // CncProgram — nested steps generated by the CNC builder from a DXF hole selection
    [JsonPropertyName("cncDxfFile")]    public string?           CncDxfFile    { get; set; }
    [JsonPropertyName("cncSafeZ")]      public double?           CncSafeZ      { get; set; }
    [JsonPropertyName("cncProgramSteps")] public List<ProgramStep>? CncProgramSteps { get; set; }
}

public class ProgramVariable
{
    [JsonPropertyName("id")]
    public string Id { get; set; } = "";
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";
    [JsonPropertyName("value")]
    public double Value { get; set; }
    [JsonPropertyName("values")]
    public List<double>? Values { get; set; }
    /// <summary>When non-null, this is a Vector6 array variable (e.g. blob detection results). Populated at runtime by RunVision.</summary>
    [JsonPropertyName("points")]
    public List<Vector6Val>? Points { get; set; }
    [JsonPropertyName("description")]
    public string? Description { get; set; }
    [JsonPropertyName("isBoolean")]
    public bool? IsBoolean { get; set; }
    /// <summary>When true, this scalar variable is shared across all concurrently running programs via the global variable store.</summary>
    [JsonPropertyName("isGlobal")]
    public bool? IsGlobal { get; set; }
    /// <summary>When true, the current runtime value of this variable is shown on the monitor page while the program runs.</summary>
    [JsonPropertyName("displayOnMonitor")]
    public bool? DisplayOnMonitor { get; set; }
    /// <summary>When true, this variable is a stopwatch; its value holds elapsed milliseconds at runtime.</summary>
    [JsonPropertyName("isStopwatch")]
    public bool? IsStopwatch { get; set; }
    /// <summary>When true, the runtime value is saved to disk when the program finishes and restored on the next run.</summary>
    [JsonPropertyName("isPersistent")]
    public bool? IsPersistent { get; set; }
    /// <summary>When true, this variable holds a string value stored in StringValue.</summary>
    [JsonPropertyName("isString")]
    public bool? IsString { get; set; }
    /// <summary>String variable initial/default value — only meaningful when IsString is true.</summary>
    [JsonPropertyName("stringValue")]
    public string? StringValue { get; set; }
}

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

public class SaveBuiltProgramImageParams
{
    [JsonPropertyName("name")]  public string Name  { get; set; } = "";
    [JsonPropertyName("image")] public string Image { get; set; } = ""; // base64-encoded JPEG bytes
}

public class SetRobotIdentityParams
{
    [JsonPropertyName("robotName")] public string? RobotName { get; set; }
    [JsonPropertyName("robotType")] public string? RobotType { get; set; }
}

public class SetRobotConfigParams
{
    [JsonPropertyName("robotType")]                 public string? RobotType                 { get; set; }
    [JsonPropertyName("homingSpeed")]               public double? HomingSpeed               { get; set; }
    [JsonPropertyName("homingSlowSpeed")]           public double? HomingSlowSpeed           { get; set; }
    [JsonPropertyName("homingBackoffMm")]           public double? HomingBackoffMm           { get; set; }
    [JsonPropertyName("j1HomeOffsetDeg")]           public double? J1HomeOffsetDeg           { get; set; }
    [JsonPropertyName("verticalHomePosition")]      public double? VerticalHomePosition      { get; set; }
    [JsonPropertyName("horizontalHomePosition")]    public double? HorizontalHomePosition    { get; set; }
    [JsonPropertyName("verticalHomingDirection")]   public int?    VerticalHomingDirection   { get; set; }
    [JsonPropertyName("horizontalHomingDirection")] public int?    HorizontalHomingDirection { get; set; }
    [JsonPropertyName("j1HomingDirection")]         public int?    J1HomingDirection         { get; set; }
    [JsonPropertyName("j4HomeOffsetDeg")]           public double? J4HomeOffsetDeg           { get; set; }
    [JsonPropertyName("m1Direction")]               public int?    M1Direction               { get; set; }
    [JsonPropertyName("m2Direction")]               public int?    M2Direction               { get; set; }
    [JsonPropertyName("m3Direction")]               public int?    M3Direction               { get; set; }
    [JsonPropertyName("m4Direction")]               public int?    M4Direction               { get; set; }
    [JsonPropertyName("enableNanoCards")]           public bool?   EnableNanoCards           { get; set; }
    [JsonPropertyName("enableRelayCard")]           public bool?   EnableRelayCard           { get; set; }
    [JsonPropertyName("enableAuxAxis")]             public bool?   EnableAuxAxis             { get; set; }
    [JsonPropertyName("enableCameras")]             public bool?   EnableCameras             { get; set; }
    [JsonPropertyName("jogSlowSpeed")]              public double? JogSlowSpeed              { get; set; }
    [JsonPropertyName("jogNormalSpeed")]            public double? JogNormalSpeed            { get; set; }
    [JsonPropertyName("jogFastSpeed")]              public double? JogFastSpeed              { get; set; }
    [JsonPropertyName("cncStepsPerRevX")]           public int?    CncStepsPerRevX           { get; set; }
    [JsonPropertyName("cncStepsPerRevY")]           public int?    CncStepsPerRevY           { get; set; }
    [JsonPropertyName("cncStepsPerRevZ")]           public int?    CncStepsPerRevZ           { get; set; }
    [JsonPropertyName("cncStepsPerRevRZ")]          public int?    CncStepsPerRevRZ          { get; set; }
    [JsonPropertyName("cncMmPerRevX")]              public double? CncMmPerRevX              { get; set; }
    [JsonPropertyName("cncMmPerRevY")]              public double? CncMmPerRevY              { get; set; }
    [JsonPropertyName("cncMmPerRevZ")]              public double? CncMmPerRevZ              { get; set; }
    [JsonPropertyName("cncDegPerRevRZ")]            public double? CncDegPerRevRZ            { get; set; }
    [JsonPropertyName("cncXHomePosition")]          public double? CncXHomePosition          { get; set; }
    [JsonPropertyName("cncYHomePosition")]          public double? CncYHomePosition          { get; set; }
    [JsonPropertyName("cncZHomePosition")]          public double? CncZHomePosition          { get; set; }
    [JsonPropertyName("cncRzHomePosition")]         public double? CncRzHomePosition         { get; set; }
    [JsonPropertyName("cncXHomingDirection")]       public int?    CncXHomingDirection       { get; set; }
    [JsonPropertyName("cncYHomingDirection")]       public int?    CncYHomingDirection       { get; set; }
    [JsonPropertyName("cncZHomingDirection")]       public int?    CncZHomingDirection       { get; set; }
}

public class CommandMessage
{
    public string Type { get; set; } = default!;
    public string Id { get; set; } = default!;
    public string Command { get; set; } = default!;
    public JsonElement? Params { get; set; }
}

// ── Grid ──────────────────────────────────────────────────────────────────────

/// <summary>
/// A 2D grid of positions defined by a base point, row/column offsets, and an
/// optional rotation about the base point's Z-axis.
/// </summary>
public class Grid
{
    [JsonPropertyName("id")]             public string Id             { get; set; } = "";
    [JsonPropertyName("name")]           public string Name           { get; set; } = "";
    [JsonPropertyName("basePointName")]  public string BasePointName  { get; set; } = "";

    [JsonPropertyName("rowOffsetX")]     public double RowOffsetX     { get; set; }
    [JsonPropertyName("rowOffsetY")]     public double RowOffsetY     { get; set; }
    [JsonPropertyName("rowOffsetZ")]     public double RowOffsetZ     { get; set; }

    [JsonPropertyName("colOffsetX")]     public double ColOffsetX     { get; set; }
    [JsonPropertyName("colOffsetY")]     public double ColOffsetY     { get; set; }
    [JsonPropertyName("colOffsetZ")]     public double ColOffsetZ     { get; set; }

    [JsonPropertyName("rowCount")]       public int?   RowCount       { get; set; }
    [JsonPropertyName("colCount")]       public int?   ColCount       { get; set; }

    /// <summary>Degrees — rotates the row/column offsets around the base-point Z-axis.</summary>
    [JsonPropertyName("rotation")]       public double Rotation       { get; set; }

    [JsonPropertyName("lastUpdatedUnixMs")] public long LastUpdatedUnixMs { get; set; }
}

/// <summary>Identifies a cell in a named grid — used as the target position in a MoveL/MoveJ step.</summary>
public class GridPointRef
{
    [JsonPropertyName("gridId")]       public string  GridId       { get; set; } = "";
    [JsonPropertyName("rowIndex")]     public double? RowIndex     { get; set; }
    [JsonPropertyName("colIndex")]     public double? ColIndex     { get; set; }
    [JsonPropertyName("gridIndex")]    public double? GridIndex    { get; set; }
    [JsonPropertyName("useGridIndex")] public bool    UseGridIndex { get; set; }
}

public class Vector6
{
    public Vector6(double x=0, double y=0, double z=0, double rx=0, double ry=0, double rz=0) { 
        this.X = x; this.Y = y; this.Z = z;
        this.RX = rx; this.RY = ry; this.RZ = rz;
    }

    public double X { get; set; }
    public double Y { get; set; }
    public double Z { get; set; }
    public double RX { get; set; }
    public double RY { get; set; }
    public double RZ { get; set; }

    public static Vector6 operator +(Vector6 a, Vector6 b) =>
        new()
        {
            X = a.X + b.X,
            Y = a.Y + b.Y,
            Z = a.Z + b.Z,
            RX = a.RX + b.RX,
            RY = a.RY + b.RY,
            RZ = a.RZ + b.RZ
        };

    public static Vector6 operator -(Vector6 a, Vector6 b) =>
        new()
        {
            X = a.X - b.X,
            Y = a.Y - b.Y,
            Z = a.Z - b.Z,
            RX = a.RX - b.RX,
            RY = a.RY - b.RY,
            RZ = a.RZ - b.RZ
        };

    public static Vector6 operator *(Vector6 a, double s) =>
        new()
        {
            X = a.X * s,
            Y = a.Y * s,
            Z = a.Z * s,
            RX = a.RX * s,
            RY = a.RY * s,
            RZ = a.RZ * s
        };

    public double Distance3(Vector6 Other)
    {
        Vector6 DeltaPosition = this - Other;
        return Math.Sqrt(
            DeltaPosition.X * DeltaPosition.X +
            DeltaPosition.Y * DeltaPosition.Y +
            DeltaPosition.Z * DeltaPosition.Z
        );
    }
    public void Copy(Vector6 Other)
    {
        X = Other.X; Y = Other.Y; Z = Other.Z;
        RX = Other.RX; RY = Other.RY; RZ = Other.RZ;
    }
    public void Copy(RobotCommand Other)
    {
        X = Other.X ??= 0;
        Y = Other.Y ??= 0;
        Z = Other.Z ??= 0;
        RX = Other.RX ??= 0;
        RY = Other.RY ??= 0;
        RZ = Other.RZ ??= 0;
    }

    public static Vector6 Zero => new()
    {
        X = 0,
        Y = 0,
        Z = 0,
        RX = 0,
        RY = 0,
        RZ = 0
    };

    public double Length() =>
    Math.Sqrt(
        X * X +
        Y * Y +
        Z * Z +
        RX * RX +
        RY * RY +
        RZ * RZ
    );

    public double Norm() =>
        Math.Sqrt(X * X + Y * Y + Z * Z + RX * RX + RY * RY + RZ * RZ);

    public double MaxAbsComponent() =>
        Math.Max(
            Math.Max(Math.Max(Math.Abs(X), Math.Abs(Y)), Math.Abs(Z)),
            Math.Max(Math.Max(Math.Abs(RX), Math.Abs(RY)), Math.Abs(RZ))
        );
}
abstract class PathSegment
{
    public double Length;
    public abstract Vector6 Sample(double s); // s in [0..Length]
}

class LineSegment : PathSegment
{
    private Vector6 a, b, delta;

    public LineSegment(Vector6 a, Vector6 b)
    {
        this.a = a;
        this.b = b;
        delta = b - a;
        Length = Math.Sqrt(delta.X * delta.X + delta.Y * delta.Y + delta.Z * delta.Z);
    }

    public override Vector6 Sample(double s)
    {
        double t = Length < 1e-9 ? 0 : s / Length;
        return a + delta * t;
    }
}

class ArcSegment : PathSegment
{
    private Vector6 center;
    private Vector6 start;
    private Vector6 normal;
    private double radius;
    private double angle;

    public ArcSegment(Vector6 center, Vector6 start, Vector6 normal, double radius, double angle)
    {
        this.center = center;
        this.start = start;
        this.normal = normal;
        this.radius = radius;
        this.angle = angle;
        Length = Math.Abs(radius * angle);
    }

    public override Vector6 Sample(double s)
    {
        double t = s / Length;
        double a = angle * t;

        double cos = Math.Cos(a);
        double sin = Math.Sin(a);

        Vector6 r = start - center;

        Vector6 p = new Vector6(
            center.X + r.X * cos - r.Y * sin,
            center.Y + r.X * sin + r.Y * cos,
            center.Z,
            start.RX + (start.RX * t),
            start.RY + (start.RY * t),
            start.RZ + (start.RZ * t)
        );

        return p;
    }
}

public class RobotCommand
{
    public string? CommandType { get; set; }

    [JsonPropertyName("name")]
    public string? Name { get; set; }

    public double? X { get; set; }
    public double? Y { get; set; }
    public double? Z { get; set; }
    public double? RX { get; set; }
    public double? RY { get; set; }
    public double? RZ { get; set; }
    public double? TX { get; set; }
    public double? TY { get; set; }
    public double? TZ { get; set; }
    public double? TRX { get; set; }
    public double? TRY { get; set; }
    public double? TRZ { get; set; }


    public double? Speed { get; set; }
    public double? Accel { get; set; }
    public double? Decel { get; set; }
    public double? Time { get; set; }

    /// <summary>
    /// Optional status update attached to this command.
    /// When the command is dequeued and starts executing the update is applied
    /// to the named program so the mobile app sees live progress.
    /// </summary>
    [JsonPropertyName("statusUpdate")]
    public ProgramCycleUpdate? StatusUpdate { get; set; }

    public Vector6 Vector6 => new(X ?? 0, Y ?? 0, Z ?? 0, RX ?? 0, RY ?? 0, RZ ?? 0);
    public Vector6 ToolOffsetVector6 => new(TX ?? 0, TY ?? 0, TZ ?? 0, TRX ?? 0, TRY ?? 0, TRZ ?? 0);
}


// ── Named-vector identity contract (shared by Point and Tool) ─────────────────

public interface INamedVector
{
    string? Name { get; set; }
    long LastUpdatedUnixMs { get; set; }
}

// ── Point ─────────────────────────────────────────────────────────────────────

public class Point : Vector6, INamedVector
{
    public string? Name { get; set; }

    // Unix ms when this point was last created or modified
    public long LastUpdatedUnixMs { get; set; }
}

// ── Tool ──────────────────────────────────────────────────────────────────────

/// <summary>TCP offset tool frame stored in the tool repository.</summary>
public class Tool : Vector6, INamedVector
{
    public string? Name { get; set; }
    public string Description { get; set; } = "";
    public long LastUpdatedUnixMs { get; set; }
}

public class ToolHistoryEntry
{
    public long TimestampUnixMs { get; set; }
    public Tool Tool { get; set; } = new();
}

// ── Tool command params ───────────────────────────────────────────────────────

public class EditToolParams
{
    [JsonPropertyName("name")]        public string  Name        { get; set; } = default!;
    [JsonPropertyName("newName")]     public string? NewName     { get; set; }
    [JsonPropertyName("description")] public string? Description { get; set; }
    [JsonPropertyName("x")]           public double? X           { get; set; }
    [JsonPropertyName("y")]           public double? Y           { get; set; }
    [JsonPropertyName("z")]           public double? Z           { get; set; }
    [JsonPropertyName("rx")]          public double? RX          { get; set; }
    [JsonPropertyName("ry")]          public double? RY          { get; set; }
    [JsonPropertyName("rz")]          public double? RZ          { get; set; }
}

public class ToolNameParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;
}

// ── Local ─────────────────────────────────────────────────────────────────────

/// <summary>Named user/work-frame coordinate system stored in the local repository.</summary>
public class Local : Vector6, INamedVector
{
    public string? Name { get; set; }
    public string Description { get; set; } = "";
    public long LastUpdatedUnixMs { get; set; }
}

public class LocalHistoryEntry
{
    public long TimestampUnixMs { get; set; }
    public Local Local { get; set; } = new();
}

// ── Local command params ──────────────────────────────────────────────────────

public class EditLocalParams
{
    [JsonPropertyName("name")]        public string  Name        { get; set; } = default!;
    [JsonPropertyName("newName")]     public string? NewName     { get; set; }
    [JsonPropertyName("description")] public string? Description { get; set; }
    [JsonPropertyName("x")]           public double? X           { get; set; }
    [JsonPropertyName("y")]           public double? Y           { get; set; }
    [JsonPropertyName("z")]           public double? Z           { get; set; }
    [JsonPropertyName("rx")]          public double? RX          { get; set; }
    [JsonPropertyName("ry")]          public double? RY          { get; set; }
    [JsonPropertyName("rz")]          public double? RZ          { get; set; }
}

public class LocalNameParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;
}

public class TeachPointParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;
}

public class EditPointParams
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = default!;

    [JsonPropertyName("newName")]
    public string? NewName { get; set; }

    [JsonPropertyName("x")]
    public double? X { get; set; }

    [JsonPropertyName("y")]
    public double? Y { get; set; }

    [JsonPropertyName("z")]
    public double? Z { get; set; }

    [JsonPropertyName("rx")]
    public double? RX { get; set; }

    [JsonPropertyName("ry")]
    public double? RY { get; set; }

    [JsonPropertyName("rz")]
    public double? RZ { get; set; }
}

public class PointHistoryEntry
{
    public long TimestampUnixMs { get; set; }
    public Point Point { get; set; } = new();
}

// ── Nano I/O command params ────────────────────────────────────────────────────

public class SetNanoOutputParams
{
    [JsonPropertyName("nanoId")] public string NanoId { get; set; } = "";
    [JsonPropertyName("pin")]    public int    Pin    { get; set; }
    [JsonPropertyName("value")]  public bool   Value  { get; set; }
}

public class SetNeoPixelParams
{
    [JsonPropertyName("nanoId")] public string NanoId { get; set; } = "";
    [JsonPropertyName("pin")]    public int    Pin    { get; set; }

    /// <summary>Array of { r, g, b } objects, one per pixel.</summary>
    [JsonPropertyName("colors")]
    public List<NeoPixelColorParams> Colors { get; set; } = new();
}

public class NeoPixelColorParams
{
    [JsonPropertyName("r")] public byte R { get; set; }
    [JsonPropertyName("g")] public byte G { get; set; }
    [JsonPropertyName("b")] public byte B { get; set; }
}

public class RenameNanoPinParams
{
    [JsonPropertyName("nanoId")]  public string NanoId  { get; set; } = "";
    [JsonPropertyName("pin")]     public int    Pin     { get; set; }
    [JsonPropertyName("name")]    public string Name    { get; set; } = "";
}

public class ConfigureNanoPinParams
{
    [JsonPropertyName("nanoId")]     public string NanoId     { get; set; } = "";
    [JsonPropertyName("pin")]        public int    Pin        { get; set; }
    [JsonPropertyName("type")]       public string Type       { get; set; } = "Input";
    [JsonPropertyName("pixelCount")] public int    PixelCount { get; set; } = 8;
}

// ── USB Relay ─────────────────────────────────────────────────────────────────

public class SetRelayParams
{
    [JsonPropertyName("relay")] public int  Relay { get; set; }  // 1–4
    [JsonPropertyName("value")] public bool Value { get; set; }
}

public class RenameRelayParams
{
    [JsonPropertyName("relay")] public int    Relay { get; set; }  // 1–4
    [JsonPropertyName("name")]  public string Name  { get; set; } = "";
}

public class SaveGridParams
{
    [JsonPropertyName("id")]             public string  Id             { get; set; } = "";
    [JsonPropertyName("name")]           public string  Name           { get; set; } = "";
    [JsonPropertyName("basePointName")]  public string  BasePointName  { get; set; } = "";
    [JsonPropertyName("rowOffsetX")]     public double  RowOffsetX     { get; set; }
    [JsonPropertyName("rowOffsetY")]     public double  RowOffsetY     { get; set; }
    [JsonPropertyName("rowOffsetZ")]     public double  RowOffsetZ     { get; set; }
    [JsonPropertyName("colOffsetX")]     public double  ColOffsetX     { get; set; }
    [JsonPropertyName("colOffsetY")]     public double  ColOffsetY     { get; set; }
    [JsonPropertyName("colOffsetZ")]     public double  ColOffsetZ     { get; set; }
    [JsonPropertyName("rowCount")]       public int?    RowCount       { get; set; }
    [JsonPropertyName("colCount")]       public int?    ColCount       { get; set; }
    [JsonPropertyName("rotation")]       public double  Rotation       { get; set; }
}

public class GridIdParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

// ── Stack ─────────────────────────────────────────────────────────────────────

/// <summary>A 1-D positional array — position = basePoint + index × offset.</summary>
public class RobotStack
{
    [JsonPropertyName("id")]             public string Id             { get; set; } = "";
    [JsonPropertyName("name")]           public string Name           { get; set; } = "";
    [JsonPropertyName("basePointName")]  public string BasePointName  { get; set; } = "";
    [JsonPropertyName("offsetX")]        public double OffsetX        { get; set; }
    [JsonPropertyName("offsetY")]        public double OffsetY        { get; set; }
    [JsonPropertyName("offsetZ")]        public double OffsetZ        { get; set; }
    /// <summary>When set, index wraps via modulo (round-robin).</summary>
    [JsonPropertyName("maxCount")]       public int?   MaxCount       { get; set; }
    [JsonPropertyName("lastUpdatedUnixMs")] public long LastUpdatedUnixMs { get; set; }
}

/// <summary>Identifies an entry in a named stack — used as the target in a MoveL/MoveJ/JumpL/JumpJ step.</summary>
public class StackPointRef
{
    [JsonPropertyName("stackId")] public string  StackId { get; set; } = "";
    [JsonPropertyName("index")]   public double? Index   { get; set; }
}

public class SaveStackParams
{
    [JsonPropertyName("id")]             public string  Id             { get; set; } = "";
    [JsonPropertyName("name")]           public string  Name           { get; set; } = "";
    [JsonPropertyName("basePointName")]  public string  BasePointName  { get; set; } = "";
    [JsonPropertyName("offsetX")]        public double  OffsetX        { get; set; }
    [JsonPropertyName("offsetY")]        public double  OffsetY        { get; set; }
    [JsonPropertyName("offsetZ")]        public double  OffsetZ        { get; set; }
    [JsonPropertyName("maxCount")]       public int?    MaxCount       { get; set; }
}

public class StackIdParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

// ── Aux Axis command params ────────────────────────────────────────────────────

public class MoveAuxParams
{
    [JsonPropertyName("deviceId")]  public string DeviceId  { get; set; } = "AUX_STEPPER_001";
    [JsonPropertyName("axis")]      public int    Axis      { get; set; }
    [JsonPropertyName("steps")]     public long   Steps     { get; set; }
    [JsonPropertyName("velocity")]  public double Velocity  { get; set; } = 1000;
    [JsonPropertyName("accel")]     public double Accel     { get; set; } = 10000;
    [JsonPropertyName("decel")]     public double Decel     { get; set; } = 10000;
}

public class JogAuxParams
{
    [JsonPropertyName("deviceId")]  public string DeviceId  { get; set; } = "AUX_STEPPER_001";
    [JsonPropertyName("axis")]      public int    Axis      { get; set; }
    /// <summary>Steps/sec. Positive=CW, negative=CCW. 0 = stop.</summary>
    [JsonPropertyName("velocity")]  public double Velocity  { get; set; }
    [JsonPropertyName("accel")]     public double Accel     { get; set; } = 10000;
    [JsonPropertyName("decel")]     public double Decel     { get; set; } = 10000;
}

public class StopAuxParams
{
    [JsonPropertyName("deviceId")]  public string DeviceId  { get; set; } = "AUX_STEPPER_001";
    [JsonPropertyName("axis")]      public int?   Axis      { get; set; }
    [JsonPropertyName("decel")]     public double Decel     { get; set; } = 10000;
    [JsonPropertyName("immediate")] public bool   Immediate { get; set; } = false;
}

public class EnableAuxParams
{
    [JsonPropertyName("deviceId")] public string DeviceId { get; set; } = "AUX_STEPPER_001";
    [JsonPropertyName("enable")]   public bool   Enable   { get; set; } = true;
}

// ── Camera command params ─────────────────────────────────────────────────────

public class AddCameraParams
{
    [JsonPropertyName("name")]        public string Name        { get; set; } = "Camera";
    [JsonPropertyName("deviceIndex")] public int    DeviceIndex { get; set; } = 0;
    [JsonPropertyName("enabled")]     public bool   Enabled     { get; set; } = true;
    [JsonPropertyName("width")]       public int    Width       { get; set; } = 640;
    [JsonPropertyName("height")]      public int    Height      { get; set; } = 480;
    [JsonPropertyName("targetFps")]   public int    TargetFps   { get; set; } = 15;
}

public class RemoveCameraParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class SetCameraConfigParams
{
    [JsonPropertyName("id")]          public string Id          { get; set; } = "";
    [JsonPropertyName("name")]        public string Name        { get; set; } = "Camera";
    [JsonPropertyName("deviceIndex")] public int    DeviceIndex { get; set; } = 0;
    [JsonPropertyName("enabled")]     public bool   Enabled     { get; set; } = true;
    [JsonPropertyName("width")]       public int    Width       { get; set; } = 640;
    [JsonPropertyName("height")]      public int    Height      { get; set; } = 480;
    [JsonPropertyName("targetFps")]   public int    TargetFps   { get; set; } = 15;
}

public class GetCameraResolutionsParams
{
    [JsonPropertyName("deviceIndex")] public int DeviceIndex { get; set; } = 0;
}

public class SetAuxAxisConfigParams
{
    [JsonPropertyName("deviceId")]       public string DeviceId       { get; set; } = "AUX_STEPPER_001";
    [JsonPropertyName("axisIndex")]      public int    AxisIndex      { get; set; }
    [JsonPropertyName("name")]           public string Name           { get; set; } = "";
    [JsonPropertyName("stepsPerRev")]    public int    StepsPerRev    { get; set; } = 1600;
    [JsonPropertyName("invertDirection")]public bool   InvertDirection{ get; set; } = false;
    [JsonPropertyName("axisType")]       public string AxisType       { get; set; } = "";
    [JsonPropertyName("gearRatio")]      public double GearRatio      { get; set; } = 1.0;
    [JsonPropertyName("mmPerRev")]       public double MmPerRev       { get; set; } = 0.0;
}

/// <summary>Relay board state included in GetIO responses.</summary>
public class UsbRelayState
{
    [JsonPropertyName("connected")] public bool     Connected { get; set; }
    [JsonPropertyName("serial")]    public string?  Serial    { get; set; }
    [JsonPropertyName("relays")]    public bool[]?  Relays    { get; set; }  // index 0 = relay 1
    [JsonPropertyName("names")]     public string[] Names     { get; set; } = [];
}

// ── Vision command params ─────────────────────────────────────────────────────

public class DeleteVisionProgramParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class StartStopVisionParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class SetSpeedOverrideParams
{
    [JsonPropertyName("percent")] public double Percent { get; set; } = 100.0;
}