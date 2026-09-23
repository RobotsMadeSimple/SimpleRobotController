using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── Program builder ───────────────────────────────────────────────────────────

[JsonConverter(typeof(JsonStringEnumConverter))]
public enum StepType { MoveL, MoveJ, JumpL, JumpJ, SetOutput, Wait, Loop, StatusUpdate, CallRoutine, SetSpeedL, SetSpeedJ, SetVariable, PauseProgram, Label, GoToLabel, IfCondition, SetTool, RunHoming, AuxMove, AuxContinuous, AuxStop, AuxEnable, RunVision, SetLocal, ClearLocal, StartBackground, StopBackground, WaitForBackground, StopwatchControl, SaveImage, ThreadMove, CncProgram, SetBlendRadius, HttpRequest, CaptureImage, HttpReceive, Unknown }

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

/// <summary>One key/expression pair for the outbound JSON body of a JsonExchange step.</summary>
public class JsonKeyValue
{
    [JsonPropertyName("key")]      public string  Key      { get; set; } = "";
    [JsonPropertyName("expr")]     public string  Expr     { get; set; } = "";
    /// <summary>When set, this row sends the named image variable as a base64 string instead of evaluating Expr.</summary>
    [JsonPropertyName("imageVar")] public string? ImageVar { get; set; }
    /// <summary>
    /// When set, this row sends the named list variable as a JSON array instead of evaluating Expr.
    /// Takes precedence over <see cref="ImageVar"/>, which in turn takes precedence over Expr — a row
    /// is exactly one of the three, and the order only matters for a malformed row that sets several.
    /// </summary>
    [JsonPropertyName("listVar")]  public string? ListVar  { get; set; }
}

/// <summary>Maps one response JSON key to a program variable for a JsonExchange step.</summary>
public class JsonInboundMapping
{
    [JsonPropertyName("key")]          public string Key          { get; set; } = "";
    [JsonPropertyName("variableName")] public string VariableName { get; set; } = "";
}

/// <summary>Maps an image variable to a JSON key for the outbound body of a JsonExchange step (sent as a base64 JPEG string).</summary>
public class JsonImageMapping
{
    [JsonPropertyName("key")]          public string Key          { get; set; } = "";
    [JsonPropertyName("variableName")] public string VariableName { get; set; } = "";
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
    [JsonPropertyName("inspectionId")]   public string  InspectionId   { get; set; } = "";
    [JsonPropertyName("coverageVar")]    public string? CoverageVar    { get; set; }
    [JsonPropertyName("passedVar")]      public string? PassedVar      { get; set; }
    /// <summary>
    /// Object-list variable to fill with one record per grid cell — fields row, col, index,
    /// coverage, passed. Only meaningful when the inspection's zone has a grid; on an
    /// ungridded zone it is left empty.
    /// </summary>
    [JsonPropertyName("cellsVar")]       public string? CellsVar       { get; set; }
    /// <summary>Scalar variable to fill with the number of cells that passed.</summary>
    [JsonPropertyName("cellsPassedVar")] public string? CellsPassedVar { get; set; }
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

    /// <summary>
    /// False = the executor skips this step (it still counts toward progress). Null or
    /// true = enabled. Lets a user park a step while debugging without deleting it.
    /// </summary>
    [JsonPropertyName("enabled")]
    public bool? Enabled { get; set; }

    /// <summary>Free-text note shown under the step in the editor. Not executed.</summary>
    [JsonPropertyName("comment")]
    public string? Comment { get; set; }

    [JsonIgnore] public bool IsEnabled => Enabled != false;

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
    // Move blending — when Blend is true the move rounds its corner into the next
    // move instead of stopping. BlendRadius optionally overrides the program's
    // current default blend radius (set by a SetBlendRadius step).
    [JsonPropertyName("blend")]
    public bool? Blend { get; set; }
    [JsonPropertyName("blendRadius")]
    public double? BlendRadius { get; set; }

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
    // "Info" | "Warning" | "Error" — severity hint set by the app
    [JsonPropertyName("statusSeverity")]
    public string? StatusSeverity { get; set; }

    // CallRoutine
    [JsonPropertyName("routineName")]
    public string? RoutineName { get; set; }

    [JsonPropertyName("routineId")]
    public string? RoutineId { get; set; }

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
    /// <summary>
    /// Coordinate frame of the blob points and polygon/ArUco centers written by RunVision:
    /// "pixel", "normalized" (0–1) or "robot" (mm on the camera's calibration plane, point
    /// Z = planeZ; needs a camera calibration). Null keeps the original mix — blob points in
    /// pixels, polygon/ArUco centers normalized. See docs/camera-calibration.md.
    /// </summary>
    [JsonPropertyName("outputFrame")]       public string? OutputFrame { get; set; }

    // Superseded by pointNameExpr, which expresses the same thing as "$name[index]".
    // Still read so programs saved before the merge keep running; the builder rewrites
    // them to pointNameExpr on save and no longer writes these.
    [JsonPropertyName("varPointName")]  public string? VarPointName  { get; set; }
    [JsonPropertyName("varPointIndex")] public string? VarPointIndex { get; set; }

    // Variable point target for move steps — overrides pointName when set. Resolved two
    // ways: an expression that is only an indexed points variable ("$pts[$i]") yields
    // those coordinates directly, anything else is interpolated to text naming a saved
    // point ("$target", "{$binPrefix}{$index}"). Either way it is re-resolved on every
    // execution, so assigning the variables it references retargets the move.
    [JsonPropertyName("pointNameExpr")] public string? PointNameExpr { get; set; }

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

    // CncProgram — toolpath spec built by the CNC builder. Steps are generated
    // at runtime from CncSpec; CncProgramSteps remains for programs saved by
    // older app versions that baked the steps in.
    [JsonPropertyName("cncDxfFile")]    public string?           CncDxfFile    { get; set; }
    [JsonPropertyName("cncSafeZ")]      public double?           CncSafeZ      { get; set; }
    [JsonPropertyName("cncProgramSteps")] public List<ProgramStep>? CncProgramSteps { get; set; }
    [JsonPropertyName("cncSpec")]       public CncSpec?          CncSpec       { get; set; }

    // JsonExchange — POST a JSON body to a URL; optionally load numeric values from the response
    [JsonPropertyName("jsonUrl")]             public string?                   JsonUrl             { get; set; }
    [JsonPropertyName("jsonWaitForResponse")] public bool?                     JsonWaitForResponse { get; set; }
    [JsonPropertyName("jsonTimeoutMs")]       public int?                      JsonTimeoutMs       { get; set; }
    [JsonPropertyName("jsonOutbound")]        public List<JsonKeyValue>?       JsonOutbound        { get; set; }
    [JsonPropertyName("jsonInbound")]         public List<JsonInboundMapping>? JsonInbound         { get; set; }
    [JsonPropertyName("jsonImageOutbound")]   public List<JsonImageMapping>?   JsonImageOutbound   { get; set; }
    // CaptureImage
    [JsonPropertyName("captureImageVariableName")] public string? CaptureImageVariableName { get; set; }
    [JsonPropertyName("captureImageCameraId")]     public string? CaptureImageCameraId     { get; set; }
    // HttpReceive
    [JsonPropertyName("httpReceiveName")]      public string?                   HttpReceiveName      { get; set; }
    [JsonPropertyName("httpReceiveTimeoutMs")] public int?                      HttpReceiveTimeoutMs { get; set; }
    [JsonPropertyName("httpReceiveInbound")]   public List<JsonInboundMapping>? HttpReceiveInbound   { get; set; }

    // Unknown — placeholder for steps whose type string could not be parsed
    [JsonPropertyName("unknownStepType")]
    public string? UnknownStepType { get; set; }
}


