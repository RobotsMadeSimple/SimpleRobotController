using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

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


public class SetAuxAxisConfigParams
{
    [JsonPropertyName("deviceId")]       public string DeviceId       { get; set; } = "AUX_STEPPER_001";
    [JsonPropertyName("axisIndex")]      public int    AxisIndex      { get; set; }
    [JsonPropertyName("name")]           public string Name           { get; set; } = "";
    [JsonPropertyName("stepsPerRev")]    public int    StepsPerRev    { get; set; } = 1600;
    [JsonPropertyName("invertDirection")]public bool   InvertDirection{ get; set; } = false;
    [JsonPropertyName("axisType"), JsonConverter(typeof(AuxAxis.AuxAxisTypeJsonConverter))]
    public AuxAxis.AuxAxisType AxisType { get; set; } = AuxAxis.AuxAxisType.Unconfigured;
    [JsonPropertyName("gearRatio")]      public double GearRatio      { get; set; } = 1.0;
    [JsonPropertyName("mmPerRev")]       public double MmPerRev       { get; set; } = 0.0;
    [JsonPropertyName("role"), JsonConverter(typeof(AuxAxis.AuxAxisRoleJsonConverter))]
    public AuxAxis.AuxAxisRole Role { get; set; } = AuxAxis.AuxAxisRole.Generic;
    [JsonPropertyName("actionDistance")] public double ActionDistance { get; set; } = 0.0;
    [JsonPropertyName("actionVelocity")] public double ActionVelocity { get; set; } = 0.0;
    [JsonPropertyName("gripperOpenMm")]   public double GripperOpenMm   { get; set; } = 0.0;
    [JsonPropertyName("gripperClosedMm")] public double GripperClosedMm { get; set; } = 0.0;
    [JsonPropertyName("gripperOpenTimeSec")]  public double GripperOpenTimeSec  { get; set; } = 0.0;
    [JsonPropertyName("gripperCloseTimeSec")] public double GripperCloseTimeSec { get; set; } = 0.0;
    [JsonPropertyName("gripperAccel")]        public double GripperAccel        { get; set; } = 0.0;
    [JsonPropertyName("gripperOverTravelMm")] public double GripperOverTravelMm { get; set; } = 0.0;
}

public class SetAuxPositionParams
{
    [JsonPropertyName("deviceId")] public string DeviceId { get; set; } = "AUX_STEPPER_001";
    [JsonPropertyName("axis")]     public int    Axis     { get; set; }
    /// <summary>Tracked step position to declare (0 clears / zeroes the axis).</summary>
    [JsonPropertyName("steps")]    public long   Steps    { get; set; }
}


