using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

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
    [JsonPropertyName("axisType")]       public string AxisType       { get; set; } = "";
    [JsonPropertyName("gearRatio")]      public double GearRatio      { get; set; } = 1.0;
    [JsonPropertyName("mmPerRev")]       public double MmPerRev       { get; set; } = 0.0;
}


