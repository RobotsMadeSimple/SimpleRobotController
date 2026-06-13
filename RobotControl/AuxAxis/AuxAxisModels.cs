using System.Collections.Generic;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.AuxAxis
{
    public class AuxAxisConfig
    {
        [JsonPropertyName("id")]             public string                    Id   { get; set; } = "AUX_STEPPER_001";
        [JsonPropertyName("name")]           public string                    Name { get; set; } = "Aux Stepper";
        [JsonPropertyName("axes")]           public List<AuxAxisChannelConfig> Axes { get; set; } = new();
    }

    public class AuxAxisChannelConfig
    {
        [JsonPropertyName("axisIndex")]       public int    AxisIndex       { get; set; }
        [JsonPropertyName("name")]            public string Name            { get; set; } = "";
        [JsonPropertyName("stepsPerRev")]     public int    StepsPerRev     { get; set; } = 1600;
        [JsonPropertyName("invertDirection")] public bool   InvertDirection { get; set; } = false;
        // "Rotary" (degrees) or "Linear" (mm). Empty = unconfigured (use raw steps).
        [JsonPropertyName("axisType")]        public string AxisType        { get; set; } = "";
        [JsonPropertyName("gearRatio")]       public double GearRatio       { get; set; } = 1.0;
        // Only used when AxisType == "Linear"
        [JsonPropertyName("mmPerRev")]        public double MmPerRev        { get; set; } = 0.0;

        /// <summary>Steps per physical output unit (mm for Linear, degrees for Rotary).
        /// Returns 0 when the axis is not configured (use raw steps instead).</summary>
        public double StepsPerUnit()
        {
            if (string.IsNullOrEmpty(AxisType)) return 0;
            if (AxisType == "Linear")
                return MmPerRev > 0 ? (StepsPerRev * GearRatio) / MmPerRev : 0;
            // Rotary
            return (StepsPerRev * GearRatio) / 360.0;
        }
    }

    public class AuxAxisManagerConfig
    {
        [JsonPropertyName("devices")]
        public List<AuxAxisConfig> Devices { get; set; } = new();
    }

    public class AuxAxisState
    {
        [JsonPropertyName("connected")]   public bool                     Connected   { get; set; }
        [JsonPropertyName("deviceId")]    public string                   DeviceId    { get; set; } = "";
        [JsonPropertyName("deviceName")]  public string                   DeviceName  { get; set; } = "";
        [JsonPropertyName("portName")]    public string?                  PortName    { get; set; }
        [JsonPropertyName("axes")]        public List<AuxAxisChannelState> Axes        { get; set; } = new();
    }

    public class AuxAxisChannelState
    {
        [JsonPropertyName("axisIndex")]       public int    AxisIndex       { get; set; }
        [JsonPropertyName("name")]            public string Name            { get; set; } = "";
        [JsonPropertyName("active")]          public bool   Active          { get; set; }
        [JsonPropertyName("position")]        public long   Position        { get; set; }
        [JsonPropertyName("stepsPerRev")]     public int    StepsPerRev     { get; set; } = 1600;
        [JsonPropertyName("invertDirection")] public bool   InvertDirection { get; set; } = false;
        [JsonPropertyName("axisType")]        public string AxisType        { get; set; } = "";
        [JsonPropertyName("gearRatio")]       public double GearRatio       { get; set; } = 1.0;
        [JsonPropertyName("mmPerRev")]        public double MmPerRev        { get; set; } = 0.0;
    }
}
