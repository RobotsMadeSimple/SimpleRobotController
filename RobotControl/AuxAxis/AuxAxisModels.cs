using System.Collections.Generic;
using System.Text.Json;
using System.Text.Json.Serialization;
using Controller.RobotControl.Io;

namespace Controller.RobotControl.AuxAxis
{
    /// <summary>Rotary (degrees) or Linear (mm) output for an aux axis. Unconfigured = use raw steps.</summary>
    [JsonConverter(typeof(AuxAxisTypeJsonConverter))]
    public enum AuxAxisType
    {
        Unconfigured,
        Rotary,
        Linear,
    }

    /// <summary>
    /// Serializes <see cref="AuxAxisType"/> the same way the on-disk config and status
    /// payload always represented it: "Rotary" / "Linear" / "" (empty string for
    /// Unconfigured — not the literal name "Unconfigured"). System.Text.Json's
    /// JsonStringEnumConverter would write/require the member name, so a custom
    /// converter is needed to keep the file and payload text byte-for-byte identical
    /// to the previous plain-string field.
    /// </summary>
    public sealed class AuxAxisTypeJsonConverter : JsonConverter<AuxAxisType>
    {
        public override AuxAxisType Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) =>
            reader.GetString() switch
            {
                "Rotary" => AuxAxisType.Rotary,
                "Linear" => AuxAxisType.Linear,
                _        => AuxAxisType.Unconfigured, // "" (and anything unrecognized) -> Unconfigured
            };

        public override void Write(Utf8JsonWriter writer, AuxAxisType value, JsonSerializerOptions options) =>
            writer.WriteStringValue(value switch
            {
                AuxAxisType.Rotary => "Rotary",
                AuxAxisType.Linear => "Linear",
                _                  => "",
            });
    }

    /// <summary>
    /// What an aux axis drives — the role that picks its manual controls and program blocks.
    /// Separate from <see cref="AuxAxisType"/> (which is just the motion unit, mm vs degrees).
    /// Serialized as its name; missing / unrecognized -> Generic for forward-compat.
    /// </summary>
    public enum AuxAxisRole
    {
        Generic,
        Gripper,
        Conveyor,
        RotaryTable,
    }

    public sealed class AuxAxisRoleJsonConverter : JsonConverter<AuxAxisRole>
    {
        public override AuxAxisRole Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) =>
            reader.GetString() switch
            {
                "Gripper"     => AuxAxisRole.Gripper,
                "Conveyor"    => AuxAxisRole.Conveyor,
                "RotaryTable" => AuxAxisRole.RotaryTable,
                _             => AuxAxisRole.Generic,
            };

        public override void Write(Utf8JsonWriter writer, AuxAxisRole value, JsonSerializerOptions options) =>
            writer.WriteStringValue(value switch
            {
                AuxAxisRole.Gripper     => "Gripper",
                AuxAxisRole.Conveyor    => "Conveyor",
                AuxAxisRole.RotaryTable => "RotaryTable",
                _                       => "Generic",
            });
    }

    public class AuxAxisConfig
    {
        [JsonPropertyName("id")]             public string                    Id   { get; set; } = "AUX_STEPPER_001";
        [JsonPropertyName("name")]           public string                    Name { get; set; } = "Aux Stepper";
        [JsonPropertyName("axes")]           public List<AuxAxisChannelConfig> Axes { get; set; } = new();
        // Digital IO on the Uno's spare pins (same model as a Nano edge device). Input/Output only.
        [JsonPropertyName("ioPins")]         public List<IoPinConfig>         IoPins { get; set; } = new();
    }

    public class AuxAxisChannelConfig
    {
        [JsonPropertyName("axisIndex")]       public int    AxisIndex       { get; set; }
        [JsonPropertyName("name")]            public string Name            { get; set; } = "";
        [JsonPropertyName("stepsPerRev")]     public int    StepsPerRev     { get; set; } = 1600;
        [JsonPropertyName("invertDirection")] public bool   InvertDirection { get; set; } = false;
        // Rotary (degrees) or Linear (mm). Unconfigured = use raw steps. Serialized as
        // "Rotary" / "Linear" / "" — see AuxAxisTypeJsonConverter.
        [JsonPropertyName("axisType"), JsonConverter(typeof(AuxAxisTypeJsonConverter))]
        public AuxAxisType AxisType { get; set; } = AuxAxisType.Unconfigured;
        [JsonPropertyName("gearRatio")]       public double GearRatio       { get; set; } = 1.0;
        // Only used when AxisType == Linear
        [JsonPropertyName("mmPerRev")]        public double MmPerRev        { get; set; } = 0.0;

        // ── Role (what this axis drives) + its action parameters ──────────────────
        // Role picks the manual controls / program blocks; it is independent of AxisType.
        [JsonPropertyName("role"), JsonConverter(typeof(AuxAxisRoleJsonConverter))]
        public AuxAxisRole Role { get; set; } = AuxAxisRole.Generic;
        // Gripper Open/Close travel and RotaryTable index increment, in the axis's physical
        // unit (mm/deg) when AxisType is set, else raw steps.
        [JsonPropertyName("actionDistance")]  public double ActionDistance  { get; set; } = 0.0;
        // Role speed, in the axis's physical unit per second (mm/s for a Linear gripper/conveyor,
        // deg/s for rotary, else steps/s). The app converts to steps/s via StepsPerUnit().
        [JsonPropertyName("actionVelocity")]  public double ActionVelocity  { get; set; } = 0.0;

        // ── Gripper geometry (Role == Gripper) ────────────────────────────────────
        // Jaw-gap width (in the Linear unit, mm) at the two mechanical extremes. Open/Close
        // actions drive to these; "grip to X" drives to an absolute gap clamped to [Closed, Open].
        // Editable + re-captured from the live position so a gripper that has lost steps can be
        // re-referenced in either direction.
        [JsonPropertyName("gripperOpenMm")]   public double GripperOpenMm   { get; set; } = 0.0;
        [JsonPropertyName("gripperClosedMm")] public double GripperClosedMm { get; set; } = 0.0;
        // Gripper motion. The per-direction speed is derived from the full-stroke time
        // (speed = |Open-Closed| / time) so "open in 0.8 s" is honoured whatever the gap
        // target is; 0 = fall back to ActionVelocity. GripperAccel is the ramp in mm/s².
        [JsonPropertyName("gripperOpenTimeSec")]  public double GripperOpenTimeSec  { get; set; } = 0.0;
        [JsonPropertyName("gripperCloseTimeSec")] public double GripperCloseTimeSec { get; set; } = 0.0;
        [JsonPropertyName("gripperAccel")]        public double GripperAccel        { get; set; } = 0.0;
        // Extra travel (mm) driven past a hard stop on a full Open/Close so the jaws seat
        // firmly against the mechanical limit; the tracked position then re-references at the
        // clamped limit (it can never read past [Closed, Open]). 0 = don't over-travel.
        [JsonPropertyName("gripperOverTravelMm")] public double GripperOverTravelMm { get; set; } = 0.0;

        /// <summary>Steps per physical output unit (mm for Linear, degrees for Rotary).
        /// Returns 0 when the axis is not configured (use raw steps instead).</summary>
        public double StepsPerUnit()
        {
            if (AxisType == AuxAxisType.Unconfigured) return 0;
            if (AxisType == AuxAxisType.Linear)
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
        [JsonPropertyName("connected")]     public bool                     Connected     { get; set; }
        [JsonPropertyName("motorEnabled")]  public bool                     MotorEnabled  { get; set; } = true;
        [JsonPropertyName("deviceId")]      public string                   DeviceId      { get; set; } = "";
        [JsonPropertyName("deviceName")]    public string                   DeviceName    { get; set; } = "";
        [JsonPropertyName("portName")]      public string?                  PortName      { get; set; }
        // Firmware from the board's "ID?" handshake: "" / null when unknown (not connected, or legacy
        // firmware before 1.1.0 that only answers "ID:<id>").
        [JsonPropertyName("firmwareVersion")] public string                 FirmwareVersion { get; set; } = "";
        [JsonPropertyName("capabilities")]    public string[]?              Capabilities    { get; set; }
        [JsonPropertyName("axes")]          public List<AuxAxisChannelState> Axes         { get; set; } = new();
    }

    public class AuxAxisChannelState
    {
        [JsonPropertyName("axisIndex")]       public int    AxisIndex       { get; set; }
        [JsonPropertyName("name")]            public string Name            { get; set; } = "";
        [JsonPropertyName("active")]          public bool   Active          { get; set; }
        [JsonPropertyName("position")]        public long   Position        { get; set; }
        [JsonPropertyName("stepsPerRev")]     public int    StepsPerRev     { get; set; } = 1600;
        [JsonPropertyName("invertDirection")] public bool   InvertDirection { get; set; } = false;
        [JsonPropertyName("axisType"), JsonConverter(typeof(AuxAxisTypeJsonConverter))]
        public AuxAxisType AxisType { get; set; } = AuxAxisType.Unconfigured;
        [JsonPropertyName("gearRatio")]       public double GearRatio       { get; set; } = 1.0;
        [JsonPropertyName("mmPerRev")]        public double MmPerRev        { get; set; } = 0.0;
        [JsonPropertyName("role"), JsonConverter(typeof(AuxAxisRoleJsonConverter))]
        public AuxAxisRole Role { get; set; } = AuxAxisRole.Generic;
        [JsonPropertyName("actionDistance")]  public double ActionDistance  { get; set; } = 0.0;
        [JsonPropertyName("actionVelocity")]  public double ActionVelocity  { get; set; } = 0.0;
        [JsonPropertyName("gripperOpenMm")]   public double GripperOpenMm   { get; set; } = 0.0;
        [JsonPropertyName("gripperClosedMm")] public double GripperClosedMm { get; set; } = 0.0;
        [JsonPropertyName("gripperOpenTimeSec")]  public double GripperOpenTimeSec  { get; set; } = 0.0;
        [JsonPropertyName("gripperCloseTimeSec")] public double GripperCloseTimeSec { get; set; } = 0.0;
        [JsonPropertyName("gripperAccel")]        public double GripperAccel        { get; set; } = 0.0;
        [JsonPropertyName("gripperOverTravelMm")] public double GripperOverTravelMm { get; set; } = 0.0;
    }
}
