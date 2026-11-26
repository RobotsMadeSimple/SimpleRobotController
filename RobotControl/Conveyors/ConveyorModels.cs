using System.Text.Json.Serialization;
using Controller.RobotControl.Persistence;

namespace Controller.RobotControl.Conveyors
{
    // The conveyor-tracking wire + file model (docs/conveyor-tracking.md). Names are the wire
    // names; every property carries its JsonPropertyName so the WebSocket (PascalCase default)
    // and conveyors.json both use them.

    /// <summary>Where the belt's travel comes from.</summary>
    public static class ConveyorSourceKinds
    {
        /// <summary>An aux stepper axis drives the belt; travel = live step count ÷ steps per mm.</summary>
        public const string Aux    = "aux";
        /// <summary>An externally driven belt: running while <c>runOutput</c> is on, at <c>speedMmS</c>.</summary>
        public const string Timed  = "timed";
        /// <summary>No motion source — travel only changes through ConveyorSetTravel.</summary>
        public const string Manual = "manual";

        public static readonly IReadOnlyList<string> All = [Aux, Timed, Manual];
    }

    /// <summary>What latches a part's belt position.</summary>
    public static class ConveyorSensorKinds
    {
        public const string None   = "none";
        /// <summary>An aux-board latch pin (CFG:&lt;pin&gt;,L): the step counts are captured at the edge.</summary>
        public const string AuxPin = "auxPin";
        /// <summary>Any IO-device input; the belt position is interpolated at the controller's receive time.</summary>
        public const string Io     = "io";
        /// <summary>A driver-board input (~20 ms polling; slow belts only).</summary>
        public const string Stb    = "stb";

        public static readonly IReadOnlyList<string> All = [None, AuxPin, Io, Stb];
    }

    /// <summary>An output reference — the same shape the SetOutput step uses.</summary>
    public sealed class ConveyorIoRef
    {
        /// <summary>"stb" | "relay" | "io".</summary>
        [JsonPropertyName("outputCard")]     public string  OutputCard     { get; set; } = "stb";
        /// <summary>The IO device (outputCard "io" only).</summary>
        [JsonPropertyName("outputDeviceId")] public string? OutputDeviceId { get; set; }
        [JsonPropertyName("outputNumber")]   public int     OutputNumber   { get; set; } = 1;
    }

    public sealed class ConveyorSource
    {
        /// <summary><see cref="ConveyorSourceKinds"/>.</summary>
        [JsonPropertyName("kind")]      public string  Kind      { get; set; } = ConveyorSourceKinds.Manual;
        /// <summary>aux: the aux device id.</summary>
        [JsonPropertyName("deviceId")]  public string? DeviceId  { get; set; }
        /// <summary>aux: the axis index on that device.</summary>
        [JsonPropertyName("axis")]      public int     Axis      { get; set; }
        /// <summary>aux: belt mm per motor step. Null = from the axis config (Linear, mmPerRev).</summary>
        [JsonPropertyName("mmPerStep")] public double? MmPerStep { get; set; }
        /// <summary>timed: the output that runs the belt (relay / VFD enable).</summary>
        [JsonPropertyName("runOutput")] public ConveyorIoRef? RunOutput { get; set; }
        /// <summary>timed: belt speed while running (mm/s).</summary>
        [JsonPropertyName("speedMmS")]  public double  SpeedMmS  { get; set; }
    }

    /// <summary>The conveyor frame in robot base coordinates. +X is the direction of belt travel.</summary>
    public sealed class ConveyorFrame
    {
        [JsonPropertyName("x")]  public double X  { get; set; }
        [JsonPropertyName("y")]  public double Y  { get; set; }
        [JsonPropertyName("z")]  public double Z  { get; set; }
        [JsonPropertyName("rz")] public double RZ { get; set; }
    }

    public sealed class ConveyorSensor
    {
        /// <summary><see cref="ConveyorSensorKinds"/>.</summary>
        [JsonPropertyName("kind")]         public string  Kind         { get; set; } = ConveyorSensorKinds.None;
        /// <summary>auxPin / io: the device id.</summary>
        [JsonPropertyName("deviceId")]     public string? DeviceId     { get; set; }
        /// <summary>auxPin / io: the pin number.</summary>
        [JsonPropertyName("pin")]          public int?    Pin          { get; set; }
        /// <summary>stb: the input number (1–4).</summary>
        [JsonPropertyName("input")]        public int?    Input        { get; set; }
        /// <summary>True: a part is present while the input reads high. False (the default,
        /// pull-up wiring): present while it reads low.</summary>
        [JsonPropertyName("activeHigh")]   public bool    ActiveHigh   { get; set; }
        /// <summary>Part X in the conveyor frame at the moment the sensor latched it (mm).</summary>
        [JsonPropertyName("offsetMm")]     public double  OffsetMm     { get; set; }
        /// <summary>A re-trigger closer than this (belt mm) to the previous latch is ignored.</summary>
        [JsonPropertyName("minSpacingMm")] public double  MinSpacingMm { get; set; } = 10;
    }

    /// <summary>The span of conveyor X where the robot may track a part.</summary>
    public sealed class ConveyorWindow
    {
        [JsonPropertyName("startMm")] public double StartMm { get; set; }
        [JsonPropertyName("endMm")]   public double EndMm   { get; set; } = 300;
    }

    /// <summary>One configured belt (conveyors.json). Linked by its stable <see cref="Id"/>.</summary>
    public sealed class Conveyor : IStoredItem
    {
        /// <summary>Stable id — letters, digits and '_' only, so it can sit in <c>$conveyor.&lt;id&gt;.…</c>.</summary>
        [JsonPropertyName("id")]       public string         Id       { get; set; } = "";
        [JsonPropertyName("name")]     public string         Name     { get; set; } = "";
        [JsonPropertyName("source")]   public ConveyorSource Source   { get; set; } = new();
        /// <summary>Multiplies counted travel (belt slip / pulley error); set by calibration.</summary>
        [JsonPropertyName("scale")]    public double         Scale    { get; set; } = 1.0;
        [JsonPropertyName("frame")]    public ConveyorFrame  Frame    { get; set; } = new();
        [JsonPropertyName("sensor")]   public ConveyorSensor Sensor   { get; set; } = new();
        [JsonPropertyName("window")]   public ConveyorWindow Window   { get; set; } = new();
        /// <summary>The queue keeps at most this many parts; the oldest is dropped (missed) beyond it.</summary>
        [JsonPropertyName("maxQueue")] public int            MaxQueue { get; set; } = 32;
        [JsonPropertyName("lastUpdatedUnixMs")] public long  LastUpdatedUnixMs { get; set; }

        /// <summary>Unit vector of belt travel (+X of the conveyor frame) in base XY.</summary>
        [JsonIgnore]
        public (double X, double Y) Direction
        {
            get
            {
                double a = Frame.RZ * Math.PI / 180.0;
                return (Math.Cos(a), Math.Sin(a));
            }
        }

        /// <summary>The part frame for a part at conveyor X = <paramref name="partX"/>: the conveyor
        /// frame shifted along its X. A local frame (x, y, z, rz) for <see cref="LocalFrame.Apply"/>.</summary>
        public Vector6 PartFrame(double partX)
        {
            var (dx, dy) = Direction;
            return new Vector6(Frame.X + dx * partX, Frame.Y + dy * partX, Frame.Z, 0, 0, Frame.RZ);
        }

        /// <summary>A deep copy (the runtime keeps its own snapshot of the config).</summary>
        public Conveyor Clone() => new()
        {
            Id = Id, Name = Name, Scale = Scale, MaxQueue = MaxQueue, LastUpdatedUnixMs = LastUpdatedUnixMs,
            Source = new ConveyorSource
            {
                Kind = Source.Kind, DeviceId = Source.DeviceId, Axis = Source.Axis, MmPerStep = Source.MmPerStep,
                SpeedMmS = Source.SpeedMmS,
                RunOutput = Source.RunOutput is { } r
                    ? new ConveyorIoRef { OutputCard = r.OutputCard, OutputDeviceId = r.OutputDeviceId, OutputNumber = r.OutputNumber }
                    : null,
            },
            Frame  = new ConveyorFrame { X = Frame.X, Y = Frame.Y, Z = Frame.Z, RZ = Frame.RZ },
            Sensor = new ConveyorSensor
            {
                Kind = Sensor.Kind, DeviceId = Sensor.DeviceId, Pin = Sensor.Pin, Input = Sensor.Input,
                ActiveHigh = Sensor.ActiveHigh, OffsetMm = Sensor.OffsetMm, MinSpacingMm = Sensor.MinSpacingMm,
            },
            Window = new ConveyorWindow { StartMm = Window.StartMm, EndMm = Window.EndMm },
        };
    }

    /// <summary>A part in a conveyor's queue.</summary>
    public sealed class ConveyorPart
    {
        /// <summary>Per-conveyor sequence number, from 1.</summary>
        public long   Id            { get; init; }
        /// <summary>Belt travel (mm) when the part was latched (or added).</summary>
        public double TravelAtLatch { get; init; }
        /// <summary>Travel at which the part sat at conveyor X = 0: X now = travel − ZeroTravel.</summary>
        public double ZeroTravel    { get; init; }
        /// <summary>Released by ConveyorEndTracking(release): rides on, never tracked again, dropped
        /// silently at the window end.</summary>
        public bool   Released      { get; set; }

        public double XAt(double travel) => travel - ZeroTravel;
    }

    // ── Status / command payloads ───────────────────────────────────────────────

    public sealed class ConveyorPartStatus
    {
        [JsonPropertyName("id")]            public long   Id            { get; init; }
        /// <summary>X in the conveyor frame now (mm).</summary>
        [JsonPropertyName("x")]             public double X             { get; init; }
        [JsonPropertyName("travelAtLatch")] public double TravelAtLatch { get; init; }
    }

    /// <summary>One entry of the status broadcast's <c>conveyors</c> list.</summary>
    public sealed class ConveyorStatus
    {
        [JsonPropertyName("id")]            public string Id            { get; init; } = "";
        [JsonPropertyName("travelMm")]      public double TravelMm      { get; init; }
        [JsonPropertyName("speedMmS")]      public double SpeedMmS      { get; init; }
        [JsonPropertyName("running")]       public bool   Running       { get; init; }
        [JsonPropertyName("tracking")]      public bool   Tracking      { get; init; }
        [JsonPropertyName("trackedPartId")] public long?  TrackedPartId { get; init; }
        /// <summary>Queued parts, oldest first, capped at 16.</summary>
        [JsonPropertyName("parts")]         public List<ConveyorPartStatus> Parts { get; init; } = new();
        [JsonPropertyName("latched")]       public int    Latched       { get; init; }
        [JsonPropertyName("picked")]        public int    Picked        { get; init; }
        [JsonPropertyName("missed")]        public int    Missed        { get; init; }
        /// <summary>Travel at the last sensor latch (the calibration wizard's sensor step), or null.</summary>
        [JsonPropertyName("lastLatchTravelMm")] public double? LastLatchTravelMm { get; init; }
        /// <summary>False while the travel source cannot be read (aux device offline).</summary>
        [JsonPropertyName("sourceOk")]      public bool   SourceOk      { get; init; }
    }

    /// <summary>One calibration sample: the TCP on the belt mark, and the belt travel then.</summary>
    public sealed class ConveyorCalibrationSample
    {
        [JsonPropertyName("x")]             public double  X             { get; set; }
        [JsonPropertyName("y")]             public double  Y             { get; set; }
        [JsonPropertyName("z")]             public double  Z             { get; set; }
        [JsonPropertyName("travelMm")]      public double  TravelMm      { get; set; }
        /// <summary>sensorSample only: travel when the part was latched.</summary>
        [JsonPropertyName("latchTravelMm")] public double? LatchTravelMm { get; set; }
    }

    /// <summary>A ConveyorCalibrate proposal (not saved).</summary>
    public sealed class ConveyorCalibrationResult
    {
        [JsonPropertyName("frame")]          public ConveyorFrame Frame          { get; init; } = new();
        [JsonPropertyName("scale")]          public double        Scale          { get; init; } = 1;
        /// <summary>Null when no sensor sample was given.</summary>
        [JsonPropertyName("sensorOffsetMm")] public double?       SensorOffsetMm { get; init; }
        /// <summary>Largest distance of a sample from the fitted belt line (mm).</summary>
        [JsonPropertyName("residualMm")]     public double        ResidualMm     { get; init; }
    }
}
