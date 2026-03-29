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
public enum StepType { MoveL, MoveJ, SetOutput, Wait, Loop, StatusUpdate }

public class ProgramStep
{
    [JsonPropertyName("id")]
    public string Id { get; set; } = "";

    [JsonPropertyName("name")]
    public string? Name { get; set; }

    [JsonPropertyName("type")]
    public StepType Type { get; set; }

    // MoveL / MoveJ
    [JsonPropertyName("pointName")]
    public string? PointName { get; set; }
    [JsonPropertyName("speed")]
    public double Speed { get; set; } = 100;
    [JsonPropertyName("accel")]
    public double Accel { get; set; } = 100;
    [JsonPropertyName("decel")]
    public double Decel { get; set; } = 100;

    // Optional position offset added to the target point (mm / deg)
    [JsonPropertyName("offsetX")]  public double? OffsetX  { get; set; }
    [JsonPropertyName("offsetY")]  public double? OffsetY  { get; set; }
    [JsonPropertyName("offsetZ")]  public double? OffsetZ  { get; set; }
    [JsonPropertyName("offsetRX")] public double? OffsetRX { get; set; }
    [JsonPropertyName("offsetRY")] public double? OffsetRY { get; set; }
    [JsonPropertyName("offsetRZ")] public double? OffsetRZ { get; set; }

    // Optional tool to apply as a TCP offset (looked up from ToolRepository at execution time)
    [JsonPropertyName("toolName")]
    public string? ToolName { get; set; }

    // SetOutput
    [JsonPropertyName("outputNumber")]
    public int? OutputNumber { get; set; }
    [JsonPropertyName("outputValue")]
    public bool? OutputValue { get; set; }

    // Wait
    [JsonPropertyName("waitMs")]
    public int? WaitMs { get; set; }

    // Loop
    [JsonPropertyName("loopCount")]
    public int? LoopCount { get; set; }          // 0 = infinite
    [JsonPropertyName("loopSteps")]
    public List<ProgramStep>? LoopSteps { get; set; }

    // StatusUpdate
    [JsonPropertyName("statusMessage")]
    public string? StatusMessage { get; set; }
    [JsonPropertyName("statusWarning")]
    public string? StatusWarning { get; set; }
    [JsonPropertyName("statusError")]
    public string? StatusError { get; set; }
}

public class BuiltProgram
{
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";
    [JsonPropertyName("description")]
    public string Description { get; set; } = "";
    [JsonPropertyName("steps")]
    public List<ProgramStep> Steps { get; set; } = new();
    [JsonPropertyName("lastUpdatedUnixMs")]
    public long LastUpdatedUnixMs { get; set; }
}

public class SaveBuiltProgramParams
{
    [JsonPropertyName("name")]        public string Name        { get; set; } = "";
    [JsonPropertyName("description")] public string Description { get; set; } = "";
    [JsonPropertyName("steps")]       public List<ProgramStep> Steps { get; set; } = new();
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

public class CommandMessage
{
    public string Type { get; set; } = default!;
    public string Id { get; set; } = default!;
    public string Command { get; set; } = default!;
    public JsonElement? Params { get; set; }
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