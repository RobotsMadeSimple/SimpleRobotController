using System;
using System.Collections.Generic;
using System.Text;
using System.Text.Json;

public abstract class Message
{
    public string Type { get; set; } = default!;
}

public class CommandMessage : Message
{
    public string Id { get; set; } = default!;
    public string Command { get; set; } = default!;
    public JsonElement? Params { get; set; }
}

public class MoveJ
{
    public int Joint { get; set; }
    public double Angle { get; set; }
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



public class SpeedSetting
{
    public double Speed { get; set; }
}

public class AccelSetting
{
    public double AccelSpeed { get; set; }
    public double DecelSpeed { get; set; }
}
public class SpeedJ
{
    public double Speed { get; set; }
}

public class AccelJ
{
    public double AccelSpeed { get; set; }
    public double DecelSpeed { get; set; }
}

public class RobotCommand
{
    public string? CommandType { get; set; }
    public double? X { get; set; }
    public double? Y { get; set; }
    public double? Z { get; set; }
    public double? RX { get; set; }
    public double? RY { get; set; }
    public double? RZ { get; set; }
    public double? Speed { get; set; }
    public double? Accel { get; set; }
    public double? Decel { get; set; }
    public double? Time { get; set; }

    // Avoid mutating your fields in a getter
    public Vector6 Vector6 => new(X ?? 0, Y ?? 0, Z ?? 0, RX ?? 0, RY ?? 0, RZ ?? 0);
}
