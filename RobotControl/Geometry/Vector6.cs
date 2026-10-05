using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

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


