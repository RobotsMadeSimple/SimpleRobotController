using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

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

/// <summary>
/// A circular arc in 3D between two tangent points, used to round the corner at a
/// waypoint (move blending). Position follows the arc; orientation (RX/RY/RZ) is
/// interpolated linearly from start to end.
/// </summary>
class ArcSegment : PathSegment
{
    private readonly Vector6 startV, endV;
    private readonly double cx, cy, cz;   // arc centre (XYZ)
    private readonly double ux, uy, uz;   // unit radial from centre → start (XYZ)
    private readonly double nx, ny, nz;   // unit rotation axis
    private readonly double radius;
    private readonly double sweep;        // swept angle in radians

    public ArcSegment(Vector6 start, Vector6 end, Vector6 centre, Vector6 axis, double radius, double sweep)
    {
        startV = start;
        endV   = end;
        cx = centre.X; cy = centre.Y; cz = centre.Z;
        this.radius = radius;
        this.sweep  = sweep;

        double rx = start.X - centre.X, ry = start.Y - centre.Y, rz = start.Z - centre.Z;
        double rlen = Math.Sqrt(rx * rx + ry * ry + rz * rz);
        if (rlen < 1e-9) rlen = 1;
        ux = rx / rlen; uy = ry / rlen; uz = rz / rlen;

        double alen = Math.Sqrt(axis.X * axis.X + axis.Y * axis.Y + axis.Z * axis.Z);
        if (alen < 1e-9) alen = 1;
        nx = axis.X / alen; ny = axis.Y / alen; nz = axis.Z / alen;

        Length = Math.Abs(radius * sweep);
    }

    public override Vector6 Sample(double s)
    {
        double t = Length < 1e-9 ? 0 : Math.Clamp(s / Length, 0, 1);
        double ang = sweep * t;
        double c = Math.Cos(ang), sn = Math.Sin(ang);

        // Rodrigues rotation of the (perpendicular) radial around the axis: u·cos + (n×u)·sin
        double cxu = ny * uz - nz * uy;
        double cyu = nz * ux - nx * uz;
        double czu = nx * uy - ny * ux;

        double rxv = ux * c + cxu * sn;
        double ryv = uy * c + cyu * sn;
        double rzv = uz * c + czu * sn;

        return new Vector6(
            cx + radius * rxv,
            cy + radius * ryv,
            cz + radius * rzv,
            startV.RX + (endV.RX - startV.RX) * t,
            startV.RY + (endV.RY - startV.RY) * t,
            startV.RZ + (endV.RZ - startV.RZ) * t
        );
    }
}


