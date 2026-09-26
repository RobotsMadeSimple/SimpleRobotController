using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

/// <summary>
/// Rigid local frame transform. Positions get the FULL rotation (R = Rz·Ry·Rx,
/// degrees) plus translation — a tilted frame maps local XY motion onto a
/// sloped plane in world space. Tool orientation is handled separately: the
/// 4-axis arm can only yaw, so the frame's RZ adds to the tool RZ while the
/// RX/RY tool tilt the frame would impose is deliberately ignored (RX/RY pass
/// through unchanged — the Cartesian slope still takes effect).
/// </summary>
public static class LocalFrame
{
    private static double[,] Rotation(Vector6 local)
    {
        double a = local.RZ * Math.PI / 180.0; // yaw
        double b = local.RY * Math.PI / 180.0; // pitch
        double g = local.RX * Math.PI / 180.0; // roll
        double ca = Math.Cos(a), sa = Math.Sin(a);
        double cb = Math.Cos(b), sb = Math.Sin(b);
        double cg = Math.Cos(g), sg = Math.Sin(g);
        return new[,]
        {
            { ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg },
            { sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg },
            { -sb,     cb * sg,                cb * cg                },
        };
    }

    /// <summary>Local-frame pose → world pose.</summary>
    public static Vector6 Apply(Vector6 local, Vector6 p)
    {
        var r = Rotation(local);
        return new Vector6(
            local.X + r[0, 0] * p.X + r[0, 1] * p.Y + r[0, 2] * p.Z,
            local.Y + r[1, 0] * p.X + r[1, 1] * p.Y + r[1, 2] * p.Z,
            local.Z + r[2, 0] * p.X + r[2, 1] * p.Y + r[2, 2] * p.Z,
            p.RX,
            p.RY,
            p.RZ + local.RZ);
    }

    /// <summary>World pose → local-frame pose (exact inverse of Apply).</summary>
    public static Vector6 Inverse(Vector6 local, Vector6 world)
    {
        var r = Rotation(local);
        double dx = world.X - local.X, dy = world.Y - local.Y, dz = world.Z - local.Z;
        return new Vector6(
            r[0, 0] * dx + r[1, 0] * dy + r[2, 0] * dz,
            r[0, 1] * dx + r[1, 1] * dy + r[2, 1] * dz,
            r[0, 2] * dx + r[1, 2] * dy + r[2, 2] * dz,
            world.RX,
            world.RY,
            world.RZ - local.RZ);
    }

    /// <summary>
    /// Rotate a direction vector (X/Y/Z) from the local frame into world space,
    /// with no translation. Orientation components (RX/RY/RZ) pass through
    /// unchanged. Used to jog along the local frame's axes: a "+X" jog moves the
    /// tool along the local X direction in world space.
    /// </summary>
    public static Vector6 Rotate(Vector6 local, Vector6 dir)
    {
        var r = Rotation(local);
        return new Vector6(
            r[0, 0] * dir.X + r[0, 1] * dir.Y + r[0, 2] * dir.Z,
            r[1, 0] * dir.X + r[1, 1] * dir.Y + r[1, 2] * dir.Z,
            r[2, 0] * dir.X + r[2, 1] * dir.Y + r[2, 2] * dir.Z,
            dir.RX,
            dir.RY,
            dir.RZ);
    }
}


