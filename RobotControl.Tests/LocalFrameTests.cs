using Controller.RobotControl;

namespace RobotControl.Tests;

public class LocalFrameTests
{
    private static void AssertVecEqual(Vector6 expected, Vector6 actual, double tol = 1e-9)
    {
        Assert.Equal(expected.X, actual.X, tol);
        Assert.Equal(expected.Y, actual.Y, tol);
        Assert.Equal(expected.Z, actual.Z, tol);
        Assert.Equal(expected.RX, actual.RX, tol);
        Assert.Equal(expected.RY, actual.RY, tol);
        Assert.Equal(expected.RZ, actual.RZ, tol);
    }

    [Fact]
    public void ApplyWithZeroRotationTranslatesOnly()
    {
        var local = new Vector6(100, 50, -10, 0, 0, 0);
        var p = new Vector6(5, 7, 2, 0, 0, 30);

        var world = LocalFrame.Apply(local, p);

        AssertVecEqual(new Vector6(105, 57, -8, 0, 0, 30), world);
    }

    [Fact]
    public void ApplyRotates90AboutLocalOrigin()
    {
        // Local frame rotated 90° about Z: local +X axis points along world +Y.
        var local = new Vector6(100, 200, 0, 0, 0, 90);
        var p = new Vector6(10, 0, 0, 0, 0, 0);

        var world = LocalFrame.Apply(local, p);

        AssertVecEqual(new Vector6(100, 210, 0, 0, 0, 90), world);
    }

    [Fact]
    public void ApplyRotatesEntireXyGeometryNotComponents()
    {
        // Two points on a local-frame square must keep their relative distance
        // and be rotated as one rigid body about the local origin.
        var local = new Vector6(50, -20, 5, 0, 0, 45);
        var a = LocalFrame.Apply(local, new Vector6(0, 0, 0));
        var b = LocalFrame.Apply(local, new Vector6(20, 0, 0));

        double dist = Math.Sqrt(Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2));
        Assert.Equal(20, dist, 1e-9);
        // 45° rotation: the segment direction should be (cos45, sin45).
        Assert.Equal(20 * Math.Cos(Math.PI / 4), b.X - a.X, 1e-9);
        Assert.Equal(20 * Math.Sin(Math.PI / 4), b.Y - a.Y, 1e-9);
    }

    [Fact]
    public void InverseUndoesApply()
    {
        var local = new Vector6(12.5, -3.75, 8, 1, -2, 33);
        var p = new Vector6(4, -9, 2.5, 0.5, 1.5, -10);

        var roundTrip = LocalFrame.Inverse(local, LocalFrame.Apply(local, p));

        AssertVecEqual(p, roundTrip);
    }

    [Fact]
    public void ApplyUndoesInverse()
    {
        // CNC anchor semantics: anchor = Inverse(local, currentPos), then
        // Apply(local, anchor) must land exactly back at currentPos.
        var local = new Vector6(-40, 15, 2, 0, 0, -72);
        var pos = new Vector6(210, 180, 55, 0, 0, 90);

        var anchor = LocalFrame.Inverse(local, pos);
        var back = LocalFrame.Apply(local, anchor);

        AssertVecEqual(pos, back);
    }

    [Fact]
    public void ToolOrientationIgnoresFrameTiltButFollowsYaw()
    {
        // The arm can't tilt: the frame's RX/RY must not leak into tool RX/RY,
        // while its RZ (yaw) is achievable and adds to the tool RZ.
        var local = new Vector6(0, 0, 0, 5, -10, 15);
        var p = new Vector6(0, 0, 0, 1, 2, 3);

        var world = LocalFrame.Apply(local, p);

        Assert.Equal(1, world.RX, 1e-9);
        Assert.Equal(2, world.RY, 1e-9);
        Assert.Equal(18, world.RZ, 1e-9);
    }

    [Fact]
    public void TiltedFrameMapsLocalXOntoZSlope()
    {
        // RY = 30°: moving +X in the local frame descends in world Z
        // (Rz·Ry·Rx convention: z' = -sin(ry)·x for pure pitch).
        var local = new Vector6(0, 0, 100, 0, 30, 0);
        var p = new Vector6(10, 0, 0);

        var world = LocalFrame.Apply(local, p);

        Assert.Equal(10 * Math.Cos(Math.PI / 6), world.X, 1e-9);
        Assert.Equal(0, world.Y, 1e-9);
        Assert.Equal(100 - 10 * Math.Sin(Math.PI / 6), world.Z, 1e-9);
    }

    [Fact]
    public void TiltedFramePreservesDistances()
    {
        // A rigid transform: distances between points survive any tilt.
        var local = new Vector6(12, -7, 3, 20, -35, 110);
        var a = new Vector6(1, 2, 3);
        var b = new Vector6(-4, 6, -2);
        double before = Dist(a, b);

        double after = Dist(LocalFrame.Apply(local, a), LocalFrame.Apply(local, b));

        Assert.Equal(before, after, 1e-9);
    }

    [Fact]
    public void TiltedFrameRoundTripsExactly()
    {
        var local = new Vector6(-30, 44, 12, 15, -25, 60);
        var p = new Vector6(7, -3, 9, 2, -1, 45);

        var roundTrip = LocalFrame.Inverse(local, LocalFrame.Apply(local, p));

        AssertVecEqual(p, roundTrip);
    }

    private static double Dist(Vector6 a, Vector6 b) =>
        Math.Sqrt(Math.Pow(b.X - a.X, 2) + Math.Pow(b.Y - a.Y, 2) + Math.Pow(b.Z - a.Z, 2));
}
