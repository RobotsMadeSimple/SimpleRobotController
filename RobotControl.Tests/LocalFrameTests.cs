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
    public void RotateMapsLocalJogDirectionToWorldWithoutTranslation()
    {
        // 90° local frame: a "+X" jog moves along world +Y, and the frame's
        // translation is irrelevant to a direction. Orientation passes through.
        var local = new Vector6(500, -300, 40, 0, 0, 90);
        var dir = new Vector6(1, 0, 0, 0, 0, 1);

        var world = LocalFrame.Rotate(local, dir);

        Assert.Equal(0, world.X, 9);
        Assert.Equal(1, world.Y, 9);
        Assert.Equal(0, world.Z, 9);
        Assert.Equal(1, world.RZ, 9);  // rotation jog unchanged
    }

    [Fact]
    public void RotatePreservesDirectionLength()
    {
        var local = new Vector6(10, 20, 30, 15, -25, 60);
        var dir = new Vector6(0.6, -0.8, 0, 0, 0, 0);

        var world = LocalFrame.Rotate(local, dir);

        double len = Math.Sqrt(world.X * world.X + world.Y * world.Y + world.Z * world.Z);
        Assert.Equal(1.0, len, 9);
    }

    [Fact]
    public void JoggingAlongLocalAxisMovesLocalCoordinateAccordingly()
    {
        // Consistency between the jog transform and the readout transform: jog by a
        // local-frame direction, apply it to the world position, and the change in
        // local-frame coordinates equals the jog direction.
        var local = new Vector6(120, -60, 15, 0, 0, 37);
        var worldPos = LocalFrame.Apply(local, new Vector6(50, 30, 10, 0, 0, 0));

        var localDir = new Vector6(1, 0, 0);
        var worldDir = LocalFrame.Rotate(local, localDir);
        var movedWorld = new Vector6(
            worldPos.X + worldDir.X * 5, worldPos.Y + worldDir.Y * 5, worldPos.Z + worldDir.Z * 5,
            worldPos.RX, worldPos.RY, worldPos.RZ);

        var localBefore = LocalFrame.Inverse(local, worldPos);
        var localAfter  = LocalFrame.Inverse(local, movedWorld);

        Assert.Equal(5, localAfter.X - localBefore.X, 6);   // moved +5 along local X
        Assert.Equal(0, localAfter.Y - localBefore.Y, 6);
        Assert.Equal(0, localAfter.Z - localBefore.Z, 6);
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
