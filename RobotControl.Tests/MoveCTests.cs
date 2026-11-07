using Controller.RobotControl;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

/// <summary>
/// MoveC (Move Arc, docs/program-language.md §4.3): the circle through start, via and end in 3D,
/// its tessellation (chord error within tolerance, at most 2° a segment, RZ by arc angle), the
/// degenerate cases, and the step driven through the executor down the blended-path dispatch.
/// </summary>
[Collection("RobotController")]
public sealed class MoveCTests : IDisposable
{
    private readonly StepsRig _rig = new();
    public void Dispose() => _rig.Dispose();

    private static Vector6 V(double x, double y, double z, double rz = 0) => new(x, y, z, 0, 0, rz);

    private static double Dist(Vector6 a, double x, double y, double z) =>
        Math.Sqrt((a.X - x) * (a.X - x) + (a.Y - y) * (a.Y - y) + (a.Z - z) * (a.Z - z));

    // ── Geometry ──────────────────────────────────────────────────────────────

    [Fact]
    public void CircleThroughThreePointsInXY()
    {
        Assert.True(ArcPath.TryCircle(V(10, 0, 5), V(0, 10, 5), V(-10, 0, 5), out var c, out _));
        Assert.Equal(0, c.CX, 9); Assert.Equal(0, c.CY, 9); Assert.Equal(5, c.CZ, 9);
        Assert.Equal(10, c.Radius, 9);
        Assert.Equal(Math.PI, c.Sweep, 9);          // a half circle, counter-clockwise through +Y
        Assert.Equal(Math.PI / 2, c.ViaAngle, 9);
    }

    [Fact]
    public void ViaDecidesTheWayRound()
    {
        // Same start and end, via on the far side: three quarters of the circle, the long way.
        Assert.True(ArcPath.TryCircle(V(10, 0, 0), V(-10, 0, 0), V(0, 10, 0), out var c, out _));
        Assert.Equal(1.5 * Math.PI, c.Sweep, 9);
        Assert.True(c.ViaAngle < c.Sweep);
        // Clockwise in XY reads as counter-clockwise about -Z: still a quarter turn.
        Assert.True(ArcPath.TryCircle(V(10, 0, 0), V(7.0710678118654755, -7.0710678118654755, 0), V(0, -10, 0), out var cw, out _));
        Assert.Equal(Math.PI / 2, cw.Sweep, 6);
    }

    [Fact]
    public void TessellationStaysOnATiltedCircle()
    {
        // A circle of radius 50 about (100, 20, 30) in a plane tilted 35° about X.
        double r = 50, tilt = 35 * Math.PI / 180;
        Vector6 On(double deg)
        {
            double a = deg * Math.PI / 180;
            double lx = r * Math.Cos(a), ly = r * Math.Sin(a);
            return V(100 + lx, 20 + ly * Math.Cos(tilt), 30 + ly * Math.Sin(tilt));
        }
        var start = On(-30); var via = On(40); var end = On(120);
        Assert.True(ArcPath.TryTessellate(start, via, end, 0.05, out var wps, out var radii, out _));

        Assert.True(ArcPath.TryCircle(start, via, end, out var c, out _));
        Assert.Equal(r, c.Radius, 6);
        Assert.Equal(150 * Math.PI / 180, c.Sweep, 6);

        // Every waypoint on the circle and in its plane (normal = (0, -sin, cos)).
        foreach (var w in wps)
        {
            Assert.Equal(r, Dist(w, 100, 20, 30), 6);
            double planeOffset = (w.Y - 20) * -Math.Sin(tilt) + (w.Z - 30) * Math.Cos(tilt);
            Assert.Equal(0, planeOffset, 6);
        }
        Assert.Equal(end, wps[^1]);
        Assert.Equal(wps.Count, radii.Count);
    }

    [Theory]
    [InlineData(100, 0.1)]
    [InlineData(100, 0.001)]
    [InlineData(5, 0.1)]
    [InlineData(500, 0.01)]
    [InlineData(2, 5)]       // tolerance larger than the circle: the 2° cap decides
    public void ChordErrorIsWithinToleranceAndSegmentsAtMostTwoDegrees(double r, double tol)
    {
        var start = V(r, 0, 0); var via = V(0, r, 0); var end = V(-r, 0, 0);
        Assert.True(ArcPath.TryTessellate(start, via, end, tol, out var wps, out _, out _));
        var prev = start;
        foreach (var w in wps)
        {
            // Sagitta of this chord: radius minus the distance from the centre to its midpoint.
            double mx = (prev.X + w.X) / 2, my = (prev.Y + w.Y) / 2, mz = (prev.Z + w.Z) / 2;
            double sagitta = r - Math.Sqrt(mx * mx + my * my + mz * mz);
            Assert.True(sagitta <= Math.Max(tol, ArcPath.MinTolerance) + 1e-9, $"sagitta {sagitta} > {tol}");
            double chord = Math.Sqrt((w.X - prev.X) * (w.X - prev.X) + (w.Y - prev.Y) * (w.Y - prev.Y));
            double angle = 2 * Math.Asin(Math.Min(1, chord / (2 * r))) * 180 / Math.PI;
            Assert.True(angle <= ArcPath.MaxSegmentDeg + 1e-9, $"segment {angle}° > 2°");
            prev = w;
        }
    }

    [Theory]
    [InlineData(100, Math.PI, 0.1, 90)]     // 2° cap: 180° / 2°
    [InlineData(100, Math.PI, 0.001, 352)]  // tolerance decides: θ = 2·acos(1 − 1e-5)
    [InlineData(100, 0.001, 0.1, 2)]        // never fewer than two
    [InlineData(1e6, 2 * Math.PI, 0.001, ArcPath.MaxSegments)] // capped
    public void SegmentCount(double r, double sweep, double tol, int expected) =>
        Assert.Equal(expected, ArcPath.SegmentCount(r, sweep, tol));

    [Fact]
    public void ToleranceBelowTheMinimumIsRaised() =>
        Assert.Equal(ArcPath.SegmentCount(100, Math.PI, ArcPath.MinTolerance), ArcPath.SegmentCount(100, Math.PI, 1e-9));

    [Fact]
    public void OrientationGoesLinearlyByArcAngleAndViaOrientationIsIgnored()
    {
        Assert.True(ArcPath.TryTessellate(V(10, 0, 0, rz: 0), V(0, 10, 0, rz: 500), V(-10, 0, 0, rz: 90), 0.1,
            out var wps, out var radii, out _));
        int n = wps.Count;
        for (int k = 0; k < n; k++) Assert.Equal(90.0 * (k + 1) / n, wps[k].RZ, 9);
        // The blend radius is the arc radius at every interior waypoint; the end is an exact stop.
        Assert.All(radii.Take(n - 1), x => Assert.Equal(10, x, 9));
        Assert.Equal(0, radii[^1]);
    }

    [Theory]
    [InlineData(0, 0, 0, 10, 0, 0, 20, 0, 0, "in a line")]
    [InlineData(0, 0, 0, 10, 10, 10, 30, 30, 30, "in a line")]
    [InlineData(0, 0, 0, 0, 0, 0, 20, 5, 0, "via point is the start")]
    [InlineData(0, 0, 0, 20, 5, 0, 20, 5, 0, "via point is the end")]
    [InlineData(0, 0, 0, 20, 5, 0, 0, 0, 0, "end point is the start")]
    public void DegenerateArcsAreRefused(double sx, double sy, double sz, double vx, double vy, double vz,
                                         double ex, double ey, double ez, string why)
    {
        Assert.False(ArcPath.TryTessellate(V(sx, sy, sz), V(vx, vy, vz), V(ex, ey, ez), 0.1, out var wps, out _, out var error));
        Assert.Contains(why, error);
        Assert.Empty(wps);
    }

    // ── Through the executor ──────────────────────────────────────────────────

    /// <summary>Saves the via and end points of a half circle of radius 40 that starts at the
    /// robot's live position and bulges towards +Y; returns their ids.</summary>
    private (string ViaId, string EndId, Vector6 Start) HalfCircle(double rz = 0)
    {
        var s = _rig.Robot.GetCurrentPosition();
        var via = _rig.Points.SavePoint("ArcVia", new Vector6(s.X + 40, s.Y + 40, s.Z, s.RX, s.RY, s.RZ));
        var end = _rig.Points.SavePoint("ArcEnd", new Vector6(s.X + 80, s.Y, s.Z, s.RX, s.RY, s.RZ + rz));
        return (via.Id!, end.Id!, s);
    }

    private static ProgramStep MoveC(string viaId, string endId, Action<ProgramStep>? init = null) =>
        StepsRig.Step(StepType.MoveC, s =>
        {
            s.ViaPointId = viaId; s.ViaPointName = "ArcVia";
            s.PointId    = endId; s.PointName    = "ArcEnd";
            init?.Invoke(s);
        });

    private RobotCommand Continuous() => Assert.Single(_rig.Dispatched, c => c.CommandType == "StartContinuous");

    [Fact]
    public void MoveCDispatchesOneBlendedPathAlongTheArc()
    {
        var (via, end, s) = HalfCircle(rz: 30);
        _rig.Run(StepsRig.Prog(MoveC(via, end, st => { st.Speed = 25; st.Accel = 300; st.ArcTolerance = 0.05; })));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Summary().GetProperty("currentStepNumber").GetInt32());

        var cmd = Continuous();
        Assert.True(cmd.ApplySpeedOverride);
        Assert.Equal(25, cmd.Speed);
        Assert.Equal(300, cmd.Accel);
        var wps = cmd.Waypoints!;
        Assert.Equal(90, wps.Count);                 // 180° at the 2° cap (0.05 mm allows ~5.7° at r = 40)
        Assert.Equal(wps.Count, cmd.BlendRadii!.Count);
        foreach (var w in wps) Assert.Equal(40, Dist(w, s.X + 40, s.Y, s.Z), 6);
        Assert.Equal(s.X + 80, wps[^1].X, 9);
        Assert.Equal(s.RZ + 30, wps[^1].RZ, 9);
        Assert.Equal(s.Y + 40, wps[wps.Count / 2 - 1].Y, 6); // the middle of the arc is the via point
    }

    [Fact]
    public void OffsetsApplyToBothViaAndEnd()
    {
        var (via, end, s) = HalfCircle();
        _rig.Run(StepsRig.Prog(MoveC(via, end, st => st.OffsetY = 10)));
        _rig.AssertComplete();
        var wps = Continuous().Waypoints!;
        Assert.Equal(s.Y + 10, wps[^1].Y, 9);
        // Start (unshifted), via + 10 and end + 10 still lie on one circle the waypoints follow.
        Assert.True(ArcPath.TryCircle(s, new Vector6(s.X + 40, s.Y + 50, s.Z, 0, 0, 0), wps[^1], out var c, out _));
        foreach (var w in wps) Assert.Equal(c.Radius, Dist(w, c.CX, c.CY, c.CZ), 6);
    }

    [Fact]
    public void ViaCanBeATemplateNamingAPoint()
    {
        var (_, end, s) = HalfCircle();
        _rig.Points.SavePoint("Via2", new Vector6(s.X + 40, s.Y - 40, s.Z, 0, 0, 0));
        var step = StepsRig.Step(StepType.MoveC, st => { st.ViaPointNameExpr = "Via{$n}"; st.PointId = end; });
        _rig.Run(StepsRig.Prog([StepsRig.Num("n", 2)], step));
        _rig.AssertComplete();
        Assert.Equal(s.Y - 40, Continuous().Waypoints![44].Y, 6); // bulges towards -Y this time
    }

    [Fact]
    public void ViaCanBeAPointsListElement()
    {
        var (_, end, s) = HalfCircle();
        var pts = StepsRig.PointList("pts", StepsRig.P(0, 0, 0), StepsRig.P(s.X + 40, s.Y + 40, s.Z));
        _rig.Run(StepsRig.Prog([pts], StepsRig.Step(StepType.MoveC, st => { st.ViaPointNameExpr = "$pts[1]"; st.PointId = end; })));
        _rig.AssertComplete();
        Assert.Equal(90, Continuous().Waypoints!.Count);
    }

    [Fact]
    public void CollinearPointsFailArcDegenerate()
    {
        var s = _rig.Robot.GetCurrentPosition();
        var via = _rig.Points.SavePoint("Mid", new Vector6(s.X + 10, s.Y, s.Z, 0, 0, 0));
        var end = _rig.Points.SavePoint("Far", new Vector6(s.X + 20, s.Y, s.Z, 0, 0, 0));
        _rig.Run(StepsRig.Prog(StepsRig.Step(StepType.MoveC, st => { st.ViaPointId = via.Id; st.PointId = end.Id; }),
                               StepsRig.Set("after", "1")));
        _rig.AssertError("in a line");
        Assert.DoesNotContain(_rig.Dispatched, c => c.CommandType == "StartContinuous");
    }

    [Fact]
    public void MissingViaEndsTheRun()
    {
        var (_, end, _) = HalfCircle();
        _rig.Run(StepsRig.Prog(StepsRig.Step(StepType.MoveC, st => st.PointId = end)));
        _rig.AssertError("no via point");
    }

    [Fact]
    public void DeletedViaPointFailsPointNotFound()
    {
        var (via, end, _) = HalfCircle();
        _rig.Points.DeletePoint("ArcVia");
        _rig.Run(StepsRig.Prog(StepsRig.Step(StepType.MoveC, st => { st.ViaPointId = via; st.PointId = end; })));
        _rig.AssertError("via point no longer exists");
    }

    [Fact]
    public void UnknownViaNameFailsWithTheResolverMessage()
    {
        var (_, end, _) = HalfCircle();
        _rig.Run(StepsRig.Prog(StepsRig.Step(StepType.MoveC, st => { st.ViaPointName = "Nowhere"; st.PointId = end; })));
        _rig.AssertError("Nowhere");
    }

    [Fact]
    public void ZeroOrMissingToleranceUsesTheDefault()
    {
        var (via, end, _) = HalfCircle();
        _rig.Run(StepsRig.Prog(MoveC(via, end, st => st.ArcTolerance = 0)));
        _rig.AssertComplete();
        Assert.Equal(ArcPath.SegmentCount(40, Math.PI, ArcPath.DefaultTolerance), Continuous().Waypoints!.Count);
    }
}
