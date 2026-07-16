using System.Reflection;
using Controller.RobotControl.MotionProfilers;

namespace RobotControl.Tests;

public class ContinuousPathingProfilerTests
{
    private static ContinuousPathingProfiler Make(List<Vector6> pts, List<double> radii)
        => new(pts, radii, speed: 100, accel: 200, decel: 200);

    /// <summary>Reads the profiler's private segment list via reflection.</summary>
    private static List<PathSegment> Segments(ContinuousPathingProfiler p)
    {
        var f = typeof(ContinuousPathingProfiler).GetField("segments", BindingFlags.NonPublic | BindingFlags.Instance)!;
        return (List<PathSegment>)f.GetValue(p)!;
    }

    private static Vector6 V(double x, double y, double z) => new(x, y, z, 0, 0, 0);

    private static void AssertNear(Vector6 a, Vector6 b, double tol = 1e-6)
    {
        Assert.True(Math.Abs(a.X - b.X) < tol && Math.Abs(a.Y - b.Y) < tol && Math.Abs(a.Z - b.Z) < tol,
            $"expected ({b.X},{b.Y},{b.Z}) but got ({a.X},{a.Y},{a.Z})");
    }

    [Fact]
    public void StraightLineTotalEqualsChord()
    {
        var p = Make([V(0, 0, 0), V(30, 40, 0)], [0, 0]);
        Assert.Equal(50, p.TotalLength, 9);
    }

    [Fact]
    public void CollinearCornerDegradesToLines()
    {
        var p = Make([V(0, 0, 0), V(50, 0, 0), V(100, 0, 0)], [0, 20, 0]);
        Assert.Equal(100, p.TotalLength, 6);
        Assert.All(Segments(p), s => Assert.IsType<LineSegment>(s));
    }

    [Fact]
    public void ZeroRadiusPassesThroughCorner()
    {
        var p = Make([V(0, 0, 0), V(50, 0, 0), V(50, 50, 0)], [0, 0, 0]);
        var segs = Segments(p);
        Assert.Equal(2, segs.Count);
        AssertNear(segs[0].Sample(segs[0].Length), V(50, 0, 0));
    }

    [Fact]
    public void RightAngleBlendShortensPathAndInsertsArc()
    {
        var p = Make([V(0, 0, 0), V(100, 0, 0), V(100, 100, 0)], [0, 20, 0]);
        Assert.True(p.TotalLength < 200, $"blended path ({p.TotalLength}) should be shorter than the sharp corner (200)");
        Assert.Contains(Segments(p), s => s is ArcSegment);
    }

    [Theory]
    [InlineData(20)]   // comfortable radius
    [InlineData(500)]  // radius far larger than the legs — must clamp, not overrun
    public void PathIsC0ContinuousAndHitsEndpoints(double radius)
    {
        var pts = new List<Vector6> { V(0, 0, 0), V(0, 0, 40), V(150, 0, 40), V(150, 0, 0) }; // jump-like arch
        var p = Make(pts, [0, radius, radius, 0]);
        var segs = Segments(p);

        // Starts at the first waypoint, ends exactly on the last
        AssertNear(segs[0].Sample(0), pts[0]);
        AssertNear(segs[^1].Sample(segs[^1].Length), pts[^1]);

        // C0 continuity at every junction — end of each segment == start of the next
        for (int i = 0; i + 1 < segs.Count; i++)
            AssertNear(segs[i].Sample(segs[i].Length), segs[i + 1].Sample(0), 1e-6);

        // Nothing overruns the waypoint bounding box (trim clamp holds)
        foreach (var s in segs)
            for (double t = 0; t <= s.Length; t += Math.Max(s.Length / 20, 1e-3))
            {
                var pt = s.Sample(t);
                Assert.InRange(pt.X, -1e-6, 150 + 1e-6);
                Assert.InRange(pt.Z, -1e-6, 40 + 1e-6);
            }
    }

    [Fact]
    public void ReversalCornerDegradesToLines()
    {
        // Out and straight back — 180° corner has no valid arc
        var p = Make([V(0, 0, 0), V(100, 0, 0), V(0, 0, 0)], [0, 25, 0]);
        Assert.Equal(200, p.TotalLength, 6);
        Assert.All(Segments(p), s => Assert.IsType<LineSegment>(s));
    }

    [Fact]
    public void SampledPathIsContinuousAlongItsLength()
    {
        var p = Make([V(0, 0, 0), V(80, 0, 0), V(80, 80, 0), V(0, 80, 0)], [0, 15, 15, 0]);
        var segs = Segments(p);

        // Walk the whole path by arc length; consecutive points must be close.
        Vector6? prev = null;
        const double step = 0.5;
        foreach (var s in segs)
            for (double t = 0; t <= s.Length; t += step)
            {
                var pt = s.Sample(t);
                if (prev != null)
                {
                    double d = Math.Sqrt(
                        (pt.X - prev.X) * (pt.X - prev.X) +
                        (pt.Y - prev.Y) * (pt.Y - prev.Y) +
                        (pt.Z - prev.Z) * (pt.Z - prev.Z));
                    Assert.True(d <= step * 1.5 + 1e-6, $"gap of {d} in sampled path");
                }
                prev = pt;
            }
    }
}
