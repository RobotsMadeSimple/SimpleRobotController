using System.Reflection;
using Controller.RobotControl;
using Controller.RobotControl.MotionProfilers;

namespace RobotControl.Tests;

/// <summary>
/// The jerk-limited S-curve option: continuous acceleration, bounded jerk, sane endpoints and
/// durations, blended paths that stay continuous, the config / per-move selection — and that the
/// trapezoid stays the default (its bit-for-bit output is pinned by <see cref="MotionProfileGoldenTests"/>).
/// </summary>
public class SCurveProfileTests
{
    private const double Dt = 1e-4;

    private static IEnumerable<double> Times(double until, double dt = Dt)
    {
        for (double t = 0; t <= until + 10 * dt; t += dt) yield return t;
    }

    public static TheoryData<double, double, double, double, double> Cases => new()
    {
        // distance, vmax, accel, decel, smoothness %
        { 300, 100, 400, 400, 50 },    // reaches vmax, cruise
        { 20,  200, 300, 300, 50 },    // too short for vmax
        { 150,  80, 500, 120, 30 },    // asymmetric
        { 5,   500, 800, 800, 100 },   // tiny move, pure S
        { 400, 250, 2000, 2000, 1 },   // nearly trapezoid
    };

    private static SCurveProfile Make(double d, double v, double a, double dec, double smooth)
    {
        var s = new MotionProfileSettings(MotionProfileKind.SCurve, smooth);
        return new SCurveProfile(d, v, a, dec, s.JerkFor(v, a), s.JerkFor(v, dec));
    }

    [Theory, MemberData(nameof(Cases))]
    public void Acceleration_is_continuous_and_jerk_is_bounded(double d, double v, double a, double dec, double smooth)
    {
        var p = Make(d, v, a, dec, smooth);
        double prevAcc = 0;
        bool first = true;
        foreach (var t in Times(p.Duration))
        {
            p.Sample(t, out _, out _, out double acc);
            if (!first)
            {
                // A jump in acceleration between two samples Dt apart can be at most J·Dt.
                Assert.True(Math.Abs(acc - prevAcc) <= p.MaxJerk * Dt * 1.0001 + 1e-9,
                    $"accel stepped {acc - prevAcc} at t={t} (jerk limit {p.MaxJerk})");
            }
            Assert.True(acc <= a * (1 + 1e-9) && acc >= -dec * (1 + 1e-9), $"accel {acc} outside [-{dec}, {a}] at t={t}");
            prevAcc = acc;
            first = false;
        }
    }

    [Theory, MemberData(nameof(Cases))]
    public void Endpoints_velocity_and_duration_are_sane(double d, double v, double a, double dec, double smooth)
    {
        var p = Make(d, v, a, dec, smooth);
        p.Sample(0, out double s0, out double v0, out double a0);
        Assert.Equal(0, s0); Assert.Equal(0, v0); Assert.Equal(0, a0);
        p.Sample(p.Duration, out double s1, out double v1, out double a1);
        Assert.Equal(d, s1); Assert.Equal(0, v1); Assert.Equal(0, a1);
        p.Sample(p.Duration + 5, out double s2, out _, out _);
        Assert.Equal(d, s2);

        // Never faster than vmax, never backwards, and the samples integrate to the position.
        double prev = 0, prevVel = 0;
        foreach (var t in Times(p.Duration))
        {
            p.Sample(t, out double s, out double vel, out _);
            Assert.True(vel <= v * (1 + 1e-9) + 1e-9, $"velocity {vel} > {v}");
            Assert.True(s >= prev - 1e-9, $"moved backwards at t={t}");
            Assert.True(s - prev <= Math.Max(vel, prevVel) * Dt + 1e-6, $"position jumped at t={t}");
            prev = s; prevVel = vel;
        }

        // Slower than the trapezoid with the same limits (the ramps are gentler), but not absurdly so.
        double trapT = TrapezoidDuration(d, v, a, dec);
        Assert.True(p.Duration >= trapT - 1e-9, $"S-curve {p.Duration}s faster than trapezoid {trapT}s");
        Assert.True(p.Duration <= trapT * 4 + 0.1, $"S-curve {p.Duration}s vs trapezoid {trapT}s");
        Assert.True(p.PeakVelocity <= v + 1e-9);
    }

    private static double TrapezoidDuration(double d, double v, double a, double dec)
    {
        double ta = v / a, td = v / dec, da = 0.5 * a * ta * ta, dd = 0.5 * dec * td * td;
        if (da + dd > d) { double vp = Math.Sqrt(2 * d * a * dec / (a + dec)); return vp / a + vp / dec; }
        return ta + td + (d - da - dd) / v;
    }

    [Fact]
    public void Full_smoothness_has_no_constant_acceleration_phase()
    {
        var p = Make(1000, 100, 400, 400, 100);
        // At 100 % the jerk is a²/v: acceleration touches the limit at one instant only.
        int atLimit = Times(p.Duration, 1e-3).Count(t => { p.Sample(t, out _, out _, out double acc); return acc > 400 * 0.999; });
        Assert.True(atLimit <= 3, $"{atLimit} samples at the accel limit");
    }

    [Fact]
    public void A_configured_jerk_overrides_the_smoothness()
    {
        var s = new MotionProfileSettings(MotionProfileKind.SCurve, 50, Jerk: 5000);
        Assert.Equal(5000, s.JerkFor(100, 400));
        Assert.Equal(5000, s.JerkFor(1, 1));
        var p = new SCurveProfile(200, 100, 400, 400, 5000, 5000);
        Assert.Equal(5000, p.MaxJerk);
    }

    [Fact]
    public void Vector6_scurve_starts_ends_and_finishes()
    {
        var from = new Vector6(100, 0, 200, 0, 0, 0);
        var to   = new Vector6(300, 150, 120, 0, 0, 90);
        var p = new Vector6MotionProfiler(from, to, 150, 600, 600, new MotionProfileSettings(MotionProfileKind.SCurve));
        var a = p.SampleAt(0);
        Assert.Equal(from.X, a.X); Assert.Equal(from.Y, a.Y); Assert.Equal(from.Z, a.Z);
        Assert.False(p.IsFinished);
        var mid = p.SampleAt(0.5);
        Assert.InRange(mid.X, from.X, to.X);
        var end = p.SampleAt(60);
        Assert.True(p.IsFinished);
        Assert.Equal(to.X, end.X, 9); Assert.Equal(to.Y, end.Y, 9); Assert.Equal(to.Z, end.Z, 9); Assert.Equal(to.RZ, end.RZ, 9);

        // The trapezoid (the default) takes less time with the same limits.
        var trap = new Vector6MotionProfiler(from, to, 150, 600, 600);
        double tTrap = FirstFinished(t => { trap.SampleAt(t); return trap.IsFinished; });
        var sc = new Vector6MotionProfiler(from, to, 150, 600, 600, new MotionProfileSettings(MotionProfileKind.SCurve));
        double tS = FirstFinished(t => { sc.SampleAt(t); return sc.IsFinished; });
        Assert.True(tS > tTrap, $"S-curve {tS}s, trapezoid {tTrap}s");
    }

    private static double FirstFinished(Func<double, bool> finishedAt)
    {
        for (double t = 0; t < 60; t += 1e-3) if (finishedAt(t)) return t;
        return double.PositiveInfinity;
    }

    [Fact]
    public void A_blended_scurve_path_is_continuous_and_lands_on_the_last_waypoint()
    {
        var pts = new List<Vector6> { new(0, 0, 100), new(120, 0, 100), new(120, 90, 100), new(240, 90, 160), new(240, 200, 160) };
        var radii = new List<double> { 0, 15, 25, 10, 0 };
        const double v = 120;
        var p = new ContinuousPathingProfiler(pts, radii, v, 500, 400, new MotionProfileSettings(MotionProfileKind.SCurve, 60));
        Vector6? prev = null;
        double dt = 1e-3, maxStep = 0;
        for (double t = 0; t < 20 && !p.IsFinished; t += dt)
        {
            var q = p.SampleAt(t);
            if (prev != null) maxStep = Math.Max(maxStep, q.Distance3(prev));
            prev = q;
        }
        Assert.True(p.IsFinished);
        Assert.True(maxStep <= v * dt * 1.01 + 1e-9, $"largest step {maxStep} mm in {dt}s");
        var end = p.SampleAt(30);
        Assert.Equal(240, end.X, 9); Assert.Equal(200, end.Y, 9); Assert.Equal(160, end.Z, 9);

        // The geometry is the trapezoid's: both pass the same corner arcs, only the timing differs.
        var trap = new ContinuousPathingProfiler(pts, radii, v, 500, 400);
        Assert.Equal(trap.TotalLength, p.TotalLength, 9);
    }

    [Fact]
    public void Degenerate_moves_finish_at_once()
    {
        var p = new SCurveProfile(0, 100, 100, 100, 1000, 1000);
        Assert.Equal(0, p.Duration);
        Assert.Equal(0, p.Position(1));
        var v = new Vector6MotionProfiler(new Vector6(1, 2, 3), new Vector6(1, 2, 3), 100, 100, 100, new MotionProfileSettings(MotionProfileKind.SCurve));
        Assert.True(v.IsFinished);
    }

    // ── Selection ─────────────────────────────────────────────────────────

    [Fact]
    public void Trapezoid_is_the_default_and_config_and_step_select_the_scurve()
    {
        var robot = new RobotController();
        Assert.Equal("trapezoid", new RobotConfig().MotionProfile);
        Assert.False(robot.ResolveProfile(null).IsSCurve);
        Assert.True(robot.ResolveProfile("scurve").IsSCurve);             // per-move override

        robot.SetConfig(new RobotConfig { MotionProfile = "scurve", ScurveSmoothness = 70, ScurveJerk = 9000 });
        var s = robot.ResolveProfile(null);
        Assert.True(s.IsSCurve);
        Assert.Equal(70, s.SmoothnessPercent);
        Assert.Equal(9000, s.Jerk);
        Assert.False(robot.ResolveProfile("trapezoid").IsSCurve);          // the move wins
        Assert.False(robot.ResolveProfile("TRAPEZOID").IsSCurve);
        Assert.True(robot.ResolveProfile("bogus").IsSCurve);               // unknown → the config's
    }

    [Fact]
    public void Homing_joint_moves_keep_the_trapezoid_even_with_scurve_configured()
    {
        var robot = new RobotController();
        robot.SetConfig(new RobotConfig { MotionProfile = "scurve" });
        ((IHomingHost)robot).MoveJoints(new Vector6(0, 0, 0, 0, 0, 30), 20, 100, 200);
        var prof = typeof(RobotController).GetField("jointMotionProfiler", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(robot)!;
        var scurve = typeof(Vector6MotionProfiler).GetField("scurve", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(prof);
        Assert.Null(scurve);

        // …while a program/point joint move does take it.
        var robot2 = new RobotController();
        robot2.SetConfig(new RobotConfig { MotionProfile = "scurve" });
        robot2.MoveJoints(new Vector6(10, 0, 0, 0, 0, 0), 20, 100, 100);
        var prof2 = typeof(RobotController).GetField("jointMotionProfiler", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(robot2)!;
        Assert.NotNull(typeof(Vector6MotionProfiler).GetField("scurve", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(prof2));
    }
}
