using Controller.RobotControl.MotionProfilers;

namespace RobotControl.Tests;

public class ScalarMotionProfilerTests
{
    private static ScalarMotionProfiler Make(double v, double a, double d, double v0 = 0)
    {
        var p = new ScalarMotionProfiler();
        p.Setup(v, a, d, v0);
        return p;
    }

    [Fact]
    public void StartsAtZeroEndsAtOne()
    {
        var p = Make(2, 4, 4);
        Assert.Equal(0, p.Calc(0, out _), 9);
        Assert.Equal(1, p.Calc(1e6, out double vEnd), 9);
        Assert.Equal(0, vEnd, 9);
        Assert.True(p.IsFinished);
    }

    [Theory]
    [InlineData(2, 4, 4, 0)]      // trapezoidal (normalized dist = 1)
    [InlineData(0.5, 4, 2, 0)]    // asymmetric
    [InlineData(10, 4, 4, 0)]     // triangular — can't reach v over dist 1
    [InlineData(2, 4, 4, 1)]      // non-zero start velocity
    public void PositionIsMonotonicAndContinuous(double v, double a, double d, double v0)
    {
        var p = Make(v, a, d, v0);

        const double dt = 0.0005;
        double prev = p.Calc(0, out _);
        // Peak velocity can't exceed max(v, v0); allow slack.
        double maxStep = Math.Max(v, v0) * dt * 1.10 + 1e-9;

        for (double t = dt; t < 20; t += dt)
        {
            double pos = p.Calc(t, out double vel);
            Assert.True(pos >= prev - 1e-9, $"position decreased at t={t}");
            Assert.True(pos - prev <= maxStep, $"position jumped at t={t}: {prev} -> {pos}");
            Assert.True(vel >= -1e-9, $"velocity went negative at t={t}: {vel}");
            prev = pos;
            if (pos >= 1) break;
        }

        Assert.Equal(1, prev, 5);
    }

    [Fact]
    public void DecelOnlyProfileWhenStartVelocityCannotStopInTime()
    {
        // v0² / (2·d) >= 1 → decel-only branch. v0=2, d=1 → stop dist 2 >= 1.
        var p = Make(5, 10, 1, 2);
        // Position advances immediately at ~v0
        double early = p.Calc(0.01, out double vel);
        Assert.InRange(early, 0.01, 0.03);
        Assert.InRange(vel, 1.9, 2.0);
        Assert.Equal(1, p.Calc(1e6, out _), 9);
    }

    [Fact]
    public void DegenerateInputsFinishImmediately()
    {
        var p = Make(0, 4, 4);
        Assert.True(p.IsFinished);
    }
}
