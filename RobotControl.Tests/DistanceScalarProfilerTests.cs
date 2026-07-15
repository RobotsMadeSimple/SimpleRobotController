using Controller.RobotControl.MotionProfilers;

namespace RobotControl.Tests;

public class DistanceScalarProfilerTests
{
    private static DistanceScalarProfiler Make(double dist, double v, double a, double d)
    {
        var p = new DistanceScalarProfiler();
        p.Setup(dist, v, a, d);
        return p;
    }

    [Fact]
    public void StartsAtZero()
    {
        var p = Make(100, 50, 100, 100);
        Assert.Equal(0, p.Sample(0), 9);
        Assert.Equal(0, p.Sample(-1), 9);
    }

    [Fact]
    public void EndsExactlyAtDistance()
    {
        var p = Make(100, 50, 100, 100);
        Assert.Equal(100, p.Sample(1e6), 9);
    }

    [Theory]
    [InlineData(100, 50, 100, 100)]   // trapezoidal, symmetric
    [InlineData(100, 50, 200, 80)]    // trapezoidal, asymmetric
    [InlineData(5, 50, 100, 100)]     // triangular (too short to reach v), symmetric
    [InlineData(5, 50, 200, 80)]      // triangular, asymmetric
    public void PositionIsMonotonicAndContinuous(double dist, double v, double a, double d)
    {
        var p = Make(dist, v, a, d);

        const double dt = 0.001;
        double prev = p.Sample(0);
        // Max distance per step at commanded speed, with slack for float noise.
        double maxStep = v * dt * 1.10 + 1e-9;

        for (double t = dt; t < 30; t += dt)
        {
            double pos = p.Sample(t);

            // Never moves backwards
            Assert.True(pos >= prev - 1e-9, $"position decreased at t={t}: {prev} -> {pos}");

            // No teleporting — this is the assertion that catches a broken decel
            // formula (position jumping to full distance when decel begins).
            Assert.True(pos - prev <= maxStep, $"position jumped at t={t}: {prev} -> {pos}");

            prev = pos;
            if (pos >= dist) break;
        }

        Assert.Equal(dist, prev, 6);
    }

    [Fact]
    public void CruisePhaseRunsAtCommandedSpeed()
    {
        // Long move: accel takes v/a = 0.5s covering 12.5mm; pick a mid-cruise window.
        var p = Make(1000, 50, 100, 100);
        double t0 = 5.0, t1 = 6.0;
        double measured = p.Sample(t1) - p.Sample(t0);
        Assert.InRange(measured, 50 * 0.98, 50 * 1.02);
    }

    [Fact]
    public void TriangularProfileNeverOvershoots()
    {
        var p = Make(2, 100, 50, 50);
        for (double t = 0; t < 10; t += 0.0005)
            Assert.True(p.Sample(t) <= 2 + 1e-9);
        Assert.Equal(2, p.Sample(10), 6);
    }
}
