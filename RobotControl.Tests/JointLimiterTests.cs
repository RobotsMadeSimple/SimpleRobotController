using Controller.RobotControl;

namespace RobotControl.Tests;

public class JointLimiterTests
{
    // Wide windows on the three joints we aren't testing so only the joint under
    // test can trip.
    private static (double lo, double hi)[] Windows(
        (double lo, double hi) j1,
        (double lo, double hi)? j2 = null,
        (double lo, double hi)? j3 = null,
        (double lo, double hi)? j4 = null)
        => new[]
        {
            j1,
            j2 ?? (-1e9, 1e9),
            j3 ?? (-1e9, 1e9),
            j4 ?? (-1e9, 1e9),
        };

    [Fact]
    public void InRangeMovePassesThroughUntouched()
    {
        var before = new Vector6(10, 100, 100, 0, 0, 10);
        var target = new Vector6(20, 120, 110, 0, 0, 15);

        var r = JointLimiter.Clamp(target, before, Windows((-180, 180), (0, 600), (0, 600), (-180, 180)));

        Assert.False(r.Violated);
        Assert.Equal(20, r.Clamped.X, 9);
        Assert.Equal(15, r.Clamped.RZ, 9);
    }

    [Fact]
    public void CrossingUpperLimitClampsAndReportsJointAndDirection()
    {
        var before = new Vector6(170, 0, 0, 0, 0, 0);
        var target = new Vector6(190, 0, 0, 0, 0, 0);  // J1 past +180

        var r = JointLimiter.Clamp(target, before, Windows((-180, 180)));

        Assert.True(r.Violated);
        Assert.Equal(0, r.Joint);       // J1 = component 0
        Assert.Equal(1, r.Direction);   // past the max
        Assert.Equal(180, r.Clamped.X, 9);
    }

    [Fact]
    public void CrossingLowerLimitReportsNegativeDirection()
    {
        var before = new Vector6(0, 20, 0, 0, 0, 0);
        var target = new Vector6(0, -10, 0, 0, 0, 0);  // radial (J2) below 0

        var r = JointLimiter.Clamp(target, before, Windows((-180, 180), (0, 600)));

        Assert.True(r.Violated);
        Assert.Equal(1, r.Joint);       // radial = component 1
        Assert.Equal(-1, r.Direction);
        Assert.Equal(0, r.Clamped.Y, 9);
    }

    [Fact]
    public void VerticalMapsToComponentTwo()
    {
        var before = new Vector6(0, 0, 590, 0, 0, 0);
        var target = new Vector6(0, 0, 650, 0, 0, 0);  // vertical (J3) above 600

        var r = JointLimiter.Clamp(target, before, Windows((-180, 180), (0, 600), (0, 600)));

        Assert.True(r.Violated);
        Assert.Equal(2, r.Joint);       // vertical = component 2
        Assert.Equal(1, r.Direction);
        Assert.Equal(600, r.Clamped.Z, 9);
    }

    [Fact]
    public void AlreadyPastLimitAllowsCorrectiveMotion()
    {
        // Joint sits 5° beyond max (e.g. limits tightened while outside).
        var before = new Vector6(185, 0, 0, 0, 0, 0);
        var target = new Vector6(183, 0, 0, 0, 0, 0);  // moving back toward range

        var r = JointLimiter.Clamp(target, before, Windows((-180, 180)));

        Assert.False(r.Violated);       // corrective motion is allowed
        Assert.Equal(183, r.Clamped.X, 9);
    }

    [Fact]
    public void AlreadyPastLimitBlocksFurtherOutwardMotion()
    {
        var before = new Vector6(185, 0, 0, 0, 0, 0);
        var target = new Vector6(188, 0, 0, 0, 0, 0);  // driving further past

        var r = JointLimiter.Clamp(target, before, Windows((-180, 180)));

        Assert.True(r.Violated);
        Assert.Equal(1, r.Direction);
        Assert.Equal(185, r.Clamped.X, 9);  // held at where it already was, no further
    }

    [Fact]
    public void ReportsTheFirstViolatedJointOnly()
    {
        var before = new Vector6(170, 20, 0, 0, 0, 0);
        var target = new Vector6(190, -10, 0, 0, 0, 0);  // both J1 and radial cross

        var r = JointLimiter.Clamp(target, before, Windows((-180, 180), (0, 600)));

        Assert.True(r.Violated);
        Assert.Equal(0, r.Joint);       // first component wins
        // both are still clamped in the output
        Assert.Equal(180, r.Clamped.X, 9);
        Assert.Equal(0, r.Clamped.Y, 9);
    }

    [Fact]
    public void UnsetBoundIsNotEnforced()
    {
        // An unset bound is represented as ±infinity — motion in that direction is
        // never clamped, while a bound that IS set on the other side still holds.
        var before = new Vector6(0, 20, 0, 0, 0, 0);
        var target = new Vector6(9999, 20, 0, 0, 0, 0); // huge J1 move, J1 max unset

        var r = JointLimiter.Clamp(target, before, Windows(
            (double.NegativeInfinity, double.PositiveInfinity),  // J1 fully unset
            (0, double.PositiveInfinity)));                      // radial min only

        Assert.False(r.Violated);
        Assert.Equal(9999, r.Clamped.X, 6);
    }

    [Fact]
    public void OneSidedWindowEnforcesOnlyTheSetBound()
    {
        var before = new Vector6(0, 10, 0, 0, 0, 0);
        var target = new Vector6(0, -5, 0, 0, 0, 0);  // radial below its min

        // radial: min 0 set, max unset (+inf)
        var r = JointLimiter.Clamp(target, before, Windows(
            (double.NegativeInfinity, double.PositiveInfinity),
            (0, double.PositiveInfinity)));

        Assert.True(r.Violated);
        Assert.Equal(1, r.Joint);
        Assert.Equal(-1, r.Direction);
        Assert.Equal(0, r.Clamped.Y, 9);
    }

    [Fact]
    public void SwappedLoHiWindowStillClampsCorrectly()
    {
        // Defensive: min/max passed in reversed order.
        var before = new Vector6(0, 0, 0, 0, 0, 0);
        var target = new Vector6(200, 0, 0, 0, 0, 0);

        var r = JointLimiter.Clamp(target, before, Windows((180, -180)));

        Assert.True(r.Violated);
        Assert.Equal(180, r.Clamped.X, 9);
    }
}
