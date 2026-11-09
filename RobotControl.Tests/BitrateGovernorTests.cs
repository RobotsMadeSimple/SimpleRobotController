using Controller.RobotControl.Camera.WebRtc;

namespace RobotControl.Tests;

/// <summary>The receiver-side bitrate cap the phone applies: lower it when the link overruns, leave it
/// alone for loss that resends fix, raise it again when things stay clean.</summary>
public sealed class BitrateGovernorTests
{
    [Fact]
    public void NoCapWhileClean()
    {
        var g = new BitrateGovernor();
        for (long t = 0; t < 20_000; t += 1000) Assert.False(g.Update(t, 16000, 1500, 3, 0, 0));
        Assert.Null(g.CapKbps);
    }

    [Fact]
    public void RandomLossThatResendsFixIsNotCongestion()
    {
        var g = new BitrateGovernor();
        for (long t = 0; t < 20_000; t += 1000) g.Update(t, 16000, 1500, 30, 0, 0);   // 2 % missing, all resent
        Assert.Null(g.CapKbps);
    }

    [Fact]
    public void UnrecoveredLossLowersTheCapBelowWhatArrives()
    {
        var g = new BitrateGovernor();
        Assert.True(g.Update(1000, 10000, 900, 20, 5, 0));
        Assert.Equal(8000, g.CapKbps);
        Assert.False(g.Update(1500, 9000, 900, 20, 5, 0));   // at most one cut a second
        Assert.True(g.Update(2000, 9000, 900, 20, 5, 0));
        Assert.Equal(6400, g.CapKbps);                        // from the cap, not the higher arriving rate
        Assert.Equal(2, g.Decreases);
    }

    [Fact]
    public void HeavyMissingOrDecoderOverflowAlsoCount()
    {
        var a = new BitrateGovernor();
        Assert.True(a.Update(1000, 10000, 900, 100, 0, 0));   // 10 % went missing, even if resent
        var b = new BitrateGovernor();
        Assert.True(b.Update(1000, 10000, 900, 0, 0, 1));
        Assert.NotNull(b.CapKbps);
    }

    [Fact]
    public void ACutIsBoundedAndNeverBelowTheFloor()
    {
        var g = new BitrateGovernor();
        g.Update(1000, 10000, 900, 0, 5, 0);
        g.Update(2000, 500, 50, 0, 5, 0);                     // arriving rate collapsed while choking
        Assert.Equal(4000, g.CapKbps);                         // half the old cap, not 0.8 x 500
        for (long t = 3000; t < 20_000; t += 1000) g.Update(t, 100, 10, 0, 5, 0);
        Assert.Equal(BitrateGovernor.FloorKbps, g.CapKbps);
    }

    [Fact]
    public void RaisesAfterAQuietSpellThenLiftsTheCap()
    {
        var g = new BitrateGovernor();
        g.Update(0, 10000, 900, 0, 5, 0);
        Assert.Equal(8000, g.CapKbps);
        long t = 1000;
        for (; t < 1000 + BitrateGovernor.CleanBeforeRaiseMs; t += 1000) Assert.False(g.Update(t, 7900, 800, 0, 0, 0));
        Assert.True(g.Update(t, 7900, 800, 0, 0, 0));
        Assert.Equal(8640, g.CapKbps);
        // The phone stays at 4 Mbps (its own setting): once the cap is far above that, it goes.
        for (t += 1000; t < 120_000 && g.CapKbps != null; t += 1000) g.Update(t, 4000, 400, 0, 0, 0);
        Assert.Null(g.CapKbps);
    }

    [Fact]
    public void NothingFlowingChangesNothing()
    {
        var g = new BitrateGovernor();
        g.Update(0, 10000, 900, 0, 5, 0);
        int? cap = g.CapKbps;
        Assert.False(g.Update(5000, 0, 0, 0, 0, 0));
        Assert.Equal(cap, g.CapKbps);
    }
}
