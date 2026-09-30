using System.Linq;
using Controller.RobotControl;
using Controller.RobotControl.Gcode;

namespace RobotControl.Tests;

/// <summary>The G-code interpreter (parse → ops) and the ops → ProgramStep converter.</summary>
public class GcodeInterpreterTests
{
    private static GcodeOp Single(GcodeInterpreter g, string line)
    {
        var ops = g.Feed(line);
        Assert.Single(ops);
        return ops[0];
    }

    // ── Linear / units / modes ─────────────────────────────────────────────

    [Fact]
    public void AbsoluteLinearMoveGivesAbsoluteTargets()
    {
        var g = new GcodeInterpreter();
        var op = Single(g, "G1 X10 Y20 Z-5 F600");
        Assert.Equal(GcodeOpKind.Move, op.Kind);
        Assert.False(op.Rapid);
        Assert.False(op.Relative);
        Assert.Equal(10, op.X);
        Assert.Equal(20, op.Y);
        Assert.Equal(-5, op.Z);
        Assert.Null(op.A);
        Assert.Equal(10.0, op.FeedMmPerSec!.Value, 6); // 600 mm/min ÷ 60
    }

    [Fact]
    public void UnmentionedAxesAreNullSoTheyHold()
    {
        var g = new GcodeInterpreter();
        g.Feed("G1 X10 Y20 F600");
        var op = Single(g, "Y30"); // modal motion mode G1, only Y present
        Assert.Null(op.X);
        Assert.Equal(30, op.Y);
        Assert.Null(op.Z);
    }

    [Fact]
    public void RapidG0HasNoFeedAndRapidFlag()
    {
        var g = new GcodeInterpreter();
        var op = Single(g, "G0 X5 Y5");
        Assert.True(op.Rapid);
        Assert.Null(op.FeedMmPerSec);
    }

    [Fact]
    public void InchModeScalesToMm()
    {
        var g = new GcodeInterpreter();
        var op = Single(g, "G20 G1 X1 F60");
        Assert.Equal(25.4, op.X!.Value, 6);
        Assert.Equal(25.4, op.FeedMmPerSec!.Value, 6); // 60 in/min = 1524 mm/min = 25.4 mm/s
    }

    [Fact]
    public void RelativeModeEmitsDeltas()
    {
        var g = new GcodeInterpreter();
        g.Feed("G90 G1 X10 F600");
        var op = Single(g, "G91 X2 Z-1");
        Assert.True(op.Relative);
        Assert.Equal(2, op.X);
        Assert.Equal(-1, op.Z);
    }

    [Fact]
    public void FeedIsModalAcrossLines()
    {
        var g = new GcodeInterpreter();
        g.Feed("G1 X1 F1200");
        var op = Single(g, "X2");
        Assert.Equal(20.0, op.FeedMmPerSec!.Value, 6); // still 1200 mm/min
    }

    [Fact]
    public void DefaultFeedUsedWhenNoFGiven()
    {
        var g = new GcodeInterpreter(new GcodeOptions { DefaultFeedMmPerSec = 42 });
        var op = Single(g, "G1 X1");
        Assert.Equal(42, op.FeedMmPerSec!.Value, 6);
    }

    // ── G92 work offset ─────────────────────────────────────────────────────

    [Fact]
    public void G92ShiftsSubsequentAbsoluteCoordinates()
    {
        var g = new GcodeInterpreter(start: (100, 0, 0, 0));
        // At machine X=100, declare this to be X=0. Then X10 means machine 110.
        var setPos = g.Feed("G92 X0");
        Assert.Contains(setPos, o => o.Kind == GcodeOpKind.SetPosition);
        var op = Single(g, "G1 X10 F600");
        Assert.Equal(110, op.X!.Value, 6);
    }

    // ── Arcs ────────────────────────────────────────────────────────────────

    [Fact]
    public void ArcExpandsToBlendedSegmentsEndingAtTarget()
    {
        var g = new GcodeInterpreter(new GcodeOptions { ArcToleranceMm = 0.1 });
        g.Feed("G1 X10 Y0 F600");          // start at (10,0)
        var ops = g.Feed("G2 X0 Y10 I-10 J0"); // quarter circle CW, center (0,0), r=10
        Assert.True(ops.Count > 3);
        Assert.All(ops, o => Assert.True(o.ArcSegment));
        var last = ops[^1];
        Assert.Equal(0, last.X!.Value, 3);
        Assert.Equal(10, last.Y!.Value, 3);
        // Midpoints stay on the radius-10 circle about the origin.
        var mid = ops[ops.Count / 2];
        double r = System.Math.Sqrt(mid.X!.Value * mid.X.Value + mid.Y!.Value * mid.Y.Value);
        Assert.Equal(10, r, 1);
    }

    // ── Non-motion ops ──────────────────────────────────────────────────────

    [Fact]
    public void DwellSecondsBecomesMilliseconds()
    {
        var g = new GcodeInterpreter();
        var op = Single(g, "G4 P2.5");
        Assert.Equal(GcodeOpKind.Dwell, op.Kind);
        Assert.Equal(2500, op.DwellMs, 3);
    }

    [Fact]
    public void SpindleOnOffParse()
    {
        var g = new GcodeInterpreter();
        var on = Single(g, "M3 S1000");
        Assert.Equal(GcodeOpKind.Spindle, on.Kind);
        Assert.True(on.SpindleOn);
        Assert.Equal(1000, on.SpindleSpeed);
        var off = Single(g, "M5");
        Assert.False(off.SpindleOn);
    }

    [Fact]
    public void CommentsAndLineNumbersAreIgnored()
    {
        var g = new GcodeInterpreter();
        Assert.Empty(g.Feed("; a comment"));
        Assert.Empty(g.Feed("(inline comment)"));
        var op = Single(g, "N20 G1 X5 F600 (go) ; trailing");
        Assert.Equal(5, op.X);
    }

    [Fact]
    public void EndCodesEmitEndOp()
    {
        var g = new GcodeInterpreter();
        Assert.Contains(g.Feed("M30"), o => o.Kind == GcodeOpKind.End);
    }

    [Fact]
    public void UnsupportedCodeThrows()
    {
        var g = new GcodeInterpreter();
        Assert.Throws<GcodeException>(() => g.Feed("G33 X1"));
        Assert.Throws<GcodeException>(() => g.Feed("M99"));
        Assert.Throws<GcodeException>(() => g.Feed("Q5"));
    }

    // ── GcodeToSteps ────────────────────────────────────────────────────────

    [Fact]
    public void GenerateProducesExpectedStepTypes()
    {
        const string program = """
            G21 G90
            G0 X0 Y0 Z5
            M3 S1000
            G1 Z-1 F300
            G1 X10 Y0
            G4 P0.5
            M5
            M30
            """;
        var steps = GcodeToSteps.Generate(program, new GcodeOptions(),
            new SpindleOutput("stb", 1));

        Assert.Contains(steps, s => s.Type == StepType.MoveL);
        Assert.Contains(steps, s => s.Type == StepType.Wait && s.WaitMs == 500);
        // M3 then M5 → two SetOutput steps on stb pin 1.
        var outs = steps.Where(s => s.Type == StepType.SetOutput).ToList();
        Assert.Equal(2, outs.Count);
        Assert.True(outs[0].OutputValue);
        Assert.False(outs[1].OutputValue);
        Assert.All(outs, o => { Assert.Equal("stb", o.OutputCard); Assert.Equal(1, o.OutputNumber); });
        // Rapid uses the configured rapid speed; the plunge uses the feed (300 ÷ 60 = 5).
        var rapid = steps.First(s => s.Type == StepType.MoveL && s.OverrideZ == 5);
        Assert.Equal(new GcodeOptions().RapidSpeedMmPerSec, rapid.Speed);
        var plunge = steps.First(s => s.Type == StepType.MoveL && s.OverrideZ == -1);
        Assert.Equal(5.0, plunge.Speed!.Value, 6);
    }

    [Fact]
    public void GenerateSkipsBadLinesButKeepsGoodOnes()
    {
        const string program = "G1 X1 F600\nBOGUS LINE\nG1 X2";
        var steps = GcodeToSteps.Generate(program, new GcodeOptions(), new SpindleOutput("none", 0));
        Assert.Equal(2, steps.Count(s => s.Type == StepType.MoveL));
    }

    [Fact]
    public void SpindleNoneEmitsNoOutputSteps()
    {
        var steps = GcodeToSteps.Generate("M3 S1\nM5", new GcodeOptions(), new SpindleOutput("none", 0));
        Assert.DoesNotContain(steps, s => s.Type == StepType.SetOutput);
    }
}
