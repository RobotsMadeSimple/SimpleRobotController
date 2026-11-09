using System.Reflection;
using Controller.RobotControl;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

/// <summary>
/// SetSpeedFactor (docs/program-language.md §4.3): the factor is stamped on every program move
/// (single, blended, jump legs, arcs, resumed), clamped to 1–100 %, readable as
/// <c>$program.speedFactor</c>, back to 100 when the run ends; and the motion side multiplies it
/// into the operator override for program moves only, never above it.
/// </summary>
[Collection("RobotController")]
public sealed class SpeedFactorTests : IDisposable
{
    private readonly StepsRig _rig = new();
    public void Dispose() => _rig.Dispose();

    private static ProgramStep Factor(double? pct, string? expr = null) =>
        StepsRig.Step(StepType.SetSpeedFactor, s =>
        {
            s.SpeedFactor = pct;
            if (expr != null) s.Expressions = new() { ["speedFactor"] = expr };
        });

    private static ProgramStep MoveL(bool blend = false) =>
        StepsRig.Step(StepType.MoveL, s => { s.OffsetX = 1; s.Blend = blend; s.BlendRadius = blend ? 1 : null; });

    private IEnumerable<RobotCommand> Moves => _rig.Dispatched.Where(c => c.ApplySpeedOverride);

    [Fact]
    public void WithoutTheStepMovesRunAtFullFactor()
    {
        _rig.Run(StepsRig.Prog(MoveL()));
        _rig.AssertComplete();
        Assert.Equal(1.0, Assert.Single(Moves).SpeedFactor);
    }

    [Fact]
    public void LaterMovesCarryTheFactor()
    {
        _rig.Run(StepsRig.Prog(MoveL(), Factor(50), MoveL(), Factor(25), MoveL()));
        _rig.AssertComplete();
        Assert.Equal([1.0, 0.5, 0.25], Moves.Select(c => c.SpeedFactor));
    }

    [Fact]
    public void BlendedRunsJumpsAndArcsCarryItToo()
    {
        var s = _rig.Robot.GetCurrentPosition();
        var via = _rig.Points.SavePoint("V", new Vector6(s.X + 10, s.Y + 10, s.Z, 0, 0, 0));
        var end = _rig.Points.SavePoint("E", new Vector6(s.X + 20, s.Y, s.Z, 0, 0, 0));
        _rig.Run(StepsRig.Prog(
            Factor(40),
            MoveL(blend: true), MoveL(),                                                    // one blended run
            StepsRig.Step(StepType.JumpL, j => { j.JumpZ = s.Z + 5; j.OffsetY = 2; }),      // three legs
            StepsRig.Step(StepType.MoveC, c => { c.ViaPointId = via.Id; c.PointId = end.Id; })));
        _rig.AssertComplete();
        var moves = Moves.ToList();
        Assert.Contains(moves, c => c.CommandType == "StartContinuous" && c.Waypoints!.Count == 2);
        Assert.True(moves.Count >= 5, $"only {moves.Count} moves");
        Assert.All(moves, c => Assert.Equal(0.4, c.SpeedFactor, 12));
    }

    [Theory]
    [InlineData(0, 0.01)]
    [InlineData(-20, 0.01)]
    [InlineData(250, 1.0)]
    [InlineData(100, 1.0)]
    [InlineData(1, 0.01)]
    [InlineData(33.3, 0.333)]
    public void TheFactorIsClampedToOneToHundredPercent(double pct, double expected)
    {
        _rig.Run(StepsRig.Prog(Factor(pct), MoveL()));
        Assert.Equal(expected, Assert.Single(Moves).SpeedFactor, 12);
    }

    [Fact]
    public void TheFactorMayBeAnExpressionAndAMissingValueIsFullSpeed()
    {
        _rig.Run(StepsRig.Prog([StepsRig.Num("slow", 30)], Factor(null, "$slow / 2"), MoveL(), Factor(null), MoveL()));
        Assert.Equal([0.15, 1.0], Moves.Select(c => Math.Round(c.SpeedFactor, 12)));
    }

    [Fact]
    public void ReadableAsAProgramPropertyAndResetWhenTheRunEnds()
    {
        _rig.Run(StepsRig.Prog([StepsRig.Num("seen")], Factor(60), StepsRig.Set("seen", "$program.speedFactor")));
        _rig.AssertComplete();
        Assert.Equal(60, _rig.Live("$seen"));
        Assert.Equal(100, _rig.Live("$program.speedFactor")); // reset with the run state at the finish

        _rig.Run(StepsRig.Prog(MoveL()));                       // a new run starts at 100 %
        Assert.Equal(1.0, Moves.Last().SpeedFactor);
    }

    [Fact]
    public void NonProgramCommandsAreNotStamped()
    {
        _rig.Run(StepsRig.Prog(Factor(50), StepsRig.Step(StepType.SetSpeedL, s => s.Speed = 200), MoveL()));
        var setting = Assert.Single(_rig.Dispatched, c => c.CommandType == "SpeedS");
        Assert.False(setting.ApplySpeedOverride);
        Assert.Equal(1.0, setting.SpeedFactor);
    }

    [Fact]
    public void AResumedMoveKeepsTheFactor()
    {
        _rig.Start(StepsRig.Prog(Factor(20), MoveL()));
        _rig.Exec.Update();               // SetSpeedFactor
        _rig.Exec.Update();               // MoveL dispatched, still queued (the motion thread has not run)
        Assert.Single(_rig.Robot.QueuedCommands);
        _rig.Exec.Stop();
        _rig.Robot.QueuedCommands.Clear();  // the drain the Stop requests of the motion thread
        _rig.Exec.Resume();
        _rig.Exec.Update();               // re-dispatches the interrupted move
        var resumed = Assert.Single(_rig.Drain());
        Assert.Equal(0.2, resumed.SpeedFactor, 12);
    }

    [Fact]
    public void TheDispatcherClampsAndResets()
    {
        var d = new MotionDispatcher(_rig.Robot);
        Assert.Equal(100, d.SpeedFactorPercent);
        d.SpeedFactorPercent = 0;          Assert.Equal(1, d.SpeedFactorPercent);
        d.SpeedFactorPercent = 1000;       Assert.Equal(100, d.SpeedFactorPercent);
        d.SpeedFactorPercent = double.NaN; Assert.Equal(100, d.SpeedFactorPercent);
        d.SpeedFactorPercent = 42;
        d.Reset();
        Assert.Equal(100, d.SpeedFactorPercent);
    }

    // ── Motion side ───────────────────────────────────────────────────────────

    [Theory]
    [InlineData(0.5, 0.5)]
    [InlineData(1.0, 1.0)]
    [InlineData(2.0, 1.0)]          // a factor can never speed a program up
    [InlineData(0.0, 0.01)]
    [InlineData(double.NaN, 1.0)]
    public void ProgramSpeedFactorIsLimited(double factor, double expected) =>
        Assert.Equal(expected, RobotController.ProgramSpeedFactor(factor));

    private static readonly FieldInfo ContinuousField =
        typeof(RobotController).GetField("continuousProfiler", BindingFlags.NonPublic | BindingFlags.Instance)!;

    /// <summary>The cruise speed of the blended path the controller just set up.</summary>
    private double ContinuousSpeed()
    {
        var profiler = ContinuousField.GetValue(_rig.Robot)!;
        var scalar = profiler.GetType().GetField("scalar", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(profiler)!;
        return (double)scalar.GetType().GetField("v", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(scalar)!;
    }

    private void StartPath(bool applyOverride, double factor)
    {
        ContinuousField.SetValue(_rig.Robot, null);
        var s = _rig.Robot.GetCurrentPosition();
        _rig.Robot.StartContinuousMove(
            [new Vector6(s.X + 500, s.Y, s.Z, s.RX, s.RY, s.RZ), new Vector6(s.X + 1000, s.Y, s.Z, s.RX, s.RY, s.RZ)],
            [0, 0], 100, 1000, 1000, applyOverride, factor);
    }

    [Fact]
    public void ThePathSpeedIsOverrideTimesFactor()
    {
        double saved = _rig.Robot.SpeedOverrideFactor;
        try
        {
            _rig.Robot.SpeedOverrideFactor = 0.8;
            StartPath(applyOverride: true, factor: 0.5);
            Assert.Equal(40, ContinuousSpeed(), 9);       // 100 × 0.8 × 0.5
            StartPath(applyOverride: true, factor: 1.0);
            Assert.Equal(80, ContinuousSpeed(), 9);
            StartPath(applyOverride: false, factor: 0.5); // a manual move ignores both
            Assert.Equal(100, ContinuousSpeed(), 9);
        }
        finally
        {
            _rig.Robot.SpeedOverrideFactor = saved;
            ContinuousField.SetValue(_rig.Robot, null);
        }
    }
}
