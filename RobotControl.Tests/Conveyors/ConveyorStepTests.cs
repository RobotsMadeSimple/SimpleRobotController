using Controller.RobotControl;
using Controller.RobotControl.Conveyors;
using Controller.RobotControl.Language;
using Controller.RobotControl.Programs;
using Controller.RobotControl.Validation;
using static RobotControl.Tests.StepsRig;

namespace RobotControl.Tests.Conveyors;

/// <summary>The four conveyor steps, the part-frame move resolution and the tracking faults, run
/// on a private executor over a fake belt (the motion thread is played by <see cref="StepsRig"/>).</summary>
public sealed class ConveyorStepTests : IDisposable
{
    private readonly StepsRig _rig = new();
    private readonly FakeClock _clock = new();
    private readonly FakeBelt _belt;
    private readonly ConveyorService _svc;
    private readonly Conveyor _c;

    public ConveyorStepTests()
    {
        _belt = new FakeBelt(_clock.Now);
        (_svc, var saved) = ConveyorKit.Service(_belt, _clock.Now, ConveyorKit.Aux("Infeed")); // frame (100, 200, 10), window 50…300
        _c = saved[0];
        _rig.Robot.Conveyors = _svc;
    }

    public void Dispose()
    {
        _rig.Dispose();
        _svc.Dispose();
    }

    private ProgramStep Track(Action<ProgramStep>? init = null) =>
        Step(StepType.ConveyorTrack, s => { s.ConveyorId = _c.Id; init?.Invoke(s); });

    private double Prop(string name) => _rig.Live($"$conveyor.{_c.Id}.{name}");

    [Fact]
    public void TrackWaitsForAPartToEnterTheWindowThenTracksIt()
    {
        _belt.StepsPerSec = 1000; // 100 mm/s
        _rig.Start(Prog(Track(), Set("x", $"$conveyor.{_c.Id}.part.id")));
        _rig.Tick(3);
        Assert.True(_rig.Exec.IsRunning);         // nothing queued yet
        long id = _svc.AddPart(_c.Id, 20);        // upstream of the window (starts at 50)
        _rig.Tick(3);
        Assert.True(_rig.Exec.IsRunning);
        _clock.Advance(0.4);                      // x = 60: inside
        for (int i = 0; i < 10 && _rig.Exec.IsRunning; i++) _rig.Tick();
        _rig.AssertComplete();
        Assert.Equal(id, _rig.Live("$x"));
    }

    [Fact]
    public void TrackTimesOutWithACatchableConveyorTimeout()
    {
        var tr = Step(StepType.Try, t =>
        {
            t.TrySteps = [Track(s => s.TrackTimeoutMs = 5)];
            t.ErrorCodeVariableName = "code";
            t.CatchSteps = [Set("caught", "1"), Set("ok", "$code.contains(\"conveyorTimeout\")")];
        });
        _rig.Start(Prog([Str("code"), Num("caught"), Num("ok")], tr));
        for (int i = 0; i < 200 && _rig.Exec.IsRunning; i++) { _rig.Tick(); Thread.Sleep(1); }
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$caught"));
        Assert.Equal(1, _rig.Live("$ok"));
    }

    [Fact]
    public void TrackWithoutWaitCarriesOnUntrackedWhenNoPartIsThere()
    {
        _rig.Run(Prog(Track(s => s.TrackWait = false), Set("t", $"$conveyor.{_c.Id}.tracking")));
        _rig.AssertComplete();
        Assert.Equal(0, _rig.Live("$t"));
        Assert.Null(_svc.Tracking);
    }

    [Fact]
    public void ABeltFasterThanTheRobotFailsWithConveyorSpeed()
    {
        _belt.StepsPerSec = 20_000; // 2000 mm/s — far above the default linear speed
        _svc.AddPart(_c.Id, 100);
        _rig.Run(Prog(Track()));
        _rig.AssertError("faster than the robot");
        Assert.Null(_svc.Tracking);
    }

    [Fact]
    public void MovesWhileTrackingResolveInThePartFrameAndRideTheBelt()
    {
        _rig.Points.SavePoint("OnPart", new Vector6(10, 0, 5));
        _belt.StepsPerSec = 100;                  // 10 mm/s
        _svc.AddPart(_c.Id, 80);
        var move = Step(StepType.MoveL, s => s.PointName = "OnPart");
        var blended = Step(StepType.MoveL, s => { s.PointName = "OnPart"; s.OffsetZ = 20; s.Blend = true; s.BlendRadius = 5; });
        var after = Step(StepType.MoveL, s => s.PointName = "OnPart");
        _rig.Run(Prog(Track(), move, blended, Set("x", "1"), Step(StepType.ConveyorEndTracking), after));
        _rig.AssertComplete();

        var moves = _rig.Dispatched.Where(c => c.CommandType == "MoveL").ToList();
        Assert.Equal(3, moves.Count);             // the blend-on move went on its own while tracking
        // Part at X = 80 → part frame (180, 200, 10); the point (10, 0, 5) on the part → (190, 200, 15).
        Assert.Equal(190, moves[0].X!.Value, 3);
        Assert.Equal(200, moves[0].Y!.Value, 3);
        Assert.Equal(15, moves[0].Z!.Value, 3);
        Assert.NotNull(moves[0].TrackTravel);
        Assert.Equal(35, moves[1].Z!.Value, 3);
        Assert.NotNull(moves[1].TrackTravel);
        // After EndTracking: an ordinary move in the base frame.
        Assert.Null(moves[2].TrackTravel);
        Assert.Equal(10, moves[2].X!.Value, 3);
        Assert.Equal(1, _svc.Status().Single().Picked);
    }

    [Fact]
    public void TheTrackedPartLeavingTheWindowIsACatchableConveyorWindowError()
    {
        _belt.StepsPerSec = 1000;                 // 100 mm/s
        _svc.AddPart(_c.Id, 280);
        var tr = Step(StepType.Try, t =>
        {
            t.TrySteps = [Track(), Step(StepType.Wait, w => w.WaitMs = 60_000)];
            t.ErrorCodeVariableName = "code";
            t.CatchSteps = [Set("caught", "1"), Set("ok", "$code.contains(\"conveyorWindow\")")];
        });
        _rig.Start(Prog([Str("code"), Num("caught"), Num("ok")], tr));
        _rig.Tick(4);
        Assert.NotNull(_svc.Tracking);            // tracking, waiting
        _clock.Advance(0.3);                      // x = 310 > 300
        _svc.Tick();
        for (int i = 0; i < 10 && _rig.Exec.IsRunning; i++) _rig.Tick();
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$caught"));
        Assert.Equal(1, _rig.Live("$ok"));
    }

    [Fact]
    public void AddPartAndClearQueueSteps()
    {
        _rig.Run(Prog(
            Step(StepType.ConveyorAddPart, s => { s.ConveyorId = _c.Id; s.Expressions = new() { ["partOffsetMm"] = "40 + 20" }; }),
            Step(StepType.ConveyorAddPart, s => { s.ConveyorId = _c.Id; s.PartOffsetMm = 10; }),
            Set("n", $"$conveyor.{_c.Id}.count"),
            Set("w", $"$conveyor.{_c.Id}.inWindow"),
            Step(StepType.ConveyorClearQueue, s => s.ConveyorId = _c.Id),
            Set("after", $"$conveyor.{_c.Id}.count")));
        _rig.AssertComplete();
        Assert.Equal(2, _rig.Live("$n"));
        Assert.Equal(1, _rig.Live("$w"));         // 60 is inside, 10 is upstream
        Assert.Equal(0, _rig.Live("$after"));
    }

    [Fact]
    public void AnUnknownConveyorEndsTheProgramWithAClearError()
    {
        _rig.Run(Prog(Step(StepType.ConveyorAddPart, s => s.ConveyorId = "nope")));
        _rig.AssertError("conveyor not found");
    }

    [Fact]
    public void TheRunEndingStopsItsTracking()
    {
        _svc.AddPart(_c.Id, 100);
        _rig.Run(Prog(Track()));
        _rig.AssertComplete();
        Assert.Null(_svc.Tracking);               // a finished run never leaves the robot following
    }

    [Fact]
    public void ConveyorPropertiesAreListedAndRead()
    {
        _belt.StepsPerSec = 500;
        _clock.Advance(1);
        Assert.Equal(50, Prop("travel"), 6);
        Assert.Equal(50, Prop("speed"), 6);
        var names = new Controller.RobotControl.Execution.RobotPropertySource(_rig.Robot, null).List().Select(p => p.Name).ToList();
        Assert.Contains($"conveyor.{_c.Id}.part.x", names);
        Assert.Contains($"conveyor.{_c.Id}.missed", names);
        Assert.Contains("conveyor", Controller.RobotControl.Execution.RobotPropertySource.Roots);
    }

    [Fact]
    public void TheValidatorChecksTheConveyor()
    {
        var ctx = new ValidationContext { ConveyorExists = id => id == _c.Id };
        var problems = ProgramValidator.Validate(Prog(
            Step(StepType.ConveyorTrack, s => s.ConveyorId = "gone"),
            Step(StepType.ConveyorClearQueue),
            Step(StepType.ConveyorEndTracking),
            Track()), ctx);
        Assert.Contains(problems, p => p.Code == ValidationCodes.UnknownConveyor);
        Assert.Contains(problems, p => p.Code == ValidationCodes.MissingField && p.Field == "conveyorId");
        Assert.Equal(2, problems.Count(p => p.Field == "conveyorId"));
    }

    [Fact]
    public void TheStepsReadAndWriteAsText()
    {
        var names = new SrsNameTable().Add(SrsRefKind.Conveyor, _c.Id, "Infeed");
        var text = "ConveyorTrack Infeed, timeout=5s\nConveyorAddPart Infeed, x=($v + 1)\nConveyorEndTracking release=on\nConveyorClearQueue Infeed\n";
        var parsed = SrsLanguage.Parse(text, null, names);
        Assert.True(parsed.Ok, string.Join("\n", parsed.Problems.Select(p => $"{p.Line}:{p.Col} {p.Code} {p.Message}")));
        var steps = parsed.Program!.Steps;
        Assert.Equal(StepType.ConveyorTrack, steps[0].Type);
        Assert.Equal(_c.Id, steps[0].ConveyorId);
        Assert.Equal(5000, steps[0].TrackTimeoutMs);
        Assert.Equal("$v + 1", steps[1].Expressions!["partOffsetMm"]);
        Assert.True(steps[2].ConveyorRelease);
        Assert.Null(steps[2].ConveyorId);
        Assert.Equal(_c.Id, steps[3].ConveyorId);
        RobotControl.Tests.Language.SrsTestKit.AssertRoundTrip(parsed.Program!, names);
    }

    [Fact]
    public void ConveyorErrorCodesAreCatchable()
    {
        foreach (var code in new[] { ErrorCodes.ConveyorWindow, ErrorCodes.ConveyorTimeout, ErrorCodes.ConveyorSpeed })
        {
            Assert.True(ErrorCodes.IsKnown(code));
            Assert.True(ErrorCodes.IsCatchable(code));
        }
        Assert.Equal("conveyorWindow", ErrorCodes.ConveyorWindow);
        Assert.Equal("conveyorTimeout", ErrorCodes.ConveyorTimeout);
        Assert.Equal("conveyorSpeed", ErrorCodes.ConveyorSpeed);
    }

    [Fact]
    public void TrackingStepsAreSkippedInBackgroundPrograms()
    {
        Assert.True(ProgramExecutor.IsRestrictedInBackground(StepType.ConveyorTrack));
        Assert.True(ProgramExecutor.IsRestrictedInBackground(StepType.ConveyorEndTracking));
        Assert.False(ProgramExecutor.IsRestrictedInBackground(StepType.ConveyorAddPart));
    }
}
