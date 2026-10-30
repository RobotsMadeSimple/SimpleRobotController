using System.Reflection;
using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.AuxAxis;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Io;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.Validation;

namespace RobotControl.Tests;

/// <summary>
/// The "stop / record on condition" move-step feature driven through the executor, tick by
/// tick (docs/program-blocks.md). There is no motion thread in these tests: a test plays its
/// part by draining <see cref="RobotController.QueuedCommands"/> once the executor has
/// dispatched a move (what <c>ExecuteMotionStop</c> / a finished move leave behind), and reads
/// the request flags the executor sets for it. Nano IO is driven by writing the live pin
/// state of the controller's own Nano device, so each tick's condition sees fresh IO exactly
/// as <c>ProgramExecutor.AddIoVariables</c> publishes it.
/// </summary>
[Collection("RobotController")]
public sealed class MoveStopConditionExecutionTests : IDisposable
{
    private const int TriggerPin  = 12;
    private const int LoadCellPin = 10;
    private const string AuxDeviceId = "AUX_STOPTEST";

    private readonly string              _tmp;
    private readonly RobotController     _robot = new();
    private readonly ProgramCycleManager _programs = new();
    private readonly ProgramExecutor     _exec;
    private readonly NanoDevice          _nano;
    private readonly string              _nanoAlias;   // identifier-safe nano name, e.g. Arduino_Nano
    private AuxAxisDevice?               _aux;

    public MoveStopConditionExecutionTests()
    {
        _tmp = Path.Combine(Path.GetTempPath(), "rms-movestop-exec-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(_tmp);

        var points = new PointRepository(Path.Combine(_tmp, "points.json"), Path.Combine(_tmp, "pointsHistory.json"));
        var grids  = new GridRepository(Path.Combine(_tmp, "grids.json"));
        var stacks = new StackRepository(Path.Combine(_tmp, "stacks.json"));
        _exec = new ProgramExecutor(_robot, _programs, points, _robot.toolRepo, _robot.localRepo,
                                    _robot.builtProgramRepo, grids, stacks);

        // The controller's first configured Nano (ctor only — never started, no serial). Two
        // pins of our own: a digital trigger and a load cell, both named so they need an alias.
        _nano      = _robot.NanoManager.GetDevices().First();
        _nanoAlias = IoVariableNames.Sanitize(_nano.Name);
        _nano.UpsertPinState(TriggerPin,  PinType.Input,    "Stop Trigger", 1);
        _nano.UpsertPinState(LoadCellPin, PinType.LoadCell, "Load Cell",    1);
        Trigger(false);
        Grams(0);
    }

    public void Dispose()
    {
        _aux?.Stop();
        try { Directory.Delete(_tmp, recursive: true); } catch { /* best effort */ }
    }

    // ── Rig helpers ───────────────────────────────────────────────────────────

    private string TriggerExpr  => $"$io.{_nanoAlias}.Stop_Trigger == 1";
    private string LoadCellExpr(double grams) => $"$io.{_nanoAlias}.Load_Cell >= {grams}";

    private void Trigger(bool on)  => _nano.GetPinState(TriggerPin)!.Value  = on;
    private void Grams(double g)   => _nano.GetPinState(LoadCellPin)!.Grams = g;

    private double Live(string expr) => _exec.EvaluateLive(expr);

    /// <summary>Plays the motion thread: takes every queued command, returning their types.</summary>
    private List<string> Drain()
    {
        var types = new List<string>();
        while (_robot.QueuedCommands.TryDequeue(out var cmd)) types.Add(cmd.CommandType ?? "");
        return types;
    }

    private string[] Queued() => _robot.QueuedCommands.Select(c => c.CommandType ?? "").ToArray();

    private static readonly FieldInfo MotionStopField =
        typeof(RobotController).GetField("_motionStopRequested", BindingFlags.NonPublic | BindingFlags.Instance)!;

    /// <summary>The flag <see cref="RobotController.RequestMotionStop"/> raises for the motion thread.</summary>
    private bool MotionStopRequested => (bool)MotionStopField.GetValue(_robot)!;

    /// <summary>Ticks until the program ends (nothing may be left queued). Fails if it never does.</summary>
    private void FinishRun(int maxTicks = 10)
    {
        for (int i = 0; i < maxTicks && _exec.IsRunning; i++) _exec.Update();
        Assert.False(_exec.IsRunning, "program did not finish");
    }

    private JsonElement Summary(string name = "Main") =>
        JsonSerializer.SerializeToElement(_programs.GetProgramsSummary())
            .EnumerateArray().First(p => p.GetProperty("name").GetString() == name);

    private string Status()           => Summary().GetProperty("status").GetString()!;
    private string ErrorDescription() => Summary().GetProperty("errorDescription").GetString() ?? "";
    private int    StepsCompleted()   => Summary().GetProperty("currentStepNumber").GetInt32();

    /// <summary>Moves the robot's reported position (no motion thread needed) so a test can
    /// tell "captured once" from "captured again later".</summary>
    private double NudgeLiveZ()
    {
        double before = _robot.GetCurrentPosition().Z;
        _robot.SetJointPosition(2, before + 37.5);   // ASTRO J2/J3 CoreXY: joint 2 is the Z carriage
        double after = _robot.GetCurrentPosition().Z;
        Assert.NotEqual(before, after);
        return after;
    }

    /// <summary>A simulated (never connected) aux device the controller can drive: 200 steps/mm,
    /// seeded at 4000 steps = 20 mm.</summary>
    private void InjectAuxDevice()
    {
        var cfg = new AuxAxisConfig
        {
            Id = AuxDeviceId, Name = "Stop Test Axis",
            Axes = { new AuxAxisChannelConfig { AxisIndex = 0, Name = "A", StepsPerRev = 1600, AxisType = AuxAxisType.Linear, MmPerRev = 8 } },
        };
        var mgr = _robot.AuxAxisManager;
        mgr.GetConfig().Devices.Add(cfg);   // GetAxisConfig reads the live config
        _aux = new AuxAxisDevice(cfg);      // not Start()ed: no serial thread, offline simulation only
        var devices = (List<AuxAxisDevice>)typeof(AuxAxisManager)
            .GetField("_devices", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(mgr)!;
        devices.Add(_aux);
        _aux.SetPosition(0, 4000);
        Assert.Equal(200, mgr.GetAxisConfig(AuxDeviceId, 0)!.StepsPerUnit());
    }

    // ── Program builders ──────────────────────────────────────────────────────

    private static ProgramVariable Num(string name)     => new() { Id = name, Name = name };
    private static ProgramVariable Points(string name)  => new() { Id = name, Name = name, ElementType = ListElementType.Point,  Items = [] };
    private static ProgramVariable Numbers(string name) => new() { Id = name, Name = name, ElementType = ListElementType.Number, Items = [] };

    private static ConditionGroup Cond(string expr) =>
        new() { Items = [new ConditionItem { Id = "c1", Expr = expr }] };

    private static BuiltProgram Prog(List<ProgramVariable> vars, params ProgramStep[] steps) =>
        new() { Id = "main", Name = "Main", Steps = steps.ToList(), Variables = vars };

    private static int _ids;

    private static ProgramStep MoveL(string? cond, string mode = "stop", string? posVar = null, string? flagVar = null,
                                     bool blend = false) => new()
    {
        Id = $"m{++_ids}", Type = StepType.MoveL,
        Blend = blend, BlendRadius = blend ? 5 : null,
        MoveStopCondition = cond == null ? null : Cond(cond), MoveStopMode = mode,
        MoveStopPositionVariableName = posVar, MoveStopFlagVariableName = flagVar,
    };

    private static ProgramStep JumpL(string cond, string mode, string? posVar, string flagVar) => new()
    {
        Id = $"j{++_ids}", Type = StepType.JumpL, JumpZ = 50,
        MoveStopCondition = Cond(cond), MoveStopMode = mode,
        MoveStopPositionVariableName = posVar, MoveStopFlagVariableName = flagVar,
    };

    private static ProgramStep AuxMove(string cond, string mode, string posVar, string flagVar) => new()
    {
        Id = $"a{++_ids}", Type = StepType.AuxMove, AuxDeviceId = AuxDeviceId, AuxAxisIndex = 0,
        AuxSteps = 1_000_000, AuxVelocity = 1, AuxAccel = 3200, AuxDecel = 3200,   // 1 step/s: effectively parked
        MoveStopCondition = Cond(cond), MoveStopMode = mode,
        MoveStopPositionVariableName = posVar, MoveStopFlagVariableName = flagVar,
    };

    private static ProgramStep SetVar(string name, string expr) =>
        new() { Id = $"s{++_ids}", Type = StepType.SetVariable, VariableName = name, VariableExpr = expr };

    private static List<ValidationProblem> Validate(BuiltProgram p) =>
        ProgramValidator.Validate(p, ValidationContext.Offline());

    // ── 1 / 2: record mode and a condition that never fires ───────────────────

    [Fact]
    public void RecordModeCapturesThePositionWithoutStoppingTheMove()
    {
        _exec.Start(Prog([Points("contact"), Num("hit")], MoveL(TriggerExpr, "record", "contact", "hit")));

        _exec.Update();                                   // dispatch + arm
        Assert.Equal(new[] { "MoveL" }, Queued());
        _exec.Update();                                   // evaluated: still false
        Assert.Equal(0, Live("$contact.length"));

        Trigger(true);
        _exec.Update();                                   // fires mid-move
        Assert.Equal(1, Live("$contact.length"));
        Assert.Equal(_robot.GetCurrentPosition().Z, Live("$contact[0].z"));
        Assert.False(MotionStopRequested);                // record: the move is left alone …
        Assert.Equal(new[] { "MoveL" }, Queued());        // … and so is the queue
        Assert.Equal(0, Live("$hit"));                    // the flag waits for completion

        Assert.Equal(new[] { "MoveL" }, Drain());         // the move runs to its target
        _exec.Update();
        Assert.Equal(1, Live("$hit"));
        FinishRun();
        Assert.Equal("Complete", Status());
        Assert.False(MotionStopRequested);
    }

    [Fact]
    public void ConditionNeverTrueLeavesThePositionUntouchedAndWritesFlagZero()
    {
        _exec.Start(Prog([Points("contact"), Num("hit")], MoveL(TriggerExpr, "stop", "contact", "hit")));

        _exec.Update();                                   // dispatch
        for (int i = 0; i < 5; i++) _exec.Update();       // evaluated every tick, never true
        Assert.True(_exec.IsRunning);
        Assert.Equal(new[] { "MoveL" }, Queued());
        Assert.False(MotionStopRequested);

        Drain();
        _exec.Update();                                   // normal completion
        Assert.Equal(0, Live("$hit"));
        Assert.Equal(0, Live("$contact.length"));
        FinishRun();
        Assert.Equal("Complete", Status());
        Assert.False(MotionStopRequested);
    }

    // ── 3: already true when the move starts ──────────────────────────────────

    [Fact]
    public void ConditionTrueOnTheFirstTickStopsTheMoveAndTheNextStepRuns()
    {
        _exec.Start(Prog([Points("contact"), Num("hit"), Num("after")],
            MoveL("1 == 1", "stop", "contact", "hit"),
            SetVar("after", "1")));

        _exec.Update();                                   // dispatch + arm (not evaluated yet)
        Assert.False(MotionStopRequested);
        _exec.Update();                                   // first evaluation: fires at once
        Assert.True(MotionStopRequested);
        Assert.Equal(1, Live("$contact.length"));
        Assert.Equal(0, Live("$after"));                  // the next step waits for the move to end

        Assert.Equal(new[] { "MoveL" }, Drain());         // ExecuteMotionStop drops it
        _exec.Update();                                   // move finished → flag, then the next step runs
        Assert.Equal(1, Live("$hit"));
        Assert.Equal(1, Live("$after"));
        FinishRun();
        Assert.Equal("Complete", Status());
        Assert.Equal(2, StepsCompleted());
    }

    // ── 4: a broken condition ends the program ────────────────────────────────

    [Fact]
    public void UnknownVariableInTheConditionEndsTheProgramWithAnError()
    {
        _exec.Start(Prog([Num("hit")], MoveL("$nope == 1", "stop", null, "hit"), SetVar("hit", "5")));

        _exec.Update();                                   // dispatch
        Assert.True(_exec.IsRunning);
        _exec.Update();                                   // first evaluation throws → Error
        Assert.False(_exec.IsRunning);
        Assert.Equal("Error", Status());
        Assert.StartsWith("Stop condition on", ErrorDescription());
        Assert.Contains("$nope", ErrorDescription());
        Assert.Equal(0, Live("$hit"));                    // nothing after the move ran
    }

    [Fact]
    public void SyntaxErrorInTheConditionEndsTheProgramWithAnError()
    {
        _exec.Start(Prog([Num("hit")], MoveL("$hit >= ", "stop", null, "hit")));

        _exec.Update();
        _exec.Update();
        Assert.False(_exec.IsRunning);
        Assert.Equal("Error", Status());
        Assert.StartsWith("Stop condition on", ErrorDescription());
    }

    // ── 5 / 6: stepped JumpL ──────────────────────────────────────────────────

    [Fact]
    public void StoppedJumpSkipsTheRemainingLegsAndCompletesTheStepOnce()
    {
        _exec.Start(Prog([Num("hit"), Num("after")],
            JumpL(TriggerExpr, "stop", null, "hit"),
            SetVar("after", "1")));

        _exec.Update();                                   // leg 1 (lift) dispatched, watch armed
        Assert.Equal(new[] { "MoveL" }, Queued());
        _exec.Update();                                   // not yet
        Assert.False(MotionStopRequested);

        Trigger(true);
        _exec.Update();                                   // fires during leg 1
        Assert.True(MotionStopRequested);
        Assert.Equal(new[] { "MoveL" }, Drain());         // the motion thread drops the leg

        _exec.Update();                                   // leg finished: the Jump ends here
        Assert.Empty(Queued());                           // legs 2 and 3 never dispatch
        Assert.Equal(1, Live("$hit"));
        Assert.Equal(1, Live("$after"));                  // the next step ran in the same tick
        FinishRun();
        Assert.Equal("Complete", Status());
        Assert.Equal(2, StepsCompleted());                // Jump once + SetVariable once
        Assert.Empty(Queued());
    }

    [Fact]
    public void RecordedJumpRunsEveryLegAndKeepsTheFirstCapturedPosition()
    {
        _exec.Start(Prog([Points("rec"), Num("hit")], JumpL(TriggerExpr, "record", "rec", "hit")));

        _exec.Update();                                   // leg 1
        Trigger(true);
        _exec.Update();                                   // fires during leg 1
        double z0 = Live("$rec[0].z");
        Assert.Equal(_robot.GetCurrentPosition().Z, z0);
        Assert.False(MotionStopRequested);
        Assert.Equal(new[] { "MoveL" }, Drain());

        double zLater = NudgeLiveZ();                     // the robot is somewhere else for legs 2 and 3
        _exec.Update();                                   // leg 1 done → leg 2 dispatched, watch kept
        Assert.Equal(new[] { "MoveL" }, Drain());
        _exec.Update();                                   // leg 3
        Assert.Equal(new[] { "MoveL" }, Drain());
        Assert.Equal(0, Live("$hit"));                    // not complete until the last leg lands
        _exec.Update();                                   // all legs done → flag written

        Assert.Equal(1, Live("$hit"));
        Assert.Equal(1, Live("$rec.length"));
        Assert.Equal(z0, Live("$rec[0].z"));              // captured once, never overwritten
        Assert.NotEqual(zLater, Live("$rec[0].z"));
        FinishRun();
        Assert.Equal("Complete", Status());
        Assert.Equal(1, StepsCompleted());
    }

    // ── 7: blending ───────────────────────────────────────────────────────────

    [Fact]
    public void WatchedMoveLDispatchesAloneAndNeverJoinsABlendedRun()
    {
        // A watched blend-on move is dispatched as a single MoveL; the blendable moves after
        // it still form their own run.
        _exec.Start(Prog([Num("hit")],
            MoveL(TriggerExpr, "stop", null, "hit", blend: true),
            MoveL(null, blend: true),
            MoveL(null, blend: false)));

        _exec.Update();
        Assert.Equal(new[] { "MoveL" }, Drain());         // not StartContinuous
        _exec.Update();                                   // watched move done → the run after it
        Assert.Equal(new[] { "StartContinuous" }, Drain());
        FinishRun();
        Assert.Equal("Complete", Status());
        Assert.Equal(3, StepsCompleted());

        // A run being gathered stops before a watched move: every move dispatches on its own.
        _exec.Start(Prog([Num("hit")],
            MoveL(null, blend: true),
            MoveL(TriggerExpr, "stop", null, "hit", blend: true),
            MoveL(null, blend: true)));

        var dispatched = new List<string>();
        for (int i = 0; i < 3; i++)
        {
            _exec.Update();
            dispatched.AddRange(Drain());
        }
        Assert.Equal(new[] { "MoveL", "MoveL", "MoveL" }, dispatched);
        FinishRun();
        Assert.Equal("Complete", Status());
        Assert.Equal(3, StepsCompleted());
    }

    // ── 8: aux moves ──────────────────────────────────────────────────────────

    [Fact]
    public void AuxStopModeStopsTheAxisSmoothlyAndRecordsItsPositionInUnits()
    {
        InjectAuxDevice();
        _exec.Start(Prog([Num("gap"), Num("hit"), Num("after")],
            AuxMove(TriggerExpr, "stop", "gap", "hit"),
            SetVar("after", "1")));

        _exec.Update();                                   // StartAuxMove → the sim axis is "moving"
        Assert.True(_robot.IsAuxMoving);
        _exec.Update();                                   // waiting, condition false
        Assert.True(_exec.IsRunning);
        Assert.True(_robot.IsAuxMoving);
        Assert.Equal(0, Live("$after"));

        Trigger(true);
        _exec.Update();                                   // fires → StopSmooth ends the sim move at once
        Assert.False(_robot.IsAuxMoving);
        Assert.Equal(1, Live("$hit"));
        Assert.InRange(Live("$gap"), 20.0, 20.01);        // 4000 steps / 200 steps per mm
        Assert.Equal(1, Live("$after"));
        FinishRun();
        Assert.Equal("Complete", Status());
        Assert.False(MotionStopRequested);                // an aux stop never touches the robot move
    }

    [Fact]
    public void AuxRecordModeCapturesThePositionAndKeepsWaitingForTheAxis()
    {
        InjectAuxDevice();
        _exec.Start(Prog([Num("gap"), Num("hit"), Num("after")],
            AuxMove(TriggerExpr, "record", "gap", "hit"),
            SetVar("after", "1")));

        _exec.Update();
        Trigger(true);
        _exec.Update();                                   // fires: position captured, axis left running
        Assert.InRange(Live("$gap"), 20.0, 20.01);
        Assert.True(_robot.IsAuxMoving);
        Assert.True(_exec.IsRunning);
        Assert.Equal(0, Live("$hit"));
        Assert.Equal(0, Live("$after"));
        _exec.Update();                                   // still waiting
        Assert.Equal(0, Live("$after"));

        _robot.AuxAxisManager.StopSmooth(AuxDeviceId, 0, 1000);   // the axis reaches its target
        _exec.Update();
        Assert.Equal(1, Live("$hit"));
        Assert.Equal(1, Live("$after"));
        FinishRun();
        Assert.Equal("Complete", Status());
    }

    // ── 9: load cell grams in the condition ───────────────────────────────────

    [Fact]
    public void LoadCellGramsReadThroughTheAliasAndStopTheMove()
    {
        // The executor's own evaluation sees the live grams under the identifier-safe alias.
        Grams(100);
        Assert.Equal(0, Live(LoadCellExpr(500)));
        Assert.Equal(100, Live($"$io.{_nanoAlias}.Load_Cell"));
        Grams(750);
        Assert.Equal(1, Live(LoadCellExpr(500)));

        Grams(100);
        _exec.Start(Prog([Points("contact"), Num("hit")], MoveL(LoadCellExpr(500), "stop", "contact", "hit")));
        _exec.Update();                                   // dispatch
        _exec.Update();                                   // 100 g: below the limit
        Assert.False(MotionStopRequested);
        Assert.Equal(0, Live("$contact.length"));

        Grams(750);
        _exec.Update();                                   // the force limit is reached
        Assert.True(MotionStopRequested);
        Assert.Equal(1, Live("$contact.length"));
        Drain();
        _exec.Update();
        Assert.Equal(1, Live("$hit"));
        FinishRun();
        Assert.Equal("Complete", Status());
    }

    // ── 10: user Stop / Continue ──────────────────────────────────────────────

    [Fact]
    public void StopAndContinueKeepTheWatchArmedSoItStillFiresAfterResume()
    {
        _exec.Start(Prog([Points("contact"), Num("hit")], MoveL(TriggerExpr, "stop", "contact", "hit")));
        _exec.Update();                                   // dispatch
        _exec.Update();                                   // false
        Assert.Equal(new[] { "MoveL" }, Queued());

        _exec.Stop();                                     // user Stop mid-move
        Assert.False(_exec.IsRunning);
        Drain();                                          // the motion thread drains the queue on a Stop
        _exec.Resume();
        _exec.Update();                                   // Continue re-dispatches the interrupted move
        Assert.True(_exec.IsRunning);
        Assert.Equal(new[] { "MoveL" }, Queued());
        Assert.False(MotionStopRequested);
        Assert.Equal(0, Live("$contact.length"));

        Trigger(true);
        _exec.Update();                                   // the watch travelled with the move
        Assert.True(MotionStopRequested);
        Assert.Equal(1, Live("$contact.length"));
        Drain();
        _exec.Update();
        Assert.Equal(1, Live("$hit"));
        FinishRun();
        Assert.Equal("Complete", Status());
    }

    [Fact]
    public void AFiredRecordWatchDoesNotCaptureAgainAfterContinue()
    {
        Trigger(true);
        _exec.Start(Prog([Points("rec"), Num("hit")], MoveL(TriggerExpr, "record", "rec", "hit")));
        _exec.Update();                                   // dispatch
        _exec.Update();                                   // fires: captured
        double z0 = Live("$rec[0].z");

        _exec.Stop();
        Drain();
        double zLater = NudgeLiveZ();                     // the robot is elsewhere when it resumes
        _exec.Resume();
        _exec.Update();                                   // re-dispatched; Fired travels with the watch
        Assert.Equal(new[] { "MoveL" }, Drain());
        _exec.Update();                                   // completes

        Assert.Equal(1, Live("$hit"));
        Assert.Equal(1, Live("$rec.length"));
        Assert.Equal(z0, Live("$rec[0].z"));              // still the first capture
        Assert.NotEqual(zLater, Live("$rec[0].z"));
        FinishRun();
        Assert.Equal("Complete", Status());
    }

    // ── 11: validator ─────────────────────────────────────────────────────────

    [Fact]
    public void ValidatorIgnoresAStopConditionOnAStepThatIsNotAMove()
    {
        // The fields are only meaningful on move steps; elsewhere they are dead data, not errors
        // (even with an unknown variable in the condition and an undeclared position variable).
        var p = Prog([], new ProgramStep
        {
            Id = "w", Type = StepType.Wait, WaitMs = 10,
            MoveStopCondition = Cond("$nope == 1"), MoveStopMode = "bogus",
            MoveStopPositionVariableName = "nowhere", MoveStopFlagVariableName = "nothing",
        });
        Assert.Empty(Validate(p));
    }

    [Fact]
    public void ValidatorWarnsWhenAnAuxMoveWithAStopConditionDoesNotWaitForDone()
    {
        var p = Prog([], new ProgramStep
        {
            Id = "a", Type = StepType.AuxMove, AuxSteps = 100, AuxWaitForDone = false,
            MoveStopCondition = Cond("$stb.in1 == 1"),
        });
        var problem = Assert.Single(Validate(p));
        Assert.Equal(ValidationSeverity.Warning, problem.Severity);
        Assert.Equal("moveStopCondition", problem.Field);
        Assert.Contains("auxWaitForDone", problem.Message);
    }

    [Fact]
    public void ValidatorRejectsANumberListAsARobotMovePositionVariable()
    {
        var p = Prog([Numbers("samples")], new ProgramStep
        {
            Id = "m", Type = StepType.MoveL,
            MoveStopCondition = Cond("$stb.in1 == 1"), MoveStopPositionVariableName = "$samples",
        });
        var problem = Assert.Single(Validate(p));
        Assert.Equal(ValidationCodes.UnknownVariable, problem.Code);
        Assert.Equal("moveStopPositionVariableName", problem.Field);
        Assert.Contains("not a points list", problem.Message);
    }

    // ── Recordings end with the run ───────────────────────────────────────────

    [Fact]
    public void AResetRunEndsTheRecordingItStarted()
    {
        _exec.Start(Prog([new ProgramVariable { Id = "log", Name = "log", ElementType = ListElementType.Record, Items = [] }],
            new ProgramStep { Id = "rec", Type = StepType.StartRecording, LoadCellDeviceId = _nano.Id,
                              LoadCellPin = LoadCellPin, RecordVariableName = "log" },
            new ProgramStep { Id = "w", Type = StepType.Wait, WaitMs = 60_000 }));

        _exec.Update();   // StartRecording
        _exec.Update();   // into the wait
        var rec = Assert.Single(_robot.IoHistory.ListRecordings(), r => r.Active);

        _exec.Stop();     // a main program pauses: the test can resume, so it keeps recording
        Assert.True(_robot.IoHistory.GetRecordingMeta(rec.Id)!.Active);

        _exec.Reset();    // the run is over without a StopRecording step
        Assert.False(_robot.IoHistory.GetRecordingMeta(rec.Id)!.Active);
    }
}
