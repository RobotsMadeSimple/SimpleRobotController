using Controller.RobotControl;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Io;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.Validation;
using System.Text.Json;

namespace RobotControl.Tests;

/// <summary>
/// The "stop / record on condition" move-step feature (docs/program-blocks.md): the wire
/// fields, the load-cell / identifier-alias IO naming, the validator rules, and the
/// MotionDispatcher watch lifecycle. Deeper executor scenarios live in their own file.
/// </summary>
[Collection("RobotController")]
public class MoveStopConditionTests
{
    // ── Wire model ────────────────────────────────────────────────────────────

    [Fact]
    public void GoldenCarriesTheStopConditionFields()
    {
        var path = Path.Combine(AppContext.BaseDirectory, "Golden", "programstep.golden.json");
        var opts = new JsonSerializerOptions
        {
            PropertyNameCaseInsensitive = true,
            UnmappedMemberHandling = System.Text.Json.Serialization.JsonUnmappedMemberHandling.Disallow,
        };
        var step = JsonSerializer.Deserialize<ProgramStep>(File.ReadAllText(path), opts)!;

        Assert.NotNull(step.MoveStopCondition);
        Assert.Equal("$io.Arduino_Nano.Load_Cell >= 2000", Assert.Single(step.MoveStopCondition!.Items).Expr);
        Assert.Equal("stop",       step.MoveStopMode);
        Assert.Equal("contactPts", step.MoveStopPositionVariableName);
        Assert.Equal("contactHit", step.MoveStopFlagVariableName);
        Assert.True(MoveStopConditions.HasCondition(step));

        // Round trip under the exact camelCase names the app writes.
        using var doc = JsonDocument.Parse(JsonSerializer.Serialize(step));
        foreach (var key in new[] { "moveStopCondition", "moveStopMode", "moveStopPositionVariableName", "moveStopFlagVariableName" })
            Assert.True(doc.RootElement.TryGetProperty(key, out _), key);
    }

    // ── IO naming ─────────────────────────────────────────────────────────────

    [Fact]
    public void SanitizerReplacesEachRunOfNonIdentifierCharsWithOneUnderscore()
    {
        Assert.Equal("Arduino_Nano",  IoVariableNames.Sanitize("Arduino Nano"));
        Assert.Equal("Load_Cell_g_",  IoVariableNames.Sanitize("Load  Cell (g)"));
        Assert.Equal("_x",            IoVariableNames.Sanitize("-x"));

        // An identifier comes back as the same instance (no per-tick allocation) and has no alias.
        const string plain = "Input_1";
        Assert.Same(plain, IoVariableNames.Sanitize(plain));
        Assert.Null(IoVariableNames.Alias("Nano1", plain));
        Assert.Equal("io.Arduino_Nano.Load_Cell", IoVariableNames.Alias("Arduino Nano", "Load Cell"));
    }

    [Fact]
    public void LoadCellsReadInGramsAndSpacedNamesGetAnAlias()
    {
        var nano = new IoDeviceState
        {
            Id = "NANO_1", Name = "Arduino Nano",
            Pins =
            {
                new IoPinState { Pin = 10, Type = PinType.LoadCell, Name = "Load Cell", Value = true, Grams = 1234.5 },
                new IoPinState { Pin = 2,  Type = PinType.Input,    Name = "D2",        Value = true },
                new IoPinState { Pin = 3,  Type = PinType.Input,    Name = "" },  // unnamed: not published
            },
        };

        var io = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
        IoVariableNames.AddPins([nano], io);

        Assert.Equal(1234.5, io["io.Arduino Nano.Load Cell"]);   // grams, not the 1/0 Value
        Assert.Equal(1234.5, io["io.Arduino_Nano.Load_Cell"]);   // identifier-safe alias
        Assert.Equal(1.0,    io["io.Arduino Nano.D2"]);
        Assert.Equal(1.0,    io["io.Arduino_Nano.D2"]);          // the nano name alone forces an alias
        Assert.Equal(4, io.Count);

        // The symbol list offers only what can be typed, and says a load cell is in grams.
        var symbols = IoVariableNames.Symbols([nano]).ToList();
        Assert.Equal(new[] { "io.Arduino_Nano.Load_Cell", "io.Arduino_Nano.D2" }, symbols.Select(s => s.Name).ToArray());
        Assert.Equal("Nano 'Arduino Nano' pin 10 (LoadCell, grams)", symbols[0].Description);
    }

    // ── Validation ────────────────────────────────────────────────────────────

    private static ProgramVariable Num(string name)    => new() { Id = name, Name = name };
    private static ProgramVariable Points(string name) => new() { Id = name, Name = name, ElementType = ListElementType.Point, Items = [] };
    private static ProgramVariable Numbers(string name) => new() { Id = name, Name = name, ElementType = ListElementType.Number, Items = [] };

    private static ConditionGroup Cond(string expr) =>
        new() { Items = [new ConditionItem { Id = "c1", Expr = expr }] };

    private static BuiltProgram Prog(List<ProgramVariable> vars, params ProgramStep[] steps) =>
        new() { Id = "main", Name = "Main", Steps = steps.ToList(), Variables = vars };

    private static List<ValidationProblem> Validate(BuiltProgram p) =>
        ProgramValidator.Validate(p, ValidationContext.Offline());

    [Fact]
    public void ValidatorAcceptsAValidStopConditionAndCountsItsVariablesAsUsed()
    {
        var p = Prog([Points("contact"), Num("hit"), Num("gap")],
            new ProgramStep
            {
                Id = "m1", Type = StepType.MoveL,
                MoveStopCondition = Cond("$stb.in1 == 1"), MoveStopMode = "Record",
                MoveStopPositionVariableName = "contact", MoveStopFlagVariableName = "$hit",
            },
            new ProgramStep
            {
                Id = "g1", Type = StepType.GripperAction, GripperAction = "Close",
                MoveStopCondition = Cond("$stb.in2 == 1"),
                MoveStopPositionVariableName = "gap",
            });

        Assert.Empty(Validate(p));   // in particular no unusedVariable for contact / hit / gap
    }

    [Fact]
    public void ValidatorRejectsBadModeAndWrongPositionVariables()
    {
        var p = Prog([Points("contact"), Numbers("samples"), Num("flag")],
            new ProgramStep   // steps[0]: unknown mode + undeclared position variable
            {
                Id = "m1", Type = StepType.MoveJ,
                MoveStopCondition = Cond("$stb.in1 == 1"), MoveStopMode = "pause",
                MoveStopPositionVariableName = "nowhere",
            },
            new ProgramStep   // steps[1]: a Number list is not a points list
            {
                Id = "m2", Type = StepType.JumpL, JumpZ = 50,
                MoveStopCondition = Cond("$stb.in1 == 1"), MoveStopPositionVariableName = "samples",
            },
            new ProgramStep   // steps[2]: an aux move records a number, not a points list
            {
                Id = "a1", Type = StepType.AuxMove, AuxSteps = 100,
                MoveStopCondition = Cond("$stb.in1 == 1"), MoveStopPositionVariableName = "contact",
            },
            new ProgramStep   // steps[3]: condition ignored without a wait (warning)
            {
                Id = "a2", Type = StepType.AuxMove, AuxSteps = 100, AuxWaitForDone = false,
                MoveStopCondition = Cond("$stb.in1 == 1"),
            },
            new ProgramStep   // steps[4]: mode / flag without any rows (warning)
            {
                Id = "m3", Type = StepType.MoveL, MoveStopMode = "stop", MoveStopFlagVariableName = "flag",
            });

        var problems = Validate(p);
        string Only(string path, string field) =>
            Assert.Single(problems, x => x.StepPath == path && x.Field == field).Code;

        Assert.Equal(ValidationCodes.MissingField,    Only("steps[0]", "moveStopMode"));
        Assert.Equal(ValidationCodes.UnknownVariable, Only("steps[0]", "moveStopPositionVariableName"));
        Assert.Equal(ValidationCodes.UnknownVariable, Only("steps[1]", "moveStopPositionVariableName"));
        Assert.Equal(ValidationCodes.UnknownVariable, Only("steps[2]", "moveStopPositionVariableName"));

        var noWait = Assert.Single(problems, x => x.StepPath == "steps[3]");
        Assert.Equal(ValidationSeverity.Warning, noWait.Severity);
        Assert.Contains("auxWaitForDone", noWait.Message);

        var noRows = Assert.Single(problems, x => x.StepPath == "steps[4]");
        Assert.Equal(ValidationSeverity.Warning, noRows.Severity);
        Assert.Equal("moveStopCondition", noRows.Field);
        Assert.Contains("no rows", noRows.Message);

        Assert.Contains("'stop' or 'record'", problems.Single(x => x.Field == "moveStopMode").Message);
        Assert.Equal(6, problems.Count);
    }

    // ── MotionDispatcher watch lifecycle ──────────────────────────────────────

    private static ProgramStep WatchedMove(string mode = "stop") => new()
    {
        Id = "m", Type = StepType.MoveL, MoveStopCondition = Cond("1 == 1"), MoveStopMode = mode,
        MoveStopPositionVariableName = "$contact", MoveStopFlagVariableName = "hit",
    };

    [Fact]
    public void DispatcherKeepsTheWatchAcrossStopAndContinue()
    {
        var robot = new RobotController();
        var motion = new MotionDispatcher(robot);

        var step  = WatchedMove("record");
        var watch = MoveStopConditions.Create(step, isAux: false)!;
        Assert.Equal(MoveStopMode.Record, watch.Mode);
        Assert.Equal("contact", watch.PositionVar);   // "$" stripped
        Assert.Equal("hit",     watch.FlagVar);
        Assert.False(watch.IsAux);

        motion.Enqueue(new RobotCommand { CommandType = "MoveL" });
        motion.AwaitingMove = true;
        motion.PendingStep  = step;
        motion.ArmWatch(watch);
        watch.Fired = true;   // the condition was seen true before the user pressed Stop

        motion.CaptureResumeSnapshot();
        Assert.False(motion.AwaitingMove);
        Assert.Null(motion.Watch);          // nothing is in flight while paused

        motion.ArmResume();
        Assert.True(motion.ResumeDispatchPending);
        motion.DispatchResume();
        Assert.True(motion.AwaitingMove);
        Assert.Same(step, motion.PendingStep);
        Assert.Same(watch, motion.Watch);   // the very same watch, Fired included
        Assert.True(motion.Watch!.Fired);

        motion.ClearWatch();
        Assert.Null(motion.Watch);
    }

    [Fact]
    public void DispatcherLeavesAnAuxWatchArmedOnStopAndClearsEverythingOnResetOrPause()
    {
        var robot = new RobotController();
        var motion = new MotionDispatcher(robot);

        var aux = MoveStopConditions.Create(new ProgramStep
        {
            Id = "a", Type = StepType.AuxMove, MoveStopCondition = Cond("1 == 1"), MoveStopMode = "bogus",
        }, isAux: true, "dev-1", 2, 4000)!;
        Assert.Equal(MoveStopMode.Stop, aux.Mode);   // unknown mode falls back to stop
        Assert.True(aux.IsAux);
        Assert.Equal(("dev-1", 2, 4000), (aux.AuxDeviceId, aux.AuxAxis, aux.AuxDecelHz));

        motion.AwaitingAux = true;
        motion.ArmWatch(aux);
        motion.CaptureResumeSnapshot();           // aux waits stay armed on a user Stop …
        Assert.True(motion.AwaitingAux);
        Assert.Same(aux, motion.Watch);           // … and so does their watch

        motion.ClearWaitsForPause();              // a PauseProgram step drops every wait
        Assert.Null(motion.Watch);

        motion.ArmWatch(aux);
        motion.Reset();
        Assert.Null(motion.Watch);
        Assert.False(motion.ResumeDispatchPending);

        Assert.Null(MoveStopConditions.Create(new ProgramStep { Id = "x", Type = StepType.MoveL }, isAux: false));
    }

    // ── Executor wiring ───────────────────────────────────────────────────────

    [Fact]
    public void StopModeFiresWritesThePositionAndFlagAndCompletesTheStep()
    {
        var tmp = Path.Combine(Path.GetTempPath(), "rms-movestop-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(tmp);
        try
        {
            var robot    = new RobotController();
            var programs = new ProgramCycleManager();
            var points   = new PointRepository(Path.Combine(tmp, "points.json"), Path.Combine(tmp, "pointsHistory.json"));
            var grids    = new GridRepository(Path.Combine(tmp, "grids.json"));
            var stacks   = new StackRepository(Path.Combine(tmp, "stacks.json"));
            var exec     = new ProgramExecutor(robot, programs, points, robot.toolRepo, robot.localRepo,
                                               robot.builtProgramRepo, grids, stacks);

            // A MoveL to the current position with a condition that is true at once.
            var program = Prog([Points("contact"), Num("hit")], WatchedMove("stop"));
            exec.Start(program);

            exec.Update();   // dispatches the move and arms the watch
            exec.Update();   // evaluates it: fires, records the pose, asks the motion thread to stop
            Assert.True(exec.IsRunning);
            Assert.Equal(1, exec.EvaluateLive("$contact.length"));
            Assert.Equal(0, exec.EvaluateLive("$hit"));   // the flag is written when the move completes

            // No motion thread here: play its part by dropping the queued move, as ExecuteMotionStop does.
            Assert.False(robot.QueuedCommands.IsEmpty);
            while (robot.QueuedCommands.TryDequeue(out _)) { }

            exec.Update();   // move finished → step completed, flag written, watch cleared, frame exhausted
            Assert.Equal(1, exec.EvaluateLive("$hit"));

            exec.Update();   // nothing left → Complete
            Assert.False(exec.IsRunning);
            Assert.Equal(1, exec.EvaluateLive("$hit"));
            Assert.Equal(robot.GetCurrentPosition().Z, exec.EvaluateLive("$contact[0].z"));
        }
        finally
        {
            try { Directory.Delete(tmp, recursive: true); } catch { /* best effort */ }
        }
    }
}
