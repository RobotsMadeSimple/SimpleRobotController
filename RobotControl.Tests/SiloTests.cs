using System.Diagnostics;
using System.Reflection;
using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.AuxAxis;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Language;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.Validation;

namespace RobotControl.Tests;

/// <summary>
/// The Silo aux role (docs/aux-silo.md): present-position math, the SiloAction step driven tick by
/// tick against a simulated (never connected) aux device, the persisted count, the text form and
/// the SiloAction WebSocket command.
/// </summary>
[Collection("RobotController")]
public sealed class SiloTests : IDisposable
{
    private const string Dev = "AUX_SILOTEST";

    private readonly string              _tmp;
    private readonly RobotController     _robot = new RobotController().WithIdentityKinematics();
    private readonly ProgramCycleManager _programs = new();
    private readonly ProgramExecutor     _exec;
    private readonly AuxAxisDevice       _aux;
    private long                         _clock = 1_000_000;

    public SiloTests()
    {
        _tmp = Path.Combine(Path.GetTempPath(), "rms-silo-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(_tmp);

        // A private aux manager (its aux_config.json in the temp dir) so a count write never lands
        // in the shared working-directory config.
        var mgr = new AuxAxisManager(Path.Combine(_tmp, "aux_config.json"));
        typeof(RobotController).GetProperty(nameof(RobotController.AuxAxisManager))!.SetValue(_robot, mgr);

        var cfg = new AuxAxisConfig { Id = Dev, Name = "Silo Test", Axes = { Silo(0), Silo(1, invert: true) } };
        mgr.GetConfig().Devices.Add(cfg);
        _aux = new AuxAxisDevice(cfg, simTimer: false);   // offline simulation, driven by SimStep
        Devices(mgr).Add(_aux);

        var points = new PointRepository(Path.Combine(_tmp, "points.json"), Path.Combine(_tmp, "pointsHistory.json"));
        var grids  = new GridRepository(Path.Combine(_tmp, "grids.json"));
        var stacks = new StackRepository(Path.Combine(_tmp, "stacks.json"));
        _exec = new ProgramExecutor(_robot, _programs, points, _robot.toolRepo, _robot.localRepo,
                                    _robot.builtProgramRepo, grids, stacks);
    }

    public void Dispose()
    {
        _aux.Stop();
        try { Directory.Delete(_tmp, recursive: true); } catch { /* best effort */ }
    }

    // 200 steps/rev × 1 / 40 mm per rev = 5 steps/mm; the checker silo numbers.
    private static AuxAxisChannelConfig Silo(int axis, bool invert = false) => new()
    {
        AxisIndex = axis, Name = $"Silo {axis}", StepsPerRev = 200, InvertDirection = invert,
        AxisType = AuxAxisType.Linear, MmPerRev = 40, Role = AuxAxisRole.Silo,
        SiloPitchMm = 3.9, SiloCapacity = 12, SiloClearanceMm = 0.5, SiloHomeOverTravelMm = 5,
        SiloHomeSpeed = 5, ActionVelocity = 20, SiloCount = 12,
    };

    private static List<AuxAxisDevice> Devices(AuxAxisManager mgr) =>
        (List<AuxAxisDevice>)typeof(AuxAxisManager).GetField("_devices", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(mgr)!;

    private AuxAxisManager Mgr => _robot.AuxAxisManager;
    private AuxAxisChannelConfig Cfg(int axis = 0) => Mgr.GetAxisConfig(Dev, axis)!;

    /// <summary>Runs the simulated axis to the end of its move (one long sim tick).</summary>
    private void FinishSimMove()
    {
        _aux.SimStep(_clock);
        _clock += Stopwatch.Frequency * 1000L;
        _aux.SimStep(_clock);
    }

    private static int _ids;
    private static ProgramStep SiloStep(string action, int axis = 0, int? count = null) => new()
    {
        Id = $"silo{++_ids}", Type = StepType.SiloAction, SiloAction = action, SiloCount = count,
        AuxDeviceId = Dev, AuxAxisIndex = axis,
    };

    private static ProgramStep SetVar(string name, string expr) =>
        new() { Id = $"s{++_ids}", Type = StepType.SetVariable, VariableName = name, VariableExpr = expr };

    private static ProgramVariable Num(string name) => new() { Id = name, Name = name };

    private static BuiltProgram Prog(List<ProgramVariable> vars, params ProgramStep[] steps) =>
        new() { Id = "main", Name = "Main", Steps = steps.ToList(), Variables = vars };

    private JsonElement Summary() =>
        JsonSerializer.SerializeToElement(_programs.GetProgramsSummary()).EnumerateArray().First(p => p.GetProperty("name").GetString() == "Main");

    private string Status() => Summary().GetProperty("status").GetString()!;
    private string ErrorDescription() => Summary().GetProperty("errorDescription").GetString() ?? "";

    private void Run(int maxTicks = 20)
    {
        for (int i = 0; i < maxTicks && _exec.IsRunning; i++) { _exec.Update(); FinishSimMove(); }
        Assert.False(_exec.IsRunning, "program did not finish");
    }

    // ── Math ──────────────────────────────────────────────────────────────────

    [Fact]
    public void PresentPositionLiftsTheTopPartToTheRim()
    {
        var c = Silo(0);
        Assert.Equal(47.3, c.SiloTravelMm(), 9);
        Assert.Equal(47.3, c.SiloPresentMm(1), 9);             // last part: full travel
        Assert.Equal(47.3 - 11 * 3.9, c.SiloPresentMm(12), 9); // full stack: 4.4 mm
        Assert.Equal(5, c.StepsPerUnit());
        Assert.Equal((long)Math.Round(4.4 * 5), SiloActions.PresentSteps(c, 12));
        Assert.Equal(-(long)Math.Round((47.3 + 5) * 5), SiloActions.HomeSteps(c));
    }

    // ── Home / Present through the executor ──────────────────────────────────

    [Fact]
    public void HomeMovesDownThenZeroesAndMarksHomed()
    {
        _aux.SetPosition(0, 1234);
        _exec.Start(Prog([], SiloStep("Home")));

        _exec.Update();                                      // dispatched
        Assert.True(_robot.IsAuxMoving);
        Assert.False(Mgr.IsSiloHomed(Dev, 0));
        FinishSimMove();
        Assert.Equal(1234 + SiloActions.HomeSteps(Cfg()), Mgr.GetPosition(Dev, 0)); // drove down, not yet zeroed
        _exec.Update();                                      // the wait ends: zero + homed
        Assert.Equal(0, Mgr.GetPosition(Dev, 0));
        Assert.True(Mgr.IsSiloHomed(Dev, 0));
        Run();
        Assert.Equal("Complete", Status());
        Assert.Equal(12, Cfg().SiloCount);                   // home leaves the count alone
    }

    [Fact]
    public void PresentGoesToTheAbsoluteTargetAndTakesOnePart()
    {
        _exec.Start(Prog([Num("n"), Num("h")], SiloStep("Home"), SiloStep("Present"), SiloStep("Present"),
            SetVar("n", $"$aux.{Dev}.0.siloCount"), SetVar("h", $"$aux.{Dev}.0.siloHomed")));
        Run();
        Assert.Equal("Complete", Status());
        Assert.Equal(10, Cfg().SiloCount);
        Assert.Equal(SiloActions.PresentSteps(Cfg(), 11), Mgr.GetPosition(Dev, 0));
        Assert.True(Mgr.IsSiloHomed(Dev, 0));                // its own moves keep it homed
        Assert.Equal(10, _exec.EvaluateLive("$n"));
        Assert.Equal(1, _exec.EvaluateLive("$h"));
    }

    [Fact]
    public void PresentWithoutHomingFailsSiloNotHomedAndIsCatchable()
    {
        _exec.Start(Prog([], SiloStep("Present")));
        Run();
        Assert.Equal("Error", Status());
        Assert.Contains("not homed", ErrorDescription());
        Assert.Equal(12, Cfg().SiloCount);

        var tryStep = new ProgramStep
        {
            Id = "try1", Type = StepType.Try, TrySteps = [SiloStep("Present")],
            CatchCodes = ["siloNotHomed"], CatchSteps = [SetVar("caught", "1")],
        };
        _exec.Start(Prog([Num("caught")], tryStep));
        Run();
        Assert.Equal("Complete", Status());
        Assert.Equal(1, _exec.EvaluateLive("$caught"));
    }

    [Fact]
    public void PresentOnAnEmptySiloFailsSiloEmptyAndIsCatchable()
    {
        var tryStep = new ProgramStep
        {
            Id = "try2", Type = StepType.Try, TrySteps = [SiloStep("Present")],
            CatchCodes = ["siloEmpty"], CatchSteps = [SetVar("caught", "1")],
        };
        _exec.Start(Prog([Num("caught")], SiloStep("Home"), SiloStep("SetCount", count: 0), tryStep));
        Run();
        Assert.Equal("Complete", Status());
        Assert.Equal(1, _exec.EvaluateLive("$caught"));
        Assert.Equal(0, Mgr.GetPosition(Dev, 0));            // nothing moved
    }

    [Fact]
    public void AcceptLowersToTheNextSlotAndAddsOnePart()
    {
        _exec.Start(Prog([Num("n")],
            SiloStep("Home"), SiloStep("SetCount", count: 3), SiloStep("Accept"),
            SetVar("n", $"$aux.{Dev}.0.siloCount")));
        Run();
        Assert.Equal("Complete", Status());
        Assert.Equal(4, Cfg().SiloCount);                                         // the part was added
        Assert.Equal(SiloActions.PresentSteps(Cfg(), 4), Mgr.GetPosition(Dev, 0)); // present height for the new part
        Assert.True(Mgr.IsSiloHomed(Dev, 0));
        Assert.Equal(4, _exec.EvaluateLive("$n"));
    }

    [Fact]
    public void AcceptOnAFullSiloFailsSiloFullAndIsCatchable()
    {
        var tryStep = new ProgramStep
        {
            Id = "tryfull", Type = StepType.Try, TrySteps = [SiloStep("Accept")],
            CatchCodes = ["siloFull"], CatchSteps = [SetVar("caught", "1")],
        };
        // Home marks it homed; the count stays at capacity (12), so Accept has nowhere to put a part.
        _exec.Start(Prog([Num("caught")], SiloStep("Home"), tryStep));
        Run();
        Assert.Equal("Complete", Status());
        Assert.Equal(1, _exec.EvaluateLive("$caught"));
        Assert.Equal(12, Cfg().SiloCount);
        Assert.Equal(0, Mgr.GetPosition(Dev, 0));             // nothing moved (still at home)
        Assert.True(Controller.RobotControl.Programs.ErrorCodes.IsCatchable("siloFull"));
    }

    [Fact]
    public void AcceptWithoutHomingFailsSiloNotHomed()
    {
        _exec.Start(Prog([], SiloStep("SetCount", count: 3), SiloStep("Accept")));
        Run();
        Assert.Equal("Error", Status());
        Assert.Contains("not homed", ErrorDescription());
        Assert.Equal(3, Cfg().SiloCount);                     // unchanged
    }

    [Fact]
    public void AnyOtherMoveOrZeroClearsHomed()
    {
        _exec.Start(Prog([], SiloStep("Home")));
        Run();
        Assert.True(Mgr.IsSiloHomed(Dev, 0));
        _robot.StartAuxMove(Dev, 0, 10, 100, 100, 100);       // a manual jog / AuxMove
        FinishSimMove();
        Assert.False(Mgr.IsSiloHomed(Dev, 0));

        _exec.Start(Prog([], SiloStep("Home")));
        Run();
        Assert.True(Mgr.IsSiloHomed(Dev, 0));
        Mgr.ZeroPosition(Dev, 0);
        Assert.False(Mgr.IsSiloHomed(Dev, 0));
        Assert.False(Mgr.IsSiloHomed(Dev, 1));                // per axis
    }

    [Fact]
    public void AHomeStoppedPartWayDoesNotZeroOrMarkHomed()
    {
        _aux.SetPosition(0, 500);
        _exec.Start(Prog([], SiloStep("Home")));
        _exec.Update();
        Mgr.StopSmooth(Dev, 0, 1000);                         // the sim move ends at once, short of its end
        _exec.Update();
        Assert.Equal(500, Mgr.GetPosition(Dev, 0));
        Assert.False(Mgr.IsSiloHomed(Dev, 0));
    }

    [Fact]
    public void RefillFillsTheSiloAndHomesIt()
    {
        Mgr.SetSiloCount(Dev, 0, 3);
        _aux.SetPosition(0, 200);
        _exec.Start(Prog([], SiloStep("Refill")));
        Run();
        Assert.Equal("Complete", Status());
        Assert.Equal(12, Cfg().SiloCount);
        Assert.True(Mgr.IsSiloHomed(Dev, 0));
        Assert.Equal(0, Mgr.GetPosition(Dev, 0));
    }

    [Fact]
    public void SetCountIsClampedAndTakesAnExpression()
    {
        var step = SiloStep("SetCount");
        step.Expressions = new() { ["siloCount"] = "2 + 3" };
        _exec.Start(Prog([], step, SiloStep("SetCount", axis: 1, count: 99)));
        Run();
        Assert.Equal(5, Cfg(0).SiloCount);
        Assert.Equal(12, Cfg(1).SiloCount);                  // clamped to capacity
        Assert.False(_robot.IsAuxMoving);
    }

    [Fact]
    public void AnAxisThatIsNotASiloFailsSiloConfig()
    {
        Cfg(1).Role = AuxAxisRole.Generic;
        _exec.Start(Prog([], SiloStep("Home", axis: 1)));
        Run();
        Assert.Equal("Error", Status());
        Assert.Contains("not set up as a silo", ErrorDescription());
        Assert.False(Controller.RobotControl.Programs.ErrorCodes.IsCatchable("siloConfig"));
        Assert.True(Controller.RobotControl.Programs.ErrorCodes.IsCatchable("siloNotHomed"));
        Assert.True(Controller.RobotControl.Programs.ErrorCodes.IsCatchable("siloEmpty"));
    }

    [Fact]
    public void TheInvertedSiloHomesTheOtherMotorDirection()
    {
        // Positive = platform up for both; the inverted one's motor turns CCW for "up", so its
        // home (negative) move turns CW. The tracked position is in "up" steps for both.
        _exec.Start(Prog([], SiloStep("Home", axis: 1), SiloStep("Present", axis: 1)));
        Run();
        Assert.Equal(-SiloActions.PresentSteps(Cfg(1), 12), Mgr.GetPosition(Dev, 1));   // motor steps run the other way
        Assert.Equal(SiloActions.PresentSteps(Cfg(1), 12), SiloActions.UpSteps(Mgr, Cfg(1), Dev, 1));
        Assert.Equal(11, Cfg(1).SiloCount);
    }

    // ── Persistence ──────────────────────────────────────────────────────────

    [Fact]
    public void SiloCountAndGeometryPersistButHomedDoesNot()
    {
        var path = Path.Combine(_tmp, "persist_aux.json");
        var a = new AuxAxisManager(path);
        var axis = a.GetConfig().Devices[0].Axes[0];
        var dev  = a.GetConfig().Devices[0].Id;
        a.UpdateAxisConfig(dev, 0, Silo(0));
        Assert.Equal(7, a.SetSiloCount(dev, 0, 7));
        Assert.Equal(12, a.SetSiloCount(dev, 0, 40));          // clamped to capacity
        Assert.Equal(0, a.SetSiloCount(dev, 0, -3));
        a.SetSiloCount(dev, 0, 9);

        // A config save (geometry) keeps the count.
        var patch = Silo(0); patch.SiloCount = 0; patch.SiloPitchMm = 4;
        a.UpdateAxisConfig(dev, 0, patch);
        Assert.Equal(9, axis.SiloCount);

        var b = new AuxAxisManager(path);
        var reloaded = b.GetAxisConfig(dev, 0)!;
        Assert.Equal(9, reloaded.SiloCount);
        Assert.Equal(AuxAxisRole.Silo, reloaded.Role);
        Assert.Equal(4, reloaded.SiloPitchMm);
        Assert.Equal(12, reloaded.SiloCapacity);
        Assert.False(b.IsSiloHomed(dev, 0));
        Assert.Contains("\"siloCount\": 9", File.ReadAllText(path));
        Assert.Contains("\"role\": \"Silo\"", File.ReadAllText(path));
        Assert.DoesNotContain("siloHomed", File.ReadAllText(path));
    }

    // ── Text form ────────────────────────────────────────────────────────────

    [Fact]
    public void SiloActionTextRoundTrips()
    {
        var names = Language.SrsTestKit.Names();
        const string src = "var n = 0\nSiloAction home, device=Aux, axis=1\nSiloAction present, device=Aux, axis=1\nSiloAction refill\n"
                         + "SiloAction setcount, count=$n + 1\nSiloAction setcount, count=4\n";
        var parsed = SrsLanguage.Parse(src, null, names);
        Assert.True(parsed.Ok, string.Join("; ", parsed.Problems.Select(p => p.Message)));
        var steps = parsed.Program!.Steps;
        Assert.Equal(5, steps.Count);
        Assert.All(steps, s => Assert.Equal(StepType.SiloAction, s.Type));
        Assert.Equal(new[] { "Home", "Present", "Refill", "SetCount", "SetCount" }, steps.Select(s => s.SiloAction ?? "").ToArray());
        Assert.Equal("aux-1", steps[0].AuxDeviceId);   // linked by the device's id
        Assert.Equal(1, steps[1].AuxAxisIndex);
        Assert.Equal("$n + 1", steps[3].Expressions!["siloCount"]);
        Assert.Equal(4, steps[4].SiloCount);

        var text = Language.SrsTestKit.AssertRoundTrip(parsed.Program!, names);
        Assert.Contains("SiloAction home, device=Aux, axis=1", text);
        Assert.Contains("SiloAction setcount, count=4", text);
    }
    [Fact]
    public void TheValidatorChecksTheActionAndTheCount()
    {
        var bad = SiloStep("Lift");
        var noCount = SiloStep("SetCount");
        var problems = ProgramValidator.Validate(Prog([], bad, noCount, SiloStep("Present")), ValidationContext.Offline());
        Assert.Contains(problems, p => p.StepId == bad.Id && p.Field == "siloAction");
        Assert.Contains(problems, p => p.StepId == noCount.Id && p.Field == "siloCount");
        Assert.Equal(2, problems.Count(p => p.Severity == ValidationSeverity.Error));
    }

    // ── WebSocket ────────────────────────────────────────────────────────────

    private JsonElement Send(string command, object parameters)
    {
        var msg = new CommandMessage
        {
            Type = "Command", Id = "t", Command = command, Params = JsonSerializer.SerializeToElement(parameters),
        };
        return JsonSerializer.SerializeToElement(_robot.AddCommand(msg).GetAwaiter().GetResult());
    }

    [Fact]
    public void SiloActionCommandAcksAtOnceAndFinishesWhenTheLiftStops()
    {
        var r = Send("SiloAction", new { deviceId = Dev, axis = 0, action = "Present" });
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.Equal("siloNotHomed", r.GetProperty("error").GetString());

        r = Send("SiloAction", new { deviceId = Dev, axis = 0, action = "Home" });
        Assert.True(r.GetProperty("moving").GetBoolean());
        Assert.False(r.GetProperty("siloHomed").GetBoolean());
        FinishSimMove();
        Assert.True(SpinUntil(() => Mgr.IsSiloHomed(Dev, 0)), "home never finished");
        Assert.Equal(0, Mgr.GetPosition(Dev, 0));

        r = Send("SiloAction", new { deviceId = Dev, axis = 0, action = "Present" });
        Assert.True(r.GetProperty("moving").GetBoolean());
        FinishSimMove();
        Assert.True(SpinUntil(() => Cfg().SiloCount == 11), "present never finished");

        r = Send("SiloAction", new { deviceId = Dev, axis = 0, action = "setcount", count = 3 });
        Assert.False(r.GetProperty("moving").GetBoolean());
        Assert.Equal(3, r.GetProperty("siloCount").GetInt32());
        Assert.True(r.GetProperty("siloHomed").GetBoolean());

        // The aux state carries both.
        var state = JsonDocument.Parse(JsonSerializer.Serialize(Mgr.GetState())).RootElement
            .EnumerateArray().First(d => d.GetProperty("deviceId").GetString() == Dev).GetProperty("axes")[0];
        Assert.Equal(3, state.GetProperty("siloCount").GetInt32());
        Assert.True(state.GetProperty("siloHomed").GetBoolean());

        r = Send("SiloAction", new { deviceId = Dev, axis = 0, action = "Spin" });
        Assert.Equal("siloConfig", r.GetProperty("error").GetString());
    }

    private static bool SpinUntil(Func<bool> cond, int ms = 3000)
    {
        var sw = Stopwatch.StartNew();
        while (sw.ElapsedMilliseconds < ms) { if (cond()) return true; Thread.Sleep(10); }
        return cond();
    }
}
