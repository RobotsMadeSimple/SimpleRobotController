using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Programs;
using Controller.RobotControl.Safety;
using static RobotControl.Tests.FlowTestKit;

namespace RobotControl.Tests;

/// <summary>
/// The hardwired e-stop (docs/estop.md): the latch/arm/reset state machine on its own, then the
/// controller's reaction through the test seam (EstopReaderOverride — there is no STB or Nano in a
/// test): commands refused while active or latched, reset refused until the input is healthy, the
/// robot unhomed, and a running program ended with the uncatchable emergencyStop.
/// </summary>
[Collection("RobotController")]
public class EstopTests
{
    private static readonly EstopConfig Nc = new() { Enabled = true, Source = EstopConfig.SourceStb, StbInput = 4 };
    private static readonly EstopConfig No = new() { Enabled = true, Source = EstopConfig.SourceStb, StbInput = 4, Polarity = EstopConfig.PolarityNormallyOpen };

    private static EstopReading In(bool? raw, bool available = true) => new(available, raw, "STB input 4");

    // ── State machine ────────────────────────────────────────────────────────

    [Fact]
    public void NormallyClosedTripLatchesUntilResetAndResetNeedsAHealthyInput()
    {
        var m = new EstopMonitor();
        Assert.False(m.Evaluate(Nc, In(true), 1));           // healthy: arms
        Assert.False(m.State.Active);

        Assert.True(m.Evaluate(Nc, In(false), 2));            // circuit opens: one trip
        Assert.False(m.Evaluate(Nc, In(false), 3));           // still open: no second trip
        Assert.True(m.State.Active);
        Assert.True(m.State.Latched);
        Assert.Equal(2, m.State.TrippedUnixMs);
        Assert.Contains("open", m.State.Reason);

        Assert.NotNull(m.TryReset(Nc, In(false)));            // still pressed → refused
        Assert.True(m.State.Latched);

        m.Evaluate(Nc, In(true), 4);                          // released: no longer active, still latched
        Assert.False(m.State.Active);
        Assert.True(m.State.Latched);
        Assert.True(m.Inhibits);

        Assert.Null(m.TryReset(Nc, In(true)));
        Assert.False(m.State.Latched);
        Assert.False(m.Inhibits);

        Assert.True(m.Evaluate(Nc, In(false), 5));            // re-armed by the reset: trips again
    }

    [Fact]
    public void NormallyOpenTripsOnAClosedInput()
    {
        var m = new EstopMonitor();
        m.Evaluate(No, In(false), 1);
        Assert.False(m.State.Active);
        Assert.True(m.Evaluate(No, In(true), 2));
        Assert.Contains("closed", m.State.Reason);
    }

    [Fact]
    public void BootingAlreadyTrippedBlocksWithoutLatching()
    {
        var m = new EstopMonitor();
        // Source not connected yet, then connected with the button pressed: never armed.
        Assert.False(m.Evaluate(Nc, In(null, available: false), 1));
        Assert.False(m.Evaluate(Nc, In(false), 2));
        Assert.True(m.State.Active);
        Assert.False(m.State.Latched);
        Assert.True(m.Inhibits);

        m.Evaluate(Nc, In(true), 3);                          // released: healthy, nothing to reset
        Assert.False(m.Inhibits);
    }

    [Fact]
    public void SourceGoingOfflineAfterBeingHealthyIsATrip()
    {
        var m = new EstopMonitor();
        m.Evaluate(Nc, In(true), 1);
        Assert.True(m.Evaluate(Nc, In(null, available: false), 2));
        Assert.Contains("offline", m.State.Reason);
        Assert.Contains("offline", m.TryReset(Nc, In(null, available: false)));
    }

    [Fact]
    public void DisabledNeverTripsAndALeftoverLatchResets()
    {
        var m = new EstopMonitor();
        m.Evaluate(Nc, In(true), 1);
        m.Evaluate(Nc, In(false), 2);
        Assert.True(m.State.Latched);

        var off = Nc.Clone(); off.Enabled = false;
        Assert.False(m.Evaluate(off, In(false), 3));
        Assert.False(m.State.Active);
        Assert.True(m.State.Latched);                         // still needs a reset…
        Assert.Null(m.TryReset(off, In(false)));              // …which succeeds with nothing to check
        Assert.False(m.Inhibits);
    }

    [Fact]
    public void ConfigValidation()
    {
        Assert.Null(Nc.Validate());
        Assert.NotNull(new EstopConfig { Source = "x" }.Validate());
        Assert.NotNull(new EstopConfig { StbInput = 5 }.Validate());
        Assert.NotNull(new EstopConfig { Polarity = "x" }.Validate());
        Assert.NotNull(new EstopConfig { Enabled = true, Source = EstopConfig.SourceIo }.Validate());
        Assert.Null(new EstopConfig { Enabled = true, Source = EstopConfig.SourceIo, DeviceId = "nano1", Pin = 7 }.Validate());
    }

    // ── Controller reaction ──────────────────────────────────────────────────

    private sealed class Rig
    {
        public RobotController Robot { get; } = new();
        public bool? Raw = true;
        public bool Available = true;
        public long Now = 1_000;

        public Rig()
        {
            Robot.UseAlarmHistoryForTest(TempAlarms());
            Robot.Config.Estop = Nc.Clone();
            Robot.EstopReaderOverride = () => new EstopReading(Available, Available ? Raw : null, "STB input 4");
            Robot.SafetyClock = () => Now += 200; // every tick also runs the alarm watch
            Robot.SafetyTick();
        }

        public JsonElement Send(string command, object? parameters = null)
        {
            var msg = new CommandMessage
            {
                Type = "Command", Id = "t", Command = command,
                Params = JsonSerializer.SerializeToElement(parameters ?? new { }),
            };
            return JsonSerializer.SerializeToElement(Robot.AddCommand(msg).GetAwaiter().GetResult());
        }
    }

    private static Controller.RobotControl.Alarms.AlarmHistory TempAlarms() =>
        new(Path.Combine(Path.GetTempPath(), "rms-alarms-" + Guid.NewGuid().ToString("N") + ".json"));

    private static string? Error(JsonElement r) =>
        r.ValueKind == JsonValueKind.Object && r.TryGetProperty("error", out var e) ? e.GetString() : null;

    [Fact]
    public void TripRefusesMotionAndProgramStartsUntilReset()
    {
        var rig = new Rig();
        var robot = rig.Robot;
        Assert.Null(robot.SafetyGate("MoveL"));

        rig.Raw = false;          // button pressed
        robot.SafetyTick();
        Assert.True(robot.Estop.State.Latched);

        foreach (var cmd in new[] { "MoveL", "MoveJ", "JogJ", "OffsetL", "Home", "SetHomed", "ExecuteBuiltProgram", "StartBackgroundProgram", "MoveAux" })
            Assert.Equal("estopActive", Error(rig.Send(cmd, new { name = "x" })));
        // Reads, stops and configuration still work.
        Assert.Null(Error(rig.Send("GetStatus")));
        Assert.Null(robot.SafetyGate("StopJog"));
        Assert.Null(robot.SafetyGate("HardStop"));

        // Reset is refused while the button is still pressed.
        var refused = rig.Send("ResetEstop");
        Assert.Equal("estopNotHealthy", Error(refused));

        // Released: no longer active, but latched until reset.
        rig.Raw = true;
        robot.SafetyTick();
        Assert.Equal("estopLatched", Error(rig.Send("MoveL", new { x = 1 })));
        Assert.Equal("estopLatched", Error(rig.Send("Home")));

        var ok = rig.Send("ResetEstop");
        Assert.True(ok.GetProperty("ok").GetBoolean());
        Assert.Null(robot.SafetyGate("MoveL"));
        Assert.Null(robot.SafetyGate("Home"));
    }

    [Fact]
    public void TripUnhomesTheRobotAndRaisesTheAlarm()
    {
        var rig = new Rig();
        var robot = rig.Robot;
        for (int j = 0; j < 4; j++) robot.SetJointPosition(j, 0);
        Assert.True(robot.Homed);
        int alarmsBefore = robot.Alarms.ActiveCount;

        rig.Raw = false;
        robot.SafetyTick();
        robot.RunPostedMotionActionsForTest(); // the motion thread would drain this on its next tick
        Assert.False(robot.Homed);
        Assert.True(robot.HardStopPendingForTest);

        var page = robot.Alarms.GetPage(0, 10, activeOnly: true).Page;
        var estop = Assert.Single(page, a => a.Code == ErrorCodes.EmergencyStop && !a.Cleared);
        Assert.Equal("critical", estop.Severity);
        Assert.Equal(alarmsBefore + 1, robot.Alarms.ActiveCount);

        // Reset clears the condition; the alarm stays listed until acknowledged.
        rig.Raw = true;
        robot.SafetyTick();
        Assert.Null(robot.ResetEstop());
        var after = robot.Alarms.GetPage(0, 50).Page.Single(a => a.Id == estop.Id);
        Assert.True(after.Cleared);
        Assert.True(after.Active);
        robot.Alarms.Acknowledge(estop.Id);
        Assert.False(robot.Alarms.GetPage(0, 50).Page.Single(a => a.Id == estop.Id).Active);
    }

    [Fact]
    public void TurningMonitoringOnWhileTrippedReactsWithoutLatching()
    {
        var rig = new Rig();
        var robot = rig.Robot;
        robot.Config.Estop.Enabled = false;
        rig.Raw = false;
        robot.SafetyTick();
        for (int j = 0; j < 4; j++) robot.SetJointPosition(j, 0);
        Assert.True(robot.Homed);

        robot.Config.Estop.Enabled = true;   // enabled while the circuit is open
        robot.SafetyTick();
        robot.RunPostedMotionActionsForTest();
        var s = robot.Estop.State;
        Assert.True(s.Active);
        Assert.False(s.Latched);             // never armed: nothing to reset
        Assert.False(robot.Homed);           // but the drivers may be off: unhomed
        Assert.Equal("estopActive", Error(rig.Send("JogL", new { x = 1 })));
        var alarm = Assert.Single(robot.Alarms.GetPage(0, 10).Page, a => a.Code == ErrorCodes.EmergencyStop);
        Assert.False(alarm.Cleared);

        rig.Raw = true;                      // input healthy again: active clears, alarm condition too
        robot.SafetyTick();
        Assert.False(robot.Estop.Inhibits);
        Assert.True(robot.Alarms.GetPage(0, 10).Page.Single(a => a.Id == alarm.Id).Cleared);
    }

    [Fact]
    public void BootingWithTheSourceOfflineRaisesNoAlarm()
    {
        var robot = new RobotController();
        robot.UseAlarmHistoryForTest(TempAlarms());
        robot.Config.Estop = Nc.Clone();
        robot.EstopReaderOverride = () => new EstopReading(false, null, "STB input 4");
        robot.SafetyTick();
        Assert.True(robot.Estop.State.Active);
        Assert.Equal(0, robot.Alarms.Count);
        Assert.NotNull(robot.SafetyGate("MoveL"));
    }

    [Fact]
    public void StatusCarriesTheEstopFields()
    {
        var rig = new Rig();
        rig.Raw = false;
        rig.Robot.SafetyTick();
        var s = rig.Send("GetStatus");
        Assert.True(s.GetProperty("estopEnabled").GetBoolean());
        Assert.True(s.GetProperty("estopActive").GetBoolean());
        Assert.True(s.GetProperty("estopLatched").GetBoolean());
        Assert.False(s.GetProperty("estopInputHealthy").GetBoolean());
        Assert.Equal("STB input 4", s.GetProperty("estopSource").GetString());
        Assert.True(s.GetProperty("activeAlarmCount").GetInt32() >= 1);
        Assert.True(s.GetProperty("lastAlarmUpdate").GetInt64() > 0);
    }

    [Fact]
    public void SetEstopConfigValidatesAndGetReturnsTheLiveInput()
    {
        var rig = new Rig();
        var bad = rig.Send("SetEstopConfig", new { source = "io", enabled = true });
        Assert.Equal("badConfig", Error(bad));
        var badInput = rig.Send("SetEstopConfig", new { stbInput = 9 });
        Assert.Equal("badConfig", Error(badInput));

        var get = rig.Send("GetEstopConfig");
        Assert.True(get.GetProperty("config").GetProperty("enabled").GetBoolean());
        Assert.Equal("normallyClosed", get.GetProperty("config").GetProperty("polarity").GetString());
        Assert.True(get.GetProperty("state").GetProperty("input").GetBoolean());
        Assert.True(get.GetProperty("state").GetProperty("inputHealthy").GetBoolean());
    }

    [Fact]
    public void AbortWithErrorEndsARunningProgramUncatchably()
    {
        using var k = new FlowTestKit();
        // A Try around a long wait: the e-stop must not be caught by it.
        var wait = Step(StepType.Wait, s => s.WaitMs = 60_000);
        var prog = Prog(Try([wait], [Set("x", "1")]));
        k.Exec.Start(prog);
        for (int i = 0; i < 5; i++) k.Exec.Update();
        Assert.True(k.Exec.IsRunning);

        Assert.True(k.Exec.AbortWithError(ErrorCodes.EmergencyStop, "E-stop pressed"));
        Assert.False(k.Exec.IsRunning);
        Assert.False(k.Exec.IsPaused);
        Assert.Equal("Error", k.Status());
        Assert.Equal("emergencyStop", k.Summary().GetProperty("errorCode").GetString());
        Assert.Equal(0, k.Value("$x"));                       // the catch steps never ran
        Assert.False(k.Exec.AbortWithError(ErrorCodes.EmergencyStop, "again")); // nothing left to abort
    }

    [Fact]
    public void ProgramErrorsAreRecordedAsAlarms()
    {
        using var k = new FlowTestKit();
        k.Robot.UseAlarmHistoryForTest(TempAlarms());
        int before = k.Robot.Alarms.Count;
        k.RunToEnd(Prog(Raise("Part missing", "partMissing")));
        var a = k.Robot.Alarms.GetPage(0, 5).Page.First();
        Assert.Equal(before + 1, k.Robot.Alarms.Count);
        Assert.Equal("partMissing", a.Code);
        Assert.Contains("Part missing", a.Message);
        Assert.Equal("main", a.Source.ProgramId);
        Assert.True(a.Cleared);          // a one-off event…
        Assert.True(a.Active);           // …active until acknowledged
    }
}
