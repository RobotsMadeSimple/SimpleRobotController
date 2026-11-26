using System.Diagnostics;
using System.Reflection;
using System.Text.Json;
using Controller.RobotControl.AuxAxis;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Io;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.Serial;

/// <summary>
/// Aux-stepper live position for conveyor tracking: POS/LAT parsing, the board-counter offset
/// model (continuity, SetPosition, sticky gripper clamp, 32-bit wrap), board→controller clock
/// mapping + extrapolation, the offline simulation's live position, latch config fallback, and
/// the role-action velocity conversion. Lines are fed through HandleLine on a synthetic clock;
/// no serial port is opened and the sim timer is off.
/// </summary>
public class AuxAxisLivePositionTests
{
    private static readonly double TicksPerMs = Stopwatch.Frequency / 1000.0;
    private const double T0Ms = 1_000_000;             // arbitrary controller-clock origin (ms)
    private const double LatencyMs = 5;                 // board → controller transport delay

    private static long Ticks(double ctrlMs) => (long)(ctrlMs * TicksPerMs);

    private static AuxAxisDevice NewDevice(bool posCaps = true, params IoPinConfig[] pins)
    {
        var cfg = new AuxAxisConfig { Id = "AUX_T", Name = "Test", IoPins = pins.ToList() };
        var dev = new AuxAxisDevice(cfg, simTimer: false);
        dev.Firmware = posCaps
            ? new FirmwareInfo("1.2.0", new[] { "stepper", "io", "pos", "latch" })
            : new FirmwareInfo("1.1.0", new[] { "stepper", "io" });
        return dev;
    }

    /// <summary>Feed POS:&lt;boardMs&gt;,&lt;p0&gt;,0,0,0 received LatencyMs later on the controller clock.</summary>
    private static void Pos(AuxAxisDevice d, uint boardMs, long p0, double latency = LatencyMs) =>
        d.HandleLine($"POS:{boardMs},{unchecked((int)p0)},0,0,0", Ticks(T0Ms + boardMs + latency));

    private static List<string> Queue(AuxAxisDevice d) =>
        ((Queue<string>)typeof(SerialLineDevice).GetField("_commandQueue", BindingFlags.NonPublic | BindingFlags.Instance)!
            .GetValue(d)!).ToList();

    // ── Parsing + offset model ───────────────────────────────────────────────

    [Fact]
    public void FirstPosSyncsWithoutAJumpThenCountsAreAuthoritative()
    {
        var d = NewDevice();
        d.SetPosition(0, 500);                         // dead-reckoned position before the board reports
        Pos(d, 1000, 12345);                           // board counter is arbitrary at sync
        Assert.Equal(500, d.GetPosition(0));
        Pos(d, 1020, 12545);
        Assert.Equal(700, d.GetPosition(0));
        d.SetPosition(0, 0);                           // re-reference: offset absorbs it
        Pos(d, 1040, 12645);
        Assert.Equal(100, d.GetPosition(0));
    }

    [Fact]
    public void NegativeCountsAndAllFourAxesParse()
    {
        var d = NewDevice();
        d.HandleLine("POS:10,0,0,0,0", Ticks(T0Ms + 15));
        d.HandleLine("POS:30,-40,7,-3,2000000000", Ticks(T0Ms + 35));
        Assert.Equal(new long[] { -40, 7, -3, 2000000000 }, Enumerable.Range(0, 4).Select(d.GetPosition).ToArray());
    }

    [Fact]
    public void Int32CounterWrapKeepsCounting()
    {
        var d = NewDevice();
        Pos(d, 100, int.MaxValue - 50);
        Pos(d, 120, (long)int.MaxValue + 50);         // wraps to a large negative int on the wire
        Assert.Equal(100, d.GetPosition(0));
    }

    [Fact]
    public void OlderSampleThanTheNewestIsIgnored()
    {
        var d = NewDevice();
        Pos(d, 100, 0);
        Pos(d, 140, 400);
        Pos(d, 120, 200);                              // a queued report overtaken by a newer one
        Assert.Equal(400, d.GetPosition(0));
    }

    [Fact]
    public void RdyResyncsAfterABoardReboot()
    {
        var d = NewDevice();
        Pos(d, 100, 0);
        Pos(d, 120, 300);
        d.HandleLine("RDY", Ticks(T0Ms + 200));
        Pos(d, 5, 0);                                  // counters + millis restarted at 0
        Assert.Equal(300, d.GetPosition(0));
        Pos(d, 25, 50);
        Assert.Equal(350, d.GetPosition(0));
    }

    [Fact]
    public void GripperClampStickyReReferencesAtTheLimit()
    {
        var d = NewDevice();
        d.SetPositionBounds(0, 0, 1000);
        Pos(d, 100, 0);
        Pos(d, 120, 1200);                             // over-driven 200 steps into the hard stop
        Assert.Equal(1000, d.GetPosition(0));
        Pos(d, 140, 1100);                             // backs off 100: moves off the limit at once
        Assert.Equal(900, d.GetPosition(0));
    }

    [Fact]
    public void DoneDoesNotAddADeltaOnTopOfBoardCounts()
    {
        var d = NewDevice();
        Pos(d, 100, 0);
        Pos(d, 200, 800);                              // the POS sent just before DONE
        d.HandleLine("DONE:0", Ticks(T0Ms + 206));
        Assert.Equal(800, d.GetPosition(0));
    }

    // ── Velocity + extrapolation ─────────────────────────────────────────────

    [Fact]
    public void MeasuredVelocityAndExtrapolationToNow()
    {
        var d = NewDevice();
        for (uint ms = 0; ms <= 200; ms += 20) Pos(d, 1000 + ms, ms * 10);   // 10 000 steps/s
        // Newest sample: board 1200 ms, 2000 steps, received at ctrl 1205. 10 ms later → +100 steps.
        Assert.True(d.TryGetLivePositionAt(0, Ticks(T0Ms + 1200 + LatencyMs + 10), out double steps, out double v));
        Assert.Equal(10_000, v, 1);
        Assert.Equal(2100, steps, 1);
    }

    [Fact]
    public void ClockMapTakesTheLeastDelayedSample()
    {
        var d = NewDevice();
        // Jittery latency 5..15 ms; the 5 ms samples define the map.
        double[] lat = { 12, 5, 15, 9, 5, 11 };
        for (int i = 0; i < lat.Length; i++) Pos(d, (uint)(1000 + i * 20), i * 200, lat[i]);
        // Newest: board 1100 ms, 1000 steps. Controller "now" 1100+5 = board 1100 → no extrapolation.
        d.TryGetLivePositionAt(0, Ticks(T0Ms + 1100 + LatencyMs), out double steps, out _);
        Assert.InRange(steps, 1000, 1010);
    }

    [Fact]
    public void ExtrapolationIsCappedWhenTheStreamGoesQuiet()
    {
        var d = NewDevice();
        for (uint ms = 0; ms <= 100; ms += 20) Pos(d, ms, ms * 10);          // 10 000 steps/s, last = 1000
        d.TryGetLivePositionAt(0, Ticks(T0Ms + 100 + LatencyMs + 5000), out double steps, out _);
        // Baseline stream (100 ms) → at most 3 periods (300 ms) ahead.
        Assert.InRange(steps, 1000 + 2990, 1000 + 3010);
    }

    [Fact]
    public void VelocityDropsToZeroWhenTheCountStopsChanging()
    {
        var d = NewDevice();
        for (uint ms = 0; ms <= 100; ms += 20) Pos(d, ms, ms * 10);
        Pos(d, 120, 1000);                             // stopped
        d.TryGetLivePositionAt(0, Ticks(T0Ms + 120 + LatencyMs + 30), out double steps, out double v);
        Assert.Equal(0, v);
        Assert.Equal(1000, steps);
    }

    [Fact]
    public void UnknownAxisReturnsFalse()
    {
        var d = NewDevice();
        Assert.False(d.TryGetLivePosition(4, out _, out _));
        Assert.False(d.TryGetLivePosition(-1, out _, out _));
    }

    // ── Latch / CHG events ───────────────────────────────────────────────────

    [Fact]
    public void LatLineRaisesAnEventWithBoardPositionsAndMappedTime()
    {
        var d = NewDevice(true, new IoPinConfig { Pin = 9, Type = PinType.Input, Latch = true });
        var events = new List<AuxLatchEvent>();
        d.InputLatched += events.Add;
        d.SetPosition(0, 1000);
        Pos(d, 1000, 0);                               // offset = 1000
        Pos(d, 1020, 200);
        d.HandleLine("LAT:9,1,1013,130,0,0,0", Ticks(T0Ms + 1013 + 20));      // queued, arrives late

        var e = Assert.Single(events);
        Assert.Equal(("AUX_T", 9, 1), (e.DeviceId, e.Pin, e.Value));
        Assert.Equal(1130, e.AxisSteps[0]);
        Assert.Equal(Ticks(T0Ms + 1013 + LatencyMs), e.StopwatchTicks, 2.0 * TicksPerMs);  // edge time, not receive time
        Assert.True(d.GetIoPinStates().Single(p => p.Pin == 9).Value);
        Assert.Equal(1200, d.GetPosition(0));          // an older LAT never rolls the position back
    }

    [Fact]
    public void ChgLineRaisesAnEventInterpolatedToReceiveTime()
    {
        var d = NewDevice(true, new IoPinConfig { Pin = 10, Type = PinType.Input });
        var events = new List<AuxLatchEvent>();
        d.InputLatched += events.Add;
        for (uint ms = 0; ms <= 100; ms += 20) Pos(d, ms, ms * 10);          // 10 000 steps/s
        long recv = Ticks(T0Ms + 100 + LatencyMs + 8);                         // 8 ms after the last POS
        d.HandleLine("CHG:10,0", recv);

        var e = Assert.Single(events);
        Assert.Equal((10, 0, recv), (e.Pin, e.Value, e.StopwatchTicks));
        Assert.Equal(1080, e.AxisSteps[0], 1);
    }

    [Fact]
    public void LatchPinConfiguresAsTypeLOnlyWhenTheFirmwareHasLatch()
    {
        var modern = NewDevice(posCaps: true);
        modern.ConfigureIoPin(9, PinType.Input, latch: true);
        modern.ConfigureIoPin(10, PinType.Input);
        modern.ConfigureIoPin(11, PinType.Output, latch: true);
        Assert.Equal(new[] { "CFG:9,L", "CFG:10,I", "CFG:11,O" }, Queue(modern));

        var legacy = NewDevice(posCaps: false);
        legacy.ConfigureIoPin(9, PinType.Input, latch: true);                // would ERR on type L
        Assert.Equal(new[] { "CFG:9,I" }, Queue(legacy));
    }

    [Fact]
    public void LatchFlagSerializesOnlyWhenSetAndPinStaysAnInput()
    {
        string plain = JsonSerializer.Serialize(new IoPinConfig { Pin = 9, Type = PinType.Input }, JsonDefaults.File);
        Assert.DoesNotContain("latch", plain);
        string latch = JsonSerializer.Serialize(new IoPinConfig { Pin = 9, Type = PinType.Input, Latch = true }, JsonDefaults.File);
        var back = JsonSerializer.Deserialize<IoPinConfig>(latch, JsonDefaults.File)!;
        Assert.True(back.Latch);
        Assert.Equal(PinType.Input, back.Type);
        var state = NewDevice(true, back).GetIoPinStates().Single();
        Assert.Equal((PinType.Input, true), (state.Type, state.Latch));
    }

    // ── Offline simulation / dead reckoning ──────────────────────────────────

    [Fact]
    public void SimContinuousJogIntegratesAndExtrapolates()
    {
        var d = NewDevice(posCaps: false);
        d.SetDirection(0, ccw: true);
        d.SetContinuous(0, 1000, 1_000_000);           // effectively instant ramp
        d.SimStep(Ticks(T0Ms));
        for (int i = 1; i <= 10; i++) d.SimStep(Ticks(T0Ms + i * 50));       // 0.5 s
        Assert.InRange(d.GetPosition(0), -501, -499);

        d.TryGetLivePositionAt(0, Ticks(T0Ms + 600), out double steps, out double v);
        Assert.Equal(-1000, v);
        Assert.InRange(steps, -601, -599);             // (-0.5: the 1 ms start ramp)

        d.SimStep(Ticks(T0Ms + 600));
        d.StopSmooth(0, 1_000_000);
        d.SimStep(Ticks(T0Ms + 650));
        d.SimStep(Ticks(T0Ms + 700));
        d.TryGetLivePositionAt(0, Ticks(T0Ms + 720), out steps, out v);
        Assert.Equal(0, v);
        Assert.InRange(steps, -601, -598);
    }

    [Fact]
    public void SimContinuousRampsAtTheCommandedAccel()
    {
        var d = NewDevice(posCaps: false);
        d.SetContinuous(0, 1000, 1000);                // 1000 steps/s reached after 1 s
        d.SimStep(Ticks(T0Ms));
        for (int i = 1; i <= 20; i++) d.SimStep(Ticks(T0Ms + i * 50));
        Assert.InRange(d.GetPosition(0), 499, 501);    // ½·a·t² = 500
        d.TryGetLivePositionAt(0, Ticks(T0Ms + 1000), out _, out double v);
        Assert.Equal(1000, v, 1);
    }

    [Fact]
    public void SimIndexedMoveReportsLivePositionBetweenTicks()
    {
        var d = NewDevice(posCaps: false);
        d.StartMove(0, 1000, 2000, 10000, 10000);
        Assert.True(d.IsMoving);
        d.SimStep(Ticks(T0Ms));
        d.SimStep(Ticks(T0Ms + 100));                  // 200 steps
        d.TryGetLivePositionAt(0, Ticks(T0Ms + 125), out double steps, out double v);
        Assert.Equal(2000, v);
        Assert.Equal(250, steps, 1);
        for (int i = 2; i <= 6; i++) d.SimStep(Ticks(T0Ms + i * 100));
        Assert.False(d.IsMoving);
        Assert.Equal(1000, d.GetPosition(0));
    }

    [Fact]
    public void BoardCountsTakeOverFromDeadReckoning()
    {
        var d = NewDevice();
        d.SetContinuous(0, 1000, 1_000_000);
        d.SimStep(Ticks(T0Ms));
        d.SimStep(Ticks(T0Ms + 200));                  // dead-reckoned 200 steps
        Pos(d, 50, 0);                                 // first board sample: continuous from 200
        Assert.Equal(200, d.GetPosition(0));
        d.SimStep(Ticks(T0Ms + 400));                  // sim no longer moves it
        Assert.Equal(200, d.GetPosition(0));
    }

    // ── Serial latency knob ──────────────────────────────────────────────────

    [Fact]
    public void AuxSessionReadTimeoutIsShortAndNanoKeepsItsDefault()
    {
        static int Timeout(SerialLineDevice dev) => (int)typeof(SerialLineDevice)
            .GetProperty("SessionReadTimeoutMs", BindingFlags.NonPublic | BindingFlags.Instance)!.GetValue(dev)!;
        Assert.True(Timeout(NewDevice()) <= 20);
        Assert.Equal(100, Timeout(new NanoDevice(new IoDeviceConfig { Id = "N", Name = "N" })));
    }

    // ── Role action velocity (AuxSteps) ──────────────────────────────────────

    [Fact]
    public void ActionVelocityConvertsPhysicalUnitsToStepsPerSecond()
    {
        var linear = new AuxAxisChannelConfig { StepsPerRev = 1600, AxisType = AuxAxisType.Linear, MmPerRev = 8, ActionVelocity = 50 };
        Assert.Equal(10_000, AuxSteps.ActionVel(linear));                     // 50 mm/s × 200 steps/mm
        var rotary = new AuxAxisChannelConfig { StepsPerRev = 3600, AxisType = AuxAxisType.Rotary, ActionVelocity = 90 };
        Assert.Equal(900, AuxSteps.ActionVel(rotary));                        // 90 °/s × 10 steps/°
        var raw = new AuxAxisChannelConfig { ActionVelocity = 800 };
        Assert.Equal(800, AuxSteps.ActionVel(raw));                           // untyped: already steps/s
        Assert.Equal(1600, AuxSteps.ActionVel(new AuxAxisChannelConfig()));   // unset → default
        Assert.Equal(1600, AuxSteps.ActionVel(null));
    }
}
