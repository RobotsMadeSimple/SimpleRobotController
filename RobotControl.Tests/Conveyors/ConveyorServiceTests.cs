using Controller.RobotControl.Conveyors;
using Controller.RobotControl.Programs;

namespace RobotControl.Tests.Conveyors;

public class ConveyorServiceTests
{
    private readonly FakeClock _clock = new();
    private readonly FakeBelt _belt;

    public ConveyorServiceTests() => _belt = new FakeBelt(() => _clock.Ticks);

    private (ConveyorService Svc, Conveyor C) One(Conveyor c)
    {
        var (svc, saved) = ConveyorKit.Service(_belt, _clock.Now, c);
        return (svc, saved[0]);
    }

    // ── Config ────────────────────────────────────────────────────────────

    [Fact]
    public void SaveAssignsAnExpressionSafeIdAndPersists()
    {
        var file = ConveyorKit.TempFile();
        Directory.CreateDirectory(Path.GetDirectoryName(file)!);
        var svc = new ConveyorService(new ConveyorRepository(file), _belt, _clock.Now);
        var c = svc.Save(ConveyorKit.Aux("Infeed"));
        Assert.Matches("^c[0-9a-f]{12}$", c.Id);
        Assert.True(ConveyorRepository.IsValidId(c.Id));

        var again = new ConveyorService(new ConveyorRepository(file), _belt, _clock.Now);
        var loaded = Assert.Single(again.GetAll());
        Assert.Equal("Infeed", loaded.Name);
        Assert.Equal(300, loaded.Window.EndMm);
        Assert.Equal(ConveyorSourceKinds.Aux, loaded.Source.Kind);
    }

    [Fact]
    public void BadConfigsAreRefused()
    {
        var (svc, _) = ConveyorKit.Service(_belt, _clock.Now);
        Assert.Throws<ArgumentException>(() => svc.Save(new Conveyor { Name = "" }));
        Assert.Throws<ArgumentException>(() => svc.Save(new Conveyor { Name = "x", Id = "has-dash" }));
        Assert.Throws<ArgumentException>(() => svc.Save(new Conveyor { Name = "x", Source = new ConveyorSource { Kind = "aux" } }));
        Assert.Throws<ArgumentException>(() => svc.Save(new Conveyor { Name = "x", Source = new ConveyorSource { Kind = "timed" } }));
        Assert.Throws<ArgumentException>(() => svc.Save(new Conveyor { Name = "x", Window = new ConveyorWindow { StartMm = 10, EndMm = 5 } }));
        Assert.Throws<ArgumentException>(() => svc.Save(new Conveyor { Name = "x", Sensor = new ConveyorSensor { Kind = "io" } }));
        Assert.Throws<ArgumentException>(() => svc.Save(new Conveyor { Name = "x", Scale = 0 }));
    }

    [Fact]
    public void AnAuxSourceAsksItsDeviceToStream()
    {
        One(ConveyorKit.Aux());
        Assert.Contains(("uno-1", ConveyorService.StreamPeriodMs), _belt.StreamRequests);
        Assert.Equal(20, ConveyorService.StreamPeriodMs);
    }

    // ── Travel ────────────────────────────────────────────────────────────

    [Fact]
    public void AuxTravelIsStepsOverStepsPerMmTimesScale()
    {
        var (svc, c) = One(ConveyorKit.Aux(scale: 0.5));
        _belt.Spm = 10; svc.Reload();
        _belt.StepsPerSec = 1000;            // 100 mm/s raw
        _clock.Advance(2);                   // 2000 steps = 200 mm raw → 100 mm scaled
        Assert.Equal(100, svc.TravelAt(c.Id, _clock.Ticks), 6);
        Assert.Equal(50, svc.SpeedOf(c.Id), 6);
        // Extrapolated to a later moment with the live velocity.
        Assert.Equal(105, svc.TravelAt(c.Id, _clock.Ticks + (long)(0.1 * FakeClock.Freq)), 6);
    }

    [Fact]
    public void MmPerStepOverridesTheAxisConfig()
    {
        var c0 = ConveyorKit.Aux();
        c0.Source.MmPerStep = 0.2;
        var (svc, c) = One(c0);
        _belt.Spm = 0;                       // the axis has no physical unit
        _belt.StepsPerSec = 100;
        _clock.Advance(1);
        Assert.Equal(20, svc.TravelAt(c.Id, _clock.Ticks), 6);
    }

    [Fact]
    public void AReZeroedCounterDoesNotMakeTravelJump()
    {
        var (svc, c) = One(ConveyorKit.Aux());
        _belt.StepsPerSec = 1000;
        _clock.Advance(1);
        double before = svc.TravelAt(c.Id, _clock.Ticks);      // 100 mm
        _belt.Rezero(0);                                         // board counter set to 0
        _clock.Advance(0.01);
        double after = svc.TravelAt(c.Id, _clock.Ticks);
        Assert.Equal(before + 1, after, 3);                      // carried on: +10 steps
    }

    [Fact]
    public void TimedTravelIntegratesWhileTheRunOutputIsOn()
    {
        var c0 = new Conveyor
        {
            Name = "Timed",
            Source = new ConveyorSource { Kind = ConveyorSourceKinds.Timed, SpeedMmS = 50, RunOutput = new ConveyorIoRef { OutputCard = "relay", OutputNumber = 2 } },
        };
        var (svc, c) = One(c0);
        _belt.Output = false;
        svc.Tick();
        _clock.Advance(1);
        svc.Tick();
        Assert.Equal(0, svc.TravelAt(c.Id, _clock.Ticks), 6);
        _belt.Output = true;
        svc.Tick();
        _clock.Advance(2);
        Assert.Equal(100, svc.TravelAt(c.Id, _clock.Ticks), 6);
        Assert.Equal(50, svc.SpeedOf(c.Id), 6);
        _belt.Output = false;
        svc.Tick();
        _clock.Advance(5);
        svc.Tick();
        Assert.Equal(100, svc.TravelAt(c.Id, _clock.Ticks), 6);
    }

    [Fact]
    public void ManualTravelIsSetDirectlyAndOtherSourcesRefuseIt()
    {
        var (svc, saved) = ConveyorKit.Service(_belt, _clock.Now,
            new Conveyor { Name = "M", Source = new ConveyorSource { Kind = ConveyorSourceKinds.Manual }, Scale = 3 },
            ConveyorKit.Aux());
        var manual = saved[0];
        svc.SetTravel(manual.Id, 123.5);
        Assert.Equal(123.5, svc.TravelAt(manual.Id, _clock.Ticks)); // scale does not apply to a set value
        Assert.Throws<InvalidOperationException>(() => svc.SetTravel(saved[1].Id, 5));
    }

    // ── Parts, latching, window ───────────────────────────────────────────

    [Fact]
    public void AddPartPlacesThePartAtItsOffsetAndItMovesWithTheBelt()
    {
        var (svc, c) = One(ConveyorKit.Aux());
        _belt.StepsPerSec = 1000; // 100 mm/s
        long id = svc.AddPart(c.Id, 60);
        Assert.Equal(1, id);
        _clock.Advance(0.5);
        var st = Assert.Single(svc.Status());
        Assert.Equal(110, Assert.Single(st.Parts).X, 3);
        Assert.Equal(1, st.Latched);
    }

    [Fact]
    public void AnIoSensorEdgeLatchesAtTheTravelThenAndRespectsMinSpacing()
    {
        var c0 = ConveyorKit.Aux();
        c0.Sensor = new ConveyorSensor { Kind = ConveyorSensorKinds.Io, DeviceId = "nano", Pin = 3, ActiveHigh = false, OffsetMm = 20, MinSpacingMm = 10 };
        var (svc, c) = One(c0);
        _belt.StepsPerSec = 1000; // 100 mm/s
        _belt.Inputs["nano:3"] = true;   // pull-up: idle high
        svc.Tick();
        _clock.Advance(0.5);
        _belt.Inputs["nano:3"] = false;  // part breaks the beam (active low)
        svc.Tick();                      // latched at travel 50
        _clock.Advance(0.02);
        _belt.Inputs["nano:3"] = true;
        svc.Tick();
        _clock.Advance(0.03);            // a bounce 5 mm later — under minSpacing
        _belt.Inputs["nano:3"] = false;
        svc.Tick();
        _clock.Advance(0.2);
        var st = svc.Status().Single();
        var part = Assert.Single(st.Parts);
        Assert.Equal(50, part.TravelAtLatch, 3);
        // X = offset + travel since latch = 20 + (75 − 50)
        Assert.Equal(45, part.X, 3);
        Assert.Equal(50, st.LastLatchTravelMm!.Value, 3);
        Assert.Equal(1, st.Latched);
    }

    [Fact]
    public void AnActiveHighStbSensorLatchesOnTheRisingEdge()
    {
        var c0 = ConveyorKit.Aux();
        c0.Sensor = new ConveyorSensor { Kind = ConveyorSensorKinds.Stb, Input = 2, ActiveHigh = true };
        var (svc, c) = One(c0);
        _belt.Inputs["stb2"] = false;
        svc.Tick();
        _belt.Inputs["stb2"] = true;
        svc.Tick();
        Assert.Single(svc.Status().Single().Parts);
    }

    [Fact]
    public void AnAuxLatchPinUsesTheStepsCapturedAtTheEdge()
    {
        var c0 = ConveyorKit.Aux();
        c0.Sensor = new ConveyorSensor { Kind = ConveyorSensorKinds.AuxPin, DeviceId = "uno-1", Pin = 7, ActiveHigh = false, OffsetMm = 5 };
        var (svc, c) = One(c0);
        _belt.StepsPerSec = 1000;
        _clock.Advance(1);                      // live: 1000 steps = 100 mm
        _belt.Latch("uno-1", 7, 0, 950);        // the edge happened at 950 steps = 95 mm
        _belt.Latch("uno-1", 7, 1, 960);        // release edge: ignored
        var part = svc.Status().Single().Parts.Single();
        Assert.Equal(95, part.TravelAtLatch, 3);
        Assert.Equal(5 + (100 - 95), part.X, 3);
    }

    [Fact]
    public void PartsPastTheWindowAreDroppedAsMissed()
    {
        var (svc, c) = One(ConveyorKit.Aux());   // window 50 … 300
        _belt.StepsPerSec = 1000;                // 100 mm/s
        svc.AddPart(c.Id, 280);
        svc.AddPart(c.Id, 100);
        _clock.Advance(0.3);                     // first at 310, second at 130
        svc.Tick();
        var st = svc.Status().Single();
        Assert.Equal(1, st.Missed);
        Assert.Equal(130, Assert.Single(st.Parts).X, 3);
        Assert.True(svc.TryGetProperty(c.Id, "missed", out var missed));
        Assert.Equal(1, missed);
    }

    [Fact]
    public void TheQueueKeepsAtMostMaxQueueParts()
    {
        var c0 = ConveyorKit.Aux();
        c0.MaxQueue = 2;
        var (svc, c) = One(c0);
        svc.AddPart(c.Id, 10); svc.AddPart(c.Id, 20); svc.AddPart(c.Id, 30);
        var st = svc.Status().Single();
        Assert.Equal([2L, 3L], st.Parts.Select(p => p.Id));
        Assert.Equal(1, st.Missed);
    }

    [Fact]
    public void OldestInWindowSkipsPartsUpstreamOfTheWindow()
    {
        var (svc, c) = One(ConveyorKit.Aux());   // window 50 … 300
        svc.AddPart(c.Id, 10);                   // upstream
        long inside = svc.AddPart(c.Id, 80);
        Assert.Equal(inside, svc.OldestInWindow(c.Id)!.Id);
        Assert.True(svc.TryGetProperty(c.Id, "inWindow", out var n));
        Assert.Equal(1, n);
        Assert.True(svc.TryGetProperty(c.Id, "count", out var count));
        Assert.Equal(2, count);
    }

    // ── Tracking ──────────────────────────────────────────────────────────

    [Fact]
    public void EndTrackingCountsPickedOrReleases()
    {
        var (svc, c) = One(ConveyorKit.Aux());
        long a = svc.AddPart(c.Id, 100), b = svc.AddPart(c.Id, 120);
        var owner = new object();
        Assert.True(svc.StartTracking(c.Id, a, owner));
        Assert.True(svc.TryGetProperty(c.Id, "tracking", out var tracking));
        Assert.Equal(1, tracking);
        Assert.True(svc.TryGetProperty(c.Id, "part.id", out var pid));
        Assert.Equal(a, pid);
        Assert.True(svc.EndTracking(release: false));
        var st = svc.Status().Single();
        Assert.Equal(1, st.Picked);
        Assert.DoesNotContain(st.Parts, p => p.Id == a);

        svc.StartTracking(c.Id, b, owner);
        svc.EndTracking(release: true);
        st = svc.Status().Single();
        Assert.Equal(1, st.Picked);
        Assert.Empty(st.Parts);                   // released parts are hidden …
        Assert.Null(svc.OldestInWindow(c.Id));    // … and never tracked again
    }

    [Fact]
    public void TheTrackedPartLeavingTheWindowRaisesConveyorWindowForItsOwner()
    {
        var (svc, c) = One(ConveyorKit.Aux());
        _belt.StepsPerSec = 1000;
        long a = svc.AddPart(c.Id, 250);
        var owner = new object();
        svc.StartTracking(c.Id, a, owner);
        _clock.Advance(0.6);                      // x = 310
        svc.Tick();
        Assert.Null(svc.Tracking);                // tracking stopped
        Assert.Null(svc.ConsumeFault(new object())); // not someone else's
        var f = svc.ConsumeFault(owner);
        Assert.NotNull(f);
        Assert.Equal(ErrorCodes.ConveyorWindow, f!.Code);
        Assert.Null(svc.ConsumeFault(owner));      // consumed once
        Assert.Equal(1, svc.Status().Single().Missed);
    }

    [Fact]
    public void ThePartFrameIsTheConveyorFrameShiftedAlongItsX()
    {
        var (svc, c) = One(ConveyorKit.Aux(rz: 90));  // belt runs along +Y; frame origin (100, 200, 10)
        _belt.StepsPerSec = 1000;
        long a = svc.AddPart(c.Id, 60);
        svc.StartTracking(c.Id, a, null);
        _clock.Advance(0.5);                           // part X = 110
        Assert.True(svc.TryPartFrame(out var f, out var travel));
        Assert.Equal(50, travel, 6);
        Assert.Equal(100, f.X, 6);
        Assert.Equal(310, f.Y, 6);
        Assert.Equal(10, f.Z, 6);
        Assert.Equal(90, f.RZ, 6);
        Assert.Equal(110, svc.TrackedPartX()!.Value, 6);
        Assert.True(svc.TryGetProperty(c.Id, "part.x", out var px));
        Assert.Equal(110, px, 6);
    }

    [Fact]
    public void ReleasingTheOwnerStopsTrackingAndDropsItsFault()
    {
        var (svc, c) = One(ConveyorKit.Aux());
        var owner = new object();
        svc.StartTracking(c.Id, svc.AddPart(c.Id, 100), owner);
        svc.RaiseFault(ErrorCodes.ConveyorSpeed, "too fast");
        svc.StartTracking(c.Id, svc.AddPart(c.Id, 110), owner);
        svc.ReleaseOwner(owner);
        Assert.Null(svc.Tracking);
        Assert.Null(svc.ConsumeFault(owner));
    }

    [Fact]
    public void ChangingTheSourceOrScaleClearsTheQueue()
    {
        var (svc, c) = One(ConveyorKit.Aux());
        svc.AddPart(c.Id, 100);
        c.Name = "Renamed";
        svc.Save(c);
        Assert.Single(svc.Status().Single().Parts); // a rename keeps the parts
        c.Scale = 1.02;
        svc.Save(c);
        Assert.Empty(svc.Status().Single().Parts);
    }

    [Fact]
    public void StatusPartsAreCappedAt16()
    {
        var c0 = ConveyorKit.Aux();
        c0.MaxQueue = 40;
        var (svc, c) = One(c0);
        for (int i = 0; i < 20; i++) svc.AddPart(c.Id, i);
        Assert.Equal(16, svc.Status().Single().Parts.Count);
    }
}
