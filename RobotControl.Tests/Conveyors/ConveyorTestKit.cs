using System.Diagnostics;
using Controller.RobotControl.Conveyors;

namespace RobotControl.Tests.Conveyors;

/// <summary>A controllable clock (Stopwatch ticks).</summary>
internal sealed class FakeClock
{
    public long Ticks = 1_000_000;
    public static readonly double Freq = Stopwatch.Frequency;
    public long Now() => Ticks;
    public void Advance(double seconds) => Ticks += (long)Math.Round(seconds * Freq);
}

/// <summary>
/// A fake belt: an aux axis whose step count runs at <see cref="StepsPerSec"/> on a clock (fake or
/// real), plus settable sensor inputs, a run output and latch events.
/// </summary>
internal sealed class FakeBelt : IConveyorTravelSource
{
    private readonly Func<long> _clock;
    private long _t0;
    private double _steps0;
    private double _sps;

    public FakeBelt(Func<long> clock)
    {
        _clock = clock;
        _t0 = clock();
    }

    /// <summary>Steps per mm the axis reports (0 = no physical unit).</summary>
    public double Spm = 10;
    public bool Online = true;
    public readonly Dictionary<string, bool?> Inputs = new();
    public bool? Output;
    public readonly List<(string Device, int Period)> StreamRequests = new();

    public double StepsPerSec
    {
        get => _sps;
        set { _steps0 = StepsNow(); _t0 = _clock(); _sps = value; }
    }

    public double StepsNow() => _steps0 + _sps * ((_clock() - _t0) / FakeClock.Freq);

    /// <summary>Re-zero the counter (SetPosition / ZeroPosition on the board).</summary>
    public void Rezero(double steps) { _steps0 = steps; _t0 = _clock(); }

    public event Action<ConveyorLatch>? InputLatched;

    public void Latch(string device, int pin, int value, double axis0Steps) =>
        InputLatched?.Invoke(new ConveyorLatch(device, pin, value, [axis0Steps, 0, 0, 0], _clock()));

    public bool TryGetLivePosition(string deviceId, int axis, out double steps, out double stepsPerSec)
    {
        steps = StepsNow();
        stepsPerSec = _sps;
        return Online;
    }

    public double StepsPerMm(string deviceId, int axis) => Spm;

    public void SetPositionStreaming(string deviceId, int periodMs) => StreamRequests.Add((deviceId, periodMs));

    public bool? ReadInput(ConveyorSensor sensor) =>
        Inputs.TryGetValue(sensor.Kind == ConveyorSensorKinds.Stb ? $"stb{sensor.Input}" : $"{sensor.DeviceId}:{sensor.Pin}", out var v) ? v : null;

    public bool? ReadOutput(ConveyorIoRef output) => Output;
}

internal static class ConveyorKit
{
    public static string TempFile() =>
        Path.Combine(Path.GetTempPath(), "rms-conv-" + Guid.NewGuid().ToString("N"), "conveyors.json");

    public static Conveyor Aux(string name = "Belt", double scale = 1, double rz = 0) => new()
    {
        Name   = name,
        Source = new ConveyorSource { Kind = ConveyorSourceKinds.Aux, DeviceId = "uno-1", Axis = 0 },
        Scale  = scale,
        Frame  = new ConveyorFrame { X = 100, Y = 200, Z = 10, RZ = rz },
        Sensor = new ConveyorSensor { Kind = ConveyorSensorKinds.None, OffsetMm = 0, MinSpacingMm = 10 },
        Window = new ConveyorWindow { StartMm = 50, EndMm = 300 },
    };

    /// <summary>A service over a fresh repository holding <paramref name="conveyors"/>.</summary>
    public static (ConveyorService Svc, List<Conveyor> Saved) Service(IConveyorTravelSource src, Func<long> clock, params Conveyor[] conveyors)
    {
        var file = TempFile();
        Directory.CreateDirectory(Path.GetDirectoryName(file)!);
        var svc = new ConveyorService(new ConveyorRepository(file), src, clock);
        var saved = conveyors.Select(svc.Save).ToList();
        return (svc, saved);
    }
}
