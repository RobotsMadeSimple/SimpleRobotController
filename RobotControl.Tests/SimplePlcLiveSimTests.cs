using System.Net.Sockets;
using System.Text;
using Controller.RobotControl.Io;
using Controller.RobotControl.SimplePlc;

namespace RobotControl.Tests;

/// <summary>
/// Exercises the REAL socket path (TcpSimplePlcTransport + UdpSimplePlcDiscovery) against cad-monkey's
/// reference simulator. Opt-in: set SIMPLC_SIM=1 and run the sim first, e.g. from DistributedIO/firmware
///   build\simplc-sim.exe --modules DI8,DO8,DI8
/// Skipped (passes trivially) when SIMPLC_SIM is not set, so CI without the sim stays green.
/// </summary>
public sealed class SimplePlcLiveSimTests : IDisposable
{
    private readonly string _path = Path.Combine(Path.GetTempPath(), $"plcsim_{Guid.NewGuid():N}.json");
    private SimplePlcManager? _mgr;
    public void Dispose() { try { _mgr?.Stop(); } catch { } try { File.Delete(_path); } catch { } }

    private static bool Enabled => Environment.GetEnvironmentVariable("SIMPLC_SIM") == "1";

    private static bool WaitUntil(Func<bool> cond, int ms)
    {
        var end = DateTime.UtcNow.AddMilliseconds(ms);
        while (DateTime.UtcNow < end) { if (cond()) return true; Thread.Sleep(25); }
        return cond();
    }

    private static void SimConsole(string line)
    {
        using var c = new TcpClient();
        c.Connect("127.0.0.1", 47812);
        var b = Encoding.UTF8.GetBytes(line + "\n");
        c.GetStream().Write(b, 0, b.Length);
    }

    [Fact]
    public void Discovers_Connects_ReadsIo_WritesOutputs_AgainstSim()
    {
        if (!Enabled) return;   // sim not running

        var heads = new UdpSimplePlcDiscovery().Discover(1500);
        Assert.True(heads.Count > 0, "discovery found no head");

        _mgr = new SimplePlcManager(_path, new TcpSimplePlcTransportFactory(), new UdpSimplePlcDiscovery());
        var id = _mgr.Add(new SimplePlcDeviceConfig { Name = "Sim", Host = "127.0.0.1", Port = SimplePlcProtocol.DefaultTcpPort, AutoConnect = true });
        _mgr.Start();

        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Connected, 4000), "did not connect to sim");
        var st = _mgr.GetAllStates().Single();
        Assert.True(st.Owned, "did not claim the sim");
        Assert.Equal(3, st.Slots.Count);   // DI8, DO8, DI8

        // Write an output bit and confirm the device reflects it.
        int slot2bit3 = 2 * SimplePlcDevice.BitsPerSlot + 3;
        Assert.True(_mgr.SetOutput(id, slot2bit3, true));
        Assert.True(WaitUntil(() => _mgr.GetAllIoStates().Single().Pins.First(p => p.Pin == slot2bit3).Value, 2000));

        // Flip a field input via the sim console and confirm the pushed event lands.
        int slot1bit2 = 1 * SimplePlcDevice.BitsPerSlot + 2;
        SimConsole("in 1 2 on");
        Assert.True(WaitUntil(() =>
        {
            var pin = _mgr.GetAllIoStates().Single().Pins.FirstOrDefault(p => p.Pin == slot1bit2);
            return pin is { Type: PinType.Input, Value: true };
        }, 3000), "input event from sim not observed");
    }
}
