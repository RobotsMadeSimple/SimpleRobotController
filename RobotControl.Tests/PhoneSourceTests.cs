using System.Linq;
using Controller.RobotControl;
using Controller.RobotControl.Camera;
using Controller.RobotControl.Devices;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

/// <summary>
/// Phones as a native source: the "phone" push camera (frames arrive via <see cref="CameraDevice.PushFrame"/>)
/// and the <see cref="DeviceManager"/> data channels that surface as <c>$device.&lt;id&gt;.&lt;channel&gt;</c>.
/// </summary>
public sealed class PhoneSourceTests
{
    // ── Phone push camera ────────────────────────────────────────────────────

    [Fact]
    public void PhoneCameraStartsWithoutACaptureThreadAndSurfacesPushedFrames()
    {
        var cam = new CameraDevice(new CameraConfig { Id = "PHONE", Name = "Phone", SourceType = "phone" });
        Assert.True(cam.IsPhone);
        Assert.False(cam.IsUsb);

        cam.Start();                                   // no-op for a phone source: no device to open
        Assert.Null(cam.GetLatestFrame());
        Assert.False(cam.GetState().Connected);        // nothing pushed yet

        var jpeg = new byte[] { 0xFF, 0xD8, 1, 2, 3, 0xFF, 0xD9 };
        cam.PushFrame(jpeg, 320, 240);

        Assert.Equal(jpeg, cam.GetLatestFrame());
        var state = cam.GetState();
        Assert.True(state.Connected);                  // fresh push → connected
        Assert.Equal(320, state.StreamWidth);
        Assert.Equal(240, state.StreamHeight);
    }

    [Fact]
    public void NormalizeSourceTypeAcceptsPhone()
    {
        Assert.Equal("phone", NetworkCameraSource.NormalizeSourceType("PHONE"));
        Assert.Equal("usb",   NetworkCameraSource.NormalizeSourceType("nonsense"));
    }

    // ── Device data channels ─────────────────────────────────────────────────

    [Fact]
    public void DeviceManagerIngestsChannelsAndReportsFreshness()
    {
        var path = Path.Combine(Path.GetTempPath(), $"devtest_{Guid.NewGuid():N}.json");
        try
        {
            var mgr = new DeviceManager(path);
            var id = mgr.AddDevice(new DeviceConfig { Name = "Phone" });
            Assert.StartsWith("DEV_", id);
            Assert.False(mgr.GetState().Single(d => d.Id == id).Connected);   // no data yet

            Assert.True(mgr.Ingest(id, "accel.x", 9.81, "m/s^2", 123));        // dotted channel kept intact
            Assert.True(mgr.TryGetChannel(id, "accel.x", out var v));
            Assert.Equal(9.81, v, 3);

            var st = mgr.GetState().Single(d => d.Id == id);
            Assert.True(st.Connected);
            Assert.Contains(st.Channels, c => c.Key == "accel.x" && c.Unit == "m/s^2");

            Assert.False(mgr.Ingest("NOPE", "x", 1, null, 0));                 // unknown device rejected
            Assert.True(mgr.RemoveDevice(id));
            Assert.DoesNotContain(mgr.GetState(), d => d.Id == id);
        }
        finally { try { File.Delete(path); } catch { } }
    }

    [Fact]
    public void ProviderConfigPersistsAndRoundTripsThroughGetState()
    {
        var path = Path.Combine(Path.GetTempPath(), $"devtest_{Guid.NewGuid():N}.json");
        try
        {
            var mgr = new DeviceManager(path);
            var id = mgr.AddDevice(new DeviceConfig { Name = "Phone" });
            Assert.Null(mgr.GetState().Single(d => d.Id == id).ProviderConfig);   // none until the app saves it

            const string cfg = "{\"mode\":\"webrtc\",\"resTier\":\"high\",\"rtcFps\":\"30\"}";
            Assert.True(mgr.UpdateDevice(id, null, null, cfg));                    // stored verbatim (opaque blob)
            Assert.Equal(cfg, mgr.GetState().Single(d => d.Id == id).ProviderConfig);

            Assert.True(mgr.UpdateDevice(id, "Renamed", null));                   // a name-only edit keeps the blob
            var st = mgr.GetState().Single(d => d.Id == id);
            Assert.Equal("Renamed", st.Name);
            Assert.Equal(cfg, st.ProviderConfig);

            // A fresh manager reads the same config back from disk (persistence).
            var reloaded = new DeviceManager(path);
            Assert.Equal(cfg, reloaded.GetState().Single(d => d.Id == id).ProviderConfig);
        }
        finally { try { File.Delete(path); } catch { } }
    }

    [Fact]
    public void DeviceChannelsResolveAsExpressionProperties()
    {
        Assert.Contains("device", RobotPropertySource.Roots);

        var robot = new RobotController();
        var id = robot.DeviceManager.AddDevice(new DeviceConfig { Name = "Phone" });
        try
        {
            robot.DeviceManager.Ingest(id, "accel.x", 1.25, null, 0);
            var props = new RobotPropertySource(robot, null);

            Assert.True(props.TryGet($"device.{id}.accel.x", out var v));
            Assert.Equal(1.25, v, 3);
            Assert.False(props.TryGet($"device.{id}.missing", out _));
            Assert.False(props.TryGet("device.NO_SUCH_DEVICE.x", out _));
            Assert.Contains(props.List(), p => p.Name == $"device.{id}.accel.x");

            // No robot → unknown, not a crash.
            Assert.False(new RobotPropertySource(null, null).TryGet("device.DEV_0.x", out _));
        }
        finally { robot.DeviceManager.RemoveDevice(id); }
    }
}
