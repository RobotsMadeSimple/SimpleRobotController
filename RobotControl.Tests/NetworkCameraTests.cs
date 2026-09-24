using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Camera;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Persistence;
using OpenCvSharp;

namespace RobotControl.Tests;

/// <summary>Network (RTSP/HTTP) camera sources — docs/network-cameras.md.</summary>
public sealed class NetworkCameraTests
{
    // ── URL building and masking ─────────────────────────────────────────────

    [Fact]
    public void BuildUrlWithoutCredentialsReturnsUrlUnchanged()
    {
        Assert.Equal("rtsp://192.168.0.50:554/stream1", NetworkCameraSource.BuildUrl("rtsp://192.168.0.50:554/stream1", "", ""));
        Assert.Equal("http://cam/mjpeg", NetworkCameraSource.BuildUrl(" http://cam/mjpeg ", null, "ignored"));
    }

    [Fact]
    public void BuildUrlInjectsEncodedCredentialsAfterScheme()
    {
        Assert.Equal("rtsp://admin:secret@192.168.0.50:554/stream1",
            NetworkCameraSource.BuildUrl("rtsp://192.168.0.50:554/stream1", "admin", "secret"));
        Assert.Equal("rtsp://admin:p%40ss%3Aw%2Frd@cam:554/live?x=1",
            NetworkCameraSource.BuildUrl("rtsp://cam:554/live?x=1", "admin", "p@ss:w/rd"));
        Assert.Equal("http://us%40er@cam/mjpeg", NetworkCameraSource.BuildUrl("http://cam/mjpeg", "us@er", ""));
        Assert.Equal("http://a:b@cam", NetworkCameraSource.BuildUrl("http://cam", "a", "b"));
    }

    [Fact]
    public void BuildUrlReplacesCredentialsEmbeddedInTheUrl()
    {
        Assert.Equal("rtsp://new:pw@cam/s", NetworkCameraSource.BuildUrl("rtsp://old:x@cam/s", "new", "pw"));
        Assert.Equal("rtsp://old:x@cam/s", NetworkCameraSource.BuildUrl("rtsp://old:x@cam/s", "", ""));
    }

    [Theory]
    [InlineData("")]
    [InlineData("   ")]
    [InlineData("not a url")]
    [InlineData("192.168.0.50:554/stream")]
    [InlineData("ftp://cam/stream")]
    [InlineData("file:///etc/passwd")]
    [InlineData("rtsp://")]
    public void BuildUrlRejectsInvalidUrls(string url)
    {
        Assert.False(NetworkCameraSource.IsValidUrl(url));
        Assert.Null(NetworkCameraSource.BuildUrl(url, "u", "p"));
    }

    [Fact]
    public void MaskUrlHidesOnlyThePassword()
    {
        var effective = NetworkCameraSource.BuildUrl("rtsp://cam:554/live", "admin", "p@ss:w/rd")!;
        var masked = NetworkCameraSource.MaskUrl(effective);
        Assert.Equal("rtsp://admin:***@cam:554/live", masked);
        Assert.DoesNotContain("p%40ss", masked);
        Assert.Equal("http://cam/mjpeg", NetworkCameraSource.MaskUrl("http://cam/mjpeg"));
        Assert.Equal("http://user@cam/mjpeg", NetworkCameraSource.MaskUrl("http://user@cam/mjpeg"));
        Assert.Equal("", NetworkCameraSource.MaskUrl(null));
    }

    [Fact]
    public void DeviceMaskedUrlNeverContainsThePassword()
    {
        var device = new CameraDevice(new CameraConfig
        {
            Id = "CAM_9", SourceType = "network", Url = "rtsp://10.0.0.5/s", Username = "u", Password = "hunter2",
        });
        Assert.True(device.IsNetwork);
        Assert.Equal("rtsp://u:***@10.0.0.5/s", device.MaskedUrl());
        Assert.Empty(device.ProbeResolutions());
    }

    // ── Config model ─────────────────────────────────────────────────────────

    [Fact]
    public void ExistingConfigWithoutSourceTypeLoadsAsUsb()
    {
        const string json = """
            {"cameras":[{"id":"CAM_0","name":"Top","deviceIndex":1,"enabled":true,"width":1280,"height":720,
                         "targetFps":15,"supportedResolutions":[{"width":640,"height":480}]}]}
            """;
        var cfg = JsonSerializer.Deserialize<CameraManagerConfig>(json, JsonDefaults.File)!;
        var cam = Assert.Single(cfg.Cameras);
        Assert.Equal("usb", cam.SourceType);
        Assert.Equal("", cam.Url);
        Assert.Equal("tcp", cam.Transport);
        Assert.Equal(1, cam.DeviceIndex);
        Assert.Equal(1280, cam.Width);

        var device = new CameraDevice(cam);
        Assert.False(device.IsNetwork);
        var state = device.GetState();
        Assert.Equal("usb", state.SourceType);
        Assert.Equal(0, state.StreamWidth);
    }

    [Fact]
    public void NetworkConfigRoundTripsThroughTheConfigFile()
    {
        var path = Path.Combine(Path.GetTempPath(), "netcam-" + Guid.NewGuid().ToString("N") + ".json");
        try
        {
            var original = new CameraManagerConfig
            {
                Cameras =
                {
                    new CameraConfig
                    {
                        Id = "CAM_1", Name = "Door", SourceType = "network", Url = "rtsp://192.168.0.50:554/stream1",
                        Username = "admin", Password = "p@ss:w/rd", Transport = "udp", TargetFps = 10,
                    },
                },
            };
            JsonFiles.Save(path, original);
            var text = File.ReadAllText(path);
            Assert.Contains("\"sourceType\"", text);
            Assert.Contains("\"transport\"", text);

            var loaded = JsonFiles.Load<CameraManagerConfig>(path)!;
            var cam = Assert.Single(loaded.Cameras);
            Assert.Equal("network", cam.SourceType);
            Assert.Equal("rtsp://192.168.0.50:554/stream1", cam.Url);
            Assert.Equal("admin", cam.Username);
            Assert.Equal("p@ss:w/rd", cam.Password);
            Assert.Equal("udp", cam.Transport);
            Assert.Equal(10, cam.TargetFps);
        }
        finally { if (File.Exists(path)) File.Delete(path); }
    }

    [Fact]
    public void CameraStateSerialisesTheNewFields()
    {
        var state = new CameraDevice(new CameraConfig { Id = "C", SourceType = "NETWORK", Url = "http://cam/x", Transport = "bogus" }).GetState();
        var json = JsonSerializer.SerializeToElement(state);
        Assert.Equal("network", json.GetProperty("sourceType").GetString());
        Assert.Equal("tcp", json.GetProperty("transport").GetString());
        Assert.Equal("http://cam/x", json.GetProperty("url").GetString());
        Assert.Equal(0, json.GetProperty("streamWidth").GetInt32());
        Assert.Equal(0, json.GetProperty("streamHeight").GetInt32());
        Assert.Equal(0, json.GetProperty("latencyMs").GetInt32());
    }

    // ── TestCameraSource ─────────────────────────────────────────────────────

    private static JsonElement RunTest(TestCameraSourceParams p, int guardMs = 30000)
    {
        var task = CameraCommands.TestCameraSourceAsync(p);
        Assert.True(task.Wait(guardMs), "TestCameraSource did not return in time");
        return JsonSerializer.SerializeToElement(task.Result);
    }

    [Fact]
    public void TestCameraSourceRejectsAnUnparseableUrl()
    {
        var r = RunTest(new TestCameraSourceParams { Url = "not a url" }, 5000);
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.Equal("invalidUrl", r.GetProperty("error").GetString());
    }

    [Fact]
    public void TestCameraSourceFailsPromptlyForAnUnreachableHost()
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var r = RunTest(new TestCameraSourceParams { Url = "rtsp://127.0.0.1:1/x", TimeoutMs = 3000 });
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.Contains(r.GetProperty("error").GetString(), new[] { "openFailed", "timeout" });
        Assert.True(sw.ElapsedMilliseconds < 10000, $"took {sw.ElapsedMilliseconds} ms");
    }

    [Fact]
    public void FfmpegBackendIsDetected()
    {
        // The Windows and Linux runtime packages both ship FFmpeg; network cameras need it.
        Assert.True(NetworkCameraSource.FfmpegAvailable, "OpenCV build has no FFmpeg backend");
    }

    /// <summary>
    /// End to end through FFmpeg: an in-process HTTP server streams a generated JPEG as
    /// multipart/x-mixed-replace (what MJPEG IP cameras serve), and TestCameraSource must
    /// open it and decode a frame of the right size.
    /// </summary>
    [Fact]
    public void TestCameraSourceReadsAnInProcessMjpegStream()
    {
        using var server = new MjpegServer(320, 240);
        var r = RunTest(new TestCameraSourceParams { Url = $"http://127.0.0.1:{server.Port}/mjpeg", TimeoutMs = 8000 });
        Assert.True(r.GetProperty("ok").GetBoolean(), r.ToString());
        Assert.Equal(320, r.GetProperty("width").GetInt32());
        Assert.Equal(240, r.GetProperty("height").GetInt32());
        Assert.True(r.GetProperty("firstFrameMs").GetInt64() >= r.GetProperty("openMs").GetInt64());
    }

    [Fact]
    public void CameraDeviceStreamsFromAnInProcessMjpegSource()
    {
        using var server = new MjpegServer(320, 240);
        var device = new CameraDevice(new CameraConfig
        {
            Id = "NET_TEST", SourceType = "network", Url = $"http://127.0.0.1:{server.Port}/mjpeg",
            Width = 1920, Height = 1080, TargetFps = 30,
        });
        try
        {
            device.Start();
            var sw = System.Diagnostics.Stopwatch.StartNew();
            while (sw.ElapsedMilliseconds < 20000 && (device.GetLatestFrame() == null || device.StreamWidth == 0))
                Thread.Sleep(50);

            var jpeg = device.GetLatestFrame();
            Assert.NotNull(jpeg);
            Assert.True(device.Connected);
            // The stream's own size, not the configured width/height (no resolution Set calls).
            Assert.Equal(320, device.StreamWidth);
            Assert.Equal(240, device.StreamHeight);
            using var decoded = Cv2.ImDecode(jpeg!, ImreadModes.Color);
            Assert.Equal(320, decoded.Width);
        }
        finally { device.Stop(); }
        Assert.False(device.Connected);
    }

    /// <summary>Minimal MJPEG-over-HTTP source on a loopback port.</summary>
    private sealed class MjpegServer : IDisposable
    {
        private readonly TcpListener _listener = new(IPAddress.Loopback, 0);
        private readonly CancellationTokenSource _cts = new();
        private readonly byte[] _jpeg;
        public int Port { get; }

        public MjpegServer(int width, int height)
        {
            using (var img = new Mat(height, width, MatType.CV_8UC3, new Scalar(40, 90, 160)))
            {
                Cv2.Rectangle(img, new Rect(width / 4, height / 4, width / 2, height / 2), new Scalar(255, 255, 255), -1);
                _jpeg = img.ImEncode(".jpg");
            }
            _listener.Start();
            Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
            _ = Task.Run(AcceptLoop);
        }

        private async Task AcceptLoop()
        {
            while (!_cts.IsCancellationRequested)
            {
                TcpClient client;
                try { client = await _listener.AcceptTcpClientAsync(_cts.Token); }
                catch { return; }
                _ = Task.Run(() => Serve(client));
            }
        }

        private async Task Serve(TcpClient client)
        {
            using (client)
            {
                try
                {
                    var stream = client.GetStream();
                    // Read the request headers (up to the blank line); the request itself is ignored.
                    var buf = new byte[4096];
                    var head = new StringBuilder();
                    while (!head.ToString().Contains("\r\n\r\n"))
                    {
                        int n = await stream.ReadAsync(buf, _cts.Token);
                        if (n == 0) return;
                        head.Append(Encoding.ASCII.GetString(buf, 0, n));
                    }
                    await stream.WriteAsync(Encoding.ASCII.GetBytes(
                        "HTTP/1.0 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nCache-Control: no-cache\r\n\r\n"), _cts.Token);
                    while (!_cts.IsCancellationRequested)
                    {
                        await stream.WriteAsync(Encoding.ASCII.GetBytes(
                            $"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {_jpeg.Length}\r\n\r\n"), _cts.Token);
                        await stream.WriteAsync(_jpeg, _cts.Token);
                        await stream.WriteAsync("\r\n"u8.ToArray(), _cts.Token);
                        await Task.Delay(50, _cts.Token);
                    }
                }
                catch { /* client closed or server stopped */ }
            }
        }

        public void Dispose()
        {
            _cts.Cancel();
            try { _listener.Stop(); } catch { }
        }
    }
}
