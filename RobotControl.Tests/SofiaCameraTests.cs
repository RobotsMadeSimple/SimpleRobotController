using System.Diagnostics;
using System.Net;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Camera;
using Controller.RobotControl.Camera.Sofia;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Persistence;
using OpenCvSharp;
using Xunit.Abstractions;

namespace RobotControl.Tests;

/// <summary>
/// Sofia / DVRIP (XMeye) camera sources — docs/network-cameras.md, "Sofia / DVRIP".
/// A fake camera on a loopback TcpListener speaks enough DVRIP to validate the client
/// (login digest, session, claim/start, frame reassembly, non-video skipping, resync).
/// Decoding real H.264 needs an encoder: the end-to-end decoder tests use Media
/// Foundation's H.264 encoder through OpenCV's MSMF VideoWriter (Windows only) and
/// return early with a note where it is unavailable.
/// </summary>
public sealed class SofiaCameraTests
{
    private readonly ITestOutputHelper _out;
    public SofiaCameraTests(ITestOutputHelper output) => _out = output;

    // ── Sofia digest ─────────────────────────────────────────────────────────

    /// <summary>The Sofia password digest, re-implemented from the reference: MD5, byte pairs folded mod 62.</summary>
    private static string ExpectedSofiaHash(string password)
    {
        const string alphabet = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
        var md5 = MD5.HashData(Encoding.UTF8.GetBytes(password));
        var sb = new StringBuilder();
        for (int i = 0; i < 16; i += 2) sb.Append(alphabet[(md5[i] + md5[i + 1]) % 62]);
        return sb.ToString();
    }

    [Fact]
    public void SofiaHashMatchesTheReferenceDigest()
    {
        // The widely published XMeye digest of the empty password.
        Assert.Equal("tlJwpbo6", ExpectedSofiaHash(""));
        foreach (var pw in new[] { "", "admin", "s3cret!", "pässwörd", "12345678901234567890" })
        {
            var h = DvripClient.SofiaHash(pw);
            Assert.Equal(8, h.Length);
            Assert.Equal(ExpectedSofiaHash(pw), h);
        }
    }

    // ── Login / claim ────────────────────────────────────────────────────────

    [Fact]
    public void ClientLogsInWithTheSofiaDigestAndParsesTheSession()
    {
        using var cam = new FakeDvripCamera { Script = FakeDvripCamera.Silent };
        using var client = new DvripClient("127.0.0.1", cam.Port, "operator", "s3cret!", "Extra1");
        client.Connect(5000);

        Assert.Equal(0x1234ABCDu, client.SessionId);
        Assert.Equal(21, client.AliveIntervalSeconds);
        Assert.True(cam.WaitForStart(5000), "monitor start never arrived");

        // 20-byte header layout: FF, version 0, 2 pad, session LE, sequence LE, 2 pad, msgid LE, length LE
        var h = cam.LoginHeader!;
        Assert.Equal(20, h.Length);
        Assert.Equal(0xFF, h[0]);
        Assert.Equal(0, h[1]);
        Assert.Equal(0u, BitConverter.ToUInt32(h, 4));     // no session yet
        Assert.Equal(0u, BitConverter.ToUInt32(h, 8));     // first packet
        Assert.Equal(1000, BitConverter.ToUInt16(h, 14));  // LoginRequest
        Assert.Equal((uint)cam.LoginBody!.Length, BitConverter.ToUInt32(h, 16));
        Assert.Equal(0x0A, cam.LoginBody[^2]);
        Assert.Equal(0x00, cam.LoginBody[^1]);

        var login = cam.LoginJson!.Value;
        Assert.Equal("operator", login.GetProperty("UserName").GetString());
        Assert.Equal(ExpectedSofiaHash("s3cret!"), login.GetProperty("PassWord").GetString());
        Assert.Equal("MD5", login.GetProperty("EncryptType").GetString());
        Assert.Equal("DVRIP-Web", login.GetProperty("LoginType").GetString());
        Assert.DoesNotContain("s3cret!", Encoding.UTF8.GetString(cam.LoginBody));

        // Claim then start, both carrying the session (in the header and the JSON)
        Assert.Equal(1413, BitConverter.ToUInt16(cam.ClaimHeader!, 14));
        Assert.Equal(0x1234ABCDu, BitConverter.ToUInt32(cam.ClaimHeader!, 4));
        Assert.Equal(1u, BitConverter.ToUInt32(cam.ClaimHeader!, 8));
        var claim = cam.ClaimJson!.Value;
        Assert.Equal("0x1234ABCD", claim.GetProperty("SessionID").GetString());
        Assert.Equal("Claim", claim.GetProperty("OPMonitor").GetProperty("Action").GetString());
        Assert.Equal("Extra1", claim.GetProperty("OPMonitor").GetProperty("Parameter").GetProperty("StreamType").GetString());
        Assert.Equal(1410, BitConverter.ToUInt16(cam.StartHeader!, 14));
        Assert.Equal("Start", cam.StartJson!.Value.GetProperty("OPMonitor").GetProperty("Action").GetString());
    }

    [Fact]
    public void RefusedLoginAndClaimSurfaceTheirStageAndRet()
    {
        using (var cam = new FakeDvripCamera { LoginRet = 203 })
        using (var client = new DvripClient("127.0.0.1", cam.Port, "admin", "wrong", "Main"))
        {
            var ex = Assert.Throws<DvripException>(() => client.Connect(5000));
            Assert.Equal(DvripStage.Login, ex.Stage);
            Assert.Equal(203, ex.Ret);
            Assert.DoesNotContain("wrong", ex.Message);
        }
        using (var cam = new FakeDvripCamera { ClaimRet = 503 })
        using (var client = new DvripClient("127.0.0.1", cam.Port, "admin", "", "Main"))
        {
            var ex = Assert.Throws<DvripException>(() => client.Connect(5000));
            Assert.Equal(DvripStage.Claim, ex.Stage);
            Assert.Equal(503, ex.Ret);
        }
    }

    // ── Media framing ────────────────────────────────────────────────────────

    [Fact]
    public void FramesAreReassembledAcrossPacketsNonVideoSkippedAndResyncRecovers()
    {
        var iFrame  = Payload(5000, 1, h264Sps: true);
        var p1      = Payload(2600, 2);
        var p2      = Payload(300, 3);
        var p3      = Payload(4096, 4);
        var iFrame2 = Payload(1500, 5, h264Sps: true);
        using var cam = new FakeDvripCamera
        {
            PacketSize = 1000,
            Script = s =>
            {
                s.Frame(FakeDvripCamera.IFrame, iFrame);          // 5 DVRIP packets
                s.Frame(FakeDvripCamera.PFrame, p1);              // 3 packets
                s.Json(new { Name = "KeepAlive", Ret = 100 });     // stray JSON reply: skipped
                s.Frame(FakeDvripCamera.Audio, Payload(640, 9));  // audio: skipped
                s.Frame(FakeDvripCamera.Jpeg, Payload(800, 8));   // snapshot JPEG: not video
                s.Frame(FakeDvripCamera.PFrame, p2);
                s.Garbage(777);                                   // desync: resync must skip it
                s.Frame(FakeDvripCamera.PFrame, p3);
                s.Frame(FakeDvripCamera.IFrame, iFrame2);
            },
        };
        using var client = new DvripClient("127.0.0.1", cam.Port, "admin", "", "Main");
        client.Connect(5000);

        var f1 = client.ReadVideoFrame();
        Assert.True(f1.IsKeyFrame);
        Assert.Equal(iFrame, f1.Data);
        var f2 = client.ReadVideoFrame();
        Assert.False(f2.IsKeyFrame);
        Assert.Equal(p1, f2.Data);
        Assert.Equal(p2, client.ReadVideoFrame().Data);           // JSON, audio and JPEG skipped
        Assert.Equal(0, client.ResyncCount);
        Assert.Equal(p3, client.ReadVideoFrame().Data);           // after the garbage run
        Assert.Equal(1, client.ResyncCount);
        Assert.Equal(777, client.ResyncSkippedBytes);
        var f5 = client.ReadVideoFrame();
        Assert.True(f5.IsKeyFrame);
        Assert.Equal(iFrame2, f5.Data);
    }

    [Fact]
    public void ResyncGivesUpOnAnEndlessGarbageStream()
    {
        using var cam = new FakeDvripCamera { Script = s => s.Garbage(DvripClient.MaxResyncScanBytes + 64 * 1024) };
        using var client = new DvripClient("127.0.0.1", cam.Port, "admin", "", "Main");
        client.Connect(5000);
        var ex = Assert.ThrowsAny<Exception>(() => client.ReadVideoFrame());
        Assert.True(ex is InvalidDataException, ex.ToString());
    }

    [Fact]
    public void CopyVideoToWritesOnlyTheVideoPayloads()
    {
        var a = Payload(1200, 1, h264Sps: true);
        var b = Payload(900, 2);
        using var cam = new FakeDvripCamera
        {
            PacketSize = 512,
            Script = s =>
            {
                s.Frame(FakeDvripCamera.IFrame, a);
                s.Frame(FakeDvripCamera.Audio, Payload(320, 7));
                s.Frame(FakeDvripCamera.PFrame, b);
            },
        };
        using var client = new DvripClient("127.0.0.1", cam.Port, "admin", "", "Main");
        client.Connect(5000);
        var ms = new MemoryStream();
        client.CopyVideoTo(ms, () => ms.Length < a.Length + b.Length);
        Assert.Equal(a.Concat(b).ToArray(), ms.ToArray());
    }

    [Fact]
    public void ASilentCameraSurfacesAsAReadTimeout()
    {
        using var cam = new FakeDvripCamera { Script = FakeDvripCamera.Silent };
        using var client = new DvripClient("127.0.0.1", cam.Port, "admin", "", "Main") { ReadTimeoutMs = 300 };
        client.Connect(5000);
        var sw = Stopwatch.StartNew();
        Assert.ThrowsAny<IOException>(() => client.ReadVideoFrame());
        Assert.True(sw.ElapsedMilliseconds < 5000, $"took {sw.ElapsedMilliseconds} ms");
    }

    [Fact]
    public void DisposeFromAnotherThreadUnblocksARead()
    {
        using var cam = new FakeDvripCamera { Script = FakeDvripCamera.Silent };
        var client = new DvripClient("127.0.0.1", cam.Port, "admin", "", "Main") { ReadTimeoutMs = 0 };
        client.Connect(5000);
        var reader = Task.Run(() => client.ReadVideoFrame());
        Thread.Sleep(200);
        client.Dispose();
        Assert.True(((IAsyncResult)reader).AsyncWaitHandle.WaitOne(3000), "read did not unblock");
        Assert.True(reader.IsFaulted);
    }

    // ── Codec detection ──────────────────────────────────────────────────────

    [Fact]
    public void DetectCodecReadsTheNalHeaderAfterTheStartCode()
    {
        Assert.Equal("h264", DvripClient.DetectCodec(new byte[] { 0, 0, 0, 1, 0x67, 0x42, 0, 0x1E }));
        Assert.Equal("h264", DvripClient.DetectCodec(new byte[] { 0, 0, 1, 0x67, 0x64 }));
        // An access-unit delimiter before the SPS is skipped
        Assert.Equal("h264", DvripClient.DetectCodec(new byte[] { 0, 0, 0, 1, 0x09, 0xF0, 0, 0, 0, 1, 0x67, 0x4D }));
        Assert.Equal("hevc", DvripClient.DetectCodec(new byte[] { 0, 0, 0, 1, 0x40, 0x01, 0x0C, 0x01 }));
        Assert.Equal("hevc", DvripClient.DetectCodec(new byte[] { 0, 0, 1, 0x42, 0x01, 0x01 }));
        Assert.Equal("unknown", DvripClient.DetectCodec(new byte[] { 0, 0, 0, 1, 0x41, 0x9A }));   // h264 P slice
        Assert.Equal("unknown", DvripClient.DetectCodec(new byte[] { 0, 0, 0, 1, 0x02, 0x01 }));   // hevc TRAIL_R
        Assert.Equal("unknown", DvripClient.DetectCodec(new byte[] { 1, 2, 3, 4, 5 }));
        Assert.Equal("unknown", DvripClient.DetectCodec(Array.Empty<byte>()));
        Assert.Equal("h264 SPS", DvripClient.DescribeNal(new byte[] { 0, 0, 0, 1, 0x67 }));
        Assert.Equal("hevc VPS", DvripClient.DescribeNal(new byte[] { 0, 0, 0, 1, 0x40, 1 }));
    }

    // ── Models / commands ────────────────────────────────────────────────────

    [Fact]
    public void ExistingConfigsLoadWithSofiaDefaults()
    {
        const string json = """
            {"cameras":[{"id":"CAM_0","name":"Top","deviceIndex":1,"enabled":true,"width":1280,"height":720,"targetFps":15},
                        {"id":"CAM_1","name":"Door","sourceType":"network","url":"rtsp://cam/s","username":"u","password":"p","transport":"udp"}]}
            """;
        var cfg = JsonSerializer.Deserialize<CameraManagerConfig>(json, JsonDefaults.File)!;
        Assert.Equal(2, cfg.Cameras.Count);
        foreach (var c in cfg.Cameras)
        {
            Assert.Equal("", c.Host);
            Assert.Equal(34567, c.Port);
            Assert.Equal("Main", c.Stream);
            Assert.Equal("h264", c.Codec);
            Assert.Equal("opencv", c.Decoder);
            Assert.Equal("ffmpeg", c.FfmpegPath);
            Assert.Equal("", c.Hwaccel);
        }
        Assert.Equal("usb", cfg.Cameras[0].SourceType);
        Assert.Equal("network", cfg.Cameras[1].SourceType);
        Assert.Equal("udp", cfg.Cameras[1].Transport);
        Assert.True(new CameraDevice(cfg.Cameras[0]).IsUsb);
        Assert.True(new CameraDevice(cfg.Cameras[1]).IsNetwork);
    }

    [Fact]
    public void SofiaConfigRoundTripsAndNormalises()
    {
        var path = Path.Combine(Path.GetTempPath(), "sofiacam-" + Guid.NewGuid().ToString("N") + ".json");
        try
        {
            JsonFiles.Save(path, new CameraManagerConfig
            {
                Cameras =
                {
                    new CameraConfig
                    {
                        Id = "CAM_2", SourceType = "sofia", Host = "192.168.1.10", Port = 34568, Username = "admin",
                        Password = "pw", Stream = "Extra1", Codec = "hevc", Decoder = "ffmpeg",
                        FfmpegPath = @"C:\ffmpeg\bin\ffmpeg.exe", Hwaccel = "d3d11va",
                    },
                },
            });
            var cam = Assert.Single(JsonFiles.Load<CameraManagerConfig>(path)!.Cameras);
            Assert.Equal("sofia", cam.SourceType);
            Assert.Equal("192.168.1.10", cam.Host);
            Assert.Equal(34568, cam.Port);
            Assert.Equal("Extra1", cam.Stream);
            Assert.Equal("hevc", cam.Codec);
            Assert.Equal("ffmpeg", cam.Decoder);
            Assert.Equal(@"C:\ffmpeg\bin\ffmpeg.exe", cam.FfmpegPath);
            Assert.Equal("d3d11va", cam.Hwaccel);
        }
        finally { if (File.Exists(path)) File.Delete(path); }

        var device = new CameraDevice(new CameraConfig
        {
            Id = "S", SourceType = "SOFIA", Host = " cam ", Port = 0, Stream = "sub", Codec = "H265", Decoder = "bogus",
            FfmpegPath = " ", Username = "", Password = "hunter2",
        });
        Assert.True(device.IsSofia);
        Assert.False(device.IsNetwork);
        Assert.False(device.IsUsb);
        Assert.Empty(device.ProbeResolutions());
        Assert.Equal("cam", device.Host);
        Assert.Equal(34567, device.Port);
        Assert.Equal("Extra1", device.Stream);
        Assert.Equal("hevc", device.Codec);
        Assert.Equal("opencv", device.Decoder);
        Assert.Equal("ffmpeg", device.FfmpegPath);
        Assert.Equal("admin", device.SofiaSettings().Username);
        Assert.DoesNotContain("hunter2", device.MaskedUrl());
        Assert.Equal("sofia://admin@cam:34567/Extra1", device.MaskedUrl());

        var state = JsonSerializer.SerializeToElement(device.GetState());
        Assert.Equal("sofia", state.GetProperty("sourceType").GetString());
        Assert.Equal("cam", state.GetProperty("host").GetString());
        Assert.Equal(34567, state.GetProperty("port").GetInt32());
        Assert.Equal("Extra1", state.GetProperty("stream").GetString());
        Assert.Equal("hevc", state.GetProperty("codec").GetString());
        Assert.Equal("opencv", state.GetProperty("decoder").GetString());
        Assert.Equal("ffmpeg", state.GetProperty("ffmpegPath").GetString());
        Assert.Equal("", state.GetProperty("hwaccel").GetString());
    }

    [Fact]
    public void SetCameraConfigKeepsUnspecifiedSofiaFieldsAndRestartsOnChange()
    {
        var cfg = new CameraConfig
        {
            Id = "S1", Name = "Sofia", SourceType = "sofia", Host = "10.0.0.7", Port = 34569, Username = "admin",
            Password = "pw", Stream = "Extra1", Codec = "hevc", Decoder = "ffmpeg", FfmpegPath = "/usr/bin/ffmpeg", Hwaccel = "auto",
        };
        var device = new CameraDevice(cfg);

        // An older app sends only the basic fields: nothing Sofia-specific may change.
        var merged = CameraCommands.MergeSetCameraConfig(new SetCameraConfigParams { Id = "S1", Name = "Renamed" }, device);
        Assert.Equal("sofia", merged.SourceType);
        Assert.Equal("10.0.0.7", merged.Host);
        Assert.Equal(34569, merged.Port);
        Assert.Equal("admin", merged.Username);
        Assert.Equal("pw", merged.Password);
        Assert.Equal("Extra1", merged.Stream);
        Assert.Equal("hevc", merged.Codec);
        Assert.Equal("ffmpeg", merged.Decoder);
        Assert.Equal("/usr/bin/ffmpeg", merged.FfmpegPath);
        Assert.Equal("auto", merged.Hwaccel);
        Assert.False(device.SourceDiffers(merged));

        // Each Sofia field is capture-affecting.
        Assert.True(device.SourceDiffers(CameraCommands.MergeSetCameraConfig(new SetCameraConfigParams { Id = "S1", Port = 34567 }, device)));
        Assert.True(device.SourceDiffers(CameraCommands.MergeSetCameraConfig(new SetCameraConfigParams { Id = "S1", Host = "10.0.0.8" }, device)));
        Assert.True(device.SourceDiffers(CameraCommands.MergeSetCameraConfig(new SetCameraConfigParams { Id = "S1", Stream = "Main" }, device)));
        Assert.True(device.SourceDiffers(CameraCommands.MergeSetCameraConfig(new SetCameraConfigParams { Id = "S1", Codec = "h264" }, device)));
        Assert.True(device.SourceDiffers(CameraCommands.MergeSetCameraConfig(new SetCameraConfigParams { Id = "S1", Decoder = "opencv" }, device)));
        Assert.True(device.SourceDiffers(CameraCommands.MergeSetCameraConfig(new SetCameraConfigParams { Id = "S1", FfmpegPath = "ffmpeg" }, device)));
        Assert.True(device.SourceDiffers(CameraCommands.MergeSetCameraConfig(new SetCameraConfigParams { Id = "S1", Hwaccel = "" }, device)));

        // A USB camera saved by an old app stays USB with default Sofia fields.
        var usb = new CameraDevice(new CameraConfig { Id = "U", DeviceIndex = 2 });
        var usbMerged = CameraCommands.MergeSetCameraConfig(new SetCameraConfigParams { Id = "U", DeviceIndex = 2 }, usb);
        Assert.Equal("usb", usbMerged.SourceType);
        Assert.False(usb.SourceDiffers(usbMerged));
    }

    [Fact]
    public void FfmpegDecoderArgumentsMatchTheContract()
    {
        Assert.Equal(
            "-hide_banner -loglevel error -f h264 -fflags nobuffer -flags low_delay -i pipe:0 -an -f mjpeg -q:v 4 -flush_packets 1 pipe:1",
            string.Join(" ", FfmpegProcessDecoder.BuildArguments("h264", "")));
        Assert.Equal(
            "-hide_banner -loglevel error -hwaccel d3d11va -f hevc -fflags nobuffer -flags low_delay -i pipe:0 -an -f mjpeg -q:v 4 -flush_packets 1 pipe:1",
            string.Join(" ", FfmpegProcessDecoder.BuildArguments("hevc", "d3d11va")));
    }

    [Fact]
    public void MissingFfmpegExecutableIsReportedAsUnavailable()
    {
        var ex = Assert.Throws<DecoderUnavailableException>(() =>
            new FfmpegProcessDecoder("t", Path.Combine(Path.GetTempPath(), "no-such-dir", "ffmpeg-missing.exe"), "h264", ""));
        Assert.Contains("ffmpeg not found at", ex.Message);
    }

    [Fact]
    public void JpegSizeIsReadFromTheFrameHeader()
    {
        using var img = new Mat(123, 321, MatType.CV_8UC3, new Scalar(1, 2, 3));
        Assert.True(JpegInfo.TryGetSize(img.ImEncode(".jpg"), out int w, out int h));
        Assert.Equal(321, w);
        Assert.Equal(123, h);
        Assert.False(JpegInfo.TryGetSize(new byte[] { 1, 2, 3, 4 }, out _, out _));
    }

    // ── TestCameraSource ─────────────────────────────────────────────────────

    private static JsonElement RunTest(TestCameraSourceParams p, int guardMs = 30000)
    {
        var task = CameraCommands.TestCameraSourceAsync(p);
        Assert.True(task.Wait(guardMs), "TestCameraSource did not return in time");
        return JsonSerializer.SerializeToElement(task.Result);
    }

    private static TestCameraSourceParams SofiaParams(int port, int timeoutMs = 3000, string? decoder = null, string? ffmpegPath = null) => new()
    {
        SourceType = "sofia", Host = "127.0.0.1", Port = port, Username = "admin", Password = "pw",
        TimeoutMs = timeoutMs, Decoder = decoder, FfmpegPath = ffmpegPath,
    };

    [Fact]
    public void TestCameraSourceReportsLoginFailedWithTheRetCode()
    {
        using var cam = new FakeDvripCamera { LoginRet = 203 };
        var r = RunTest(SofiaParams(cam.Port));
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.Equal("loginFailed", r.GetProperty("error").GetString());
        Assert.Contains("203", r.GetProperty("message").GetString());
        Assert.DoesNotContain("pw", r.GetProperty("message").GetString()!.Replace("DVRIP", ""));
    }

    [Fact]
    public void TestCameraSourceReportsClaimFailed()
    {
        using var cam = new FakeDvripCamera { ClaimRet = 503 };
        var r = RunTest(SofiaParams(cam.Port));
        Assert.Equal("claimFailed", r.GetProperty("error").GetString());
    }

    [Fact]
    public void TestCameraSourceReportsConnectFailedForAClosedPort()
    {
        var l = new TcpListener(IPAddress.Loopback, 0);
        l.Start();
        int port = ((IPEndPoint)l.LocalEndpoint).Port;
        l.Stop();
        var sw = Stopwatch.StartNew();
        var r = RunTest(SofiaParams(port));
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.Equal("connectFailed", r.GetProperty("error").GetString());
        Assert.True(sw.ElapsedMilliseconds < 5000, $"took {sw.ElapsedMilliseconds} ms");
    }

    [Fact]
    public void TestCameraSourceReportsNoFrameWhenTheCameraNeverStreams()
    {
        using var cam = new FakeDvripCamera { Script = FakeDvripCamera.Silent };
        var sw = Stopwatch.StartNew();
        var r = RunTest(SofiaParams(cam.Port, timeoutMs: 2000));
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.Equal("noFrame", r.GetProperty("error").GetString());
        Assert.True(r.GetProperty("loginMs").GetInt64() < 2000);
        Assert.True(sw.ElapsedMilliseconds < 4500, $"took {sw.ElapsedMilliseconds} ms");
    }

    [Fact]
    public void TestCameraSourceReportsDecoderUnavailableForAMissingFfmpeg()
    {
        var frame = Payload(900, 1, h264Sps: true);
        using var cam = new FakeDvripCamera { Script = s => s.Frame(FakeDvripCamera.IFrame, frame) };
        var r = RunTest(SofiaParams(cam.Port, decoder: "ffmpeg", ffmpegPath: Path.Combine(Path.GetTempPath(), "nope", "ffmpeg.exe")));
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.Equal("decoderUnavailable", r.GetProperty("error").GetString());
        Assert.Contains("ffmpeg not found at", r.GetProperty("message").GetString());
        // The handshake and first frame still succeeded and are reported.
        Assert.Equal("h264", r.GetProperty("detectedCodec").GetString());
        Assert.Equal(frame.Length, r.GetProperty("firstFrameBytes").GetInt32());
    }

    [Fact]
    public void TestCameraSourceSucceedsWithoutSizeWhenTheStreamCannotBeDecoded()
    {
        // Synthetic (non-decodable) HEVC-looking frames: login and first frame work, the
        // in-process decoder produces no picture within the budget, so ok with width/height 0.
        var frame = Payload(700, 1, hevcVps: true);
        using var cam = new FakeDvripCamera { Script = s => s.Frame(FakeDvripCamera.IFrame, frame) };
        var sw = Stopwatch.StartNew();
        var r = RunTest(SofiaParams(cam.Port, timeoutMs: 2500));
        Assert.True(r.GetProperty("ok").GetBoolean(), r.ToString());
        Assert.Null(r.GetProperty("error").GetString());
        Assert.Equal("hevc", r.GetProperty("detectedCodec").GetString());
        Assert.Equal(0, r.GetProperty("width").GetInt32());
        Assert.True(r.GetProperty("firstFrameMs").GetInt64() >= r.GetProperty("loginMs").GetInt64());
        Assert.True(sw.ElapsedMilliseconds < 6000, $"took {sw.ElapsedMilliseconds} ms");
    }

    [Fact]
    public void NetworkTestCameraSourceIsUnchangedWhenSourceTypeIsAbsent()
    {
        var r = RunTest(new TestCameraSourceParams { Url = "not a url" }, 5000);
        Assert.Equal("invalidUrl", r.GetProperty("error").GetString());
        Assert.True(r.TryGetProperty("openMs", out _));
    }

    // ── CameraDevice ─────────────────────────────────────────────────────────

    [Fact]
    public void SofiaCameraWithMissingFfmpegStaysDisconnectedAndStopsPromptly()
    {
        using var cam = new FakeDvripCamera { Loop = true, Script = s => s.Frame(FakeDvripCamera.IFrame, Payload(500, 1, h264Sps: true)) };
        var device = new CameraDevice(new CameraConfig
        {
            Id = "SOFIA_NOFF", SourceType = "sofia", Host = "127.0.0.1", Port = cam.Port, Decoder = "ffmpeg",
            FfmpegPath = Path.Combine(Path.GetTempPath(), "nope", "ffmpeg.exe"),
        });
        try
        {
            device.Start();
            var sw = Stopwatch.StartNew();
            while (sw.ElapsedMilliseconds < 5000 && cam.Connections < 1) Thread.Sleep(50);
            Assert.True(cam.Connections >= 1, "the device never connected to the fake camera");
            Thread.Sleep(500);
            Assert.False(device.Connected);
            Assert.Null(device.GetLatestFrame());
        }
        finally
        {
            var sw = Stopwatch.StartNew();
            device.Stop();
            Assert.True(sw.ElapsedMilliseconds < 3000, $"Stop took {sw.ElapsedMilliseconds} ms");
        }
    }

    // ── End to end with real H.264 (Media Foundation encoder, Windows) ────────

    [Fact]
    public void TestCameraSourceDecodesARealH264StreamInProcess()
    {
        var aus = EncodedH264.AccessUnits(_out);
        if (aus == null) return;   // no H.264 encoder here (see EncodedH264): nothing to decode

        using var cam = new FakeDvripCamera { Loop = true, FrameDelayMs = 66, Script = s => { foreach (var au in aus) s.Frame(au.Key ? FakeDvripCamera.IFrame : FakeDvripCamera.PFrame, au.Data); } };
        var r = RunTest(SofiaParams(cam.Port, timeoutMs: 8000));
        _out.WriteLine(r.ToString());
        Assert.True(r.GetProperty("ok").GetBoolean(), r.ToString());
        Assert.Equal("h264", r.GetProperty("detectedCodec").GetString());
        Assert.Equal(320, r.GetProperty("width").GetInt32());
        Assert.Equal(240, r.GetProperty("height").GetInt32());
    }

    [Fact]
    public void CameraDeviceStreamsARealH264SofiaSourceInProcess()
    {
        var aus = EncodedH264.AccessUnits(_out);
        if (aus == null) return;

        using var cam = new FakeDvripCamera { Loop = true, FrameDelayMs = 66, Script = s => { foreach (var au in aus) s.Frame(au.Key ? FakeDvripCamera.IFrame : FakeDvripCamera.PFrame, au.Data); } };
        var device = new CameraDevice(new CameraConfig
        {
            // Configured as hevc on purpose: the detected codec (h264) must win.
            Id = "SOFIA_E2E", SourceType = "sofia", Host = "127.0.0.1", Port = cam.Port, Codec = "hevc", TargetFps = 30,
        });
        try
        {
            device.Start();
            var sw = Stopwatch.StartNew();
            while (sw.ElapsedMilliseconds < 20000 && (device.GetLatestFrame() == null || device.StreamWidth == 0))
                Thread.Sleep(50);
            var jpeg = device.GetLatestFrame();
            Assert.NotNull(jpeg);
            Assert.True(device.Connected);
            Assert.Equal(320, device.StreamWidth);
            Assert.Equal(240, device.StreamHeight);
            using var decoded = Cv2.ImDecode(jpeg!, ImreadModes.Color);
            Assert.Equal(320, decoded.Width);
            _out.WriteLine($"first picture after {sw.ElapsedMilliseconds} ms, latency {device.LatencyMs} ms");
        }
        finally { device.Stop(); }
        Assert.False(device.Connected);
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    /// <summary>Deterministic payload bytes; optionally starting with an H.264 SPS or HEVC VPS NAL.</summary>
    private static byte[] Payload(int length, int seed, bool h264Sps = false, bool hevcVps = false)
    {
        var data = new byte[length];
        new Random(seed).NextBytes(data);
        if (h264Sps) new byte[] { 0, 0, 0, 1, 0x67, 0x42, 0x00, 0x1E }.CopyTo(data, 0);
        if (hevcVps) new byte[] { 0, 0, 0, 1, 0x40, 0x01, 0x0C, 0x01 }.CopyTo(data, 0);
        return data;
    }

    /// <summary>
    /// A 320x240 H.264 clip encoded by Media Foundation (OpenCV's MSMF writer) and converted
    /// from MP4 (length-prefixed NALs) to Annex-B access units, SPS/PPS in front of each IDR.
    /// null (with a note) where no H.264 encoder is available — the OpenCV FFmpeg writer has
    /// no H.264 encoder in the prebuilt runtimes (it needs openh264), and MSMF is Windows-only.
    /// </summary>
    private static class EncodedH264
    {
        private static readonly Lazy<List<(byte[] Data, bool Key)>?> _aus = new(Build);
        private static string _note = "";

        public static List<(byte[] Data, bool Key)>? AccessUnits(ITestOutputHelper o)
        {
            var v = _aus.Value;
            if (v == null) o.WriteLine("SKIPPED: " + _note);
            return v;
        }

        private static List<(byte[] Data, bool Key)>? Build()
        {
            if (!OperatingSystem.IsWindows()) { _note = "no H.264 encoder: Media Foundation is Windows-only and the OpenCV FFmpeg writer lacks one"; return null; }
            var path = Path.Combine(Path.GetTempPath(), "sofia-h264-" + Guid.NewGuid().ToString("N") + ".mp4");
            try
            {
                using (var w = new VideoWriter(path, VideoCaptureAPIs.MSMF, FourCC.FromString("avc1"), 15, new OpenCvSharp.Size(320, 240)))
                {
                    if (!w.IsOpened()) { _note = "the MSMF H.264 writer did not open"; return null; }
                    for (int i = 0; i < 30; i++)
                    {
                        using var m = new Mat(240, 320, MatType.CV_8UC3, new Scalar(i * 8, 100, 200));
                        Cv2.PutText(m, i.ToString(), new OpenCvSharp.Point(60, 140), HersheyFonts.HersheySimplex, 2, Scalar.White, 3);
                        w.Write(m);
                    }
                }
                var aus = Mp4ToAnnexB(File.ReadAllBytes(path));
                if (aus.Count == 0 || !aus[0].Key) { _note = "could not parse the encoded MP4"; return null; }
                return aus;
            }
            catch (Exception ex) { _note = "H.264 encode failed: " + ex.Message; return null; }
            finally { try { File.Delete(path); } catch { } }
        }

        private static uint U32(byte[] b, int o) => (uint)(b[o] << 24 | b[o + 1] << 16 | b[o + 2] << 8 | b[o + 3]);

        private static IEnumerable<(string Type, int Start, int End)> Boxes(byte[] b, int start, int end)
        {
            int o = start;
            while (o + 8 <= end)
            {
                long size = U32(b, o);
                string type = Encoding.ASCII.GetString(b, o + 4, 4);
                int hdr = 8;
                if (size == 1) { size = (long)U32(b, o + 8) << 32 | U32(b, o + 12); hdr = 16; }
                else if (size == 0) size = end - o;
                if (size < hdr) yield break;
                yield return (type, o + hdr, (int)(o + size));
                o += (int)size;
            }
        }

        private static (int Start, int End) Find(byte[] b, int s, int e, params string[] path)
        {
            foreach (var box in Boxes(b, s, e))
                if (box.Type == path[0])
                    return path.Length == 1 ? (box.Start, box.End) : Find(b, box.Start, box.End, path[1..]);
            throw new InvalidDataException("box not found: " + path[0]);
        }

        private static List<(byte[] Data, bool Key)> Mp4ToAnnexB(byte[] mp4)
        {
            var stbl = Find(mp4, 0, mp4.Length, "moov", "trak", "mdia", "minf", "stbl");
            var stsd = Find(mp4, stbl.Start, stbl.End, "stsd");
            int entry = stsd.Start + 8;                                   // version/flags + entry count
            int entryEnd = entry + (int)U32(mp4, entry);
            var avcC = Find(mp4, entry + 8 + 78, entryEnd, "avcC");       // after the visual sample entry fields
            int p = avcC.Start;
            int lenSize = (mp4[p + 4] & 3) + 1;
            int nsps = mp4[p + 5] & 0x1F;
            p += 6;
            var parameterSets = new List<byte[]>();
            for (int i = 0; i < nsps; i++) { int n = mp4[p] << 8 | mp4[p + 1]; parameterSets.Add(mp4[(p + 2)..(p + 2 + n)]); p += 2 + n; }
            int npps = mp4[p++];
            for (int i = 0; i < npps; i++) { int n = mp4[p] << 8 | mp4[p + 1]; parameterSets.Add(mp4[(p + 2)..(p + 2 + n)]); p += 2 + n; }

            var stsz = Find(mp4, stbl.Start, stbl.End, "stsz");
            var stco = Find(mp4, stbl.Start, stbl.End, "stco");
            var stsc = Find(mp4, stbl.Start, stbl.End, "stsc");
            uint fixedSize = U32(mp4, stsz.Start + 4);
            int count = (int)U32(mp4, stsz.Start + 8);
            var sizes = new int[count];
            for (int i = 0; i < count; i++) sizes[i] = (int)(fixedSize != 0 ? fixedSize : U32(mp4, stsz.Start + 12 + 4 * i));
            int chunks = (int)U32(mp4, stco.Start + 4);
            int nsc = (int)U32(mp4, stsc.Start + 4);
            var sc = new List<(int First, int Per)>();
            for (int i = 0; i < nsc; i++) sc.Add(((int)U32(mp4, stsc.Start + 8 + 12 * i), (int)U32(mp4, stsc.Start + 12 + 12 * i)));

            var result = new List<(byte[] Data, bool Key)>();
            int sample = 0;
            byte[] startCode = { 0, 0, 0, 1 };
            for (int c = 0; c < chunks && sample < count; c++)
            {
                int per = sc.Last(x => x.First <= c + 1).Per;
                long o = U32(mp4, stco.Start + 8 + 4 * c);
                for (int k = 0; k < per && sample < count; k++, sample++)
                {
                    var ms = new MemoryStream();
                    int q = (int)o, end = q + sizes[sample];
                    bool idr = false;
                    while (q + lenSize <= end)
                    {
                        int n = 0;
                        for (int j = 0; j < lenSize; j++) n = n << 8 | mp4[q + j];
                        q += lenSize;
                        if ((mp4[q] & 0x1F) == 5 && !idr)
                        {
                            idr = true;
                            foreach (var ps in parameterSets) { ms.Write(startCode); ms.Write(ps); }
                        }
                        ms.Write(startCode);
                        ms.Write(mp4, q, n);
                        q += n;
                    }
                    result.Add((ms.ToArray(), idr));
                    o += sizes[sample];
                }
            }
            return result;
        }
    }

    /// <summary>
    /// A fake XMeye camera: answers login (Ret, SessionID 0x1234ABCD, AliveInterval 21) and
    /// the monitor claim, records what the client sent, then plays a script of DVRIP media
    /// packets (Sofia frames split into packets of <see cref="PacketSize"/>), optionally in a loop.
    /// </summary>
    private sealed class FakeDvripCamera : IDisposable
    {
        public const uint IFrame = 0x000001FC;
        public const uint PFrame = 0x000001FD;
        public const uint Jpeg   = 0x000001FE;
        public const uint Audio  = 0x000001FA;

        public static readonly Action<ScriptWriter> Silent = _ => { };

        private readonly TcpListener _listener = new(IPAddress.Loopback, 0);
        private readonly CancellationTokenSource _cts = new();
        private readonly ManualResetEventSlim _started = new(false);
        private int _connections;

        public int  Port        { get; }
        public int  LoginRet    { get; init; } = 100;
        public int  ClaimRet    { get; init; } = 100;
        public int  PacketSize  { get; init; } = 8192;
        public bool Loop        { get; init; }
        public int  FrameDelayMs { get; init; }
        public Action<ScriptWriter> Script { get; init; } = Silent;
        public int  Connections => Volatile.Read(ref _connections);

        public byte[]? LoginHeader, LoginBody, ClaimHeader, StartHeader;
        public JsonElement? LoginJson, ClaimJson, StartJson;

        public FakeDvripCamera()
        {
            _listener.Start();
            Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
            _ = Task.Run(AcceptLoop);
        }

        public bool WaitForStart(int ms) => _started.Wait(ms);

        private async Task AcceptLoop()
        {
            while (!_cts.IsCancellationRequested)
            {
                TcpClient c;
                try { c = await _listener.AcceptTcpClientAsync(_cts.Token); }
                catch { return; }
                Interlocked.Increment(ref _connections);
                _ = Task.Run(() => Serve(c));
            }
        }

        private void Serve(TcpClient c)
        {
            using (c)
            {
                try
                {
                    c.NoDelay = true;
                    var s = c.GetStream();
                    var (h, body) = ReadPacket(s);
                    LoginHeader = h; LoginBody = body; LoginJson = ParseJson(body);
                    Reply(s, 1001, LoginRet == 100
                        ? new { Ret = LoginRet, SessionID = "0x1234ABCD", AliveInterval = 21, DeviceType = "IPC" }
                        : (object)new { Ret = LoginRet, SessionID = "0x00000000" });
                    if (LoginRet != 100) { Thread.Sleep(200); return; }

                    (h, body) = ReadPacket(s);
                    ClaimHeader = h; ClaimJson = ParseJson(body);
                    Reply(s, 1414, new { Name = "OPMonitor", Ret = ClaimRet, SessionID = "0x1234ABCD" });
                    if (ClaimRet != 100) { Thread.Sleep(200); return; }

                    (h, body) = ReadPacket(s);
                    StartHeader = h; StartJson = ParseJson(body);
                    _started.Set();

                    // Drain keepalives so the client's writes never block.
                    _ = Task.Run(() => { var buf = new byte[4096]; try { while (s.Read(buf) > 0) { } } catch { } });

                    var writer = new ScriptWriter(s, PacketSize, FrameDelayMs, _cts.Token);
                    do Script(writer); while (Loop && !_cts.IsCancellationRequested);
                    // Keep the connection open (a silent camera) until disposed.
                    _cts.Token.WaitHandle.WaitOne();
                }
                catch { /* client went away or server stopped */ }
            }
        }

        private static JsonElement ParseJson(byte[] body)
        {
            int end = body.Length;
            while (end > 0 && (body[end - 1] == 0 || body[end - 1] == 0x0A)) end--;
            using var doc = JsonDocument.Parse(body.AsMemory(0, end));
            return doc.RootElement.Clone();
        }

        private static (byte[] Header, byte[] Body) ReadPacket(NetworkStream s)
        {
            var h = ReadExact(s, 20);
            var body = ReadExact(s, (int)BitConverter.ToUInt32(h, 16));
            return (h, body);
        }

        private static byte[] ReadExact(NetworkStream s, int n)
        {
            var b = new byte[n];
            int o = 0;
            while (o < n)
            {
                int r = s.Read(b, o, n - o);
                if (r <= 0) throw new EndOfStreamException();
                o += r;
            }
            return b;
        }

        private static void Reply(NetworkStream s, int msgId, object json)
        {
            var body = JsonSerializer.SerializeToUtf8Bytes(json).Concat(new byte[] { 0x0A, 0x00 }).ToArray();
            s.Write(Header(msgId, body.Length, 0));
            s.Write(body);
        }

        public static byte[] Header(int msgId, int length, uint seq)
        {
            var h = new byte[20];
            h[0] = 0xFF;
            h[1] = 0x01;
            BitConverter.GetBytes(0x1234ABCDu).CopyTo(h, 4);
            BitConverter.GetBytes(seq).CopyTo(h, 8);
            BitConverter.GetBytes((ushort)msgId).CopyTo(h, 14);
            BitConverter.GetBytes((uint)length).CopyTo(h, 16);
            return h;
        }

        public sealed class ScriptWriter
        {
            private readonly NetworkStream _s;
            private readonly int _packet, _delay;
            private readonly CancellationToken _ct;
            private uint _seq;

            public ScriptWriter(NetworkStream s, int packet, int delay, CancellationToken ct)
            {
                _s = s; _packet = Math.Max(16, packet); _delay = delay; _ct = ct;
            }

            /// <summary>One Sofia frame (16-byte header for I/JPEG, 8-byte otherwise) split into DVRIP packets.</summary>
            public void Frame(uint magic, byte[] payload)
            {
                _ct.ThrowIfCancellationRequested();
                byte[] header;
                if (magic == IFrame || magic == Jpeg)
                {
                    header = new byte[16];
                    header[4] = 0x02; header[5] = 15; header[6] = 320 / 8; header[7] = 240 / 8;   // type, fps, w/8, h/8
                    BitConverter.GetBytes((uint)payload.Length).CopyTo(header, 12);
                }
                else
                {
                    header = new byte[8];
                    BitConverter.GetBytes((uint)payload.Length).CopyTo(header, 4);
                }
                header[0] = (byte)(magic >> 24); header[1] = (byte)(magic >> 16); header[2] = (byte)(magic >> 8); header[3] = (byte)magic;

                var frame = header.Concat(payload).ToArray();
                for (int o = 0; o < frame.Length; o += _packet)
                {
                    int n = Math.Min(_packet, frame.Length - o);
                    _s.Write(Header(1412, n, _seq++));
                    _s.Write(frame, o, n);
                }
                _s.Flush();
                if (_delay > 0) Thread.Sleep(_delay);
            }

            /// <summary>A DVRIP packet carrying JSON instead of media (the client must skip it).</summary>
            public void Json(object json)
            {
                var body = JsonSerializer.SerializeToUtf8Bytes(json).Concat(new byte[] { 0x0A, 0x00 }).ToArray();
                _s.Write(Header(1007, body.Length, _seq++));
                _s.Write(body);
            }

            /// <summary>Raw bytes where a DVRIP header is expected: an out-of-range length forces a resync.</summary>
            public void Garbage(int count)
            {
                var g = new byte[count];
                Array.Fill(g, (byte)0xEE);
                _s.Write(g);
                _s.Flush();
            }
        }

        public void Dispose()
        {
            _cts.Cancel();
            try { _listener.Stop(); } catch { }
        }
    }
}
