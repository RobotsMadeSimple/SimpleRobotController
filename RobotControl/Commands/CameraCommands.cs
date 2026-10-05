using System.Text.Json;
using Controller.RobotControl.Camera;
using Controller.RobotControl.Camera.Sofia;

namespace Controller.RobotControl.Commands;

/// <summary>Camera configuration: USB devices, network (RTSP/HTTP) streams and Sofia/DVRIP cameras.</summary>
internal sealed class CameraCommands
{
    private readonly RobotController _robot;

    public CameraCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetCameras",                GetCameras);
        d.Add("AddCamera",                 AddCamera);
        d.Add("RemoveCamera",              RemoveCamera);
        d.Add("SetCameraConfig",           SetCameraConfig);
        d.AddAsync("GetCameraResolutions", GetCameraResolutions);
        d.AddAsync("TestCameraSource",     TestCameraSource);
    }

    private object? GetCameras(CommandMessage msg)
    {
        var states     = _robot.CameraManager.GetState();
        foreach (var s in states)
        {
            var cal = _robot.CalibrationRepo.Get(s.Id);
            s.Calibrated       = cal != null;
            s.CalibratedUnixMs = cal?.CalibratedUnixMs;
        }
        var statesJson = JsonSerializer.Serialize(states, CommandJson.CamelCase);
        return new { cameras = statesJson };
    }

    private void AddCamera(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<AddCameraParams>(msg);
        var sourceType = NetworkCameraSource.NormalizeSourceType(p.SourceType);
        _robot.CameraManager.AddCamera(new CameraConfig
        {
            Name        = p.Name,
            DeviceIndex = p.DeviceIndex,
            Enabled     = p.Enabled,
            Width       = p.Width,
            Height      = p.Height,
            TargetFps   = p.TargetFps,
            SourceType  = sourceType,
            Url         = p.Url?.Trim() ?? "",
            // A Sofia camera without a username logs in as "admin" (the contract default).
            Username    = string.IsNullOrEmpty(p.Username) && sourceType == NetworkCameraSource.SourceSofia
                              ? SofiaCameraSource.DefaultUsername : p.Username ?? "",
            Password    = p.Password ?? "",
            Transport   = NetworkCameraSource.NormalizeTransport(p.Transport),
            Host        = p.Host?.Trim() ?? "",
            Port        = SofiaCameraSource.NormalizePort(p.Port),
            Stream      = SofiaCameraSource.NormalizeStream(p.Stream),
            Codec       = SofiaCameraSource.NormalizeCodec(p.Codec),
            Decoder     = SofiaCameraSource.NormalizeDecoder(p.Decoder),
            FfmpegPath  = SofiaCameraSource.NormalizeFfmpegPath(p.FfmpegPath),
            Hwaccel     = SofiaCameraSource.NormalizeHwaccel(p.Hwaccel),
        });
    }

    private void RemoveCamera(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<RemoveCameraParams>(msg);
        _robot.CameraManager.RemoveCamera(p.Id);
    }

    private void SetCameraConfig(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetCameraConfigParams>(msg);
        // Absent network / Sofia fields keep the camera's current values (an older app does not send them).
        _robot.CameraManager.UpdateCamera(p.Id, MergeSetCameraConfig(p, _robot.CameraManager.GetCamera(p.Id)));
    }

    /// <summary>The config SetCameraConfig applies: the given fields, with absent source fields taken from <paramref name="current"/>.</summary>
    internal static CameraConfig MergeSetCameraConfig(SetCameraConfigParams p, CameraDevice? current)
    {
        return new CameraConfig
        {
            Id          = p.Id,
            Name        = p.Name,
            DeviceIndex = p.DeviceIndex,
            Enabled     = p.Enabled,
            Width       = p.Width,
            Height      = p.Height,
            TargetFps   = p.TargetFps,
            SourceType  = NetworkCameraSource.NormalizeSourceType(p.SourceType ?? current?.SourceType),
            Url         = (p.Url ?? current?.Url ?? "").Trim(),
            Username    = p.Username ?? current?.Username ?? "",
            Password    = p.Password ?? current?.Password ?? "",
            Transport   = NetworkCameraSource.NormalizeTransport(p.Transport ?? current?.Transport),
            Host        = (p.Host ?? current?.Host ?? "").Trim(),
            Port        = SofiaCameraSource.NormalizePort(p.Port ?? current?.Port),
            Stream      = SofiaCameraSource.NormalizeStream(p.Stream ?? current?.Stream),
            Codec       = SofiaCameraSource.NormalizeCodec(p.Codec ?? current?.Codec),
            Decoder     = SofiaCameraSource.NormalizeDecoder(p.Decoder ?? current?.Decoder),
            FfmpegPath  = SofiaCameraSource.NormalizeFfmpegPath(p.FfmpegPath ?? current?.FfmpegPath),
            Hwaccel     = SofiaCameraSource.NormalizeHwaccel(p.Hwaccel ?? current?.Hwaccel),
        };
    }

    private async Task<object?> GetCameraResolutions(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<GetCameraResolutionsParams>(msg);
        // A network or Sofia camera has no resolution list: the stream's own size is used.
        if (!string.IsNullOrEmpty(p.Id) && _robot.CameraManager.GetCamera(p.Id) is { IsUsb: false })
            return new { resolutions = "[]" };
        var deviceIndex = p.DeviceIndex;
        var resolutions = await Task.Run(() => _robot.CameraManager.ProbeResolutionsForIndex(deviceIndex));
        var json = JsonSerializer.Serialize(resolutions, CommandJson.CamelCase);
        return new { resolutions = json };
    }

    private Task<object?> TestCameraSource(CommandMessage msg) =>
        TestCameraSourceAsync(CommandJson.LoadParams<TestCameraSourceParams>(msg));

    /// <summary>The <c>TestCameraSource</c> response (static so tests can call it without a controller).</summary>
    internal static async Task<object?> TestCameraSourceAsync(TestCameraSourceParams p)
    {
        if (NetworkCameraSource.NormalizeSourceType(p.SourceType) == NetworkCameraSource.SourceSofia)
            return await TestSofiaSourceAsync(p);

        var r = await NetworkCameraSource.Test(p.Url, p.Username, p.Password, p.Transport,
                                               p.TimeoutMs > 0 ? p.TimeoutMs : NetworkCameraSource.DefaultOpenTimeoutMs);
        return new
        {
            ok           = r.Ok,
            width        = r.Width,
            height       = r.Height,
            openMs       = r.OpenMs,
            firstFrameMs = r.FirstFrameMs,
            error        = r.Error,
        };
    }

    private static async Task<object?> TestSofiaSourceAsync(TestCameraSourceParams p)
    {
        var settings = SofiaCameraSource.Settings(p.Host, p.Port, p.Username, p.Password, p.Stream,
                                                  p.Codec, p.Decoder, p.FfmpegPath, p.Hwaccel);
        var r = await SofiaCameraSource.Test(settings, p.TimeoutMs > 0 ? p.TimeoutMs : SofiaCameraSource.DefaultTimeoutMs);
        return new
        {
            ok              = r.Ok,
            loginMs         = r.LoginMs,
            firstFrameMs    = r.FirstFrameMs,
            detectedCodec   = r.DetectedCodec,
            firstFrameBytes = r.FirstFrameBytes,
            width           = r.Width,
            height          = r.Height,
            error           = r.Error,
            message         = r.Message,
        };
    }
}
