using System.Text.Json;
using Controller.RobotControl.Camera;

namespace Controller.RobotControl.Commands;

/// <summary>Camera configuration: USB devices and network (RTSP/HTTP) streams.</summary>
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
        _robot.CameraManager.AddCamera(new CameraConfig
        {
            Name        = p.Name,
            DeviceIndex = p.DeviceIndex,
            Enabled     = p.Enabled,
            Width       = p.Width,
            Height      = p.Height,
            TargetFps   = p.TargetFps,
            SourceType  = NetworkCameraSource.NormalizeSourceType(p.SourceType),
            Url         = p.Url?.Trim() ?? "",
            Username    = p.Username ?? "",
            Password    = p.Password ?? "",
            Transport   = NetworkCameraSource.NormalizeTransport(p.Transport),
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
        // Absent network fields keep the camera's current values (an older app does not send them).
        var current = _robot.CameraManager.GetCamera(p.Id);
        _robot.CameraManager.UpdateCamera(p.Id, new CameraConfig
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
        });
    }

    private async Task<object?> GetCameraResolutions(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<GetCameraResolutionsParams>(msg);
        // A network camera has no resolution list: the stream's own size is used.
        if (!string.IsNullOrEmpty(p.Id) && _robot.CameraManager.GetCamera(p.Id) is { IsNetwork: true })
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
}
