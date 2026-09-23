using System.Text.Json;
using Controller.RobotControl.Camera;

namespace Controller.RobotControl.Commands;

/// <summary>USB camera configuration.</summary>
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
        _robot.CameraManager.UpdateCamera(p.Id, new CameraConfig
        {
            Id          = p.Id,
            Name        = p.Name,
            DeviceIndex = p.DeviceIndex,
            Enabled     = p.Enabled,
            Width       = p.Width,
            Height      = p.Height,
            TargetFps   = p.TargetFps,
        });
    }

    private async Task<object?> GetCameraResolutions(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<GetCameraResolutionsParams>(msg);
        var deviceIndex = p.DeviceIndex;
        var resolutions = await Task.Run(() => _robot.CameraManager.ProbeResolutionsForIndex(deviceIndex));
        var json = JsonSerializer.Serialize(resolutions, CommandJson.CamelCase);
        return new { resolutions = json };
    }
}
