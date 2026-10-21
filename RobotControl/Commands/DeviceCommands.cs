using System.Text.Json;

namespace Controller.RobotControl.Commands;

/// <summary>
/// Phone / client data devices (docs/plugins.md is unrelated — this is native). Add/remove/configure
/// a device; the phone then streams channel values in over <c>/devices/{id}/ingest</c>, which
/// surface as <c>$device.&lt;id&gt;.&lt;channel&gt;</c> in expressions. State is returned as a
/// JSON string under <c>devices</c>, mirroring <see cref="CameraCommands"/>.
/// </summary>
internal sealed class DeviceCommands
{
    private readonly RobotController _robot;

    public DeviceCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetDevices",      GetDevices);
        d.Add("AddDevice",       AddDevice);
        d.Add("RemoveDevice",    RemoveDevice);
        d.Add("SetDeviceConfig", SetDeviceConfig);
    }

    private object? GetDevices(CommandMessage msg) =>
        new { devices = JsonSerializer.Serialize(_robot.DeviceManager.GetState(), CommandJson.CamelCase) };

    private object? AddDevice(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<AddDeviceParams>(msg);
        var id = _robot.DeviceManager.AddDevice(new Devices.DeviceConfig { Name = p.Name, Enabled = p.Enabled });
        return new { id, devices = JsonSerializer.Serialize(_robot.DeviceManager.GetState(), CommandJson.CamelCase) };
    }

    private object? RemoveDevice(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<RemoveDeviceParams>(msg);
        if (string.IsNullOrEmpty(p.Id)) return new { ok = false, error = "badParams", message = "id is required" };
        return _robot.DeviceManager.RemoveDevice(p.Id)
            ? new { devices = JsonSerializer.Serialize(_robot.DeviceManager.GetState(), CommandJson.CamelCase) }
            : (object)new { ok = false, error = "unknownDevice", message = $"No device '{p.Id}'" };
    }

    private object? SetDeviceConfig(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetDeviceConfigParams>(msg);
        if (string.IsNullOrEmpty(p.Id)) return new { ok = false, error = "badParams", message = "id is required" };
        return _robot.DeviceManager.UpdateDevice(p.Id, p.Name, p.Enabled, p.ProviderConfig)
            ? new { devices = JsonSerializer.Serialize(_robot.DeviceManager.GetState(), CommandJson.CamelCase) }
            : (object)new { ok = false, error = "unknownDevice", message = $"No device '{p.Id}'" };
    }
}
