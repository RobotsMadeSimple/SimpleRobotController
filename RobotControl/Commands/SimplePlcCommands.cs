using System.Text.Json;
using Controller.RobotControl.SimplePlc;

namespace Controller.RobotControl.Commands;

/// <summary>
/// User-added SimplePLC heads: discover on the network, CRUD devices, connect/disconnect, and the
/// head actions (identify, rescan, rename). State is returned as a JSON string under
/// <c>simplePlcDevices</c>, mirroring <see cref="IoCommands"/>'s <c>ioDevices</c> and the serial
/// devices. The module bits themselves ride the normal IO path (GetIO / SetIoOutput / <c>$io</c>).
/// </summary>
internal sealed class SimplePlcCommands
{
    private readonly RobotController _robot;

    public SimplePlcCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetSimplePlcDevices",          GetDevices);
        d.Add("DiscoverSimplePlc",            Discover);
        d.Add("AddSimplePlcDevice",           Add);
        d.Add("RemoveSimplePlcDevice",        Remove);
        d.Add("SetSimplePlcDeviceConfig",     SetConfig);
        d.Add("SetSimplePlcDeviceConnected",  SetConnected);
        d.Add("IdentifySimplePlc",            Identify);
        d.Add("RescanSimplePlc",              Rescan);
        d.Add("SetSimplePlcHeadName",         SetHeadName);
    }

    private string DevicesJson() =>
        JsonSerializer.Serialize(_robot.SimplePlcManager.GetAllStates(), CommandJson.CamelCaseWithEnums);

    private object? GetDevices(CommandMessage msg) => new { simplePlcDevices = DevicesJson() };

    private object? Discover(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<DiscoverSimplePlcParams>(msg);
        var heads = _robot.SimplePlcManager.Discover(p.TimeoutMs is > 0 and <= 5000 ? p.TimeoutMs.Value : 1000);
        return new { heads = JsonSerializer.Serialize(heads, CommandJson.CamelCaseWithEnums) };
    }

    private object? Add(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<AddSimplePlcDeviceParams>(msg);
        var cfg = new SimplePlcDeviceConfig
        {
            Name        = p.Name,
            Host        = p.Host,
            Port        = p.Port > 0 ? p.Port : SimplePlcProtocol.DefaultTcpPort,
            Mac         = p.Mac ?? "",
            AutoConnect = p.AutoConnect ?? true,
            WatchdogMs  = p.WatchdogMs ?? 500,
            FilterMs    = p.FilterMs ?? 3,
            OnTimeout   = p.OnTimeout is "hold" or "off" ? p.OnTimeout : "hold",
        };
        var id = _robot.SimplePlcManager.Add(cfg);
        // Return the new id as "deviceId", NOT "id": the WS ack envelope keys on the message id and
        // merges the handler payload over it, so a payload "id" would clobber it (see SerialCommands).
        return new { deviceId = id, simplePlcDevices = DevicesJson() };
    }

    private object? Remove(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SimplePlcDeviceIdParams>(msg);
        if (!_robot.SimplePlcManager.Remove(p.Id))
            return new { ok = false, error = $"SimplePLC device '{p.Id}' not found." };
        return new { ok = true, simplePlcDevices = DevicesJson() };
    }

    private object? SetConfig(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetSimplePlcDeviceConfigParams>(msg);
        if (!_robot.SimplePlcManager.SetConfig(p.Id, p.Name, p.Host, p.Port, p.Mac, p.AutoConnect, p.WatchdogMs, p.FilterMs, p.OnTimeout))
            return new { ok = false, error = $"SimplePLC device '{p.Id}' not found." };
        return new { ok = true, simplePlcDevices = DevicesJson() };
    }

    private object? SetConnected(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetSimplePlcDeviceConnectedParams>(msg);
        if (!_robot.SimplePlcManager.SetConnected(p.Id, p.Connected))
            return new { ok = false, error = $"SimplePLC device '{p.Id}' not found." };
        return new { ok = true, simplePlcDevices = DevicesJson() };
    }

    private object? Identify(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<IdentifySimplePlcParams>(msg);
        bool ok = _robot.SimplePlcManager.Identify(p.Id, p.Slot, p.Seconds is > 0 and <= 60 ? p.Seconds : 5);
        return ok ? new { ok = true } : new { ok = false, error = $"SimplePLC device '{p.Id}' not connected." };
    }

    private object? Rescan(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SimplePlcDeviceIdParams>(msg);
        bool ok = _robot.SimplePlcManager.Rescan(p.Id);
        return ok ? new { ok = true } : new { ok = false, error = $"SimplePLC device '{p.Id}' not connected." };
    }

    private object? SetHeadName(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetSimplePlcHeadNameParams>(msg);
        bool ok = _robot.SimplePlcManager.SetHeadName(p.Id, p.Name);
        return ok ? new { ok = true, simplePlcDevices = DevicesJson() } : new { ok = false, error = $"SimplePLC device '{p.Id}' not connected." };
    }
}
