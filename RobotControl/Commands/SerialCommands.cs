using System.Text.Json;
using Controller.RobotControl.Serial;

namespace Controller.RobotControl.Commands;

/// <summary>
/// User-defined generic serial devices: list ports, CRUD devices, send named/raw commands and
/// manually connect/disconnect. State is returned as a JSON string under <c>serialDevices</c>,
/// mirroring <see cref="IoCommands"/>'s <c>ioDevices</c> and <see cref="DeviceCommands"/>.
/// </summary>
internal sealed class SerialCommands
{
    private readonly RobotController _robot;

    public SerialCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("ListSerialPorts",          ListSerialPorts);
        d.Add("GetSerialDevices",         GetSerialDevices);
        d.Add("AddSerialDevice",          AddSerialDevice);
        d.Add("RemoveSerialDevice",       RemoveSerialDevice);
        d.Add("SetSerialDeviceConfig",    SetSerialDeviceConfig);
        d.Add("SendSerialCommand",        SendSerialCommand);
        d.Add("SetSerialDeviceConnected", SetSerialDeviceConnected);
    }

    private string DevicesJson() =>
        JsonSerializer.Serialize(_robot.SerialManager.GetAllStates(), CommandJson.CamelCaseWithEnums);

    private object? ListSerialPorts(CommandMessage msg)
    {
        var ports   = SerialPortEnumerator.List();
        var configs = _robot.SerialManager.Configs();

        foreach (var p in ports)
        {
            var owner = SerialPortRegistry.OwnerOf(p.PortName);
            if (owner != null)
            {
                p.InUse  = true;
                p.UsedBy = NameForOwner(owner);
            }
            // A serial device configured to this port counts as in-use even before it connects.
            var sd = configs.FirstOrDefault(c => string.Equals(c.PortName, p.PortName, StringComparison.OrdinalIgnoreCase));
            if (sd != null)
            {
                p.InUse = true;
                if (string.IsNullOrEmpty(p.UsedBy)) p.UsedBy = sd.Name;
            }
        }

        return new { ports };
    }

    /// <summary>Best-effort friendly name for the device that owns a claimed port.</summary>
    private string NameForOwner(string ownerId)
    {
        var nano = _robot.NanoManager.GetDevice(ownerId);
        if (nano != null) return nano.Name;
        var aux = _robot.AuxAxisManager.GetDevice(ownerId);
        if (aux != null) return aux.Name;
        var serial = _robot.SerialManager.GetDevice(ownerId);
        if (serial != null) return serial.Name;
        return ownerId;
    }

    private object? GetSerialDevices(CommandMessage msg) =>
        new { serialDevices = DevicesJson() };

    private object? AddSerialDevice(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<AddSerialDeviceParams>(msg);
        var cfg = new SerialDeviceConfig
        {
            Name            = p.Name,
            PortName        = p.PortName,
            BaudRate        = p.BaudRate,
            DataBits        = p.DataBits ?? 8,
            Parity          = p.Parity ?? System.IO.Ports.Parity.None,
            StopBits        = p.StopBits ?? System.IO.Ports.StopBits.One,
            WriteLineEnding = p.WriteLineEnding ?? "lf",
        };
        var id = _robot.SerialManager.Add(cfg);
        // NOTE: the key must NOT be "id" — the WS ack envelope keys on the message id and merges the
        // handler payload over it (WebSocketServer), so a payload "id" would clobber the message id and
        // no client could match the ack. Return it as "deviceId".
        return new { deviceId = id, serialDevices = DevicesJson() };
    }

    private object? RemoveSerialDevice(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SerialDeviceIdParams>(msg);
        if (!_robot.SerialManager.Remove(p.Id))
            return new { ok = false, error = $"Serial device '{p.Id}' not found." };
        return new { ok = true, serialDevices = DevicesJson() };
    }

    private object? SetSerialDeviceConfig(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetSerialDeviceConfigParams>(msg);
        if (!_robot.SerialManager.SetConfig(p.Id, p.Name, p.PortName, p.BaudRate, p.DataBits,
                p.Parity, p.StopBits, p.WriteLineEnding, p.AutoConnect, p.Commands))
            return new { ok = false, error = $"Serial device '{p.Id}' not found." };
        return new { ok = true, serialDevices = DevicesJson() };
    }

    private object? SendSerialCommand(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SendSerialCommandParams>(msg);
        var r = _robot.SerialManager.Send(p.Id, p.CommandId, p.Text, p.ExpectReply, p.ReplyTimeoutMs);
        return r.Ok
            ? new { ok = true, reply = r.Reply, lines = r.Lines }
            : (object)new { ok = false, error = r.Error };
    }

    private object? SetSerialDeviceConnected(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetSerialDeviceConnectedParams>(msg);
        if (!_robot.SerialManager.SetConnected(p.Id, p.Connected))
            return new { ok = false, error = $"Serial device '{p.Id}' not found." };
        return new { ok = true, serialDevices = DevicesJson() };
    }
}
