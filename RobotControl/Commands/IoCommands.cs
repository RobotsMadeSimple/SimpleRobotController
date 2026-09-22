using System.Text.Json;
using Controller.RobotControl.Nano;
using Controller.RobotControl.UsbRelay;

namespace Controller.RobotControl.Commands;

/// <summary>Digital IO: STB4100 outputs, Nano IO cards and the USB relay board.</summary>
internal sealed class IoCommands
{
    private readonly RobotController _robot;

    public IoCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("SetSTBOutput",     SetSTBOutput);
        d.Add("GetIO",            GetIO);
        d.Add("SetNanoOutput",    SetNanoOutput);
        d.Add("SetNeoPixel",      SetNeoPixel);
        d.Add("RenameNanoPin",    RenameNanoPin);
        d.Add("ConfigureNanoPin", ConfigureNanoPin);
        d.Add("SetRelay",         SetRelay);
        d.Add("RenameRelay",      RenameRelay);
        d.Add("GetRelayState",    GetRelayState);
    }

    private UsbRelayState RelayState()
    {
        var relay = _robot.RelayManager;
        return new UsbRelayState
        {
            Connected = relay.IsConnected,
            Serial    = relay.GetSerial(),
            Relays    = relay.GetRelayStates(),
            Names     = relay.GetRelayNames(),
        };
    }

    private void SetSTBOutput(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetNanoOutputParams>(msg); // reuse same params shape
        _robot.stb.SetOutput(p.Pin, p.Value);
    }

    private object? GetIO(CommandMessage msg)
    {
        var states   = _robot.NanoManager.GetAllStates();
        var nanoJson = JsonSerializer.Serialize(states, CommandJson.CamelCaseWithEnums);
        var relayJson = JsonSerializer.Serialize(RelayState(), CommandJson.CamelCase);
        return new { nanos = nanoJson, relay = relayJson };
    }

    private void SetNanoOutput(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetNanoOutputParams>(msg);
        _robot.NanoManager.SetOutput(p.NanoId, p.Pin, p.Value);
    }

    private void SetNeoPixel(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetNeoPixelParams>(msg);
        var colors = p.Colors
            .Select(c => new NeoPixelColor(c.R, c.G, c.B))
            .ToArray();
        _robot.NanoManager.SetNeoPixel(p.NanoId, p.Pin, colors);
    }

    private void RenameNanoPin(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<RenameNanoPinParams>(msg);
        _robot.NanoManager.RenamePin(p.NanoId, p.Pin, p.Name);
    }

    private void ConfigureNanoPin(CommandMessage msg)
    {
        var p    = CommandJson.LoadParams<ConfigureNanoPinParams>(msg);
        var type = p.Type switch
        {
            "Output"       => PinType.Output,
            "Neopixel"     => PinType.Neopixel,
            "Unconfigured" => PinType.Unconfigured,
            _              => PinType.Input,
        };
        _robot.NanoManager.SetPinType(p.NanoId, p.Pin, type, p.PixelCount);
    }

    private void SetRelay(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetRelayParams>(msg);
        _robot.RelayManager.SetRelay(p.Relay, p.Value);
    }

    private void RenameRelay(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<RenameRelayParams>(msg);
        _robot.RelayManager.RenameRelay(p.Relay, p.Name);
    }

    private object? GetRelayState(CommandMessage msg) => RelayState();
}
