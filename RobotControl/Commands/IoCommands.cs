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
        d.Add("ConfigureLoadCell", ConfigureLoadCell);
        d.Add("TareLoadCell",      TareLoadCell);
        d.Add("CalibrateLoadCell", CalibrateLoadCell);
        d.Add("SetLoadCellUnit",   SetLoadCellUnit);
        d.Add("SetRelay",         SetRelay);
        d.Add("RenameRelay",      RenameRelay);
        d.Add("GetRelayState",    GetRelayState);

        // IO value history + recordings
        d.Add("GetIoHistoryConfig",    GetIoHistoryConfig);
        d.Add("SetIoHistoryRetention", SetIoHistoryRetention);
        d.Add("GetIoHistoryWindow",    GetIoHistoryWindow);
        d.Add("StartIoRecording",      StartIoRecording);
        d.Add("StopIoRecording",       StopIoRecording);
        d.Add("ListIoRecordings",      ListIoRecordings);
        d.Add("GetIoRecordingWindow",  GetIoRecordingWindow);
        d.Add("SaveIoRecording",       SaveIoRecording);
        d.Add("DeleteIoRecording",     DeleteIoRecording);
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
        if (!_robot.NanoManager.RenamePin(p.NanoId, p.Pin, p.Name))
            throw new InvalidOperationException(
                $"Nano is not connected (or pin {p.Pin} is not configured) — the name was not saved.");
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
        // SetPinType returns false when the device isn't live — surface that
        // instead of ACKing a save that silently changed nothing.
        if (!_robot.NanoManager.SetPinType(p.NanoId, p.Pin, type, p.PixelCount))
            throw new InvalidOperationException(
                $"Nano is not connected — pin {p.Pin} was not saved.");
    }

    private void ConfigureLoadCell(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ConfigureLoadCellParams>(msg);
        if (!_robot.NanoManager.SetLoadCellConfig(p.NanoId, p.DtPin, p.SckPin, p.Unit))
            throw new InvalidOperationException($"Nano '{p.NanoId}' not found — load cell was not saved.");
    }

    private object? TareLoadCell(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<LoadCellPinParams>(msg);
        var offset = _robot.NanoManager.TareLoadCell(p.NanoId, p.Pin);
        if (offset == null)
            throw new InvalidOperationException($"Load cell on pin {p.Pin} not found or not connected.");
        return new { tareOffset = offset.Value };
    }

    private object? CalibrateLoadCell(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<CalibrateLoadCellParams>(msg);
        var cpg = _robot.NanoManager.CalibrateLoadCell(p.NanoId, p.Pin, p.KnownGrams);
        if (cpg == null)
            throw new InvalidOperationException(
                $"Load cell on pin {p.Pin} not found, not connected, or the known weight was zero.");
        return new { countsPerGram = cpg.Value };
    }

    private void SetLoadCellUnit(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetLoadCellUnitParams>(msg);
        if (!_robot.NanoManager.SetLoadCellUnit(p.NanoId, p.Pin, p.Unit))
            throw new InvalidOperationException($"Load cell on pin {p.Pin} not found.");
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

    // ── IO value history + recordings ──────────────────────────────────────────

    private object? GetIoHistoryConfig(CommandMessage msg) =>
        new { maxRetentionMs = _robot.IoHistory.MaxRetentionMs };

    private object? SetIoHistoryRetention(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetIoHistoryRetentionParams>(msg);
        return new { maxRetentionMs = _robot.IoHistory.SetRetention(p.MaxRetentionMs) };
    }

    private object? GetIoHistoryWindow(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<IoHistoryWindowParams>(msg);
        var w = _robot.IoHistory.GetWindow(p.ChannelKey, p.FromUnixMs, p.ToUnixMs, p.MaxPoints ?? 1000);
        return new { window = w };
    }

    private object? StartIoRecording(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<StartIoRecordingParams>(msg);
        if (string.IsNullOrWhiteSpace(p.ChannelKey))
            throw new InvalidOperationException("A channelKey is required to start a recording.");
        return new { recording = _robot.IoHistory.StartRecording(p.ChannelKey, p.Name) };
    }

    private object? StopIoRecording(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<IoRecordingIdParams>(msg);
        var rec = _robot.IoHistory.StopRecording(p.Id)
                  ?? throw new InvalidOperationException($"Recording '{p.Id}' not found.");
        return new { recording = rec };
    }

    private object? ListIoRecordings(CommandMessage msg) =>
        new { recordings = _robot.IoHistory.ListRecordings() };

    private object? GetIoRecordingWindow(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<IoRecordingWindowParams>(msg);
        var w = _robot.IoHistory.GetRecordingWindow(p.Id, p.FromUnixMs, p.ToUnixMs, p.MaxPoints ?? 2000)
                ?? throw new InvalidOperationException($"Recording '{p.Id}' not found.");
        var meta = _robot.IoHistory.GetRecordingMeta(p.Id);
        return new { window = w, recording = meta };
    }

    private object? SaveIoRecording(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<IoRecordingIdParams>(msg);
        var rec = _robot.IoHistory.SaveRecording(p.Id)
                  ?? throw new InvalidOperationException($"Recording '{p.Id}' not found.");
        return new { recording = rec };
    }

    private void DeleteIoRecording(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<IoRecordingIdParams>(msg);
        if (!_robot.IoHistory.DeleteRecording(p.Id))
            throw new InvalidOperationException($"Recording '{p.Id}' not found.");
    }
}
