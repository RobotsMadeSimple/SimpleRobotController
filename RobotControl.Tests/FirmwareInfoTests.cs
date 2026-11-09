using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.AuxAxis;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Io;
using Controller.RobotControl.Serial;

namespace RobotControl.Tests;

/// <summary>
/// The "ID?" handshake parser (legacy "ID:&lt;id&gt;" and 1.1.0+ "ID:&lt;id&gt;;fw=…;caps=…"),
/// the permissive-when-unknown capability rule, and the refusal message.
/// </summary>
public sealed class FirmwareInfoTests
{
    private const string Nano = "ROBOT_NANO_001";

    [Fact]
    public void Legacy_reply_is_accepted_as_unknown_firmware()
    {
        Assert.True(FirmwareInfo.TryParseHandshake("ID:ROBOT_NANO_001", Nano, out var fw));
        Assert.False(fw.IsKnown);
        Assert.Equal("", fw.Version);
        Assert.Null(fw.Capabilities);
        Assert.Null(fw.CapabilityArray());
    }

    [Fact]
    public void New_reply_parses_version_and_capabilities()
    {
        Assert.True(FirmwareInfo.TryParseHandshake("ID:ROBOT_NANO_001;fw=1.1.0;caps=io,neopixel,loadcell", Nano, out var fw));
        Assert.True(fw.IsKnown);
        Assert.Equal("1.1.0", fw.Version);
        Assert.Equal(new[] { "io", "loadcell", "neopixel" }, fw.CapabilityArray());
    }

    [Fact]
    public void Uno_reply_parses()
    {
        Assert.True(FirmwareInfo.TryParseHandshake("ID:AUX_STEPPER_001;fw=1.1.0;caps=stepper,io", "AUX_STEPPER_001", out var fw));
        Assert.True(fw.Has("stepper"));
        Assert.True(fw.Has("io"));
        Assert.False(fw.Has("loadcell"));
    }

    [Fact]
    public void Unknown_fields_odd_spacing_and_case_are_tolerated()
    {
        var line = "  ID:ROBOT_NANO_001 ; FW = 1.2.3 ; build=abc ; CAPS = IO , NeoPixel ,, ; flag \r\n";
        Assert.True(FirmwareInfo.TryParseHandshake(line, Nano, out var fw));
        Assert.Equal("1.2.3", fw.Version);
        Assert.True(fw.IsKnown);
        Assert.Equal(new[] { "io", "neopixel" }, fw.CapabilityArray());
        Assert.True(fw.Has("neopixel"));
        Assert.True(fw.Has("NEOPIXEL"));   // capability lookup is case-insensitive
        Assert.False(fw.Has("loadcell"));
    }

    [Theory]
    [InlineData("ID:ROBOT_NANO_002")]
    [InlineData("ID:ROBOT_NANO_002;fw=1.1.0;caps=io")]
    [InlineData("ID:")]
    [InlineData("ID:robot_nano_001")]          // ids are exact
    [InlineData("OK")]
    [InlineData("RDY")]
    [InlineData("")]
    [InlineData(null)]
    public void Wrong_id_or_non_id_lines_are_rejected(string? line)
    {
        Assert.False(FirmwareInfo.TryParseHandshake(line, Nano, out var fw));
        Assert.Same(FirmwareInfo.Unknown, fw);
    }

    [Fact]
    public void Version_without_caps_stays_permissive()
    {
        Assert.True(FirmwareInfo.TryParseHandshake("ID:ROBOT_NANO_001;fw=1.1.0", Nano, out var fw));
        Assert.Equal("1.1.0", fw.Version);
        Assert.False(fw.IsKnown);
        Assert.True(fw.Has("loadcell"));
    }

    [Fact]
    public void Has_is_permissive_when_unknown_and_strict_when_known()
    {
        Assert.True(FirmwareInfo.Unknown.Has("loadcell"));
        Assert.True(FirmwareInfo.Unknown.Has("anything"));

        var known = new FirmwareInfo("1.1.0", new[] { "io" });
        Assert.True(known.Has("io"));
        Assert.False(known.Has("loadcell"));

        var empty = new FirmwareInfo("1.1.0", Array.Empty<string>());
        Assert.True(empty.IsKnown);
        Assert.False(empty.Has("io"));
    }

    [Fact]
    public void Describe_reads_like_the_connect_log_line()
    {
        Assert.Equal("firmware unknown (legacy, pre-1.1.0)", FirmwareInfo.Unknown.Describe());
        Assert.Equal("firmware 1.1.0 (io, loadcell, neopixel)",
            new FirmwareInfo("1.1.0", new[] { "io", "neopixel", "loadcell" }).Describe());
    }

    [Fact]
    public void Gate_refuses_only_a_known_missing_capability()
    {
        var old = new FirmwareInfo("1.0.0", new[] { "io", "neopixel" });
        var ex = Assert.Throws<InvalidOperationException>(() =>
            FirmwareGate.Require(old, Nano, "loadcell", "Load cell", "NanoEdgeDevice"));
        Assert.Equal("Load cell not supported by ROBOT_NANO_001 firmware 1.0.0 — flash NanoEdgeDevice 1.1.0 or newer", ex.Message);

        // Present → silent. Unknown (legacy) → silent.
        FirmwareGate.Require(old, Nano, "neopixel", "NeoPixel", "NanoEdgeDevice");
        FirmwareGate.Require(FirmwareInfo.Unknown, Nano, "loadcell", "Load cell", "NanoEdgeDevice");
    }

    [Fact]
    public void Device_gate_names_the_devices_sketch()
    {
        var nano = new NanoDevice(new IoDeviceConfig { Id = Nano, Name = "Nano" });
        Assert.Same(FirmwareInfo.Unknown, nano.Firmware);
        nano.RequireCapability("loadcell", "Load cell");   // legacy: permissive

        nano.Firmware = new FirmwareInfo("1.0.0", new[] { "io" });
        var ex = Assert.Throws<InvalidOperationException>(() => nano.RequireCapability("loadcell", "Load cell"));
        Assert.Equal("Load cell not supported by ROBOT_NANO_001 firmware 1.0.0 — flash NanoEdgeDevice 1.1.0 or newer", ex.Message);

        using var aux = new AuxAxisDevice(new AuxAxisConfig { Id = "AUX_STEPPER_001", Name = "Aux" });
        aux.Firmware = new FirmwareInfo("1.1.0", new[] { "stepper" });
        var ex2 = Assert.Throws<InvalidOperationException>(() => aux.RequireCapability("io", "Digital IO"));
        Assert.Equal("Digital IO not supported by AUX_STEPPER_001 firmware 1.1.0 — flash AuxStepperDriver 1.1.0 or newer", ex2.Message);
        aux.RequireCapability("stepper", "Stepper motion");
    }

    [Fact]
    public void Io_and_aux_state_payloads_carry_firmware_fields_in_camelCase()
    {
        var io = new IoDeviceState { Id = Nano, FirmwareVersion = "1.1.0", Capabilities = new[] { "io" } };
        var ioJson = JsonSerializer.Serialize(io, CommandJson.CamelCaseWithEnums);
        Assert.Contains("\"firmwareVersion\":\"1.1.0\"", ioJson);
        Assert.Contains("\"capabilities\":[\"io\"]", ioJson);

        var unknown = JsonSerializer.Serialize(new IoDeviceState { Id = Nano }, CommandJson.CamelCaseWithEnums);
        Assert.Contains("\"firmwareVersion\":\"\"", unknown);
        Assert.Contains("\"capabilities\":null", unknown);

        var aux = new AuxAxisState { DeviceId = "AUX_STEPPER_001", FirmwareVersion = "1.1.0", Capabilities = new[] { "io", "stepper" } };
        var auxJson = JsonSerializer.Serialize(aux, CommandJson.CamelCase);
        Assert.Contains("\"firmwareVersion\":\"1.1.0\"", auxJson);
        Assert.Contains("\"capabilities\":[\"io\",\"stepper\"]", auxJson);
        Assert.Contains("\"capabilities\":null", JsonSerializer.Serialize(new AuxAxisState(), CommandJson.CamelCase));
    }
}

/// <summary>
/// Capability gating through the real command handlers (dispatcher + RobotController, no hardware):
/// a Nano whose firmware reports no "loadcell"/"neopixel" capability gets a refusal ack error, and the
/// GetIO payload exposes the firmware fields. Aux devices only exist after AuxAxisManager.Start()
/// (which scans serial ports), so the aux gating is covered by the FirmwareGate/device tests above.
/// </summary>
[Collection("RobotController")]
public sealed class FirmwareGatingCommandTests : IDisposable
{
    private readonly RobotController _robot = new();   // ctor only — no threads/devices started
    private readonly CommandDispatcher _d = new();
    private readonly NanoDevice _nano;

    public FirmwareGatingCommandTests()
    {
        new IoCommands(_robot).Register(_d);
        _nano = _robot.NanoManager.GetDevices().First();   // io_devices.json default: ROBOT_NANO_001
    }

    public void Dispose() => _nano.Firmware = FirmwareInfo.Unknown;

    private object? Send(string command, object parameters)
    {
        Assert.True(_d.TryGet(command, out var handler), command);
        var msg = new CommandMessage
        {
            Type = "Command", Id = "t", Command = command,
            Params = JsonSerializer.SerializeToElement(parameters),
        };
        return handler(msg).GetAwaiter().GetResult();
    }

    [Fact]
    public void ConfigureLoadCell_is_refused_on_firmware_without_loadcell()
    {
        _nano.Firmware = new FirmwareInfo("1.0.0", new[] { "io", "neopixel" });
        var ex = Assert.Throws<InvalidOperationException>(() =>
            Send("ConfigureLoadCell", new { deviceId = _nano.Id, dtPin = 10, sckPin = 11, unit = "g" }));
        Assert.Equal($"Load cell not supported by {_nano.Id} firmware 1.0.0 — flash NanoEdgeDevice 1.1.0 or newer", ex.Message);
    }

    [Fact]
    public void ConfigureIoPin_neopixel_is_refused_on_firmware_without_neopixel()
    {
        _nano.Firmware = new FirmwareInfo("1.1.0", new[] { "io", "loadcell" });
        var ex = Assert.Throws<InvalidOperationException>(() =>
            Send("ConfigureIoPin", new { deviceId = _nano.Id, pin = 6, type = "Neopixel", pixelCount = 12 }));
        Assert.Equal($"NeoPixel not supported by {_nano.Id} firmware 1.1.0 — flash NanoEdgeDevice 1.1.0 or newer", ex.Message);

        var ex2 = Assert.Throws<InvalidOperationException>(() =>
            Send("SetNeoPixel", new { deviceId = _nano.Id, pin = 6, colors = new[] { new { r = 1, g = 2, b = 3 } } }));
        Assert.StartsWith("NeoPixel not supported by", ex2.Message);
    }

    [Fact]
    public void SetIoOutput_is_refused_on_firmware_without_io()
    {
        _nano.Firmware = new FirmwareInfo("1.1.0", new[] { "neopixel" });
        var ex = Assert.Throws<InvalidOperationException>(() =>
            Send("SetIoOutput", new { deviceId = _nano.Id, pin = 7, value = true }));
        Assert.Equal($"Digital IO not supported by {_nano.Id} firmware 1.1.0 — flash NanoEdgeDevice 1.1.0 or newer", ex.Message);
    }

    [Fact]
    public void Legacy_firmware_is_not_gated()
    {
        _nano.Firmware = FirmwareInfo.Unknown;
        // Device is not connected, so SetIoOutput is a queued no-op — but it must not be refused.
        Send("SetIoOutput", new { deviceId = _nano.Id, pin = 7, value = false });
    }

    [Fact]
    public void GetIO_exposes_firmware_version_and_capabilities()
    {
        _nano.Firmware = new FirmwareInfo("1.1.0", new[] { "io", "neopixel", "loadcell" });
        var result = Send("GetIO", new { })!;
        var ioDevices = JsonSerializer.SerializeToElement(result).GetProperty("ioDevices").GetString()!;
        var row = JsonSerializer.Deserialize<JsonElement>(ioDevices).EnumerateArray()
            .First(d => d.GetProperty("id").GetString() == _nano.Id);
        Assert.Equal("1.1.0", row.GetProperty("firmwareVersion").GetString());
        Assert.Equal(new[] { "io", "loadcell", "neopixel" },
            row.GetProperty("capabilities").EnumerateArray().Select(c => c.GetString()).ToArray());

        _nano.Firmware = FirmwareInfo.Unknown;
        ioDevices = JsonSerializer.SerializeToElement(Send("GetIO", new { })!).GetProperty("ioDevices").GetString()!;
        row = JsonSerializer.Deserialize<JsonElement>(ioDevices).EnumerateArray()
            .First(d => d.GetProperty("id").GetString() == _nano.Id);
        Assert.Equal("", row.GetProperty("firmwareVersion").GetString());
        Assert.Equal(JsonValueKind.Null, row.GetProperty("capabilities").ValueKind);
    }
}
