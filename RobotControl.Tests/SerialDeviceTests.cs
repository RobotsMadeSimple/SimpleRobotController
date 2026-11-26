using System.IO.Ports;
using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.Serial;

namespace RobotControl.Tests;

/// <summary>
/// Generic serial device: config round-trip (serial_devices.json), the line-ending/framing map,
/// the GenericSerialManager CRUD with no hardware, and the WebSocket commands through the dispatcher.
/// </summary>
public class SerialDeviceConfigJsonTests
{
    // The exact serial_devices.json shape from the wire contract §1.
    private const string Sample = """
        {
          "serialDevices": [
            {
              "id": "SERIAL_001",
              "name": "My Pump",
              "portName": "COM5",
              "baudRate": 9600,
              "dataBits": 8,
              "parity": "None",
              "stopBits": "One",
              "writeLineEnding": "lf",
              "autoConnect": true,
              "commands": [
                { "id": "cmd_a1", "name": "Open", "payload": "G1 OPEN", "expectReply": false, "replyTimeoutMs": 1000 }
              ]
            }
          ]
        }
        """;

    [Fact]
    public void ConfigRoundTripsWithSharedFileOptions()
    {
        var cfg = JsonSerializer.Deserialize<SerialDevicesConfig>(Sample, JsonDefaults.File)!;
        var d = cfg.SerialDevices.Single();
        Assert.Equal("SERIAL_001", d.Id);
        Assert.Equal("My Pump", d.Name);
        Assert.Equal("COM5", d.PortName);
        Assert.Equal(9600, d.BaudRate);
        Assert.Equal(8, d.DataBits);
        Assert.Equal(Parity.None, d.Parity);
        Assert.Equal(StopBits.One, d.StopBits);
        Assert.Equal("lf", d.WriteLineEnding);
        Assert.True(d.AutoConnect);
        Assert.Equal("cmd_a1", d.Commands.Single().Id);
        Assert.Equal("G1 OPEN", d.Commands.Single().Payload);
        Assert.Equal(1000, d.Commands.Single().ReplyTimeoutMs);

        // Re-serializes to the contract's camelCase field names and string enums.
        var json = JsonSerializer.Serialize(cfg, JsonDefaults.File);
        Assert.Contains("\"serialDevices\"", json);
        Assert.Contains("\"portName\": \"COM5\"", json);
        Assert.Contains("\"parity\": \"None\"", json);
        Assert.Contains("\"stopBits\": \"One\"", json);
        Assert.Contains("\"writeLineEnding\": \"lf\"", json);
    }

    [Theory]
    [InlineData("lf",   "\n")]
    [InlineData("crlf", "\r\n")]
    [InlineData("cr",   "\r")]
    [InlineData("none", "")]
    [InlineData(null,   "\n")]   // default
    [InlineData("LF",   "\n")]   // case-insensitive
    public void LineEndingMapsCorrectly(string? writeLineEnding, string expected)
    {
        Assert.Equal(expected, SerialDeviceConfig.LineEnding(writeLineEnding));
    }
}

public sealed class GenericSerialManagerTests : IDisposable
{
    private readonly string _path = Path.Combine(Path.GetTempPath(), $"serialtest_{Guid.NewGuid():N}.json");

    public void Dispose() { try { File.Delete(_path); } catch { } }

    [Fact]
    public void AddGetSend_NoHardware()
    {
        var mgr = new GenericSerialManager(_path);   // not Start()ed — no port is opened

        var id = mgr.Add(new SerialDeviceConfig { Name = "Pump", PortName = "COM42", BaudRate = 19200 });
        Assert.Equal("SERIAL_001", id);

        var states = mgr.GetAllStates();
        var s = Assert.Single(states);
        Assert.Equal(id, s.Id);
        Assert.Equal("Pump", s.Name);
        Assert.Equal("COM42", s.PortName);
        Assert.Equal(19200, s.BaudRate);
        Assert.False(s.Connected);           // never started

        // Send on a disconnected device degrades gracefully (no throw, Ok=false).
        var r = mgr.Send(id, null, "HELLO", expectReply: false, replyTimeoutMs: null);
        Assert.False(r.Ok);
        Assert.Contains("not connected", r.Error);

        // Config update (rename + add a saved command) persists and reloads.
        Assert.True(mgr.SetConfig(id, "Pump 2", null, null, null, null, null, null, null,
            new List<SerialCommandDef> { new() { Id = "c1", Name = "Open", Payload = "OPEN" } }));
        Assert.Equal("Pump 2", mgr.GetAllStates().Single().Name);

        var reloaded = new GenericSerialManager(_path);
        var rs = reloaded.GetAllStates().Single();
        Assert.Equal("Pump 2", rs.Name);
        Assert.Equal("Open", rs.Commands.Single().Name);

        Assert.True(mgr.Remove(id));
        Assert.Empty(mgr.GetAllStates());
    }

    [Fact]
    public void Resolve_PrefersIdThenName()
    {
        var mgr = new GenericSerialManager(_path);
        var id = mgr.Add(new SerialDeviceConfig { Name = "Scanner", PortName = "COM9" });
        Assert.NotNull(mgr.Resolve(id, null));
        Assert.NotNull(mgr.Resolve(null, "Scanner"));
        Assert.Null(mgr.Resolve("NOPE", "also-nope"));
    }
}

[Collection("RobotController")]
public sealed class SerialCommandsTests : IDisposable
{
    private readonly RobotController _robot = new();   // ctor only — no threads/devices started
    private readonly CommandDispatcher _d = new();
    private readonly List<string> _created = new();

    public SerialCommandsTests() => new SerialCommands(_robot).Register(_d);

    public void Dispose()
    {
        foreach (var id in _created)
            try { Send("RemoveSerialDevice", new { id }); } catch { }
    }

    private JsonElement Send(string command, object? parameters = null)
    {
        Assert.True(_d.TryGet(command, out var handler), command);
        var msg = new CommandMessage
        {
            Type = "Command", Id = "t", Command = command,
            Params = parameters == null ? null : JsonSerializer.SerializeToElement(parameters),
        };
        var result = handler(msg).GetAwaiter().GetResult();
        return result == null ? JsonSerializer.SerializeToElement(new { }) : JsonSerializer.SerializeToElement(result);
    }

    [Fact]
    public void ListSerialPorts_Returns_A_Ports_Array()
    {
        var ports = Send("ListSerialPorts").GetProperty("ports");
        Assert.Equal(JsonValueKind.Array, ports.ValueKind);   // may be empty on a CI box with no ports
    }

    [Fact]
    public void Add_Get_Send_Remove_Through_Commands()
    {
        var id = Send("AddSerialDevice", new { name = "Pump", portName = "COM77", baudRate = 9600 })
            .GetProperty("deviceId").GetString()!;
        _created.Add(id);
        Assert.StartsWith("SERIAL_", id);

        // GetSerialDevices returns the state list as a JSON string (like GetIO's ioDevices).
        var devicesJson = Send("GetSerialDevices").GetProperty("serialDevices").GetString()!;
        using var devices = JsonDocument.Parse(devicesJson);
        Assert.Contains(devices.RootElement.EnumerateArray(),
            e => e.GetProperty("id").GetString() == id && e.GetProperty("name").GetString() == "Pump");

        // Send to a device that never connected → ok=false, not a throw.
        var send = Send("SendSerialCommand", new { id, text = "PING", expectReply = false });
        Assert.False(send.GetProperty("ok").GetBoolean());

        // Save a named command via SetSerialDeviceConfig.
        var set = Send("SetSerialDeviceConfig", new
        {
            id,
            commands = new[] { new { id = "c1", name = "Ping", payload = "PING", expectReply = true, replyTimeoutMs = 500 } },
        });
        Assert.True(set.GetProperty("ok").GetBoolean());

        var removed = Send("RemoveSerialDevice", new { id });
        Assert.True(removed.GetProperty("ok").GetBoolean());
        _created.Remove(id);
    }

    [Fact]
    public void SendSerialCommand_UnknownDevice_IsGraceful()
    {
        var send = Send("SendSerialCommand", new { id = "SERIAL_999", text = "X" });
        Assert.False(send.GetProperty("ok").GetBoolean());
        Assert.False(string.IsNullOrEmpty(send.GetProperty("error").GetString()));
    }
}
