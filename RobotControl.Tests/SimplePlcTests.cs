using System.Collections.Concurrent;
using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Io;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.SimplePlc;

namespace RobotControl.Tests;

// ── A fake head: speaks the §4 JSON protocol over an in-process transport, no sockets ───────────

/// <summary>
/// A stand-in SimplePLC head for tests. Acts as the transport factory and discovery, and answers the
/// native protocol (hello/modules/claim/config/read/out/ping/identify/rescan/setname) over an
/// in-memory line channel. Test code can push input events and inspect what the device wrote.
/// </summary>
internal sealed class FakePlcHead : ISimplePlcTransportFactory, ISimplePlcDiscovery
{
    public bool AllowClaim = true;
    public int Watchdog = 500, Filter = 3;
    public string OnTimeout = "hold";
    /// <summary>The only host Connect() accepts; also the address discovery advertises.</summary>
    public string GoodHost = "fake";
    // Optional §4.4/4.5 fields the newer firmware reports.
    public int SupplyMv = 24000;
    public string MacSource = "eeprom";       // or "default" (shared MAC → UI warning)
    public bool Do8FieldPower = true;          // the DO8 slot's 24 V field supply present
    public int Do8TypeId = 2;                  // output-slot type id (2 = DO8, 6 = DO8N sinking twin)
    public string Do8TypeName = "DO8";         // output-slot type name reported by the head
    public readonly List<string> StateFaults = new();   // entries for the read reply's faults array
    public volatile int PingCount;
    public readonly ConcurrentQueue<(int slot, int mask, int value)> OutWrites = new();
    public readonly ConcurrentDictionary<int, int> InValues = new();   // slot → packed input bits
    public readonly ConcurrentDictionary<int, int> OutValues = new();  // slot → packed output bits

    private volatile FakeSession? _session;

    public ISimplePlcTransport Connect(string host, int port, int timeoutMs)
    {
        if (!string.Equals(host, GoodHost, StringComparison.OrdinalIgnoreCase))
            throw new System.IO.IOException($"no route to {host}");
        var s = new FakeSession(this);
        _session = s;
        return s;
    }

    public IReadOnlyList<SimplePlcDiscovered> Discover(int timeoutMs) => new[]
    {
        new SimplePlcDiscovered { Name = "SimplePLC-FAKE", Mac = "02:53:50:AA:BB:CC", Host = GoodHost, Port = SimplePlcProtocol.DefaultTcpPort, Firmware = "0.1.0", Modules = 2 },
    };

    /// <summary>Push an input-change event to the connected client.</summary>
    public void PushInput(int slot, int value)
    {
        InValues[slot] = value;
        _session?.Enqueue(new Dictionary<string, object?> { ["op"] = "in", ["slot"] = slot, ["value"] = value, ["t"] = 1 });
    }

    /// <summary>Push a fault event (field_power carries slot; supply_low carries mv).</summary>
    public void PushFault(string code, int? slot = null, int? mv = null)
    {
        var d = new Dictionary<string, object?> { ["op"] = "fault", ["code"] = code };
        if (slot.HasValue) d["slot"] = slot.Value;
        if (mv.HasValue) d["mv"] = mv.Value;
        _session?.Enqueue(d);
    }

    /// <summary>Clear a previously-pushed fault.</summary>
    public void PushFaultClear(string code, int? slot = null)
    {
        var d = new Dictionary<string, object?> { ["op"] = "fault_clear", ["code"] = code };
        if (slot.HasValue) d["slot"] = slot.Value;
        _session?.Enqueue(d);
    }

    // Handles one request line from the client, enqueues the reply/replies.
    internal void OnRequest(string line, FakeSession session)
    {
        using var doc = JsonDocument.Parse(line);
        var m = doc.RootElement;
        string op = m.GetProperty("op").GetString() ?? "";
        object? id = m.TryGetProperty("id", out var idEl) && idEl.ValueKind == JsonValueKind.Number ? idEl.GetInt64() : null;
        Dictionary<string, object?> R(string rop, params (string k, object? v)[] fields)
        {
            var d = new Dictionary<string, object?> { ["op"] = rop, ["id"] = id };
            foreach (var (k, v) in fields) d[k] = v;
            return d;
        }

        switch (op)
        {
            case "hello":
                session.Enqueue(R("hello", ("name", "SimplePLC-FAKE"), ("mac", "02:53:50:AA:BB:CC"), ("fw", "0.1.0"), ("hw", "HEAD r1"), ("uptime_ms", 1000), ("mac_source", MacSource), ("supply_mv", SupplyMv)));
                break;
            case "modules":
                session.Enqueue(R("modules", ("modules", new object[]
                {
                    new Dictionary<string, object?> { ["slot"] = 1, ["type"] = "DI8", ["type_id"] = 1, ["in"] = 8, ["out"] = 0, ["state"] = "ok" },
                    new Dictionary<string, object?> { ["slot"] = 2, ["type"] = Do8TypeName, ["type_id"] = Do8TypeId, ["in"] = 0, ["out"] = 8, ["state"] = "ok", ["field_power"] = Do8FieldPower },
                })));
                break;
            case "claim":
                session.Enqueue(AllowClaim ? R("ok") : R("error", ("code", "owned"), ("msg", "owned")));
                break;
            case "config":
                if (m.TryGetProperty("watchdog_ms", out var w) && w.ValueKind == JsonValueKind.Number) Watchdog = w.GetInt32();
                if (m.TryGetProperty("filter_ms", out var f) && f.ValueKind == JsonValueKind.Number) Filter = f.GetInt32();
                if (m.TryGetProperty("on_timeout", out var o) && o.ValueKind == JsonValueKind.String) OnTimeout = o.GetString() ?? OnTimeout;
                session.Enqueue(R("config", ("watchdog_ms", Watchdog), ("filter_ms", Filter), ("sync", false), ("on_timeout", OnTimeout)));
                break;
            case "read":
                session.Enqueue(R("state", ("in", DictOf(InValues)), ("out", DictOf(OutValues)), ("faults", StateFaults.ToArray()), ("supply_mv", SupplyMv)));
                break;
            case "out":
            {
                int slot = m.GetProperty("slot").GetInt32();
                int mask = m.TryGetProperty("mask", out var mk) && mk.ValueKind == JsonValueKind.Number ? mk.GetInt32() : ~0;
                int value = m.GetProperty("value").GetInt32();
                OutValues[slot] = (OutValues.TryGetValue(slot, out var cur) ? cur : 0) & ~mask | (value & mask);
                OutWrites.Enqueue((slot, mask, value));
                session.Enqueue(R("ok"));
                break;
            }
            case "ping":    PingCount++; session.Enqueue(R("pong", ("t", 1), ("supply_mv", SupplyMv))); break;   // head keeps voltage live on the pong
            case "subscribe": break;   // no reply, like the real head's default-on subscription
            case "identify":
            case "rescan":
            case "setname": session.Enqueue(R("ok")); break;
            default: session.Enqueue(R("error", ("code", "unknown_op"))); break;
        }
    }

    private static Dictionary<string, object?> DictOf(ConcurrentDictionary<int, int> src)
    {
        var d = new Dictionary<string, object?>();
        foreach (var kv in src) d[kv.Key.ToString()] = kv.Value;
        return d;
    }
}

/// <summary>One in-memory session: client writes are handed to the head; head replies are read back.</summary>
internal sealed class FakeSession : ISimplePlcTransport
{
    private readonly FakePlcHead _head;
    private readonly BlockingCollection<string> _toClient = new();

    public FakeSession(FakePlcHead head) => _head = head;

    internal void Enqueue(Dictionary<string, object?> msg)
    {
        try { _toClient.Add(JsonSerializer.Serialize(msg)); } catch { /* completed */ }
    }

    public string? ReadLine()
    {
        try { return _toClient.Take(); }
        catch { return null; }   // CompleteAdding → stream end
    }

    public void WriteLine(string json) => _head.OnRequest(json, this);

    public void Dispose() { try { _toClient.CompleteAdding(); } catch { } }
}

// ── Tests ───────────────────────────────────────────────────────────────────────

public class SimplePlcConfigJsonTests
{
    private const string Sample = """
        {
          "simplePlcDevices": [
            {
              "id": "PLC_001",
              "name": "Cell 3 IO",
              "host": "192.168.4.50",
              "port": 47811,
              "mac": "02:53:50:1A:2B:3C",
              "autoConnect": true,
              "watchdogMs": 500,
              "filterMs": 3,
              "onTimeout": "hold"
            }
          ]
        }
        """;

    [Fact]
    public void ConfigRoundTripsWithSharedFileOptions()
    {
        var cfg = JsonSerializer.Deserialize<SimplePlcConfig>(Sample, JsonDefaults.File)!;
        var d = cfg.SimplePlcDevices.Single();
        Assert.Equal("PLC_001", d.Id);
        Assert.Equal("Cell 3 IO", d.Name);
        Assert.Equal("192.168.4.50", d.Host);
        Assert.Equal(47811, d.Port);
        Assert.Equal("02:53:50:1A:2B:3C", d.Mac);
        Assert.True(d.AutoConnect);
        Assert.Equal(500, d.WatchdogMs);
        Assert.Equal("hold", d.OnTimeout);

        var json = JsonSerializer.Serialize(cfg, JsonDefaults.File);
        Assert.Contains("\"simplePlcDevices\"", json);
        Assert.Contains("\"host\": \"192.168.4.50\"", json);
        Assert.Contains("\"onTimeout\": \"hold\"", json);
    }
}

public sealed class SimplePlcManagerCrudTests : IDisposable
{
    private readonly string _path = Path.Combine(Path.GetTempPath(), $"plctest_{Guid.NewGuid():N}.json");
    public void Dispose() { try { File.Delete(_path); } catch { } }

    [Fact]
    public void AddSetRemove_NoNetwork()
    {
        var mgr = new SimplePlcManager(_path);   // not Start()ed — no connection attempted

        var id = mgr.Add(new SimplePlcDeviceConfig { Name = "Cell 3", Host = "192.168.4.50", AutoConnect = false });
        Assert.Equal("PLC_001", id);

        var s = Assert.Single(mgr.GetAllStates());
        Assert.Equal(id, s.Id);
        Assert.Equal("Cell 3", s.Name);
        Assert.Equal("192.168.4.50", s.Host);
        Assert.False(s.Connected);

        Assert.True(mgr.SetConfig(id, "Cell 3B", null, 47812, null, null, 750, null, "off"));
        var reloaded = new SimplePlcManager(_path).GetAllStates().Single();
        Assert.Equal("Cell 3B", reloaded.Name);
        Assert.Equal(47812, reloaded.Port);
        Assert.Equal(750, reloaded.WatchdogMs);
        Assert.Equal("off", reloaded.OnTimeout);

        Assert.True(mgr.Remove(id));
        Assert.Empty(mgr.GetAllStates());
    }

    [Fact]
    public void SecondId_Increments()
    {
        var mgr = new SimplePlcManager(_path);
        Assert.Equal("PLC_001", mgr.Add(new SimplePlcDeviceConfig { Name = "a", AutoConnect = false }));
        Assert.Equal("PLC_002", mgr.Add(new SimplePlcDeviceConfig { Name = "b", AutoConnect = false }));
    }
}

public sealed class SimplePlcDeviceLifecycleTests : IDisposable
{
    private readonly string _path = Path.Combine(Path.GetTempPath(), $"plclife_{Guid.NewGuid():N}.json");
    private SimplePlcManager? _mgr;
    public void Dispose() { try { _mgr?.Stop(); } catch { } try { File.Delete(_path); } catch { } }

    private static bool WaitUntil(Func<bool> cond, int ms = 3000)
    {
        var end = DateTime.UtcNow.AddMilliseconds(ms);
        while (DateTime.UtcNow < end) { if (cond()) return true; Thread.Sleep(20); }
        return cond();
    }

    [Fact]
    public void Connects_Claims_ReadsModules_ExposesIo_WritesOutputs()
    {
        var head = new FakePlcHead();
        _mgr = new SimplePlcManager(_path, head, head);
        var id = _mgr.Add(new SimplePlcDeviceConfig { Name = "Fake", Host = "fake", AutoConnect = true });
        _mgr.Start();

        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Connected), "device never connected");
        var st = _mgr.GetAllStates().Single();
        Assert.True(st.Owned);
        Assert.Equal("SimplePLC-FAKE", st.HeadName);
        Assert.Equal(2, st.Slots.Count);
        Assert.Equal("DI8", st.Slots[0].Type);
        Assert.Equal(8, st.Slots[0].InBits);
        Assert.Equal("DO8", st.Slots[1].Type);
        Assert.Equal(8, st.Slots[1].OutBits);

        // IO exposure: 8 inputs (slot 1) + 8 outputs (slot 2) = 16 pins.
        var io = Assert.Single(_mgr.GetAllIoStates());
        Assert.Equal(16, io.Pins.Count);
        Assert.Equal(8, io.Pins.Count(p => p.Type == PinType.Input));
        Assert.Equal(8, io.Pins.Count(p => p.Type == PinType.Output));

        // An input event flips the right pins (bits 0 and 2 of slot 1).
        head.PushInput(1, 0b101);
        Assert.True(WaitUntil(() => _mgr.GetAllIoStates().Single().Pins.Count(p => p.Type == PinType.Input && p.Value) == 2));
        var inPins = _mgr.GetAllIoStates().Single().Pins.Where(p => p.Type == PinType.Input).OrderBy(p => p.Pin).ToList();
        Assert.True(inPins[0].Value);  // bit 0
        Assert.False(inPins[1].Value); // bit 1
        Assert.True(inPins[2].Value);  // bit 2

        // Writing an output pin reaches the head and reflects back in state.
        int slot2bit3 = 2 * SimplePlcDevice.BitsPerSlot + 3;
        Assert.True(_mgr.SetOutput(id, slot2bit3, true));
        Assert.True(WaitUntil(() => head.OutWrites.Any(o => o.slot == 2 && (o.value & (1 << 3)) != 0)));
        var outPin = _mgr.GetAllIoStates().Single().Pins.First(p => p.Pin == slot2bit3);
        Assert.True(outPin.Value);

        // The watchdog is being fed (owner pings while idle).
        Assert.True(WaitUntil(() => head.PingCount > 0, 2000), "no watchdog ping sent");
    }

    [Fact]
    public void Rediscovers_ByMac_WhenHeadAddressMoves()
    {
        var head = new FakePlcHead { GoodHost = "10.0.0.5" };   // the head now lives at a new (DHCP) address
        _mgr = new SimplePlcManager(_path, head, head);
        // Configured with a STALE host but the head's stable MAC.
        var id = _mgr.Add(new SimplePlcDeviceConfig { Name = "Fake", Host = "192.168.9.99", Mac = "02:53:50:AA:BB:CC", AutoConnect = true });
        _mgr.Start();

        // Fails on the stale host, rediscovers by MAC, reconnects at the real address, persists it.
        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Connected, 12000), "did not rediscover + reconnect");
        Assert.Equal("10.0.0.5", _mgr.GetAllStates().Single().Host);
    }

    [Fact]
    public void Reports_Supply_MacSource_FieldPower_And_Faults()
    {
        var head = new FakePlcHead { MacSource = "default", SupplyMv = 23980, Do8FieldPower = false };
        _mgr = new SimplePlcManager(_path, head, head);
        var id = _mgr.Add(new SimplePlcDeviceConfig { Name = "Fake", Host = "fake", AutoConnect = true });
        _mgr.Start();

        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Connected));
        var st = _mgr.GetAllStates().Single();
        // hello fields: shared default MAC (UI warns) + supply voltage.
        Assert.Equal("default", st.MacSource);
        Assert.Equal(23980, st.SupplyMv);
        // DO8 slot reports its field-power rail (here: cut); the DI8 slot doesn't report one.
        Assert.False(st.Slots.Single(s => s.Type == "DO8").FieldPower);
        Assert.Null(st.Slots.Single(s => s.Type == "DI8").FieldPower);

        // A field-power fault event on the DO8 slot shows up with its slot number.
        head.PushFault("field_power", slot: 2);
        head.PushFault("supply_low", mv: 18500);
        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Faults.Count == 2));
        var faults = _mgr.GetAllStates().Single().Faults;
        var fp = faults.Single(f => f.Code == "field_power");
        Assert.Equal(2, fp.Slot);
        var sl = faults.Single(f => f.Code == "supply_low");
        Assert.Equal(18500, sl.Mv);
        // Legacy single-code field is populated with one of the active faults for simple UIs.
        Assert.Contains(_mgr.GetAllStates().Single().Fault, new[] { "field_power", "supply_low" });

        // Clearing the field-power fault removes just that one.
        head.PushFaultClear("field_power", slot: 2);
        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Faults.Count == 1));
        Assert.Equal("supply_low", _mgr.GetAllStates().Single().Faults.Single().Code);
    }

    [Fact]
    public void DO8N_SinkingOutput_IsTreatedAsOutputCard()
    {
        // DO8N (type_id 6) is wire-identical to DO8: 8 outputs + field power, only the name differs.
        var head = new FakePlcHead { Do8TypeId = 6, Do8TypeName = "DO8N" };
        _mgr = new SimplePlcManager(_path, head, head);
        var id = _mgr.Add(new SimplePlcDeviceConfig { Name = "Fake", Host = "fake", AutoConnect = true });
        _mgr.Start();

        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Connected));
        var outSlot = _mgr.GetAllStates().Single().Slots.Single(s => s.OutBits > 0);
        Assert.Equal("DO8N", outSlot.Type);
        Assert.Equal(8, outSlot.OutBits);
        Assert.True(outSlot.FieldPower);

        // Its bits are exposed as output pins and writable, like any output card.
        var io = Assert.Single(_mgr.GetAllIoStates());
        Assert.Equal(8, io.Pins.Count(p => p.Type == PinType.Output && p.Pin / SimplePlcDevice.BitsPerSlot == 2));
        Assert.True(_mgr.SetOutput(id, 2 * SimplePlcDevice.BitsPerSlot + 1, true));
        Assert.True(WaitUntil(() => head.OutWrites.Any(o => o.slot == 2 && (o.value & (1 << 1)) != 0)));
    }

    [Fact]
    public void TypeName_Maps_Known_ModuleTypes()
    {
        Assert.Equal("DI8",  SimplePlcProtocol.TypeName(1));
        Assert.Equal("DO8",  SimplePlcProtocol.TypeName(2));
        Assert.Equal("DO8N", SimplePlcProtocol.TypeName(6));
        Assert.Equal("",     SimplePlcProtocol.TypeName(99));
    }

    [Fact]
    public void Pong_Keeps_SupplyMv_Live()
    {
        var head = new FakePlcHead { SupplyMv = 24010 };
        _mgr = new SimplePlcManager(_path, head, head);
        _mgr.Add(new SimplePlcDeviceConfig { Name = "Fake", Host = "fake", AutoConnect = true });
        _mgr.Start();

        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Connected));
        Assert.Equal(24010, _mgr.GetAllStates().Single().SupplyMv);
        // The head doesn't push state on voltage change — the watchdog pong carries supply_mv.
        head.SupplyMv = 22500;
        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().SupplyMv == 22500, 3000), "supply_mv did not refresh from pong");
    }

    [Fact]
    public void StateFaults_Array_Rebuilds_On_Read()
    {
        // The read reply's faults array is the authoritative snapshot (slot-suffixed entries).
        var head = new FakePlcHead();
        head.StateFaults.Add("field_power:2");
        head.StateFaults.Add("supply_low");
        _mgr = new SimplePlcManager(_path, head, head);
        _mgr.Add(new SimplePlcDeviceConfig { Name = "Fake", Host = "fake", AutoConnect = true });
        _mgr.Start();

        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Faults.Count == 2));
        var faults = _mgr.GetAllStates().Single().Faults;
        Assert.Equal(2, faults.Single(f => f.Code == "field_power").Slot);
        Assert.Null(faults.Single(f => f.Code == "supply_low").Slot);
    }

    [Fact]
    public void ReadOnly_WhenClaimRefused()
    {
        var head = new FakePlcHead { AllowClaim = false };
        _mgr = new SimplePlcManager(_path, head, head);
        var id = _mgr.Add(new SimplePlcDeviceConfig { Name = "Fake", Host = "fake", AutoConnect = true });
        _mgr.Start();

        Assert.True(WaitUntil(() => _mgr.GetAllStates().Single().Connected));
        Assert.False(_mgr.GetAllStates().Single().Owned);
        // A write when not owned is refused, not thrown.
        Assert.False(_mgr.SetOutput(id, 2 * SimplePlcDevice.BitsPerSlot + 0, true));
    }
}

[Collection("RobotController")]
public sealed class SimplePlcCommandsTests : IDisposable
{
    private readonly RobotController _robot = new();   // ctor only — no threads/connections started
    private readonly CommandDispatcher _d = new();
    public SimplePlcCommandsTests() => new SimplePlcCommands(_robot).Register(_d);
    public void Dispose() { }

    private JsonElement Send(string command, object? parameters = null)
    {
        Assert.True(_d.TryGet(command, out var handler), command);
        var msg = new CommandMessage { Type = "Command", Id = "t", Command = command, Params = parameters == null ? null : JsonSerializer.SerializeToElement(parameters) };
        var result = handler(msg).GetAwaiter().GetResult();
        return result == null ? JsonSerializer.SerializeToElement(new { }) : JsonSerializer.SerializeToElement(result);
    }

    [Fact]
    public void Add_Get_Remove_Through_Commands()
    {
        var id = Send("AddSimplePlcDevice", new { name = "Cell 3", host = "192.168.4.50", autoConnect = false })
            .GetProperty("deviceId").GetString()!;   // must be "deviceId", not "id"
        Assert.StartsWith("PLC_", id);

        var json = Send("GetSimplePlcDevices").GetProperty("simplePlcDevices").GetString()!;
        using var devices = JsonDocument.Parse(json);
        Assert.Contains(devices.RootElement.EnumerateArray(),
            e => e.GetProperty("id").GetString() == id && e.GetProperty("name").GetString() == "Cell 3");

        Assert.True(Send("SetSimplePlcDeviceConfig", new { id, name = "Cell 3B" }).GetProperty("ok").GetBoolean());
        Assert.True(Send("RemoveSimplePlcDevice", new { id }).GetProperty("ok").GetBoolean());
    }

    [Fact]
    public void Remove_Unknown_IsGraceful()
    {
        var r = Send("RemoveSimplePlcDevice", new { id = "PLC_999" });
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.False(string.IsNullOrEmpty(r.GetProperty("error").GetString()));
    }
}
