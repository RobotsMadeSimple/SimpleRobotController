using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Threading;
using Controller.RobotControl.Io;

namespace Controller.RobotControl.SimplePlc
{
    /// <summary>
    /// Owns one head: a supervisor thread that connects, runs the §4 handshake (hello → modules →
    /// claim → config → subscribe → read), keeps the host watchdog fed with periodic <c>ping</c>s,
    /// applies pushed events, writes outputs on request, and reconnects when the session drops. All
    /// shared state is guarded by <see cref="_lock"/>; network calls are made outside the lock.
    ///
    /// Each module slot's bits are surfaced as controller IO pins (<see cref="GetIoPins"/>): a stable
    /// pin number encodes slot and bit (<c>slot * BitsPerSlot + bit</c>), so programs and the IO
    /// dashboard read inputs and write outputs the same way they do for a Nano.
    /// </summary>
    public sealed class SimplePlcDevice
    {
        public const int BitsPerSlot = 32;   // encoding room per slot (digital modules use ≤ 8)

        private const int ConnectTimeoutMs = 2500;
        private const int RequestTimeoutMs = 2500;
        private const int ReconnectDelayMs = 2000;
        /// <summary>Failed connects to the stored host before we rediscover the head by MAC (the head
        /// defaults to DHCP, so its address can move; the MAC is the stable identity).</summary>
        private const int RediscoverAfterFailures = 2;

        private readonly ISimplePlcTransportFactory _factory;
        private readonly ISimplePlcDiscovery? _discovery;
        private readonly Action? _persist;
        private readonly object _lock = new();

        private SimplePlcDeviceConfig _cfg;
        private Thread? _thread;
        private volatile bool _running;
        private volatile bool _wantConnected;
        private SimplePlcConnection? _conn;
        private long _lastActivityTick;
        private int _failedAttempts;
        private string _headMac = "";

        // Live state (under _lock)
        private bool _connected;
        private bool _owned;
        private string _headName = "";
        private string _firmware = "";
        private string _macSource = "";
        private int _supplyMv;
        private string _lastError = "";
        /// <summary>Active faults keyed by code (head-wide) or "code:slot" (per-module, e.g. field_power).</summary>
        private readonly Dictionary<string, SimplePlcFault> _faults = new();
        private readonly SortedDictionary<int, SlotRt> _slots = new();

        /// <summary>Raised when an input pin's value changes (for live UI / history), pin already updated.</summary>
        public event Action<IoPinState>? InputChanged;

        public SimplePlcDevice(SimplePlcDeviceConfig cfg, ISimplePlcTransportFactory factory,
            ISimplePlcDiscovery? discovery = null, Action? persist = null)
        {
            _cfg = cfg;
            _factory = factory;
            _discovery = discovery;
            _persist = persist;
            _wantConnected = cfg.AutoConnect;
        }

        public string Id => _cfg.Id;
        public string Name => _cfg.Name;
        public bool Connected => _connected;

        // ── Lifecycle ───────────────────────────────────────────────────────────

        public void Start()
        {
            if (_running) return;
            _running = true;
            _thread = new Thread(Supervise) { IsBackground = true, Name = $"SimplePlc:{_cfg.Id}" };
            _thread.Start();
        }

        public void Stop()
        {
            _running = false;
            var c = _conn;
            try { c?.Dispose(); } catch { }
            _thread?.Join(1500);
        }

        public void Open()  { _wantConnected = true; }
        public void Close() { _wantConnected = false; var c = _conn; try { c?.Dispose(); } catch { } }

        /// <summary>Replace the config (host/port/settings/name). Reconnects so changes take effect.</summary>
        public void ApplyConfig(SimplePlcDeviceConfig cfg)
        {
            lock (_lock) _cfg = cfg;
            _wantConnected = cfg.AutoConnect || _wantConnected;
            var c = _conn; try { c?.Dispose(); } catch { }   // drop so the supervisor reconnects with new settings
        }

        // ── Supervisor thread ─────────────────────────────────────────────────────

        private void Supervise()
        {
            while (_running)
            {
                if (!_wantConnected) { Thread.Sleep(150); continue; }
                if (_failedAttempts >= RediscoverAfterFailures) MaybeRediscoverByMac();
                SimplePlcConnection? conn = null;
                try
                {
                    conn = Connect();
                    _conn = conn;
                    _failedAttempts = 0;
                    RunSession(conn);
                }
                catch (Exception ex)
                {
                    _failedAttempts++;
                    SetError(ex.Message);
                }
                finally
                {
                    try { conn?.Dispose(); } catch { }
                    if (ReferenceEquals(_conn, conn)) _conn = null;
                    MarkDisconnected();
                }
                if (_running && _wantConnected) Thread.Sleep(ReconnectDelayMs);
            }
        }

        private SimplePlcConnection Connect()
        {
            SimplePlcDeviceConfig cfg; lock (_lock) cfg = _cfg;
            var transport = _factory.Connect(cfg.Host, cfg.Port, ConnectTimeoutMs);
            var conn = new SimplePlcConnection(transport);
            conn.OnEvent += HandleEvent;
            Touch();

            // hello → identity
            var hello = conn.Request("hello", new Dictionary<string, object?> { ["client"] = SimplePlcProtocol.ClientName, ["proto"] = SimplePlcProtocol.ProtoVersion }, RequestTimeoutMs);
            // modules → slot layout
            var modules = conn.Request("modules", null, RequestTimeoutMs);
            // claim ownership so we may write outputs and push settings (may be refused if Modbus/another owns it)
            var claim = conn.Request("claim", null, RequestTimeoutMs);
            bool owned = SimplePlcProtocol.Op(claim) == "ok";
            // push our settings once owned (the head echoes them)
            JsonElement? cfgReply = null;
            if (owned)
                cfgReply = conn.Request("config", new Dictionary<string, object?>
                {
                    ["watchdog_ms"] = cfg.WatchdogMs,
                    ["filter_ms"]   = cfg.FilterMs,
                    ["on_timeout"]  = cfg.OnTimeout,
                }, RequestTimeoutMs);
            conn.Send("subscribe", new Dictionary<string, object?> { ["in"] = true });
            var state = conn.Request("read", null, RequestTimeoutMs);

            ApplyHello(hello);
            ApplyModules(modules);
            ApplyReadState(state);
            if (cfgReply is { } cr) ApplyConfigReply(cr);
            lock (_lock)
            {
                _owned = owned;
                _connected = true;
                _lastError = owned ? "" : "another client owns the outputs (read-only)";
            }
            return conn;
        }

        private void RunSession(SimplePlcConnection conn)
        {
            while (_running && _wantConnected && !conn.Closed)
            {
                int pingEvery = PingIntervalMs();
                if (_owned && Environment.TickCount64 - Interlocked.Read(ref _lastActivityTick) >= pingEvery)
                {
                    try { conn.Send("ping"); Touch(); }
                    catch { break; }   // write failed → reconnect
                }
                Thread.Sleep(50);
            }
        }

        private int PingIntervalMs()
        {
            int wd; lock (_lock) wd = _cfg.WatchdogMs;
            return wd > 0 ? Math.Max(100, wd / 3) : 2000;
        }

        private void Touch() => Interlocked.Exchange(ref _lastActivityTick, Environment.TickCount64);

        /// <summary>
        /// After repeated connect failures, broadcast discovery and, if a head with our stored MAC
        /// answers at a different address, adopt that address and persist it. The head defaults to
        /// DHCP, so a lease change moves its IP; the MAC is the stable identity.
        /// </summary>
        private void MaybeRediscoverByMac()
        {
            if (_discovery == null) return;
            string mac, host;
            lock (_lock) { mac = !string.IsNullOrEmpty(_cfg.Mac) ? _cfg.Mac : _headMac; host = _cfg.Host; }
            if (string.IsNullOrEmpty(mac)) return;

            List<SimplePlcDiscovered> heads;
            try { heads = _discovery.Discover(1500).ToList(); } catch { return; }
            var match = heads.FirstOrDefault(h => MacEq(h.Mac, mac));
            if (match == null || string.IsNullOrEmpty(match.Host)) return;

            bool changed;
            lock (_lock)
            {
                changed = !string.Equals(_cfg.Host, match.Host, StringComparison.OrdinalIgnoreCase)
                          || (match.Port > 0 && _cfg.Port != match.Port);
                if (changed)
                {
                    _cfg.Host = match.Host;
                    if (match.Port > 0) _cfg.Port = match.Port;
                    _lastError = $"head moved to {match.Host} — rediscovered by MAC";
                }
            }
            if (changed) { _failedAttempts = 0; _persist?.Invoke(); }   // give the new address a fresh try
        }

        private static bool MacEq(string? a, string? b) =>
            !string.IsNullOrEmpty(a) && !string.IsNullOrEmpty(b) && string.Equals(a.Trim(), b.Trim(), StringComparison.OrdinalIgnoreCase);

        // ── Pushed events (§4.5) ──────────────────────────────────────────────────

        private void HandleEvent(JsonElement m)
        {
            switch (SimplePlcProtocol.Op(m))
            {
                case "in":        OnInputEvent(m); break;
                case "module":    OnModuleEvent(m); break;
                case "modules":   ApplyModules(m); break;
                case "state":     ApplyReadState(m); break;   // the head pushes state after hello / on change
                case "pong":      lock (_lock) { if (TryInt(m, "supply_mv", out int sv)) _supplyMv = sv; } break;  // watchdog ping keeps the headline voltage live
                case "fault":       OnFault(m, clear: false); break;
                case "fault_clear": OnFault(m, clear: true);  break;
            }
        }

        /// <summary>Apply a fault / fault_clear event (§4.5). field_power carries a slot, supply_low an mv.</summary>
        private void OnFault(JsonElement m, bool clear)
        {
            string code = Str(m, "code");
            if (string.IsNullOrEmpty(code)) return;
            int? slot = TryInt(m, "slot", out int sv) ? sv : (int?)null;
            int? mv   = TryInt(m, "mv",   out int mvv) ? mvv : (int?)null;
            string key = slot.HasValue ? $"{code}:{slot.Value}" : code;
            lock (_lock)
            {
                if (clear) _faults.Remove(key);
                else _faults[key] = new SimplePlcFault { Code = code, Slot = slot, Mv = mv };
            }
        }

        private void OnInputEvent(JsonElement m)
        {
            if (!TryInt(m, "slot", out int slot) || !TryInt(m, "value", out int value)) return;
            var changed = new List<IoPinState>();
            lock (_lock)
            {
                if (!_slots.TryGetValue(slot, out var s)) return;
                int old = s.InValue; s.InValue = value;
                if (old != value) changed.AddRange(PinsForSlot(s, inputs: true));
            }
            foreach (var pin in changed) { try { InputChanged?.Invoke(pin); } catch { } }
        }

        private void OnModuleEvent(JsonElement m)
        {
            if (!TryInt(m, "slot", out int slot)) return;
            lock (_lock) { if (_slots.TryGetValue(slot, out var s)) s.State = Str(m, "state"); }
        }

        // ── Reply appliers ────────────────────────────────────────────────────────

        private void ApplyHello(JsonElement m)
        {
            bool learned = false;
            string mac = Str(m, "mac");
            lock (_lock)
            {
                _headName = Str(m, "name");
                _firmware = Str(m, "fw");
                _macSource = Str(m, "mac_source");                 // "eeprom" | "default" | "" (older fw)
                if (TryInt(m, "supply_mv", out int sv)) _supplyMv = sv;
                if (!string.IsNullOrEmpty(mac))
                {
                    _headMac = mac;
                    if (string.IsNullOrEmpty(_cfg.Mac)) { _cfg.Mac = mac; learned = true; }   // learn MAC for future rediscovery
                }
            }
            if (learned) _persist?.Invoke();
        }

        private void ApplyModules(JsonElement m)
        {
            if (!m.TryGetProperty("modules", out var arr) || arr.ValueKind != JsonValueKind.Array) return;
            lock (_lock)
            {
                var seen = new HashSet<int>();
                foreach (var e in arr.EnumerateArray())
                {
                    if (!TryInt(e, "slot", out int slot)) continue;
                    seen.Add(slot);
                    if (!_slots.TryGetValue(slot, out var s)) { s = new SlotRt { Slot = slot }; _slots[slot] = s; }
                    s.TypeId  = IntOr(e, "type_id", s.TypeId);
                    s.Type    = m.ValueKind == JsonValueKind.Object && e.TryGetProperty("type", out var t) && t.ValueKind == JsonValueKind.String
                                ? t.GetString() ?? SimplePlcProtocol.TypeName(s.TypeId) : SimplePlcProtocol.TypeName(s.TypeId);
                    s.InBits  = IntOr(e, "in", s.InBits);
                    s.OutBits = IntOr(e, "out", s.OutBits);
                    s.State   = e.TryGetProperty("state", out var st) && st.ValueKind == JsonValueKind.String ? st.GetString() ?? "ok" : s.State;
                    if (e.TryGetProperty("field_power", out var fp) && (fp.ValueKind == JsonValueKind.True || fp.ValueKind == JsonValueKind.False))
                        s.FieldPower = fp.GetBoolean();   // only output modules report this
                }
                foreach (var slot in _slots.Keys.Where(k => !seen.Contains(k)).ToList()) _slots.Remove(slot);
            }
        }

        private void ApplyReadState(JsonElement m)
        {
            lock (_lock)
            {
                if (m.TryGetProperty("in", out var ins) && ins.ValueKind == JsonValueKind.Object)
                    foreach (var p in ins.EnumerateObject())
                        if (int.TryParse(p.Name, out int slot) && _slots.TryGetValue(slot, out var s) && p.Value.ValueKind == JsonValueKind.Number)
                            s.InValue = p.Value.GetInt32();
                if (m.TryGetProperty("out", out var outs) && outs.ValueKind == JsonValueKind.Object)
                    foreach (var p in outs.EnumerateObject())
                        if (int.TryParse(p.Name, out int slot) && _slots.TryGetValue(slot, out var s) && p.Value.ValueKind == JsonValueKind.Number)
                            s.OutValue = p.Value.GetInt32();
                if (TryInt(m, "supply_mv", out int sv)) _supplyMv = sv;
                // state.faults is the authoritative snapshot — rebuild from it (events fill gaps between reads).
                if (m.TryGetProperty("faults", out var fa) && fa.ValueKind == JsonValueKind.Array)
                {
                    _faults.Clear();
                    foreach (var f in fa.EnumerateArray())
                    {
                        if (f.ValueKind != JsonValueKind.String) continue;
                        var entry = f.GetString() ?? "";
                        if (entry.Length == 0) continue;
                        int colon = entry.IndexOf(':');
                        string code = colon >= 0 ? entry[..colon] : entry;              // "field_power:2" → code "field_power"
                        int? slot = colon >= 0 && int.TryParse(entry.AsSpan(colon + 1), out int sl) ? sl : (int?)null;
                        _faults[entry] = new SimplePlcFault { Code = code, Slot = slot };
                    }
                }
            }
        }

        private void ApplyConfigReply(JsonElement m)
        {
            lock (_lock)
            {
                _cfg.WatchdogMs = IntOr(m, "watchdog_ms", _cfg.WatchdogMs);
                _cfg.FilterMs   = IntOr(m, "filter_ms", _cfg.FilterMs);
                var ot = Str(m, "on_timeout");
                if (ot is "hold" or "off") _cfg.OnTimeout = ot;
            }
        }

        private void SetError(string msg) { lock (_lock) { _lastError = msg; } }

        private void MarkDisconnected()
        {
            lock (_lock) { _connected = false; _owned = false; _faults.Clear(); }   // live wire state; re-read on reconnect
        }

        // ── Outputs ───────────────────────────────────────────────────────────────

        /// <summary>Set one output bit (pin = slot*BitsPerSlot + bit). Returns false if not owned /
        /// not an output / not connected.</summary>
        public bool SetOutput(int pin, bool value)
        {
            int slot = pin / BitsPerSlot, bit = pin % BitsPerSlot;
            SimplePlcConnection? conn; bool owned;
            lock (_lock)
            {
                conn = _conn; owned = _owned;
                if (!_slots.TryGetValue(slot, out var s) || bit >= s.OutBits) return false;
            }
            if (conn == null || !owned) return false;
            try
            {
                int mask = 1 << bit;
                var reply = conn.Request("out", new Dictionary<string, object?>
                { ["slot"] = slot, ["mask"] = mask, ["value"] = value ? mask : 0 }, RequestTimeoutMs);
                Touch();
                if (SimplePlcProtocol.Op(reply) != "ok") return false;
                lock (_lock) { if (_slots.TryGetValue(slot, out var s)) s.OutValue = value ? (s.OutValue | mask) : (s.OutValue & ~mask); }
                return true;
            }
            catch { return false; }
        }

        /// <summary>Ask the head to blink a module's LED (slot 0 = the head).</summary>
        public bool Identify(int slot, int seconds)
        {
            var conn = _conn;
            if (conn == null) return false;
            try { var r = conn.Request("identify", new Dictionary<string, object?> { ["slot"] = slot, ["seconds"] = seconds }, RequestTimeoutMs); Touch(); return SimplePlcProtocol.Op(r) == "ok"; }
            catch { return false; }
        }

        /// <summary>Re-address the module stack (outputs go off during the scan).</summary>
        public bool Rescan()
        {
            var conn = _conn;
            if (conn == null) return false;
            try { var r = conn.Request("rescan", null, RequestTimeoutMs); Touch(); return SimplePlcProtocol.Op(r) == "ok"; }
            catch { return false; }
        }

        /// <summary>Rename the head itself (persisted on the head's flash).</summary>
        public bool SetHeadName(string name)
        {
            var conn = _conn;
            if (conn == null) return false;
            try { var r = conn.Request("setname", new Dictionary<string, object?> { ["name"] = name }, RequestTimeoutMs); Touch(); return SimplePlcProtocol.Op(r) == "ok"; }
            catch { return false; }
        }

        // ── State snapshots ───────────────────────────────────────────────────────

        public SimplePlcDeviceState GetState()
        {
            lock (_lock)
            {
                return new SimplePlcDeviceState
                {
                    Id = _cfg.Id, Name = _cfg.Name, Host = _cfg.Host, Port = _cfg.Port, Mac = _cfg.Mac,
                    AutoConnect = _cfg.AutoConnect, Connected = _connected, Owned = _owned,
                    HeadName = _headName, Firmware = _firmware, MacSource = _macSource, SupplyMv = _supplyMv,
                    WatchdogMs = _cfg.WatchdogMs, FilterMs = _cfg.FilterMs, OnTimeout = _cfg.OnTimeout,
                    Fault = _faults.Values.FirstOrDefault()?.Code ?? "",
                    Faults = _faults.Values.Select(f => new SimplePlcFault { Code = f.Code, Slot = f.Slot, Mv = f.Mv }).ToList(),
                    LastError = _lastError,
                    Slots = _slots.Values.Select(s => new SimplePlcSlotState
                    {
                        Slot = s.Slot, Type = s.Type, TypeId = s.TypeId, InBits = s.InBits, OutBits = s.OutBits,
                        State = s.State, InValue = s.InValue, OutValue = s.OutValue, FieldPower = s.FieldPower,
                    }).ToList(),
                };
            }
        }

        /// <summary>This head as an IO device for the dashboard / <c>$io</c> — one pin per module bit.</summary>
        public IoDeviceState GetIoDeviceState()
        {
            lock (_lock)
            {
                var pins = new List<IoPinState>();
                foreach (var s in _slots.Values)
                {
                    pins.AddRange(PinsForSlot(s, inputs: true));
                    pins.AddRange(PinsForSlot(s, inputs: false));
                }
                return new IoDeviceState
                {
                    Id = _cfg.Id, Name = _cfg.Name, Connected = _connected,
                    FirmwareVersion = _firmware, Capabilities = new[] { "io" }, Pins = pins,
                };
            }
        }

        private List<IoPinState> PinsForSlot(SlotRt s, bool inputs)
        {
            var list = new List<IoPinState>();
            int bits = inputs ? s.InBits : s.OutBits;
            int value = inputs ? s.InValue : s.OutValue;
            for (int bit = 0; bit < bits; bit++)
                list.Add(new IoPinState
                {
                    Pin = s.Slot * BitsPerSlot + bit,
                    Type = inputs ? PinType.Input : PinType.Output,
                    Value = ((value >> bit) & 1) != 0,
                    Name = $"S{s.Slot} {(inputs ? "IN" : "OUT")}{bit}",
                    DeviceId = _cfg.Id, DeviceName = _cfg.Name,
                });
            return list;
        }

        // ── small JSON readers ────────────────────────────────────────────────────

        private static string Str(JsonElement m, string k) =>
            m.ValueKind == JsonValueKind.Object && m.TryGetProperty(k, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() ?? "" : "";
        private static bool TryInt(JsonElement m, string k, out int v)
        {
            v = 0;
            return m.ValueKind == JsonValueKind.Object && m.TryGetProperty(k, out var e) && e.ValueKind == JsonValueKind.Number && e.TryGetInt32(out v);
        }
        private static int IntOr(JsonElement m, string k, int d) => TryInt(m, k, out var v) ? v : d;

        private sealed class SlotRt
        {
            public int Slot, TypeId, InBits, OutBits, InValue, OutValue;
            public string Type = "", State = "ok";
            public bool? FieldPower;   // output modules only; null when not reported
        }
    }
}
