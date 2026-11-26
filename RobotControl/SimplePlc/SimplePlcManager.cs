using System;
using System.Collections.Generic;
using System.Linq;
using Controller.RobotControl.Io;

namespace Controller.RobotControl.SimplePlc
{
    /// <summary>
    /// Loads simple_plc.json, runs one <see cref="SimplePlcDevice"/> per configured head, and offers
    /// CRUD + discovery + output writes over them. Mirrors <c>GenericSerialManager</c> / <c>NanoManager</c>:
    /// the config list is persisted and guarded for mutation; each head's I/O runs on its own thread.
    /// Construction is side-effect free; threads start only in <see cref="Start"/>.
    /// </summary>
    public sealed class SimplePlcManager : IDisposable
    {
        private readonly string _configPath;
        private readonly SimplePlcConfig _config;
        private readonly List<SimplePlcDevice> _devices = new();
        private readonly ISimplePlcTransportFactory _factory;
        private readonly ISimplePlcDiscovery _discovery;
        private readonly object _lock = new();
        private bool _started;

        public SimplePlcManager(string configPath)
            : this(configPath, new TcpSimplePlcTransportFactory(), new UdpSimplePlcDiscovery()) { }

        /// <summary>Test seam: inject a fake transport factory and discovery.</summary>
        public SimplePlcManager(string configPath, ISimplePlcTransportFactory factory, ISimplePlcDiscovery discovery)
        {
            _configPath = configPath;
            _factory = factory;
            _discovery = discovery;
            _config = Persistence.JsonFiles.Load<SimplePlcConfig>(configPath, logTag: "SimplePlcManager") ?? new SimplePlcConfig();
            foreach (var cfg in _config.SimplePlcDevices)
                _devices.Add(new SimplePlcDevice(cfg, _factory, _discovery, Save));
        }

        // ── Lifecycle ───────────────────────────────────────────────────────────

        public void Start()
        {
            lock (_lock) { _started = true; foreach (var d in _devices) d.Start(); }
        }

        public void Stop()
        {
            lock (_lock) foreach (var d in _devices) d.Stop();
        }

        public void Dispose() => Stop();

        // ── Access ──────────────────────────────────────────────────────────────

        public SimplePlcDevice? GetDevice(string? id) =>
            string.IsNullOrEmpty(id) ? null : _devices.FirstOrDefault(d => d.Id == id);

        public bool HasDevice(string? id) => GetDevice(id) != null;

        public List<SimplePlcDeviceState> GetAllStates()
        {
            lock (_lock) return _devices.Select(d => d.GetState()).ToList();
        }

        /// <summary>Each head as an IO device, for <c>RobotController.GetAllIoStates()</c> (dashboard + <c>$io</c>).</summary>
        public List<IoDeviceState> GetAllIoStates()
        {
            lock (_lock) return _devices.Select(d => d.GetIoDeviceState()).ToList();
        }

        // ── CRUD ────────────────────────────────────────────────────────────────

        /// <summary>Add a head from its config; generates an id, persists, starts it. Returns the id.</summary>
        public string Add(SimplePlcDeviceConfig cfg)
        {
            lock (_lock)
            {
                if (string.IsNullOrEmpty(cfg.Id)) cfg.Id = NextId();
                _config.SimplePlcDevices.Add(cfg);
                var dev = new SimplePlcDevice(cfg, _factory, _discovery, Save);
                _devices.Add(dev);
                Save();
                if (_started) dev.Start();
                return cfg.Id;
            }
        }

        public bool Remove(string id)
        {
            lock (_lock)
            {
                var dev = GetDevice(id);
                if (dev == null) return false;
                dev.Stop();
                _devices.Remove(dev);
                _config.SimplePlcDevices.RemoveAll(c => c.Id == id);
                Save();
                return true;
            }
        }

        /// <summary>Update a head's config in place (only non-null fields), persist, and reconnect.</summary>
        public bool SetConfig(string id, string? name, string? host, int? port, string? mac,
            bool? autoConnect, int? watchdogMs, int? filterMs, string? onTimeout)
        {
            lock (_lock)
            {
                var cfg = _config.SimplePlcDevices.FirstOrDefault(c => c.Id == id);
                var dev = GetDevice(id);
                if (cfg == null || dev == null) return false;

                if (name != null)            cfg.Name        = name;
                if (host != null)            cfg.Host        = host;
                if (port.HasValue)           cfg.Port        = port.Value;
                if (mac != null)             cfg.Mac         = mac;
                if (autoConnect.HasValue)    cfg.AutoConnect = autoConnect.Value;
                if (watchdogMs.HasValue)     cfg.WatchdogMs  = watchdogMs.Value;
                if (filterMs.HasValue)       cfg.FilterMs    = filterMs.Value;
                if (onTimeout is "hold" or "off") cfg.OnTimeout = onTimeout;

                dev.ApplyConfig(cfg);
                Save();
                return true;
            }
        }

        public bool SetConnected(string id, bool connected)
        {
            var dev = GetDevice(id);
            if (dev == null) return false;
            if (connected) dev.Open(); else dev.Close();
            lock (_lock)
            {
                var cfg = _config.SimplePlcDevices.FirstOrDefault(c => c.Id == id);
                if (cfg != null) { cfg.AutoConnect = connected; Save(); }
            }
            return true;
        }

        // ── Outputs / actions ─────────────────────────────────────────────────────

        /// <summary>Route an IO-dashboard / program output write to the right head (pin encodes slot+bit).</summary>
        public bool SetOutput(string deviceId, int pin, bool value) => GetDevice(deviceId)?.SetOutput(pin, value) ?? false;

        public bool Identify(string deviceId, int slot, int seconds) => GetDevice(deviceId)?.Identify(slot, seconds) ?? false;
        public bool Rescan(string deviceId) => GetDevice(deviceId)?.Rescan() ?? false;
        public bool SetHeadName(string deviceId, string name) => GetDevice(deviceId)?.SetHeadName(name) ?? false;

        // ── Discovery ─────────────────────────────────────────────────────────────

        /// <summary>Probe the network for heads, flagging the ones already configured.</summary>
        public List<SimplePlcDiscovered> Discover(int timeoutMs)
        {
            var heads = _discovery.Discover(timeoutMs).ToList();
            HashSet<string> knownMacs;
            lock (_lock) knownMacs = _config.SimplePlcDevices.Select(c => c.Mac).Where(m => !string.IsNullOrEmpty(m)).ToHashSet(StringComparer.OrdinalIgnoreCase);
            foreach (var h in heads) h.Added = !string.IsNullOrEmpty(h.Mac) && knownMacs.Contains(h.Mac);
            return heads;
        }

        // ── Internals ─────────────────────────────────────────────────────────────

        private string NextId()
        {
            int max = 0;
            foreach (var c in _config.SimplePlcDevices)
                if (c.Id is { } s && s.StartsWith("PLC_", StringComparison.OrdinalIgnoreCase)
                    && int.TryParse(s.AsSpan("PLC_".Length), out var n) && n > max) max = n;
            return $"PLC_{max + 1:000}";
        }

        private void Save()
        {
            // Lockable so a device thread (e.g. after MAC rediscovery updates its host) can persist
            // safely while the manager may also be mutating the list; Monitor is reentrant for the
            // CRUD callers that already hold _lock.
            lock (_lock)
            {
                try { Persistence.JsonFiles.Save(_configPath, _config); }
                catch (Exception ex) { Console.WriteLine($"[SimplePlcManager] Failed to save config: {ex.Message}"); }
            }
        }
    }
}
