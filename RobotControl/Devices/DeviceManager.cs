using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;

namespace Controller.RobotControl.Devices
{
    /// <summary>
    /// Owns phone/client "data devices" and their latest pushed channel values. Mirrors
    /// <see cref="Camera.CameraManager"/>: the config list (id/name/enabled) is persisted,
    /// while the live values are in-memory and arrive over <c>/devices/{id}/ingest</c>.
    /// Channels are discovered from what the phone sends (not pre-declared), and surface in
    /// expressions as <c>$device.&lt;id&gt;.&lt;channel&gt;</c> via <c>RobotPropertySource</c>.
    /// Lock-free reads; the config list is guarded for mutation.
    /// </summary>
    public sealed class DeviceManager
    {
        private sealed class Runtime
        {
            public readonly ConcurrentDictionary<string, DeviceChannelState> Channels =
                new(StringComparer.OrdinalIgnoreCase);
            public long LastIngestMs = long.MinValue / 2;
        }

        /// <summary>A device with no push in this long reads as disconnected.</summary>
        private const int StaleMs = 5000;

        private readonly string _configPath;
        private readonly DeviceManagerConfig _config;
        private readonly object _configLock = new();
        private readonly ConcurrentDictionary<string, Runtime> _runtime =
            new(StringComparer.OrdinalIgnoreCase);

        public DeviceManager(string configPath)
        {
            _configPath = configPath;
            _config = Load();
            foreach (var d in _config.Devices) _runtime.TryAdd(d.Id, new Runtime());
        }

        private List<DeviceConfig> Snapshot()
        {
            lock (_configLock) return _config.Devices.ToList();
        }

        public bool Exists(string id) => _runtime.ContainsKey(id);

        // ── Config mutations ──────────────────────────────────────────────────

        public string AddDevice(DeviceConfig cfg)
        {
            lock (_configLock)
            {
                if (string.IsNullOrEmpty(cfg.Id)) cfg.Id = $"DEV_{_config.Devices.Count}";
                _config.Devices.Add(cfg);
                Save();
            }
            _runtime.TryAdd(cfg.Id, new Runtime());
            return cfg.Id;
        }

        public bool RemoveDevice(string id)
        {
            bool removed;
            lock (_configLock)
            {
                removed = _config.Devices.RemoveAll(d => string.Equals(d.Id, id, StringComparison.OrdinalIgnoreCase)) > 0;
                if (removed) Save();
            }
            _runtime.TryRemove(id, out _);
            return removed;
        }

        public bool UpdateDevice(string id, string? name, bool? enabled, string? providerConfig = null)
        {
            lock (_configLock)
            {
                var d = _config.Devices.FirstOrDefault(x => string.Equals(x.Id, id, StringComparison.OrdinalIgnoreCase));
                if (d == null) return false;
                if (name != null) d.Name = name;
                if (enabled.HasValue) d.Enabled = enabled.Value;
                // Opaque app-owned blob: stored verbatim. Passed only when the app saves settings.
                if (providerConfig != null) d.ProviderConfig = providerConfig;
                Save();
                return true;
            }
        }

        // ── Ingest + reads ──────────────────────────────────────────────────────

        /// <summary>Records a pushed channel value. False if the device id is unknown.</summary>
        public bool Ingest(string id, string channel, double value, string? unit, long unixMs)
        {
            if (string.IsNullOrEmpty(channel) || !_runtime.TryGetValue(id, out var rt)) return false;
            rt.Channels[channel] = new DeviceChannelState { Key = channel, Value = value, Unit = unit, UnixMs = unixMs };
            rt.LastIngestMs = Environment.TickCount64;
            return true;
        }

        /// <summary>Latest value of <c>$device.&lt;id&gt;.&lt;channel&gt;</c>; false if unknown.</summary>
        public bool TryGetChannel(string id, string channel, out double value)
        {
            value = 0;
            if (!_runtime.TryGetValue(id, out var rt)) return false;
            if (!rt.Channels.TryGetValue(channel, out var ch)) return false;
            value = ch.Value;
            return true;
        }

        private static bool Fresh(Runtime rt) => (Environment.TickCount64 - rt.LastIngestMs) < StaleMs;

        public List<DeviceState> GetState()
        {
            var list = new List<DeviceState>();
            foreach (var cfg in Snapshot())
            {
                _runtime.TryGetValue(cfg.Id, out var rt);
                list.Add(new DeviceState
                {
                    Id             = cfg.Id,
                    Name           = cfg.Name,
                    Enabled        = cfg.Enabled,
                    Connected      = rt != null && Fresh(rt),
                    ProviderConfig = cfg.ProviderConfig,
                    Channels  = rt == null
                        ? new List<DeviceChannelState>()
                        : rt.Channels.Values.OrderBy(c => c.Key, StringComparer.OrdinalIgnoreCase).ToList(),
                });
            }
            return list;
        }

        /// <summary>Every <c>(deviceId, channel, unit)</c> currently known — for the expression picker/validation.</summary>
        public IEnumerable<(string Id, string Channel, string Unit)> ListChannels()
        {
            foreach (var cfg in Snapshot())
                if (_runtime.TryGetValue(cfg.Id, out var rt))
                    foreach (var ch in rt.Channels.Values.OrderBy(c => c.Key, StringComparer.OrdinalIgnoreCase))
                        yield return (cfg.Id, ch.Key, ch.Unit ?? "");
        }

        // ── Persistence ──────────────────────────────────────────────────────────

        private DeviceManagerConfig Load() =>
            Persistence.JsonFiles.Load<DeviceManagerConfig>(_configPath, logTag: "DeviceManager") ?? new DeviceManagerConfig();

        private void Save()
        {
            try { Persistence.JsonFiles.Save(_configPath, _config); }
            catch (Exception ex) { Console.WriteLine($"[DeviceManager] Failed to save config: {ex.Message}"); }
        }
    }
}
