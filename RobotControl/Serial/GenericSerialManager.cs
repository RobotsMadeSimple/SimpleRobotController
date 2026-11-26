using System.Collections.Generic;

namespace Controller.RobotControl.Serial
{
    /// <summary>
    /// Loads serial_devices.json, creates a <see cref="GenericSerialDevice"/> per configured device,
    /// runs their connection threads, and offers CRUD + send over them. Mirrors <c>NanoManager</c>
    /// and the phone <c>DeviceManager</c>: the config list is persisted and guarded for mutation;
    /// device I/O runs on each device's own background thread.
    /// </summary>
    public sealed class GenericSerialManager : IDisposable
    {
        private readonly string _configPath;
        private readonly SerialDevicesConfig _config;
        private readonly List<GenericSerialDevice> _devices = new();
        private readonly object _lock = new();
        private bool _started;

        public GenericSerialManager(string configPath)
        {
            _configPath = configPath;
            _config = Persistence.JsonFiles.Load<SerialDevicesConfig>(configPath, logTag: "GenericSerialManager")
                      ?? new SerialDevicesConfig();
            foreach (var cfg in _config.SerialDevices)
                _devices.Add(new GenericSerialDevice(cfg));
        }

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            lock (_lock)
            {
                _started = true;
                foreach (var d in _devices) d.Start();
            }
        }

        public void Stop()
        {
            lock (_lock)
                foreach (var d in _devices) d.Stop();
        }

        public void Dispose() => Stop();

        // ── Access ───────────────────────────────────────────────────────────────

        public GenericSerialDevice? GetDevice(string? id) =>
            string.IsNullOrEmpty(id) ? null : _devices.FirstOrDefault(d => d.Id == id);

        /// <summary>Resolve by stable id first, then by display name (id-over-name).</summary>
        public GenericSerialDevice? Resolve(string? id, string? name)
        {
            var byId = GetDevice(id);
            if (byId != null) return byId;
            if (string.IsNullOrEmpty(name)) return null;
            var cfg = _config.SerialDevices.FirstOrDefault(c => string.Equals(c.Name, name, StringComparison.OrdinalIgnoreCase));
            return cfg == null ? null : GetDevice(cfg.Id);
        }

        public List<SerialDeviceState> GetAllStates()
        {
            lock (_lock) return _devices.Select(d => d.GetState()).ToList();
        }

        /// <summary>The configured port names (for ListSerialPorts in-use cross-reference), id→name→port.</summary>
        public IReadOnlyList<SerialDeviceConfig> Configs()
        {
            lock (_lock) return _config.SerialDevices.ToList();
        }

        // ── CRUD ───────────────────────────────────────────────────────────────

        /// <summary>Add a device from the Add params. Generates an id, persists, starts it. Returns the id.</summary>
        public string Add(SerialDeviceConfig cfg)
        {
            lock (_lock)
            {
                if (string.IsNullOrEmpty(cfg.Id)) cfg.Id = NextId();
                _config.SerialDevices.Add(cfg);
                var dev = new GenericSerialDevice(cfg);
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
                _config.SerialDevices.RemoveAll(c => c.Id == id);
                Save();
                return true;
            }
        }

        /// <summary>
        /// Update a device's config in place (only the non-null fields), persist, and reopen so new
        /// framing/port takes effect. Returns false if the device is unknown.
        /// </summary>
        public bool SetConfig(string id, string? name, string? portName, int? baudRate, int? dataBits,
            System.IO.Ports.Parity? parity, System.IO.Ports.StopBits? stopBits, string? writeLineEnding,
            bool? autoConnect, List<SerialCommandDef>? commands)
        {
            lock (_lock)
            {
                var cfg = _config.SerialDevices.FirstOrDefault(c => c.Id == id);
                var dev = GetDevice(id);
                if (cfg == null || dev == null) return false;

                if (name != null)            cfg.Name            = name;
                if (portName != null)        cfg.PortName        = portName;
                if (baudRate.HasValue)       cfg.BaudRate        = baudRate.Value;
                if (dataBits.HasValue)       cfg.DataBits        = dataBits.Value;
                if (parity.HasValue)         cfg.Parity          = parity.Value;
                if (stopBits.HasValue)       cfg.StopBits        = stopBits.Value;
                if (writeLineEnding != null) cfg.WriteLineEnding = writeLineEnding;
                if (autoConnect.HasValue)    cfg.AutoConnect     = autoConnect.Value;
                if (commands != null)        cfg.Commands        = commands;

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
            // Persist the intent as autoConnect so a restart honours it.
            lock (_lock)
            {
                var cfg = _config.SerialDevices.FirstOrDefault(c => c.Id == id);
                if (cfg != null) { cfg.AutoConnect = connected; Save(); }
            }
            return true;
        }

        // ── Send ───────────────────────────────────────────────────────────────

        public sealed class SendResult
        {
            public bool Ok { get; init; }
            public string Error { get; init; } = "";
            public string Reply { get; init; } = "";
            public List<string> Lines { get; init; } = new();
        }

        /// <summary>Send a named command (by id) or raw <paramref name="text"/> to a device, optionally
        /// waiting for a reply. Never throws — an unknown device / closed port returns Ok=false.</summary>
        public SendResult Send(string id, string? commandId, string? text, bool? expectReply, int? replyTimeoutMs)
        {
            var dev = GetDevice(id);
            if (dev == null) return new SendResult { Ok = false, Error = $"Serial device '{id}' not found." };

            string payload;
            bool wait;
            int timeout;

            var cmd = dev.GetCommand(commandId);
            if (cmd != null)
            {
                payload = cmd.Payload;
                wait    = expectReply ?? cmd.ExpectReply;
                timeout = replyTimeoutMs ?? cmd.ReplyTimeoutMs;
            }
            else if (!string.IsNullOrEmpty(commandId))
            {
                return new SendResult { Ok = false, Error = $"Command '{commandId}' not found on device '{id}'." };
            }
            else
            {
                payload = text ?? "";
                wait    = expectReply ?? false;
                timeout = replyTimeoutMs ?? 1000;
            }

            if (!dev.Connected)
                return new SendResult { Ok = false, Error = $"Serial device '{id}' is not connected." };

            if (!wait)
            {
                dev.SendLine(payload);
                return new SendResult { Ok = true };
            }

            var lines = dev.SendAndWait(payload, timeout).ToList();
            return new SendResult
            {
                Ok    = true,
                Reply = lines.Count > 0 ? lines[0] : "",
                Lines = lines,
            };
        }

        // ── Internals ────────────────────────────────────────────────────────────

        private string NextId()
        {
            int max = 0;
            foreach (var c in _config.SerialDevices)
            {
                var s = c.Id;
                if (s != null && s.StartsWith("SERIAL_", StringComparison.OrdinalIgnoreCase)
                    && int.TryParse(s.AsSpan("SERIAL_".Length), out var n) && n > max)
                    max = n;
            }
            return $"SERIAL_{max + 1:000}";
        }

        private void Save()
        {
            try { Persistence.JsonFiles.Save(_configPath, _config); }
            catch (Exception ex) { Console.WriteLine($"[GenericSerialManager] Failed to save config: {ex.Message}"); }
        }
    }
}
