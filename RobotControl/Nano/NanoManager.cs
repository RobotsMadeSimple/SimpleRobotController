using System.Text.Json;

namespace Controller.RobotControl.Nano
{
    /// <summary>
    /// Loads nano_config.json, creates a NanoDevice per configured Nano, starts their
    /// connection threads, and provides a unified API for reading/writing IO across
    /// all devices.
    /// </summary>
    public sealed class NanoManager : IDisposable
    {
        private readonly List<NanoDevice> _devices = new();

        private static readonly JsonSerializerOptions _jsonOpts = new()
        {
            Converters           = { new System.Text.Json.Serialization.JsonStringEnumConverter() },
            PropertyNameCaseInsensitive = true,
            WriteIndented        = true,
        };

        // Path to the config file — kept so we can persist renames
        private readonly string _configPath;
        private NanoConfig      _config;

        /// <summary>Fired (on a device thread) when any Input pin changes.</summary>
        public event Action<string, NanoPinState>? InputChanged;  // (nanoId, pinState)

        public NanoManager(string configPath)
        {
            _configPath = configPath;

            if (!File.Exists(configPath))
            {
                Console.WriteLine($"[NanoManager] Config not found — writing default to {configPath}");
                _config = BuildDefaultConfig();
                SaveConfig();
            }
            else
            {
                var json = File.ReadAllText(configPath);
                _config = JsonSerializer.Deserialize<NanoConfig>(json, _jsonOpts) ?? BuildDefaultConfig();
            }

            foreach (var nanoCfg in _config.Nanos)
            {
                var device = new NanoDevice(nanoCfg);
                device.InputChanged      += state   => InputChanged?.Invoke(device.Id, state);
                device.ConnectionChanged += connected =>
                    Console.WriteLine($"[NanoManager] {device.Id} {(connected ? "connected" : "disconnected")}");
                _devices.Add(device);
            }
        }

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            foreach (var d in _devices) d.Start();
        }

        public void Stop()
        {
            foreach (var d in _devices) d.Stop();
        }

        public void Dispose() => Stop();

        // ── Device access ──────────────────────────────────────────────────────

        public IReadOnlyList<NanoDevice> GetDevices() => _devices;

        public NanoDevice? GetDevice(string nanoId) =>
            _devices.FirstOrDefault(d => d.Id == nanoId);

        // ── IO reads ───────────────────────────────────────────────────────────

        /// <summary>Returns the full live state of every configured nano + its pins.</summary>
        public List<NanoState> GetAllStates()
        {
            var result = new List<NanoState>(_devices.Count);
            foreach (var dev in _devices)
            {
                result.Add(new NanoState
                {
                    Id        = dev.Id,
                    Name      = dev.Name,
                    Connected = dev.Connected,
                    Pins      = dev.GetPinStates(),
                });
            }
            return result;
        }

        // ── IO writes ──────────────────────────────────────────────────────────

        public void SetOutput(string nanoId, int pin, bool value) =>
            GetDevice(nanoId)?.SetOutput(pin, value);

        public void SetNeoPixel(string nanoId, int pin, NeoPixelColor[] colors) =>
            GetDevice(nanoId)?.SetNeoPixel(pin, colors);

        public void ConfigurePin(string nanoId, int pin, PinType type, int pixelCount = 8) =>
            GetDevice(nanoId)?.ConfigurePin(pin, type, pixelCount);

        // ── Config mutations (persist to disk) ─────────────────────────────────

        /// <summary>
        /// Renames a pin label in the live state and in nano_config.json.
        /// Returns false if the nano or pin was not found.
        /// </summary>
        public bool RenamePin(string nanoId, int pin, string newName)
        {
            var dev = GetDevice(nanoId);
            if (dev == null) return false;

            var state = dev.GetPinState(pin);
            if (state == null) return false;

            state.Name = newName;

            // Persist to config
            var nanoCfg = _config.Nanos.FirstOrDefault(n => n.Id == nanoId);
            var pinCfg  = nanoCfg?.Pins.FirstOrDefault(p => p.Pin == pin);
            if (pinCfg != null)
            {
                pinCfg.Name = newName;
                SaveConfig();
            }

            return true;
        }

        /// <summary>
        /// Adds, updates, or removes a pin on both the live device state and
        /// nano_config.json.  Setting type = Unconfigured removes the pin.
        /// Creates the pin entry if it does not yet exist.
        /// </summary>
        public bool SetPinType(string nanoId, int pin, PinType type, int pixelCount = 8)
        {
            var dev = GetDevice(nanoId);
            if (dev == null) return false;

            var nanoCfg = _config.Nanos.FirstOrDefault(n => n.Id == nanoId);

            if (type == PinType.Unconfigured)
            {
                // Remove the pin entirely
                dev.RemovePinState(pin);
                nanoCfg?.Pins.RemoveAll(p => p.Pin == pin);
                SaveConfig();
                return true;
            }

            // Add or update live state
            dev.UpsertPinState(pin, type, dev.GetPinState(pin)?.Name ?? "", pixelCount);

            // Send firmware config command
            dev.ConfigurePin(pin, type, pixelCount);

            // Persist to config
            if (nanoCfg != null)
            {
                var pinCfg = nanoCfg.Pins.FirstOrDefault(p => p.Pin == pin);
                if (pinCfg != null)
                {
                    pinCfg.Type       = type;
                    pinCfg.PixelCount = pixelCount;
                }
                else
                {
                    nanoCfg.Pins.Add(new NanoPinConfig
                    {
                        Pin        = pin,
                        Type       = type,
                        Name       = "",
                        PixelCount = pixelCount,
                    });
                }
                SaveConfig();
            }

            return true;
        }

        // ── Status-light helper ────────────────────────────────────────────────

        /// <summary>
        /// Returns the first Neopixel pin found across all devices (used for the
        /// status light).  Returns null if no neopixel is configured.
        /// </summary>
        public (NanoDevice device, NanoPinState pin)? FindFirstNeopixel()
        {
            foreach (var dev in _devices)
            {
                foreach (var pin in dev.GetPinStates())
                {
                    if (pin.Type == PinType.Neopixel)
                        return (dev, pin);
                }
            }
            return null;
        }

        // ── Internal helpers ───────────────────────────────────────────────────

        private void SaveConfig()
        {
            try
            {
                var json = JsonSerializer.Serialize(_config, _jsonOpts);
                File.WriteAllText(_configPath, json);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[NanoManager] Failed to save config: {ex.Message}");
            }
        }

        private static NanoConfig BuildDefaultConfig() => new()
        {
            Nanos = new List<NanoDeviceConfig>
            {
                new()
                {
                    Id   = "ROBOT_NANO_001",
                    Name = "Main IO Board",
                    Pins = new List<NanoPinConfig>
                    {
                        new() { Pin = 2, Type = PinType.Input,    Name = "Input 1",      PixelCount = 1  },
                        new() { Pin = 3, Type = PinType.Input,    Name = "Input 2",      PixelCount = 1  },
                        new() { Pin = 4, Type = PinType.Input,    Name = "Input 3",      PixelCount = 1  },
                        new() { Pin = 5, Type = PinType.Input,    Name = "Input 4",      PixelCount = 1  },
                        new() { Pin = 7, Type = PinType.Output,   Name = "Output 1",     PixelCount = 1  },
                        new() { Pin = 8, Type = PinType.Output,   Name = "Output 2",     PixelCount = 1  },
                        new() { Pin = 6, Type = PinType.Neopixel, Name = "Status Light", PixelCount = 12 },
                    }
                }
            }
        };
    }
}
