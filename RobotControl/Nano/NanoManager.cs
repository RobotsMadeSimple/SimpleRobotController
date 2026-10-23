namespace Controller.RobotControl.Io
{
    /// <summary>
    /// Loads io_devices.json, creates a NanoDevice per configured Nano, starts their
    /// connection threads, and provides a unified API for reading/writing IO across
    /// all devices.
    /// </summary>
    public sealed class NanoManager : IDisposable
    {
        private readonly List<NanoDevice> _devices = new();

        // Path to the config file — kept so we can persist renames
        private readonly string _configPath;
        private IoDevicesConfig      _config;

        /// <summary>Fired (on a device thread) when any Input pin changes.</summary>
        public event Action<string, IoPinState>? InputChanged;  // (deviceId, pinState)

        /// <summary>Fired (on a device thread) for every HX711 load-cell reading. (deviceId, pinState, unixMs)</summary>
        public event Action<string, IoPinState, long>? LoadCellSampled;

        public NanoManager(string configPath)
        {
            _configPath = configPath;

            var loaded = Persistence.JsonFiles.Load<IoDevicesConfig>(configPath, logTag: "NanoManager");
            if (loaded != null)
            {
                _config = loaded;
            }
            else
            {
                Console.WriteLine($"[NanoManager] Config not found — writing default to {configPath}");
                _config = BuildDefaultConfig();
                SaveConfig();
            }

            foreach (var nanoCfg in _config.IoDevices)
            {
                var device = new NanoDevice(nanoCfg);
                device.InputChanged += state => InputChanged?.Invoke(device.Id, state);
                device.LoadCellSampled += (state, ts) => LoadCellSampled?.Invoke(device.Id, state, ts);
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

        public NanoDevice? GetDevice(string deviceId) =>
            _devices.FirstOrDefault(d => d.Id == deviceId);

        // ── IO reads ───────────────────────────────────────────────────────────

        /// <summary>Returns the full live state of every configured nano + its pins.</summary>
        public List<IoDeviceState> GetAllStates()
        {
            var result = new List<IoDeviceState>(_devices.Count);
            foreach (var dev in _devices)
            {
                result.Add(new IoDeviceState
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

        public void SetOutput(string deviceId, int pin, bool value) =>
            GetDevice(deviceId)?.SetOutput(pin, value);

        public void SetNeoPixel(string deviceId, int pin, NeoPixelColor[] colors) =>
            GetDevice(deviceId)?.SetNeoPixel(pin, colors);

        public void ConfigurePin(string deviceId, int pin, PinType type, int pixelCount = 8) =>
            GetDevice(deviceId)?.ConfigurePin(pin, type, pixelCount);

        // ── Load cell (HX711) ──────────────────────────────────────────────────

        /// <summary>Configure a pin as an HX711 load cell (DT = <paramref name="dtPin"/>, SCK = <paramref name="sckPin"/>)
        /// and persist it. Keeps any existing calibration/name for that pin.</summary>
        public bool SetLoadCellConfig(string deviceId, int dtPin, int sckPin, string unit)
        {
            var dev = GetDevice(deviceId);
            var nanoCfg = _config.IoDevices.FirstOrDefault(n => n.Id == deviceId);
            if (dev == null || nanoCfg == null) return false;

            var pinCfg = nanoCfg.Pins.FirstOrDefault(p => p.Pin == dtPin);
            if (pinCfg == null)
            {
                pinCfg = new IoPinConfig { Pin = dtPin, Name = "Load Cell" };
                nanoCfg.Pins.Add(pinCfg);
            }
            pinCfg.Type   = PinType.LoadCell;
            pinCfg.SckPin = sckPin;
            pinCfg.Unit   = string.IsNullOrEmpty(unit) ? "g" : unit;

            dev.UpsertPinState(dtPin, PinType.LoadCell, pinCfg.Name, pinCfg.PixelCount,
                sckPin, pinCfg.Unit, pinCfg.TareOffset, pinCfg.CountsPerGram);
            dev.ConfigurePin(dtPin, PinType.LoadCell, pinCfg.PixelCount, sckPin);
            SaveConfig();
            return true;
        }

        /// <summary>Zero the load cell: capture its current raw reading as the tare offset. Returns the new offset.</summary>
        public long? TareLoadCell(string deviceId, int pin)
        {
            var (dev, pinCfg, state) = ResolveLoadCell(deviceId, pin);
            if (dev == null || pinCfg == null || state == null) return null;

            pinCfg.TareOffset = state.Raw;
            dev.ApplyLoadCellCalibration(pin, pinCfg.TareOffset, pinCfg.CountsPerGram, pinCfg.Unit);
            SaveConfig();
            return pinCfg.TareOffset;
        }

        /// <summary>Second calibration point: with <paramref name="knownGrams"/> on the cell, derive counts-per-gram
        /// from the current raw reading (relative to the tare offset). Returns the new counts-per-gram.</summary>
        public double? CalibrateLoadCell(string deviceId, int pin, double knownGrams)
        {
            var (dev, pinCfg, state) = ResolveLoadCell(deviceId, pin);
            if (dev == null || pinCfg == null || state == null) return null;
            if (Math.Abs(knownGrams) < 1e-9) return null;   // need a non-zero known weight

            pinCfg.CountsPerGram = (state.Raw - pinCfg.TareOffset) / knownGrams;
            dev.ApplyLoadCellCalibration(pin, pinCfg.TareOffset, pinCfg.CountsPerGram, pinCfg.Unit);
            SaveConfig();
            return pinCfg.CountsPerGram;
        }

        /// <summary>Set the load cell's display unit (grams stays the master behind the scenes).</summary>
        public bool SetLoadCellUnit(string deviceId, int pin, string unit)
        {
            var (dev, pinCfg, _) = ResolveLoadCell(deviceId, pin);
            if (dev == null || pinCfg == null) return false;

            pinCfg.Unit = string.IsNullOrEmpty(unit) ? "g" : unit;
            dev.ApplyLoadCellCalibration(pin, pinCfg.TareOffset, pinCfg.CountsPerGram, pinCfg.Unit);
            SaveConfig();
            return true;
        }

        /// <summary>Current calibrated weight (grams) of a load cell, or null if not a connected load cell.</summary>
        public double? GetLoadCellGrams(string deviceId, int pin) =>
            GetDevice(deviceId)?.GetPinState(pin) is { Type: PinType.LoadCell } s ? s.Grams : null;

        private (NanoDevice? dev, IoPinConfig? cfg, IoPinState? state) ResolveLoadCell(string deviceId, int pin)
        {
            var dev = GetDevice(deviceId);
            var pinCfg = _config.IoDevices.FirstOrDefault(n => n.Id == deviceId)?.Pins
                .FirstOrDefault(p => p.Pin == pin && p.Type == PinType.LoadCell);
            var state = dev?.GetPinState(pin);
            return (dev, pinCfg, state);
        }

        // ── Config mutations (persist to disk) ─────────────────────────────────

        /// <summary>
        /// Renames a pin label in the live state and in io_devices.json.
        /// Returns false if the nano or pin was not found.
        /// </summary>
        public bool RenamePin(string deviceId, int pin, string newName)
        {
            var dev = GetDevice(deviceId);
            if (dev == null) return false;

            var state = dev.GetPinState(pin);
            if (state == null) return false;

            state.Name = newName;

            // Persist to config
            var nanoCfg = _config.IoDevices.FirstOrDefault(n => n.Id == deviceId);
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
        /// io_devices.json.  Setting type = Unconfigured removes the pin.
        /// Creates the pin entry if it does not yet exist.
        /// </summary>
        public bool SetPinType(string deviceId, int pin, PinType type, int pixelCount = 8)
        {
            var dev = GetDevice(deviceId);
            if (dev == null) return false;

            var nanoCfg = _config.IoDevices.FirstOrDefault(n => n.Id == deviceId);

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
                    nanoCfg.Pins.Add(new IoPinConfig
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
        public (NanoDevice device, IoPinState pin)? FindFirstNeopixel()
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
                Persistence.JsonFiles.Save(_configPath, _config);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[NanoManager] Failed to save config: {ex.Message}");
            }
        }

        private static IoDevicesConfig BuildDefaultConfig() => new()
        {
            IoDevices = new List<IoDeviceConfig>
            {
                new()
                {
                    Id   = "ROBOT_NANO_001",
                    Name = "Arduino Nano",
                    Pins = new List<IoPinConfig>
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
