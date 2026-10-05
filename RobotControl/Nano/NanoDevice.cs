using System.IO.Ports;
using Controller.RobotControl.Serial;

namespace Controller.RobotControl.Nano
{
    /// <summary>
    /// Manages the serial connection to a single Arduino Nano edge device.
    /// Runs entirely on its own background thread.
    /// </summary>
    public sealed class NanoDevice : SerialLineDevice
    {
        private readonly NanoDeviceConfig _config;

        // Live pin state keyed by Arduino pin number
        private readonly Dictionary<int, NanoPinState> _pinStates = new();

        /// <summary>Fired on the device thread when an Input pin changes value.</summary>
        public event Action<NanoPinState>? InputChanged;

        // ── SerialLineDevice knobs ─────────────────────────────────────────────
        protected override string DeviceTag                    => "Nano";
        protected override string ThreadNamePrefix              => "NanoDevice";
        protected override int    ProbeAttempts                 => 1;
        protected override int    ProbeResponseDelayMs          => 100;
        protected override bool   KeepProbePortOpenForSession   => false;

        public NanoDevice(NanoDeviceConfig config) : base(config.Id, config.Name)
        {
            _config = config;

            // Pre-populate pin state map from config so callers can read it even before connect
            foreach (var p in config.Pins)
            {
                _pinStates[p.Pin] = new NanoPinState
                {
                    Pin        = p.Pin,
                    Type       = p.Type,
                    Name       = p.Name,
                    NanoId     = config.Id,
                    NanoName   = config.Name,
                    PixelCount = p.PixelCount,
                    Value      = false,
                };
            }
        }

        // ── Session hooks ───────────────────────────────────────────────────────

        protected override void OnSessionStarted(SerialPort port)
        {
            Console.WriteLine($"[{LogPrefix}] Session started on {port.PortName}");

            // ── Send pin configuration ────────────────────────────────────────
            foreach (var pin in _config.Pins)
            {
                string typeChar = pin.Type switch
                {
                    PinType.Input    => "I",
                    PinType.Output   => "O",
                    PinType.Neopixel => "N",
                    _                => "I",
                };

                string cmd = pin.Type == PinType.Neopixel
                    ? $"CFG:{pin.Pin},{typeChar},{pin.PixelCount}"
                    : $"CFG:{pin.Pin},{typeChar}";

                SafeWrite(cmd);
                Thread.Sleep(30);
            }

            // Request full state so we are up-to-date immediately
            SafeWrite("GET");
        }

        protected override void OnSessionEnded()
        {
            Console.WriteLine($"[{LogPrefix}] Session ended");
        }

        // ── Protocol parsing ───────────────────────────────────────────────────

        protected override void OnLine(string line)
        {
            if (string.IsNullOrEmpty(line)) return;

            // STATE:<pin>,<type>,<value>;<pin>,<type>,<value>;...
            if (line.StartsWith("STATE:"))
            {
                foreach (string entry in line.Substring(6).Split(';'))
                {
                    string[] parts = entry.Split(',');
                    if (parts.Length >= 3 && int.TryParse(parts[0], out int pin))
                    {
                        NanoPinState? state;
                        lock (_lock) _pinStates.TryGetValue(pin, out state);
                        if (state != null)
                            state.Value = parts[2] != "0";
                    }
                }
                return;
            }

            // CHG:<pin>,<value>
            if (line.StartsWith("CHG:"))
            {
                string[] parts = line.Substring(4).Split(',');
                if (parts.Length >= 2 && int.TryParse(parts[0], out int pin))
                {
                    NanoPinState? state;
                    lock (_lock) _pinStates.TryGetValue(pin, out state);
                    if (state != null)
                    {
                        state.Value = parts[1] != "0";
                        if (state.Type == PinType.Input)
                            InputChanged?.Invoke(state);
                    }
                }
                return;
            }

            // Other lines (ID echo etc.) — ignore silently
        }

        // ── Public command API ─────────────────────────────────────────────────

        /// <summary>Queues a digital output change. Thread-safe.</summary>
        public void SetOutput(int pin, bool value)
        {
            NanoPinState? state;
            lock (_lock)
            {
                _commandQueue.Enqueue($"SET:{pin},{(value ? 1 : 0)}");
                _pinStates.TryGetValue(pin, out state);
            }
            // Optimistically update tracked state
            if (state != null)
                state.Value = value;
        }

        /// <summary>Queues a full neopixel strip update. Thread-safe.</summary>
        public void SetNeoPixel(int pin, NeoPixelColor[] colors)
        {
            if (colors.Length == 0) return;

            var sb = new System.Text.StringBuilder();
            sb.Append($"NEO:{pin}");
            foreach (var c in colors)
                sb.Append($",{c.R},{c.G},{c.B}");

            lock (_lock)
                _commandQueue.Enqueue(sb.ToString());
        }

        /// <summary>Queues a pin reconfiguration command. Thread-safe.</summary>
        public void ConfigurePin(int pin, PinType type, int pixelCount = 8)
        {
            string typeChar = type switch
            {
                PinType.Input    => "I",
                PinType.Output   => "O",
                PinType.Neopixel => "N",
                _                => "I",
            };

            string cmd = type == PinType.Neopixel
                ? $"CFG:{pin},{typeChar},{pixelCount}"
                : $"CFG:{pin},{typeChar}";

            lock (_lock)
                _commandQueue.Enqueue(cmd);
        }

        // ── State accessors ────────────────────────────────────────────────────

        public List<NanoPinState> GetPinStates()
        {
            lock (_lock) return _pinStates.Values.ToList();
        }

        public NanoPinState? GetPinState(int pin)
        {
            lock (_lock) return _pinStates.TryGetValue(pin, out var s) ? s : null;
        }

        /// <summary>
        /// Adds or updates the tracked state for a pin without sending a firmware
        /// command — call ConfigurePin separately when the device is connected.
        /// </summary>
        public void UpsertPinState(int pin, PinType type, string name, int pixelCount)
        {
            lock (_lock)
            {
                if (_pinStates.TryGetValue(pin, out var existing))
                {
                    existing.Type       = type;
                    existing.Name       = name;
                    existing.PixelCount = pixelCount;
                }
                else
                {
                    _pinStates[pin] = new NanoPinState
                    {
                        Pin        = pin,
                        Type       = type,
                        Name       = name,
                        NanoId     = _config.Id,
                        NanoName   = _config.Name,
                        PixelCount = pixelCount,
                        Value      = false,
                    };
                }
            }
        }

        /// <summary>Removes a pin from live state tracking.</summary>
        public void RemovePinState(int pin)
        {
            lock (_lock) _pinStates.Remove(pin);
        }
    }
}
