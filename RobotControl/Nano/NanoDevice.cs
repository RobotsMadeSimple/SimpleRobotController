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

        /// <summary>Fired on the device thread for every HX711 load-cell reading (pinState, unixMs).
        /// Carries the just-updated state (Raw/Grams/Unit/Name) so a recorder can capture every sample.</summary>
        public event Action<NanoPinState, long>? LoadCellSampled;

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
                    Pin           = p.Pin,
                    Type          = p.Type,
                    Name          = p.Name,
                    NanoId        = config.Id,
                    NanoName      = config.Name,
                    PixelCount    = p.PixelCount,
                    Value         = false,
                    SckPin        = p.SckPin,
                    Unit          = p.Unit,
                    TareOffset    = p.TareOffset,
                    CountsPerGram = p.CountsPerGram,
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
                SafeWrite(ConfigCommand(pin.Pin, pin.Type, pin.PixelCount, pin.SckPin));
                Thread.Sleep(30);
            }

            // Request full state so we are up-to-date immediately
            SafeWrite("GET");
        }

        protected override void OnSessionEnded()
        {
            Console.WriteLine($"[{LogPrefix}] Session ended");
        }

        /// <summary>The firmware CFG command for a pin. Load cells carry the SCK pin in place of pixel count.</summary>
        private static string ConfigCommand(int pin, PinType type, int pixelCount, int sckPin) => type switch
        {
            PinType.Neopixel => $"CFG:{pin},N,{pixelCount}",
            PinType.LoadCell => $"CFG:{pin},L,{sckPin}",
            PinType.Output   => $"CFG:{pin},O",
            _                => $"CFG:{pin},I",
        };

        /// <summary>Convert a raw HX711 count to grams using the channel's calibration (0 when uncalibrated).</summary>
        private static double RawToGrams(long raw, long tareOffset, double countsPerGram) =>
            countsPerGram != 0 ? (raw - tareOffset) / countsPerGram : 0;

        // Per-load-cell sample-rate estimate (ticks of the last LC line, smoothed).
        private readonly Dictionary<int, long> _lcLastTicks = new();

        // ── Protocol parsing ───────────────────────────────────────────────────

        protected override void OnLine(string line)
        {
            if (string.IsNullOrEmpty(line)) return;

            // STATE:<pin>,<type>,<value>;<pin>,<type>,<value>;...  (type L: value is the raw count)
            if (line.StartsWith("STATE:"))
            {
                foreach (string entry in line.Substring(6).Split(';'))
                {
                    string[] parts = entry.Split(',');
                    if (parts.Length >= 3 && int.TryParse(parts[0], out int pin))
                    {
                        NanoPinState? state;
                        lock (_lock) _pinStates.TryGetValue(pin, out state);
                        if (state == null) continue;
                        if (parts[1] == "L" && long.TryParse(parts[2], out long raw))
                            UpdateLoadCellRaw(state, raw);
                        else
                            state.Value = parts[2] != "0";
                    }
                }
                return;
            }

            // LC:<pin>,<raw>  — streamed HX711 reading
            if (line.StartsWith("LC:"))
            {
                string[] parts = line.Substring(3).Split(',');
                if (parts.Length >= 2 && int.TryParse(parts[0], out int pin) && long.TryParse(parts[1], out long raw))
                {
                    NanoPinState? state;
                    lock (_lock) _pinStates.TryGetValue(pin, out state);
                    if (state != null) UpdateLoadCellRaw(state, raw);
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

        /// <summary>Store a new raw HX711 reading on a load-cell pin: update grams + estimate the sample rate.</summary>
        private void UpdateLoadCellRaw(NanoPinState state, long raw)
        {
            state.Raw   = raw;
            state.Grams = RawToGrams(raw, state.TareOffset, state.CountsPerGram);

            long now = Environment.TickCount64;
            lock (_lock)
            {
                if (_lcLastTicks.TryGetValue(state.Pin, out long last) && now > last)
                {
                    double inst = 1000.0 / (now - last);
                    state.SampleRateHz = state.SampleRateHz <= 0 ? inst : state.SampleRateHz * 0.8 + inst * 0.2;
                }
                _lcLastTicks[state.Pin] = now;
            }

            // Per-sample fidelity for the history/recording engine. Timestamp is assigned here
            // (receive time) since the firmware sends no timestamp. Fires on the device thread.
            LoadCellSampled?.Invoke(state, DateTimeOffset.UtcNow.ToUnixTimeMilliseconds());
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

        /// <summary>Queues a pin reconfiguration command. Thread-safe. For a load cell, pass the HX711 SCK pin.</summary>
        public void ConfigurePin(int pin, PinType type, int pixelCount = 8, int sckPin = -1)
        {
            lock (_lock)
                _commandQueue.Enqueue(ConfigCommand(pin, type, pixelCount, sckPin));
        }

        /// <summary>Apply a load cell's calibration to its live state so grams recompute immediately. Thread-safe.</summary>
        public void ApplyLoadCellCalibration(int pin, long tareOffset, double countsPerGram, string unit)
        {
            lock (_lock)
            {
                if (!_pinStates.TryGetValue(pin, out var s)) return;
                s.TareOffset    = tareOffset;
                s.CountsPerGram = countsPerGram;
                s.Unit          = unit;
                s.Grams         = RawToGrams(s.Raw, tareOffset, countsPerGram);
            }
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
        public void UpsertPinState(int pin, PinType type, string name, int pixelCount,
            int sckPin = -1, string unit = "g", long tareOffset = 0, double countsPerGram = 0)
        {
            lock (_lock)
            {
                if (_pinStates.TryGetValue(pin, out var existing))
                {
                    existing.Type          = type;
                    existing.Name          = name;
                    existing.PixelCount    = pixelCount;
                    existing.SckPin        = sckPin;
                    existing.Unit          = unit;
                    existing.TareOffset    = tareOffset;
                    existing.CountsPerGram = countsPerGram;
                }
                else
                {
                    _pinStates[pin] = new NanoPinState
                    {
                        Pin           = pin,
                        Type          = type,
                        Name          = name,
                        NanoId        = _config.Id,
                        NanoName      = _config.Name,
                        PixelCount    = pixelCount,
                        Value         = false,
                        SckPin        = sckPin,
                        Unit          = unit,
                        TareOffset    = tareOffset,
                        CountsPerGram = countsPerGram,
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
