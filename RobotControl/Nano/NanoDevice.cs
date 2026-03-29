using System.IO.Ports;

namespace Controller.RobotControl.Nano
{
    /// <summary>
    /// Manages the serial connection to a single Arduino Nano edge device.
    /// Runs entirely on its own background thread.
    /// </summary>
    public sealed class NanoDevice : IDisposable
    {
        private readonly NanoDeviceConfig _config;
        private SerialPort? _port;
        private Thread?     _thread;
        private volatile bool _running;

        private readonly object        _lock         = new();
        private readonly Queue<string> _commandQueue = new();

        /// <summary>Physical port this device is currently connected on, or null.</summary>
        public string? PortName   { get; private set; }
        public string  Id         => _config.Id;
        public string  Name       => _config.Name;
        public bool    Connected  { get; private set; }

        // Live pin state keyed by Arduino pin number
        private readonly Dictionary<int, NanoPinState> _pinStates = new();

        /// <summary>Fired on the device thread when an Input pin changes value.</summary>
        public event Action<NanoPinState>? InputChanged;

        /// <summary>Fired on the device thread when the connection is established or lost.</summary>
        public event Action<bool>? ConnectionChanged;

        public NanoDevice(NanoDeviceConfig config)
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

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            _running = true;
            _thread  = new Thread(ConnectionLoop)
            {
                IsBackground = true,
                Name         = $"NanoDevice-{_config.Id}"
            };
            _thread.Start();
        }

        public void Stop()
        {
            _running = false;
            try { _port?.Close(); } catch { }
        }

        public void Dispose() => Stop();

        // ── Connection loop ────────────────────────────────────────────────────

        private void ConnectionLoop()
        {
            while (_running)
            {
                try
                {
                    string? port = ScanForDevice();
                    if (port == null)
                    {
                        Thread.Sleep(3000);
                        continue;
                    }

                    RunSession(port);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Nano:{_config.Id}] Unhandled error: {ex.Message}");
                }

                if (Connected)
                {
                    Connected = false;
                    PortName  = null;
                    ConnectionChanged?.Invoke(false);
                }

                if (_running) Thread.Sleep(3000);
            }
        }

        /// <summary>Iterates available COM ports looking for the device with our ID.</summary>
        private string? ScanForDevice()
        {
            string[] ports = SerialPort.GetPortNames();
            if (ports.Length == 0) return null;

            Console.WriteLine($"[Nano:{_config.Id}] Scanning {ports.Length} port(s): {string.Join(", ", ports)}");

            foreach (string portName in ports)
            {
                if (!_running) return null;

                try
                {
                    using var probe = new SerialPort(portName, 115200)
                    {
                        ReadTimeout  = 2500,
                        WriteTimeout = 1000,
                        NewLine      = "\n",
                    };
                    probe.Open();

                    // Arduino resets on DTR toggle when the port opens — wait for boot
                    Thread.Sleep(2000);

                    probe.WriteLine("ID?");
                    Thread.Sleep(100);

                    string line = probe.ReadLine().Trim();
                    Console.WriteLine($"[Nano:{_config.Id}] {portName} → \"{line}\"");

                    if (line == $"ID:{_config.Id}")
                    {
                        Console.WriteLine($"[Nano:{_config.Id}] Matched on {portName}");
                        return portName;
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Nano:{_config.Id}] {portName}: {ex.Message}");
                }
            }

            return null;
        }

        /// <summary>Opens the known port, configures pins, then runs the read/write loop.</summary>
        private void RunSession(string portName)
        {
            _port = new SerialPort(portName, 115200)
            {
                ReadTimeout  = 100,   // short so we can service the write queue promptly
                WriteTimeout = 2000,
                NewLine      = "\n",
            };

            _port.Open();

            // Another reset delay after the real open
            Thread.Sleep(2000);

            PortName  = portName;
            Connected = true;
            ConnectionChanged?.Invoke(true);

            Console.WriteLine($"[Nano:{_config.Id}] Session started on {portName}");

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

            // ── Read / write loop ─────────────────────────────────────────────
            string readBuf = "";

            while (_running && _port.IsOpen)
            {
                // Drain outbound command queue
                lock (_lock)
                {
                    while (_commandQueue.Count > 0)
                        SafeWrite(_commandQueue.Dequeue());
                }

                // Read one byte; ReadTimeout=100 means we spin back to flush the queue
                try
                {
                    int b = _port.ReadByte();
                    if (b == '\n')
                    {
                        ProcessLine(readBuf.Trim());
                        readBuf = "";
                    }
                    else if (b != '\r' && b >= 0)
                    {
                        readBuf += (char)b;
                    }
                }
                catch (TimeoutException) { /* normal — no data this tick */ }
                catch (InvalidOperationException) { break; /* port closed */ }
            }

            try { _port.Close(); } catch { }
            _port = null;

            Console.WriteLine($"[Nano:{_config.Id}] Session ended");
        }

        private void SafeWrite(string line)
        {
            try { _port?.WriteLine(line); }
            catch (Exception ex) { Console.WriteLine($"[Nano:{_config.Id}] Write error: {ex.Message}"); }
        }

        // ── Protocol parsing ───────────────────────────────────────────────────

        private void ProcessLine(string line)
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
                        if (_pinStates.TryGetValue(pin, out var state))
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
                    if (_pinStates.TryGetValue(pin, out var state))
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
            lock (_lock)
            {
                _commandQueue.Enqueue($"SET:{pin},{(value ? 1 : 0)}");
            }
            // Optimistically update tracked state
            if (_pinStates.TryGetValue(pin, out var state))
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

        public List<NanoPinState> GetPinStates() =>
            _pinStates.Values.ToList();

        public NanoPinState? GetPinState(int pin) =>
            _pinStates.TryGetValue(pin, out var s) ? s : null;

        /// <summary>
        /// Adds or updates the tracked state for a pin without sending a firmware
        /// command — call ConfigurePin separately when the device is connected.
        /// </summary>
        public void UpsertPinState(int pin, PinType type, string name, int pixelCount)
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

        /// <summary>Removes a pin from live state tracking.</summary>
        public void RemovePinState(int pin) => _pinStates.Remove(pin);
    }
}
