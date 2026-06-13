using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Threading;

namespace Controller.RobotControl.AuxAxis
{
    /// <summary>
    /// Serial connection to a single AuxStepperDriver (Arduino Uno + CNC Shield V3/V4).
    /// Mirrors the NanoDevice pattern: auto-discovers by ID, runs on its own background thread.
    ///
    /// Protocol: ASCII newline-terminated at 115200 baud.
    ///   HOST -> ARDUINO:  E:1  D:axis,dir  M:axis,steps,vel,acc,dec  C:axis,vel,acc  S:axis,dec  X
    ///   ARDUINO -> HOST:  RDY  OK  DONE:axis  ERR
    /// </summary>
    public sealed class AuxAxisDevice : IDisposable
    {
        private readonly AuxAxisConfig _config;
        private SerialPort?  _port;
        private Thread?      _thread;
        private volatile bool _running;

        private readonly object        _lock         = new();
        private readonly Queue<string> _commandQueue = new();

        public string  Id        => _config.Id;
        public string  Name      => _config.Name;
        public string? PortName  { get; private set; }
        public bool    Connected { get; private set; }

        // Counts M commands sent but not yet DONE — drives IsMoving.
        private volatile int _inflightCount = 0;

        /// <summary>True while an indexed M move is executing on the Arduino.</summary>
        public bool IsMoving => _inflightCount > 0;

        /// <summary>Fired on the device thread when connection state changes.</summary>
        public event Action<bool>? ConnectionChanged;

        /// <summary>Fired when the Arduino reports DONE for a completed M move.</summary>
        public event Action<int>? MoveDone;

        public AuxAxisDevice(AuxAxisConfig config) => _config = config;

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            _running = true;
            _thread  = new Thread(ConnectionLoop)
            {
                IsBackground = true,
                Name         = $"AuxAxis-{_config.Id}"
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
                    if (port == null) { Thread.Sleep(3000); continue; }
                    RunSession(port);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[AuxAxis:{Id}] Error: {ex.Message}");
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

        private string? ScanForDevice()
        {
            string[] ports = SerialPort.GetPortNames();
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
                    Thread.Sleep(2000); // wait for Arduino reset

                    probe.WriteLine("ID?");
                    Thread.Sleep(100);

                    string line = probe.ReadLine().Trim();
                    if (line == $"ID:{_config.Id}")
                        return portName;
                }
                catch { }
            }
            return null;
        }

        private void RunSession(string portName)
        {
            _port = new SerialPort(portName, 115200)
            {
                ReadTimeout  = 100,
                WriteTimeout = 2000,
                NewLine      = "\n",
            };
            _port.Open();
            Thread.Sleep(2000);

            PortName  = portName;
            Connected = true;
            ConnectionChanged?.Invoke(true);
            Console.WriteLine($"[AuxAxis:{Id}] Connected on {portName}");

            SafeWrite("E:1");

            string readBuf = "";
            while (_running && _port.IsOpen)
            {
                lock (_lock)
                {
                    while (_commandQueue.Count > 0)
                    {
                        var cmd = _commandQueue.Dequeue();
                        SafeWrite(cmd);
                        if (cmd[0] == 'M') _inflightCount++;
                    }
                }

                try
                {
                    int b = _port.ReadByte();
                    if (b == '\n')
                    {
                        ProcessLine(readBuf.Trim());
                        readBuf = "";
                    }
                    else if (b != '\r' && b >= 0)
                        readBuf += (char)b;
                }
                catch (TimeoutException) { }
                catch (InvalidOperationException) { break; }
            }

            try { _port.Close(); } catch { }
            _port = null;
            Console.WriteLine($"[AuxAxis:{Id}] Disconnected");
        }

        private void SafeWrite(string line)
        {
            try { _port?.WriteLine(line); }
            catch (Exception ex) { Console.WriteLine($"[AuxAxis:{Id}] Write error: {ex.Message}"); }
        }

        private void ProcessLine(string line)
        {
            if (string.IsNullOrEmpty(line)) return;
            if (line.StartsWith("DONE:") && int.TryParse(line.Substring(5), out int axis))
            {
                if (_inflightCount > 0) _inflightCount--;
                MoveDone?.Invoke(axis);
            }
        }

        // ── Public command API — all thread-safe ──────────────────────────────

        /// <summary>
        /// Start an indexed move: Arduino runs the full trapezoidal profile and reports DONE when complete.
        /// </summary>
        public void StartMove(int axis, long steps, int velocityHz, int accelHz, int decelHz)
        {
            if (steps <= 0) return;
            lock (_lock)
                _commandQueue.Enqueue($"M:{axis},{steps},{Math.Max(1, velocityHz)},{Math.Max(1, accelHz)},{Math.Max(1, decelHz)}");
        }

        /// <summary>Start continuous stepping on axis, ramping up to velocityHz at accelHz.</summary>
        public void SetContinuous(int axis, int velocityHz, int accelHz)
        {
            lock (_lock)
                _commandQueue.Enqueue($"C:{axis},{Math.Max(1, velocityHz)},{Math.Max(1, accelHz)}");
        }

        /// <summary>Decelerate axis to rest at the given rate.</summary>
        public void StopSmooth(int axis, int decelHz)
        {
            lock (_lock)
                _commandQueue.Enqueue($"S:{axis},{Math.Max(1, decelHz)}");
        }

        /// <summary>Set direction for an axis (false=CW, true=CCW).</summary>
        public void SetDirection(int axis, bool ccw)
        {
            lock (_lock) _commandQueue.Enqueue($"D:{axis},{(ccw ? 1 : 0)}");
        }

        /// <summary>Enable or disable all stepper drivers.</summary>
        public void Enable(bool enable)
        {
            lock (_lock) _commandQueue.Enqueue($"E:{(enable ? 1 : 0)}");
        }

        /// <summary>Emergency stop — halt all axes immediately, clear queued commands.</summary>
        public void StopAll()
        {
            lock (_lock)
            {
                _commandQueue.Clear();
                _inflightCount = 0;
                _commandQueue.Enqueue("X");
            }
        }
    }
}
