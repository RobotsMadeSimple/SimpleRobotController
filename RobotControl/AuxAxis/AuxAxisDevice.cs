using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using Controller.RobotControl.Serial;

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

        // Tracks whether motor drivers are currently enabled.
        private volatile bool _motorEnabled = true;
        public bool MotorEnabled => _motorEnabled;

        // The operator's chosen enable state. Persists across reconnects so a
        // transient reset restores their intent instead of the Arduino default.
        private volatile bool _desiredEnabled = true;

        // Port that last answered our ID probe — tried first on reconnect.
        private string? _lastGoodPort;

        // Per-axis position tracking (in steps, updated when DONE is received).
        private readonly long[]       _position     = new long[4];
        private readonly bool[]       _dirCcw       = new bool[4];
        private readonly Queue<long>[] _pendingMoves = new[] {
            new Queue<long>(), new Queue<long>(), new Queue<long>(), new Queue<long>()
        };

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
                SerialPort? port = null;
                try
                {
                    port = ScanForDevice();
                    if (port == null) { Thread.Sleep(3000); continue; }
                    RunSession(port);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[AuxAxis:{Id}] Error: {ex.Message}");
                }
                finally
                {
                    if (port != null) SerialPortRegistry.Release(port.PortName, Id);
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

        /// <summary>
        /// Candidate ports in probe order. Skips built-in serial ports on Linux
        /// (only USB adapters carry our devices) and tries the last known-good
        /// port first so reconnects are near-instant.
        /// </summary>
        private IEnumerable<string> CandidatePorts()
        {
            var all = SerialPort.GetPortNames();
            IEnumerable<string> ports = all;

            if (!OperatingSystem.IsWindows())
            {
                var usb = all.Where(p => p.Contains("ttyUSB") || p.Contains("ttyACM")).ToArray();
                if (usb.Length > 0) ports = usb;
            }

            if (_lastGoodPort != null)
                ports = ports.OrderByDescending(p => p == _lastGoodPort);

            return ports;
        }

        /// <summary>
        /// Probes candidate ports for our device and returns the OPEN port on a
        /// match (so the session doesn't have to re-open and reset the Arduino a
        /// second time). Returns null if not found.
        /// </summary>
        private SerialPort? ScanForDevice()
        {
            foreach (string portName in CandidatePorts())
            {
                if (!_running) return null;
                // Don't touch a port another device already owns — opening it
                // would DTR-reset that device's Arduino.
                if (SerialPortRegistry.IsClaimedByOther(portName, Id)) continue;

                SerialPort? probe = null;
                bool matched = false;
                try
                {
                    // Serialize probes on this port so two scanners can't open it at once.
                    lock (SerialPortRegistry.LockFor(portName))
                    {
                        if (SerialPortRegistry.IsClaimedByOther(portName, Id)) continue;

                        probe = new SerialPort(portName, 115200)
                        {
                            ReadTimeout  = 2500,
                            WriteTimeout = 1000,
                            NewLine      = "\n",
                        };
                        probe.Open();
                        // The Arduino resets when the port opens; its bootloader can
                        // take up to ~2s before the sketch runs and can answer ID?.
                        Thread.Sleep(2000);
                        probe.DiscardInBuffer();  // flush boot "RDY" (Linux DTR reset sends it)

                        // Ask a couple of times in case the first query lands just as
                        // the sketch is starting up.
                        for (int attempt = 0; attempt < 3 && _running; attempt++)
                        {
                            probe.WriteLine("ID?");
                            Thread.Sleep(150);

                            string line;
                            try { line = probe.ReadLine().Trim(); }
                            catch (TimeoutException) { continue; }

                            if (line == $"ID:{_config.Id}")
                            {
                                matched       = true;
                                _lastGoodPort = portName;
                                probe.ReadTimeout = 100; // fast servicing during the session
                                SerialPortRegistry.Claim(portName, Id);
                                return probe;
                            }
                        }
                    }
                }
                catch { }
                finally
                {
                    if (!matched) { try { probe?.Close(); probe?.Dispose(); } catch { } }
                }
            }
            return null;
        }

        private void RunSession(SerialPort port)
        {
            _port = port;
            string portName = port.PortName;

            PortName  = portName;
            Connected = true;
            ConnectionChanged?.Invoke(true);
            Console.WriteLine($"[AuxAxis:{Id}] Connected on {portName}");

            // Restore the operator's chosen enable state rather than forcing it on.
            _motorEnabled = _desiredEnabled;
            SafeWrite(_desiredEnabled ? "E:1" : "E:0");

            string readBuf = "";
            while (_running && port.IsOpen)
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
                    int b = port.ReadByte();
                    if (b == '\n')
                    {
                        ProcessLine(readBuf.Trim());
                        readBuf = "";
                    }
                    else if (b != '\r' && b >= 0)
                        readBuf += (char)b;
                }
                catch (TimeoutException) { }
                catch (InvalidOperationException) { break; } // port closed
                catch (IOException)             { break; }   // USB unplugged / device gone
            }

            try { port.Close(); } catch { }
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
                if ((uint)axis < 4)
                {
                    lock (_lock)
                    {
                        if (_pendingMoves[axis].Count > 0)
                            _position[axis] += _pendingMoves[axis].Dequeue();
                    }
                }
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
            bool ccw = (uint)axis < 4 && _dirCcw[axis];
            long signedSteps = (ccw ? -1L : 1L) * steps;
            lock (_lock)
            {
                if ((uint)axis < 4) _pendingMoves[axis].Enqueue(signedSteps);
                _commandQueue.Enqueue($"M:{axis},{steps},{Math.Max(1, velocityHz)},{Math.Max(1, accelHz)},{Math.Max(1, decelHz)}");
            }
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
            if ((uint)axis < 4) _dirCcw[axis] = ccw;
            lock (_lock) _commandQueue.Enqueue($"D:{axis},{(ccw ? 1 : 0)}");
        }

        /// <summary>Enable or disable all stepper drivers.</summary>
        public void Enable(bool enable)
        {
            _desiredEnabled = enable;  // remembered across reconnects
            _motorEnabled   = enable;
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
                foreach (var q in _pendingMoves) q.Clear();
            }
        }

        /// <summary>Returns the tracked step position for an axis (updated on each DONE event).</summary>
        public long GetPosition(int axis)
        {
            lock (_lock) return (uint)axis < 4 ? _position[axis] : 0;
        }

        /// <summary>Reset the tracked position for an axis to zero (e.g. after homing).</summary>
        public void ZeroPosition(int axis)
        {
            lock (_lock) { if ((uint)axis < 4) _position[axis] = 0; }
        }
    }
}
