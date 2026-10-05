using System.IO;
using System.IO.Ports;
using System.Text;

namespace Controller.RobotControl.Serial
{
    /// <summary>
    /// Shared plumbing for a device that talks ASCII newline-terminated lines over a
    /// serial port and is discovered by probing candidate ports with "ID?" until one
    /// answers "ID:&lt;id&gt;". Both <c>NanoDevice</c> and <c>AuxAxisDevice</c> are
    /// this shape: auto-discover on a background thread, reconnect every 3s while
    /// disconnected, drain an outbound command queue every read-loop tick, and parse
    /// inbound lines with a device-specific protocol.
    ///
    /// What differs between devices is expressed through the abstract/virtual members
    /// below rather than subclasses re-implementing the loop:
    ///   - <see cref="ProbeAttempts"/> / <see cref="ProbeResponseDelayMs"/> — how hard
    ///     to retry the "ID?" probe on a candidate port.
    ///   - <see cref="KeepProbePortOpenForSession"/> — AuxAxisDevice keeps the probe's
    ///     SerialPort open and reuses it for the session (avoids a second DTR reset);
    ///     NanoDevice closes the probe and reopens a fresh SerialPort for the session
    ///     (its original behaviour, preserved as-is).
    ///   - <see cref="CatchIOException"/> — AuxAxisDevice treats a mid-session
    ///     IOException (USB unplugged) as a normal disconnect; NanoDevice does not
    ///     catch it here, so it propagates to the connection loop's catch-all instead
    ///     (this was NanoDevice's original behaviour and is preserved, not "fixed").
    ///   - <see cref="OnSessionStarted"/> / <see cref="OnLine"/> / <see cref="OnSessionEnded"/>
    ///     — device-specific handshake, protocol parsing, and disconnect cleanup.
    /// </summary>
    public abstract class SerialLineDevice : IDisposable
    {
        // ── Shared timing constants — identical across every current subclass ──────
        protected const int BaudRate             = 115200;
        protected const int ProbeReadTimeoutMs    = 2500;
        protected const int ProbeWriteTimeoutMs   = 1000;
        protected const int PostOpenResetDelayMs  = 2000; // Arduino reboots on DTR toggle when a port opens; wait for boot.
        protected const int SessionReadTimeoutMs  = 100;  // short so the write queue is serviced promptly
        protected const int ReconnectDelayMs      = 3000;

        // ── Per-device knobs ────────────────────────────────────────────────────
        /// <summary>Log prefix tag, e.g. "Nano" / "AuxAxis" — lines read "[{DeviceTag}:{Id}] ...".</summary>
        protected abstract string DeviceTag { get; }

        /// <summary>Background thread Name prefix (kept distinct from <see cref="DeviceTag"/> to match original naming).</summary>
        protected abstract string ThreadNamePrefix { get; }

        /// <summary>Number of "ID?" probe attempts per candidate port before giving up on it.</summary>
        protected abstract int ProbeAttempts { get; }

        /// <summary>Delay after writing "ID?" before reading the response.</summary>
        protected abstract int ProbeResponseDelayMs { get; }

        /// <summary>
        /// True to keep the probe's already-open SerialPort for the session (AuxAxisDevice —
        /// avoids a second reset). False to close the probe and open a fresh SerialPort for
        /// the session (NanoDevice's original behaviour).
        /// </summary>
        protected abstract bool KeepProbePortOpenForSession { get; }

        /// <summary>
        /// True to treat a mid-session IOException (e.g. USB unplugged) as a normal
        /// disconnect. False leaves it uncaught here — it propagates to the connection
        /// loop's catch-all, matching NanoDevice's original (unhandled) behaviour.
        /// </summary>
        protected virtual bool CatchIOException => false;

        /// <summary>Verb used in the connection loop's unhandled-error log line.</summary>
        protected virtual string UnhandledErrorLogVerb => "Unhandled error";

        /// <summary>WriteTimeout applied when NanoDevice-style subclasses reopen a fresh session port.</summary>
        protected virtual int SessionWriteTimeoutMs => 2000;

        // ── Identity / public state ────────────────────────────────────────────
        public string  Id        { get; }
        public string  Name      { get; }
        public string? PortName  { get; private set; }
        public bool    Connected { get; private set; }

        /// <summary>Fired on the device thread when the connection is established or lost.</summary>
        public event Action<bool>? ConnectionChanged;

        protected string LogPrefix => $"{DeviceTag}:{Id}";

        // ── Shared mutable state ───────────────────────────────────────────────
        protected readonly object        _lock         = new();
        protected readonly Queue<string> _commandQueue = new();

        private SerialPort? _port;
        private Thread?     _thread;
        private volatile bool _running;

        // Port that last answered our ID probe — tried first on reconnect.
        private string? _lastGoodPort;

        protected SerialLineDevice(string id, string name)
        {
            Id   = id;
            Name = name;
        }

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            _running = true;
            _thread  = new Thread(ConnectionLoop)
            {
                IsBackground = true,
                Name         = $"{ThreadNamePrefix}-{Id}"
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
                    if (port == null) { Thread.Sleep(ReconnectDelayMs); continue; }
                    RunSession(port);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[{LogPrefix}] {UnhandledErrorLogVerb}: {ex.Message}");
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

                if (_running) Thread.Sleep(ReconnectDelayMs);
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
        /// Probes candidate ports for our device. Returns an OPEN SerialPort ready for
        /// the session on a match (either the reused probe port, or a freshly reopened
        /// one — see <see cref="KeepProbePortOpenForSession"/>), or null if not found.
        /// </summary>
        private SerialPort? ScanForDevice()
        {
            foreach (string portName in CandidatePorts())
            {
                if (!_running) return null;
                // Skip ports another device already owns — opening one would
                // DTR-reset that device's Arduino.
                if (SerialPortRegistry.IsClaimedByOther(portName, Id)) continue;

                SerialPort? probe = null;
                bool matched = false;
                try
                {
                    // Serialize probes on this port so two scanners can't open it at once.
                    lock (SerialPortRegistry.LockFor(portName))
                    {
                        if (SerialPortRegistry.IsClaimedByOther(portName, Id)) continue;

                        probe = new SerialPort(portName, BaudRate)
                        {
                            ReadTimeout  = ProbeReadTimeoutMs,
                            WriteTimeout = ProbeWriteTimeoutMs,
                            NewLine      = "\n",
                        };
                        probe.Open();

                        // Arduino resets on DTR toggle when the port opens — wait for boot
                        Thread.Sleep(PostOpenResetDelayMs);
                        probe.DiscardInBuffer();

                        for (int attempt = 0; attempt < ProbeAttempts && _running; attempt++)
                        {
                            probe.WriteLine("ID?");
                            Thread.Sleep(ProbeResponseDelayMs);

                            string line;
                            try { line = probe.ReadLine().Trim(); }
                            catch (TimeoutException) { continue; }

                            if (line == $"ID:{Id}")
                            {
                                matched       = true;
                                _lastGoodPort = portName;
                                SerialPortRegistry.Claim(portName, Id);

                                if (KeepProbePortOpenForSession)
                                {
                                    probe.ReadTimeout = SessionReadTimeoutMs; // fast servicing during the session
                                    return probe;
                                }

                                try { probe.Close(); probe.Dispose(); } catch { }
                                var session = OpenSessionPort(portName);
                                Thread.Sleep(PostOpenResetDelayMs); // second reset delay after the real open
                                return session;
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

        /// <summary>Opens a fresh session SerialPort for devices that don't keep the probe port open.</summary>
        protected virtual SerialPort OpenSessionPort(string portName)
        {
            var port = new SerialPort(portName, BaudRate)
            {
                ReadTimeout  = SessionReadTimeoutMs,
                WriteTimeout = SessionWriteTimeoutMs,
                NewLine      = "\n",
            };
            port.Open();
            return port;
        }

        /// <summary>Runs the connected session: handshake, then the read/write loop until disconnect.</summary>
        private void RunSession(SerialPort port)
        {
            _port     = port;
            PortName  = port.PortName;
            Connected = true;
            ConnectionChanged?.Invoke(true);

            OnSessionStarted(port);

            var readBuf = new StringBuilder();

            while (_running && port.IsOpen)
            {
                // Drain outbound command queue
                lock (_lock)
                {
                    while (_commandQueue.Count > 0)
                        SafeWrite(_commandQueue.Dequeue());
                }

                // Read one byte; ReadTimeout=SessionReadTimeoutMs means we spin back to flush the queue
                try
                {
                    int b = port.ReadByte();
                    if (b == '\n')
                    {
                        OnLine(readBuf.ToString().Trim());
                        readBuf.Clear();
                    }
                    else if (b != '\r' && b >= 0)
                    {
                        readBuf.Append((char)b);
                    }
                }
                catch (TimeoutException) { /* normal — no data this tick */ }
                catch (InvalidOperationException) { break; /* port closed */ }
                catch (IOException) when (CatchIOException) { break; /* USB unplugged / device gone */ }
            }

            try { _port.Close(); } catch { }
            _port = null;

            OnSessionEnded();
        }

        protected void SafeWrite(string line)
        {
            try { _port?.WriteLine(line); }
            catch (Exception ex) { Console.WriteLine($"[{LogPrefix}] Write error: {ex.Message}"); }
        }

        // ── Hooks for subclasses ───────────────────────────────────────────────

        /// <summary>Called once the session port is open and Connected/PortName/ConnectionChanged are set. Send the handshake here.</summary>
        protected abstract void OnSessionStarted(SerialPort port);

        /// <summary>Called for each newline-terminated, trimmed line received during the session.</summary>
        protected abstract void OnLine(string line);

        /// <summary>Called after the session port is closed. Do device-specific cleanup (and logging) here.</summary>
        protected abstract void OnSessionEnded();
    }
}
