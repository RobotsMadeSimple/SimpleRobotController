using System.Collections.Generic;
using System.IO.Ports;

namespace Controller.RobotControl.Serial
{
    /// <summary>
    /// A user-defined generic serial device: one fixed, user-chosen port opened at a configurable
    /// baud/parity/stop/data framing with no handshake. Reuses <see cref="SerialLineDevice"/>'s
    /// connection/reconnect loop and newline framing (frames rx on '\n', trims a trailing '\r'),
    /// but bypasses the "ID?" auto-discovery via the fixed-port hooks. Keeps a rolling buffer of the
    /// most recent tx/rx lines and lets callers send a line (optionally waiting for reply lines).
    /// </summary>
    public sealed class GenericSerialDevice : SerialLineDevice
    {
        private const int RecentCap = 100;

        private sealed record Entry(long Seq, string Dir, string Text);

        private readonly object _recentLock = new();
        private readonly LinkedList<Entry> _recent = new();
        private long _seq;

        private volatile bool _wantOpen;
        private SerialDeviceConfig _config;

        public GenericSerialDevice(SerialDeviceConfig config) : base(config.Id, config.Name)
        {
            _config   = config;
            _wantOpen = config.AutoConnect;
        }

        // ── SerialLineDevice knobs (unused by the fixed-port path, but required) ───
        protected override string DeviceTag                  => "Serial";
        protected override string ThreadNamePrefix            => "SerialDevice";
        protected override int    ProbeAttempts               => 0;
        protected override int    ProbeResponseDelayMs        => 0;
        protected override bool   KeepProbePortOpenForSession => false;
        public    override string SketchName                  => "";

        // ── Fixed-port hooks ──────────────────────────────────────────────────────
        protected override bool    UsesIdProbe     => false;
        protected override string? FixedPortName   => string.IsNullOrWhiteSpace(_config.PortName) ? null : _config.PortName;
        protected override int     ConfiguredBaud  => _config.BaudRate;
        protected override bool    WantConnection  => _wantOpen;
        // A fixed port has no boot window to wait out — retry its one port more snappily than a probe.
        protected override int     ReconnectDelayMs => 1000;

        protected override SerialPort CreateFixedPort(string portName) =>
            new(portName, _config.BaudRate, _config.Parity, _config.DataBits, _config.StopBits)
            {
                ReadTimeout  = SessionReadTimeoutMs,
                WriteTimeout = SessionWriteTimeoutMs,
                NewLine      = "\n",
                Handshake    = Handshake.None,
                DtrEnable    = false,
                RtsEnable    = false,
            };

        // ── Session hooks ─────────────────────────────────────────────────────────
        protected override void OnSessionStarted(SerialPort port) { /* no handshake */ }

        protected override void OnLine(string line) => AddRecent("rx", line);

        protected override void OnSessionEnded() { }

        /// <summary>Applies the configured line ending to each queued write.</summary>
        protected override void WriteOutbound(string line) =>
            WriteRaw(line + SerialDeviceConfig.LineEnding(_config.WriteLineEnding));

        // ── Public control ──────────────────────────────────────────────────────

        /// <summary>Manually open (connect) the port — the reconnect loop opens it on its next pass.</summary>
        public void Open() => _wantOpen = true;

        /// <summary>Manually close (disconnect) the port and stop reconnecting until <see cref="Open"/>.</summary>
        public void Close()
        {
            _wantOpen = false;
            DropConnection();
        }

        /// <summary>Replace the live config (name/framing/commands) and reopen if the connection params
        /// may have changed, so the next session picks up the new framing/port.</summary>
        public void ApplyConfig(SerialDeviceConfig config)
        {
            _config = config;
            // Drop the current session so a new port/baud/framing takes effect; the reconnect loop
            // re-reads FixedPortName/ConfiguredBaud and reopens (only if a connection is still wanted).
            DropConnection();
        }

        /// <summary>Queue a line to send (written on the next read-loop tick) and record it as tx.</summary>
        public void SendLine(string text)
        {
            text ??= "";
            lock (_lock) _commandQueue.Enqueue(text);
            AddRecent("tx", text);
        }

        /// <summary>Current rx sequence mark — pass to <see cref="RxLinesSince"/> to collect replies
        /// that arrive after this point.</summary>
        public long RxSeq { get { lock (_recentLock) return _seq; } }

        /// <summary>Rx line texts received after <paramref name="sinceSeq"/> (oldest first).</summary>
        public IReadOnlyList<string> RxLinesSince(long sinceSeq)
        {
            lock (_recentLock)
                return _recent.Where(e => e.Dir == "rx" && e.Seq > sinceSeq).Select(e => e.Text).ToList();
        }

        /// <summary>Send a line and block up to <paramref name="timeoutMs"/> for reply line(s). Returns
        /// the reply lines (possibly empty on timeout or when disconnected).</summary>
        public IReadOnlyList<string> SendAndWait(string text, int timeoutMs)
        {
            long mark = RxSeq;
            SendLine(text);
            long deadline = Environment.TickCount64 + Math.Max(0, timeoutMs);
            while (true)
            {
                var lines = RxLinesSince(mark);
                if (lines.Count > 0)
                {
                    // Brief grace so trailing lines of a multi-line reply are included.
                    Thread.Sleep(30);
                    return RxLinesSince(mark);
                }
                if (Environment.TickCount64 >= deadline) return RxLinesSince(mark);
                Thread.Sleep(5);
            }
        }

        /// <summary>Build the wire state for this device.</summary>
        public SerialDeviceState GetState()
        {
            List<SerialLineEntry> recent;
            lock (_recentLock)
                recent = _recent.Select(e => new SerialLineEntry { Dir = e.Dir, Text = e.Text }).ToList();

            return new SerialDeviceState
            {
                Id              = _config.Id,
                Name            = _config.Name,
                PortName        = _config.PortName,
                BaudRate        = _config.BaudRate,
                DataBits        = _config.DataBits,
                Parity          = _config.Parity,
                StopBits        = _config.StopBits,
                WriteLineEnding = _config.WriteLineEnding,
                AutoConnect     = _config.AutoConnect,
                Connected       = Connected,
                RecentLines     = recent,
                Commands    = _config.Commands.Select(c => new SerialCommandDef
                {
                    Id = c.Id, Name = c.Name, Payload = c.Payload,
                    ExpectReply = c.ExpectReply, ReplyTimeoutMs = c.ReplyTimeoutMs,
                }).ToList(),
            };
        }

        /// <summary>Find a saved command by id.</summary>
        public SerialCommandDef? GetCommand(string? commandId) =>
            string.IsNullOrEmpty(commandId) ? null : _config.Commands.FirstOrDefault(c => c.Id == commandId);

        private void AddRecent(string dir, string text)
        {
            lock (_recentLock)
            {
                _recent.AddLast(new Entry(++_seq, dir, text));
                while (_recent.Count > RecentCap) _recent.RemoveFirst();
            }
        }
    }
}
