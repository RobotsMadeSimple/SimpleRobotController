namespace Controller.RobotControl.UsbRelay
{
    /// <summary>
    /// Manages the lifecycle of a USB relay board, including auto-reconnect.
    /// Runs a background polling thread that tries to (re)open the device every
    /// 3 seconds whenever it is disconnected.
    /// </summary>
    public sealed class UsbRelayManager : IDisposable
    {
        private UsbRelayDevice? _device;
        private readonly Lock   _lock = new();
        private readonly CancellationTokenSource _cts = new();
        private Thread? _reconnectThread;

        private RelayConfig _config;

        // Cached relay state so the frequent status broadcast can report live
        // relay state without a USB read on every tick. Updated on every Set and
        // refreshed from the board by the poll loop. Guarded by _lock.
        private readonly bool[] _cached = new bool[UsbRelayDevice.RelayCount];

        /// <summary>Number of relay channels on the board.</summary>
        public int RelayCount => UsbRelayDevice.RelayCount;

        /// <summary>True when the board is currently open and responsive.</summary>
        public bool IsConnected
        {
            get { lock (_lock) return _device != null; }
        }

        public UsbRelayManager()
        {
            _config = RelayConfigService.Load();
        }

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            _reconnectThread = new Thread(ReconnectLoop)
            {
                IsBackground = true,
                Name         = "UsbRelayReconnect",
            };
            _reconnectThread.Start();
        }

        public void Stop()
        {
            _cts.Cancel();
            lock (_lock)
            {
                _device?.Dispose();
                _device = null;
            }
        }

        public void Dispose() => Stop();

        // ── Public API ─────────────────────────────────────────────────────────

        /// <summary>Turn a relay on or off. relay: 1–4. No-op if disconnected.</summary>
        public void SetRelay(int relay, bool on)
        {
            lock (_lock)
            {
                if (_device == null) return;
                try
                {
                    _device.Set(relay, on);
                    if (relay >= 1 && relay <= RelayCount) _cached[relay - 1] = on;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[UsbRelay] Write error: {ex.Message} — marking disconnected.");
                    _device.Dispose();
                    _device = null;
                }
            }
        }

        /// <summary>
        /// Returns current relay states as a bool array indexed [0] = relay 1 … [3] = relay 4.
        /// Returns null if disconnected.
        /// </summary>
        public bool[]? GetRelayStates()
        {
            lock (_lock)
            {
                // Return the cached state (kept fresh by SetRelay and the poll
                // loop) rather than a blocking USB read — this is called on every
                // status broadcast.
                if (_device == null) return null;
                return (bool[])_cached.Clone();
            }
        }

        /// <summary>Returns the board serial string, or null if disconnected.</summary>
        public string? GetSerial()
        {
            lock (_lock) return _device?.Serial;
        }

        /// <summary>Returns the display names for all 4 relay channels.</summary>
        public string[] GetRelayNames() => _config.RelayNames.ToArray();

        /// <summary>Renames a relay channel and persists to relay-config.json. relay: 1–4.</summary>
        public void RenameRelay(int relay, string name)
        {
            if (relay < 1 || relay > RelayCount) return;
            _config.RelayNames[relay - 1] = name;
            RelayConfigService.Save(_config);
        }

        // ── Reconnect loop ─────────────────────────────────────────────────────

        private void ReconnectLoop()
        {
            while (!_cts.Token.IsCancellationRequested)
            {
                bool needsConnect;
                lock (_lock) needsConnect = _device == null;

                if (needsConnect)
                {
                    // Open outside the lock — it can block for a moment.
                    var dev = UsbRelayDevice.TryOpen();
                    if (dev != null)
                        lock (_lock) _device = dev;
                }

                // While connected, refresh the cached state from the board so the
                // status broadcast reflects relays changed by a running program
                // (or anything else) without a USB read per broadcast.
                lock (_lock)
                {
                    if (_device != null)
                    {
                        try
                        {
                            int mask = _device.ReadBitmask();
                            for (int i = 0; i < RelayCount; i++)
                                _cached[i] = UsbRelayDevice.IsOn(mask, i + 1);
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"[UsbRelay] Read error: {ex.Message} — marking disconnected.");
                            _device.Dispose();
                            _device = null;
                        }
                    }
                }

                // Poll ~1s while connected; back off to 3s while searching.
                bool connected;
                lock (_lock) connected = _device != null;
                try { Task.Delay(connected ? 1000 : 3000, _cts.Token).Wait(); }
                catch (OperationCanceledException) { break; }
            }
        }
    }
}
