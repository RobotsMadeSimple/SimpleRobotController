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

        /// <summary>Number of relay channels on the board.</summary>
        public int RelayCount => UsbRelayDevice.RelayCount;

        /// <summary>True when the board is currently open and responsive.</summary>
        public bool IsConnected
        {
            get { lock (_lock) return _device != null; }
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
                if (_device == null) return null;
                try
                {
                    int mask   = _device.ReadBitmask();
                    var states = new bool[RelayCount];
                    for (int i = 0; i < RelayCount; i++)
                        states[i] = UsbRelayDevice.IsOn(mask, i + 1);
                    return states;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[UsbRelay] Read error: {ex.Message} — marking disconnected.");
                    _device.Dispose();
                    _device = null;
                    return null;
                }
            }
        }

        /// <summary>Returns the board serial string, or null if disconnected.</summary>
        public string? GetSerial()
        {
            lock (_lock) return _device?.Serial;
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
                    var dev = UsbRelayDevice.TryOpen();
                    if (dev != null)
                        lock (_lock) _device = dev;
                }

                try { Task.Delay(3000, _cts.Token).Wait(); }
                catch (OperationCanceledException) { break; }
            }
        }
    }
}
