using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Threading;
using Controller.RobotControl.Io;
using Controller.RobotControl.Serial;

namespace Controller.RobotControl.AuxAxis
{
    /// <summary>
    /// Serial connection to a single AuxStepperDriver (Arduino Uno + CNC Shield V3/V4).
    /// Mirrors the NanoDevice pattern: auto-discovers by ID, runs on its own background thread.
    ///
    /// Protocol: ASCII newline-terminated at 115200 baud.
    ///   HOST -> ARDUINO:  E:1  D:axis,dir  M:axis,steps,vel,acc,dec  C:axis,vel,acc  S:axis,dec  X
    ///                     Z:axis  POS?  PS:periodMs  (cap pos)   CFG:pin,L  (cap latch)
    ///   ARDUINO -> HOST:  RDY  OK  DONE:axis  ERR  POS:ms,p0,p1,p2,p3  LAT:pin,val,ms,p0,p1,p2,p3
    ///
    /// Position tracking: with firmware 1.2.0+ (cap <c>pos</c>) the board's signed step counters are
    /// authoritative — the tracked position is <c>board counter + offset</c>, the offset absorbing
    /// SetPosition/Zero and the gripper soft-bound clamp. The board streams POS lines (always at
    /// least every <see cref="BaselineStreamMs"/>, faster on <see cref="SetPositionStreaming"/>) and
    /// sends one whenever an axis comes to rest, before DONE. Older firmware falls back to dead
    /// reckoning: DONE deltas for M moves, plus the commanded continuous velocity integrated on the
    /// controller (see <see cref="SimStep"/>) — roughly right for a belt, never exact.
    /// </summary>
    public sealed class AuxAxisDevice : SerialLineDevice
    {
        private readonly AuxAxisConfig _config;

        // Counts M moves commanded but not yet DONE — drives IsMoving. Incremented
        // at enqueue time (StartMove), NOT when the command reaches the serial port
        // on the device thread, so IsMoving is true the instant a move is commanded.
        // Otherwise a caller that polls "is it moving?" right after StartMove (the
        // program executor's aux-wait) sees false and treats the not-yet-sent move
        // as already finished, skipping ahead before the motor even gets its command.
        // Written from the caller thread and the device thread, so guard every access
        // with Interlocked writes and Volatile reads.
        private int _inflightCount = 0;

        // Tracks whether motor drivers are currently enabled.
        private volatile bool _motorEnabled = true;
        public bool MotorEnabled => _motorEnabled;

        // The operator's chosen enable state. Persists across reconnects so a
        // transient reset restores their intent instead of the Arduino default.
        private volatile bool _desiredEnabled = true;

        // Per-axis position tracking (in steps, updated when DONE is received).
        private readonly long[]       _position     = new long[4];
        private readonly bool[]       _dirCcw       = new bool[4];
        private readonly Queue<long>[] _pendingMoves = new[] {
            new Queue<long>(), new Queue<long>(), new Queue<long>(), new Queue<long>()
        };

        // Per-axis soft position bounds (steps). A gripper pins these to its [closed, open]
        // jaw-gap range so over-driving a mechanical hard stop — which physically can't move
        // the jaws any further — never advances the dead-reckoned position past the limit.
        // Full long range = unbounded (a generic axis). Guarded by _lock.
        private readonly long[] _posMin = { long.MinValue, long.MinValue, long.MinValue, long.MinValue };
        private readonly long[] _posMax = { long.MaxValue, long.MaxValue, long.MaxValue, long.MaxValue };

        // ── Live board position (firmware cap "pos") ──────────────────────────
        // All guarded by _lock, which is never held across serial I/O, so the motion thread can
        // read a live position without waiting on the port. Board counters/millis are 32-bit and
        // unwrapped here into longs by adding each sample's (int)(new - last) difference.
        /// <summary>Stream period used whenever nobody asked for a faster one: keeps GetPosition
        /// live during continuous jogs on a pos-capable board for ~3% of the link.</summary>
        public const int BaselineStreamMs = 100;
        private const double VelWindowMs  = 100;   // velocity measured over this span of samples
        private const double ClockTauMs   = 400;   // drift-follow time constant of the clock map
        private const int    HistLen      = 16;

        private bool   _posSynced;                    // a POS arrived this session: board counts rule
        private uint   _lastRawMs;
        private long   _boardMs;                      // unwrapped board millis of the newest sample
        private readonly int[]    _lastRawPos = new int[4];
        private readonly long[]   _boardPos   = new long[4];   // unwrapped counters, newest sample
        private readonly long[]   _posOffset  = new long[4];   // tracked = board counter + offset
        private readonly double[] _liveVel    = new double[4]; // measured steps/s
        private double _clockOffsetMs;                // controller ms − board ms (lower envelope)
        private bool   _clockInit;
        private readonly long[]   _histMs  = new long[HistLen];
        private readonly long[,]  _histPos = new long[HistLen, 4];
        private int    _histCount, _histHead;         // ring of recent samples (head = next slot)
        private volatile int _streamPeriodMs;         // requested PS period (0 = baseline only)

        // ── Offline simulation ─────────────────────────────────────────────────
        // With no board connected, mirror the Arduino's effect locally so an axis can
        // still be jogged / moved and its dead-reckoned position advances in the UI
        // (e.g. to set up a gripper or dry-run a program without hardware present).
        // All sim state is guarded by _lock; _simPos is the running fractional position
        // that _position is rounded from while a sim motion is active, and is re-seeded
        // from _position at the start of each motion so SetPosition/Zero stay authoritative.
        // The continuous part also runs for a CONNECTED board without the "pos" capability:
        // that firmware doesn't count continuous steps, so the controller integrates the
        // commanded velocity (ramped like the firmware) to keep the position roughly right.
        private readonly double[] _simPos       = new double[4]; // running simulated position (steps)
        private readonly double[] _simVel       = new double[4]; // continuous signed steps/s now (0 = none)
        private readonly double[] _simVelTarget = new double[4]; // continuous signed target steps/s
        private readonly double[] _simAccel     = new double[4]; // ramp rate toward the target (steps/s²)
        private readonly double[] _simRate      = new double[4]; // indexed move speed (steps/s; >0 = active)
        private readonly long[]   _simTarget    = new long[4];   // indexed move target (valid when _simRate>0)
        private readonly Timer?   _simTimer;
        private long _simLastTicks;
        private const int SimTickMs = 50;

        // Per-axis "reference epoch": bumped by every move, jog, zero / set-position and by a
        // (re)connect or disconnect — anything that may change what the tracked position means.
        // A silo records the epoch when it homes; it is still homed while the epoch is unchanged.
        private readonly long[] _refEpoch = new long[4];

        /// <summary>The axis's reference epoch (see <c>_refEpoch</c>); 0 for an unknown axis.</summary>
        public long ReferenceEpoch(int axis) => (uint)axis < 4 ? Interlocked.Read(ref _refEpoch[axis]) : 0;

        private void BumpEpoch(int axis) { if ((uint)axis < 4) Interlocked.Increment(ref _refEpoch[axis]); }
        private void BumpAllEpochs() { for (int i = 0; i < 4; i++) Interlocked.Increment(ref _refEpoch[i]); }

        /// <summary>True while an indexed M move is executing on the Arduino.</summary>
        public bool IsMoving => Volatile.Read(ref _inflightCount) > 0;

        /// <summary>Fired when the Arduino reports DONE for a completed M move.</summary>
        public event Action<int>? MoveDone;

        /// <summary>
        /// Fired (on the device thread) for every input edge: a latch pin's <c>LAT:</c> (exact board
        /// positions + board time) and a plain input's <c>CHG:</c> (positions interpolated to the
        /// controller receive time), so any Uno input can serve as a part sensor.
        /// </summary>
        public event Action<AuxLatchEvent>? InputLatched;

        // ── SerialLineDevice knobs ─────────────────────────────────────────────
        protected override string DeviceTag                    => "AuxAxis";
        protected override string ThreadNamePrefix              => "AuxAxis";
        protected override int    ProbeAttempts                 => 3;
        protected override int    ProbeResponseDelayMs          => 150;
        protected override bool   KeepProbePortOpenForSession   => true;
        protected override bool   CatchIOException              => true;
        protected override string UnhandledErrorLogVerb          => "Error";
        // Short read timeout: queued commands (a belt start/stop) go out within ~10 ms instead of
        // waiting out a 100 ms idle read. While POS lines stream the loop never idles anyway.
        protected override int    SessionReadTimeoutMs          => 10;
        public    override string SketchName                    => "AuxStepperDriver";

        // Digital IO on the Uno's spare pins — same model/protocol as a Nano edge device
        // (Input/Output only; no HX711/NeoPixel). Guarded by the base _lock.
        private readonly Dictionary<int, IoPinState> _ioPins = new();

        public AuxAxisDevice(AuxAxisConfig config) : this(config, simTimer: true) { }

        /// <summary>Tests pass <paramref name="simTimer"/> = false and drive <see cref="SimStep"/> with
        /// their own clock instead of the 50 ms wall-clock timer.</summary>
        internal AuxAxisDevice(AuxAxisConfig config, bool simTimer) : base(config.Id, config.Name)
        {
            _config   = config;
            if (simTimer) _simTimer = new Timer(_ => SimStep(Stopwatch.GetTimestamp()), null, SimTickMs, SimTickMs);

            // Pre-populate IO pin state so callers can read it before the device connects.
            foreach (var p in config.IoPins)
                _ioPins[p.Pin] = new IoPinState
                {
                    Pin = p.Pin, Type = p.Type, Name = p.Name, Latch = p.Latch,
                    DeviceId = config.Id, DeviceName = config.Name, Value = false,
                };
        }

        private bool BoardHasPos   => Firmware.Capabilities?.Contains(FirmwareInfo.CapPos)   == true;
        private bool BoardHasLatch => Firmware.Capabilities?.Contains(FirmwareInfo.CapLatch) == true;

        /// <summary>The firmware CFG command for an IO pin. A latch input falls back to a plain input
        /// (CHG: edges) on firmware without the "latch" capability, which would ERR on type L.</summary>
        private string IoConfigCommand(int pin, PinType type, bool latch) =>
            type == PinType.Output    ? $"CFG:{pin},O"
            : latch && BoardHasLatch  ? $"CFG:{pin},L"
                                      : $"CFG:{pin},I";

        private int EffectiveStreamMs => _streamPeriodMs > 0 ? _streamPeriodMs : BaselineStreamMs;

        private static double TicksToMs(long ticks) => ticks * 1000.0 / Stopwatch.Frequency;

        // ── Session hooks ───────────────────────────────────────────────────────

        protected override void OnSessionStarted(SerialPort port)
        {
            // (The base class has already logged "Connected on <port> — firmware …".)

            // A real board now drives the axes — abandon any offline simulation so the
            // two don't both move the tracked position. The board just rebooted (DTR), so its
            // counters restart at 0: re-sync the offset from the first POS it sends.
            lock (_lock)
            {
                for (int i = 0; i < 4; i++) { _simVel[i] = 0; _simVelTarget[i] = 0; _simRate[i] = 0; }
                ResetLiveSync();
            }
            Interlocked.Exchange(ref _inflightCount, 0);
            BumpAllEpochs();

            // Restore the operator's chosen enable state rather than forcing it on.
            _motorEnabled = _desiredEnabled;
            SafeWrite(_desiredEnabled ? "E:1" : "E:0");

            // Push IO pin configuration, then request a full state snapshot.
            foreach (var p in _config.IoPins)
            {
                SafeWrite(IoConfigCommand(p.Pin, p.Type, p.Latch));
                Thread.Sleep(20);
            }
            if (_config.IoPins.Count > 0) SafeWrite("GET");

            // Position stream (re-sent every session — the board boots with it off).
            if (BoardHasPos) SafeWrite($"PS:{EffectiveStreamMs}");
        }

        protected override void OnSessionEnded()
        {
            // Drop any queued/in-flight moves — a mid-move disconnect means they can
            // never report DONE, so clear them rather than leave IsMoving stuck true
            // and hang a program that is waiting on this aux axis.
            lock (_lock) { _commandQueue.Clear(); ResetLiveSync(); }
            Interlocked.Exchange(ref _inflightCount, 0);
            BumpAllEpochs();
            Console.WriteLine($"[{LogPrefix}] Disconnected");
        }

        /// <summary>Forget the board-counter sync (the tracked _position is kept). Caller holds _lock.</summary>
        private void ResetLiveSync()
        {
            _posSynced = false;
            _clockInit = false;
            _histCount = 0;
            _histHead  = 0;
            Array.Clear(_liveVel);
        }

        protected override void OnLine(string line) => HandleLine(line, Stopwatch.GetTimestamp());

        /// <summary>Parse one received line; <paramref name="recvTicks"/> is the Stopwatch time it
        /// arrived. Split from <see cref="OnLine"/> so tests can feed lines on their own clock.</summary>
        internal void HandleLine(string line, long recvTicks)
        {
            if (string.IsNullOrEmpty(line)) return;

            // POS:<ms>,<p0>,<p1>,<p2>,<p3>  — board step counters (stream / POS? / at rest)
            if (line.StartsWith("POS:"))
            {
                var parts = line.Substring(4).Split(',');
                if (parts.Length >= 5 && TryParseCounts(parts, 0, out uint ms, out int[] raw))
                    lock (_lock) ApplyPosSample(ms, raw, recvTicks, isStream: true);
                return;
            }

            // LAT:<pin>,<val>,<ms>,<p0>,<p1>,<p2>,<p3>  — a latch pin edge with its counters
            if (line.StartsWith("LAT:"))
            {
                var parts = line.Substring(4).Split(',');
                if (parts.Length < 7 || !int.TryParse(parts[0], out int pin) || !int.TryParse(parts[1], out int val)
                    || !TryParseCounts(parts, 2, out uint ms, out int[] raw)) return;

                AuxLatchEvent evt;
                lock (_lock)
                {
                    if (_ioPins.TryGetValue(pin, out var s)) s.Value = val != 0;
                    ApplyPosSample(ms, raw, recvTicks, isStream: false);  // also a valid sample
                    var steps = new double[4];
                    for (int a = 0; a < 4; a++) steps[a] = ClampPos(a, _boardPos[a] + unchecked(raw[a] - _lastRawPos[a]) + _posOffset[a]);
                    long boardMs = _boardMs + unchecked((int)(ms - _lastRawMs));
                    long ticks   = (long)((boardMs + _clockOffsetMs) * Stopwatch.Frequency / 1000.0);
                    evt = new AuxLatchEvent(Id, pin, val, steps, Math.Min(ticks, recvTicks));
                }
                RaiseLatched(evt);
                return;
            }

            // RDY — the board rebooted (brown-out / reset button): its counters restarted at 0.
            if (line == "RDY")
            {
                lock (_lock) ResetLiveSync();
                return;
            }

            // STATE:<pin>,<I|O>,<value>;...  — full IO snapshot (response to GET)
            if (line.StartsWith("STATE:"))
            {
                foreach (string entry in line.Substring(6).Split(';'))
                {
                    string[] parts = entry.Split(',');
                    if (parts.Length >= 3 && int.TryParse(parts[0], out int pin))
                        lock (_lock)
                            if (_ioPins.TryGetValue(pin, out var s)) s.Value = parts[2] != "0";
                }
                return;
            }

            // CHG:<pin>,<value>  — an input changed (10 ms scan; positions interpolated to now)
            if (line.StartsWith("CHG:"))
            {
                string[] parts = line.Substring(4).Split(',');
                if (parts.Length >= 2 && int.TryParse(parts[0], out int pin))
                {
                    lock (_lock)
                        if (_ioPins.TryGetValue(pin, out var s)) s.Value = parts[1] != "0";
                    var steps = new double[4];
                    for (int a = 0; a < 4; a++) TryGetLivePositionAt(a, recvTicks, out steps[a], out _);
                    RaiseLatched(new AuxLatchEvent(Id, pin, parts[1] != "0" ? 1 : 0, steps, recvTicks));
                }
                return;
            }

            if (line.StartsWith("DONE:") && int.TryParse(line.Substring(5), out int axis))
            {
                // Floor at zero in case of a spurious or duplicate DONE.
                if (Interlocked.Decrement(ref _inflightCount) < 0)
                    Interlocked.Increment(ref _inflightCount);
                if ((uint)axis < 4)
                {
                    lock (_lock)
                    {
                        // With board counters the POS sent just before DONE already holds the
                        // final position — only dead reckoning adds the move's delta.
                        if (_pendingMoves[axis].Count > 0)
                        {
                            long delta = _pendingMoves[axis].Dequeue();
                            if (!_posSynced) _position[axis] = ClampPos(axis, _position[axis] + delta);
                        }
                    }
                }
                MoveDone?.Invoke(axis);
            }
        }

        private static bool TryParseCounts(string[] parts, int start, out uint ms, out int[] raw)
        {
            raw = new int[4];
            if (!uint.TryParse(parts[start], NumberStyles.None, CultureInfo.InvariantCulture, out ms)) return false;
            for (int a = 0; a < 4; a++)
                if (!int.TryParse(parts[start + 1 + a], NumberStyles.AllowLeadingSign, CultureInfo.InvariantCulture, out raw[a]))
                    return false;
            return true;
        }

        private void RaiseLatched(AuxLatchEvent evt)
        {
            // A subscriber's exception must not tear down the serial session.
            try { InputLatched?.Invoke(evt); }
            catch (Exception ex) { Console.WriteLine($"[{LogPrefix}] InputLatched handler error: {ex.Message}"); }
        }

        /// <summary>
        /// Fold one board sample into the live state: unwrap the 32-bit counters/millis, update the
        /// tracked position (clamping sticks by moving the offset, so a gripper over-driven into its
        /// stop re-references at the limit), the velocity estimate and the board→controller clock
        /// map. A sample older than the newest (a POS? reply overtaking a queued report) is ignored.
        /// Caller holds _lock.
        /// </summary>
        private void ApplyPosSample(uint rawMs, int[] raw, long recvTicks, bool isStream)
        {
            long dMs;
            if (!_posSynced)
            {
                _posSynced = true;
                _lastRawMs = rawMs;
                _boardMs   = rawMs;
                dMs        = 0;
                for (int a = 0; a < 4; a++)
                {
                    _lastRawPos[a] = raw[a];
                    _boardPos[a]   = raw[a];
                    _posOffset[a]  = _position[a] - raw[a];   // keep the tracked position continuous
                    // Board counts take over from any dead-reckoned continuous jog.
                    _simVel[a] = 0; _simVelTarget[a] = 0;
                }
            }
            else
            {
                dMs = unchecked((int)(rawMs - _lastRawMs));
                if (dMs < 0) return;
                _lastRawMs = rawMs;
                _boardMs  += dMs;
                for (int a = 0; a < 4; a++)
                {
                    _boardPos[a]  += unchecked(raw[a] - _lastRawPos[a]);
                    _lastRawPos[a] = raw[a];
                }
            }

            for (int a = 0; a < 4; a++)
            {
                long tracked = _boardPos[a] + _posOffset[a];
                long clamped = ClampPos(a, tracked);
                if (clamped != tracked) _posOffset[a] += clamped - tracked;
                _position[a] = clamped;
            }

            // Clock map: the lowest (receive − board) seen is the least-delayed sample; follow the
            // resonator's drift upward slowly so serial-latency jitter doesn't drag it.
            if (isStream || !_clockInit)
            {
                double s = TicksToMs(recvTicks) - _boardMs;
                if (!_clockInit || s < _clockOffsetMs) { _clockOffsetMs = s; _clockInit = true; }
                else _clockOffsetMs += (s - _clockOffsetMs) * Math.Min(1.0, dMs / ClockTauMs);
            }

            // Sample history → velocity.
            int newest = _histHead;
            _histMs[newest] = _boardMs;
            for (int a = 0; a < 4; a++) _histPos[newest, a] = _boardPos[a];
            _histHead = (_histHead + 1) % HistLen;
            if (_histCount < HistLen) _histCount++;
            if (_histCount < 2) return;

            int prev = (newest - 1 + HistLen) % HistLen;
            // Oldest sample within the window (at least the previous one, if it's recent enough).
            int  pick = -1;
            for (int k = 1; k < _histCount; k++)
            {
                int  j   = (newest - k + HistLen) % HistLen;
                long gap = _boardMs - _histMs[j];
                if (gap <= 0) continue;
                if (gap > VelWindowMs) { if (pick < 0 && gap <= 1000) pick = j; break; }
                pick = j;
            }
            for (int a = 0; a < 4; a++)
            {
                if (pick < 0 || _histPos[prev, a] == _boardPos[a]) { _liveVel[a] = 0; continue; } // at rest
                _liveVel[a] = (_boardPos[a] - _histPos[pick, a]) * 1000.0 / (_boardMs - _histMs[pick]);
            }
        }

        // ── Live position API (conveyor tracking) ──────────────────────────────

        /// <summary>
        /// The axis's position (steps, same space as <see cref="GetPosition"/>) extrapolated to now,
        /// and its velocity (signed steps/s). Thread-safe and never waits on the serial port —
        /// callable from the motion thread. Source, best first: the board's streamed counters
        /// (extrapolated from the newest sample with the measured velocity, at most a few stream
        /// periods ahead); else dead reckoning — the offline simulation, or for firmware without the
        /// "pos" capability the controller-integrated continuous velocity. False only for a bad axis.
        /// </summary>
        public bool TryGetLivePosition(int axis, out double steps, out double stepsPerSec) =>
            TryGetLivePositionAt(axis, Stopwatch.GetTimestamp(), out steps, out stepsPerSec);

        internal bool TryGetLivePositionAt(int axis, long nowTicks, out double steps, out double stepsPerSec)
        {
            steps = 0; stepsPerSec = 0;
            if ((uint)axis >= 4) return false;
            lock (_lock)
            {
                if (_posSynced)
                {
                    double capMs  = Math.Max(60, 3 * EffectiveStreamMs);
                    double aheadS = Math.Clamp(TicksToMs(nowTicks) - _clockOffsetMs - _boardMs, 0, capMs) / 1000.0;
                    stepsPerSec = _liveVel[axis];
                    steps = ClampPosD(axis, _boardPos[axis] + _posOffset[axis] + stepsPerSec * aheadS);
                    return true;
                }

                // Dead reckoning, extrapolated from the last sim tick.
                double dt = _simLastTicks == 0 ? 0
                          : Math.Clamp((nowTicks - _simLastTicks) / (double)Stopwatch.Frequency, 0, 0.2);
                if (_simRate[axis] > 0)
                {
                    double dir = Math.Sign(_simTarget[axis] - _simPos[axis]);
                    double p   = _simPos[axis] + dir * _simRate[axis] * dt;
                    if ((dir > 0 && p > _simTarget[axis]) || (dir < 0 && p < _simTarget[axis])) p = _simTarget[axis];
                    stepsPerSec = dir * _simRate[axis];
                    steps       = ClampPosD(axis, p);
                }
                else if (_simVel[axis] != 0 || _simVelTarget[axis] != 0)
                {
                    stepsPerSec = _simVel[axis];
                    steps       = ClampPosD(axis, _simPos[axis] + _simVel[axis] * dt);
                }
                else
                {
                    steps = _position[axis];
                }
                return true;
            }
        }

        /// <summary>
        /// Ask the board to stream POS lines every <paramref name="periodMs"/> (0 = back to the
        /// <see cref="BaselineStreamMs"/> baseline; minimum 10). Remembered and re-sent on every
        /// reconnect. No-op on firmware without the "pos" capability.
        /// </summary>
        public void SetPositionStreaming(int periodMs)
        {
            _streamPeriodMs = periodMs <= 0 ? 0 : Math.Max(10, periodMs);
            if (Connected && BoardHasPos)
                lock (_lock) _commandQueue.Enqueue($"PS:{EffectiveStreamMs}");
        }

        /// <summary>Request an immediate POS report (no-op without the "pos" capability).</summary>
        public void RequestPosition()
        {
            if (Connected && BoardHasPos)
                lock (_lock) _commandQueue.Enqueue("POS?");
        }

        // ── Public command API — all thread-safe ──────────────────────────────

        /// <summary>
        /// Start an indexed move: Arduino runs the full trapezoidal profile and reports DONE when complete.
        /// </summary>
        public void StartMove(int axis, long steps, int velocityHz, int accelHz, int decelHz)
        {
            if (steps <= 0) return;
            BumpEpoch(axis);
            bool ccw = (uint)axis < 4 && _dirCcw[axis];
            long signedSteps = (ccw ? -1L : 1L) * steps;

            if (!Connected)
            {
                // No board — simulate the move so the position advances in the UI.
                if ((uint)axis >= 4) return;
                lock (_lock)
                {
                    _simPos[axis]    = _position[axis];
                    _simTarget[axis] = _position[axis] + signedSteps;
                    _simRate[axis]   = Math.Max(1, velocityHz);
                    _simVel[axis]    = 0;
                    _simVelTarget[axis] = 0;
                }
                Interlocked.Increment(ref _inflightCount);
                return;
            }

            lock (_lock)
            {
                if ((uint)axis < 4)
                {
                    _pendingMoves[axis].Enqueue(signedSteps);
                    // The M replaces any continuous jog on the board — stop integrating it.
                    _simVel[axis] = 0; _simVelTarget[axis] = 0;
                }
                _commandQueue.Enqueue($"M:{axis},{steps},{Math.Max(1, velocityHz)},{Math.Max(1, accelHz)},{Math.Max(1, decelHz)}");
                // Count the move in-flight now, at enqueue — not when it reaches the
                // serial port on the device thread — so IsMoving is true the moment
                // this returns and a caller polling completion cannot mistake the
                // not-yet-sent move for a finished one.
                Interlocked.Increment(ref _inflightCount);
            }
        }

        /// <summary>Start continuous stepping on axis, ramping up to velocityHz at accelHz.</summary>
        public void SetContinuous(int axis, int velocityHz, int accelHz)
        {
            BumpEpoch(axis);
            if ((uint)axis < 4)
            {
                lock (_lock)
                {
                    // Offline: jog the simulated position. Connected without board counters: integrate
                    // the commanded velocity so the position still advances (dead reckoning).
                    if (!Connected || !_posSynced)
                    {
                        if (!Connected) ClearSimIndexed(axis);
                        if (_simVel[axis] == 0 && _simVelTarget[axis] == 0) _simPos[axis] = _position[axis];
                        double target = (_dirCcw[axis] ? -1 : 1) * Math.Max(1, velocityHz);
                        // Like the firmware: a reversal restarts from rest, a same-direction change ramps.
                        if (Math.Sign(target) != Math.Sign(_simVel[axis])) _simVel[axis] = 0;
                        _simVelTarget[axis] = target;
                        _simAccel[axis]     = Math.Max(1, accelHz);
                    }
                }
            }
            if (!Connected) return;
            lock (_lock)
                _commandQueue.Enqueue($"C:{axis},{Math.Max(1, velocityHz)},{Math.Max(1, accelHz)}");
        }

        /// <summary>Decelerate axis to rest at the given rate.</summary>
        public void StopSmooth(int axis, int decelHz)
        {
            if ((uint)axis < 4)
            {
                lock (_lock)
                {
                    if (!Connected) ClearSimIndexed(axis);   // a sim indexed move ends at once
                    if (!Connected || !_posSynced)
                    {
                        _simVelTarget[axis] = 0;              // a continuous jog ramps down
                        _simAccel[axis]     = Math.Max(1, decelHz);
                    }
                }
            }
            if (!Connected) return;
            lock (_lock)
                _commandQueue.Enqueue($"S:{axis},{Math.Max(1, decelHz)}");
        }

        /// <summary>Set direction for an axis (false=CW, true=CCW).</summary>
        public void SetDirection(int axis, bool ccw)
        {
            if ((uint)axis < 4) _dirCcw[axis] = ccw;
            if (Connected)
                lock (_lock) _commandQueue.Enqueue($"D:{axis},{(ccw ? 1 : 0)}");
        }

        /// <summary>Enable or disable all stepper drivers.</summary>
        public void Enable(bool enable)
        {
            _desiredEnabled = enable;  // remembered across reconnects
            _motorEnabled   = enable;
            if (Connected)
                lock (_lock) _commandQueue.Enqueue($"E:{(enable ? 1 : 0)}");
        }

        /// <summary>Emergency stop — halt all axes immediately, clear queued commands.</summary>
        public void StopAll()
        {
            lock (_lock)
            {
                _commandQueue.Clear();
                Interlocked.Exchange(ref _inflightCount, 0);
                _commandQueue.Enqueue("X");
                foreach (var q in _pendingMoves) q.Clear();
                for (int i = 0; i < 4; i++) { _simVel[i] = 0; _simVelTarget[i] = 0; _simRate[i] = 0; }
            }
        }

        // ── Digital IO API (Uno spare pins) — all thread-safe ───────────────────

        /// <summary>Live state of every configured IO pin.</summary>
        public List<IoPinState> GetIoPinStates()
        {
            lock (_lock) return _ioPins.Values.ToList();
        }

        /// <summary>Queue a digital output change. Optimistically updates tracked state.</summary>
        public void SetIoOutput(int pin, bool value)
        {
            IoPinState? state;
            lock (_lock)
            {
                _commandQueue.Enqueue($"SET:{pin},{(value ? 1 : 0)}");
                _ioPins.TryGetValue(pin, out state);
            }
            if (state != null) state.Value = value;
        }

        /// <summary>Queue a pin reconfiguration (Input/Output; an Input may be a latch input).</summary>
        public void ConfigureIoPin(int pin, PinType type, bool latch = false)
        {
            lock (_lock) _commandQueue.Enqueue(IoConfigCommand(pin, type, latch));
        }

        /// <summary>Add or update the tracked IO pin state without sending a firmware command.</summary>
        public void UpsertIoPin(int pin, PinType type, string name, bool latch = false)
        {
            lock (_lock)
            {
                if (_ioPins.TryGetValue(pin, out var existing))
                {
                    existing.Type  = type;
                    existing.Name  = name;
                    existing.Latch = latch;
                }
                else
                {
                    _ioPins[pin] = new IoPinState
                    {
                        Pin = pin, Type = type, Name = name, Latch = latch,
                        DeviceId = Id, DeviceName = Name, Value = false,
                    };
                }
            }
        }

        /// <summary>Remove an IO pin from tracking.</summary>
        public void RemoveIoPin(int pin)
        {
            lock (_lock) _ioPins.Remove(pin);
        }

        /// <summary>Set the tracked label of an IO pin. Returns false if the pin isn't tracked.</summary>
        public bool RenameIoPin(int pin, string name)
        {
            lock (_lock)
            {
                if (!_ioPins.TryGetValue(pin, out var s)) return false;
                s.Name = name;
                return true;
            }
        }

        // ── Offline simulation internals ───────────────────────────────────────

        /// <summary>Cancel an in-flight simulated indexed move on an axis. Caller holds _lock.</summary>
        private void ClearSimIndexed(int axis)
        {
            if (_simRate[axis] > 0)
            {
                _simRate[axis] = 0;
                if (Interlocked.Decrement(ref _inflightCount) < 0)
                    Interlocked.Increment(ref _inflightCount);
            }
        }

        /// <summary>Advance the simulation / dead reckoning to <paramref name="nowTicks"/> (Stopwatch).
        /// Run every 50 ms by the sim timer. No-op once board counters are live (they drive position);
        /// indexed moves are only simulated offline (a connected board reports DONE).</summary>
        internal void SimStep(long nowTicks)
        {
            List<int>? completed = null;
            lock (_lock)
            {
                if (Connected && _posSynced) { _simLastTicks = 0; return; }
                if (_simLastTicks == 0) { _simLastTicks = nowTicks; return; }
                double dt = (nowTicks - _simLastTicks) / (double)Stopwatch.Frequency;
                _simLastTicks = nowTicks;
                if (dt <= 0) return;

                for (int axis = 0; axis < 4; axis++)
                {
                    if (_simRate[axis] > 0) // indexed move toward a target
                    {
                        double dir = Math.Sign(_simTarget[axis] - _simPos[axis]);
                        if (dir == 0)
                        {
                            _simRate[axis] = 0;
                            (completed ??= new()).Add(axis);
                            continue;
                        }
                        _simPos[axis] += dir * _simRate[axis] * dt;
                        if ((dir > 0 && _simPos[axis] >= _simTarget[axis]) ||
                            (dir < 0 && _simPos[axis] <= _simTarget[axis]))
                        {
                            _simPos[axis]  = _simTarget[axis];
                            _simRate[axis] = 0;
                            (completed ??= new()).Add(axis);
                        }
                        _position[axis] = ClampPos(axis, (long)Math.Round(_simPos[axis]));
                    }
                    else if (_simVel[axis] != 0 || _simVelTarget[axis] != 0) // continuous jog, ramped
                    {
                        // Ramp for tr (≤ dt) to v1, then hold v1 for the rest of the tick.
                        double v0 = _simVel[axis], tgt = _simVelTarget[axis], acc = Math.Max(1, _simAccel[axis]);
                        double tr = Math.Min(dt, Math.Abs(tgt - v0) / acc);
                        double v1 = v0 + Math.Sign(tgt - v0) * acc * tr;
                        _simPos[axis]  += (v0 + v1) * 0.5 * tr + v1 * (dt - tr);
                        _simVel[axis]   = v1;
                        _position[axis] = ClampPos(axis, (long)Math.Round(_simPos[axis]));
                    }
                }
            }

            if (completed != null)
                foreach (int axis in completed)
                {
                    if (Interlocked.Decrement(ref _inflightCount) < 0)
                        Interlocked.Increment(ref _inflightCount);
                    MoveDone?.Invoke(axis);
                }
        }

        /// <summary>Returns the tracked step position for an axis (board counters when live, else DONE / dead reckoning).</summary>
        public long GetPosition(int axis)
        {
            lock (_lock) return (uint)axis < 4 ? _position[axis] : 0;
        }

        /// <summary>Reset the tracked position for an axis to zero (e.g. after homing).</summary>
        public void ZeroPosition(int axis)
        {
            lock (_lock) { if ((uint)axis < 4) SetTracked(axis, 0); }
            BumpEpoch(axis);
        }

        /// <summary>Declare the tracked step position for an axis — resolves ("sets" or, with 0,
        /// "clears") the dead-reckoned position without moving the motor.</summary>
        public void SetPosition(int axis, long steps)
        {
            lock (_lock) { if ((uint)axis < 4) SetTracked(axis, steps); }
            BumpEpoch(axis);
        }

        /// <summary>Constrain an axis's tracked position to [min, max] steps (a gripper's closed/open
        /// gap). Re-clamps the current position immediately. Pass min &lt;= max.</summary>
        public void SetPositionBounds(int axis, long min, long max)
        {
            if ((uint)axis >= 4 || min > max) return;
            lock (_lock)
            {
                _posMin[axis] = min;
                _posMax[axis] = max;
                SetTracked(axis, _position[axis]);
            }
        }

        /// <summary>Remove an axis's position bounds (generic, unbounded travel).</summary>
        public void ClearPositionBounds(int axis)
        {
            if ((uint)axis >= 4) return;
            lock (_lock) { _posMin[axis] = long.MinValue; _posMax[axis] = long.MaxValue; }
        }

        /// <summary>Set the tracked position (clamped) and keep its sources coherent: the board-counter
        /// offset when live, the running sim/dead-reckoned position during a continuous jog. Caller holds _lock.</summary>
        private void SetTracked(int axis, long steps)
        {
            _position[axis] = ClampPos(axis, steps);
            if (_posSynced) _posOffset[axis] = _position[axis] - _boardPos[axis];
            if (_simRate[axis] == 0 && (_simVel[axis] != 0 || _simVelTarget[axis] != 0)) _simPos[axis] = _position[axis];
        }

        /// <summary>Clamp a step position to an axis's soft bounds. Caller holds _lock.</summary>
        private long ClampPos(int axis, long v) =>
            v < _posMin[axis] ? _posMin[axis] : (v > _posMax[axis] ? _posMax[axis] : v);

        private double ClampPosD(int axis, double v) =>
            v < _posMin[axis] ? _posMin[axis] : (v > _posMax[axis] ? _posMax[axis] : v);
    }
}
