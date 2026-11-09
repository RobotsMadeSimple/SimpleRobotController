using System;
using System.Collections.Generic;
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
    ///   ARDUINO -> HOST:  RDY  OK  DONE:axis  ERR
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

        // ── Offline simulation ─────────────────────────────────────────────────
        // With no board connected, mirror the Arduino's effect locally so an axis can
        // still be jogged / moved and its dead-reckoned position advances in the UI
        // (e.g. to set up a gripper or dry-run a program without hardware present).
        // All sim state is guarded by _lock; _simPos is the running fractional position
        // that _position is rounded from while a sim motion is active, and is re-seeded
        // from _position at the start of each motion so SetPosition/Zero stay authoritative.
        private readonly double[] _simPos    = new double[4]; // running simulated position (steps)
        private readonly double[] _simVel    = new double[4]; // continuous signed steps/s (0 = none)
        private readonly double[] _simRate   = new double[4]; // indexed move speed (steps/s; >0 = active)
        private readonly long[]   _simTarget = new long[4];   // indexed move target (valid when _simRate>0)
        private readonly Timer    _simTimer;
        private long _simLastMs;
        private const int SimTickMs = 50;

        /// <summary>True while an indexed M move is executing on the Arduino.</summary>
        public bool IsMoving => Volatile.Read(ref _inflightCount) > 0;

        /// <summary>Fired when the Arduino reports DONE for a completed M move.</summary>
        public event Action<int>? MoveDone;

        // ── SerialLineDevice knobs ─────────────────────────────────────────────
        protected override string DeviceTag                    => "AuxAxis";
        protected override string ThreadNamePrefix              => "AuxAxis";
        protected override int    ProbeAttempts                 => 3;
        protected override int    ProbeResponseDelayMs          => 150;
        protected override bool   KeepProbePortOpenForSession   => true;
        protected override bool   CatchIOException              => true;
        protected override string UnhandledErrorLogVerb          => "Error";
        public    override string SketchName                    => "AuxStepperDriver";

        // Digital IO on the Uno's spare pins — same model/protocol as a Nano edge device
        // (Input/Output only; no HX711/NeoPixel). Guarded by the base _lock.
        private readonly Dictionary<int, IoPinState> _ioPins = new();

        public AuxAxisDevice(AuxAxisConfig config) : base(config.Id, config.Name)
        {
            _config   = config;
            _simTimer = new Timer(SimTick, null, SimTickMs, SimTickMs);

            // Pre-populate IO pin state so callers can read it before the device connects.
            foreach (var p in config.IoPins)
                _ioPins[p.Pin] = new IoPinState
                {
                    Pin = p.Pin, Type = p.Type, Name = p.Name,
                    DeviceId = config.Id, DeviceName = config.Name, Value = false,
                };
        }

        /// <summary>The firmware CFG command for an IO pin (Input/Output only).</summary>
        private static string IoConfigCommand(int pin, PinType type) =>
            type == PinType.Output ? $"CFG:{pin},O" : $"CFG:{pin},I";

        // ── Session hooks ───────────────────────────────────────────────────────

        protected override void OnSessionStarted(SerialPort port)
        {
            // (The base class has already logged "Connected on <port> — firmware …".)

            // Restore the operator's chosen enable state rather than forcing it on.
            _motorEnabled = _desiredEnabled;
            SafeWrite(_desiredEnabled ? "E:1" : "E:0");

            // Push IO pin configuration, then request a full state snapshot.
            foreach (var p in _config.IoPins)
            {
                SafeWrite(IoConfigCommand(p.Pin, p.Type));
                Thread.Sleep(20);
            }
            if (_config.IoPins.Count > 0) SafeWrite("GET");

            // A real board now drives the axes — abandon any offline simulation so the
            // two don't both move the tracked position.
            lock (_lock)
                for (int i = 0; i < 4; i++) { _simVel[i] = 0; _simRate[i] = 0; }
            Interlocked.Exchange(ref _inflightCount, 0);
            _simLastMs = 0;
        }

        protected override void OnSessionEnded()
        {
            // Drop any queued/in-flight moves — a mid-move disconnect means they can
            // never report DONE, so clear them rather than leave IsMoving stuck true
            // and hang a program that is waiting on this aux axis.
            lock (_lock) _commandQueue.Clear();
            Interlocked.Exchange(ref _inflightCount, 0);
            Console.WriteLine($"[{LogPrefix}] Disconnected");
        }

        protected override void OnLine(string line)
        {
            if (string.IsNullOrEmpty(line)) return;

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

            // CHG:<pin>,<value>  — an input changed
            if (line.StartsWith("CHG:"))
            {
                string[] parts = line.Substring(4).Split(',');
                if (parts.Length >= 2 && int.TryParse(parts[0], out int pin))
                    lock (_lock)
                        if (_ioPins.TryGetValue(pin, out var s)) s.Value = parts[1] != "0";
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
                        if (_pendingMoves[axis].Count > 0)
                            _position[axis] = ClampPos(axis, _position[axis] + _pendingMoves[axis].Dequeue());
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
                }
                Interlocked.Increment(ref _inflightCount);
                return;
            }

            lock (_lock)
            {
                if ((uint)axis < 4) _pendingMoves[axis].Enqueue(signedSteps);
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
            if (!Connected)
            {
                // No board — jog the simulated position at the signed velocity until stopped.
                if ((uint)axis >= 4) return;
                lock (_lock)
                {
                    ClearSimIndexed(axis);
                    _simPos[axis] = _position[axis];
                    _simVel[axis] = (_dirCcw[axis] ? -1 : 1) * Math.Max(1, velocityHz);
                }
                return;
            }
            lock (_lock)
                _commandQueue.Enqueue($"C:{axis},{Math.Max(1, velocityHz)},{Math.Max(1, accelHz)}");
        }

        /// <summary>Decelerate axis to rest at the given rate.</summary>
        public void StopSmooth(int axis, int decelHz)
        {
            if (!Connected)
            {
                if ((uint)axis >= 4) return;
                lock (_lock) { _simVel[axis] = 0; ClearSimIndexed(axis); }
                return;
            }
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
                for (int i = 0; i < 4; i++) { _simVel[i] = 0; _simRate[i] = 0; }
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

        /// <summary>Queue a pin reconfiguration (Input/Output).</summary>
        public void ConfigureIoPin(int pin, PinType type)
        {
            lock (_lock) _commandQueue.Enqueue(IoConfigCommand(pin, type));
        }

        /// <summary>Add or update the tracked IO pin state without sending a firmware command.</summary>
        public void UpsertIoPin(int pin, PinType type, string name)
        {
            lock (_lock)
            {
                if (_ioPins.TryGetValue(pin, out var existing))
                {
                    existing.Type = type;
                    existing.Name = name;
                }
                else
                {
                    _ioPins[pin] = new IoPinState
                    {
                        Pin = pin, Type = type, Name = name,
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

        /// <summary>Advance the offline simulation each tick. No-op when a board is connected
        /// (it drives position via DONE) or when nothing is moving.</summary>
        private void SimTick(object? _)
        {
            if (Connected) { _simLastMs = 0; return; }

            List<int>? completed = null;
            lock (_lock)
            {
                long now = Environment.TickCount64;
                if (_simLastMs == 0) { _simLastMs = now; return; }
                double dt = (now - _simLastMs) / 1000.0;
                _simLastMs = now;
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
                    else if (_simVel[axis] != 0) // continuous jog
                    {
                        _simPos[axis]  += _simVel[axis] * dt;
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

        /// <summary>Returns the tracked step position for an axis (updated on each DONE event).</summary>
        public long GetPosition(int axis)
        {
            lock (_lock) return (uint)axis < 4 ? _position[axis] : 0;
        }

        /// <summary>Reset the tracked position for an axis to zero (e.g. after homing).</summary>
        public void ZeroPosition(int axis)
        {
            lock (_lock) { if ((uint)axis < 4) _position[axis] = ClampPos(axis, 0); }
        }

        /// <summary>Declare the tracked step position for an axis — resolves ("sets" or, with 0,
        /// "clears") the dead-reckoned position without moving the motor.</summary>
        public void SetPosition(int axis, long steps)
        {
            lock (_lock) { if ((uint)axis < 4) _position[axis] = ClampPos(axis, steps); }
        }

        /// <summary>Constrain an axis's tracked position to [min, max] steps (a gripper's closed/open
        /// gap). Re-clamps the current position immediately. Pass min &lt;= max.</summary>
        public void SetPositionBounds(int axis, long min, long max)
        {
            if ((uint)axis >= 4 || min > max) return;
            lock (_lock)
            {
                _posMin[axis]   = min;
                _posMax[axis]   = max;
                _position[axis] = ClampPos(axis, _position[axis]);
            }
        }

        /// <summary>Remove an axis's position bounds (generic, unbounded travel).</summary>
        public void ClearPositionBounds(int axis)
        {
            if ((uint)axis >= 4) return;
            lock (_lock) { _posMin[axis] = long.MinValue; _posMax[axis] = long.MaxValue; }
        }

        /// <summary>Clamp a step position to an axis's soft bounds. Caller holds _lock.</summary>
        private long ClampPos(int axis, long v) =>
            v < _posMin[axis] ? _posMin[axis] : (v > _posMax[axis] ? _posMax[axis] : v);
    }
}
