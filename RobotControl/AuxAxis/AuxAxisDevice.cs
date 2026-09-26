using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
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

        public AuxAxisDevice(AuxAxisConfig config) : base(config.Id, config.Name) => _config = config;

        // ── Session hooks ───────────────────────────────────────────────────────

        protected override void OnSessionStarted(SerialPort port)
        {
            Console.WriteLine($"[{LogPrefix}] Connected on {port.PortName}");

            // Restore the operator's chosen enable state rather than forcing it on.
            _motorEnabled = _desiredEnabled;
            SafeWrite(_desiredEnabled ? "E:1" : "E:0");
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
                Interlocked.Exchange(ref _inflightCount, 0);
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
