using System;
using System.Globalization;
using System.Threading;
using Controller.RobotControl.Execution;

namespace Controller.RobotControl.Gcode
{
    /// <summary>
    /// One live G-code stream (raw TCP or WebSocket). Each line is interpreted and its moves are
    /// enqueued on the same motion queue the program executor uses, with GRBL-style per-line flow
    /// control: a motion line only acks once the planner queue has room (≤ <see cref="QueueCap"/>),
    /// so the sender paces itself. Non-motion lines (spindle, dwell, settings) ack immediately.
    /// While a session is open it owns the motion queue — program runs are refused, and a session
    /// refuses to open while a program is running.
    /// </summary>
    internal sealed class GcodeStreamSession : IDisposable
    {
        private const int QueueCap = 16;

        private readonly RobotController  _robot;
        private readonly GcodeInterpreter _interp;
        private readonly SpindleOutput    _spindle;
        private double _x, _y, _z, _rz; // tracked machine target (seeded from the robot)
        private bool   _disposed;

        /// <summary>Opens a session unless a program is running or another stream is active.</summary>
        public static bool TryCreate(RobotController robot, out GcodeStreamSession? session, out string error)
        {
            session = null; error = "";
            if (robot.ProgramRunning)    { error = "A program is running";       return false; }
            if (robot.GcodeStreamActive) { error = "Another G-code stream is active"; return false; }
            robot.GcodeStreamActive = true;
            session = new GcodeStreamSession(robot);
            return true;
        }

        private GcodeStreamSession(RobotController robot)
        {
            _robot   = robot;
            var cfg  = robot.Config;
            _spindle = cfg.GcodeSpindle();
            var pos  = robot.GetCurrentPosition();
            _x = pos.X; _y = pos.Y; _z = pos.Z; _rz = pos.RZ;
            _interp = new GcodeInterpreter(cfg.GcodeOptions(), (pos.X, pos.Y, pos.Z, pos.RZ));
        }

        /// <summary>A short banner senders can show on connect.</summary>
        public static string Banner => "SimpleRobot G-code stream ready";

        /// <summary>
        /// Interpret one line and return the reply (usually "ok", or "error:&lt;msg&gt;"). Blocks
        /// while the planner queue is full so the sender's next line waits — classic streaming
        /// flow control. <paramref name="ct"/> unblocks it if the client goes away.
        /// </summary>
        public string Feed(string line, CancellationToken ct)
        {
            if (_disposed) return "error:stream closed";
            System.Collections.Generic.IReadOnlyList<GcodeOp> ops;
            try { ops = _interp.Feed(line); }
            catch (GcodeException ex) { return "error:" + ex.Message; }

            foreach (var op in ops)
            {
                switch (op.Kind)
                {
                    case GcodeOpKind.Move:    EnqueueMove(op, ct); break;
                    case GcodeOpKind.Dwell:   Dwell(op.DwellMs, ct); break;
                    case GcodeOpKind.Spindle: ApplySpindle(op.SpindleOn); break;
                    case GcodeOpKind.Home:    _robot.RequestHome(); break;
                    case GcodeOpKind.End:     ApplySpindle(false); break;
                    case GcodeOpKind.Pause:   break; // M0/M1 — not a hold in stream mode
                    case GcodeOpKind.SetPosition: break; // interpreter offset only
                }
            }
            return "ok";
        }

        private void EnqueueMove(GcodeOp op, CancellationToken ct)
        {
            double nx = op.Relative ? _x + (op.X ?? 0) : (op.X ?? _x);
            double ny = op.Relative ? _y + (op.Y ?? 0) : (op.Y ?? _y);
            double nz = op.Relative ? _z + (op.Z ?? 0) : (op.Z ?? _z);
            double nr = op.Relative ? _rz + (op.A ?? 0) : (op.A ?? _rz);

            // Flow control: wait for planner room before enqueuing (paces the sender).
            while (!ct.IsCancellationRequested && _robot.QueuedCommands.Count >= QueueCap)
                Thread.Sleep(2);
            if (ct.IsCancellationRequested) return;

            _robot.QueuedCommands.Enqueue(new RobotCommand
            {
                CommandType = "MoveL",
                X = nx, Y = ny, Z = nz, RZ = nr,
                Speed = op.Rapid ? _robot.Config.GcodeRapidSpeed : op.FeedMmPerSec,
                ApplySpeedOverride = false,
            });
            _x = nx; _y = ny; _z = nz; _rz = nr;
        }

        private void Dwell(double ms, CancellationToken ct)
        {
            long end = Environment.TickCount64 + (long)Math.Max(0, ms);
            while (Environment.TickCount64 < end && !ct.IsCancellationRequested)
                Thread.Sleep(Math.Min(20, (int)Math.Max(1, end - Environment.TickCount64)));
        }

        private void ApplySpindle(bool on)
        {
            if (!_spindle.Enabled) return;
            IoSteps.ApplyOutput(_robot, _spindle.Type, _spindle.Pin, on, null);
        }

        // ── Realtime controls (single bytes from the transport) ──

        /// <summary>GRBL '?' — a one-line status report.</summary>
        public string Status()
        {
            var p = _robot.GetCurrentPosition();
            string state = _robot.MotionBusy ? "Run" : "Idle";
            var c = CultureInfo.InvariantCulture;
            return $"<{state}|MPos:{p.X.ToString("0.000", c)},{p.Y.ToString("0.000", c)},{p.Z.ToString("0.000", c)}|A:{p.RZ.ToString("0.000", c)}>";
        }

        /// <summary>GRBL '!' (feed hold) and 0x18 (soft reset): stop motion. We have no resumable
        /// hold, so both stop — documented in docs/gcode.md.</summary>
        public void Stop() => _robot.HardStop();

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            // A dropped connection mid-run should not keep cutting.
            ApplySpindle(false);
            _robot.HardStop();
            _robot.GcodeStreamActive = false;
        }
    }
}
