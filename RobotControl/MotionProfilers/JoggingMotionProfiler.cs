using System;
using System.Collections.Generic;
using System.Text;

namespace Controller.RobotControl.MotionProfilers
{
    using System;
    using System.Diagnostics;

    public class JoggingMotionProfiler
    {
        private Vector6 _vel = new(0, 0, 0, 0, 0, 0);
        private readonly Stopwatch _sw = new();
        private bool _started;
        private Stopwatch autoResetSw = new();
        private double defaultResetTime = 0.1f;
        private double resetTime = 0.1f;

        public bool IsFinished { get; private set; } = true;
        public double StopEpsilon { get; set; } = 1e-6;
        private Vector6 jogAxis = new();
        private double speed;
        private double accel;
        private double decel;

        public Vector6 Update(Vector6 currentPosition)
        {
            bool enabled = !IsFinished && autoResetSw.Elapsed.TotalSeconds < resetTime;

            double dt = GetDtSec();
            if (dt <= 0) return currentPosition;

            double maxSpeed = Math.Abs(speed);

            Vector6 targetVel = enabled
                ? new Vector6(
                    jogAxis.X * maxSpeed,
                    jogAxis.Y * maxSpeed,
                    jogAxis.Z * maxSpeed,
                    jogAxis.RX * maxSpeed,
                    jogAxis.RY * maxSpeed,
                    jogAxis.RZ * maxSpeed
                  )
                : Vector6.Zero;

            double rate = enabled ? Math.Abs(accel) : Math.Abs(decel);
            double maxDv = rate * dt;

            Vector6 velPrev = _vel;

            _vel = (maxDv <= 0)
                ? targetVel
                : new Vector6(
                    MoveTowards(_vel.X, targetVel.X, maxDv),
                    MoveTowards(_vel.Y, targetVel.Y, maxDv),
                    MoveTowards(_vel.Z, targetVel.Z, maxDv),
                    MoveTowards(_vel.RX, targetVel.RX, maxDv),
                    MoveTowards(_vel.RY, targetVel.RY, maxDv),
                    MoveTowards(_vel.RZ, targetVel.RZ, maxDv)
                  );

            if (!enabled && IsNearlyZero(_vel))
            {
                _vel = Vector6.Zero;
                IsFinished = true;
                _started = false;
                return currentPosition;
            }

            IsFinished = false;

            // average velocity integration (prevents the “first tick jump”)
            return new Vector6(
                currentPosition.X + (velPrev.X + _vel.X) * 0.5 * dt,
                currentPosition.Y + (velPrev.Y + _vel.Y) * 0.5 * dt,
                currentPosition.Z + (velPrev.Z + _vel.Z) * 0.5 * dt,
                currentPosition.RX + (velPrev.RX + _vel.RX) * 0.5 * dt,
                currentPosition.RY + (velPrev.RY + _vel.RY) * 0.5 * dt,
                currentPosition.RZ + (velPrev.RZ + _vel.RZ) * 0.5 * dt
            );
        }

        public void Jog(Vector6 jogAxis, double speed, double accel, double decel, double? resetTime = null)
        {
            if (resetTime != null)
            {
                this.resetTime = resetTime.Value;
            }
            else
            {
                this.resetTime = defaultResetTime;
            }

            this.jogAxis = jogAxis;
            this.speed = speed;
            this.accel = accel;
            this.decel = decel;
            autoResetSw.Restart();

            if (IsFinished)
            {
                _started = false;
                _vel = Vector6.Zero;
            }
            IsFinished = false;
        }

        private double GetDtSec()
        {
            if (!_started)
            {
                _started = true;
                _sw.Restart();
                return 0.0;
            }

            double dt = _sw.Elapsed.TotalSeconds;
            _sw.Restart();

            if (dt > 0.25) dt = 0.25; // clamps big pauses (breakpoints)
            return dt;
        }

        private static double MoveTowards(double current, double target, double maxDelta)
        {
            double delta = target - current;
            if (Math.Abs(delta) <= maxDelta) return target;
            return current + Math.Sign(delta) * maxDelta;
        }

        private bool IsNearlyZero(Vector6 v) =>
            Math.Abs(v.X) <= StopEpsilon &&
            Math.Abs(v.Y) <= StopEpsilon &&
            Math.Abs(v.Z) <= StopEpsilon &&
            Math.Abs(v.RX) <= StopEpsilon &&
            Math.Abs(v.RY) <= StopEpsilon &&
            Math.Abs(v.RZ) <= StopEpsilon;
    }

}
