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
        private double defaultResetTime = 1.0f;
        private double resetTime = 0.6f;

        public bool IsFinished { get; private set; } = true;
        public double StopEpsilon { get; set; } = 1e-6;
        private Vector6 jogAxis = new();
        private double speed;
        private double accel;
        private double decel;
        private volatile bool _stopRequested = false;

        public Vector6 Update(Vector6 currentPosition)
        {
            bool enabled = !IsFinished && !_stopRequested && autoResetSw.Elapsed.TotalSeconds < resetTime;

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

            // Pick the ramp rate per axis: accel while an axis is building speed toward its
            // target, decel while it's slowing toward (or reversing through) zero. Tying decel
            // only to the disabled state meant a joystick reversal — target flips sign while the
            // jog stays enabled — ramped down through zero at `accel` instead of `decel`. This is
            // behaviour-identical for the button pad (a release drives target to 0, so every axis
            // decelerates) while giving the joystick a proper decel before it changes direction.
            double aRate = Math.Abs(accel);
            double dRate = Math.Abs(decel);
            double dt2 = dt;

            _vel = new Vector6(
                StepAxis(_vel.X,  targetVel.X,  aRate, dRate, dt2),
                StepAxis(_vel.Y,  targetVel.Y,  aRate, dRate, dt2),
                StepAxis(_vel.Z,  targetVel.Z,  aRate, dRate, dt2),
                StepAxis(_vel.RX, targetVel.RX, aRate, dRate, dt2),
                StepAxis(_vel.RY, targetVel.RY, aRate, dRate, dt2),
                StepAxis(_vel.RZ, targetVel.RZ, aRate, dRate, dt2)
              );

            if (!enabled && IsNearlyZero(_vel))
            {
                _vel = Vector6.Zero;
                IsFinished = true;
                _started = false;
                _stopRequested = false;
                return currentPosition;
            }

            IsFinished = false;

            return new Vector6(
                currentPosition.X + _vel.X * dt,
                currentPosition.Y + _vel.Y * dt,
                currentPosition.Z + _vel.Z * dt,
                currentPosition.RX + _vel.RX * dt,
                currentPosition.RY + _vel.RY * dt,
                currentPosition.RZ + _vel.RZ * dt
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
            _stopRequested = false;
            autoResetSw.Restart();

            if (IsFinished)
            {
                _started = false;
                _vel = Vector6.Zero;
            }
            IsFinished = false;
        }

        public void StopJog() => _stopRequested = true;

        public void ForceStop()
        {
            _vel = Vector6.Zero;
            IsFinished = true;
            _started = false;
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

        // Ramp one axis toward its target, choosing accel vs decel by whether the axis is
        // building speed or slowing toward/through zero. An axis is "decelerating" when its
        // target has a smaller magnitude than its current velocity, or points the opposite way
        // (a reversal first bleeds speed down to zero at decel, then builds up the other way at
        // accel). maxDelta <= 0 snaps straight to target (matches the old maxDv<=0 path).
        private static double StepAxis(double current, double target, double accel, double decel, double dt)
        {
            bool reversing     = current != 0 && Math.Sign(target) != Math.Sign(current);
            bool decelerating  = reversing || Math.Abs(target) < Math.Abs(current);
            double rate        = decelerating ? decel : accel;
            double maxDelta     = rate * dt;
            if (maxDelta <= 0) return target;
            return MoveTowards(current, target, maxDelta);
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
