using System;

namespace Controller.RobotControl.AuxAxis
{
    /// <summary>
    /// Trapezoidal velocity profiler for one aux stepper axis.
    /// The C# host owns all motion math; the Arduino only executes pulse batches.
    ///
    /// Units: steps and seconds.
    ///   velocity -> steps/sec
    ///   accel    -> steps/sec^2
    /// </summary>
    public class AuxAxisMotionProfiler
    {
        private long   _totalSteps;
        private long   _stepsSent;
        private double _velocity;
        private double _targetVelocity;
        private double _accel;
        private double _decel;
        private double _fracAccum;
        private bool   _decelerating;
        private bool   _continuous;

        public bool   IsFinished       { get; private set; }
        public bool   IsContinuous     => _continuous;
        public double CurrentVelocity  => _velocity;

        // ── Indexed move (exact step count) ──────────────────────────────────

        public void SetupIndexed(long totalSteps, double maxVelocity, double accel, double decel)
        {
            _totalSteps     = Math.Max(1, totalSteps);
            _stepsSent      = 0;
            _velocity       = 0;
            _targetVelocity = Math.Max(1, maxVelocity);
            _accel          = Math.Max(1, accel);
            _decel          = Math.Max(1, decel);
            _fracAccum      = 0;
            _decelerating   = false;
            _continuous     = false;
            IsFinished      = false;
        }

        // ── Continuous move (ramps up, holds until RequestStop) ───────────────

        public void SetupContinuous(double maxVelocity, double accel)
        {
            _totalSteps     = long.MaxValue;
            _stepsSent      = 0;
            _velocity       = 0;
            _targetVelocity = Math.Max(1, maxVelocity);
            _accel          = Math.Max(1, accel);
            _decel          = _accel;
            _fracAccum      = 0;
            _decelerating   = false;
            _continuous     = true;
            IsFinished      = false;
        }

        /// <summary>Request a controlled decel-to-stop. Marks IsFinished when velocity reaches 0.</summary>
        public void RequestStop(double decel)
        {
            if (IsFinished) return;
            _decel        = Math.Max(1, decel);
            _decelerating = true;
            _continuous   = false;
            long stepsToStop = (long)Math.Ceiling(_velocity * _velocity / (2.0 * _decel)) + 1;
            _totalSteps   = _stepsSent + stepsToStop;
        }

        /// <summary>Immediate stop with no ramp.</summary>
        public void RequestImmediateStop() => IsFinished = true;

        /// <summary>
        /// Advance the profile by dt seconds.
        /// Returns the integer step count to send to the Arduino this tick.
        /// </summary>
        public int Update(double dt)
        {
            if (IsFinished) return 0;

            long stepsRemaining = _totalSteps - _stepsSent;

            // Trigger decel phase when remaining distance equals braking distance
            if (!_continuous && !_decelerating)
            {
                double stepsToStop = _velocity * _velocity / (2.0 * _decel);
                if (stepsRemaining <= stepsToStop)
                    _decelerating = true;
            }

            // Integrate velocity
            if (!_decelerating)
                _velocity = Math.Min(_velocity + _accel * dt, _targetVelocity);
            else
            {
                _velocity = Math.Max(_velocity - _decel * dt, 0);
                if (_velocity <= 0)
                {
                    IsFinished = true;
                    return 0;
                }
            }

            // Fractional step accumulation
            _fracAccum += _velocity * dt;
            int steps = (int)_fracAccum;
            if (!_continuous)
                steps = (int)Math.Min(steps, stepsRemaining);

            _fracAccum -= steps;
            _stepsSent += steps;

            if (!_continuous && _stepsSent >= _totalSteps)
                IsFinished = true;

            return steps;
        }
    }
}
