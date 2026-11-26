using System;

namespace Controller.RobotControl.MotionProfilers
{
    /// <summary>The speed profile a move runs with (robot config <c>motionProfile</c>, or a move's own override).</summary>
    internal enum MotionProfileKind
    {
        /// <summary>Constant acceleration ramps (the default — what every move has always used).</summary>
        Trapezoid,
        /// <summary>Jerk-limited ramps: acceleration itself ramps up and down, so it never steps.</summary>
        SCurve,
    }

    /// <summary>
    /// How a move is profiled: the kind, and for an S-curve how hard the acceleration may change.
    /// <see cref="SmoothnessPercent"/> (1–100) shapes every ramp the same way whatever its units:
    /// it is the share of each acceleration phase spent ramping the acceleration (100 = no
    /// constant-acceleration segment at all). <see cref="Jerk"/>, when set, is an absolute jerk
    /// limit (mm/s³ for linear moves, joint units/s³ for joint moves) and replaces the smoothness.
    /// </summary>
    internal readonly record struct MotionProfileSettings(MotionProfileKind Kind, double SmoothnessPercent = 50, double? Jerk = null)
    {
        public const string TrapezoidName = "trapezoid";
        public const string SCurveName    = "scurve";

        public static MotionProfileSettings Trapezoid => new(MotionProfileKind.Trapezoid);

        public bool IsSCurve => Kind == MotionProfileKind.SCurve;

        /// <summary>"trapezoid" / "scurve" (case-insensitive) → the kind; anything else → null.</summary>
        public static MotionProfileKind? ParseKind(string? name) =>
            string.Equals(name?.Trim(), SCurveName, StringComparison.OrdinalIgnoreCase) ? MotionProfileKind.SCurve
            : string.Equals(name?.Trim(), TrapezoidName, StringComparison.OrdinalIgnoreCase) ? MotionProfileKind.Trapezoid
            : null;

        /// <summary>
        /// The jerk for one ramp with velocity limit <paramref name="vmax"/> and acceleration
        /// <paramref name="accel"/>: the configured jerk when set, else the one that makes the
        /// two jerk phases a <see cref="SmoothnessPercent"/> share of a full ramp up to vmax.
        /// </summary>
        /// <remarks>
        /// A full ramp lasts vmax/a + a/j with 2a/j of it spent ramping the acceleration, so a
        /// share f needs j = a²(2 − f) / (f · vmax). The ratio a²/v keeps the shape independent
        /// of units, which is what lets one percentage serve mm and degree moves alike.
        /// </remarks>
        public double JerkFor(double vmax, double accel)
        {
            if (Jerk is double j && double.IsFinite(j) && j > 0) return j;
            double f = Math.Clamp(double.IsFinite(SmoothnessPercent) ? SmoothnessPercent : 50, 1, 100) / 100.0;
            if (vmax <= 0 || accel <= 0) return double.PositiveInfinity;
            return accel * accel * (2 - f) / (f * vmax);
        }
    }

    /// <summary>
    /// A rest-to-rest, jerk-limited ("S-curve") speed profile over a distance: up to a peak
    /// velocity with accel ≤ <c>accel</c> and |jerk| ≤ <c>jerkUp</c>, an optional cruise, then
    /// down to rest with decel ≤ <c>decel</c> and |jerk| ≤ <c>jerkDown</c>. Acceleration is
    /// continuous (it ramps instead of stepping), velocity never exceeds the limit, and the
    /// profile ends exactly at <see cref="Distance"/> with zero velocity and acceleration.
    /// </summary>
    /// <remarks>
    /// Each ramp is the classic three-phase S: jerk +j, constant acceleration (only when the
    /// limit is reached), jerk −j. Its velocity curve is point-symmetric about its middle, so a
    /// ramp to peak velocity vp over time T covers exactly vp·T/2. When the move is too short to
    /// reach vmax, the peak velocity is found by bisection on that closed-form distance (it only
    /// grows with vp). The decel ramp is evaluated as the accel ramp run backwards from the end,
    /// the same trick <see cref="DistanceScalarProfiler"/> uses for its decel phase.
    /// Pure math: no clock — callers pass elapsed seconds. Generates setpoints only; nothing here
    /// touches the STB loop or its timing (docs/stb-loop-timing.md).
    /// </remarks>
    internal sealed class SCurveProfile
    {
        private readonly Ramp _up, _down;
        private readonly double _cruise;

        public double Distance     { get; }
        public double PeakVelocity { get; }
        public double Duration     { get; }
        /// <summary>The larger of the two jerk limits actually used.</summary>
        public double MaxJerk      { get; }

        public SCurveProfile(double distance, double vmax, double accel, double decel, double jerkUp, double jerkDown)
        {
            Distance = Math.Max(0, distance);
            vmax  = Math.Abs(vmax);
            accel = Math.Abs(accel);
            decel = Math.Abs(decel);
            if (!(jerkUp > 0))   jerkUp   = double.PositiveInfinity;
            if (!(jerkDown > 0)) jerkDown = double.PositiveInfinity;
            MaxJerk = Math.Max(jerkUp, jerkDown);

            if (Distance <= 1e-12 || vmax <= 1e-12 || accel <= 1e-12 || decel <= 1e-12)
            {
                // Nothing to move (or degenerate limits): finished at once.
                _up = _down = default;
                Duration = 0;
                return;
            }

            double reach = Ramp.Make(vmax, accel, jerkUp).Dist + Ramp.Make(vmax, decel, jerkDown).Dist;
            double vp = vmax;
            if (reach > Distance)
            {
                double lo = 0, hi = vmax;
                for (int i = 0; i < 80; i++)
                {
                    double mid = 0.5 * (lo + hi);
                    double d = Ramp.Make(mid, accel, jerkUp).Dist + Ramp.Make(mid, decel, jerkDown).Dist;
                    if (d > Distance) hi = mid; else lo = mid;
                }
                vp = lo;
            }

            _up   = Ramp.Make(vp, accel, jerkUp);
            _down = Ramp.Make(vp, decel, jerkDown);
            PeakVelocity = vp;
            _cruise  = vp > 0 ? Math.Max(0, (Distance - _up.Dist - _down.Dist) / vp) : 0;
            Duration = _up.T + _cruise + _down.T;
        }

        /// <summary>Position at <paramref name="t"/> seconds (0 before the start, <see cref="Distance"/> after the end).</summary>
        public double Position(double t)
        {
            Sample(t, out double s, out _, out _);
            return s;
        }

        /// <summary>Position, velocity and acceleration at <paramref name="t"/> seconds.</summary>
        public void Sample(double t, out double pos, out double vel, out double acc)
        {
            if (Duration <= 0 || t >= Duration)
            {
                pos = Distance; vel = 0; acc = 0;
                return;
            }
            if (t <= 0)
            {
                pos = 0; vel = 0; acc = 0;
                return;
            }

            if (t < _up.T)
            {
                _up.At(t, out pos, out vel, out acc);
                return;
            }
            if (t < _up.T + _cruise)
            {
                pos = _up.Dist + PeakVelocity * (t - _up.T);
                vel = PeakVelocity;
                acc = 0;
                return;
            }

            // Decel: the down ramp is an up ramp run backwards from the end.
            double tau = Duration - t;
            _down.At(tau, out double sd, out double vd, out double ad);
            pos = Distance - sd;
            vel = vd;
            acc = -ad;
        }

        /// <summary>One S ramp from rest to peak velocity <see cref="Vp"/>.</summary>
        private readonly struct Ramp
        {
            public readonly double Vp;    // peak velocity reached
            public readonly double Ap;    // peak acceleration reached (≤ the limit)
            public readonly double J;     // jerk used (∞ = a step, i.e. a trapezoid ramp)
            public readonly double Tj;    // each jerk phase
            public readonly double Tc;    // constant-acceleration phase
            public readonly double T;     // whole ramp
            public readonly double Dist;  // distance covered (= Vp·T/2 by symmetry)

            private Ramp(double vp, double ap, double j, double tj, double tc)
            {
                Vp = vp; Ap = ap; J = j; Tj = tj; Tc = tc;
                T = 2 * tj + tc;
                Dist = vp * T / 2;
            }

            public static Ramp Make(double vp, double a, double j)
            {
                if (vp <= 0) return new Ramp(0, 0, j, 0, 0);
                if (double.IsPositiveInfinity(j))
                    return new Ramp(vp, a, j, 0, vp / a);                   // plain trapezoid ramp
                if (vp * j >= a * a)
                    return new Ramp(vp, a, j, a / j, vp / a - a / j);       // reaches the accel limit
                double ap = Math.Sqrt(vp * j);
                return new Ramp(vp, ap, j, ap / j, 0);                      // pure S, accel limit not reached
            }

            /// <summary>Position / velocity / acceleration <paramref name="t"/> seconds into the ramp.</summary>
            public void At(double t, out double s, out double v, out double a)
            {
                if (t <= 0) { s = 0; v = 0; a = 0; return; }
                if (t >= T) { s = Dist; v = Vp; a = 0; return; }

                if (t < Tj)
                {
                    a = J * t;
                    v = J * t * t / 2;
                    s = J * t * t * t / 6;
                    return;
                }
                double v1 = Tj > 0 ? J * Tj * Tj / 2 : 0;
                double s1 = Tj > 0 ? J * Tj * Tj * Tj / 6 : 0;
                if (t < Tj + Tc)
                {
                    double u = t - Tj;
                    a = Ap;
                    v = v1 + Ap * u;
                    s = s1 + v1 * u + Ap * u * u / 2;
                    return;
                }
                // Last jerk phase, measured back from the ramp's end (velocity is symmetric).
                double r = T - t;
                a = Tj > 0 ? J * r : 0;
                v = Vp - (Tj > 0 ? J * r * r / 2 : 0);
                s = Dist - (Vp * r - (Tj > 0 ? J * r * r * r / 6 : 0));
            }
        }
    }
}
