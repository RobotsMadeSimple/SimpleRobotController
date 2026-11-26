using System;
using System.Diagnostics;

namespace Controller.RobotControl.MotionProfilers
{
    public class Vector6MotionProfiler
    {
        private readonly ScalarMotionProfiler? scalar;
        // Set instead of `scalar` when the move runs a jerk-limited S-curve (normalised 0 → 1).
        private readonly SCurveProfile? scurve;
        private bool scurveFinished;
        private readonly Stopwatch stopwatch = new();

        private readonly Vector6 start;
        public readonly Vector6 delta;

        private readonly double pathLength;

        public bool IsFinished =>
            scurve != null ? scurveFinished : scalar == null || scalar.IsFinished;

        /// <summary>Normalised path position (0 → 1) of the last sample — conveyor tracking ramps its
        /// belt offset in over this for a move that starts untracked.</summary>
        public double Progress { get; private set; }

        /// <summary>Planned duration of the move (s); 0 for a no-motion profile.</summary>
        public double Duration => scurve?.Duration ?? scalar?.TotalTime ?? 0;

        // -----------------------------------------
        // Setup
        // -----------------------------------------
        public Vector6MotionProfiler(
            Vector6 startPose,
            Vector6 endPose,
            double speed,
            double accel,
            double decel,
            double startScalarVelocity = 0.0
        ) : this(startPose, endPose, speed, accel, decel, MotionProfileSettings.Trapezoid, startScalarVelocity)
        {
        }

        /// <summary>
        /// A straight-line (Cartesian or joint-space) move with the given speed profile. A
        /// trapezoid is exactly the classic profile above; an S-curve swaps the scalar for a
        /// jerk-limited <see cref="SCurveProfile"/> over the same normalised 0 → 1 path.
        /// </summary>
        internal Vector6MotionProfiler(
            Vector6 startPose,
            Vector6 endPose,
            double speed,
            double accel,
            double decel,
            MotionProfileSettings profile,
            double startScalarVelocity = 0.0
        )
        {
            start = startPose;
            delta = endPose - startPose;

            pathLength = delta.Length();

            if (pathLength <= 1e-9 || speed <= 1e-9)
            {
                // No motion needed (zero distance or zero speed)
                scalar = null;
                stopwatch.Reset();
                return;
            }

            double scalarSpeed = speed / pathLength;
            double scalarAccel = accel / pathLength;
            double scalarDecel = decel / pathLength;

            if (profile.IsSCurve)
            {
                // Jerk in path units (configured mm/s³ or units/s³) scales like the other limits.
                double jerkUp   = profile.Jerk is > 0 ? profile.JerkFor(speed, accel) / pathLength : profile.JerkFor(scalarSpeed, scalarAccel);
                double jerkDown = profile.Jerk is > 0 ? profile.JerkFor(speed, decel) / pathLength : profile.JerkFor(scalarSpeed, scalarDecel);
                scurve = new SCurveProfile(1.0, scalarSpeed, scalarAccel, scalarDecel, jerkUp, jerkDown);
                scurveFinished = scurve.Duration <= 0;
                return;
            }

            scalar = new ScalarMotionProfiler();
            scalar.Setup(
                maxSpeed: scalarSpeed,
                accel: scalarAccel,
                decel: scalarDecel,
                startVelocity: startScalarVelocity
            );

            // Clock intentionally NOT started here — it is started lazily on the
            // first Update() so the move cannot begin partway down the path. See Update().
        }

        // -----------------------------------------
        // Update
        // -----------------------------------------
        public Vector6 Update()
        {
            if (scalar == null && scurve == null)
                return start;

            // Start the move clock on the FIRST sample, not at construction. The clock
            // measures how far along the move we should be; if it started in the ctor,
            // any gap before the first sample (a busy control-loop tick, a heavy program
            // step on the same shared thread, a GC pause) would make the first sample
            // land partway down the path — the robot lurches into the middle of the move
            // instead of easing off the start. Lazy start makes the first sample
            // elapsed≈0, so it returns `start` exactly.
            if (!stopwatch.IsRunning)
                stopwatch.Restart();

            return SampleAt(stopwatch.Elapsed.TotalSeconds);
        }

        /// <summary>
        /// The pose <paramref name="elapsed"/> seconds into the move. <see cref="Update"/> is this
        /// with the move clock; tests sample it directly (the trapezoid output is pinned by
        /// Golden/motionprofile-trapezoid.golden.json).
        /// </summary>
        internal Vector6 SampleAt(double elapsed)
        {
            if (scurve != null)
            {
                if (elapsed >= scurve.Duration) scurveFinished = true;
                double p = scurve.Position(elapsed);
                Progress = scurveFinished ? 1 : p;
                return start + delta * p;
            }

            if (scalar == null)
            {
                Progress = 1;
                return start;
            }

            double s = scalar.Calc(elapsed, out _);
            Progress = scalar.IsFinished ? 1 : s;

            return start + delta * s;
        }
    }
}
