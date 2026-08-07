using System;
using System.Diagnostics;

namespace Controller.RobotControl.MotionProfilers
{
    public class Vector6MotionProfiler
    {
        private readonly ScalarMotionProfiler? scalar;
        private readonly Stopwatch stopwatch = new();

        private readonly Vector6 start;
        public readonly Vector6 delta;

        private readonly double pathLength;

        public bool IsFinished =>
            scalar == null || scalar.IsFinished;

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
            if (scalar == null)
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

            double elapsed = stopwatch.Elapsed.TotalSeconds;

            double s = scalar.Calc(elapsed, out _);

            return start + delta * s;
        }
    }
}
