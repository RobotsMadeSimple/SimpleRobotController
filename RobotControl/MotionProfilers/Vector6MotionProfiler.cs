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

            if (pathLength <= 1e-9)
            {
                // No motion needed
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

            stopwatch.Restart();
        }

        // -----------------------------------------
        // Update
        // -----------------------------------------
        public Vector6 Update()
        {
            if (scalar == null)
                return start;

            double elapsed = stopwatch.Elapsed.TotalSeconds;

            double s = scalar.Calc(elapsed, out _);

            return start + delta * s;
        }
    }
}
