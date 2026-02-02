using System;

namespace Controller.RobotControl.MotionProfilers
{
    public class ScalarMotionProfiler
    {
        // Config
        private double maxSpeed;
        private double accel;
        private double decel;
        private double startVelocity;

        // Timing
        private double accelTime;
        private double cruiseTime;
        private double decelTime;
        private double totalTime;

        // Peak
        private double peakVelocity;

        public bool IsFinished { get; private set; }

        // -----------------------------------------
        // Setup (normalized 0 → 1)
        // -----------------------------------------
        public void Setup(
            double maxSpeed,
            double accel,
            double decel,
            double startVelocity=0.0
        )
        {
            this.startVelocity = Math.Max(0, startVelocity);
            this.maxSpeed = Math.Abs(maxSpeed);
            this.accel = Math.Abs(accel);
            this.decel = Math.Abs(decel);

            IsFinished = false;

            double distance = 1.0;
            double v0 = Math.Min(this.startVelocity, this.maxSpeed);

            // Can we stop before reaching 1?
            double stopDist = v0 * v0 / (2.0 * decel);
            if (stopDist >= distance)
            {
                // Decel-only profile
                peakVelocity = v0;
                accelTime = 0;
                cruiseTime = 0;
                decelTime = v0 / decel;
                totalTime = decelTime;
                return;
            }

            double accelDist = (maxSpeed * maxSpeed - v0 * v0) / (2.0 * accel);
            double decelDist = maxSpeed * maxSpeed / (2.0 * decel);

            if (accelDist + decelDist >= distance)
            {
                // Triangular
                peakVelocity = Math.Sqrt(
                    (2 * distance * accel * decel + v0 * v0 * decel) /
                    (accel + decel)
                );

                accelTime = (peakVelocity - v0) / accel;
                cruiseTime = 0;
                decelTime = peakVelocity / decel;
            }
            else
            {
                // Trapezoidal
                peakVelocity = maxSpeed;
                accelTime = (peakVelocity - v0) / accel;

                double cruiseDist = distance - accelDist - decelDist;
                cruiseTime = cruiseDist / peakVelocity;

                decelTime = peakVelocity / decel;
            }

            totalTime = accelTime + cruiseTime + decelTime;
        }

        // -----------------------------------------
        // Sample scalar (0 → 1) + velocity
        // -----------------------------------------
        public double Calc(double elapsed, out double velocity)
        {
            if (elapsed <= 0)
            {
                velocity = startVelocity;
                return 0.0;
            }

            if (elapsed >= totalTime)
            {
                IsFinished = true;
                velocity = 0.0;
                return 1.0;
            }

            double pos;
            double v;

            if (elapsed < accelTime)
            {
                // Accel
                v = startVelocity + accel * elapsed;
                pos =
                    startVelocity * elapsed +
                    0.5 * accel * elapsed * elapsed;
            }
            else if (elapsed < accelTime + cruiseTime)
            {
                // Cruise
                double t = elapsed - accelTime;
                v = peakVelocity;

                pos =
                    startVelocity * accelTime +
                    0.5 * accel * accelTime * accelTime +
                    peakVelocity * t;
            }
            else
            {
                // Decel
                double t = elapsed - accelTime - cruiseTime;
                v = peakVelocity - decel * t;

                pos =
                    startVelocity * accelTime +
                    0.5 * accel * accelTime * accelTime +
                    peakVelocity * cruiseTime +
                    peakVelocity * t -
                    0.5 * decel * t * t;
            }

            velocity = v;
            return pos;
        }
    }
}
