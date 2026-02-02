using System;
using System.Collections.Generic;
using System.Text;

namespace Controller.RobotControl.MotionProfilers
{
    internal class AxisMotionProfiler
    {
        public double Position { get; set; }
        public double Velocity { get; private set; }

        private double target;
        private double accel;
        private double decel;

        private double accelTime;
        private double cruiseTime;
        private double decelTime;
        private double totalTime;

        private double direction;
        private double peakVelocity;

        public void Setup(
            double currentPos,
            double currentVel,
            double targetPos,
            double accel,
            double decel,
            double maxVelocity,
            double syncTime
        )
        {
            Position = currentPos;
            Velocity = currentVel;
            target = targetPos;
            this.accel = accel;
            this.decel = decel;

            double dist = target - Position;
            direction = Math.Sign(dist);
            dist = Math.Abs(dist);

            double v0 = currentVel * direction;

            // Solve required peak velocity to hit target in syncTime
            // Using symmetric accel/decel for stability
            peakVelocity = 2 * dist / syncTime - v0;
            peakVelocity = Math.Min(peakVelocity, maxVelocity);

            accelTime = Math.Max(0, (peakVelocity - v0) / accel);
            decelTime = peakVelocity / decel;
            cruiseTime = syncTime - accelTime - decelTime;

            if (cruiseTime < 0)
            {
                // Triangle fallback
                peakVelocity = Math.Sqrt(
                    (2 * dist * accel * decel + v0 * v0 * decel) /
                    (accel + decel)
                );

                accelTime = (peakVelocity - v0) / accel;
                cruiseTime = 0;
                decelTime = peakVelocity / decel;
            }

            totalTime = accelTime + cruiseTime + decelTime;
        }

        public void Update(double dt)
        {
            if (dt <= 0) return;

            if (dt < accelTime)
            {
                Velocity += accel * dt * direction;
            }
            else if (dt < accelTime + cruiseTime)
            {
                Velocity = peakVelocity * direction;
            }
            else if (dt < totalTime)
            {
                Velocity -= decel * dt * direction;
            }
            else
            {
                Velocity = 0;
                Position = target;
                return;
            }

            Position += Velocity * dt;
        }

        public void ForceTarget()
        {
            Position = target;
            Velocity = 0;
        }
    }

}
