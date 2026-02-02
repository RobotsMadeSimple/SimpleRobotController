using System;
using System.Collections.Generic;
using System.Text;

namespace Controller.RobotControl.MotionProfilers
{
    internal class DistanceScalarProfiler
    {
        private double v, a, d, tTotal, dist;

        public void Setup(double distance, double v, double a, double d)
        {
            this.v = v;
            this.a = a;
            this.d = d;
            dist = distance;

            double ta = v / a;
            double td = v / d;

            double da = 0.5 * a * ta * ta;
            double dd = 0.5 * d * td * td;

            if (da + dd > dist)
            {
                v = Math.Sqrt(2 * dist * a * d / (a + d));
                ta = v / a;
                td = v / d;
                tTotal = ta + td;
            }
            else
            {
                double tc = (dist - da - dd) / v;
                tTotal = ta + tc + td;
            }
        }

        public double Sample(double t)
        {
            if (t <= 0) return 0;
            if (t >= tTotal) return dist;

            double ta = v / a;
            double td = v / d;

            if (t < ta)
                return 0.5 * a * t * t;

            if (t < tTotal - td)
                return 0.5 * a * ta * ta + v * (t - ta);

            double dt = t - (tTotal - td);
            return dist - 0.5 * d * dt * dt;
        }
    }

}
