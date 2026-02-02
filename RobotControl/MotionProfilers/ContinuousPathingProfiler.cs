using System;
using System.Collections.Generic;
using System.Text;

namespace Controller.RobotControl.MotionProfilers
{
    using System.Diagnostics;

    internal class ContinuousPathingProfiler
    {
        private List<PathSegment> segments = new();
        private List<double> cumulative = new();
        private DistanceScalarProfiler scalar = new();
        private Stopwatch sw = Stopwatch.StartNew();

        private double totalLength;

        public ContinuousPathingProfiler(
            List<Vector6> points,
            double blendRadius,
            double speed,
            double accel,
            double decel
        )
        {
            BuildPath(points, blendRadius);

            scalar.Setup(totalLength, speed, accel, decel);
            sw.Restart();
        }

        private void BuildPath(List<Vector6> pts, double r)
        {
            segments.Clear();
            cumulative.Clear();
            totalLength = 0;

            for (int i = 0; i < pts.Count - 1; i++)
            {
                var seg = new LineSegment(pts[i], pts[i + 1]);
                segments.Add(seg);
                totalLength += seg.Length;
                cumulative.Add(totalLength);
            }
        }

        public Vector6 Loop()
        {
            double s = scalar.Sample(sw.Elapsed.TotalSeconds);

            for (int i = 0; i < segments.Count; i++)
            {
                double prev = i == 0 ? 0 : cumulative[i - 1];
                if (s <= cumulative[i])
                    return segments[i].Sample(s - prev);
            }

            return segments[^1].Sample(segments[^1].Length);
        }
    }

}
