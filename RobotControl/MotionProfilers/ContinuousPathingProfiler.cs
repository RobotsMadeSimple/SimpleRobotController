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
        private double lastS;

        /// <summary>Total arc length of the blended path.</summary>
        public double TotalLength => totalLength;

        /// <summary>True once the path has been fully traversed.</summary>
        public bool IsFinished => lastS >= totalLength - 1e-6;

        public ContinuousPathingProfiler(
            List<Vector6> points,
            List<double> blendRadii,   // per-waypoint corner radius; index i applies at points[i]
            double speed,
            double accel,
            double decel
        )
        {
            BuildPath(points, blendRadii);

            scalar.Setup(totalLength, speed, accel, decel);
            sw.Restart();
        }

        // Build a blended path: straight line segments joined by tangent arcs of the
        // given blend radius at each interior waypoint. A blend radius of 0 (or a
        // collinear / reversing corner) falls back to passing straight through the point.
        private void BuildPath(List<Vector6> pts, List<double> radii)
        {
            segments.Clear();
            cumulative.Clear();
            totalLength = 0;

            if (pts == null || pts.Count < 2) return;

            // Running start of the next line — advances to each blend's exit point.
            Vector6 prevEnd = pts[0];

            for (int i = 1; i < pts.Count - 1; i++)
            {
                double r = (radii != null && i < radii.Count) ? radii[i] : 0;

                Vector6 P = pts[i - 1];   // previous waypoint
                Vector6 V = pts[i];       // corner
                Vector6 B = pts[i + 1];   // next waypoint

                double inx = V.X - P.X, iny = V.Y - P.Y, inz = V.Z - P.Z;
                double outx = B.X - V.X, outy = B.Y - V.Y, outz = B.Z - V.Z;
                double lin  = Math.Sqrt(inx * inx + iny * iny + inz * inz);
                double lout = Math.Sqrt(outx * outx + outy * outy + outz * outz);

                if (r <= 0 || lin < 1e-6 || lout < 1e-6)
                {
                    AddLine(prevEnd, V);
                    prevEnd = V;
                    continue;
                }

                double dinx = inx / lin,  diny = iny / lin,  dinz = inz / lin;
                double doutx = outx / lout, douty = outy / lout, doutz = outz / lout;

                double cosang = Math.Clamp(dinx * doutx + diny * douty + dinz * doutz, -1, 1);
                double alpha  = Math.Acos(cosang);   // turn (deflection) angle

                double half = alpha / 2.0;
                double sinHalf = Math.Sin(half);
                if (alpha < 1e-4 || sinHalf < 1e-6)
                {
                    // Nearly straight — no arc needed.
                    AddLine(prevEnd, V);
                    prevEnd = V;
                    continue;
                }

                double trim   = r * Math.Tan(half);
                double rEff   = r;
                double maxTrim = 0.5 * Math.Min(lin, lout);
                if (trim > maxTrim)
                {
                    trim = maxTrim;
                    rEff = trim / Math.Tan(half);
                }

                // Tangent points — orientation is held at the corner's value through the arc.
                var tin  = new Vector6(V.X - dinx * trim,  V.Y - diny * trim,  V.Z - dinz * trim,  V.RX, V.RY, V.RZ);
                var tout = new Vector6(V.X + doutx * trim, V.Y + douty * trim, V.Z + doutz * trim, V.RX, V.RY, V.RZ);

                // Corner-interior bisector: unit(-din + dout). Zero only on a 180° reversal.
                double bx = -dinx + doutx, by = -diny + douty, bz = -dinz + doutz;
                double blen = Math.Sqrt(bx * bx + by * by + bz * bz);
                if (blen < 1e-9)
                {
                    AddLine(prevEnd, V);
                    prevEnd = V;
                    continue;
                }
                bx /= blen; by /= blen; bz /= blen;

                double dc = rEff / Math.Cos(half);   // distance from corner to arc centre
                var centre = new Vector6(V.X + bx * dc, V.Y + by * dc, V.Z + bz * dc, 0, 0, 0);

                // Rotation axis = din × dout.
                var axis = new Vector6(
                    diny * doutz - dinz * douty,
                    dinz * doutx - dinx * doutz,
                    dinx * douty - diny * doutx,
                    0, 0, 0);

                AddLine(prevEnd, tin);
                AddArc(tin, tout, centre, axis, rEff, alpha);
                prevEnd = tout;
            }

            AddLine(prevEnd, pts[^1]);
        }

        private void AddLine(Vector6 a, Vector6 b)
        {
            var seg = new LineSegment(a, b);
            if (seg.Length < 1e-9) return;
            segments.Add(seg);
            totalLength += seg.Length;
            cumulative.Add(totalLength);
        }

        private void AddArc(Vector6 a, Vector6 b, Vector6 centre, Vector6 axis, double radius, double sweep)
        {
            var seg = new ArcSegment(a, b, centre, axis, radius, sweep);
            if (seg.Length < 1e-9) return;
            segments.Add(seg);
            totalLength += seg.Length;
            cumulative.Add(totalLength);
        }

        public Vector6 Loop()
        {
            double s = scalar.Sample(sw.Elapsed.TotalSeconds);
            lastS = s;

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
