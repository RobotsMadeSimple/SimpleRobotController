using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision.Tools
{
    using Point = OpenCvSharp.Point;

    // ── caliper ─────────────────────────────────────────────────────────────────

    /// <summary>
    /// Edges along a rectangle: its pixels are averaged across its height into one profile
    /// running along its width (left → right in the region's own frame). mode single = one
    /// edge chosen by pick; pair = an edge of the polarity followed by one of the opposite
    /// polarity (a bar), optionally pairWidth ± pairTolerance px apart; all = every edge.
    /// </summary>
    internal sealed class CaliperTool : IVisionTool
    {
        public void Run(ToolContext c)
        {
            var p = c.P;
            string polarity = p.Str("polarity", "any"), mode = p.Str("mode", "single"), pick = p.Str("pick", "strongest");
            double contrast = p.Num("contrast", 20), smoothing = p.Num("smoothing", 1);

            var (patch, toImage) = ToolMath.Unroll(c.Gray, c.Region);
            double[] profile;
            double midY;
            using (patch)
            {
                using var row = new Mat();
                Cv2.Reduce(patch, row, ReduceDimension.Row, ReduceTypes.Avg, MatType.CV_64F);
                int cols = row.Cols;
                profile = new double[cols];
                for (int x = 0; x < cols; x++) profile[x] = row.At<double>(0, x);
                midY = patch.Height / 2.0;
            }
            double middle = profile.Length / 2.0;
            Point2d At(double pos) => ToolMath.Map(toImage, pos, midY);

            if (mode == "pair")
            {
                // Edges of either sign; a pair starts with the asked polarity.
                var edges = ToolMath.Edges(profile, smoothing, contrast, "any");
                double want = p.Num("pairWidth", 0), tol = p.Num("pairTolerance", 5);
                var pairs = new List<(double A, double B, double Strength)>();
                for (int i = 0; i < edges.Count; i++)
                {
                    var a = edges[i];
                    if (polarity == "darkToLight" && a.Contrast <= 0) continue;
                    if (polarity == "lightToDark" && a.Contrast >= 0) continue;
                    for (int j = i + 1; j < edges.Count; j++)
                    {
                        var b = edges[j];
                        if (Math.Sign(b.Contrast) == Math.Sign(a.Contrast)) continue;
                        double width = b.Pos - a.Pos;
                        if (want > 0 && Math.Abs(width - want) > tol) continue;
                        pairs.Add((a.Pos, b.Pos, Math.Abs(a.Contrast) + Math.Abs(b.Contrast)));
                        break;   // the nearest closing edge
                    }
                }
                c.Out.Set("count", pairs.Count);
                if (pairs.Count == 0) { c.Fail("no edge pair found"); return; }
                var best = pick switch
                {
                    "first"  => pairs[0],
                    "last"   => pairs[^1],
                    "center" => pairs.OrderBy(q => Math.Abs((q.A + q.B) / 2 - middle)).First(),
                    _        => pairs.OrderByDescending(q => q.Strength).First(),
                };
                var pa = At(best.A); var pb = At(best.B);
                var mid = At((best.A + best.B) / 2);
                c.Out.Set("x", mid.X);
                c.Out.Set("y", mid.Y);
                c.Out.Set("width", Geo.Dist(pa, pb));
                c.Out.Points.Add(new PxPoint(pa.X, pa.Y));
                c.Out.Points.Add(new PxPoint(pb.X, pb.Y));
                return;
            }

            var all = ToolMath.Edges(profile, smoothing, contrast, polarity);
            c.Out.Set("count", all.Count);
            if (all.Count == 0) { c.Fail("no edge found"); return; }
            if (mode == "all")
            {
                foreach (var e in all) { var q = At(e.Pos); c.Out.Points.Add(new PxPoint(q.X, q.Y)); }
                var f = At(all[0].Pos);
                c.Out.Set("x", f.X);
                c.Out.Set("y", f.Y);
                return;
            }
            var pickE = pick switch
            {
                "first"  => all[0],
                "last"   => all[^1],
                "center" => all.OrderBy(e => Math.Abs(e.Pos - middle)).First(),
                _        => all.OrderByDescending(e => Math.Abs(e.Contrast)).First(),
            };
            var pt = At(pickE.Pos);
            c.Out.Set("x", pt.X);
            c.Out.Set("y", pt.Y);
            c.Out.Set("contrast", pickE.Contrast);
            c.Out.Points.Add(new PxPoint(pt.X, pt.Y));
        }
    }

    // ── distance / angle ────────────────────────────────────────────────────────

    /// <summary>Distance between two tools' points (pointPoint) or from a point to a line (pointLine), px.</summary>
    internal sealed class DistanceTool : IVisionTool
    {
        public bool UsesRegion => false;
        public IReadOnlyList<string> Sources(VisionTool tool) => tool.P.StrList("sources");

        public void Run(ToolContext c)
        {
            var ids = c.P.StrList("sources");
            if (ids.Count < 2) { c.Fail("needs two sources"); return; }
            var a = c.Source(ids[0]); var b = c.Source(ids[1]);
            if (a == null || b == null) { c.Fail("a source tool has no result"); return; }
            if (Geometry.PointOf(a) is not { } pa) { c.Fail("source 1 has no point"); return; }
            Point2d pb;
            if (c.P.Str("mode", "pointPoint") == "pointLine")
            {
                if (Geometry.LineOf(b) is not { } l) { c.Fail("source 2 has no line"); return; }
                double dx = l.X2 - l.X1, dy = l.Y2 - l.Y1, len2 = dx * dx + dy * dy;
                if (len2 < 1e-12) { c.Fail("source 2's line has no length"); return; }
                double t = ((pa.X - l.X1) * dx + (pa.Y - l.Y1) * dy) / len2;
                pb = new Point2d(l.X1 + t * dx, l.Y1 + t * dy);
            }
            else
            {
                if (Geometry.PointOf(b) is not { } q) { c.Fail("source 2 has no point"); return; }
                pb = q;
            }
            c.Out.Set("distance", Geo.Dist(pa, pb));
            c.Out.Set("dx", pb.X - pa.X);
            c.Out.Set("dy", pb.Y - pa.Y);
            c.Out.Points.Add(new PxPoint(pa.X, pa.Y));
            c.Out.Points.Add(new PxPoint(pb.X, pb.Y));
            c.Out.Lines.Add(new PxLine(pa.X, pa.Y, pb.X, pb.Y));
        }
    }

    /// <summary>A line's angle against the image x-axis, or the angle from line A to line B ((−180, 180]).</summary>
    internal sealed class AngleTool : IVisionTool
    {
        public bool UsesRegion => false;
        public IReadOnlyList<string> Sources(VisionTool tool) => tool.P.StrList("sources");

        public void Run(ToolContext c)
        {
            var ids = c.P.StrList("sources");
            if (ids.Count < 1) { c.Fail("needs a line source"); return; }
            if (c.Source(ids[0]) is not { } a || Geometry.LineOf(a) is not { } la) { c.Fail("source 1 has no line"); return; }
            double angA = Geo.AngleDeg(la.X2 - la.X1, la.Y2 - la.Y1);
            c.Out.Lines.Add(la);
            if (ids.Count < 2) { c.Out.Set("angle", angA); return; }
            if (c.Source(ids[1]) is not { } b || Geometry.LineOf(b) is not { } lb) { c.Fail("source 2 has no line"); return; }
            c.Out.Lines.Add(lb);
            c.Out.Set("angle", Geo.Wrap180(Geo.AngleDeg(lb.X2 - lb.X1, lb.Y2 - lb.Y1) - angA));
        }
    }

    // ── corners / lines ─────────────────────────────────────────────────────────

    /// <summary>Feature corners (Shi-Tomasi or Harris) in the region, strongest first.</summary>
    internal sealed class CornersTool : IVisionTool
    {
        public void Run(ToolContext c)
        {
            var p = c.P;
            var (mask, box) = ToolMath.RegionMask(c.Region, c.W, c.H);
            using (mask)
            {
                if (box.Width < 3 || box.Height < 3) { c.Out.Set("count", 0); c.Fail("region is outside the image"); return; }
                using var gray = new Mat(c.Gray, box);
                var pts = Cv2.GoodFeaturesToTrack(gray, Math.Max(1, p.Int("maxCorners", 50)), Math.Clamp(p.Num("quality", 0.01), 1e-4, 1),
                    Math.Max(0, p.Num("minDistance", 10)), mask, Math.Max(2, p.Int("blockSize", 3)), p.Str("method", "shiTomasi") == "harris", 0.04);
                c.Out.Set("count", pts.Length);
                c.Out.Points.AddRange(pts.Select(q => new PxPoint(q.X + box.X, q.Y + box.Y)));
            }
        }
    }

    /// <summary>Straight segments (Canny + probabilistic Hough) in the region, filtered by angle (0–180, undirected).</summary>
    internal sealed class LinesTool : IVisionTool
    {
        private static double Len(LineSegmentPoint s) => Math.Sqrt((double)(s.P2.X - s.P1.X) * (s.P2.X - s.P1.X) + (double)(s.P2.Y - s.P1.Y) * (s.P2.Y - s.P1.Y));

        public void Run(ToolContext c)
        {
            var p = c.P;
            var (mask, box) = ToolMath.RegionMask(c.Region, c.W, c.H);
            using (mask)
            {
                if (box.Width < 3 || box.Height < 3) { c.Out.Set("count", 0); c.Fail("region is outside the image"); return; }
                using var gray = new Mat(c.Gray, box);
                using var edges = new Mat();
                Cv2.Canny(gray, edges, p.Num("canny1", 50), p.Num("canny2", 150));
                Cv2.BitwiseAnd(edges, mask, edges);
                var segs = Cv2.HoughLinesP(edges, 1, Math.PI / 180, Math.Max(1, p.Int("houghThreshold", 50)),
                    p.Num("minLineLength", 30), p.Num("maxLineGap", 10));
                double lo = p.Num("minAngle", 0), hi = p.Num("maxAngle", 180);
                foreach (var s in segs.OrderByDescending(s => Len(s)))
                {
                    double a = Geo.AngleDeg(s.P2.X - s.P1.X, s.P2.Y - s.P1.Y);
                    a = ((a % 180) + 180) % 180;
                    bool inRange = lo <= hi ? a >= lo && a <= hi : a >= lo || a <= hi;
                    if (!inRange) continue;
                    c.Out.Lines.Add(new PxLine(s.P1.X + box.X, s.P1.Y + box.Y, s.P2.X + box.X, s.P2.Y + box.Y));
                    c.Out.Items.Add(new Dictionary<string, object>
                    {
                        ["x1"] = s.P1.X + box.X, ["y1"] = s.P1.Y + box.Y, ["x2"] = s.P2.X + box.X, ["y2"] = s.P2.Y + box.Y,
                        ["angle"] = Math.Round(a, 2), ["length"] = Math.Round(Len(s), 1),
                    });
                }
                c.Out.Set("count", c.Out.Lines.Count);
            }
        }
    }
}
