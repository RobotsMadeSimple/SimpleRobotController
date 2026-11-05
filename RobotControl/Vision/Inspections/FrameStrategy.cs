using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision.Inspections
{
    using Point = OpenCvSharp.Point;

    internal sealed class FrameDetection
    {
        public required FrameResult Result { get; init; }
        /// <summary>Pixel corners TL, TR, BR, BL; null when not found.</summary>
        public Point2f[]? Corners { get; init; }
    }

    /// <summary>
    /// Finds a four-cornered frame — a board's outline — in the search zone, three ways:
    /// <list type="bullet">
    /// <item><b>contour</b>: Canny edges → external contours → the largest convex 4-gon.</item>
    /// <item><b>lines</b>: Canny → Hough segments → two direction families → the two outermost
    /// lines of each → their four intersections. Copes with a frame whose corners are hidden
    /// (a hand, a piece) as long as part of every edge is visible.</item>
    /// <item><b>corners</b>: Shi-Tomasi corners → convex hull → the four extreme hull points.</item>
    /// </list>
    /// Corners always come back ordered TL, TR, BR, BL by x+y / x−y, the order zones use.
    /// </summary>
    internal sealed class FrameStrategy : IInspectionStrategy<FrameInspection, FrameDetection>
    {
        public FrameDetection Run(FrameContext ctx, FrameInspection insp, VisionZone? zone)
        {
            int w = ctx.Width, h = ctx.Height;
            var mask = ctx.ZoneMask(zone);
            Point2f[]? quad = (insp.Method ?? "contour").Trim().ToLowerInvariant() switch
            {
                "lines"   => ByLines(ctx, insp, zone, mask),
                "corners" => ByCorners(ctx, insp, mask),
                _         => ByContour(ctx, insp, mask),
            };
            if (quad != null) quad = Order(quad);
            if (quad != null && !ZoneGeometry.Convex(quad)) quad = null;

            var result = new FrameResult { InspectionId = insp.Id, Name = insp.Name, Method = insp.Method ?? "contour" };
            if (quad != null)
            {
                result.Found   = true;
                result.Corners = quad.Select(p => new[] { Math.Round(p.X / (double)w, 4), Math.Round(p.Y / (double)h, 4) }).ToList();
                result.Angle   = Math.Round(Math.Atan2(quad[1].Y - quad[0].Y, quad[1].X - quad[0].X) * 180 / Math.PI, 2);
                result.AreaPct = Math.Round(Math.Abs(Cv2.ContourArea(quad)) * 100.0 / (w * (double)h), 2);
            }
            return new FrameDetection { Result = result, Corners = quad };
        }

        /// <summary>Sorts four points TL, TR, BR, BL; null when two roles land on the same point.</summary>
        internal static Point2f[]? Order(IReadOnlyList<Point2f> pts)
        {
            if (pts.Count < 4) return null;
            var tl = pts.OrderBy(p => p.X + p.Y).First();
            var br = pts.OrderByDescending(p => p.X + p.Y).First();
            var tr = pts.OrderByDescending(p => p.X - p.Y).First();
            var bl = pts.OrderBy(p => p.X - p.Y).First();
            var q = new[] { tl, tr, br, bl };
            return q.Distinct().Count() == 4 ? q : null;
        }

        private static Mat MaskedEdges(FrameContext ctx, double lo, double hi, Mat mask)
        {
            var edges = new Mat();
            Cv2.BitwiseAnd(ctx.Canny(lo, hi), mask, edges);
            return edges;
        }

        private static Point2f[]? ByContour(FrameContext ctx, FrameInspection insp, Mat mask)
        {
            using var edges = MaskedEdges(ctx, insp.CannyLow, insp.CannyHigh, mask);
            using var kernel = Cv2.GetStructuringElement(MorphShapes.Rect, new Size(3, 3));
            Cv2.Dilate(edges, edges, kernel);   // closes one-pixel breaks in the outline
            Cv2.FindContours(edges, out Point[][] contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);

            double minArea = Cv2.CountNonZero(mask) * Math.Max(0, insp.MinAreaPct) / 100.0;
            Point2f[]? best = null;
            Point[]? bestContour = null;
            double bestArea = 0;
            foreach (var c in contours)
            {
                double peri = Cv2.ArcLength(c, true);
                var approx = Cv2.ApproxPolyDP(c, Math.Max(0.001, insp.ApproxEpsilon) * peri, true);
                if (approx.Length != 4 || !Cv2.IsContourConvex(approx)) continue;
                double area = Math.Abs(Cv2.ContourArea(approx));
                if (area < minArea || area <= bestArea) continue;
                bestArea = area;
                bestContour = c;
                best = approx.Select(p => new Point2f(p.X, p.Y)).ToArray();
            }
            return best == null ? null : Refine(best, bestContour!) ?? best;
        }

        /// <summary>
        /// ApproxPolyDP's vertices are contour points, so on a large epsilon a "corner" can sit
        /// tens of pixels along an edge. Fit a line to each side's contour points (ignoring the
        /// ends, where the outline rounds off) and intersect neighbouring sides instead.
        /// </summary>
        internal static Point2f[]? Refine(Point2f[] quad, Point[] contour)
        {
            var sides = new List<Point2f>[4];
            for (int i = 0; i < 4; i++) sides[i] = new List<Point2f>();
            foreach (var p in contour)
            {
                int bestSide = -1; double bestD = double.MaxValue, bestT = 0;
                for (int i = 0; i < 4; i++)
                {
                    var a = quad[i]; var b = quad[(i + 1) % 4];
                    double dx = b.X - a.X, dy = b.Y - a.Y, len2 = dx * dx + dy * dy;
                    if (len2 < 1) continue;
                    double t = ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / len2;
                    double tc = Math.Clamp(t, 0, 1);
                    double ex = a.X + tc * dx - p.X, ey = a.Y + tc * dy - p.Y, d = ex * ex + ey * ey;
                    if (d < bestD) { bestD = d; bestSide = i; bestT = t; }
                }
                if (bestSide >= 0 && bestT > 0.1 && bestT < 0.9) sides[bestSide].Add(new Point2f(p.X, p.Y));
            }
            var lines = new Line2[4];
            for (int i = 0; i < 4; i++)
            {
                if (sides[i].Count < 2) return null;
                var l = Cv2.FitLine(sides[i], DistanceTypes.L2, 0, 0.01, 0.01);
                lines[i] = new Line2(new Point2f((float)l.X1, (float)l.Y1), new Point2f((float)l.Vx, (float)l.Vy));
            }
            var outQuad = new Point2f[4];
            for (int i = 0; i < 4; i++)
            {
                // Corner i joins side i-1 (ending at it) and side i (starting from it).
                var p = Intersect(lines[(i + 3) % 4], lines[i]);
                if (p == null) return null;
                // A refined corner should stay near the rough one.
                if (Math.Abs(p.Value.X - quad[i].X) > 60 || Math.Abs(p.Value.Y - quad[i].Y) > 60) return null;
                outQuad[i] = p.Value;
            }
            return outQuad;
        }

        private static Point2f[]? ByCorners(FrameContext ctx, FrameInspection insp, Mat mask)
        {
            var pts = Cv2.GoodFeaturesToTrack(ctx.Gray, Math.Max(4, insp.MaxCorners), Math.Max(1e-4, insp.CornerQuality),
                Math.Max(1, insp.MinCornerDistance), mask, 3, false, 0.04);
            if (pts.Length < 4) return null;
            var hull = Cv2.ConvexHull(pts);
            return Order(hull);
        }

        internal readonly record struct Line2(Point2f P, Point2f D);   // point + unit direction

        // Scored-quad search limits: candidate lines kept per direction, and the share of a
        // side that must sit on edge pixels (a hand or a piece over a corner still passes).
        private const int    MaxLinesPerFamily = 20;
        private const double MinSideSupport    = 0.6;

        /// <summary>
        /// Edges → Hough segments → two direction families → each family's segments merged into
        /// whole lines → every pair of lines from one family with every pair from the other makes
        /// a candidate quad → keep the largest quad whose four sides are really on edges.
        ///
        /// Taking the two outermost lines of each family is not enough on a real table: wood
        /// grain or a ruler gives long straight edges outside the board, and pieces cut the
        /// board's own lines into short pieces. A grain line never closes a quad with the
        /// board's sides — the sides' extensions to it are not on any edge — so it scores low.
        /// </summary>
        private static Point2f[]? ByLines(FrameContext ctx, FrameInspection insp, VisionZone? zone, Mat mask)
        {
            int w = ctx.Width, h = ctx.Height;
            var bounds = zone != null ? ZoneGeometry.ZoneBounds(zone.Geometry, w, h) : new Rect(0, 0, w, h);
            double minLen = Math.Max(5, Math.Min(bounds.Width, bounds.Height) * Math.Max(0, insp.MinLineLengthPct) / 100.0);
            using var edges = MaskedEdges(ctx, insp.CannyLow, insp.CannyHigh, mask);
            // Hough runs on edges thickened to 3 px: a long one-pixel edge a fraction of a degree
            // off the 1° angle grid otherwise breaks into short pieces under minLen and vanishes,
            // while perfectly level clutter (wood grain) survives.
            using var thick = new Mat();
            using (var k3 = Cv2.GetStructuringElement(MorphShapes.Rect, new Size(3, 3)))
                Cv2.Dilate(edges, thick, k3);
            var segs = Cv2.HoughLinesP(thick, 1, Math.PI / 180, Math.Max(1, insp.HoughThreshold), minLen, Math.Max(0, insp.MaxLineGap));
            if (segs.Length < 4) return null;

            var (famA, famB) = SplitDirections(segs);
            var linesA = MergeLines(famA);
            var linesB = MergeLines(famB);
            if (linesA.Count < 2 || linesB.Count < 2) return null;

            using var support = new Mat();
            using (var kernel = Cv2.GetStructuringElement(MorphShapes.Rect, new Size(5, 5)))
                Cv2.Dilate(edges, support, kernel);   // a line a pixel or two off its edge still counts

            double minArea = Cv2.CountNonZero(mask) * Math.Max(0, insp.MinAreaPct) / 100.0;
            Point2f[]? best = null;
            double bestArea = 0;
            for (int i = 0; i < linesA.Count; i++)
            for (int j = i + 1; j < linesA.Count; j++)
            for (int k = 0; k < linesB.Count; k++)
            for (int l = k + 1; l < linesB.Count; l++)
            {
                var corners = new List<Point2f>(4);
                foreach (var la in new[] { linesA[i], linesA[j] })
                foreach (var lb in new[] { linesB[k], linesB[l] })
                    if (Intersect(la, lb) is { } p) corners.Add(p);
                if (corners.Count != 4) continue;
                var q = Order(corners);
                if (q == null || !ZoneGeometry.Convex(q)) continue;
                if (q.Any(p => p.X < -2 || p.X > w + 2 || p.Y < -2 || p.Y > h + 2)) continue;
                double area = Math.Abs(Cv2.ContourArea(q));
                if (area < minArea || area <= bestArea) continue;
                if (Enumerable.Range(0, 4).Any(s => SideSupport(support, q[s], q[(s + 1) % 4]) < MinSideSupport)) continue;
                // A frame's corners are real: lines extended past the board into empty table
                // (to meet a grain line) leave the stretch next to the corner bare. One corner may
                // be hidden (a hand), so three of four must be on edges.
                if (Enumerable.Range(0, 4).Count(i => CornerSupported(support, q, i)) < 3) continue;
                best = q; bestArea = area;
            }
            return best == null ? null : RefineSides(edges, best) ?? best;
        }

        /// <summary>
        /// Re-fits each side of a found quad to the edge pixels in a narrow band around it, and
        /// intersects the refits: Hough lines are quantised to a degree and a pixel, and a corner
        /// a few pixels off moves every aligned grid cell with it.
        /// </summary>
        private static Point2f[]? RefineSides(Mat edges, Point2f[] q)
        {
            const double band = 3;
            var lines = new Line2[4];
            for (int s = 0; s < 4; s++)
            {
                var a = q[s]; var b = q[(s + 1) % 4];
                double dx = b.X - a.X, dy = b.Y - a.Y, len = Math.Sqrt(dx * dx + dy * dy);
                if (len < 10) return null;
                double ux = dx / len, uy = dy / len;
                var pts = new List<Point2f>();
                int x0 = (int)Math.Max(0, Math.Min(a.X, b.X) - band), x1 = (int)Math.Min(edges.Width - 1, Math.Max(a.X, b.X) + band);
                int y0 = (int)Math.Max(0, Math.Min(a.Y, b.Y) - band), y1 = (int)Math.Min(edges.Height - 1, Math.Max(a.Y, b.Y) + band);
                for (int y = y0; y <= y1; y++)
                for (int x = x0; x <= x1; x++)
                {
                    if (edges.At<byte>(y, x) == 0) continue;
                    double rx = x - a.X, ry = y - a.Y;
                    double t = rx * ux + ry * uy, d = Math.Abs(rx * uy - ry * ux);
                    if (d <= band && t > 0.08 * len && t < 0.92 * len) pts.Add(new Point2f(x, y));
                }
                if (pts.Count < 10) return null;
                var fl = Cv2.FitLine(pts, DistanceTypes.Huber, 0, 0.01, 0.01);
                lines[s] = new Line2(new Point2f((float)fl.X1, (float)fl.Y1), new Point2f((float)fl.Vx, (float)fl.Vy));
            }
            var outQ = new Point2f[4];
            for (int i = 0; i < 4; i++)
            {
                if (Intersect(lines[(i + 3) % 4], lines[i]) is not { } p) return null;
                if (Math.Abs(p.X - q[i].X) > 8 || Math.Abs(p.Y - q[i].Y) > 8) return null;   // refits only nudge
                outQ[i] = p;
            }
            return outQ;
        }

        /// <summary>
        /// True when corner i is a real corner: both sides lie on edges for the stretch next to
        /// it, and neither carries on past it — a board edge ends at the board's corner, while a
        /// table-grain line it was intersected with keeps going.
        /// </summary>
        private static bool CornerSupported(Mat edges, Point2f[] q, int i)
        {
            var c = q[i];
            foreach (var other in new[] { q[(i + 1) % 4], q[(i + 3) % 4] })
            {
                var near   = new Point2f(c.X + (other.X - c.X) * 0.15f, c.Y + (other.Y - c.Y) * 0.15f);
                var beyond = new Point2f(c.X - (other.X - c.X) * 0.12f, c.Y - (other.Y - c.Y) * 0.12f);
                if (SideSupport(edges, c, near, 0.1, 0.9) < 0.75) return false;
                // A line that carries on gives one long unbroken run; lines merely crossing the
                // stretch (grain, a ruler) give short blips.
                if (LongestRun(edges, c, beyond, 0.25, 1.0) > 0.5) return false;
            }
            return true;
        }

        /// <summary>The longest unbroken run of edge pixels along a→b (between t0..t1), as a share of the samples.</summary>
        private static double LongestRun(Mat edges, Point2f a, Point2f b, double t0, double t1)
        {
            const int samples = 40;
            int run = 0, best = 0, n = 0;
            for (int i = 0; i < samples; i++)
            {
                double t = t0 + (t1 - t0) * i / (samples - 1);
                int x = (int)Math.Round(a.X + (b.X - a.X) * t), y = (int)Math.Round(a.Y + (b.Y - a.Y) * t);
                if (x < 0 || y < 0 || x >= edges.Width || y >= edges.Height) { run = 0; continue; }
                n++;
                run = edges.At<byte>(y, x) != 0 ? run + 1 : 0;
                best = Math.Max(best, run);
            }
            return n == 0 ? 0 : best / (double)n;
        }

        /// <summary>Share of points along a→b (between fractions t0..t1) that land on an edge pixel.</summary>
        private static double SideSupport(Mat edges, Point2f a, Point2f b, double t0 = 0.05, double t1 = 0.95)
        {
            const int samples = 40;
            int on = 0, n = 0;
            for (int i = 0; i < samples; i++)
            {
                double t = t0 + (t1 - t0) * i / (samples - 1);
                int x = (int)Math.Round(a.X + (b.X - a.X) * t), y = (int)Math.Round(a.Y + (b.Y - a.Y) * t);
                if (x < 0 || y < 0 || x >= edges.Width || y >= edges.Height) continue;
                n++;
                if (edges.At<byte>(y, x) != 0) on++;
            }
            return n == 0 ? 0 : on / (double)n;
        }

        /// <summary>
        /// Segments of one direction family merged into whole lines: segments at about the same
        /// angle and offset are pieces of one line (cut by pieces, a hand, gaps). Longest first.
        /// </summary>
        internal static List<Line2> MergeLines(List<LineSegmentPoint> fam)
        {
            double Len(LineSegmentPoint s) => Math.Sqrt(Math.Pow(s.P2.X - s.P1.X, 2) + Math.Pow(s.P2.Y - s.P1.Y, 2));
            var groups = new List<(double Angle, double Offset, List<LineSegmentPoint> Segs, double Total)>();
            foreach (var sgm in fam.OrderByDescending(Len))
            {
                double ang = LineStrategy.SegmentAngle(sgm) * Math.PI / 180;
                double nx = -Math.Sin(ang), ny = Math.Cos(ang);
                double off = (sgm.P1.X + sgm.P2.X) / 2.0 * nx + (sgm.P1.Y + sgm.P2.Y) / 2.0 * ny;
                int gi = groups.FindIndex(g =>
                {
                    double d = Math.Abs(g.Angle - ang) % Math.PI; d = Math.Min(d, Math.PI - d);
                    if (d > 3 * Math.PI / 180) return false;
                    double gnx = -Math.Sin(g.Angle), gny = Math.Cos(g.Angle);
                    double o = (sgm.P1.X + sgm.P2.X) / 2.0 * gnx + (sgm.P1.Y + sgm.P2.Y) / 2.0 * gny;
                    return Math.Abs(o - g.Offset) < 3;
                });
                if (gi < 0) groups.Add((ang, off, new List<LineSegmentPoint> { sgm }, Len(sgm)));
                else { var g = groups[gi]; g.Segs.Add(sgm); groups[gi] = (g.Angle, g.Offset, g.Segs, g.Total + Len(sgm)); }
            }
            return groups.OrderByDescending(g => g.Total).Take(MaxLinesPerFamily).Select(g =>
            {
                var pts = g.Segs.SelectMany(x => new[] { new Point2f(x.P1.X, x.P1.Y), new Point2f(x.P2.X, x.P2.Y) }).ToArray();
                var fl = Cv2.FitLine(pts, DistanceTypes.L2, 0, 0.01, 0.01);
                return new Line2(new Point2f((float)fl.X1, (float)fl.Y1), new Point2f((float)fl.Vx, (float)fl.Vy));
            }).ToList();
        }

        /// <summary>
        /// Splits segments into the frame's two edge directions: 2-means on the doubled angle
        /// (so 0° and 180° coincide), weighted by length, seeded with the longest segment and
        /// the one most unlike it. Perspective fans each family out over tens of degrees, which
        /// a fixed tolerance around one direction would cut in half.
        /// </summary>
        internal static (List<LineSegmentPoint> A, List<LineSegmentPoint> B) SplitDirections(IReadOnlyList<LineSegmentPoint> segs)
        {
            double Len(LineSegmentPoint s) => Math.Sqrt(Math.Pow(s.P2.X - s.P1.X, 2) + Math.Pow(s.P2.Y - s.P1.Y, 2));
            (double X, double Y) Dir(LineSegmentPoint s)
            {
                double a = 2 * Math.Atan2(s.P2.Y - s.P1.Y, s.P2.X - s.P1.X);
                return (Math.Cos(a), Math.Sin(a));
            }
            var dirs = segs.Select(Dir).ToArray();
            var lens = segs.Select(Len).ToArray();
            int first = Array.IndexOf(lens, lens.Max());
            var ca = dirs[first];
            var cb = dirs.OrderBy(d => d.X * ca.X + d.Y * ca.Y).First();   // most unlike the longest
            var inA = new bool[segs.Count];
            for (int iter = 0; iter < 10; iter++)
            {
                for (int i = 0; i < segs.Count; i++)
                    inA[i] = dirs[i].X * ca.X + dirs[i].Y * ca.Y >= dirs[i].X * cb.X + dirs[i].Y * cb.Y;
                (double X, double Y) Mean(bool want)
                {
                    double x = 0, y = 0;
                    for (int i = 0; i < segs.Count; i++) if (inA[i] == want) { x += dirs[i].X * lens[i]; y += dirs[i].Y * lens[i]; }
                    double n = Math.Sqrt(x * x + y * y);
                    return n < 1e-9 ? (want ? ca : cb) : (x / n, y / n);
                }
                ca = Mean(true); cb = Mean(false);
            }
            var a = new List<LineSegmentPoint>(); var b = new List<LineSegmentPoint>();
            for (int i = 0; i < segs.Count; i++) (inA[i] ? a : b).Add(segs[i]);
            // Two "families" pointing nearly the same way are one family: not a frame.
            if (ca.X * cb.X + ca.Y * cb.Y > Math.Cos(2 * 30 * Math.PI / 180)) b.Clear();
            return (a, b);
        }

        private static Point2f? Intersect(Line2 a, Line2 b)
        {
            double den = a.D.X * b.D.Y - a.D.Y * b.D.X;
            if (Math.Abs(den) < 1e-6) return null;   // parallel
            double t = ((b.P.X - a.P.X) * b.D.Y - (b.P.Y - a.P.Y) * b.D.X) / den;
            return new Point2f((float)(a.P.X + t * a.D.X), (float)(a.P.Y + t * a.D.Y));
        }

        private static readonly string[] CornerNames = { "TL", "TR", "BR", "BL" };

        public void Annotate(Mat annotated, FrameDetection d, FrameInspection insp, VisionZone? zone, LabelStack labels)
        {
            int w = annotated.Width, h = annotated.Height;
            if (zone != null)
                VisionDrawing.DrawZoneBorder(annotated, zone.Geometry, w, h, insp.Name, VisionPalette.ZoneBorder);
            if (d.Corners != null)
            {
                Cv2.Polylines(annotated, new[] { ZoneGeometry.ToIntPoints(d.Corners) }, true, VisionPalette.Frame, VisionDrawing.OverlayThickness + 1);
                for (int i = 0; i < 4; i++)
                {
                    var p = new Point((int)d.Corners[i].X, (int)d.Corners[i].Y);
                    Cv2.Circle(annotated, p, 5, VisionPalette.Frame, -1);
                    VisionDrawing.DrawOutlinedText(annotated, CornerNames[i], new Point(p.X + 6, p.Y - 6), 0.45, VisionPalette.Frame);
                }
            }
            labels.DrawStatusLabel(annotated, insp.Name, d.Result.Found);
        }
    }
}
