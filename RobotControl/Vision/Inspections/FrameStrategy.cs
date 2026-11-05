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

        private readonly record struct Line2(Point2f P, Point2f D);   // point + unit direction

        private static Point2f[]? ByLines(FrameContext ctx, FrameInspection insp, VisionZone? zone, Mat mask)
        {
            int w = ctx.Width, h = ctx.Height;
            var bounds = zone != null ? ZoneGeometry.ZoneBounds(zone.Geometry, w, h) : new Rect(0, 0, w, h);
            double minLen = Math.Max(5, Math.Min(bounds.Width, bounds.Height) * Math.Max(0, insp.MinLineLengthPct) / 100.0);
            using var edges = MaskedEdges(ctx, insp.CannyLow, insp.CannyHigh, mask);
            var segs = Cv2.HoughLinesP(edges, 1, Math.PI / 180, Math.Max(1, insp.HoughThreshold), minLen, Math.Max(0, insp.MaxLineGap));
            if (segs.Length < 4) return null;

            var (famA, famB) = SplitDirections(segs);
            var outA = Outermost(famA);
            var outB = Outermost(famB);
            if (outA == null || outB == null) return null;

            var corners = new List<Point2f>();
            foreach (var la in new[] { outA.Value.Lo, outA.Value.Hi })
            foreach (var lb in new[] { outB.Value.Lo, outB.Value.Hi })
            {
                var p = Intersect(la, lb);
                if (p == null) return null;
                // A corner far outside the frame means the "edges" were not a frame.
                if (p.Value.X < -0.25 * w || p.Value.X > 1.25 * w || p.Value.Y < -0.25 * h || p.Value.Y > 1.25 * h) return null;
                corners.Add(p.Value);
            }
            return Order(corners);
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

        /// <summary>The two lines of a family furthest apart along the family's normal.</summary>
        private static (Line2 Lo, Line2 Hi)? Outermost(List<LineSegmentPoint> fam)
        {
            if (fam.Count < 2) return null;
            // Mean direction (doubled-angle average handles the 0/180 wrap).
            double sx = 0, sy = 0;
            foreach (var s in fam)
            {
                double a = 2 * Math.Atan2(s.P2.Y - s.P1.Y, s.P2.X - s.P1.X);
                double len = Math.Sqrt(Math.Pow(s.P2.X - s.P1.X, 2) + Math.Pow(s.P2.Y - s.P1.Y, 2));
                sx += Math.Cos(a) * len; sy += Math.Sin(a) * len;
            }
            double mean = Math.Atan2(sy, sx) / 2;
            double nx = -Math.Sin(mean), ny = Math.Cos(mean);
            double Offset(LineSegmentPoint s) => ((s.P1.X + s.P2.X) / 2.0) * nx + ((s.P1.Y + s.P2.Y) / 2.0) * ny;
            var lo = fam.OrderBy(Offset).First();
            var hi = fam.OrderByDescending(Offset).First();
            if (Math.Abs(Offset(hi) - Offset(lo)) < 10) return null;   // only one edge of the frame seen
            return (ToLine(lo), ToLine(hi));
        }

        private static Line2 ToLine(LineSegmentPoint s)
        {
            double dx = s.P2.X - s.P1.X, dy = s.P2.Y - s.P1.Y, len = Math.Sqrt(dx * dx + dy * dy);
            return new Line2(new Point2f(s.P1.X, s.P1.Y), new Point2f((float)(dx / len), (float)(dy / len)));
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
