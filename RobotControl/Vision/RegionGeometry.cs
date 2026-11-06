using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using Controller.RobotControl.Vision.Calibration;

namespace Controller.RobotControl.Vision
{
    using Point = OpenCvSharp.Point;

    internal enum RegionKind { Rect, Circle, Polygon, Quad }

    /// <summary>
    /// A tool region in pixels, after any fixture transform: a rotated rectangle (centre, size,
    /// angle), a circle, a polygon or a perspective quad (TL, TR, BR, BL).
    /// </summary>
    internal sealed class PixelRegion
    {
        public RegionKind Kind { get; private init; }
        public double Cx { get; private init; }
        public double Cy { get; private init; }
        /// <summary>Rect: width/height (px) and tilt (deg, clockwise on screen). Circle: R.</summary>
        public double W { get; private init; }
        public double H { get; private init; }
        public double Angle { get; private init; }
        public double R { get; private init; }
        public Point2d[] Pts { get; private init; } = Array.Empty<Point2d>();

        public static PixelRegion Rect(double cx, double cy, double w, double h, double angle) =>
            new() { Kind = RegionKind.Rect, Cx = cx, Cy = cy, W = Math.Abs(w), H = Math.Abs(h), Angle = angle };
        public static PixelRegion Circle(double cx, double cy, double r) =>
            new() { Kind = RegionKind.Circle, Cx = cx, Cy = cy, R = Math.Abs(r) };
        public static PixelRegion Polygon(IEnumerable<Point2d> pts) => new() { Kind = RegionKind.Polygon, Pts = pts.ToArray() };
        public static PixelRegion Quad(IEnumerable<Point2d> pts) => new() { Kind = RegionKind.Quad, Pts = pts.ToArray() };
        public static PixelRegion Whole(int w, int h) => Rect(w / 2.0, h / 2.0, w, h, 0);

        /// <summary>A drawn (normalized) region on a w × h frame; null region = the whole frame.</summary>
        public static PixelRegion FromNormalized(VisionRegion? r, int w, int h)
        {
            if (r == null) return Whole(w, h);
            switch ((r.Shape ?? "rect").Trim().ToLowerInvariant())
            {
                case "circle":
                    return Circle(r.Cx * w, r.Cy * h, r.Radius * Math.Min(w, h));
                case "polygon":
                    if (r.Points is { Count: >= 3 } pp) return Polygon(pp.Select(p => new Point2d(p.X * w, p.Y * h)));
                    return Whole(w, h);
                case "quad":
                    if (r.Points is { Count: 4 } qp) return Quad(qp.Select(p => new Point2d(p.X * w, p.Y * h)));
                    return Whole(w, h);
                default:
                    return Rect((r.X + r.Width / 2) * w, (r.Y + r.Height / 2) * h, r.Width * w, r.Height * h, r.Rotation);
            }
        }

        /// <summary>
        /// The four corners TL, TR, BR, BL of a rect or quad; for a circle or polygon, its
        /// axis-aligned bounding box. Directional tools (findLine, caliper) work in this frame.
        /// </summary>
        public Point2d[] Corners()
        {
            switch (Kind)
            {
                case RegionKind.Rect:
                    return LocalQuad(Cx, Cy, -W / 2, -H / 2, W / 2, H / 2, Angle);
                case RegionKind.Quad:
                    return Pts;
                case RegionKind.Circle:
                    return new[] { new Point2d(Cx - R, Cy - R), new Point2d(Cx + R, Cy - R), new Point2d(Cx + R, Cy + R), new Point2d(Cx - R, Cy + R) };
                default:
                {
                    double x0 = Pts.Min(p => p.X), x1 = Pts.Max(p => p.X), y0 = Pts.Min(p => p.Y), y1 = Pts.Max(p => p.Y);
                    return new[] { new Point2d(x0, y0), new Point2d(x1, y0), new Point2d(x1, y1), new Point2d(x0, y1) };
                }
            }
        }

        /// <summary>The outline to draw (a circle is approximated by 48 points).</summary>
        public Point2d[] Outline()
        {
            if (Kind == RegionKind.Circle)
                return Enumerable.Range(0, 48).Select(i => new Point2d(Cx + R * Math.Cos(i * Math.PI / 24), Cy + R * Math.Sin(i * Math.PI / 24))).ToArray();
            if (Kind == RegionKind.Polygon) return Pts;
            return Corners();
        }

        public Point2d Center() => Kind switch
        {
            RegionKind.Rect or RegionKind.Circle => new Point2d(Cx, Cy),
            _ => new Point2d(Pts.Average(p => p.X), Pts.Average(p => p.Y)),
        };

        /// <summary>Pixel bounding box clamped to a w × h frame (may be empty when off-frame).</summary>
        public Rect Bounds(int w, int h)
        {
            var o = Outline();
            if (o.Length == 0) return new Rect(0, 0, 0, 0);
            int x0 = Math.Max(0, (int)Math.Floor(o.Min(p => p.X))), x1 = Math.Min(w, (int)Math.Ceiling(o.Max(p => p.X)) + 1);
            int y0 = Math.Max(0, (int)Math.Floor(o.Min(p => p.Y))), y1 = Math.Min(h, (int)Math.Ceiling(o.Max(p => p.Y)) + 1);
            return new Rect(x0, y0, Math.Max(0, x1 - x0), Math.Max(0, y1 - y0));
        }

        /// <summary>Paints the region white onto a mask whose (0, 0) is <paramref name="offset"/> in the frame.</summary>
        public void Fill(Mat mask, Point offset)
        {
            if (Kind == RegionKind.Circle)
            {
                // Region coordinates put pixel i's centre at i + 0.5; OpenCV draws with it at i.
                const int shift = Geo.FillShift;
                Cv2.Circle(mask, new Point((int)Math.Round((Cx - 0.5 - offset.X) * (1 << shift)), (int)Math.Round((Cy - 0.5 - offset.Y) * (1 << shift))),
                    Math.Max(1 << shift, (int)Math.Round(R * (1 << shift))), Scalar.White, -1, LineTypes.Link8, shift);
                return;
            }
            Geo.FillPolygon(mask, Outline(), offset);
        }

        public bool Contains(double x, double y)
        {
            switch (Kind)
            {
                case RegionKind.Circle:
                    return (x - Cx) * (x - Cx) + (y - Cy) * (y - Cy) <= R * R;
                default:
                {
                    var q = Outline();
                    bool inside = false;
                    for (int i = 0, j = q.Length - 1; i < q.Length; j = i++)
                        if ((q[i].Y > y) != (q[j].Y > y) && x < (q[j].X - q[i].X) * (y - q[i].Y) / (q[j].Y - q[i].Y) + q[i].X)
                            inside = !inside;
                    return inside;
                }
            }
        }

        /// <summary>
        /// Cell (row, col) of a rows × cols grid with <paramref name="inset"/> (0–0.45 of the cell)
        /// trimmed off each side, as a quad TL, TR, BR, BL. A rect's lattice turns with the rect, a
        /// quad's follows its perspective, a circle's or polygon's is laid over its bounding box.
        /// </summary>
        public Point2d[] Cell(int rows, int cols, int row, int col, double inset)
        {
            double t = Math.Clamp(inset, 0, 0.45);
            double u0 = (col + t) / cols, u1 = (col + 1 - t) / cols;
            double v0 = (row + t) / rows, v1 = (row + 1 - t) / rows;
            if (Kind == RegionKind.Quad)
            {
                var hq = UnitSquareTo(Pts);
                Point2d P(double u, double v) { var (x, y) = Homography.Apply(hq, u, v); return new Point2d(x, y); }
                return new[] { P(u0, v0), P(u1, v0), P(u1, v1), P(u0, v1) };
            }
            if (Kind == RegionKind.Rect)
                return LocalQuad(Cx, Cy, -W / 2 + W * u0, -H / 2 + H * v0, -W / 2 + W * u1, -H / 2 + H * v1, Angle);
            var c = Corners();
            double bw = c[1].X - c[0].X, bh = c[3].Y - c[0].Y;
            Point2d B(double u, double v) => new(c[0].X + bw * u, c[0].Y + bh * v);
            return new[] { B(u0, v0), B(u1, v0), B(u1, v1), B(u0, v1) };
        }

        /// <summary>This region moved by a fixture transform.</summary>
        public PixelRegion Transform(FixtureTransform t)
        {
            if (t.IsIdentity) return this;
            Point2d M(Point2d p) => t.Apply(p);
            switch (Kind)
            {
                case RegionKind.Rect when t.Similarity:
                {
                    var c = M(new Point2d(Cx, Cy));
                    return Rect(c.X, c.Y, W * t.Scale, H * t.Scale, Angle + t.Rotation);
                }
                case RegionKind.Rect:
                    return Quad(Corners().Select(M));
                case RegionKind.Circle:
                {
                    var c = M(new Point2d(Cx, Cy));
                    // Local scale at the centre: how far a point one radius away lands.
                    var e = M(new Point2d(Cx + R, Cy));
                    double r = Math.Sqrt((e.X - c.X) * (e.X - c.X) + (e.Y - c.Y) * (e.Y - c.Y));
                    return Circle(c.X, c.Y, r);
                }
                case RegionKind.Quad:
                    return Quad(Pts.Select(M));
                default:
                    return Polygon(Pts.Select(M));
            }
        }

        /// <summary>A sub-region in the same pixel space (used for findCorner's second side).</summary>
        public static Point2d[] LocalQuad(double cx, double cy, double x0, double y0, double x1, double y1, double degrees)
        {
            double a = degrees * Math.PI / 180.0, cos = Math.Cos(a), sin = Math.Sin(a);
            Point2d P(double lx, double ly) => new(cx + lx * cos - ly * sin, cy + lx * sin + ly * cos);
            return new[] { P(x0, y0), P(x1, y0), P(x1, y1), P(x0, y1) };
        }

        /// <summary>The homography taking the unit square onto a quad: (0,0)→TL, (1,0)→TR, (1,1)→BR, (0,1)→BL.</summary>
        public static double[][] UnitSquareTo(Point2d[] quad)
        {
            var src = new[] { new Point2f(0, 0), new Point2f(1, 0), new Point2f(1, 1), new Point2f(0, 1) };
            var dst = quad.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray();
            using var m = Cv2.GetPerspectiveTransform(src, dst);
            return Homography.FromMat(m);
        }
    }

    /// <summary>
    /// How a fixture moved since its reference: a similarity (shift + rotate + scale) or, when
    /// both poses carry a quad, a perspective homography. Maps reference pixels → pixels now.
    /// </summary>
    internal sealed class FixtureTransform
    {
        public static readonly FixtureTransform Identity = new(Homography.Identity(), true, 0, 1);

        private FixtureTransform(double[][] h, bool similarity, double rotation, double scale)
        {
            H = h; Similarity = similarity; Rotation = rotation; Scale = scale;
        }

        public double[][] H { get; }
        public bool Similarity { get; }
        /// <summary>Similarity only: rotation (deg) and scale.</summary>
        public double Rotation { get; }
        public double Scale { get; }

        public bool IsIdentity
        {
            get
            {
                var i = Homography.Identity();
                for (int r = 0; r < 3; r++) for (int c = 0; c < 3; c++) if (Math.Abs(H[r][c] - i[r][c]) > 1e-9) return false;
                return true;
            }
        }

        public Point2d Apply(Point2d p) { var (x, y) = Homography.Apply(H, p.X, p.Y); return new Point2d(x, y); }

        /// <summary>The transform from <paramref name="reference"/> to <paramref name="now"/>; null when degenerate.</summary>
        public static FixtureTransform? Between(VisionPose reference, VisionPose now)
        {
            if (reference.Quad is { Count: 4 } rq && now.Quad is { Count: 4 } nq)
            {
                var a = rq.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray();
                var b = nq.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray();
                if (!Geo.Convex(a) || !Geo.Convex(b)) return null;
                try
                {
                    using var m = Cv2.GetPerspectiveTransform(a, b);
                    var h = Homography.FromMat(m);
                    if (h.Any(r => r.Any(v => double.IsNaN(v) || double.IsInfinity(v)))) return null;
                    return new FixtureTransform(h, false, now.Angle - reference.Angle, 1);
                }
                catch (OpenCVException) { return null; }
            }
            double refScale = reference.Scale > 0 ? reference.Scale : 1, nowScale = now.Scale > 0 ? now.Scale : 1;
            double s = nowScale / refScale, rot = now.Angle - reference.Angle;
            double a2 = rot * Math.PI / 180, cos = Math.Cos(a2) * s, sin = Math.Sin(a2) * s;
            // p_now = R·S·(p_ref − ref) + now
            double tx = now.X - (cos * reference.X - sin * reference.Y);
            double ty = now.Y - (sin * reference.X + cos * reference.Y);
            return new FixtureTransform(new[] { new[] { cos, -sin, tx }, new[] { sin, cos, ty }, new[] { 0.0, 0, 1 } }, true, rot, s);
        }
    }

    /// <summary>Small 2-D geometry helpers (pixel space).</summary>
    internal static class Geo
    {
        /// <summary>Angle of the direction (dx, dy) in degrees: atan2 with y down (clockwise on screen is positive).</summary>
        public static double AngleDeg(double dx, double dy) => Math.Atan2(dy, dx) * 180 / Math.PI;

        /// <summary>Wraps an angle into (−180, 180].</summary>
        public static double Wrap180(double a)
        {
            a %= 360;
            if (a > 180) a -= 360;
            if (a <= -180) a += 360;
            return a;
        }

        public static double Dist(Point2d a, Point2d b) => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

        /// <summary>Intersection of the infinite lines through (a1, a2) and (b1, b2); null when parallel.</summary>
        public static Point2d? Intersect(Point2d a1, Point2d a2, Point2d b1, Point2d b2)
        {
            double d1x = a2.X - a1.X, d1y = a2.Y - a1.Y, d2x = b2.X - b1.X, d2y = b2.Y - b1.Y;
            double den = d1x * d2y - d1y * d2x;
            double scale = Math.Sqrt((d1x * d1x + d1y * d1y) * (d2x * d2x + d2y * d2y));
            if (scale < 1e-12 || Math.Abs(den) < 1e-9 * scale) return null;
            double t = ((b1.X - a1.X) * d2y - (b1.Y - a1.Y) * d2x) / den;
            return new Point2d(a1.X + t * d1x, a1.Y + t * d1y);
        }

        public static Point2d? Intersect(PxLine a, PxLine b) =>
            Intersect(new Point2d(a.X1, a.Y1), new Point2d(a.X2, a.Y2), new Point2d(b.X1, b.Y1), new Point2d(b.X2, b.Y2));

        /// <summary>True when the four points form a convex quadrilateral with real area.</summary>
        public static bool Convex(Point2f[] q)
        {
            if (q.Length != 4) return false;
            int sign = 0;
            for (int i = 0; i < 4; i++)
            {
                var a = q[i]; var b = q[(i + 1) % 4]; var c = q[(i + 2) % 4];
                double cross = (b.X - a.X) * (c.Y - b.Y) - (b.Y - a.Y) * (c.X - b.X);
                if (Math.Abs(cross) < 1e-9) return false;
                int sg = Math.Sign(cross);
                if (sign == 0) sign = sg;
                else if (sg != sign) return false;
            }
            return true;
        }

        public static Point[] ToInt(IEnumerable<Point2d> pts) => pts.Select(p => new Point((int)Math.Round(p.X), (int)Math.Round(p.Y))).ToArray();

        public static List<PxPoint> ToPx(IEnumerable<Point2d> pts) => pts.Select(p => new PxPoint(p.X, p.Y)).ToList();

        /// <summary>A quad's mask cut to its own bounding box within a w × h frame; empty box when off-frame.</summary>
        public static (Mat Mask, Rect Box) QuadMask(Point2d[] quad, int w, int h)
        {
            int x0 = Math.Max(0, (int)Math.Floor(quad.Min(p => p.X))), x1 = Math.Min(w, (int)Math.Ceiling(quad.Max(p => p.X)) + 1);
            int y0 = Math.Max(0, (int)Math.Floor(quad.Min(p => p.Y))), y1 = Math.Min(h, (int)Math.Ceiling(quad.Max(p => p.Y)) + 1);
            var box = new Rect(x0, y0, Math.Max(0, x1 - x0), Math.Max(0, y1 - y0));
            var mask = new Mat(Math.Max(1, box.Height), Math.Max(1, box.Width), MatType.CV_8UC1, Scalar.Black);
            if (box.Width > 0 && box.Height > 0) FillPolygon(mask, quad, new Point(box.X, box.Y));
            return (mask, box);
        }

        /// <summary>Sub-pixel bits used when filling region masks.</summary>
        public const int FillShift = 8;

        /// <summary>
        /// Fills a polygon given in region coordinates (pixel i spans [i, i + 1)) onto a mask whose
        /// (0, 0) is <paramref name="offset"/>: a pixel is in when its centre is, so neighbouring
        /// grid cells do not both claim the pixels on their shared edge.
        /// </summary>
        public static void FillPolygon(Mat mask, IReadOnlyList<Point2d> pts, Point offset)
        {
            if (pts.Count < 3) return;
            const double k = 1 << FillShift;
            // Pull the outline in by a hair so a cell edge lying exactly on a row of pixel
            // centres leaves them to one side only.
            var poly = pts.Select(p => new Point((int)Math.Round((p.X - 0.5 - offset.X) * k - 1), (int)Math.Round((p.Y - 0.5 - offset.Y) * k - 1))).ToArray();
            Cv2.FillPoly(mask, new[] { poly }, Scalar.White, LineTypes.Link8, FillShift);
        }
    }
}
