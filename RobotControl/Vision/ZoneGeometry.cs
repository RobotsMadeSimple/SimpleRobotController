using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision
{
    using Point = OpenCvSharp.Point;

    /// <summary>
    /// Pixel-space geometry for vision zones: containment, masks, bounding boxes and grid
    /// lattices. Zones are stored as 0–1 fractions of the frame; every helper here takes the
    /// frame size and works in pixels.
    ///
    /// A tilt of 0 is the overwhelmingly common case and every helper branches on it, keeping
    /// the original axis-aligned code path intact. That is deliberate: the untilted paths are
    /// cheaper (a Rect ROI beats a polygon mask) and already proven, so rotation adds a route
    /// rather than replacing one.
    /// </summary>
    internal static class ZoneGeometry
    {
        // Shapes we've already warned about for IsInsideZone's fail-closed default — logged
        // once per shape value rather than once per frame.
        private static readonly HashSet<VisionZoneShape> _loggedUnknownShapes = new();

        // ── Containment ──────────────────────────────────────────────────────────

        /// <summary>True when pixel point (px, py) lies inside the zone on a w×h frame.</summary>
        public static bool IsInsideZone(VisionZoneGeometry geom, float px, float py, int w, int h)
        {
            if (IsRotatedRect(geom))
            {
                // Rotate the point back into the rectangle's own frame rather than testing it
                // against the tilted quad — once untilted it is an axis-aligned compare again.
                double hw = geom.Width * w / 2.0, hh = geom.Height * h / 2.0;
                double a  = -geom.Rotation * Math.PI / 180.0;
                double dx = px - (geom.X * w + hw), dy = py - (geom.Y * h + hh);
                double lx = dx * Math.Cos(a) - dy * Math.Sin(a);
                double ly = dx * Math.Sin(a) + dy * Math.Cos(a);
                return Math.Abs(lx) <= hw && Math.Abs(ly) <= hh;
            }

            switch (geom.Shape)
            {
                case VisionZoneShape.Rectangle:
                {
                    double rx = geom.X * w, ry = geom.Y * h;
                    double rw = geom.Width * w, rh = geom.Height * h;
                    return px >= rx && px <= rx + rw && py >= ry && py <= ry + rh;
                }
                case VisionZoneShape.Circle:
                {
                    double cx = geom.Cx * w, cy = geom.Cy * h;
                    double r  = geom.Radius * Math.Min(w, h);
                    double dx = px - cx, dy = py - cy;
                    return dx * dx + dy * dy <= r * r;
                }
                case VisionZoneShape.Polygon:
                case VisionZoneShape.Quad:
                {
                    var pts = geom.Points;
                    int n   = pts.Count;
                    if (n < 3 || (geom.Shape == VisionZoneShape.Quad && n != 4)) return false;
                    bool inside = false;
                    for (int i = 0, j = n - 1; i < n; j = i++)
                    {
                        double xi = pts[i][0] * w, yi = pts[i][1] * h;
                        double xj = pts[j][0] * w, yj = pts[j][1] * h;
                        if (((yi > py) != (yj > py)) &&
                            (px < (xj - xi) * (py - yi) / (yj - yi) + xi))
                            inside = !inside;
                    }
                    return inside;
                }
                default:
                {
                    bool firstTime;
                    lock (_loggedUnknownShapes) firstTime = _loggedUnknownShapes.Add(geom.Shape);
                    if (firstTime)
                        Console.WriteLine($"[Vision] IsInsideZone: unrecognized zone shape {geom.Shape} — treating point as outside zone");
                    return false;
                }
            }
        }

        /// <summary>Paints the zone white onto a single-channel mask the size of the frame.</summary>
        public static void FillZoneMask(Mat mask, VisionZoneGeometry geom, int w, int h)
        {
            if (IsRotatedRect(geom))
            {
                Cv2.FillPoly(mask, new[] { ToIntPoints(RectCorners(geom, w, h)) }, Scalar.White);
                return;
            }

            switch (geom.Shape)
            {
                case VisionZoneShape.Rectangle:
                {
                    int rx = Math.Clamp((int)(geom.X * w), 0, w - 1);
                    int ry = Math.Clamp((int)(geom.Y * h), 0, h - 1);
                    int rw = Math.Clamp((int)(geom.Width  * w), 1, w - rx);
                    int rh = Math.Clamp((int)(geom.Height * h), 1, h - ry);
                    using (var roi = mask[new Rect(rx, ry, rw, rh)])
                        roi.SetTo(Scalar.White);
                    break;
                }
                case VisionZoneShape.Circle:
                {
                    var center = new Point((int)(geom.Cx * w), (int)(geom.Cy * h));
                    int radius = (int)(geom.Radius * Math.Min(w, h));
                    Cv2.Circle(mask, center, Math.Max(1, radius), Scalar.White, -1);
                    break;
                }
                case VisionZoneShape.Polygon:
                case VisionZoneShape.Quad:
                {
                    if (geom.Shape == VisionZoneShape.Quad ? geom.Points.Count == 4 : geom.Points.Count >= 3)
                    {
                        var pts = geom.Points
                            .Select(p => new Point(
                                Math.Clamp((int)(p[0] * w), 0, w - 1),
                                Math.Clamp((int)(p[1] * h), 0, h - 1)))
                            .ToArray();
                        Cv2.FillPoly(mask, new[] { pts }, Scalar.White);
                    }
                    break;
                }
            }
        }

        // ── Rotated rectangles and quads ─────────────────────────────────────────

        /// <summary>True when this geometry is a rectangle that is actually tilted.</summary>
        public static bool IsRotatedRect(VisionZoneGeometry geom) =>
            geom.Shape == VisionZoneShape.Rectangle && Math.Abs(geom.Rotation) > 1e-9;

        /// <summary>True for a well-formed four-corner (perspective) zone.</summary>
        public static bool IsQuad(VisionZoneGeometry geom) =>
            geom.Shape == VisionZoneShape.Quad && geom.Points.Count == 4;

        /// <summary>
        /// True when grid cells are quadrilaterals rather than upright rects: a tilted
        /// rectangle or a perspective quad. Those cells are measured with a mask of their own.
        /// </summary>
        public static bool HasQuadCells(VisionZoneGeometry geom) => IsRotatedRect(geom) || IsQuad(geom);

        /// <summary>A quad zone's corners (TL, TR, BR, BL) in pixels.</summary>
        public static Point2f[] QuadCorners(VisionZoneGeometry geom, int w, int h) =>
            geom.Points.Select(p => new Point2f((float)(p[0] * w), (float)(p[1] * h))).ToArray();

        /// <summary>
        /// The homography taking the unit square onto a quad: (0,0)→TL, (1,0)→TR, (1,1)→BR,
        /// (0,1)→BL. A grid laid out in the unit square and mapped through this follows the
        /// perspective of a board seen at an angle — far rows come out shorter, as they look.
        /// </summary>
        public static double[][] UnitSquareTo(Point2f[] quad)
        {
            var src = new[] { new Point2f(0, 0), new Point2f(1, 0), new Point2f(1, 1), new Point2f(0, 1) };
            using var m = Cv2.GetPerspectiveTransform(src, quad);
            return Calibration.Homography.FromMat(m);
        }

        /// <summary>
        /// The homography mapping four points onto four others (both [x, y] lists, same
        /// order), or null when either set is degenerate (three in a line, repeated points).
        /// </summary>
        public static double[][]? Between(IReadOnlyList<double[]> from, IReadOnlyList<double[]> to)
        {
            if (from.Count != 4 || to.Count != 4) return null;
            var a = from.Select(p => new Point2f((float)p[0], (float)p[1])).ToArray();
            var b = to.Select(p => new Point2f((float)p[0], (float)p[1])).ToArray();
            if (!Convex(a) || !Convex(b)) return null;
            try
            {
                using var m = Cv2.GetPerspectiveTransform(a, b);
                var hm = Calibration.Homography.FromMat(m);
                return hm.Any(r => r.Any(v => double.IsNaN(v) || double.IsInfinity(v))) ? null : hm;
            }
            catch (OpenCVException) { return null; }
        }

        /// <summary>True when the four points form a convex quadrilateral with real area.</summary>
        internal static bool Convex(Point2f[] q)
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

        /// <summary>
        /// The zone as warped by <paramref name="hNorm"/> (a homography in normalized 0–1
        /// coordinates). Rectangles — tilted or not — become Quads of their four corners, so a
        /// gridded rectangle turns into a perspective-correct gridded quad; polygon and quad
        /// points map one by one; a circle keeps its radius and moves its centre. The id, name
        /// and grid are kept.
        /// </summary>
        public static VisionZone Warp(VisionZone zone, double[][] hNorm, int w, int h)
        {
            var g = zone.Geometry;
            double[] Map(double x, double y)
            {
                var (mx, my) = Calibration.Homography.Apply(hNorm, x, y);
                return new[] { mx, my };
            }
            VisionZoneGeometry warped;
            switch (g.Shape)
            {
                case VisionZoneShape.Rectangle:
                {
                    var corners = RectCorners(g, w, h);   // pixel space: rotation is applied there
                    warped = new VisionZoneGeometry
                    {
                        Shape  = VisionZoneShape.Quad,
                        Points = corners.Select(c => Map(c.X / (double)w, c.Y / (double)h)).ToList(),
                    };
                    break;
                }
                case VisionZoneShape.Circle:
                {
                    var c = Map(g.Cx, g.Cy);
                    warped = new VisionZoneGeometry { Shape = VisionZoneShape.Circle, Cx = c[0], Cy = c[1], Radius = g.Radius };
                    break;
                }
                default:
                    warped = new VisionZoneGeometry { Shape = g.Shape, Points = g.Points.Select(p => Map(p[0], p[1])).ToList() };
                    break;
            }
            return new VisionZone { Id = zone.Id, Name = zone.Name, Grid = zone.Grid, AlignTo = zone.AlignTo, Geometry = warped };
        }

        /// <summary>The zone's outline as normalized points (a circle is approximated).</summary>
        public static List<double[]> Outline(VisionZoneGeometry g, int w, int h)
        {
            switch (g.Shape)
            {
                case VisionZoneShape.Rectangle:
                    return RectCorners(g, w, h).Select(c => new[] { c.X / (double)w, c.Y / (double)h }).ToList();
                case VisionZoneShape.Circle:
                {
                    double r = g.Radius * Math.Min(w, h);
                    return Enumerable.Range(0, 24).Select(i =>
                    {
                        double a = i * Math.PI / 12;
                        return new[] { g.Cx + r * Math.Cos(a) / w, g.Cy + r * Math.Sin(a) / h };
                    }).ToList();
                }
                default:
                    return g.Points.Select(p => new[] { p[0], p[1] }).ToList();
            }
        }

        /// <summary>
        /// Corners of an axis-aligned box expressed in the rectangle's own frame (offsets from
        /// its center), tilted by <paramref name="degrees"/> and placed back on the center.
        /// The rectangle itself and each of its grid cells are both boxes in that frame, so
        /// they rotate through the same code and cannot drift out of alignment.
        /// </summary>
        private static Point2f[] LocalQuad(
            double cx, double cy, double x0, double y0, double x1, double y1, double degrees)
        {
            double a = degrees * Math.PI / 180.0, cos = Math.Cos(a), sin = Math.Sin(a);
            Point2f P(double lx, double ly) => new(
                (float)(cx + lx * cos - ly * sin),
                (float)(cy + lx * sin + ly * cos));
            return new[] { P(x0, y0), P(x1, y0), P(x1, y1), P(x0, y1) };
        }

        /// <summary>
        /// The rectangle's four corners in pixel space, clockwise from top-left.
        ///
        /// Pixel space, not the normalized 0–1 space the geometry is stored in: x and y are
        /// scaled by width and height independently, and rotating inside a non-uniform scale
        /// shears the rectangle instead of turning it. On a 640×480 frame a 45° "rotation"
        /// applied before scaling comes out as a parallelogram.
        /// </summary>
        public static Point2f[] RectCorners(VisionZoneGeometry geom, int w, int h)
        {
            double hw = geom.Width * w / 2.0, hh = geom.Height * h / 2.0;
            return LocalQuad(geom.X * w + hw, geom.Y * h + hh, -hw, -hh, hw, hh, geom.Rotation);
        }

        /// <summary>
        /// Corners of grid cell (row, col) on a tilted rectangle. The lattice is laid out in
        /// the rectangle's own frame and rotated as a whole, so cells stay square to the zone
        /// rather than to the image — which is the entire point of tilting a gridded zone.
        /// </summary>
        public static Point2f[] CellQuad(
            VisionZoneGeometry geom, int rows, int cols, int row, int col, int w, int h)
        {
            if (IsQuad(geom))
            {
                // Laid out in the unit square, then mapped with the perspective.
                var hq = UnitSquareTo(QuadCorners(geom, w, h));
                Point2f P(double u, double v)
                {
                    var (x, y) = Calibration.Homography.Apply(hq, u, v);
                    return new Point2f((float)x, (float)y);
                }
                double u0 = col / (double)cols, u1 = (col + 1) / (double)cols;
                double v0 = row / (double)rows, v1 = (row + 1) / (double)rows;
                return new[] { P(u0, v0), P(u1, v0), P(u1, v1), P(u0, v1) };
            }
            double hw = geom.Width * w / 2.0, hh = geom.Height * h / 2.0;
            double x0 = -hw + 2 * hw * col / (double)cols, x1 = -hw + 2 * hw * (col + 1) / (double)cols;
            double y0 = -hh + 2 * hh * row / (double)rows, y1 = -hh + 2 * hh * (row + 1) / (double)rows;
            return LocalQuad(geom.X * w + hw, geom.Y * h + hh, x0, y0, x1, y1, geom.Rotation);
        }

        /// <summary>
        /// The centre part of grid cell (row, col): the cell with <paramref name="inset"/>
        /// (0–0.45, a fraction of the cell) trimmed off every side, as a quad TL, TR, BR, BL.
        /// On a perspective quad the trim follows the perspective; any other zone uses its
        /// bounding-box lattice, like the cells themselves.
        /// </summary>
        public static Point2f[] CellWindow(
            VisionZoneGeometry geom, int rows, int cols, int row, int col, double inset, int w, int h)
        {
            double t = Math.Clamp(inset, 0, 0.45);
            double u0 = (col + t) / cols, u1 = (col + 1 - t) / cols;
            double v0 = (row + t) / rows, v1 = (row + 1 - t) / rows;
            if (IsQuad(geom))
            {
                var hq = UnitSquareTo(QuadCorners(geom, w, h));
                Point2f P(double u, double v)
                {
                    var (x, y) = Calibration.Homography.Apply(hq, u, v);
                    return new Point2f((float)x, (float)y);
                }
                return new[] { P(u0, v0), P(u1, v0), P(u1, v1), P(u0, v1) };
            }
            if (IsRotatedRect(geom))
            {
                double hw = geom.Width * w / 2.0, hh = geom.Height * h / 2.0;
                return LocalQuad(geom.X * w + hw, geom.Y * h + hh,
                    -hw + 2 * hw * u0, -hh + 2 * hh * v0, -hw + 2 * hw * u1, -hh + 2 * hh * v1, geom.Rotation);
            }
            var b = ZoneBounds(geom, w, h);
            float X(double u) => (float)(b.X + b.Width * u);
            float Y(double v) => (float)(b.Y + b.Height * v);
            return new[] { new Point2f(X(u0), Y(v0)), new Point2f(X(u1), Y(v0)), new Point2f(X(u1), Y(v1)), new Point2f(X(u0), Y(v1)) };
        }

        /// <summary>
        /// A zone's four corners (TL, TR, BR, BL) in pixels when it has four: a quad, a
        /// rectangle (tilted or not) or a four-point polygon. Null for anything else.
        /// </summary>
        public static Point2f[]? FourCorners(VisionZoneGeometry geom, int w, int h)
        {
            if (IsQuad(geom)) return QuadCorners(geom, w, h);
            if (geom.Shape == VisionZoneShape.Rectangle) return RectCorners(geom, w, h);
            if (geom.Shape == VisionZoneShape.Polygon && geom.Points.Count == 4)
                return Inspections.FrameStrategy.Order(QuadCorners(geom, w, h));
            return null;
        }

        /// <summary>Axis-aligned pixel bounding box of a quad, clamped to the frame.</summary>
        public static Rect QuadBounds(Point2f[] quad, int w, int h)
        {
            int x0 = (int)Math.Floor(quad.Min(p => p.X)), x1 = (int)Math.Ceiling(quad.Max(p => p.X));
            int y0 = (int)Math.Floor(quad.Min(p => p.Y)), y1 = (int)Math.Ceiling(quad.Max(p => p.Y));
            return ClampRect(x0, y0, x1 - x0, y1 - y0, w, h);
        }

        public static Point[] ToIntPoints(Point2f[] quad) =>
            quad.Select(p => new Point((int)Math.Round(p.X), (int)Math.Round(p.Y))).ToArray();

        // ── Bounds and grid lattice ──────────────────────────────────────────────

        /// <summary>Pixel bounding box of a zone — the rectangle a grid is laid out over.</summary>
        public static Rect ZoneBounds(VisionZoneGeometry geom, int w, int h)
        {
            if (IsRotatedRect(geom))
                return QuadBounds(RectCorners(geom, w, h), w, h);

            switch (geom.Shape)
            {
                case VisionZoneShape.Circle:
                {
                    int radius = Math.Max(1, (int)(geom.Radius * Math.Min(w, h)));
                    int cx     = (int)(geom.Cx * w), cy = (int)(geom.Cy * h);
                    return ClampRect(cx - radius, cy - radius, radius * 2, radius * 2, w, h);
                }
                case VisionZoneShape.Quad when geom.Points.Count == 4:
                case VisionZoneShape.Polygon when geom.Points.Count >= 3:
                {
                    // Each edge is converted to pixels on its own and the size derived from
                    // those — scaling the difference instead loses a pixel to rounding
                    // (0.6 - 0.2 is not exactly 0.4 in binary), leaving the last column short.
                    int x0 = (int)(geom.Points.Min(p => p[0]) * w), x1 = (int)(geom.Points.Max(p => p[0]) * w);
                    int y0 = (int)(geom.Points.Min(p => p[1]) * h), y1 = (int)(geom.Points.Max(p => p[1]) * h);
                    return ClampRect(x0, y0, x1 - x0, y1 - y0, w, h);
                }
                default:
                    return ClampRect((int)(geom.X * w), (int)(geom.Y * h),
                                     (int)(geom.Width * w), (int)(geom.Height * h), w, h);
            }
        }

        /// <summary>
        /// Clamps a rect to a w×h frame: the origin is pulled inside the frame and the size is
        /// at least 1 and never runs past the far edge.
        /// </summary>
        public static Rect ClampRect(int x, int y, int rw, int rh, int w, int h)
        {
            int cx = Math.Clamp(x, 0, w - 1);
            int cy = Math.Clamp(y, 0, h - 1);
            return new Rect(cx, cy, Math.Clamp(rw, 1, w - cx), Math.Clamp(rh, 1, h - cy));
        }

        /// <summary>
        /// Cell (row, col) of a rows×cols lattice over <paramref name="bounds"/>. Edges are
        /// computed from the bounds rather than by accumulating a cell width, so the cells
        /// tile the box exactly with no rounding gap or overhang at the far edge.
        /// </summary>
        public static Rect CellRect(Rect bounds, int rows, int cols, int row, int col)
        {
            int x0 = bounds.X + bounds.Width  * col       / cols;
            int x1 = bounds.X + bounds.Width  * (col + 1) / cols;
            int y0 = bounds.Y + bounds.Height * row       / rows;
            int y1 = bounds.Y + bounds.Height * (row + 1) / rows;
            return new Rect(x0, y0, Math.Max(1, x1 - x0), Math.Max(1, y1 - y0));
        }
    }
}
