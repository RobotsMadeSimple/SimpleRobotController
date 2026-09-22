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
                {
                    var pts = geom.Points;
                    int n   = pts.Count;
                    if (n < 3) return false;
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
                {
                    if (geom.Points.Count >= 3)
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

        // ── Rotated rectangles ───────────────────────────────────────────────────

        /// <summary>True when this geometry is a rectangle that is actually tilted.</summary>
        public static bool IsRotatedRect(VisionZoneGeometry geom) =>
            geom.Shape == VisionZoneShape.Rectangle && Math.Abs(geom.Rotation) > 1e-9;

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
            double hw = geom.Width * w / 2.0, hh = geom.Height * h / 2.0;
            double x0 = -hw + 2 * hw * col / (double)cols, x1 = -hw + 2 * hw * (col + 1) / (double)cols;
            double y0 = -hh + 2 * hh * row / (double)rows, y1 = -hh + 2 * hh * (row + 1) / (double)rows;
            return LocalQuad(geom.X * w + hw, geom.Y * h + hh, x0, y0, x1, y1, geom.Rotation);
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
