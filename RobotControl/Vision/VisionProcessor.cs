using OpenCvSharp;
using OpenCvSharp.Aruco;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Threading;
using ZXing;
using ZXing.Common;

namespace Controller.RobotControl.Vision
{
    /// <summary>
    /// Runs blob inspections against each frame, filters detections by zone center-point
    /// containment, annotates frames, and exposes the latest JPEG and results for streaming.
    /// </summary>
    public class VisionProcessor
    {
        private VisionProgram        _program;
        private readonly Camera.CameraDevice _camera;

        private byte[]?       _latestAnnotated;
        private VisionResult? _latestResult;
        private readonly object _lock = new();

        private Thread?       _thread;
        private volatile bool _running;

        private byte[]?       _latestRaw;

        private static readonly int[] JpegParams = { (int)ImwriteFlags.JpegQuality, 80 };

        public string ProgramId => _program.Id;

        public VisionProcessor(VisionProgram program, Camera.CameraDevice camera)
        {
            _program = program;
            _camera  = camera;
        }

        public void Start()
        {
            _running = true;
            _thread  = new Thread(ProcessLoop) { IsBackground = true, Name = $"Vision-{_program.Id}" };
            _thread.Start();
        }

        public void Stop()
        {
            _running = false;
            _thread?.Join(2000);
        }

        public void UpdateProgram(VisionProgram updated)
        {
            Interlocked.Exchange(ref _program, updated);
        }

        public byte[]? GetLatestAnnotated()
        {
            lock (_lock) return _latestAnnotated;
        }

        public byte[]? GetLatestRaw()
        {
            lock (_lock) return _latestRaw;
        }

        public VisionResult? GetLatestResult()
        {
            lock (_lock) return _latestResult;
        }

        // ── Processing loop ───────────────────────────────────────────────────────

        private void ProcessLoop()
        {
            while (_running)
            {
                try
                {
                    var jpeg = _camera.GetLatestFrame();
                    if (jpeg == null) { Thread.Sleep(50); continue; }

                    using var src = Cv2.ImDecode(jpeg, ImreadModes.Color);
                    if (src.Empty()) { Thread.Sleep(50); continue; }

                    var prog   = _program;
                    var result = new VisionResult
                    {
                        ProgramId      = prog.Id,
                        TimestampMs    = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                        Inspections    = new List<InspectionResult>(),
                        ColorResults   = new List<ColorCoverageResult>(),
                        PolygonResults = new List<PolygonResult>(),
                        ArucoResults   = new List<ArucoResult>(),
                        LineResults    = new List<LineResult>(),
                        BarcodeResults = new List<BarcodeResult>(),
                    };

                    using var annotated = src.Clone();

                    // Draw borders only for zones actually used by an enabled inspection.
                    // This reflects any runtime zone override (all inspections then point
                    // at the chosen zone) and avoids outlining unused zones — so the frame
                    // shows the zone genuinely in use rather than every defined zone.
                    var usedZoneIds = new HashSet<string>();
                    void MarkZone(string? id) { if (!string.IsNullOrEmpty(id)) usedZoneIds.Add(id!); }
                    foreach (var i in prog.Inspections)        if (i.Enabled) MarkZone(i.ZoneId);
                    foreach (var i in prog.ColorInspections)   if (i.Enabled) MarkZone(i.ZoneId);
                    foreach (var i in prog.PolygonInspections) if (i.Enabled) MarkZone(i.ZoneId);
                    foreach (var i in prog.ArucoInspections)   if (i.Enabled) MarkZone(i.ZoneId);
                    foreach (var i in prog.LineInspections)    if (i.Enabled) MarkZone(i.ZoneId);
                    foreach (var i in prog.BarcodeInspections) if (i.Enabled) MarkZone(i.ZoneId);

                    foreach (var zone in prog.Zones)
                        if (usedZoneIds.Contains(zone.Id))
                            DrawZoneBorder(annotated, zone.Geometry, src.Width, src.Height, zone.Name);

                    // Run each enabled inspection
                    foreach (var insp in prog.Inspections)
                    {
                        if (!insp.Enabled) continue;
                        var sw = System.Diagnostics.Stopwatch.StartNew();

                        var ir = new InspectionResult
                        {
                            InspectionId = insp.Id,
                            Name         = insp.Name,
                            Blobs        = new List<BlobResult>(),
                        };

                        try
                        {
                            var allBlobs = DetectBlobs(src, insp.BlobParams);

                            // Resolve zone if specified
                            VisionZone? zone = string.IsNullOrEmpty(insp.ZoneId)
                                ? null
                                : prog.Zones.FirstOrDefault(z => z.Id == insp.ZoneId);

                            foreach (var kp in allBlobs)
                            {
                                float bx = kp.Pt.X, by = kp.Pt.Y;

                                if (zone != null && !IsInsideZone(zone.Geometry, bx, by, src.Width, src.Height))
                                    continue;

                                var pt = new OpenCvSharp.Point((int)bx, (int)by);
                                Cv2.Circle(annotated, pt, Math.Max(2, (int)(kp.Size / 2)), new Scalar(0, 255, 0), 2);
                                Cv2.Circle(annotated, pt, 3, new Scalar(0, 255, 0), -1);
                                ir.Blobs.Add(new BlobResult { X = bx, Y = by, Size = kp.Size });
                            }
                        }
                        catch { /* blob detection on this inspection failed — skip */ }

                        result.Inspections.Add(ir);
                        result.Timings[insp.Id] = Math.Round(sw.Elapsed.TotalMilliseconds, 1);
                    }

                    // Run color coverage inspections
                    int colorLabelY = 20;
                    foreach (var colorInsp in prog.ColorInspections)
                    {
                        if (!colorInsp.Enabled) continue;
                        var sw = System.Diagnostics.Stopwatch.StartNew();
                        try
                        {
                            var cr = RunColorInspection(src, annotated, colorInsp, prog.Zones, ref colorLabelY);
                            result.ColorResults.Add(cr);
                        }
                        catch { /* skip failed color inspection */ }
                        result.Timings[colorInsp.Id] = Math.Round(sw.Elapsed.TotalMilliseconds, 1);
                    }

                    // Run polygon inspections
                    foreach (var polyInsp in prog.PolygonInspections)
                    {
                        if (!polyInsp.Enabled) continue;
                        var sw = System.Diagnostics.Stopwatch.StartNew();
                        try
                        {
                            var pr = RunPolygonInspection(src, annotated, polyInsp, prog.Zones, ref colorLabelY);
                            result.PolygonResults.Add(pr);
                        }
                        catch { /* skip failed polygon inspection */ }
                        result.Timings[polyInsp.Id] = Math.Round(sw.Elapsed.TotalMilliseconds, 1);
                    }

                    // Run ArUco inspections
                    foreach (var arucoInsp in prog.ArucoInspections)
                    {
                        if (!arucoInsp.Enabled) continue;
                        var sw = System.Diagnostics.Stopwatch.StartNew();
                        try
                        {
                            var ar = RunArucoInspection(src, annotated, arucoInsp, prog.Zones, ref colorLabelY);
                            result.ArucoResults.Add(ar);
                        }
                        catch { /* skip failed ArUco inspection */ }
                        result.Timings[arucoInsp.Id] = Math.Round(sw.Elapsed.TotalMilliseconds, 1);
                    }

                    // Run line inspections
                    foreach (var lineInsp in prog.LineInspections)
                    {
                        if (!lineInsp.Enabled) continue;
                        var sw = System.Diagnostics.Stopwatch.StartNew();
                        try
                        {
                            var lr = RunLineInspection(src, annotated, lineInsp, prog.Zones, ref colorLabelY);
                            result.LineResults.Add(lr);
                        }
                        catch { /* skip failed line inspection */ }
                        result.Timings[lineInsp.Id] = Math.Round(sw.Elapsed.TotalMilliseconds, 1);
                    }

                    // Run barcode/QR inspections
                    foreach (var barcodeInsp in prog.BarcodeInspections)
                    {
                        if (!barcodeInsp.Enabled) continue;
                        var sw = System.Diagnostics.Stopwatch.StartNew();
                        try
                        {
                            var br = RunBarcodeInspection(src, annotated, barcodeInsp, prog.Zones, ref colorLabelY);
                            result.BarcodeResults.Add(br);
                        }
                        catch { /* skip failed barcode inspection */ }
                        result.Timings[barcodeInsp.Id] = Math.Round(sw.Elapsed.TotalMilliseconds, 1);
                    }

                    Cv2.ImEncode(".jpg", annotated, out var buf, JpegParams);

                    lock (_lock)
                    {
                        _latestRaw       = jpeg;
                        _latestAnnotated = buf;
                        _latestResult    = result;
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Vision] {_program.Id} error: {ex.Message}");
                    Thread.Sleep(200);
                    continue;
                }

                Thread.Sleep(50); // ~20 fps
            }
        }

        // ── Zone containment (pixel-space) ────────────────────────────────────────

        private static bool IsInsideZone(VisionZoneGeometry geom, float blobX, float blobY, int w, int h)
        {
            if (IsRotatedRect(geom))
            {
                // Rotate the point back into the rectangle's own frame rather than testing it
                // against the tilted quad — once untilted it is an axis-aligned compare again.
                double hw = geom.Width * w / 2.0, hh = geom.Height * h / 2.0;
                double a  = -geom.Rotation * Math.PI / 180.0;
                double dx = blobX - (geom.X * w + hw), dy = blobY - (geom.Y * h + hh);
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
                    return blobX >= rx && blobX <= rx + rw && blobY >= ry && blobY <= ry + rh;
                }
                case VisionZoneShape.Circle:
                {
                    double cx = geom.Cx * w, cy = geom.Cy * h;
                    double r  = geom.Radius * Math.Min(w, h);
                    double dx = blobX - cx, dy = blobY - cy;
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
                        if (((yi > blobY) != (yj > blobY)) &&
                            (blobX < (xj - xi) * (blobY - yi) / (yj - yi) + xi))
                            inside = !inside;
                    }
                    return inside;
                }
                default:
                    return true;
            }
        }

        // ── Drawing helpers ───────────────────────────────────────────────────────

        private static void DrawZoneBorder(Mat img, VisionZoneGeometry geom, int w, int h, string label)
        {
            var color = new Scalar(255, 255, 0); // cyan (BGR)
            if (IsRotatedRect(geom)) { DrawRotatedRect(img, geom, w, h, label, color); return; }
            switch (geom.Shape)
            {
                case VisionZoneShape.Rectangle:
                {
                    var tl = new OpenCvSharp.Point((int)(geom.X * w), (int)(geom.Y * h));
                    var br = new OpenCvSharp.Point((int)((geom.X + geom.Width) * w), (int)((geom.Y + geom.Height) * h));
                    Cv2.Rectangle(img, tl, br, color, 2);
                    Cv2.PutText(img, label, new OpenCvSharp.Point(tl.X + 4, tl.Y - 6), HersheyFonts.HersheySimplex, 0.45, color, 1);
                    break;
                }
                case VisionZoneShape.Circle:
                {
                    var center = new OpenCvSharp.Point((int)(geom.Cx * w), (int)(geom.Cy * h));
                    int radius = (int)(geom.Radius * Math.Min(w, h));
                    Cv2.Circle(img, center, radius, color, 2);
                    Cv2.PutText(img, label, new OpenCvSharp.Point(center.X + 4, center.Y - radius - 4), HersheyFonts.HersheySimplex, 0.45, color, 1);
                    break;
                }
                case VisionZoneShape.Polygon:
                {
                    if (geom.Points.Count >= 2)
                    {
                        var pArr = new OpenCvSharp.Point[geom.Points.Count];
                        for (int i = 0; i < geom.Points.Count; i++)
                            pArr[i] = new OpenCvSharp.Point((int)(geom.Points[i][0] * w), (int)(geom.Points[i][1] * h));
                        Cv2.Polylines(img, new[] { pArr }, true, color, 2);
                        Cv2.PutText(img, label, new OpenCvSharp.Point(pArr[0].X + 4, pArr[0].Y - 6), HersheyFonts.HersheySimplex, 0.45, color, 1);
                    }
                    break;
                }
            }
        }

        private static KeyPoint[] DetectBlobs(Mat src, BlobDetectionParams p)
        {
            using var detector = SimpleBlobDetector.Create(new SimpleBlobDetector.Params
            {
                FilterByArea         = true,
                MinArea              = p.MinArea,
                MaxArea              = p.MaxArea,
                FilterByCircularity  = p.FilterByCircularity,
                MinCircularity       = p.MinCircularity,
                FilterByConvexity    = p.FilterByConvexity,
                MinConvexity         = p.MinConvexity,
                FilterByInertia      = p.FilterByInertia,
                MinInertiaRatio      = p.MinInertiaRatio,
                MinThreshold         = p.MinThreshold,
                MaxThreshold         = p.MaxThreshold,
                FilterByColor        = p.FilterByColor,
                BlobColor            = (byte)Math.Clamp(p.BlobColor, 0, 255),
            });

            return detector.Detect(src);
        }

        // ── Color coverage inspection ─────────────────────────────────────────────

        private ColorCoverageResult RunColorInspection(
            Mat src, Mat annotated, ColorCoverageInspection insp,
            List<VisionZone> zones, ref int labelY)
        {
            int w = src.Width, h = src.Height;

            VisionZone? zone = string.IsNullOrEmpty(insp.ZoneId)
                ? null
                : zones.FirstOrDefault(z => z.Id == insp.ZoneId);

            // Build zone mask (white = area to measure)
            using var zoneMask = new Mat(src.Size(), MatType.CV_8UC1, Scalar.Black);
            if (zone == null)
                zoneMask.SetTo(Scalar.White);
            else
                FillZoneMask(zoneMask, zone.Geometry, w, h);

            // Build combined color match mask from all color entries
            using var colorMask = new Mat(src.Size(), MatType.CV_8UC1, Scalar.Black);
            foreach (var ce in insp.Colors)
            {
                int delta = (int)Math.Round(ce.Tolerance * 2.55);
                var lower = new Scalar(
                    Math.Max(0, ce.B - delta),
                    Math.Max(0, ce.G - delta),
                    Math.Max(0, ce.R - delta));
                var upper = new Scalar(
                    Math.Min(255, ce.B + delta),
                    Math.Min(255, ce.G + delta),
                    Math.Min(255, ce.R + delta));
                using var oneMask = new Mat();
                Cv2.InRange(src, lower, upper, oneMask);
                Cv2.BitwiseOr(colorMask, oneMask, colorMask);
            }

            // Restrict match to zone
            using var matchInZone = new Mat();
            Cv2.BitwiseAnd(colorMask, zoneMask, matchInZone);

            int total   = Cv2.CountNonZero(zoneMask);
            int matched = Cv2.CountNonZero(matchInZone);
            double coverage = total > 0 ? matched * 100.0 / total : 0;

            bool InRange(double c) => (!insp.MinCoverage.HasValue || c >= insp.MinCoverage.Value)
                                   && (!insp.MaxCoverage.HasValue || c <= insp.MaxCoverage.Value);

            bool passed = InRange(coverage);

            // Grid: measure each cell on its own. The masks are already built, so an upright
            // cell is just a sub-rect view of them — no extra allocation, and the zone shape
            // still clips each cell because zoneMask carries it. A tilted cell is not a rect,
            // so it needs a mask of its own; that mask is only the size of the cell's own
            // bounding box, which keeps the cost close to the upright path.
            var grid = zone?.Grid;
            List<ColorCellResult>? cells = null;
            if (grid != null && grid.Rows * grid.Cols > 1)
            {
                int rows   = Math.Clamp(grid.Rows, 1, 64);
                int cols   = Math.Clamp(grid.Cols, 1, 64);
                var geom   = zone!.Geometry;
                bool tilted = IsRotatedRect(geom);
                var bounds = ZoneBounds(geom, w, h);
                cells = new List<ColorCellResult>(rows * cols);

                double MeasureCell(int r, int c)
                {
                    int cellTotal, cellMatched;
                    if (tilted)
                    {
                        var quad = CellQuad(geom, rows, cols, r, c, w, h);
                        var box  = QuadBounds(quad, w, h);
                        using var cellMask = new Mat(box.Size, MatType.CV_8UC1, Scalar.Black);
                        Cv2.FillPoly(cellMask, new[] { ToIntPoints(quad)
                            .Select(p => new OpenCvSharp.Point(p.X - box.X, p.Y - box.Y)).ToArray() },
                            Scalar.White);

                        using var zoneBox  = new Mat(zoneMask,    box);
                        using var matchBox = new Mat(matchInZone, box);
                        using var zoneIn   = new Mat();
                        using var matchIn  = new Mat();
                        Cv2.BitwiseAnd(zoneBox,  cellMask, zoneIn);
                        Cv2.BitwiseAnd(matchBox, cellMask, matchIn);
                        cellTotal   = Cv2.CountNonZero(zoneIn);
                        cellMatched = Cv2.CountNonZero(matchIn);
                    }
                    else
                    {
                        var rect = CellRect(bounds, rows, cols, r, c);
                        using var cellZone  = new Mat(zoneMask,    rect);
                        using var cellMatch = new Mat(matchInZone, rect);
                        cellTotal   = Cv2.CountNonZero(cellZone);
                        cellMatched = Cv2.CountNonZero(cellMatch);
                    }
                    return cellTotal > 0 ? cellMatched * 100.0 / cellTotal : 0;
                }

                for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                {
                    double cellCov = MeasureCell(r, c);
                    cells.Add(new ColorCellResult
                    {
                        Row      = r,
                        Col      = c,
                        Index    = r * cols + c,
                        Coverage = Math.Round(cellCov, 1),
                        Passed   = InRange(cellCov),
                    });
                }

                // A zone-wide average hides half-full cells, so on a grid "passed" means
                // every cell passed.
                passed = cells.TrueForAll(cell => cell.Passed);
            }

            // Annotate: semi-transparent tint over matching pixels
            using var tint    = new Mat(annotated.Size(), annotated.Type(), new Scalar(0, 200, 60));
            using var blended = new Mat();
            Cv2.AddWeighted(annotated, 0.55, tint, 0.45, 0, blended);
            blended.CopyTo(annotated, matchInZone);

            // Draw zone border in magenta for color inspections
            if (zone != null)
                DrawColorZoneBorder(annotated, zone.Geometry, w, h, insp.Name);

            if (cells != null)
                DrawGridCells(annotated, zone!.Geometry,
                              Math.Clamp(grid!.Rows, 1, 64), Math.Clamp(grid.Cols, 1, 64), cells, w, h);

            // Name label only — the coverage/pass values are reported as text in the app
            var labelColor = passed ? new Scalar(0, 220, 0) : new Scalar(0, 0, 220);
            string label   = insp.Name;
            Cv2.PutText(annotated, label,
                new OpenCvSharp.Point(6, labelY),
                HersheyFonts.HersheySimplex, 0.5, new Scalar(0, 0, 0), 3);
            Cv2.PutText(annotated, label,
                new OpenCvSharp.Point(6, labelY),
                HersheyFonts.HersheySimplex, 0.5, labelColor, 1);
            labelY += 22;

            return new ColorCoverageResult
            {
                InspectionId = insp.Id,
                Name         = insp.Name,
                Coverage     = Math.Round(coverage, 1),
                Passed       = passed,
                Cells        = cells,
                CellsPassed  = cells?.Count(cell => cell.Passed),
            };
        }

        // ── Rotated rectangles ────────────────────────────────────────────────────
        //
        // A tilt of 0 is the overwhelmingly common case and every caller below branches on
        // it, keeping the original axis-aligned code path intact. That is deliberate: the
        // untilted paths are cheaper (a Rect ROI beats a polygon mask) and already proven,
        // so rotation adds a route rather than replacing one.

        /// <summary>True when this geometry is a rectangle that is actually tilted.</summary>
        internal static bool IsRotatedRect(VisionZoneGeometry geom) =>
            geom.Shape == VisionZoneShape.Rectangle && Math.Abs(geom.Rotation) > 1e-9;

        /// <summary>
        /// Corners of an axis-aligned box expressed in the rectangle's own frame (offsets from
        /// its center), tilted by <paramref name="degrees"/> and placed back on the center.
        /// The rectangle itself and each of its grid cells are both boxes in that frame, so
        /// they rotate through the same code and cannot drift out of alignment.
        /// </summary>
        private static OpenCvSharp.Point2f[] LocalQuad(
            double cx, double cy, double x0, double y0, double x1, double y1, double degrees)
        {
            double a = degrees * Math.PI / 180.0, cos = Math.Cos(a), sin = Math.Sin(a);
            OpenCvSharp.Point2f P(double lx, double ly) => new(
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
        internal static OpenCvSharp.Point2f[] RectCorners(VisionZoneGeometry geom, int w, int h)
        {
            double hw = geom.Width * w / 2.0, hh = geom.Height * h / 2.0;
            return LocalQuad(geom.X * w + hw, geom.Y * h + hh, -hw, -hh, hw, hh, geom.Rotation);
        }

        /// <summary>
        /// Corners of grid cell (row, col) on a tilted rectangle. The lattice is laid out in
        /// the rectangle's own frame and rotated as a whole, so cells stay square to the zone
        /// rather than to the image — which is the entire point of tilting a gridded zone.
        /// </summary>
        internal static OpenCvSharp.Point2f[] CellQuad(
            VisionZoneGeometry geom, int rows, int cols, int row, int col, int w, int h)
        {
            double hw = geom.Width * w / 2.0, hh = geom.Height * h / 2.0;
            double x0 = -hw + 2 * hw * col / (double)cols, x1 = -hw + 2 * hw * (col + 1) / (double)cols;
            double y0 = -hh + 2 * hh * row / (double)rows, y1 = -hh + 2 * hh * (row + 1) / (double)rows;
            return LocalQuad(geom.X * w + hw, geom.Y * h + hh, x0, y0, x1, y1, geom.Rotation);
        }

        private static Rect QuadBounds(OpenCvSharp.Point2f[] quad, int w, int h)
        {
            int x0 = (int)Math.Floor(quad.Min(p => p.X)), x1 = (int)Math.Ceiling(quad.Max(p => p.X));
            int y0 = (int)Math.Floor(quad.Min(p => p.Y)), y1 = (int)Math.Ceiling(quad.Max(p => p.Y));
            return ClampRect(x0, y0, x1 - x0, y1 - y0, w, h);
        }

        private static OpenCvSharp.Point[] ToIntPoints(OpenCvSharp.Point2f[] quad) =>
            quad.Select(p => new OpenCvSharp.Point((int)Math.Round(p.X), (int)Math.Round(p.Y))).ToArray();

        /// <summary>Pixel bounding box of a zone — the rectangle a grid is laid out over.</summary>
        internal static Rect ZoneBounds(VisionZoneGeometry geom, int w, int h)
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

        private static Rect ClampRect(int x, int y, int rw, int rh, int w, int h)
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
        internal static Rect CellRect(Rect bounds, int rows, int cols, int row, int col)
        {
            int x0 = bounds.X + bounds.Width  * col       / cols;
            int x1 = bounds.X + bounds.Width  * (col + 1) / cols;
            int y0 = bounds.Y + bounds.Height * row       / rows;
            int y1 = bounds.Y + bounds.Height * (row + 1) / rows;
            return new Rect(x0, y0, Math.Max(1, x1 - x0), Math.Max(1, y1 - y0));
        }

        /// <summary>Outlines each grid cell green/red so the preview shows which ones failed.</summary>
        private static void DrawGridCells(
            Mat img, VisionZoneGeometry geom, int rows, int cols, List<ColorCellResult> cells, int w, int h)
        {
            bool tilted = IsRotatedRect(geom);
            var bounds  = ZoneBounds(geom, w, h);

            foreach (var cell in cells)
            {
                var color = cell.Passed ? new Scalar(0, 220, 0) : new Scalar(0, 0, 220);
                OpenCvSharp.Point label;

                if (tilted)
                {
                    var quad = ToIntPoints(CellQuad(geom, rows, cols, cell.Row, cell.Col, w, h));
                    Cv2.Polylines(img, new[] { quad }, true, color, 1);
                    // Anchor on the cell's own centre — on a tilt there is no reliable
                    // "top-left" corner, and a centred number stays inside its cell.
                    label = new OpenCvSharp.Point(
                        (int)quad.Average(p => p.X) - 8, (int)quad.Average(p => p.Y) + 4);
                }
                else
                {
                    var rect = CellRect(bounds, rows, cols, cell.Row, cell.Col);
                    Cv2.Rectangle(img, rect, color, 1);
                    label = new OpenCvSharp.Point(rect.X + 3, rect.Y + 13);
                }

                Cv2.PutText(img, $"{cell.Coverage:0}", label,
                    HersheyFonts.HersheySimplex, 0.35, new Scalar(255, 255, 255), 1);
            }
        }

        private static void FillZoneMask(Mat mask, VisionZoneGeometry geom, int w, int h)
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
                    mask[new Rect(rx, ry, rw, rh)].SetTo(Scalar.White);
                    break;
                }
                case VisionZoneShape.Circle:
                {
                    var center = new OpenCvSharp.Point((int)(geom.Cx * w), (int)(geom.Cy * h));
                    int radius = (int)(geom.Radius * Math.Min(w, h));
                    Cv2.Circle(mask, center, Math.Max(1, radius), Scalar.White, -1);
                    break;
                }
                case VisionZoneShape.Polygon:
                {
                    if (geom.Points.Count >= 3)
                    {
                        var pts = geom.Points
                            .Select(p => new OpenCvSharp.Point(
                                Math.Clamp((int)(p[0] * w), 0, w - 1),
                                Math.Clamp((int)(p[1] * h), 0, h - 1)))
                            .ToArray();
                        Cv2.FillPoly(mask, new[] { pts }, Scalar.White);
                    }
                    break;
                }
            }
        }

        // ── Polygon inspection ────────────────────────────────────────────────────

        private PolygonResult RunPolygonInspection(
            Mat src, Mat annotated, PolygonInspection insp,
            List<VisionZone> zones, ref int labelY)
        {
            int w = src.Width, h = src.Height;

            VisionZone? zone = string.IsNullOrEmpty(insp.ZoneId)
                ? null
                : zones.FirstOrDefault(z => z.Id == insp.ZoneId);

            using var gray    = new Mat();
            using var blurred = new Mat();
            using var thresh  = new Mat();
            Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);
            Cv2.GaussianBlur(gray, blurred, new OpenCvSharp.Size(5, 5), 0);
            Cv2.InRange(blurred, new Scalar(insp.MinThreshold), new Scalar(insp.MaxThreshold), thresh);
            if (insp.InvertThreshold) Cv2.BitwiseNot(thresh, thresh);

            Cv2.FindContours(thresh, out var contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);

            int    count    = 0;
            double angle    = 0;
            double centerX  = 0, centerY = 0;
            double bestArea = 0;

            var drawColor = new Scalar(0, 165, 255); // orange (BGR)

            foreach (var contour in contours)
            {
                double area = Cv2.ContourArea(contour);
                if (area < insp.MinArea || area > insp.MaxArea) continue;

                double peri   = Cv2.ArcLength(contour, true);
                var    approx = Cv2.ApproxPolyDP(contour, insp.Epsilon * peri, true);

                if (approx.Length != insp.Sides) continue;

                // Centroid from moments for zone check
                var m = Cv2.Moments(contour);
                if (m.M00 == 0) continue;
                float cx = (float)(m.M10 / m.M00);
                float cy = (float)(m.M01 / m.M00);

                if (zone != null && !IsInsideZone(zone.Geometry, cx, cy, w, h)) continue;

                // Draw approximated polygon
                Cv2.Polylines(annotated, new[] { approx }, true, drawColor, 2);

                // Orientation arrow from MinAreaRect
                var rect = Cv2.MinAreaRect(approx);
                double rad = rect.Angle * Math.PI / 180.0;
                double len = Math.Sqrt(area) * 0.4;
                var cpt = new OpenCvSharp.Point((int)cx, (int)cy);
                var tip = new OpenCvSharp.Point((int)(cx + Math.Cos(rad) * len), (int)(cy + Math.Sin(rad) * len));
                Cv2.ArrowedLine(annotated, cpt, tip, new Scalar(0, 255, 255), 2);

                count++;

                if (area > bestArea)
                {
                    bestArea = area;
                    angle    = Math.Round(rect.Angle, 2);
                    centerX  = Math.Round(cx / w, 4);
                    centerY  = Math.Round(cy / h, 4);
                }
            }

            // Label
            var labelColor = count > 0 ? new Scalar(0, 220, 0) : new Scalar(0, 0, 220);
            string label   = insp.Name;
            Cv2.PutText(annotated, label, new OpenCvSharp.Point(6, labelY), HersheyFonts.HersheySimplex, 0.5, new Scalar(0, 0, 0), 3);
            Cv2.PutText(annotated, label, new OpenCvSharp.Point(6, labelY), HersheyFonts.HersheySimplex, 0.5, labelColor, 1);
            labelY += 22;

            return new PolygonResult
            {
                InspectionId = insp.Id,
                Name         = insp.Name,
                Count        = count,
                Found        = count > 0,
                Angle        = angle,
                CenterX      = centerX,
                CenterY      = centerY,
            };
        }

        // ── ArUco marker detection ────────────────────────────────────────────────

        private ArucoResult RunArucoInspection(
            Mat src, Mat annotated, ArucoInspection insp,
            List<VisionZone> zones, ref int labelY)
        {
            int w = src.Width, h = src.Height;

            VisionZone? zone = string.IsNullOrEmpty(insp.ZoneId)
                ? null
                : zones.FirstOrDefault(z => z.Id == insp.ZoneId);

            var parameters = new DetectorParameters();
            var markers    = new List<ArucoMarkerResult>();
            var drawColor  = new Scalar(0, 255, 127); // spring green (BGR)

            IEnumerable<int> dictIds = insp.DictionaryId == -1
                ? Enumerable.Range(0, 17)
                : new[] { insp.DictionaryId };

            var seenCenters = new HashSet<(int x, int y)>();

            foreach (var dictId in dictIds)
            {
                var dict     = CvAruco.GetPredefinedDictionary((PredefinedDictionaryType)dictId);
                var detector = new ArucoDetector(dict, parameters, new RefineParameters());

                detector.DetectMarkers(src, out var corners, out var ids, out _);

                if (ids == null || ids.Length == 0) continue;

                for (int i = 0; i < ids.Length; i++)
                {
                    var c = corners[i];

                    float minX = c.Min(p => p.X), maxX = c.Max(p => p.X);
                    float minY = c.Min(p => p.Y), maxY = c.Max(p => p.Y);
                    float area = (maxX - minX) * (maxY - minY);

                    if (area < insp.MinMarkerArea || area > insp.MaxMarkerArea) continue;

                    float cx = c.Average(p => p.X);
                    float cy = c.Average(p => p.Y);

                    if (zone != null && !IsInsideZone(zone.Geometry, cx, cy, w, h)) continue;

                    // deduplicate by pixel center to avoid multi-dict double-counting
                    var key = ((int)cx, (int)cy);
                    if (!seenCenters.Add(key)) continue;

                    markers.Add(new ArucoMarkerResult
                    {
                        MarkerId = ids[i],
                        CenterX  = Math.Round(cx / w, 4),
                        CenterY  = Math.Round(cy / h, 4),
                    });

                    var pts = c.Select(p => new OpenCvSharp.Point((int)p.X, (int)p.Y)).ToArray();
                    Cv2.Polylines(annotated, new[] { pts }, isClosed: true, drawColor, 2);
                }
            }

            var labelColor = markers.Count > 0 ? new Scalar(0, 220, 0) : new Scalar(0, 0, 220);
            string label   = insp.Name;
            Cv2.PutText(annotated, label, new OpenCvSharp.Point(6, labelY), HersheyFonts.HersheySimplex, 0.5, new Scalar(0, 0, 0), 3);
            Cv2.PutText(annotated, label, new OpenCvSharp.Point(6, labelY), HersheyFonts.HersheySimplex, 0.5, labelColor, 1);
            labelY += 22;

            return new ArucoResult
            {
                InspectionId = insp.Id,
                Name         = insp.Name,
                Count        = markers.Count,
                Found        = markers.Count > 0,
                Markers      = markers,
            };
        }

        // ── Barcode / QR code detection ──────────────────────────────────────────

        private BarcodeResult RunBarcodeInspection(
            Mat src, Mat annotated, BarcodeInspection insp,
            List<VisionZone> zones, ref int labelY)
        {
            int w = src.Width, h = src.Height;

            VisionZone? zone = string.IsNullOrEmpty(insp.ZoneId)
                ? null
                : zones.FirstOrDefault(z => z.Id == insp.ZoneId);

            // Compute ROI bounding rect from zone geometry
            int roiX = 0, roiY = 0, roiW = w, roiH = h;
            if (zone != null)
            {
                var g = zone.Geometry;
                if (IsRotatedRect(g))
                {
                    // The crop stays axis-aligned, so a tilted zone gets its bounding box —
                    // wider than the zone, but the decoder only needs the barcode inside it.
                    var b = ZoneBounds(g, w, h);
                    roiX = b.X; roiY = b.Y; roiW = b.Width; roiH = b.Height;
                }
                else if (g.Shape == VisionZoneShape.Rectangle)
                {
                    roiX = (int)(g.X * w);
                    roiY = (int)(g.Y * h);
                    roiW = (int)(g.Width  * w);
                    roiH = (int)(g.Height * h);
                }
                else if (g.Shape == VisionZoneShape.Circle)
                {
                    var rad = g.Radius * Math.Min(w, h);
                    roiX = (int)Math.Max(0, g.Cx * w - rad);
                    roiY = (int)Math.Max(0, g.Cy * h - rad);
                    roiW = (int)(rad * 2);
                    roiH = (int)(rad * 2);
                }
                else if (g.Shape == VisionZoneShape.Polygon && g.Points.Count >= 3)
                {
                    var xs = g.Points.Select(p => p[0] * w).ToList();
                    var ys = g.Points.Select(p => p[1] * h).ToList();
                    roiX = (int)xs.Min();
                    roiY = (int)ys.Min();
                    roiW = (int)(xs.Max() - roiX);
                    roiH = (int)(ys.Max() - roiY);
                }
                roiW = Math.Max(1, Math.Min(roiW, w - roiX));
                roiH = Math.Max(1, Math.Min(roiH, h - roiY));
            }

            using var roi = (roiX == 0 && roiY == 0 && roiW == w && roiH == h)
                ? src.Clone()
                : src.SubMat(roiY, roiY + roiH, roiX, roiX + roiW).Clone();

            using var gray = new Mat();
            Cv2.CvtColor(roi, gray, ColorConversionCodes.BGR2GRAY);

            var pixelBytes = new byte[gray.Width * gray.Height];
            Marshal.Copy(gray.Data, pixelBytes, 0, pixelBytes.Length);

            var luminance = new RGBLuminanceSource(pixelBytes, gray.Width, gray.Height,
                RGBLuminanceSource.BitmapFormat.Gray8);

            var reader = new BarcodeReaderGeneric { Options = { TryHarder = true, TryInverted = true } };
            if (insp.Formats.Count > 0)
            {
                reader.Options.PossibleFormats = insp.Formats
                    .Where(f => Enum.TryParse<BarcodeFormat>(f, out _))
                    .Select(f => Enum.Parse<BarcodeFormat>(f))
                    .ToList();
            }

            var rawResults = reader.DecodeMultiple(luminance) ?? Array.Empty<ZXing.Result>();

            var codes     = new List<BarcodeCodeResult>();
            var drawColor = new Scalar(30, 144, 255); // dodger blue (BGR)

            foreach (var r in rawResults)
            {
                if (r?.Text == null) continue;

                // Map result point centers back to full-image coordinates
                double cx, cy;
                if (r.ResultPoints is { Length: > 0 })
                {
                    cx = roiX + r.ResultPoints.Where(p => p != null).Average(p => p.X);
                    cy = roiY + r.ResultPoints.Where(p => p != null).Average(p => p.Y);
                }
                else
                {
                    cx = roiX + roiW / 2.0;
                    cy = roiY + roiH / 2.0;
                }

                codes.Add(new BarcodeCodeResult
                {
                    Value   = r.Text,
                    Format  = r.BarcodeFormat.ToString(),
                    CenterX = Math.Round(cx / w, 4),
                    CenterY = Math.Round(cy / h, 4),
                });

                // Draw outline and label on annotated frame
                if (r.ResultPoints is { Length: >= 2 })
                {
                    var pts = r.ResultPoints
                        .Where(p => p != null)
                        .Select(p => new OpenCvSharp.Point((int)(p.X + roiX), (int)(p.Y + roiY)))
                        .ToArray();
                    Cv2.Polylines(annotated, new[] { pts }, isClosed: pts.Length > 2, drawColor, 2);
                }
            }

            var labelColor = codes.Count > 0 ? new Scalar(0, 220, 0) : new Scalar(0, 0, 220);
            string label = insp.Name;
            Cv2.PutText(annotated, label, new OpenCvSharp.Point(6, labelY), HersheyFonts.HersheySimplex, 0.5, new Scalar(0, 0, 0), 3);
            Cv2.PutText(annotated, label, new OpenCvSharp.Point(6, labelY), HersheyFonts.HersheySimplex, 0.5, labelColor, 1);
            labelY += 22;

            return new BarcodeResult
            {
                InspectionId = insp.Id,
                Name         = insp.Name,
                Count        = codes.Count,
                Found        = codes.Count > 0,
                Codes        = codes,
            };
        }

        // ── Polygon debug frame ───────────────────────────────────────────────────

        public byte[]? GetPolygonDebugFrame(string inspectionId)
        {
            var insp = _program.PolygonInspections.FirstOrDefault(i => i.Id == inspectionId);
            if (insp == null) return null;

            byte[]? raw;
            lock (_lock) raw = _latestRaw;
            if (raw == null) return null;

            using var src = Cv2.ImDecode(raw, ImreadModes.Color);
            if (src.Empty()) return null;

            int w = src.Width, h = src.Height;

            using var gray    = new Mat();
            using var blurred = new Mat();
            using var thresh  = new Mat();
            Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);
            Cv2.GaussianBlur(gray, blurred, new OpenCvSharp.Size(5, 5), 0);
            Cv2.InRange(blurred, new Scalar(insp.MinThreshold), new Scalar(insp.MaxThreshold), thresh);
            if (insp.InvertThreshold) Cv2.BitwiseNot(thresh, thresh);

            // Debug canvas: threshold mask in color so overlays are visible
            using var debug = new Mat();
            Cv2.CvtColor(thresh, debug, ColorConversionCodes.GRAY2BGR);

            Cv2.FindContours(thresh, out var contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);

            int areaPass = 0, matchCount = 0;
            var orange = new Scalar(0, 140, 255);
            var green  = new Scalar(0, 210, 0);
            var gray80 = new Scalar(80, 80, 80);

            foreach (var contour in contours)
            {
                double area = Cv2.ContourArea(contour);

                if (area < insp.MinArea || area > insp.MaxArea)
                {
                    Cv2.DrawContours(debug, new[] { contour }, -1, gray80, 1);
                    continue;
                }

                areaPass++;
                double peri   = Cv2.ArcLength(contour, true);
                var    approx = Cv2.ApproxPolyDP(contour, insp.Epsilon * peri, true);
                var    m      = Cv2.Moments(contour);
                int    cx     = m.M00 > 0 ? (int)(m.M10 / m.M00) : 0;
                int    cy     = m.M00 > 0 ? (int)(m.M01 / m.M00) : 0;

                if (approx.Length != insp.Sides)
                {
                    Cv2.DrawContours(debug, new[] { approx }, -1, orange, 2);
                    string sideLabel = $"{approx.Length}s";
                    Cv2.PutText(debug, sideLabel, new OpenCvSharp.Point(cx + 4, cy), HersheyFonts.HersheySimplex, 0.5, new Scalar(0,0,0), 3);
                    Cv2.PutText(debug, sideLabel, new OpenCvSharp.Point(cx + 4, cy), HersheyFonts.HersheySimplex, 0.5, orange, 1);
                }
                else
                {
                    Cv2.DrawContours(debug, new[] { approx }, -1, green, 2);
                    var    rect      = Cv2.MinAreaRect(approx);
                    string angleLabel = $"{rect.Angle:F1}deg";
                    Cv2.PutText(debug, angleLabel, new OpenCvSharp.Point(cx + 4, cy), HersheyFonts.HersheySimplex, 0.5, new Scalar(0,0,0), 3);
                    Cv2.PutText(debug, angleLabel, new OpenCvSharp.Point(cx + 4, cy), HersheyFonts.HersheySimplex, 0.5, green, 1);
                    matchCount++;
                }
            }

            // Info strip at bottom
            string[] lines =
            {
                $"Thresh {insp.MinThreshold}-{insp.MaxThreshold}{(insp.InvertThreshold ? " (inverted)" : "")}   Eps {insp.Epsilon}   Need {insp.Sides} sides",
                $"Contours: {contours.Length}   Area pass: {areaPass}   Matched: {matchCount}",
                "Gray=area fail   Orange=wrong sides   Green=matched",
            };
            int stripH = lines.Length * 22 + 8;
            int stripY = Math.Max(0, h - stripH);
            debug[new Rect(0, stripY, w, h - stripY)].SetTo(new Scalar(18, 18, 18));
            int ly = stripY + 18;
            foreach (var line in lines)
            {
                Cv2.PutText(debug, line, new OpenCvSharp.Point(6, ly), HersheyFonts.HersheySimplex, 0.42, new Scalar(180, 180, 180), 1);
                ly += 22;
            }

            Cv2.ImEncode(".jpg", debug, out var buf, new[] { (int)ImwriteFlags.JpegQuality, 88 });
            return buf;
        }

        // ── Line detection inspection ─────────────────────────────────────────────

        private LineResult RunLineInspection(
            Mat src, Mat annotated, LineInspection insp,
            List<VisionZone> zones, ref int labelY)
        {
            int w = src.Width, h = src.Height;

            VisionZone? zone = string.IsNullOrEmpty(insp.ZoneId)
                ? null
                : zones.FirstOrDefault(z => z.Id == insp.ZoneId);

            using var gray  = new Mat();
            using var edges = new Mat();
            Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);
            Cv2.Canny(gray, edges, insp.CannyThreshold1, insp.CannyThreshold2);

            if (zone != null)
            {
                using var zoneMask = new Mat(src.Size(), MatType.CV_8UC1, Scalar.Black);
                FillZoneMask(zoneMask, zone.Geometry, w, h);
                Cv2.BitwiseAnd(edges, zoneMask, edges);
            }

            var rawLines  = Cv2.HoughLinesP(edges, 1, Math.PI / 180,
                insp.HoughThreshold, insp.MinLineLength, insp.MaxLineGap);

            var segments  = new List<LineSegment>();
            var drawColor = new Scalar(168, 85, 247); // violet (BGR: 247,85,168)

            foreach (var seg in rawLines)
            {
                double dx = seg.P2.X - seg.P1.X, dy = seg.P2.Y - seg.P1.Y;
                double angleDeg = Math.Atan2(dy, dx) * 180.0 / Math.PI;
                angleDeg = ((angleDeg % 180) + 180) % 180;
                double length = Math.Sqrt(dx * dx + dy * dy);

                if (insp.FilterByAngle && (angleDeg < insp.MinAngle || angleDeg > insp.MaxAngle))
                    continue;

                float mx = (seg.P1.X + seg.P2.X) / 2f, my = (seg.P1.Y + seg.P2.Y) / 2f;
                if (zone != null && !IsInsideZone(zone.Geometry, mx, my, w, h)) continue;

                segments.Add(new LineSegment
                {
                    X1     = Math.Round(seg.P1.X / (double)w, 4),
                    Y1     = Math.Round(seg.P1.Y / (double)h, 4),
                    X2     = Math.Round(seg.P2.X / (double)w, 4),
                    Y2     = Math.Round(seg.P2.Y / (double)h, 4),
                    Angle  = Math.Round(angleDeg, 2),
                    Length = Math.Round(length, 1),
                });

                Cv2.Line(annotated,
                    new OpenCvSharp.Point(seg.P1.X, seg.P1.Y),
                    new OpenCvSharp.Point(seg.P2.X, seg.P2.Y),
                    drawColor, 2);
            }

            var labelColor = segments.Count > 0 ? new Scalar(0, 220, 0) : new Scalar(0, 0, 220);
            string label   = insp.Name;
            Cv2.PutText(annotated, label, new OpenCvSharp.Point(6, labelY), HersheyFonts.HersheySimplex, 0.5, new Scalar(0, 0, 0), 3);
            Cv2.PutText(annotated, label, new OpenCvSharp.Point(6, labelY), HersheyFonts.HersheySimplex, 0.5, labelColor, 1);
            labelY += 22;

            return new LineResult
            {
                InspectionId = insp.Id,
                Name         = insp.Name,
                Count        = segments.Count,
                Found        = segments.Count > 0,
                Lines        = segments,
            };
        }

        // ── Line debug frame ──────────────────────────────────────────────────────

        public byte[]? GetLineDebugFrame(string inspectionId)
        {
            var insp = _program.LineInspections.FirstOrDefault(i => i.Id == inspectionId);
            if (insp == null) return null;

            byte[]? raw;
            lock (_lock) raw = _latestRaw;
            if (raw == null) return null;

            using var src = Cv2.ImDecode(raw, ImreadModes.Color);
            if (src.Empty()) return null;

            int w = src.Width, h = src.Height;

            using var gray  = new Mat();
            using var edges = new Mat();
            Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);
            Cv2.Canny(gray, edges, insp.CannyThreshold1, insp.CannyThreshold2);

            if (!string.IsNullOrEmpty(insp.ZoneId))
            {
                var zone = _program.Zones.FirstOrDefault(z => z.Id == insp.ZoneId);
                if (zone != null)
                {
                    using var zoneMask = new Mat(src.Size(), MatType.CV_8UC1, Scalar.Black);
                    FillZoneMask(zoneMask, zone.Geometry, w, h);
                    Cv2.BitwiseAnd(edges, zoneMask, edges);
                }
            }

            using var debug = new Mat();
            Cv2.CvtColor(edges, debug, ColorConversionCodes.GRAY2BGR);

            var rawLines = Cv2.HoughLinesP(edges, 1, Math.PI / 180,
                insp.HoughThreshold, insp.MinLineLength, insp.MaxLineGap);

            int total = rawLines.Length, matched = 0;
            var green  = new Scalar(0, 210, 0);
            var orange = new Scalar(0, 140, 255);

            foreach (var seg in rawLines)
            {
                double dx = seg.P2.X - seg.P1.X, dy = seg.P2.Y - seg.P1.Y;
                double angleDeg = Math.Atan2(dy, dx) * 180.0 / Math.PI;
                angleDeg = ((angleDeg % 180) + 180) % 180;

                bool pass = !insp.FilterByAngle || (angleDeg >= insp.MinAngle && angleDeg <= insp.MaxAngle);
                var color = pass ? green : orange;

                Cv2.Line(debug,
                    new OpenCvSharp.Point(seg.P1.X, seg.P1.Y),
                    new OpenCvSharp.Point(seg.P2.X, seg.P2.Y),
                    color, 2);

                int mx = (seg.P1.X + seg.P2.X) / 2, my = (seg.P1.Y + seg.P2.Y) / 2;
                string lbl = $"{angleDeg:F0}°";
                Cv2.PutText(debug, lbl, new OpenCvSharp.Point(mx + 4, my), HersheyFonts.HersheySimplex, 0.4, new Scalar(0, 0, 0), 3);
                Cv2.PutText(debug, lbl, new OpenCvSharp.Point(mx + 4, my), HersheyFonts.HersheySimplex, 0.4, color, 1);

                if (pass) matched++;
            }

            string angleDesc = insp.FilterByAngle ? $"   Angle {insp.MinAngle:F0}°–{insp.MaxAngle:F0}°" : "";
            string[] lines =
            {
                $"Canny {insp.CannyThreshold1}/{insp.CannyThreshold2}   Threshold {insp.HoughThreshold}   MinLen {insp.MinLineLength}   MaxGap {insp.MaxLineGap}{angleDesc}",
                $"Segments: {total}   Matched: {matched}",
                insp.FilterByAngle ? "Orange=angle filtered   Green=matched" : "Green=all matched segments",
            };
            int stripH = lines.Length * 22 + 8;
            int stripY = Math.Max(0, h - stripH);
            debug[new Rect(0, stripY, w, h - stripY)].SetTo(new Scalar(18, 18, 18));
            int ly = stripY + 18;
            foreach (var line in lines)
            {
                Cv2.PutText(debug, line, new OpenCvSharp.Point(6, ly), HersheyFonts.HersheySimplex, 0.42, new Scalar(180, 180, 180), 1);
                ly += 22;
            }

            Cv2.ImEncode(".jpg", debug, out var buf, new[] { (int)ImwriteFlags.JpegQuality, 88 });
            return buf;
        }

        /// <summary>
        /// Outline for a tilted rectangle. Shared by both border colors — the label is anchored
        /// to whichever corner sits highest so it does not end up written across the shape.
        /// </summary>
        private static void DrawRotatedRect(
            Mat img, VisionZoneGeometry geom, int w, int h, string label, Scalar color)
        {
            var pts = ToIntPoints(RectCorners(geom, w, h));
            Cv2.Polylines(img, new[] { pts }, true, color, 2);
            var top = pts.OrderBy(p => p.Y).First();
            Cv2.PutText(img, label, new OpenCvSharp.Point(top.X + 4, Math.Max(12, top.Y - 6)),
                        HersheyFonts.HersheySimplex, 0.45, color, 1);
        }

        private static void DrawColorZoneBorder(Mat img, VisionZoneGeometry geom, int w, int h, string label)
        {
            var color = new Scalar(255, 0, 255); // magenta (BGR)
            if (IsRotatedRect(geom)) { DrawRotatedRect(img, geom, w, h, label, color); return; }
            switch (geom.Shape)
            {
                case VisionZoneShape.Rectangle:
                {
                    var tl = new OpenCvSharp.Point((int)(geom.X * w), (int)(geom.Y * h));
                    var br = new OpenCvSharp.Point((int)((geom.X + geom.Width) * w), (int)((geom.Y + geom.Height) * h));
                    Cv2.Rectangle(img, tl, br, color, 2);
                    Cv2.PutText(img, label, new OpenCvSharp.Point(tl.X + 4, tl.Y - 6), HersheyFonts.HersheySimplex, 0.45, color, 1);
                    break;
                }
                case VisionZoneShape.Circle:
                {
                    var center = new OpenCvSharp.Point((int)(geom.Cx * w), (int)(geom.Cy * h));
                    int radius = (int)(geom.Radius * Math.Min(w, h));
                    Cv2.Circle(img, center, radius, color, 2);
                    Cv2.PutText(img, label, new OpenCvSharp.Point(center.X + 4, center.Y - radius - 4), HersheyFonts.HersheySimplex, 0.45, color, 1);
                    break;
                }
                case VisionZoneShape.Polygon:
                {
                    if (geom.Points.Count >= 2)
                    {
                        var pArr = geom.Points
                            .Select(p => new OpenCvSharp.Point((int)(p[0] * w), (int)(p[1] * h)))
                            .ToArray();
                        Cv2.Polylines(img, new[] { pArr }, true, color, 2);
                        Cv2.PutText(img, label, new OpenCvSharp.Point(pArr[0].X + 4, pArr[0].Y - 6), HersheyFonts.HersheySimplex, 0.45, color, 1);
                    }
                    break;
                }
            }
        }
    }
}
