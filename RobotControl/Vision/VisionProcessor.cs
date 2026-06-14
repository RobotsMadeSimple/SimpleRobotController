using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;

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
                    };

                    using var annotated = src.Clone();

                    // Draw all zone borders as spatial context
                    foreach (var zone in prog.Zones)
                        DrawZoneBorder(annotated, zone.Geometry, src.Width, src.Height, zone.Name);

                    // Run each enabled inspection
                    foreach (var insp in prog.Inspections)
                    {
                        if (!insp.Enabled) continue;

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
                    }

                    // Run color coverage inspections
                    int colorLabelY = 20;
                    foreach (var colorInsp in prog.ColorInspections)
                    {
                        if (!colorInsp.Enabled) continue;
                        try
                        {
                            var cr = RunColorInspection(src, annotated, colorInsp, prog.Zones, ref colorLabelY);
                            result.ColorResults.Add(cr);
                        }
                        catch { /* skip failed color inspection */ }
                    }

                    // Run polygon inspections
                    foreach (var polyInsp in prog.PolygonInspections)
                    {
                        if (!polyInsp.Enabled) continue;
                        try
                        {
                            var pr = RunPolygonInspection(src, annotated, polyInsp, prog.Zones, ref colorLabelY);
                            result.PolygonResults.Add(pr);
                        }
                        catch { /* skip failed polygon inspection */ }
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

            bool passed = (!insp.MinCoverage.HasValue || coverage >= insp.MinCoverage.Value)
                       && (!insp.MaxCoverage.HasValue || coverage <= insp.MaxCoverage.Value);

            // Annotate: semi-transparent tint over matching pixels
            using var tint    = new Mat(annotated.Size(), annotated.Type(), new Scalar(0, 200, 60));
            using var blended = new Mat();
            Cv2.AddWeighted(annotated, 0.55, tint, 0.45, 0, blended);
            blended.CopyTo(annotated, matchInZone);

            // Draw zone border in magenta for color inspections
            if (zone != null)
                DrawColorZoneBorder(annotated, zone.Geometry, w, h, insp.Name);

            // Coverage / pass label
            var labelColor = passed ? new Scalar(0, 220, 0) : new Scalar(0, 0, 220);
            string label   = $"{insp.Name}: {coverage:F1}%  {(passed ? "PASS" : "FAIL")}";
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
            };
        }

        private static void FillZoneMask(Mat mask, VisionZoneGeometry geom, int w, int h)
        {
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
                Cv2.PutText(annotated, $"{rect.Angle:F1}deg",
                    new OpenCvSharp.Point((int)cx + 5, (int)cy - 5),
                    HersheyFonts.HersheySimplex, 0.4, drawColor, 1);

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
            string label   = count > 0
                ? $"{insp.Name}: {count} polygon(s) · {angle:F1}deg"
                : $"{insp.Name}: none";
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

        private static void DrawColorZoneBorder(Mat img, VisionZoneGeometry geom, int w, int h, string label)
        {
            var color = new Scalar(255, 0, 255); // magenta (BGR)
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
