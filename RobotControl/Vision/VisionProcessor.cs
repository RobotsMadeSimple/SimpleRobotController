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
                        ProgramId    = prog.Id,
                        TimestampMs  = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                        Inspections  = new List<InspectionResult>(),
                        ColorResults = new List<ColorCoverageResult>(),
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
