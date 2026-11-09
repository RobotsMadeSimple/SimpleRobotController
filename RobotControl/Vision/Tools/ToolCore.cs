using OpenCvSharp;
using OpenCvSharp.Aruco;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision.Tools
{
    using Point = OpenCvSharp.Point;

    /// <summary>
    /// One building-block tool type. Implementations are stateless; anything cached across
    /// frames (ArUco detectors, decoded templates) lives on the <see cref="VisionRuntime"/>.
    /// </summary>
    internal interface IVisionTool
    {
        /// <summary>False for tools that work from other tools' results (makeFixture, distance, angle).</summary>
        bool UsesRegion => true;
        /// <summary>True when the tool outputs a pose (and so can be a fixture and has a reference).</summary>
        bool MakesPose => false;
        /// <summary>The value each grid cell is judged on, or null when the tool has no grid.</summary>
        string? MainValue(VisionTool tool) => null;
        /// <summary>Tool ids this tool reads results from (params.sources).</summary>
        IReadOnlyList<string> Sources(VisionTool tool) => Array.Empty<string>();

        void Run(ToolContext c);
    }

    /// <summary>Everything a tool needs for one frame.</summary>
    internal sealed class ToolContext
    {
        public required LayerCache        Layers    { get; init; }
        public required VisionTool        Tool      { get; init; }
        public required PixelRegion       Region    { get; init; }
        public required FixtureTransform  Transform { get; init; }
        public required VisionResult      Result    { get; init; }
        public required ToolResult        Out       { get; init; }
        public required VisionRuntime     Runtime   { get; init; }

        public int W => Layers.Width;
        public int H => Layers.Height;
        public ParamReader P => Tool.P;
        /// <summary>The tool's input layer as it is (colour or single channel).</summary>
        public Mat Input => Layers.Get(Tool.Input);
        /// <summary>The tool's input layer as 8-bit gray.</summary>
        public Mat Gray  => Layers.Gray(Tool.Input);

        public ToolResult? Source(string id) => Result.Tool(id);

        /// <summary>A second drawn region (e.g. findCorner's side B), moved by the same fixture.</summary>
        public PixelRegion ExtraRegion(VisionRegion r) => PixelRegion.FromNormalized(r, W, H).Transform(Transform);

        public void Fail(string message)
        {
            Out.Status  = ToolStatus.Fail;
            Out.Message = message;
        }
    }

    /// <summary>Per-processor cache of expensive native objects shared across frames.</summary>
    internal sealed class VisionRuntime : IDisposable
    {
        /// <summary>The 17 classic ArUco dictionaries ("any" = -1). AprilTag families are left out: 16h5 false-fires on ordinary scenes.</summary>
        internal static readonly PredefinedDictionaryType[] AllDictionaryTypes =
            Enum.GetValues<PredefinedDictionaryType>().Distinct().Where(t => (int)t <= 16).ToArray();

        private readonly Dictionary<PredefinedDictionaryType, (Dictionary Dict, ArucoDetector Detector)> _aruco = new();
        private readonly Dictionary<string, (string Key, Mat Mat)> _templates = new();
        private readonly object _lock = new();

        public ArucoDetector Detector(PredefinedDictionaryType type)
        {
            lock (_lock)
            {
                if (_aruco.TryGetValue(type, out var c)) return c.Detector;
                var dict = CvAruco.GetPredefinedDictionary(type);
                var det  = new ArucoDetector(dict, new DetectorParameters(), new RefineParameters());
                _aruco[type] = (dict, det);
                return det;
            }
        }

        /// <summary>A decoded template image for a tool, re-decoded only when its PNG changes. Owned by the runtime.</summary>
        public Mat? Template(string toolId, string pngBase64)
        {
            lock (_lock)
            {
                if (_templates.TryGetValue(toolId, out var c) && c.Key == pngBase64) return c.Mat;
                byte[] bytes;
                try { bytes = Convert.FromBase64String(pngBase64); }
                catch (FormatException) { return null; }
                var mat = Cv2.ImDecode(bytes, ImreadModes.Grayscale);
                if (mat.Empty()) { mat.Dispose(); return null; }
                if (_templates.TryGetValue(toolId, out var old)) old.Mat.Dispose();
                _templates[toolId] = (pngBase64, mat);
                return mat;
            }
        }

        public void Dispose()
        {
            lock (_lock)
            {
                foreach (var e in _aruco.Values) { e.Detector.Dispose(); e.Dict.Dispose(); }
                foreach (var t in _templates.Values) t.Mat.Dispose();
                _aruco.Clear();
                _templates.Clear();
            }
        }
    }

    /// <summary>The tool catalogue (docs/vision-tools.md).</summary>
    internal static class ToolRegistry
    {
        private static readonly Dictionary<string, IVisionTool> _tools = new()
        {
            ["findLine"]     = new FindLineTool(),
            ["findCorner"]   = new FindCornerTool(),
            ["findCircle"]   = new FindCircleTool(),
            ["blob"]         = new BlobTool(),
            ["contourMatch"] = new ContourMatchTool(),
            ["patternMatch"] = new PatternMatchTool(),
            ["aruco"]        = new ArucoTool(),
            ["makeFixture"]  = new MakeFixtureTool(),
            ["caliper"]      = new CaliperTool(),
            ["distance"]     = new DistanceTool(),
            ["angle"]        = new AngleTool(),
            ["corners"]      = new CornersTool(),
            ["lines"]        = new LinesTool(),
            ["pixelCount"]   = new PixelCountTool(),
            ["intensity"]    = new IntensityTool(),
            ["edgePixels"]   = new EdgePixelsTool(),
            ["polygon"]      = new PolygonTool(),
            ["barcode"]      = new BarcodeTool(),
        };

        public static IReadOnlyCollection<string> Types => _tools.Keys;

        public static IVisionTool? Get(string? type) =>
            type != null && _tools.TryGetValue(type, out var t) ? t : null;
    }

    /// <summary>Shared measurement helpers.</summary>
    internal static class ToolMath
    {
        public const int MaxGridDimension = 64;

        /// <summary>The region's mask cut to its bounding box (both owned by the caller via the Mat).</summary>
        public static (Mat Mask, Rect Box) RegionMask(PixelRegion region, int w, int h)
        {
            var box = region.Bounds(w, h);
            var mask = new Mat(Math.Max(1, box.Height), Math.Max(1, box.Width), MatType.CV_8UC1, Scalar.Black);
            if (box.Width > 0 && box.Height > 0) region.Fill(mask, new Point(box.X, box.Y));
            return (mask, box);
        }

        /// <summary>Otsu's threshold over the masked pixels of an 8-bit gray image.</summary>
        public static double OtsuLevel(Mat gray, Mat? mask)
        {
            using var histMat = new Mat();
            using var noMask = new Mat();
            Cv2.CalcHist(new[] { gray }, new[] { 0 }, mask ?? noMask, histMat, 1, new[] { 256 }, new[] { new Rangef(0, 256) });
            var hist = new double[256];
            double n = 0;
            for (int i = 0; i < 256; i++) { hist[i] = histMat.At<float>(i); n += hist[i]; }
            if (n == 0) return 128;
            double sum = 0;
            for (int i = 0; i < 256; i++) sum += i * hist[i];
            double sumB = 0, wB = 0, best = -1, level = 128;
            for (int t = 0; t < 256; t++)
            {
                wB += hist[t];
                if (wB == 0) continue;
                double wF = n - wB;
                if (wF == 0) break;
                sumB += t * hist[t];
                double mB = sumB / wB, mF = (sum - sumB) / wF;
                double between = wB * wF * (mB - mF) * (mB - mF);
                if (between > best) { best = between; level = t; }
            }
            return level;
        }

        /// <summary>A Mat's pixels copied out row by row (8-bit, single channel).</summary>
        public static byte[] Bytes(Mat m)
        {
            int rows = m.Rows, cols = m.Cols;
            var data = new byte[rows * cols];
            using var c = m.IsContinuous() ? null : m.Clone();
            var src = c ?? m;
            System.Runtime.InteropServices.Marshal.Copy(src.Data, data, 0, data.Length);
            return data;
        }

        /// <summary>A Mat's pixels copied out row by row (32-bit float, single channel).</summary>
        public static float[] Floats(Mat m)
        {
            int rows = m.Rows, cols = m.Cols;
            var data = new float[rows * cols];
            using var c = m.IsContinuous() ? null : m.Clone();
            var src = c ?? m;
            System.Runtime.InteropServices.Marshal.Copy(src.Data, data, 0, data.Length);
            return data;
        }

        /// <summary>
        /// Object pixels (255) inside the region: gray below (polarity "dark") or above ("light")
        /// a level — Otsu over the region's own pixels, or params.level when threshold is "fixed".
        /// Returned cut to the region's bounding box. Caller disposes.
        /// </summary>
        public static (Mat Binary, Rect Box) Binarize(ToolContext c)
        {
            var (mask, box) = RegionMask(c.Region, c.W, c.H);
            using (mask)
            {
                var bin = new Mat(mask.Size(), MatType.CV_8UC1, Scalar.Black);
                if (box.Width <= 0 || box.Height <= 0) return (bin, box);
                using var gray = new Mat(c.Gray, box);
                bool dark = c.P.Str("polarity", "dark") != "light";
                double level = c.P.Str("threshold", "otsu") == "fixed" ? c.P.Num("level", 128) : OtsuLevel(gray, mask);
                Cv2.Threshold(gray, bin, level, 255, dark ? ThresholdTypes.BinaryInv : ThresholdTypes.Binary);
                Cv2.BitwiseAnd(bin, mask, bin);
                return (bin, box);
            }
        }

        /// <summary>Principal-axis angle of a contour (deg, (−90, 90]), from its second moments.</summary>
        public static double Orientation(Moments m)
        {
            if (m.M00 == 0) return 0;
            return 0.5 * Math.Atan2(2 * m.Mu11, m.Mu20 - m.Mu02) * 180 / Math.PI;
        }

        /// <summary>
        /// Edge candidates along a 1-D profile: peaks of the smoothed profile's gradient, refined
        /// to sub-sample. An edge's contrast is its intensity step, s[i+k] − s[i−k] of the smoothed
        /// profile (k = 3 + 2·smoothing); edges below <paramref name="minContrast"/> are dropped,
        /// as are edges of the wrong sign (polarity "darkToLight" = rising along the profile,
        /// "lightToDark" = falling, "any").
        /// </summary>
        public static List<(double Pos, double Contrast)> Edges(double[] profile, double smoothing, double minContrast, string polarity)
        {
            var s = Smooth(profile, smoothing);
            int n = s.Length, k = 3 + (int)Math.Ceiling(2 * Math.Max(0, smoothing));
            var d = new double[n];
            for (int i = 1; i < n - 1; i++) d[i] = (s[i + 1] - s[i - 1]) / 2;
            var edges = new List<(double, double)>();
            for (int i = 1; i < n - 1; i++)
            {
                double v = d[i], av = Math.Abs(v), al = Math.Abs(d[i - 1]), ar = Math.Abs(d[i + 1]);
                if (av <= 1e-9 || av < ar || av <= al) continue;   // a tie with the left neighbour was taken there
                if (polarity == "darkToLight" && v <= 0) continue;
                if (polarity == "lightToDark" && v >= 0) continue;
                double step = s[Math.Min(n - 1, i + k)] - s[Math.Max(0, i - k)];
                if (Math.Abs(step) < minContrast || Math.Sign(step) != Math.Sign(v)) continue;
                double den = al - 2 * av + ar;
                double off = Math.Abs(den) > 1e-9 ? 0.5 * (al - ar) / den : 0;
                edges.Add((i + Math.Clamp(off, -0.5, 0.5), step));
            }
            return edges;
        }

        public static double[] Smooth(double[] p, double sigma)
        {
            if (sigma <= 0.01 || p.Length < 3) return (double[])p.Clone();
            int r = (int)Math.Ceiling(sigma * 3);
            var kern = new double[2 * r + 1];
            double sum = 0;
            for (int i = -r; i <= r; i++) { kern[i + r] = Math.Exp(-i * i / (2 * sigma * sigma)); sum += kern[i + r]; }
            var o = new double[p.Length];
            for (int i = 0; i < p.Length; i++)
            {
                double acc = 0, wsum = 0;
                for (int j = -r; j <= r; j++)
                {
                    int idx = i + j;
                    if (idx < 0 || idx >= p.Length) continue;
                    acc += p[idx] * kern[j + r];
                    wsum += kern[j + r];
                }
                o[i] = acc / wsum;
            }
            return o;
        }

        /// <summary>Bilinear sample of 8-bit gray pixels (row-major, w × h); NaN outside the image.</summary>
        public static double Sample(byte[] g, int w, int h, double x, double y)
        {
            if (x < 0 || y < 0 || x > w - 1 || y > h - 1) return double.NaN;
            int x0 = (int)x, y0 = (int)y, x1 = Math.Min(w - 1, x0 + 1), y1 = Math.Min(h - 1, y0 + 1);
            double fx = x - x0, fy = y - y0;
            return g[y0 * w + x0] * (1 - fx) * (1 - fy) + g[y0 * w + x1] * fx * (1 - fy) + g[y1 * w + x0] * (1 - fx) * fy + g[y1 * w + x1] * fx * fy;
        }

        /// <summary>
        /// A rectangular region's pixels unrolled into an upright patch: patch x runs along the
        /// region's width (TL→TR), patch y along its height (TL→BL). Works for tilted and
        /// perspective (quad) regions. Returns the patch and the homography patch → image.
        /// </summary>
        public static (Mat Patch, double[][] ToImage) Unroll(Mat gray, PixelRegion region)
        {
            var q = region.Corners();
            double pw = (Geo.Dist(q[0], q[1]) + Geo.Dist(q[3], q[2])) / 2, ph = (Geo.Dist(q[0], q[3]) + Geo.Dist(q[1], q[2])) / 2;
            int w = Math.Max(2, (int)Math.Round(pw)), h = Math.Max(2, (int)Math.Round(ph));
            var dst = new[] { new Point2f(0, 0), new Point2f(w, 0), new Point2f(w, h), new Point2f(0, h) };
            var src = q.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray();
            using var toPatch = Cv2.GetPerspectiveTransform(src, dst);
            using var toImage = Cv2.GetPerspectiveTransform(dst, src);
            var patch = new Mat();
            Cv2.WarpPerspective(gray, patch, toPatch, new Size(w, h), InterpolationFlags.Linear, BorderTypes.Replicate);
            return (patch, Calibration.Homography.FromMat(toImage));
        }

        public static Point2d Map(double[][] h, double x, double y)
        {
            var (mx, my) = Calibration.Homography.Apply(h, x, y);
            return new Point2d(mx, my);
        }

        /// <summary>Total-least-squares line through points: a point on it and its unit direction.</summary>
        public static (Point2d P, Point2d D) FitLine(IReadOnlyList<Point2d> pts)
        {
            double mx = pts.Average(p => p.X), my = pts.Average(p => p.Y);
            double sxx = 0, syy = 0, sxy = 0;
            foreach (var p in pts) { sxx += (p.X - mx) * (p.X - mx); syy += (p.Y - my) * (p.Y - my); sxy += (p.X - mx) * (p.Y - my); }
            double th = 0.5 * Math.Atan2(2 * sxy, sxx - syy);
            return (new Point2d(mx, my), new Point2d(Math.Cos(th), Math.Sin(th)));
        }

        public static double LineResidual(Point2d p, Point2d on, Point2d d) => Math.Abs((p.X - on.X) * d.Y - (p.Y - on.Y) * d.X);

        /// <summary>Algebraic (Kåsa) circle fit; null when degenerate.</summary>
        public static (double Cx, double Cy, double R)? FitCircle(IReadOnlyList<Point2d> pts)
        {
            if (pts.Count < 3) return null;
            double mx = pts.Average(p => p.X), my = pts.Average(p => p.Y);
            double suu = 0, svv = 0, suv = 0, suuu = 0, svvv = 0, suvv = 0, svuu = 0;
            foreach (var p in pts)
            {
                double u = p.X - mx, v = p.Y - my;
                suu += u * u; svv += v * v; suv += u * v;
                suuu += u * u * u; svvv += v * v * v; suvv += u * v * v; svuu += v * u * u;
            }
            double a = suu, b = suv, c2 = svv, e = 0.5 * (suuu + suvv), f = 0.5 * (svvv + svuu);
            double det = a * c2 - b * b;
            if (Math.Abs(det) < 1e-9) return null;
            double uc = (e * c2 - b * f) / det, vc = (a * f - b * e) / det;
            double r = Math.Sqrt(uc * uc + vc * vc + (suu + svv) / pts.Count);
            return (uc + mx, vc + my, r);
        }

        /// <summary>
        /// Measures a presence tool over its region, then over each grid cell, judging each cell
        /// on the tool's limit for its main value. <paramref name="measure"/> gets the masked area
        /// (a mask the size of box) and returns the values.
        /// </summary>
        public static void MeasureWithGrid(ToolContext c, string mainKey, Func<Mat, Rect, Dictionary<string, double>> measure)
        {
            var (mask, box) = RegionMask(c.Region, c.W, c.H);
            using (mask)
            {
                var whole = box.Width > 0 && box.Height > 0 ? measure(mask, box) : new Dictionary<string, double>();
                foreach (var kv in whole) c.Out.Set(kv.Key, kv.Value);
                if (box.Width <= 0 || box.Height <= 0) { c.Fail("region is outside the image"); return; }
            }

            var grid = c.Tool.Grid;
            if (grid == null || grid.Rows * grid.Cols <= 1) return;
            int rows = Math.Clamp(grid.Rows, 1, MaxGridDimension), cols = Math.Clamp(grid.Cols, 1, MaxGridDimension);
            VisionLimit? limit = c.Tool.Limits != null && c.Tool.Limits.TryGetValue(mainKey, out var l) ? l : null;
            bool clip = c.Region.Kind is RegionKind.Circle or RegionKind.Polygon;
            int passed = 0;
            for (int r = 0; r < rows; r++)
                for (int col = 0; col < cols; col++)
                {
                    var quad = c.Region.Cell(rows, cols, r, col, grid.Inset);
                    var (cm, cb) = Geo.QuadMask(quad, c.W, c.H);
                    double value = 0;
                    using (cm)
                    {
                        if (clip && cb.Width > 0 && cb.Height > 0)
                        {
                            using var rm = new Mat(cm.Size(), MatType.CV_8UC1, Scalar.Black);
                            c.Region.Fill(rm, new Point(cb.X, cb.Y));
                            Cv2.BitwiseAnd(cm, rm, cm);
                        }
                        if (cb.Width > 0 && cb.Height > 0 && Cv2.CountNonZero(cm) > 0)
                            value = measure(cm, cb).GetValueOrDefault(mainKey);
                    }
                    bool ok = limit?.Accepts(value) ?? true;
                    if (ok) passed++;
                    c.Out.Cells.Add(new CellResult
                    {
                        Row = r, Col = col, Index = r * cols + col, Value = Math.Round(value, 2), Passed = ok,
                        Quad = Geo.ToPx(quad),
                    });
                }
            c.Out.Set("cellsPassed", passed);
        }
    }
}
