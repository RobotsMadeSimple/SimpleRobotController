using OpenCvSharp;
using OpenCvSharp.Aruco;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision.Tools
{
    using Point = OpenCvSharp.Point;

    // ── findLine / findCorner ───────────────────────────────────────────────────

    /// <summary>
    /// A straight edge found by caliper probes: the region's width is split into
    /// <c>probes</c> bands; each band's pixels are averaged across into a 1-D profile running
    /// from the region's top side to its bottom side, the profile's best edge is taken, and a
    /// line is fitted through those points, dropping the <c>ignore</c> worst ones.
    /// </summary>
    internal static class LineFinder
    {
        public sealed record Fit(Point2d P1, Point2d P2, Point2d Mid, double Angle, double FitError, List<Point2d> Points);

        public static Fit? Find(Mat gray, PixelRegion region, ParamReader p, out string why)
        {
            why = "";
            int probes = Math.Clamp(p.Int("probes", 8), 2, 200);
            string polarity = p.Str("polarity", "any");
            double contrast = p.Num("contrast", 20), smoothing = p.Num("smoothing", 1);
            int ignore = Math.Max(0, p.Int("ignore", 0));
            string pick = p.Str("pick", "strongest");

            var (patch, toImage) = ToolMath.Unroll(gray, region);
            var pts = new List<Point2d>();
            using (patch)
            {
                int w = patch.Width, h = patch.Height;
                double step = w / (double)probes;
                int band = Math.Max(1, (int)Math.Round(step * 0.5));
                for (int i = 0; i < probes; i++)
                {
                    double xc = (i + 0.5) * step;
                    int x0 = Math.Clamp((int)Math.Round(xc - band / 2.0), 0, w - 1);
                    int bw = Math.Clamp(band, 1, w - x0);
                    var profile = ColumnProfile(patch, x0, bw);
                    var edges = ToolMath.Edges(profile, smoothing, contrast, polarity);
                    if (edges.Count == 0) continue;
                    var e = pick switch
                    {
                        "first" => edges[0],
                        "last"  => edges[^1],
                        _       => edges.OrderByDescending(x => Math.Abs(x.Contrast)).First(),
                    };
                    pts.Add(ToolMath.Map(toImage, xc, e.Pos));
                }
            }
            if (pts.Count < 2) { why = $"found {pts.Count} edge point(s), need 2"; return null; }

            var used = new List<Point2d>(pts);
            var (on, d) = ToolMath.FitLine(used);
            for (int k = 0; k < ignore && used.Count > 2; k++)
            {
                var worst = used.OrderByDescending(q => ToolMath.LineResidual(q, on, d)).First();
                used.Remove(worst);
                (on, d) = ToolMath.FitLine(used);
            }

            // Direct the line along the region's width (TL → TR) so its angle is stable.
            var c = region.Corners();
            var u = new Point2d(c[1].X - c[0].X + c[2].X - c[3].X, c[1].Y - c[0].Y + c[2].Y - c[3].Y);
            if (d.X * u.X + d.Y * u.Y < 0) d = new Point2d(-d.X, -d.Y);

            Point2d Foot(Point2d q)
            {
                double t = (q.X - on.X) * d.X + (q.Y - on.Y) * d.Y;
                return new Point2d(on.X + t * d.X, on.Y + t * d.Y);
            }
            var left  = new Point2d((c[0].X + c[3].X) / 2, (c[0].Y + c[3].Y) / 2);
            var right = new Point2d((c[1].X + c[2].X) / 2, (c[1].Y + c[2].Y) / 2);
            var mid   = Foot(region.Center());
            double rms = Math.Sqrt(used.Average(q => Math.Pow(ToolMath.LineResidual(q, on, d), 2)));
            return new Fit(Foot(left), Foot(right), mid, Geo.AngleDeg(d.X, d.Y), rms, used);
        }

        private static double[] ColumnProfile(Mat patch, int x0, int bw)
        {
            using var band = new Mat(patch, new Rect(x0, 0, bw, patch.Height));
            using var col = new Mat();
            Cv2.Reduce(band, col, ReduceDimension.Column, ReduceTypes.Avg, MatType.CV_64F);
            int rows = col.Rows;
            var v = new double[rows];
            for (int y = 0; y < rows; y++) v[y] = col.At<double>(y, 0);
            return v;
        }

        public static void Write(ToolResult o, Fit f)
        {
            o.Set("found", true);
            o.Set("x", f.Mid.X);
            o.Set("y", f.Mid.Y);
            o.Set("angle", f.Angle);
            o.Set("fitError", f.FitError);
            o.Set("pointsUsed", f.Points.Count);
            o.Lines.Add(new PxLine(f.P1.X, f.P1.Y, f.P2.X, f.P2.Y));
            o.Points.AddRange(Geo.ToPx(f.Points));
        }
    }

    internal sealed class FindLineTool : IVisionTool
    {
        public bool MakesPose => true;

        public void Run(ToolContext c)
        {
            var fit = LineFinder.Find(c.Gray, c.Region, c.P, out var why);
            if (fit == null) { c.Out.Set("found", false); c.Fail($"no line: {why}"); return; }
            LineFinder.Write(c.Out, fit);
            c.Out.Pose = new VisionPose { X = Math.Round(fit.Mid.X, 2), Y = Math.Round(fit.Mid.Y, 2), Angle = Math.Round(fit.Angle, 3) };
        }
    }

    internal sealed class FindCornerTool : IVisionTool
    {
        public bool MakesPose => true;

        public void Run(ToolContext c)
        {
            c.Out.Set("found", false);
            var b = c.P.Obj<VisionRegion>("regionB");
            if (b == null) { c.Fail("side B region (params.regionB) is not set"); return; }
            var regionB = c.ExtraRegion(b);

            var fa = LineFinder.Find(c.Gray, c.Region, c.P, out var whyA);
            var fb = LineFinder.Find(c.Gray, regionB, c.P, out var whyB);
            if (fa == null) { c.Fail($"side A: {whyA}"); return; }
            if (fb == null) { c.Fail($"side B: {whyB}"); return; }
            var corner = Geo.Intersect(fa.P1, fa.P2, fb.P1, fb.P2);
            if (corner == null) { c.Fail("the two sides are parallel"); return; }

            double between = Math.Abs(Geo.Wrap180(fb.Angle - fa.Angle));
            c.Out.Set("found", true);
            c.Out.Set("x", corner.Value.X);
            c.Out.Set("y", corner.Value.Y);
            c.Out.Set("angle", fa.Angle);
            c.Out.Set("cornerAngle", between);
            c.Out.Lines.Add(new PxLine(fa.P1.X, fa.P1.Y, fa.P2.X, fa.P2.Y));
            c.Out.Lines.Add(new PxLine(fb.P1.X, fb.P1.Y, fb.P2.X, fb.P2.Y));
            c.Out.Points.Add(new PxPoint(corner.Value.X, corner.Value.Y));
            c.Out.Points.AddRange(Geo.ToPx(fa.Points.Concat(fb.Points)));
            c.Out.Pose = new VisionPose { X = Math.Round(corner.Value.X, 2), Y = Math.Round(corner.Value.Y, 2), Angle = Math.Round(fa.Angle, 3) };
        }
    }

    // ── findCircle ──────────────────────────────────────────────────────────────

    /// <summary>
    /// A circle found by radial caliper probes across an annulus (inner radius = innerRatio ×
    /// radius), each searching inward or outward for its best edge, then a least-squares
    /// circle fit dropping the <c>ignore</c> worst points.
    /// </summary>
    internal sealed class FindCircleTool : IVisionTool
    {
        public bool MakesPose => true;

        public void Run(ToolContext c)
        {
            var p = c.P;
            c.Out.Set("found", false);
            double cx, cy, r;
            if (c.Region.Kind == RegionKind.Circle) { cx = c.Region.Cx; cy = c.Region.Cy; r = c.Region.R; }
            else
            {
                var q = c.Region.Corners();
                var ctr = c.Region.Center();
                cx = ctr.X; cy = ctr.Y;
                r = Math.Min(Geo.Dist(q[0], q[1]), Geo.Dist(q[0], q[3])) / 2;
            }
            double rIn = r * Math.Clamp(p.Num("innerRatio", 0.5), 0, 0.98);
            int probes = Math.Clamp(p.Int("probes", 16), 3, 360);
            bool inward = p.Str("direction", "inward") != "outward";
            string polarity = p.Str("polarity", "any"), pick = p.Str("pick", "strongest");
            double contrast = p.Num("contrast", 20), smoothing = p.Num("smoothing", 1);
            int ignore = Math.Max(0, p.Int("ignore", 0));

            var gray = c.Gray;
            var gi = ToolMath.Bytes(gray);
            int gw = gray.Width, gh = gray.Height;
            int len = Math.Max(3, (int)Math.Round(r - rIn));
            var pts = new List<Point2d>();
            for (int i = 0; i < probes; i++)
            {
                double t = 2 * Math.PI * i / probes, dx = Math.Cos(t), dy = Math.Sin(t);
                var prof = new List<double>(len + 1);
                for (int s = 0; s <= len; s++)
                {
                    double rr = inward ? r - s : rIn + s;
                    double acc = 0; int n = 0;
                    for (int o = -1; o <= 1; o++)
                    {
                        double v = ToolMath.Sample(gi, gw, gh, cx + dx * rr - dy * o, cy + dy * rr + dx * o);
                        if (!double.IsNaN(v)) { acc += v; n++; }
                    }
                    if (n == 0) break;
                    prof.Add(acc / n);
                }
                if (prof.Count < 3) continue;
                var edges = ToolMath.Edges(prof.ToArray(), smoothing, contrast, polarity);
                if (edges.Count == 0) continue;
                var e = pick switch
                {
                    "first" => edges[0],
                    "last"  => edges[^1],
                    _       => edges.OrderByDescending(x => Math.Abs(x.Contrast)).First(),
                };
                double re = inward ? r - e.Pos : rIn + e.Pos;
                pts.Add(new Point2d(cx + dx * re, cy + dy * re));
            }
            if (pts.Count < 3) { c.Fail($"found {pts.Count} edge point(s), need 3"); return; }

            var used = new List<Point2d>(pts);
            var fit = ToolMath.FitCircle(used);
            for (int k = 0; k < ignore && fit != null && used.Count > 3; k++)
            {
                var f = fit.Value;
                used.Remove(used.OrderByDescending(q => Math.Abs(Geo.Dist(q, new Point2d(f.Cx, f.Cy)) - f.R)).First());
                fit = ToolMath.FitCircle(used);
            }
            if (fit == null) { c.Fail("edge points do not fit a circle"); return; }
            var (fx, fy, fr) = fit.Value;
            double rms = Math.Sqrt(used.Average(q => Math.Pow(Geo.Dist(q, new Point2d(fx, fy)) - fr, 2)));
            c.Out.Set("found", true);
            c.Out.Set("x", fx);
            c.Out.Set("y", fy);
            c.Out.Set("radius", fr);
            c.Out.Set("fitError", rms);
            c.Out.Set("pointsUsed", used.Count);
            c.Out.Points.AddRange(Geo.ToPx(used));
            c.Out.Pose = new VisionPose { X = Math.Round(fx, 2), Y = Math.Round(fy, 2), Angle = 0 };
        }
    }

    // ── blob ────────────────────────────────────────────────────────────────────

    /// <summary>Connected dark or light areas in the region (Otsu over the region, or a fixed level).</summary>
    internal sealed class BlobTool : IVisionTool
    {
        public bool MakesPose => true;

        public void Run(ToolContext c)
        {
            var p = c.P;
            double minArea = p.Num("minArea", 20), maxArea = p.Num("maxArea", 1e9), minCirc = p.Num("minCircularity", 0);
            int maxCount = p.Int("maxCount", 100);
            var (bin, box) = ToolMath.Binarize(c);
            var items = new List<(double X, double Y, double Area, double Angle, double Circ)>();
            using (bin)
            {
                if (box.Width > 0 && box.Height > 0)
                {
                    Cv2.FindContours(bin, out var contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxNone);
                    foreach (var ct in contours)
                    {
                        double area = Cv2.ContourArea(ct);
                        if (area < minArea || area > maxArea) continue;
                        double per = Cv2.ArcLength(ct, true);
                        double circ = per > 0 ? Math.Min(1, 4 * Math.PI * area / (per * per)) : 0;
                        if (circ < minCirc) continue;
                        var m = Cv2.Moments(ct);
                        if (m.M00 == 0) continue;
                        items.Add((m.M10 / m.M00 + box.X, m.M01 / m.M00 + box.Y, area, ToolMath.Orientation(m), circ));
                    }
                }
            }
            items = p.Str("sort", "area") switch
            {
                "x" => items.OrderBy(i => i.X).ToList(),
                "y" => items.OrderBy(i => i.Y).ToList(),
                _   => items.OrderByDescending(i => i.Area).ToList(),
            };
            if (maxCount > 0 && items.Count > maxCount) items = items.Take(maxCount).ToList();

            c.Out.Set("count", items.Count);
            c.Out.Set("found", items.Count > 0);
            foreach (var i in items)
                c.Out.Items.Add(new Dictionary<string, object>
                {
                    ["x"] = Math.Round(i.X, 2), ["y"] = Math.Round(i.Y, 2), ["area"] = Math.Round(i.Area, 1),
                    ["angle"] = Math.Round(i.Angle, 2), ["circularity"] = Math.Round(i.Circ, 3),
                });
            if (items.Count > 0)
            {
                c.Out.Set("x", items[0].X);
                c.Out.Set("y", items[0].Y);
                c.Out.Pose = new VisionPose { X = Math.Round(items[0].X, 2), Y = Math.Round(items[0].Y, 2), Angle = Math.Round(items[0].Angle, 2) };
            }
        }
    }

    // ── contourMatch ────────────────────────────────────────────────────────────

    /// <summary>
    /// Shapes in the thresholded region compared with a taught contour (Hu-moment
    /// MatchShapes, so scale and rotation do not matter). score = exp(−10·distance), 1 = same shape.
    /// </summary>
    internal sealed class ContourMatchTool : IVisionTool
    {
        public bool MakesPose => true;

        public sealed class Template
        {
            [System.Text.Json.Serialization.JsonPropertyName("points")] public List<VisionPoint> Points { get; set; } = new();
            [System.Text.Json.Serialization.JsonPropertyName("width")]  public double Width  { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("height")] public double Height { get; set; }
        }

        public static Point[] Contour(Template t)
        {
            double w = Math.Max(1, t.Width), h = Math.Max(1, t.Height);
            double k = Math.Max(1, 400 / Math.Max(w, h));   // enough resolution for the moments
            return t.Points.Select(q => new Point((int)Math.Round(q.X * w * k), (int)Math.Round(q.Y * h * k))).ToArray();
        }

        public void Run(ToolContext c)
        {
            var p = c.P;
            c.Out.Set("count", 0);
            c.Out.Set("score", 0);
            var tpl = p.Obj<Template>("template");
            if (tpl == null || tpl.Points.Count < 3) { c.Fail("no contour taught (Teach the tool on a sample)"); return; }
            var tc = Contour(tpl);
            double tplAngle = ToolMath.Orientation(Cv2.Moments(tc));
            double minScore = p.Num("minScore", 0.7), minArea = p.Num("minArea", 50);
            int maxCount = p.Int("maxCount", 10);

            var found = new List<(double X, double Y, double Angle, double Score)>();
            var (bin, box) = ToolMath.Binarize(c);
            using (bin)
            {
                if (box.Width > 0 && box.Height > 0)
                {
                    Cv2.FindContours(bin, out var contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxNone);
                    foreach (var ct in contours)
                    {
                        if (ct.Length < 5 || Cv2.ContourArea(ct) < minArea) continue;
                        double d = Cv2.MatchShapes(ct, tc, ShapeMatchModes.I1);
                        double score = Math.Exp(-10 * d);
                        if (score < minScore) continue;
                        var m = Cv2.Moments(ct);
                        double ang = ToolMath.Orientation(m) - tplAngle;
                        while (ang > 90) ang -= 180;
                        while (ang <= -90) ang += 180;
                        found.Add((m.M10 / m.M00 + box.X, m.M01 / m.M00 + box.Y, ang, score));
                    }
                }
            }
            found = found.OrderByDescending(f => f.Score).Take(Math.Max(1, maxCount)).ToList();
            MatchOutput.Write(c, found, "no shape matches the taught contour");
        }
    }

    internal static class MatchOutput
    {
        public static void Write(ToolContext c, List<(double X, double Y, double Angle, double Score)> found, string none)
        {
            c.Out.Set("count", found.Count);
            c.Out.Set("score", found.Count > 0 ? found[0].Score : 0);
            foreach (var f in found)
                c.Out.Items.Add(new Dictionary<string, object>
                {
                    ["x"] = Math.Round(f.X, 2), ["y"] = Math.Round(f.Y, 2), ["angle"] = Math.Round(f.Angle, 2), ["score"] = Math.Round(f.Score, 3),
                });
            if (found.Count == 0) { c.Fail(none); return; }
            c.Out.Set("x", found[0].X);
            c.Out.Set("y", found[0].Y);
            c.Out.Set("angle", found[0].Angle);
            c.Out.Pose = new VisionPose { X = Math.Round(found[0].X, 2), Y = Math.Round(found[0].Y, 2), Angle = Math.Round(found[0].Angle, 2) };
        }
    }

    // ── patternMatch ────────────────────────────────────────────────────────────

    /// <summary>
    /// A taught gray patch searched for over the region's bounding box with normalized
    /// cross-correlation (TM_CCOEFF_NORMED). For a rotated search the image is turned instead
    /// of the template, once per angle in −angleRange…+angleRange (angleStep), so no corner of
    /// the template is ever blank. Reported x/y is where the template's origin lands.
    /// </summary>
    internal sealed class PatternMatchTool : IVisionTool
    {
        public bool MakesPose => true;

        public sealed class Template
        {
            [System.Text.Json.Serialization.JsonPropertyName("png")]     public string Png     { get; set; } = "";
            [System.Text.Json.Serialization.JsonPropertyName("originX")] public double OriginX { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("originY")] public double OriginY { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("width")]   public int    Width   { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("height")]  public int    Height  { get; set; }
        }

        public void Run(ToolContext c)
        {
            var p = c.P;
            c.Out.Set("count", 0);
            c.Out.Set("score", 0);
            var t = p.Obj<Template>("template");
            var tpl = t == null || string.IsNullOrEmpty(t.Png) ? null : c.Runtime.Template(c.Tool.Id, t.Png);
            if (t == null || tpl == null) { c.Fail("no pattern taught (Teach the tool on a sample)"); return; }

            double minScore = p.Num("minScore", 0.7), range = Math.Abs(p.Num("angleRange", 0)), stepA = Math.Max(0.5, Math.Abs(p.Num("angleStep", 5)));
            int maxCount = Math.Max(1, p.Int("maxCount", 1));
            var box = c.Region.Bounds(c.W, c.H);
            if (box.Width < tpl.Width || box.Height < tpl.Height) { c.Fail("search region is smaller than the pattern"); return; }

            using var crop = new Mat(c.Gray, box);
            var cands = new List<(double X, double Y, double Angle, double Score)>();
            var angles = new List<double> { 0 };
            for (double a = stepA; a <= range + 1e-9; a += stepA) { angles.Add(a); angles.Add(-a); }
            foreach (var a in angles)
                Search(crop, tpl, t, a, minScore, maxCount, box, cands);

            // Best first; one result per object (the same object found at neighbouring angles).
            double sep = Math.Min(tpl.Width, tpl.Height) * 0.5;
            var kept = new List<(double X, double Y, double Angle, double Score)>();
            foreach (var cd in cands.OrderByDescending(x => x.Score))
            {
                if (!c.Region.Contains(cd.X, cd.Y)) continue;
                if (kept.Any(k => Math.Abs(k.X - cd.X) < sep && Math.Abs(k.Y - cd.Y) < sep)) continue;
                kept.Add(cd);
                if (kept.Count >= maxCount) break;
            }
            MatchOutput.Write(c, kept, "pattern not found");
        }

        private static void Search(Mat crop, Mat tpl, Template t, double angle, double minScore, int maxCount, Rect box,
                                   List<(double, double, double, double)> into)
        {
            Mat img = crop;
            double[,]? back = null;
            Mat? rotated = null;
            try
            {
                if (Math.Abs(angle) > 1e-9)
                {
                    // Turn the image by −angle (an object at +angle comes upright) on a canvas big
                    // enough to keep all of it; GetRotationMatrix2D's positive angle is
                    // counter-clockwise on screen, i.e. −angle in this file's convention.
                    int diag = (int)Math.Ceiling(Math.Sqrt(crop.Width * crop.Width + crop.Height * crop.Height));
                    using var m = Cv2.GetRotationMatrix2D(new Point2f(crop.Width / 2f, crop.Height / 2f), angle, 1);
                    m.Set(0, 2, m.At<double>(0, 2) + (diag - crop.Width) / 2.0);
                    m.Set(1, 2, m.At<double>(1, 2) + (diag - crop.Height) / 2.0);
                    rotated = new Mat();
                    Cv2.WarpAffine(crop, rotated, m, new Size(diag, diag), InterpolationFlags.Linear, BorderTypes.Constant, Scalar.Black);
                    using var inv = new Mat();
                    Cv2.InvertAffineTransform(m, inv);
                    back = new double[2, 3];
                    for (int r = 0; r < 2; r++) for (int col = 0; col < 3; col++) back[r, col] = inv.At<double>(r, col);
                    img = rotated;
                }
                if (img.Width < tpl.Width || img.Height < tpl.Height) return;
                using var res = new Mat();
                Cv2.MatchTemplate(img, tpl, res, TemplateMatchModes.CCoeffNormed);
                for (int n = 0; n < maxCount * 3; n++)
                {
                    Cv2.MinMaxLoc(res, out _, out double max, out _, out Point loc);
                    if (max < minScore || double.IsNaN(max)) break;
                    // Sub-pixel peak.
                    double ox = loc.X + SubPixel(res, loc, true), oy = loc.Y + SubPixel(res, loc, false);
                    double px = ox + t.OriginX, py = oy + t.OriginY;
                    if (back != null)
                    {
                        double bx = back[0, 0] * px + back[0, 1] * py + back[0, 2];
                        double by = back[1, 0] * px + back[1, 1] * py + back[1, 2];
                        px = bx; py = by;
                    }
                    into.Add((px + box.X, py + box.Y, angle, max));
                    int hw = Math.Max(1, tpl.Width / 2), hh = Math.Max(1, tpl.Height / 2);
                    Cv2.Rectangle(res, new Rect(loc.X - hw, loc.Y - hh, 2 * hw + 1, 2 * hh + 1), Scalar.All(-1), -1);
                }
            }
            finally { rotated?.Dispose(); }
        }

        private static double SubPixel(Mat res, Point loc, bool alongX)
        {
            int x = loc.X, y = loc.Y;
            if (alongX ? (x <= 0 || x >= res.Width - 1) : (y <= 0 || y >= res.Height - 1)) return 0;
            float l = alongX ? res.At<float>(y, x - 1) : res.At<float>(y - 1, x);
            float m = res.At<float>(y, x);
            float r = alongX ? res.At<float>(y, x + 1) : res.At<float>(y + 1, x);
            double den = l - 2 * m + r;
            return Math.Abs(den) > 1e-9 ? Math.Clamp(0.5 * (l - r) / den, -0.5, 0.5) : 0;
        }

        /// <summary>Cuts the region (unrolled upright) from a gray image as the template.</summary>
        public static Template Teach(Mat gray, PixelRegion region)
        {
            var (patch, _) = ToolMath.Unroll(gray, region);
            using (patch)
            {
                Cv2.ImEncode(".png", patch, out var png);
                return new Template
                {
                    Png = Convert.ToBase64String(png), Width = patch.Width, Height = patch.Height,
                    OriginX = patch.Width / 2.0, OriginY = patch.Height / 2.0,
                };
            }
        }
    }

    // ── aruco ───────────────────────────────────────────────────────────────────

    /// <summary>ArUco markers whose centre lies in the region.</summary>
    internal sealed class ArucoTool : IVisionTool
    {
        public bool MakesPose => true;

        public void Run(ToolContext c)
        {
            var p = c.P;
            int dictId = p.Int("dictionaryId", 1), markerId = p.Int("markerId", -1);
            double minArea = p.Num("minArea", 100), maxArea = p.Num("maxArea", 1e9);
            IEnumerable<PredefinedDictionaryType> types = dictId < 0 ? VisionRuntime.AllDictionaryTypes : new[] { (PredefinedDictionaryType)dictId };
            var seen = new HashSet<(int, int)>();
            var items = new List<(int Id, double X, double Y, double Angle)>();
            foreach (var type in types)
            {
                c.Runtime.Detector(type).DetectMarkers(c.Gray, out var corners, out var ids, out _);
                if (ids == null) continue;
                for (int i = 0; i < ids.Length; i++)
                {
                    if (markerId >= 0 && ids[i] != markerId) continue;
                    var q = corners[i];
                    double area = Math.Abs(Cv2.ContourArea(q));
                    if (area < minArea || area > maxArea) continue;
                    double cx = q.Average(v => v.X), cy = q.Average(v => v.Y);
                    if (!c.Region.Contains(cx, cy) || !seen.Add(((int)cx, (int)cy))) continue;
                    items.Add((ids[i], cx, cy, Geo.AngleDeg(q[1].X - q[0].X, q[1].Y - q[0].Y)));
                    c.Out.Lines.AddRange(Enumerable.Range(0, 4).Select(k => new PxLine(q[k].X, q[k].Y, q[(k + 1) % 4].X, q[(k + 1) % 4].Y)));
                }
            }
            c.Out.Set("count", items.Count);
            foreach (var i in items)
                c.Out.Items.Add(new Dictionary<string, object> { ["id"] = i.Id, ["x"] = Math.Round(i.X, 2), ["y"] = Math.Round(i.Y, 2), ["angle"] = Math.Round(i.Angle, 2) });
            if (items.Count == 0) { c.Fail("no marker found"); return; }
            c.Out.Set("id", items[0].Id);
            c.Out.Set("x", items[0].X);
            c.Out.Set("y", items[0].Y);
            c.Out.Set("angle", items[0].Angle);
            c.Out.Pose = new VisionPose { X = Math.Round(items[0].X, 2), Y = Math.Round(items[0].Y, 2), Angle = Math.Round(items[0].Angle, 2) };
        }
    }

    // ── makeFixture ─────────────────────────────────────────────────────────────

    /// <summary>
    /// A pose built from other tools: pointAngle, twoPoints, lineIntersection, fourLines
    /// (top, right, bottom, left → corners TL, TR, BR, BL) or fourPoints (TL, TR, BR, BL).
    /// The four-* modes carry a quad, so dependents follow the perspective.
    /// </summary>
    internal sealed class MakeFixtureTool : IVisionTool
    {
        public bool UsesRegion => false;
        public bool MakesPose => true;
        public IReadOnlyList<string> Sources(VisionTool tool) => tool.P.StrList("sources");

        public void Run(ToolContext c)
        {
            c.Out.Set("found", false);
            var ids = c.P.StrList("sources");
            var mode = c.P.Str("mode", "pointAngle");
            int need = mode switch { "fourLines" or "fourPoints" => 4, _ => 2 };
            if (ids.Count < need) { c.Fail($"mode {mode} needs {need} sources"); return; }
            var src = ids.Take(need).Select(id => c.Source(id)).ToList();
            if (src.Any(s => s == null)) { c.Fail("a source tool has no result"); return; }

            Point2d origin; double angle; List<Point2d>? quad = null;
            switch (mode)
            {
                case "twoPoints":
                {
                    if (Geometry.PointOf(src[0]!) is not { } a || Geometry.PointOf(src[1]!) is not { } b) { c.Fail("a source has no point"); return; }
                    origin = a; angle = Geo.AngleDeg(b.X - a.X, b.Y - a.Y);
                    break;
                }
                case "lineIntersection":
                {
                    if (Geometry.LineOf(src[0]!) is not { } la || Geometry.LineOf(src[1]!) is not { } lb) { c.Fail("a source has no line"); return; }
                    if (Geo.Intersect(la, lb) is not { } x) { c.Fail("the lines are parallel"); return; }
                    origin = x; angle = Geo.AngleDeg(la.X2 - la.X1, la.Y2 - la.Y1);
                    break;
                }
                case "fourLines":
                {
                    var lines = src.Select(s => Geometry.LineOf(s!)).ToList();
                    if (lines.Any(l => l == null)) { c.Fail("a source has no line"); return; }
                    var (top, right, bottom, left) = (lines[0]!, lines[1]!, lines[2]!, lines[3]!);
                    var corners = new[] { Geo.Intersect(top, left), Geo.Intersect(top, right), Geo.Intersect(bottom, right), Geo.Intersect(bottom, left) };
                    if (corners.Any(k => k == null)) { c.Fail("two of the lines are parallel"); return; }
                    quad = corners.Select(k => k!.Value).ToList();
                    origin = new Point2d(quad.Average(q => q.X), quad.Average(q => q.Y));
                    angle = Geo.AngleDeg(quad[1].X - quad[0].X, quad[1].Y - quad[0].Y);
                    break;
                }
                case "fourPoints":
                {
                    var pts = src.Select(s => Geometry.PointOf(s!)).ToList();
                    if (pts.Any(k => k == null)) { c.Fail("a source has no point"); return; }
                    quad = pts.Select(k => k!.Value).ToList();
                    origin = new Point2d(quad.Average(q => q.X), quad.Average(q => q.Y));
                    angle = Geo.AngleDeg(quad[1].X - quad[0].X, quad[1].Y - quad[0].Y);
                    break;
                }
                default:   // pointAngle
                {
                    if (Geometry.PointOf(src[0]!) is not { } a) { c.Fail("source 1 has no point"); return; }
                    if (src[1]!.Pose == null) { c.Fail("source 2 has no pose (angle)"); return; }
                    origin = a; angle = src[1]!.Pose!.Angle;
                    break;
                }
            }
            if (quad != null && !Geo.Convex(quad.Select(q => new Point2f((float)q.X, (float)q.Y)).ToArray()))
            {
                c.Fail("the four corners do not form a convex quad");
                return;
            }
            c.Out.Set("found", true);
            c.Out.Set("x", origin.X);
            c.Out.Set("y", origin.Y);
            c.Out.Set("angle", angle);
            c.Out.Pose = new VisionPose { X = Math.Round(origin.X, 2), Y = Math.Round(origin.Y, 2), Angle = Math.Round(angle, 3), Quad = quad == null ? null : Geo.ToPx(quad) };
            if (quad != null)
            {
                c.Out.Points.AddRange(Geo.ToPx(quad));
                for (int i = 0; i < 4; i++) c.Out.Lines.Add(new PxLine(quad[i].X, quad[i].Y, quad[(i + 1) % 4].X, quad[(i + 1) % 4].Y));
            }
            else c.Out.Points.Add(new PxPoint(origin.X, origin.Y));
        }
    }

    /// <summary>Reading other tools' results as geometry sources.</summary>
    internal static class Geometry
    {
        /// <summary>A tool's point: its pose, else its first point, else values x/y.</summary>
        public static Point2d? PointOf(ToolResult r)
        {
            if (r.Pose != null) return new Point2d(r.Pose.X, r.Pose.Y);
            if (r.Points.Count > 0) return new Point2d(r.Points[0].X, r.Points[0].Y);
            if (r.TryGetNumber("x", out var x) && r.TryGetNumber("y", out var y)) return new Point2d(x, y);
            return null;
        }

        /// <summary>A tool's line: its first line.</summary>
        public static PxLine? LineOf(ToolResult r) => r.Lines.Count > 0 ? r.Lines[0] : null;
    }
}
