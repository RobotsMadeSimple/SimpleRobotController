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

            var found = new List<(double X, double Y, double Angle, double Score, Point[] Outline)>();
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
                        found.Add((m.M10 / m.M00 + box.X, m.M01 / m.M00 + box.Y, ang, score, Cv2.ApproxPolyDP(ct, 1.0, true)));
                    }
                }
            }
            found = found.OrderByDescending(f => f.Score).Take(Math.Max(1, maxCount)).ToList();
            MatchOutput.Write(c, found.Select(f => (f.X, f.Y, f.Angle, f.Score)).ToList(), "no shape matches the taught contour");
            // Each matched outline where it was found, for the overlay.
            foreach (var f in found.Take(20))
                c.Out.Contours.Add(f.Outline.Select(q => new PxPoint(q.X + box.X, q.Y + box.Y)).ToList());
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
    /// cross-correlation (TM_CCOEFF_NORMED). The image is turned instead of the template, once
    /// per angle in −angleRange…+angleRange (angleStep) about the taught angle, so no corner of
    /// the template is ever blank (angleRange 180 = every angle). A turned search runs coarse on
    /// a shrunk image, then each candidate is refined at full size in position and angle, to a
    /// fraction of a degree. Reported x/y is where the template's origin (the centre of the
    /// taught area) lands; angle is relative to the taught orientation (0 = as taught). Each
    /// match's taught outlines are placed in <c>contours</c> for the overlay.
    /// </summary>
    internal sealed class PatternMatchTool : IVisionTool
    {
        public bool MakesPose => true;

        /// <summary>Matches whose outlines are drawn (the overlay is capped; items are not).</summary>
        private const int MaxDrawn = 20;
        /// <summary>The coarse search shrinks the image while the template's short side stays at least this long (px).</summary>
        private const int CoarseMinSide = 24;

        public sealed class Template
        {
            [System.Text.Json.Serialization.JsonPropertyName("png")]     public string Png     { get; set; } = "";
            [System.Text.Json.Serialization.JsonPropertyName("originX")] public double OriginX { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("originY")] public double OriginY { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("width")]   public int    Width   { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("height")]  public int    Height  { get; set; }
            /// <summary>The taught area's angle in the teach image (deg, clockwise on screen); an output angle of 0 means this orientation.</summary>
            [System.Text.Json.Serialization.JsonPropertyName("angle")]   public double Angle   { get; set; }
            /// <summary>Closed outlines found in the patch, in its pixels (x right, y down from its top-left).</summary>
            [System.Text.Json.Serialization.JsonPropertyName("contours")] public List<List<VisionPoint>>? Contours { get; set; }
            /// <summary>The area it was taught from, normalized to the teach frame; null when taught from the tool's region.</summary>
            [System.Text.Json.Serialization.JsonPropertyName("region")]  public VisionRegion? Region { get; set; }
        }

        /// <summary>A match: the template origin's position (crop pixels), its absolute angle in the image, and its score.</summary>
        private readonly record struct Hit(double X, double Y, double Angle, double Score);

        public void Run(ToolContext c)
        {
            var p = c.P;
            c.Out.Set("count", 0);
            c.Out.Set("score", 0);
            var t = p.Obj<Template>("template");
            var tpl = t == null || string.IsNullOrEmpty(t.Png) ? null : c.Runtime.Template(c.Tool.Id, t.Png);
            if (t == null || tpl == null) { c.Fail("no pattern taught (Teach the tool on a sample)"); return; }

            double minScore = p.Num("minScore", 0.7);
            double range = Math.Min(180, Math.Abs(p.Num("angleRange", 0)));
            double stepA = Math.Clamp(Math.Abs(p.Num("angleStep", 5)), 0.5, 90);
            int maxCount = Math.Max(1, p.Int("maxCount", 1));
            var box = c.Region.Bounds(c.W, c.H);
            if (box.Width < tpl.Width || box.Height < tpl.Height) { c.Fail("search region is smaller than the pattern"); return; }

            using var crop = new Mat(c.Gray, box);
            double sep = Math.Min(tpl.Width, tpl.Height) * 0.5;
            var origin = new Point2d(t.OriginX, t.OriginY);
            List<Hit> hits;
            if (range < 1e-9)
                hits = Peaks(crop, tpl, origin, t.Angle, 1, minScore, maxCount * 3);
            else
            {
                // Coarse: every search angle on a shrunk copy, with a looser score (shrinking and
                // the angle step both cost score); then each distinct candidate refined at full size.
                double k = CoarseScale(tpl);
                Mat sCrop = crop, sTpl = tpl;
                try
                {
                    if (k < 1)
                    {
                        sCrop = new Mat(); sTpl = new Mat();
                        Cv2.Resize(crop, sCrop, new Size(), k, k, InterpolationFlags.Area);
                        Cv2.Resize(tpl, sTpl, new Size(), k, k, InterpolationFlags.Area);
                    }
                    double loose = minScore * (k < 1 ? 0.7 : 0.85);
                    var sOrigin = new Point2d((origin.X + 0.5) * k - 0.5, (origin.Y + 0.5) * k - 0.5);
                    var cands = new List<Hit>();
                    foreach (var off in Offsets(range, stepA))
                        cands.AddRange(Peaks(sCrop, sTpl, sOrigin, t.Angle + off, k, loose, maxCount * 2));
                    int margin = k < 1 ? (int)Math.Ceiling(1.5 / k) + 2 : 3;
                    hits = Distinct(cands, sep, maxCount * 3 + 2)
                        .Select(h => Refine(crop, tpl, origin, h, stepA, margin))
                        .Where(h => h.Score >= minScore)
                        .ToList();
                }
                finally
                {
                    if (!ReferenceEquals(sCrop, crop)) sCrop.Dispose();
                    if (!ReferenceEquals(sTpl, tpl)) sTpl.Dispose();
                }
            }

            // Best first; one result per object (the same object found at neighbouring angles).
            var kept = Distinct(hits.Where(h => c.Region.Contains(h.X + box.X, h.Y + box.Y)), sep, maxCount)
                .Select(h => h with { X = h.X + box.X, Y = h.Y + box.Y })
                .ToList();
            MatchOutput.Write(c, kept.Select(h => (h.X, h.Y, Geo.Wrap180(h.Angle - t.Angle), h.Score)).ToList(), "pattern not found");
            foreach (var h in kept.Take(MaxDrawn))
                c.Out.Contours.AddRange(Placed(t, h.X, h.Y, h.Angle));
        }

        /// <summary>The template's outlines moved so its origin sits on (x, y) and turned to <paramref name="angle"/> (absolute, deg).</summary>
        public static IEnumerable<List<PxPoint>> Placed(Template t, double x, double y, double angle)
        {
            if (t.Contours == null) yield break;
            double a = angle * Math.PI / 180, cos = Math.Cos(a), sin = Math.Sin(a);
            foreach (var ct in t.Contours)
                if (ct.Count >= 2)
                    yield return ct.Select(q =>
                    {
                        double dx = q.X - t.OriginX, dy = q.Y - t.OriginY;
                        return new PxPoint(x + dx * cos - dy * sin, y + dx * sin + dy * cos);
                    }).ToList();
        }

        /// <summary>Search angles about the taught one: 0, ±step, ±2·step… up to ±range; a range of 180 covers the full turn evenly.</summary>
        private static List<double> Offsets(double range, double step)
        {
            var list = new List<double>();
            if (range >= 180 - 1e-9)
            {
                int n = Math.Max(1, (int)Math.Round(360 / step));
                for (int i = 0; i < n; i++) list.Add(Geo.Wrap180(i * 360.0 / n));
                return list;
            }
            list.Add(0);
            for (double a = step; a <= range + 1e-9; a += step) { list.Add(a); list.Add(-a); }
            return list;
        }

        /// <summary>How far the coarse search shrinks the image: halved while the template's short side stays ≥ CoarseMinSide, at most to a quarter.</summary>
        private static double CoarseScale(Mat tpl)
        {
            double k = 1, side = Math.Min(tpl.Width, tpl.Height);
            while (k > 0.25 && side * k * 0.5 >= CoarseMinSide) k *= 0.5;
            return k;
        }

        /// <summary>Best-first, dropping any hit within <paramref name="sep"/> of a better one; at most <paramref name="max"/>.</summary>
        private static List<Hit> Distinct(IEnumerable<Hit> hits, double sep, int max)
        {
            var kept = new List<Hit>();
            foreach (var h in hits.OrderByDescending(x => x.Score))
            {
                if (kept.Any(k => Math.Abs(k.X - h.X) < sep && Math.Abs(k.Y - h.Y) < sep)) continue;
                kept.Add(h);
                if (kept.Count >= max) break;
            }
            return kept;
        }

        /// <summary>
        /// The 2×3 map from a window's pixels to <paramref name="img"/>'s: window point
        /// <paramref name="at"/> lands on image point <paramref name="on"/>, turned by
        /// <paramref name="angle"/> (deg, clockwise on screen) — so an object turned by +angle
        /// in the image stands upright in the window.
        /// </summary>
        private static double[] WindowMap(Point2d on, Point2d at, double angle)
        {
            double a = angle * Math.PI / 180, cos = Math.Cos(a), sin = Math.Sin(a);
            return new[] { cos, -sin, on.X - (cos * at.X - sin * at.Y), sin, cos, on.Y - (sin * at.X + cos * at.Y) };
        }

        private static Point2d Apply(double[] m, double x, double y) => new(m[0] * x + m[1] * y + m[2], m[3] * x + m[4] * y + m[5]);

        private static Mat Warp(Mat img, double[] m, int w, int h, BorderTypes border)
        {
            using var mat = new Mat(2, 3, MatType.CV_64FC1);
            for (int i = 0; i < 6; i++) mat.Set(i / 3, i % 3, m[i]);
            var win = new Mat();
            Cv2.WarpAffine(img, win, mat, new Size(w, h), InterpolationFlags.Linear | InterpolationFlags.WarpInverseMap, border, Scalar.Black);
            return win;
        }

        /// <summary>
        /// Up to <paramref name="n"/> peaks ≥ <paramref name="minScore"/> over the whole of
        /// <paramref name="img"/> turned by <paramref name="angle"/>; positions are scaled back by
        /// 1/<paramref name="k"/> to full-size crop pixels.
        /// </summary>
        private static List<Hit> Peaks(Mat img, Mat tpl, Point2d origin, double angle, double k, double minScore, int n)
        {
            var hits = new List<Hit>();
            Mat? turned = null;
            double[] m;
            if (Math.Abs(angle) < 1e-9) m = new double[] { 1, 0, 0, 0, 1, 0 };
            else
            {
                // A canvas big enough to keep all of the image at any angle, centred on its centre.
                int diag = (int)Math.Ceiling(Math.Sqrt(img.Width * img.Width + img.Height * img.Height));
                m = WindowMap(new Point2d(img.Width / 2.0, img.Height / 2.0), new Point2d(diag / 2.0, diag / 2.0), angle);
                turned = Warp(img, m, diag, diag, BorderTypes.Constant);
            }
            try
            {
                var src = turned ?? img;
                if (src.Width < tpl.Width || src.Height < tpl.Height) return hits;
                using var res = new Mat();
                Cv2.MatchTemplate(src, tpl, res, TemplateMatchModes.CCoeffNormed);
                for (int i = 0; i < n; i++)
                {
                    Cv2.MinMaxLoc(res, out _, out double max, out _, out Point loc);
                    if (max < minScore || double.IsNaN(max)) break;
                    var q = Apply(m, loc.X + SubPixel(res, loc, true) + origin.X, loc.Y + SubPixel(res, loc, false) + origin.Y);
                    hits.Add(new Hit((q.X + 0.5) / k - 0.5, (q.Y + 0.5) / k - 0.5, angle, max));
                    int hw = Math.Max(1, tpl.Width / 2), hh = Math.Max(1, tpl.Height / 2);
                    Cv2.Rectangle(res, new Rect(loc.X - hw, loc.Y - hh, 2 * hw + 1, 2 * hh + 1), Scalar.All(-1), -1);
                }
                return hits;
            }
            finally { turned?.Dispose(); }
        }

        /// <summary>
        /// The best match in a small window round (x, y) at one angle: the window is the template
        /// plus <paramref name="margin"/> px on each side, turned about the guessed origin.
        /// </summary>
        private static Hit Local(Mat crop, Mat tpl, Point2d origin, double angle, double x, double y, int margin)
        {
            int w = tpl.Width + 2 * margin, h = tpl.Height + 2 * margin;
            var m = WindowMap(new Point2d(x, y), new Point2d(margin + origin.X, margin + origin.Y), angle);
            using var win = Warp(crop, m, w, h, BorderTypes.Replicate);
            using var res = new Mat();
            Cv2.MatchTemplate(win, tpl, res, TemplateMatchModes.CCoeffNormed);
            Cv2.MinMaxLoc(res, out _, out double max, out _, out Point loc);
            if (double.IsNaN(max)) return new Hit(x, y, angle, 0);
            var q = Apply(m, loc.X + SubPixel(res, loc, true) + origin.X, loc.Y + SubPixel(res, loc, false) + origin.Y);
            return new Hit(q.X, q.Y, angle, max);
        }

        /// <summary>A coarse hit refined at full size: position first, then the angle by halving steps, then a parabola through the last three.</summary>
        private static Hit Refine(Mat crop, Mat tpl, Point2d origin, Hit h, double step, int margin)
        {
            var best = Local(crop, tpl, origin, h.Angle, h.X, h.Y, margin);
            best = Local(crop, tpl, origin, best.Angle, best.X, best.Y, 2);
            double d = step / 2;
            for (; d >= 0.25; d /= 2)
            {
                var lo = Local(crop, tpl, origin, best.Angle - d, best.X, best.Y, 2);
                var hi = Local(crop, tpl, origin, best.Angle + d, best.X, best.Y, 2);
                if (lo.Score > best.Score && lo.Score >= hi.Score) best = lo;
                else if (hi.Score > best.Score) best = hi;
            }
            d *= 2;
            var l = Local(crop, tpl, origin, best.Angle - d, best.X, best.Y, 2);
            var r = Local(crop, tpl, origin, best.Angle + d, best.X, best.Y, 2);
            double den = l.Score - 2 * best.Score + r.Score;
            if (den < -1e-9)
            {
                double off = Math.Clamp(0.5 * (l.Score - r.Score) / den, -0.5, 0.5) * d;
                var fine = Local(crop, tpl, origin, best.Angle + off, best.X, best.Y, 2);
                if (fine.Score >= best.Score - 1e-3) best = fine;
            }
            return best;
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

        /// <summary>
        /// Cuts the region (unrolled upright, so a turned rectangle is deskewed to its own axes)
        /// from a gray image as the template. Its angle — the direction of the region's top edge,
        /// TL → TR — becomes the reference orientation; the patch's outlines are kept for drawing.
        /// </summary>
        public static Template Teach(Mat gray, PixelRegion region, VisionRegion? drawn = null)
        {
            var (patch, _) = ToolMath.Unroll(gray, region);
            using (patch)
            {
                var q = region.Corners();
                double angle = region.Kind is RegionKind.Rect or RegionKind.Quad ? Geo.AngleDeg(q[1].X - q[0].X, q[1].Y - q[0].Y) : 0;
                Cv2.ImEncode(".png", patch, out var png);
                return new Template
                {
                    Png = Convert.ToBase64String(png), Width = patch.Width, Height = patch.Height,
                    OriginX = patch.Width / 2.0, OriginY = patch.Height / 2.0,
                    Angle = Math.Round(angle, 3), Contours = Outlines(patch), Region = drawn,
                };
            }
        }

        /// <summary>
        /// The patch's outlines: Otsu-thresholded (the smaller side taken as the part), every
        /// contour bigger than a speck that does not just trace the patch's edge, simplified until
        /// there are at most ~400 points in all. A flat patch has none.
        /// </summary>
        public static List<List<VisionPoint>> Outlines(Mat patch)
        {
            var list = new List<List<VisionPoint>>();
            using var blur = new Mat();
            Cv2.GaussianBlur(patch, blur, new Size(3, 3), 0);
            Cv2.MeanStdDev(blur, out _, out var sd);
            if (sd.Val0 < 4) return list;
            using var bin = new Mat();
            Cv2.Threshold(blur, bin, 0, 255, ThresholdTypes.Binary | ThresholdTypes.Otsu);
            if (Cv2.CountNonZero(bin) * 2 > bin.Rows * bin.Cols) Cv2.BitwiseNot(bin, bin);
            Cv2.FindContours(bin, out var contours, out _, RetrievalModes.List, ContourApproximationModes.ApproxNone);
            double minArea = Math.Max(9, patch.Rows * patch.Cols * 0.002);
            int w = patch.Cols, h = patch.Rows;
            // A contour running mostly along the patch's own edge is the cut, not the part.
            bool AlongEdge(Point[] ct) => ct.Count(q => q.X <= 1 || q.Y <= 1 || q.X >= w - 2 || q.Y >= h - 2) * 2 > ct.Length;
            var keep = contours.Where(ct => Cv2.ContourArea(ct) >= minArea && !AlongEdge(ct))
                .OrderByDescending(ct => Cv2.ContourArea(ct)).Take(24).ToList();
            double eps = 0.8;
            List<Point[]> simple;
            do
            {
                simple = keep.Select(ct => Cv2.ApproxPolyDP(ct, eps, true)).Where(ct => ct.Length >= 3).ToList();
                eps *= 1.6;
            } while (simple.Sum(ct => ct.Length) > 400 && eps < 50);
            foreach (var ct in simple) list.Add(ct.Select(pt => new VisionPoint(pt.X, pt.Y)).ToList());
            return list;
        }

        /// <summary>The taught patch enlarged, its outlines drawn on it, and its origin and 0° direction marked — a PNG for the teach preview.</summary>
        public static byte[] PreviewPng(Template t, Mat patch)
        {
            int s = Math.Max(1, (int)Math.Ceiling(320.0 / Math.Max(1, Math.Max(patch.Width, patch.Height))));
            s = Math.Min(s, 8);
            using var big = new Mat();
            Cv2.Resize(patch, big, new Size(patch.Width * s, patch.Height * s), 0, 0, InterpolationFlags.Linear);
            using var color = new Mat();
            Cv2.CvtColor(big, color, ColorConversionCodes.GRAY2BGR);
            Point P(double x, double y) => new((int)Math.Round((x + 0.5) * s - 0.5), (int)Math.Round((y + 0.5) * s - 0.5));
            foreach (var ct in t.Contours ?? new())
                if (ct.Count >= 2) Cv2.Polylines(color, new[] { ct.Select(q => P(q.X, q.Y)).ToArray() }, true, VisionPalette.DebugMatched, 2, LineTypes.AntiAlias);
            var o = P(t.OriginX, t.OriginY);
            int len = Math.Max(12, Math.Min(color.Width, color.Height) / 5);
            Cv2.ArrowedLine(color, o, new Point(o.X + len, o.Y), VisionPalette.Fail, 2, LineTypes.AntiAlias, 0, 0.25);
            Cv2.Line(color, o, new Point(o.X, o.Y + len), VisionPalette.Pass, 2, LineTypes.AntiAlias);
            Cv2.Circle(color, o, 3, Scalar.White, -1, LineTypes.AntiAlias);
            Cv2.ImEncode(".png", color, out var png);
            return png;
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
