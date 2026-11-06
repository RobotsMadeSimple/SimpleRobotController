using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using ZXing;

namespace Controller.RobotControl.Vision.Tools
{
    using Point = OpenCvSharp.Point;

    // ── pixelCount ──────────────────────────────────────────────────────────────

    /// <summary>
    /// Percentage of the region (or of each cell) whose pixels match: mode color = within
    /// tolerance (0–100 → ±2.55·tolerance per channel) of any of <c>colors</c> [{r,g,b,tolerance}];
    /// mode gray = gray level within grayMin…grayMax.
    /// </summary>
    internal sealed class PixelCountTool : IVisionTool
    {
        public string? MainValue(VisionTool tool) => "percent";

        private sealed class ColorParam
        {
            [System.Text.Json.Serialization.JsonPropertyName("r")]         public int R { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("g")]         public int G { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("b")]         public int B { get; set; }
            [System.Text.Json.Serialization.JsonPropertyName("tolerance")] public double Tolerance { get; set; } = 20;
        }

        public void Run(ToolContext c)
        {
            var p = c.P;
            using var match = new Mat(c.H, c.W, MatType.CV_8UC1, Scalar.Black);
            if (p.Str("mode", "color") == "gray")
                Cv2.InRange(c.Gray, new Scalar(p.Num("grayMin", 0)), new Scalar(p.Num("grayMax", 127)), match);
            else
            {
                var input = c.Input;
                foreach (var col in p.Obj<List<ColorParam>>("colors") ?? new())
                {
                    int d = (int)Math.Round(col.Tolerance * 2.55);
                    using var one = new Mat();
                    if (input.Channels() >= 3)
                        Cv2.InRange(input, new Scalar(col.B - d, col.G - d, col.R - d), new Scalar(col.B + d, col.G + d, col.R + d), one);
                    else
                    {
                        double lum = 0.299 * col.R + 0.587 * col.G + 0.114 * col.B;
                        Cv2.InRange(c.Gray, new Scalar(lum - d), new Scalar(lum + d), one);
                    }
                    Cv2.BitwiseOr(match, one, match);
                }
            }
            ToolMath.MeasureWithGrid(c, "percent", (mask, box) =>
            {
                using var sub = new Mat(match, box);
                using var both = new Mat();
                Cv2.BitwiseAnd(sub, mask, both);
                int total = Cv2.CountNonZero(mask);
                return new Dictionary<string, double> { ["percent"] = total > 0 ? Cv2.CountNonZero(both) * 100.0 / total : 0 };
            });
        }
    }

    // ── intensity ───────────────────────────────────────────────────────────────

    /// <summary>Gray statistics of the region or each cell: mean, stdDev, min, max, contrast (max − min).</summary>
    internal sealed class IntensityTool : IVisionTool
    {
        public string? MainValue(VisionTool tool) => Statistic(tool);

        private static string Statistic(VisionTool tool) => tool.P.Str("statistic", "mean") switch
        {
            "stdDev" => "stdDev", "min" => "min", "max" => "max", "contrast" => "contrast", _ => "mean",
        };

        public void Run(ToolContext c)
        {
            var gray = c.Gray;
            ToolMath.MeasureWithGrid(c, Statistic(c.Tool), (mask, box) =>
            {
                using var sub = new Mat(gray, box);
                if (Cv2.CountNonZero(mask) == 0)
                    return new Dictionary<string, double> { ["mean"] = 0, ["stdDev"] = 0, ["min"] = 0, ["max"] = 0, ["contrast"] = 0 };
                Cv2.MeanStdDev(sub, out var mean, out var std, mask);
                Cv2.MinMaxLoc(sub, out double min, out double max, out _, out _, mask);
                return new Dictionary<string, double>
                {
                    ["mean"] = mean.Val0, ["stdDev"] = std.Val0, ["min"] = min, ["max"] = max, ["contrast"] = max - min,
                };
            });
        }
    }

    // ── edgePixels ──────────────────────────────────────────────────────────────

    /// <summary>Canny edge density (% of the region's pixels) and the dominant edge direction (0–180°).</summary>
    internal sealed class EdgePixelsTool : IVisionTool
    {
        public string? MainValue(VisionTool tool) => "density";

        public void Run(ToolContext c)
        {
            var p = c.P;
            using var edges = new Mat();
            Cv2.Canny(c.Gray, edges, p.Num("cannyLow", 50), p.Num("cannyHigh", 150));
            ToolMath.MeasureWithGrid(c, "density", (mask, box) =>
            {
                using var sub = new Mat(edges, box);
                using var both = new Mat();
                Cv2.BitwiseAnd(sub, mask, both);
                int total = Cv2.CountNonZero(mask);
                return new Dictionary<string, double> { ["density"] = total > 0 ? Cv2.CountNonZero(both) * 100.0 / total : 0 };
            });

            // Dominant direction over the whole region: gradient orientations of the edge
            // pixels, weighted by strength; an edge runs at 90° to its gradient.
            var (rmask, rbox) = ToolMath.RegionMask(c.Region, c.W, c.H);
            using (rmask)
            {
                if (rbox.Width < 3 || rbox.Height < 3) return;
                using var g = new Mat(c.Gray, rbox);
                using var gx = new Mat();
                using var gy = new Mat();
                Cv2.Sobel(g, gx, MatType.CV_32F, 1, 0);
                Cv2.Sobel(g, gy, MatType.CV_32F, 0, 1);
                using var e = new Mat(edges, rbox);
                var hist = new double[180];
                var ei = ToolMath.Bytes(e); var mi = ToolMath.Bytes(rmask);
                var xi = ToolMath.Floats(gx); var yi = ToolMath.Floats(gy);
                for (int i = 0; i < ei.Length; i++)
                    {
                        if (ei[i] == 0 || mi[i] == 0) continue;
                        double gxv = xi[i], gyv = yi[i];
                        double a = (Geo.AngleDeg(gxv, gyv) + 90 + 360) % 180;
                        hist[(int)a % 180] += Math.Sqrt(gxv * gxv + gyv * gyv);
                    }
                if (hist.Max() > 0) c.Out.Set("angle", Array.IndexOf(hist, hist.Max()));
            }
        }
    }

    // ── polygon ─────────────────────────────────────────────────────────────────

    /// <summary>Closed shapes with the given number of sides in a binary (thresholded) input.</summary>
    internal sealed class PolygonTool : IVisionTool
    {
        public string? MainValue(VisionTool tool) => "count";

        public void Run(ToolContext c)
        {
            var p = c.P;
            int sides = p.Int("sides", 4);
            double minArea = p.Num("minArea", 100), maxArea = p.Num("maxArea", 1e9), eps = Math.Clamp(p.Num("epsilon", 0.04), 0.001, 0.5);
            var gray = c.Gray;
            List<(double X, double Y, double Angle, double Area)> Find(Mat mask, Rect box)
            {
                var list = new List<(double, double, double, double)>();
                using var sub = new Mat(gray, box);
                using var bin = new Mat();
                Cv2.Threshold(sub, bin, 127, 255, ThresholdTypes.Binary);
                Cv2.BitwiseAnd(bin, mask, bin);
                Cv2.FindContours(bin, out var contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);
                foreach (var ct in contours)
                {
                    double area = Cv2.ContourArea(ct);
                    if (area < minArea || area > maxArea) continue;
                    var approx = Cv2.ApproxPolyDP(ct, eps * Cv2.ArcLength(ct, true), true);
                    if (approx.Length != sides) continue;
                    var m = Cv2.Moments(ct);
                    if (m.M00 == 0) continue;
                    var rr = Cv2.MinAreaRect(ct);
                    list.Add((m.M10 / m.M00 + box.X, m.M01 / m.M00 + box.Y, rr.Angle, area));
                }
                return list;
            }
            // MeasureWithGrid measures the whole region first, then each cell.
            List<(double X, double Y, double Angle, double Area)>? whole = null;
            ToolMath.MeasureWithGrid(c, "count", (mask, box) =>
            {
                var f = Find(mask, box);
                whole ??= f;
                return new Dictionary<string, double> { ["count"] = f.Count };
            });
            whole ??= new();
            foreach (var w in whole.OrderByDescending(w => w.Area))
                c.Out.Items.Add(new Dictionary<string, object>
                {
                    ["x"] = Math.Round(w.X, 2), ["y"] = Math.Round(w.Y, 2), ["angle"] = Math.Round(w.Angle, 2), ["area"] = Math.Round(w.Area, 1),
                });
            if (c.Out.Items.Count > 0)
            {
                c.Out.Set("x", (double)c.Out.Items[0]["x"]);
                c.Out.Set("y", (double)c.Out.Items[0]["y"]);
                c.Out.Set("angle", (double)c.Out.Items[0]["angle"]);
            }
        }
    }

    // ── barcode ─────────────────────────────────────────────────────────────────

    /// <summary>1-D and 2-D codes (ZXing) in the region's bounding box. formats: ZXing format names; empty = all.</summary>
    internal sealed class BarcodeTool : IVisionTool
    {
        public void Run(ToolContext c)
        {
            var box = c.Region.Bounds(c.W, c.H);
            c.Out.Set("count", 0);
            c.Out.Set("value", "");
            if (box.Width < 8 || box.Height < 8) { c.Fail("region is outside the image"); return; }
            using var view = new Mat(c.Gray, box);
            using var crop = view.Clone();
            var bytes = new byte[crop.Width * crop.Height];
            Marshal.Copy(crop.Data, bytes, 0, bytes.Length);
            var source = new RGBLuminanceSource(bytes, crop.Width, crop.Height, RGBLuminanceSource.BitmapFormat.Gray8);
            var reader = new BarcodeReaderGeneric { Options = { TryHarder = true, TryInverted = true } };
            var names = c.P.StrList("formats");
            if (names.Count > 0)
                reader.Options.PossibleFormats = names.Select(n => Enum.TryParse<BarcodeFormat>(n, out var f) ? (BarcodeFormat?)f : null)
                                                      .Where(f => f != null).Select(f => f!.Value).ToList();
            var results = reader.DecodeMultiple(source) ?? Array.Empty<ZXing.Result>();
            foreach (var r in results)
            {
                if (r?.Text == null) continue;
                var rp = r.ResultPoints?.Where(q => q != null).ToList();
                double x = rp is { Count: > 0 } ? rp.Average(q => q.X) + box.X : box.X + box.Width / 2.0;
                double y = rp is { Count: > 0 } ? rp.Average(q => q.Y) + box.Y : box.Y + box.Height / 2.0;
                c.Out.Items.Add(new Dictionary<string, object>
                {
                    ["value"] = r.Text, ["format"] = r.BarcodeFormat.ToString(), ["x"] = Math.Round(x, 2), ["y"] = Math.Round(y, 2),
                });
                if (rp is { Count: >= 2 })
                    for (int i = 0; i < rp.Count; i++)
                        c.Out.Lines.Add(new PxLine(rp[i].X + box.X, rp[i].Y + box.Y, rp[(i + 1) % rp.Count].X + box.X, rp[(i + 1) % rp.Count].Y + box.Y));
            }
            c.Out.Set("count", c.Out.Items.Count);
            if (c.Out.Items.Count == 0) { c.Fail("no code read"); return; }
            c.Out.Set("value", (string)c.Out.Items[0]["value"]);
            c.Out.Set("format", (string)c.Out.Items[0]["format"]);
            c.Out.Set("x", (double)c.Out.Items[0]["x"]);
            c.Out.Set("y", (double)c.Out.Items[0]["y"]);
        }
    }
}
