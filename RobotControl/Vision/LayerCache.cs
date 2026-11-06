using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;

namespace Controller.RobotControl.Vision
{
    /// <summary>
    /// One decoded frame plus every image layer computed from it, built lazily and cached so
    /// several tools reading the same layer share one filter run. Every Mat handed out is
    /// owned by the cache and disposed with it — callers treat them as read-only. The source
    /// Mat is borrowed. Single-threaded: one cache per frame per thread.
    /// </summary>
    internal sealed class LayerCache : IDisposable
    {
        private readonly Dictionary<string, ImageLayer> _defs;
        private readonly Dictionary<string, Mat> _layers = new();
        private readonly Dictionary<string, Mat> _grays  = new();
        private readonly HashSet<string> _building = new();
        private Mat? _sourceGray;
        private bool _disposed;

        public LayerCache(Mat source, IEnumerable<ImageLayer>? layers)
        {
            Source = source;
            _defs = new Dictionary<string, ImageLayer>();
            foreach (var l in layers ?? [])
                if (!string.IsNullOrEmpty(l.Id)) _defs.TryAdd(l.Id, l);
        }

        /// <summary>The decoded BGR frame.</summary>
        public Mat Source { get; }
        public int Width  => Source.Width;
        public int Height => Source.Height;

        /// <summary>Layer ids computed so far, in the order they were built.</summary>
        public List<string> Computed { get; } = new();
        /// <summary>Milliseconds spent building each layer.</summary>
        public Dictionary<string, double> Timings { get; } = new();

        public bool Exists(string? id) => IsCamera(id) || _defs.ContainsKey(id!);

        private static bool IsCamera(string? id) => string.IsNullOrEmpty(id) || id == VisionTool.CameraInput;

        /// <summary>The image of a layer ("camera"/empty = the frame itself).</summary>
        public Mat Get(string? id)
        {
            ThrowIfDisposed();
            if (IsCamera(id)) return Source;
            if (_layers.TryGetValue(id!, out var cached)) return cached;
            if (!_defs.TryGetValue(id!, out var def)) throw new InvalidOperationException($"layer '{id}' does not exist");
            if (!_building.Add(id!)) throw new InvalidOperationException($"layer '{id}' reads itself (a cycle)");
            try
            {
                var sw = Stopwatch.StartNew();
                var input = Get(def.Input);
                var mat = VisionFilters.ApplyAll(input, def.Filters, def.Filters?.Count ?? 0);
                _layers[id!] = mat;
                Computed.Add(id!);
                Timings[id!] = Math.Round(sw.Elapsed.TotalMilliseconds, 1);
                return mat;
            }
            finally { _building.Remove(id!); }
        }

        /// <summary>A single-channel 8-bit version of a layer (colour is converted to gray).</summary>
        public Mat Gray(string? id)
        {
            var m = Get(id);
            if (m.Channels() == 1 && m.Depth() == MatType.CV_8U) return m;
            if (IsCamera(id))
                return _sourceGray ??= VisionFilters.ToGray(Source);
            if (!_grays.TryGetValue(id!, out var g))
                _grays[id!] = g = VisionFilters.ToGray(m);
            return g;
        }

        /// <summary>
        /// A new Mat: the layer's input run through its filters up to and including index
        /// <paramref name="upTo"/> (null = all; −1 = none, the input as it is). Caller disposes.
        /// </summary>
        public Mat Preview(string layerId, int? upTo)
        {
            if (!_defs.TryGetValue(layerId, out var def)) throw new InvalidOperationException($"layer '{layerId}' does not exist");
            int count = def.Filters?.Count ?? 0;
            int n = upTo is int u ? Math.Clamp(u + 1, 0, count) : count;
            return VisionFilters.ApplyAll(Get(def.Input), def.Filters, n);
        }

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(LayerCache));
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            foreach (var m in _layers.Values) m.Dispose();
            foreach (var m in _grays.Values)  m.Dispose();
            _sourceGray?.Dispose();
            _layers.Clear();
            _grays.Clear();
        }
    }

    /// <summary>The layer filters (docs/vision-tools.md, "Image layers").</summary>
    internal static class VisionFilters
    {
        public static Mat ToGray(Mat m)
        {
            var g = new Mat();
            if (m.Channels() == 3) Cv2.CvtColor(m, g, ColorConversionCodes.BGR2GRAY);
            else if (m.Channels() == 4) Cv2.CvtColor(m, g, ColorConversionCodes.BGRA2GRAY);
            else m.CopyTo(g);
            if (g.Depth() != MatType.CV_8U)
            {
                var c = new Mat();
                g.ConvertTo(c, MatType.CV_8U);
                g.Dispose();
                return c;
            }
            return g;
        }

        private static Mat ToBgr(Mat m)
        {
            var c = new Mat();
            if (m.Channels() == 3) m.CopyTo(c);
            else Cv2.CvtColor(m, c, ColorConversionCodes.GRAY2BGR);
            return c;
        }

        private static int Odd(double v, int min = 1)
        {
            int k = Math.Max(min, (int)Math.Round(v));
            return k % 2 == 0 ? k + 1 : k;
        }

        /// <summary>Applies the first <paramref name="count"/> filters (disabled ones are skipped) to a copy of <paramref name="input"/>.</summary>
        public static Mat ApplyAll(Mat input, IReadOnlyList<VisionFilter>? filters, int count)
        {
            var cur = input.Clone();
            for (int i = 0; i < count && filters != null && i < filters.Count; i++)
            {
                var f = filters[i];
                if (f == null || !f.Enabled) continue;
                var next = Apply(cur, f);
                if (!ReferenceEquals(next, cur)) cur.Dispose();
                cur = next;
            }
            return cur;
        }

        /// <summary>One filter on <paramref name="src"/>; returns a new Mat (src is not changed).</summary>
        public static Mat Apply(Mat src, VisionFilter f)
        {
            var p = f.P;
            switch ((f.Type ?? "").Trim())
            {
                case "channel":
                {
                    string ch = p.Str("channel", "gray").ToLowerInvariant();
                    if (src.Channels() == 1) return src.Clone();
                    if (ch == "gray") return ToGray(src);
                    if (ch is "b" or "g" or "r")
                        return src.ExtractChannel(ch == "b" ? 0 : ch == "g" ? 1 : 2);
                    using var hsv = new Mat();
                    Cv2.CvtColor(src, hsv, ColorConversionCodes.BGR2HSV);
                    return hsv.ExtractChannel(ch == "h" ? 0 : ch == "s" ? 1 : 2);
                }
                case "colorMask":
                {
                    using var bgr = ToBgr(src);
                    using var hsv = new Mat();
                    Cv2.CvtColor(bgr, hsv, ColorConversionCodes.BGR2HSV);
                    var dst = new Mat();
                    double hMin = p.Num("hMin", 0), hMax = p.Num("hMax", 179);
                    var lo = new Scalar(0, p.Num("sMin", 0), p.Num("vMin", 0));
                    var hi = new Scalar(179, p.Num("sMax", 255), p.Num("vMax", 255));
                    if (hMin <= hMax)
                        Cv2.InRange(hsv, new Scalar(hMin, lo.Val1, lo.Val2), new Scalar(hMax, hi.Val1, hi.Val2), dst);
                    else
                    {
                        // A hue range that wraps past 179 (reds): two bands.
                        using var a = new Mat();
                        using var b = new Mat();
                        Cv2.InRange(hsv, new Scalar(hMin, lo.Val1, lo.Val2), new Scalar(179, hi.Val1, hi.Val2), a);
                        Cv2.InRange(hsv, new Scalar(0, lo.Val1, lo.Val2), new Scalar(hMax, hi.Val1, hi.Val2), b);
                        Cv2.BitwiseOr(a, b, dst);
                    }
                    return dst;
                }
                case "blur":
                {
                    var dst = new Mat();
                    int k = Odd(p.Num("size", 5));
                    Cv2.GaussianBlur(src, dst, new Size(k, k), 0);
                    return dst;
                }
                case "median":
                {
                    var dst = new Mat();
                    Cv2.MedianBlur(src, dst, Odd(p.Num("size", 5), 3));
                    return dst;
                }
                case "threshold":
                {
                    using var g = ToGray(src);
                    var dst = new Mat();
                    switch (p.Str("mode", "fixed"))
                    {
                        case "otsu":
                            Cv2.Threshold(g, dst, 0, 255, ThresholdTypes.Binary | ThresholdTypes.Otsu);
                            break;
                        case "adaptive":
                            Cv2.AdaptiveThreshold(g, dst, 255, AdaptiveThresholdTypes.MeanC, ThresholdTypes.Binary,
                                Odd(p.Num("blockSize", 15), 3), p.Num("c", 5));
                            break;
                        default:
                            Cv2.InRange(g, new Scalar(p.Num("low", 128)), new Scalar(p.Num("high", 255)), dst);
                            break;
                    }
                    if (p.Bool("invert", false)) Cv2.BitwiseNot(dst, dst);
                    return dst;
                }
                case "morph":
                {
                    int k = Math.Max(1, p.Int("size", 3));
                    using var kernel = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(k, k));
                    var op = p.Str("op", "open") switch
                    {
                        "erode"  => MorphTypes.Erode,
                        "dilate" => MorphTypes.Dilate,
                        "close"  => MorphTypes.Close,
                        _        => MorphTypes.Open,
                    };
                    var dst = new Mat();
                    Cv2.MorphologyEx(src, dst, op, kernel, iterations: Math.Max(1, p.Int("iterations", 1)));
                    return dst;
                }
                case "contrast":
                {
                    switch (p.Str("mode", "linear"))
                    {
                        case "equalize":
                        {
                            using var g = ToGray(src);
                            var dst = new Mat();
                            Cv2.EqualizeHist(g, dst);
                            return dst;
                        }
                        case "clahe":
                        {
                            using var g = ToGray(src);
                            int tile = Math.Max(1, p.Int("tileSize", 8));
                            using var clahe = Cv2.CreateCLAHE(p.Num("clipLimit", 2), new Size(tile, tile));
                            var dst = new Mat();
                            clahe.Apply(g, dst);
                            return dst;
                        }
                        default:
                        {
                            var dst = new Mat();
                            src.ConvertTo(dst, src.Type(), p.Num("contrast", 1), p.Num("brightness", 0));
                            return dst;
                        }
                    }
                }
                case "edges":
                {
                    using var g = ToGray(src);
                    var dst = new Mat();
                    if (p.Str("mode", "sobel") == "canny")
                        Cv2.Canny(g, dst, p.Num("low", 50), p.Num("high", 150));
                    else
                    {
                        using var gx = new Mat();
                        using var gy = new Mat();
                        using var mag = new Mat();
                        Cv2.Sobel(g, gx, MatType.CV_32F, 1, 0);
                        Cv2.Sobel(g, gy, MatType.CV_32F, 0, 1);
                        Cv2.Magnitude(gx, gy, mag);
                        Cv2.ConvertScaleAbs(mag, dst, 0.25);
                    }
                    return dst;
                }
                case "backgroundRemove":
                {
                    using var g = ToGray(src);
                    using var f32 = new Mat();
                    g.ConvertTo(f32, MatType.CV_32F);
                    using var bg = new Mat();
                    int k = Odd(p.Num("size", 31), 3);
                    Cv2.GaussianBlur(f32, bg, new Size(k, k), 0);
                    using var diff = new Mat();
                    Cv2.Subtract(f32, bg, diff);
                    var dst = new Mat();
                    diff.ConvertTo(dst, MatType.CV_8U, 1, 128);
                    return dst;
                }
                case "invert":
                {
                    var dst = new Mat();
                    Cv2.BitwiseNot(src, dst);
                    return dst;
                }
                default:
                    throw new InvalidOperationException($"unknown filter type '{f.Type}'");
            }
        }
    }
}
