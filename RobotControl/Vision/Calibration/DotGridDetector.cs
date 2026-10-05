using OpenCvSharp;

namespace Controller.RobotControl.Vision.Calibration
{
    /// <summary>Blob filter settings for <see cref="DotGridDetector"/>.</summary>
    public sealed class DotDetectorParams
    {
        public const double DefaultMinAreaPx = 30;
        public const double DefaultMaxAreaPx = 20_000;

        public double MinAreaPx      { get; set; } = DefaultMinAreaPx;
        public double MaxAreaPx      { get; set; } = DefaultMaxAreaPx;
        /// <summary>Dark dots on a light sheet (true) or light dots on a dark sheet.</summary>
        public bool   DarkDots       { get; set; } = true;
        public double MinCircularity { get; set; } = 0.6;

        public DotDetectorParams Clone() => (DotDetectorParams)MemberwiseClone();
    }

    /// <summary>A candidate blob before lattice fitting (pixel centroid and area).</summary>
    public readonly record struct DotBlob(double X, double Y, double Area);

    /// <summary>One dot of the fitted grid.</summary>
    public sealed class DetectedDot
    {
        /// <summary>Sequential number, ordered row by row (j, then i).</summary>
        public int    Index  { get; init; }
        public int    I      { get; init; }
        public int    J      { get; init; }
        /// <summary>Pixel centroid.</summary>
        public double X      { get; init; }
        public double Y      { get; init; }
        /// <summary>Normalized centroid (0–1).</summary>
        public double U      { get; init; }
        public double V      { get; init; }
        public double AreaPx { get; init; }
        /// <summary>Distance from the lattice point the grid homography predicts, px.</summary>
        public double ErrorPx { get; init; }
    }

    /// <summary>The fitted dot grid of one frame.</summary>
    public sealed class DotGridResult
    {
        public int ImageWidth  { get; init; }
        public int ImageHeight { get; init; }
        public List<DetectedDot> Dots { get; init; } = new();
        public int Rows { get; init; }
        public int Cols { get; init; }
        /// <summary>Reprojection RMS of the kept dots through <see cref="LatticeToPixel"/>, px.</summary>
        public double RmsPx { get; init; }
        /// <summary>Median nearest-neighbour distance between dots, px.</summary>
        public double MedianSpacingPx { get; init; }
        /// <summary>Homography from lattice indices (i, j) to pixels.</summary>
        public double[][] LatticeToPixel { get; init; } = Homography.Identity();
        /// <summary>Blobs that passed the area/circularity filter but were not kept on the lattice.</summary>
        public int DroppedCount { get; init; }
        public List<string> Warnings { get; init; } = new();

        /// <summary>Homography pixel → sheet millimetres for a given dot pitch.</summary>
        public double[][] PixelToSheet(double pitchMm) =>
            Homography.ScaleOutput(Homography.Invert(LatticeToPixel), pitchMm);

        public DetectedDot? FindDot(int index) => Dots.FirstOrDefault(d => d.Index == index);
    }

    /// <summary>
    /// Finds the dots of a calibration sheet and fits the regular lattice through them — the
    /// rules of docs/camera-calibration.md "Grid fitting rules".
    /// </summary>
    public static class DotGridDetector
    {
        /// <summary>Neighbours are dots closer than this many median spacings (diagonals sit at √2).</summary>
        private const double NeighbourRadius   = 1.3;
        /// <summary>How far from an integer lattice step an offset may be and still count as one.</summary>
        private const double LatticeTolerance  = 0.35;
        private const int    MaxIterations     = 5;
        /// <summary>Outliers lie beyond this multiple of the RMS…</summary>
        private const double OutlierRmsFactor  = 3.0;
        /// <summary>…but never closer than this (px), so near-perfect grids do not shed good dots to quantisation noise.</summary>
        private const double OutlierFloorPx    = 0.5;
        /// <summary><c>gridNotFound</c> when the fit RMS exceeds this fraction of the median dot spacing.</summary>
        private const double MaxRmsFraction    = 0.02;
        private const int    MaxBlobs          = 5000;

        /// <summary>Decodes a JPEG and runs <see cref="Detect(Mat, DotDetectorParams)"/>.</summary>
        public static DotGridResult DetectJpeg(byte[] jpeg, DotDetectorParams p)
        {
            using var img = Cv2.ImDecode(jpeg, ImreadModes.Color);
            if (img.Empty()) throw new CalibrationException(CalibrationErrors.NoDotsFound, "The camera frame could not be decoded");
            return Detect(img, p);
        }

        /// <summary>Detects the dots and fits the grid. Throws <see cref="CalibrationException"/> (noDotsFound / gridNotFound).</summary>
        public static DotGridResult Detect(Mat image, DotDetectorParams p)
        {
            var blobs = DetectBlobs(image, p);
            if (blobs.Count == 0)
                throw new CalibrationException(CalibrationErrors.NoDotsFound,
                    $"No {(p.DarkDots ? "dark" : "light")} dots between {p.MinAreaPx:0}–{p.MaxAreaPx:0} px² were found");
            return FitGrid(blobs, image.Width, image.Height);
        }

        /// <summary>
        /// Grayscale → Gaussian blur → Otsu threshold (inverted for dark dots) → contours, kept
        /// by area and circularity; the centroid comes from the contour moments.
        /// </summary>
        public static List<DotBlob> DetectBlobs(Mat image, DotDetectorParams p)
        {
            using var gray = new Mat();
            if (image.Channels() == 1) image.CopyTo(gray);
            else if (image.Channels() == 4) Cv2.CvtColor(image, gray, ColorConversionCodes.BGRA2GRAY);
            else Cv2.CvtColor(image, gray, ColorConversionCodes.BGR2GRAY);

            using var blurred = new Mat();
            Cv2.GaussianBlur(gray, blurred, new Size(5, 5), 0);

            using var binary = new Mat();
            var type = (p.DarkDots ? ThresholdTypes.BinaryInv : ThresholdTypes.Binary) | ThresholdTypes.Otsu;
            Cv2.Threshold(blurred, binary, 0, 255, type);

            // List, not External: dots inside a region that is itself foreground (the dark
            // surround of a sheet, say) must still be found.
            Cv2.FindContours(binary, out var contours, out _, RetrievalModes.List, ContourApproximationModes.ApproxNone);

            var blobs = new List<DotBlob>();
            foreach (var c in contours)
            {
                double area = Cv2.ContourArea(c);
                if (area < p.MinAreaPx || area > p.MaxAreaPx) continue;
                double perimeter = Cv2.ArcLength(c, true);
                if (perimeter <= 0) continue;
                double circularity = 4 * Math.PI * area / (perimeter * perimeter);
                if (circularity < p.MinCircularity) continue;
                var m = Cv2.Moments(c);
                if (Math.Abs(m.M00) < 1e-9) continue;
                blobs.Add(new DotBlob(m.M10 / m.M00, m.M01 / m.M00, area));
            }
            return blobs;
        }

        /// <summary>Fits the lattice through already-detected blob centroids.</summary>
        public static DotGridResult FitGrid(IReadOnlyList<DotBlob> blobs, int imageWidth, int imageHeight)
        {
            int n = blobs.Count;
            if (n < 4)
                throw new CalibrationException(CalibrationErrors.GridNotFound, $"Only {n} dot(s) found; at least 4 are needed");
            if (n > MaxBlobs)
                throw new CalibrationException(CalibrationErrors.GridNotFound,
                    $"{n} blobs found; that is too many for a dot sheet (check the area range and lighting)");

            var px = blobs.Select(b => b.X).ToArray();
            var py = blobs.Select(b => b.Y).ToArray();

            // ── Spacing and basis vectors ─────────────────────────────────────
            var nn = new double[n];
            for (int k = 0; k < n; k++)
            {
                double best = double.MaxValue;
                for (int l = 0; l < n; l++)
                    if (l != k) best = Math.Min(best, Dist(px, py, k, l));
                nn[k] = best;
            }
            double spacing = Median(nn);
            if (!(spacing > 0))
                throw new CalibrationException(CalibrationErrors.GridNotFound, "Dots overlap; no grid spacing could be measured");

            var vectors = new List<(double X, double Y)>();
            for (int k = 0; k < n; k++)
                for (int l = 0; l < n; l++)
                    if (l != k && Dist(px, py, k, l) < NeighbourRadius * spacing)
                        vectors.Add((px[l] - px[k], py[l] - py[k]));
            if (vectors.Count < 4)
                throw new CalibrationException(CalibrationErrors.GridNotFound, "The dots have too few regular neighbours to form a grid");

            // The two lattice axes are ~90° apart, so their angles agree modulo 90°: the
            // circular mean of 4θ finds that shared direction.
            double sc = 0, ss = 0;
            foreach (var v in vectors)
            {
                double th = Math.Atan2(v.Y, v.X);
                sc += Math.Cos(4 * th); ss += Math.Sin(4 * th);
            }
            double phi = Math.Atan2(ss, sc) / 4;
            var ax1 = (X: Math.Cos(phi), Y: Math.Sin(phi));
            var ax2 = (X: -Math.Sin(phi), Y: Math.Cos(phi));

            var c1 = new List<(double X, double Y)>();
            var c2 = new List<(double X, double Y)>();
            foreach (var v in vectors)
            {
                double d1 = v.X * ax1.X + v.Y * ax1.Y, d2 = v.X * ax2.X + v.Y * ax2.Y;
                if (Math.Abs(d1) >= Math.Abs(d2)) c1.Add(d1 >= 0 ? v : (-v.X, -v.Y));
                else                              c2.Add(d2 >= 0 ? v : (-v.X, -v.Y));
            }
            if (c1.Count == 0 || c2.Count == 0)
                throw new CalibrationException(CalibrationErrors.GridNotFound, "The dots do not form a two-dimensional grid");

            var a = (X: Median(c1.Select(v => v.X)), Y: Median(c1.Select(v => v.Y)));
            var b = (X: Median(c2.Select(v => v.X)), Y: Median(c2.Select(v => v.Y)));
            // i runs along the axis closer to image +x, j along image +y, so dot 0 is top-left.
            if (Math.Abs(a.X) < Math.Abs(b.X)) (a, b) = (b, a);
            if (a.X < 0) a = (-a.X, -a.Y);
            if (b.Y < 0) b = (-b.X, -b.Y);

            double det = a.X * b.Y - a.Y * b.X;
            double la = Math.Sqrt(a.X * a.X + a.Y * a.Y), lb = Math.Sqrt(b.X * b.X + b.Y * b.Y);
            if (Math.Abs(det) < 0.3 * la * lb)
                throw new CalibrationException(CalibrationErrors.GridNotFound, "The dot rows and columns are not distinct directions");

            // ── Initial integer indices: grow from the dot nearest the centre ─
            (int I, int J)?[] idx = new (int, int)?[n];
            double cx = px.Average(), cy = py.Average();
            int start = Enumerable.Range(0, n).MinBy(k => (px[k] - cx) * (px[k] - cx) + (py[k] - cy) * (py[k] - cy));
            idx[start] = (0, 0);
            var queue = new Queue<int>();
            queue.Enqueue(start);
            while (queue.Count > 0)
            {
                int q = queue.Dequeue();
                for (int k = 0; k < n; k++)
                {
                    if (idx[k] != null || Dist(px, py, q, k) >= NeighbourRadius * spacing) continue;
                    double dx = px[k] - px[q], dy = py[k] - py[q];
                    double fi = ( b.Y * dx - b.X * dy) / det;
                    double fj = (-a.Y * dx + a.X * dy) / det;
                    int ri = (int)Math.Round(fi), rj = (int)Math.Round(fj);
                    if ((ri == 0 && rj == 0) || Math.Abs(ri) + Math.Abs(rj) > 2) continue;
                    if (Math.Abs(fi - ri) > LatticeTolerance || Math.Abs(fj - rj) > LatticeTolerance) continue;
                    idx[k] = (idx[q]!.Value.I + ri, idx[q]!.Value.J + rj);
                    queue.Enqueue(k);
                }
            }
            idx = ResolveDuplicates(idx, px, py, null);

            // ── Fit H (i, j) → pixel, re-assign by the nearest lattice point, repeat ─
            double[][] h = Homography.Identity();
            for (int iter = 0; iter < MaxIterations; iter++)
            {
                h = FitLatticeHomography(idx, px, py, ransac: true, spacing);
                var hInv = Homography.Invert(h);

                var next = new (int I, int J)?[n];
                for (int k = 0; k < n; k++)
                {
                    var (fi, fj) = Homography.Apply(hInv, px[k], py[k]);
                    if (double.IsNaN(fi)) continue;
                    int ri = (int)Math.Round(fi), rj = (int)Math.Round(fj);
                    if (Math.Abs(fi - ri) > LatticeTolerance || Math.Abs(fj - rj) > LatticeTolerance) continue;
                    next[k] = (ri, rj);
                }
                next = ResolveDuplicates(next, px, py, h);

                bool same = true;
                for (int k = 0; k < n && same; k++) same = Nullable.Equals(next[k], idx[k]);
                idx = next;
                if (same) break;
            }

            // ── Drop outliers (> 3 × RMS), then the final least-squares fit ───
            h = FitLatticeHomography(idx, px, py, ransac: false, spacing);
            var err = Errors(idx, px, py, h);
            double rms = Rms(idx, err);
            double limit = Math.Max(OutlierRmsFactor * rms, OutlierFloorPx);
            for (int k = 0; k < n; k++)
                if (idx[k] != null && err[k] > limit) idx[k] = null;

            h   = FitLatticeHomography(idx, px, py, ransac: false, spacing);
            err = Errors(idx, px, py, h);
            rms = Rms(idx, err);

            int kept = idx.Count(x => x != null);
            if (kept < 4)
                throw new CalibrationException(CalibrationErrors.GridNotFound, $"Only {kept} dot(s) fit a regular grid; at least 4 are needed");
            if (rms > MaxRmsFraction * spacing)
                throw new CalibrationException(CalibrationErrors.GridNotFound,
                    $"The dots do not form a consistent grid (fit error {rms:0.00} px is over 2 % of the {spacing:0.0} px spacing)");

            // ── Shift indices to start at 0 and number the dots row by row ────
            int minI = idx.Where(x => x != null).Min(x => x!.Value.I);
            int minJ = idx.Where(x => x != null).Min(x => x!.Value.J);
            h = Homography.Normalize(Homography.Multiply(h, [[1, 0, minI], [0, 1, minJ], [0, 0, 1]]));

            var ordered = Enumerable.Range(0, n).Where(k => idx[k] != null)
                .Select(k => (k, I: idx[k]!.Value.I - minI, J: idx[k]!.Value.J - minJ))
                .OrderBy(t => t.J).ThenBy(t => t.I).ToList();

            var dots = ordered.Select((t, seq) => new DetectedDot
            {
                Index   = seq,
                I       = t.I,
                J       = t.J,
                X       = px[t.k],
                Y       = py[t.k],
                U       = imageWidth  > 0 ? px[t.k] / imageWidth  : 0,
                V       = imageHeight > 0 ? py[t.k] / imageHeight : 0,
                AreaPx  = blobs[t.k].Area,
                ErrorPx = err[t.k],
            }).ToList();

            int rows = dots.Max(d => d.J) + 1, cols = dots.Max(d => d.I) + 1;
            var warnings = new List<string>();
            int dropped = n - dots.Count;
            if (dropped > 0)
                warnings.Add($"{dropped} blob(s) ignored because they do not sit on the grid (partial dots at the edge, dirt or reflections)");
            if (dots.Count < rows * cols)
                warnings.Add($"The grid is incomplete: {dots.Count} of {rows} × {cols} positions have a dot");
            if (dots.Count < 9)
                warnings.Add($"Only {dots.Count} dots: the perspective correction may be unreliable; show more of the sheet");
            if (imageWidth > 0 && imageHeight > 0)
            {
                double bw = dots.Max(d => d.X) - dots.Min(d => d.X), bh = dots.Max(d => d.Y) - dots.Min(d => d.Y);
                double cover = bw * bh / ((double)imageWidth * imageHeight);
                if (cover < 0.25)
                    warnings.Add($"The grid covers only {cover * 100:0}% of the image; positions far from the sheet will be less accurate");
            }

            return new DotGridResult
            {
                ImageWidth      = imageWidth,
                ImageHeight     = imageHeight,
                Dots            = dots,
                Rows            = rows,
                Cols            = cols,
                RmsPx           = rms,
                MedianSpacingPx = spacing,
                LatticeToPixel  = h,
                DroppedCount    = dropped,
                Warnings        = warnings,
            };
        }

        // ── Helpers ───────────────────────────────────────────────────────────

        private static double[][] FitLatticeHomography((int I, int J)?[] idx, double[] px, double[] py, bool ransac, double spacing)
        {
            var src = new List<Point2d>();
            var dst = new List<Point2d>();
            for (int k = 0; k < idx.Length; k++)
            {
                if (idx[k] is not { } ij) continue;
                src.Add(new Point2d(ij.I, ij.J));
                dst.Add(new Point2d(px[k], py[k]));
            }
            if (src.Count < 4)
                throw new CalibrationException(CalibrationErrors.GridNotFound, $"Only {src.Count} dot(s) fit a regular grid; at least 4 are needed");

            using var hm = Cv2.FindHomography(src, dst, ransac ? HomographyMethods.Ransac : HomographyMethods.None,
                                               Math.Max(1.0, 0.1 * spacing));
            if (hm.Empty() || hm.Rows != 3 || hm.Cols != 3)
                throw new CalibrationException(CalibrationErrors.GridNotFound, "No consistent grid could be fitted through the dots");
            var h = Homography.FromMat(hm);
            if (Math.Abs(Homography.Determinant(h)) < 1e-12)
                throw new CalibrationException(CalibrationErrors.GridNotFound, "No consistent grid could be fitted through the dots");
            return h;
        }

        /// <summary>Two blobs claiming the same (i, j): the one nearer its lattice point (or the first found) keeps it.</summary>
        private static (int I, int J)?[] ResolveDuplicates((int I, int J)?[] idx, double[] px, double[] py, double[][]? h)
        {
            var owner = new Dictionary<(int, int), int>();
            var result = ((int I, int J)?[])idx.Clone();
            for (int k = 0; k < idx.Length; k++)
            {
                if (idx[k] is not { } ij) continue;
                if (!owner.TryGetValue(ij, out var other)) { owner[ij] = k; continue; }
                if (h != null && ErrorOf(ij, px[k], py[k], h) < ErrorOf(ij, px[other], py[other], h))
                {
                    result[other] = null;
                    owner[ij] = k;
                }
                else result[k] = null;
            }
            return result;
        }

        private static double ErrorOf((int I, int J) ij, double x, double y, double[][] h)
        {
            var (ex, ey) = Homography.Apply(h, ij.I, ij.J);
            return Math.Sqrt((ex - x) * (ex - x) + (ey - y) * (ey - y));
        }

        private static double[] Errors((int I, int J)?[] idx, double[] px, double[] py, double[][] h)
        {
            var e = new double[idx.Length];
            for (int k = 0; k < idx.Length; k++)
                e[k] = idx[k] is { } ij ? ErrorOf(ij, px[k], py[k], h) : double.NaN;
            return e;
        }

        private static double Rms((int I, int J)?[] idx, double[] err)
        {
            double sum = 0; int count = 0;
            for (int k = 0; k < idx.Length; k++)
                if (idx[k] != null) { sum += err[k] * err[k]; count++; }
            return count == 0 ? 0 : Math.Sqrt(sum / count);
        }

        private static double Dist(double[] px, double[] py, int a, int b) =>
            Math.Sqrt((px[a] - px[b]) * (px[a] - px[b]) + (py[a] - py[b]) * (py[a] - py[b]));

        private static double Median(IEnumerable<double> values)
        {
            var s = values.OrderBy(v => v).ToArray();
            if (s.Length == 0) return 0;
            return s.Length % 2 == 1 ? s[s.Length / 2] : (s[s.Length / 2 - 1] + s[s.Length / 2]) / 2;
        }
    }
}
