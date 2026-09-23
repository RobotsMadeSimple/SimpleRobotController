using System.Text.Json.Serialization;

namespace Controller.RobotControl.Vision.Calibration
{
    /// <summary>
    /// 2-D rigid transform: <c>out = R(θ) · (x, m·y) + t</c>, where <c>m = −1</c> when
    /// <see cref="Mirrored"/> (the input's y axis is reflected before rotating).
    /// </summary>
    public sealed class RigidTransform2D
    {
        [JsonPropertyName("cos")]      public double Cos      { get; set; } = 1;
        [JsonPropertyName("sin")]      public double Sin      { get; set; }
        [JsonPropertyName("tx")]       public double Tx       { get; set; }
        [JsonPropertyName("ty")]       public double Ty       { get; set; }
        [JsonPropertyName("mirrored")] public bool   Mirrored { get; set; }

        public (double X, double Y) Apply(double x, double y)
        {
            double my = Mirrored ? -y : y;
            return (Cos * x - Sin * my + Tx, Sin * x + Cos * my + Ty);
        }

        [JsonIgnore] public double AngleDeg => Math.Atan2(Sin, Cos) * 180.0 / Math.PI;
    }

    /// <summary>Result of <see cref="RigidFit2D.Fit"/>.</summary>
    public sealed class RigidFitResult
    {
        public RigidTransform2D Transform { get; init; } = new();
        /// <summary>Distance between each transformed source point and its target, in target units.</summary>
        public double[] Residuals { get; init; } = [];
        public double   Rms       { get; init; }
        public double   Max       { get; init; }
        /// <summary>Summed pairwise target distances / summed pairwise source distances (1 = scales agree).</summary>
        public double   ScaleEstimate { get; init; } = 1;
        /// <summary>True when the points could not decide the handedness (fewer than 3, or collinear) and it was assumed.</summary>
        public bool     HandednessAssumed { get; init; }
    }

    /// <summary>
    /// Least-squares rotation + translation (2-D Procrustes / Kabsch, no scale) from source
    /// points to target points, optionally allowing a reflection.
    /// </summary>
    public static class RigidFit2D
    {
        /// <param name="src">Source points (sheet mm).</param>
        /// <param name="dst">Target points (robot mm), same order.</param>
        /// <param name="allowReflection">Also fit the mirrored solution and keep whichever fits better.</param>
        /// <param name="preferMirrored">
        /// The handedness to use when the points cannot decide it (two points, or all on one
        /// line): both solutions then fit equally well.
        /// </param>
        public static RigidFitResult Fit(IReadOnlyList<(double X, double Y)> src, IReadOnlyList<(double X, double Y)> dst,
                                         bool allowReflection = true, bool preferMirrored = false)
        {
            if (src.Count != dst.Count) throw new ArgumentException("Point lists differ in length");
            if (src.Count < 2) throw new ArgumentException("At least two point pairs are needed");

            var normal = FitOne(src, dst, mirrored: false);
            if (!allowReflection) return normal;

            var mirror = FitOne(src, dst, mirrored: true);
            if (IsDegenerate(src))
            {
                var chosen = preferMirrored ? mirror : normal;
                return new RigidFitResult
                {
                    Transform = chosen.Transform, Residuals = chosen.Residuals, Rms = chosen.Rms, Max = chosen.Max,
                    ScaleEstimate = chosen.ScaleEstimate, HandednessAssumed = true,
                };
            }
            return mirror.Rms < normal.Rms ? mirror : normal;
        }

        /// <summary>True when there are fewer than three points or they all lie (nearly) on one line.</summary>
        public static bool IsDegenerate(IReadOnlyList<(double X, double Y)> pts)
        {
            if (pts.Count < 3) return true;
            double mx = pts.Average(p => p.X), my = pts.Average(p => p.Y);
            double sxx = 0, syy = 0, sxy = 0;
            foreach (var p in pts)
            {
                double dx = p.X - mx, dy = p.Y - my;
                sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
            }
            // Eigenvalues of the 2×2 scatter matrix; the smaller one is ~0 for collinear points.
            double tr = sxx + syy, det = sxx * syy - sxy * sxy;
            double disc = Math.Sqrt(Math.Max(0, tr * tr / 4 - det));
            double lMax = tr / 2 + disc, lMin = tr / 2 - disc;
            return lMax <= 0 || lMin / lMax < 1e-6;
        }

        private static RigidFitResult FitOne(IReadOnlyList<(double X, double Y)> src, IReadOnlyList<(double X, double Y)> dst, bool mirrored)
        {
            int n = src.Count;
            double m = mirrored ? -1 : 1;
            double sx = 0, sy = 0, dx = 0, dy = 0;
            for (int k = 0; k < n; k++) { sx += src[k].X; sy += m * src[k].Y; dx += dst[k].X; dy += dst[k].Y; }
            sx /= n; sy /= n; dx /= n; dy /= n;

            // Optimal angle: atan2(Σ cross, Σ dot) over the centred pairs.
            double sumDot = 0, sumCross = 0;
            for (int k = 0; k < n; k++)
            {
                double ax = src[k].X - sx, ay = m * src[k].Y - sy;
                double bx = dst[k].X - dx, by = dst[k].Y - dy;
                sumDot   += ax * bx + ay * by;
                sumCross += ax * by - ay * bx;
            }
            double theta = Math.Atan2(sumCross, sumDot);
            double c = Math.Cos(theta), s = Math.Sin(theta);
            var t = new RigidTransform2D
            {
                Cos = c, Sin = s, Mirrored = mirrored,
                Tx = dx - (c * sx - s * sy),
                Ty = dy - (s * sx + c * sy),
            };

            var res = new double[n];
            for (int k = 0; k < n; k++)
            {
                var (px, py) = t.Apply(src[k].X, src[k].Y);
                res[k] = Math.Sqrt((px - dst[k].X) * (px - dst[k].X) + (py - dst[k].Y) * (py - dst[k].Y));
            }

            double srcSum = 0, dstSum = 0;
            for (int a = 0; a < n; a++)
                for (int b = a + 1; b < n; b++)
                {
                    srcSum += Dist(src[a], src[b]);
                    dstSum += Dist(dst[a], dst[b]);
                }

            return new RigidFitResult
            {
                Transform     = t,
                Residuals     = res,
                Rms           = Math.Sqrt(res.Sum(r => r * r) / n),
                Max           = res.Max(),
                ScaleEstimate = srcSum > 1e-12 ? dstSum / srcSum : 1,
            };
        }

        private static double Dist((double X, double Y) a, (double X, double Y) b) =>
            Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
    }
}
