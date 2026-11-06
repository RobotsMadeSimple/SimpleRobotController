using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision.Inspections
{
    /// <summary>
    /// Fits a rows × cols checkerboard to the squares in the image: moves a quad's four corners
    /// until the grid's lines sit on the boundaries between squares. The score is the mean
    /// edge strength across each grid line (|gradient · line normal|), sampled along the
    /// interior lines only (rows−1 + cols−1): those are always square-to-square boundaries,
    /// while the board's outer edge is often invisible (dark squares against a dark frame)
    /// and the frame's own outline — usually the strongest edge in view — would pull the
    /// grid out onto it. A board seen at an angle is followed by the perspective quad.
    ///
    /// Hill-climbing from a start quad, coarse to fine: first only whole-grid moves (shift,
    /// scale, rotate, stretch) on a heavily blurred gradient so a start a fair way off still
    /// slides in, then each corner on its own on sharper gradients. A checker pattern also
    /// scores well one whole square off (7 of 9 lines still land on boundaries), so the fit
    /// only ever snaps to the squares nearest its start.
    ///
    /// <see cref="EdgeRatio"/> judges a fit independent of lighting: the edge strength on the
    /// grid lines over that on the lines halfway between them (through square centres).
    /// </summary>
    internal sealed class CheckerGridFit : IDisposable
    {
        private const int SamplesPerCell = 10;

        private readonly Mat _gray;
        private readonly int _rows, _cols, _w, _h;
        private readonly Dictionary<int, (float[] Gx, float[] Gy)> _grads = new();
        private readonly (double U, double V, bool Vertical)[] _onLine, _offLine;

        public CheckerGridFit(Mat gray, int rows, int cols)
        {
            _gray = new Mat();
            gray.ConvertTo(_gray, MatType.CV_32FC1);
            _rows = Math.Clamp(rows, 2, 32);
            _cols = Math.Clamp(cols, 2, 32);
            _w = gray.Width; _h = gray.Height;
            _onLine  = Samples(0.0);
            _offLine = Samples(0.5);
        }

        public void Dispose() => _gray.Dispose();

        /// <summary>
        /// Sample points (in grid units: u across 0..cols, v down 0..rows) along the interior
        /// grid lines (offset 0), or along the lines through cell centres (offset 0.5).
        /// </summary>
        private (double, double, bool)[] Samples(double offset)
        {
            var list = new List<(double, double, bool)>();
            int first = offset == 0 ? 1 : 0;
            for (int i = first; i < _cols; i++)
                for (int k = 0; k < _rows * SamplesPerCell; k++)
                    list.Add((i + offset, (k + 0.5) / SamplesPerCell, true));
            for (int j = first; j < _rows; j++)
                for (int k = 0; k < _cols * SamplesPerCell; k++)
                    list.Add(((k + 0.5) / SamplesPerCell, j + offset, false));
            return list.ToArray();
        }

        /// <summary>Sobel gradients of the gray image blurred by sigma (px), cached per sigma.</summary>
        private (float[] Gx, float[] Gy) Grads(double sigma)
        {
            int key = (int)Math.Round(Math.Max(0, sigma) * 4);
            if (_grads.TryGetValue(key, out var g)) return g;
            using var blurred = new Mat();
            if (sigma > 0.3) Cv2.GaussianBlur(_gray, blurred, new Size(0, 0), sigma);
            else _gray.CopyTo(blurred);
            using var gx = new Mat();
            using var gy = new Mat();
            Cv2.Sobel(blurred, gx, MatType.CV_32FC1, 1, 0, 3);
            Cv2.Sobel(blurred, gy, MatType.CV_32FC1, 0, 1, 3);
            gx.GetArray(out float[] ax);
            gy.GetArray(out float[] ay);
            return _grads[key] = (ax, ay);
        }

        /// <summary>
        /// Unit-square → quad projective map (Heckbert): corners TL(0,0) TR(1,0) BR(1,1) BL(0,1).
        /// Closed form, no allocation — it runs thousands of times per fit.
        /// </summary>
        private readonly struct SquareToQuad
        {
            private readonly double a, b, c, d, e, f, g, h;
            public SquareToQuad(Point2f[] q)
            {
                double x0 = q[0].X, y0 = q[0].Y, x1 = q[1].X, y1 = q[1].Y, x2 = q[2].X, y2 = q[2].Y, x3 = q[3].X, y3 = q[3].Y;
                double sx = x0 - x1 + x2 - x3, sy = y0 - y1 + y2 - y3;
                if (Math.Abs(sx) < 1e-9 && Math.Abs(sy) < 1e-9)
                {
                    a = x1 - x0; b = x3 - x0; c = x0; d = y1 - y0; e = y3 - y0; f = y0; g = 0; h = 0;
                }
                else
                {
                    double dx1 = x1 - x2, dx2 = x3 - x2, dy1 = y1 - y2, dy2 = y3 - y2;
                    double den = dx1 * dy2 - dx2 * dy1;
                    if (Math.Abs(den) < 1e-12) den = 1e-12;
                    g = (sx * dy2 - dx2 * sy) / den;
                    h = (dx1 * sy - sx * dy1) / den;
                    a = x1 - x0 + g * x1; b = x3 - x0 + h * x3; c = x0;
                    d = y1 - y0 + g * y1; e = y3 - y0 + h * y3; f = y0;
                }
            }
            public (double X, double Y) Map(double u, double v)
            {
                double z = g * u + h * v + 1;
                return ((a * u + b * v + c) / z, (d * u + e * v + f) / z);
            }
        }

        private double Measure((double U, double V, bool Vertical)[] samples, Point2f[] q, double sigma)
        {
            if (!ZoneGeometry.Convex(q)) return 0;
            var (gx, gy) = Grads(sigma);
            var m = new SquareToQuad(q);
            const double eps = 0.02;
            double total = 0; int n = 0;
            foreach (var (u, v, vertical) in samples)
            {
                double su = u / _cols, sv = v / _rows;
                var (x, y) = m.Map(su, sv);
                int xi = (int)Math.Round(x), yi = (int)Math.Round(y);
                n++;
                if (xi < 0 || yi < 0 || xi >= _w || yi >= _h) continue;
                // Direction along the line in the image, then its normal.
                var (x2, y2) = vertical ? m.Map(su, sv + eps / _rows) : m.Map(su + eps / _cols, sv);
                double dx = x2 - x, dy = y2 - y, len = Math.Sqrt(dx * dx + dy * dy);
                if (len < 1e-9) continue;
                double nx = -dy / len, ny = dx / len;
                int idx = yi * _w + xi;
                total += Math.Abs(gx[idx] * nx + gy[idx] * ny);
            }
            return n == 0 ? 0 : total / n;
        }

        /// <summary>Mean edge strength across the grid lines (higher = better aligned).</summary>
        public double Score(Point2f[] q, double sigma = 1.0) => Measure(_onLine, q, sigma);

        /// <summary>Grid-line edge strength over square-centre-line edge strength (sharp gradients).</summary>
        public double EdgeRatio(Point2f[] q)
        {
            double on = Measure(_onLine, q, 1.0), off = Measure(_offLine, q, 1.0);
            return on / Math.Max(1e-6, off);
        }

        /// <summary>The narrowest cell edge of the quad, px — the scale every step is set against.</summary>
        private double CellSize(Point2f[] q)
        {
            static double D(Point2f a, Point2f b) => Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y));
            return Math.Max(4, Math.Min(Math.Min(D(q[0], q[1]), D(q[3], q[2])) / _cols, Math.Min(D(q[0], q[3]), D(q[1], q[2])) / _rows));
        }

        private static IEnumerable<Point2f[]> GlobalMoves(Point2f[] q, double step, double cell)
        {
            float cx = q.Average(p => p.X), cy = q.Average(p => p.Y);
            Point2f[] Each(Func<Point2f, Point2f> f) => q.Select(f).ToArray();
            foreach (var (dx, dy) in Dirs) yield return Each(p => new Point2f(p.X + (float)(dx * step), p.Y + (float)(dy * step)));
            double k = step / (cell * 4);   // about `step` px at the edge of the board
            foreach (var s in new[] { 1 + k, 1 - k })
            {
                yield return Each(p => new Point2f(cx + (float)((p.X - cx) * s), cy + (float)((p.Y - cy) * s)));
                yield return Each(p => new Point2f(cx + (float)((p.X - cx) * s), p.Y));
                yield return Each(p => new Point2f(p.X, cy + (float)((p.Y - cy) * s)));
            }
            foreach (var a in new[] { k, -k })
            {
                double cos = Math.Cos(a), sin = Math.Sin(a);
                yield return Each(p => new Point2f(cx + (float)((p.X - cx) * cos - (p.Y - cy) * sin),
                                                   cy + (float)((p.X - cx) * sin + (p.Y - cy) * cos)));
            }
        }

        private static readonly (int, int)[] Dirs = { (1, 0), (-1, 0), (0, 1), (0, -1) };

        private static IEnumerable<Point2f[]> CornerMoves(Point2f[] q, double step)
        {
            for (int i = 0; i < 4; i++)
                foreach (var (dx, dy) in Dirs)
                {
                    var t = (Point2f[])q.Clone();
                    t[i] = new Point2f(t[i].X + (float)(dx * step), t[i].Y + (float)(dy * step));
                    yield return t;
                }
        }

        private (Point2f[] Q, double Score) Climb(Point2f[] q, double sigma, double step, double stop,
                                                  Func<Point2f[], double, IEnumerable<Point2f[]>> moves)
        {
            double best = Score(q, sigma);
            int guard = 0;
            while (step > stop && guard < 2000)
            {
                bool improved = true;
                while (improved && guard++ < 2000)
                {
                    improved = false;
                    foreach (var t in moves(q, step))
                    {
                        double s = Score(t, sigma);
                        if (s > best + 1e-6) { best = s; q = t; improved = true; }
                    }
                }
                step /= 2;
            }
            return (q, best);
        }

        /// <summary>
        /// Small correction from a start that is already close (the last fit, the reference):
        /// a camera nudged by up to about a third of a square.
        /// </summary>
        public (Point2f[] Quad, double Score) Track(Point2f[] start)
        {
            double cell = CellSize(start);
            var (q, _) = Climb(start, cell * 0.15, cell * 0.15, 1.0, (p, s) => GlobalMoves(p, s, cell));
            return Climb(q, 1.0, 3.0, 0.25, (p, s) => CornerMoves(p, s).Concat(GlobalMoves(p, s, cell)));
        }

        /// <summary>
        /// Full fit from a rough start (a zone drawn by hand over the squares). It snaps to the
        /// nearest squares, so the start must be within about half a square of the board: a
        /// grid one whole square off scores nearly as well (dark squares at the board's edge
        /// meet a dark frame with no edge to tell them apart), so it is never searched for.
        /// </summary>
        public (Point2f[] Quad, double Score) Fit(Point2f[] start)
        {
            double cell = CellSize(start);
            var q = start;
            foreach (var blur in new[] { cell * 0.15, cell * 0.07 })
                (q, _) = Climb(q, blur, cell * 0.15, 0.5, (p, s) => CornerMoves(p, s).Concat(GlobalMoves(p, s, cell)));
            return Climb(q, 1.0, 2.0, 0.25, (p, s) => CornerMoves(p, s).Concat(GlobalMoves(p, s, cell)));
        }
    }
}
