using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision.Inspections
{
    using Point = OpenCvSharp.Point;

    internal sealed class TextureDetection
    {
        public required TextureResult Result { get; init; }
        /// <summary>The measured windows (pixel quads) with their pass/fail, for the overlay.</summary>
        public required List<(Point2f[] Window, bool Passed, double Score)> Windows { get; init; }
    }

    /// <summary>
    /// Texture score of a zone or of each grid cell: the standard deviation of the gray image
    /// after subtracting a blurred copy of itself. The blur is a fixed fraction of the cell's
    /// size, so the score means the same on a near (big) and a far (small) cell, and at any
    /// camera resolution: detail finer than ~3% of a cell (checker rings, print, grain) counts,
    /// slow shading (glare, shadows, a lighting gradient) does not.
    /// </summary>
    internal sealed class TextureStrategy : IInspectionStrategy<TextureInspection, TextureDetection>
    {
        /// <summary>Blur sigma as a fraction of the cell's longest side.</summary>
        public const double DetailFraction = 0.03;

        public TextureDetection Run(FrameContext ctx, TextureInspection insp, VisionZone? zone)
        {
            int w = ctx.Width, h = ctx.Height;
            double inset = Math.Clamp(insp.CellInset, 0, 45) / 100.0;
            bool InRange(double s) => (!insp.MinScore.HasValue || s >= insp.MinScore.Value)
                                   && (!insp.MaxScore.HasValue || s <= insp.MaxScore.Value);

            var windows = new List<(Point2f[], bool, double)>();
            var grid = zone?.Grid;
            if (zone != null && grid != null && grid.Rows * grid.Cols > 1)
            {
                int rows = Math.Clamp(grid.Rows, 1, ColorCoverageStrategy.MaxGridDimension);
                int cols = Math.Clamp(grid.Cols, 1, ColorCoverageStrategy.MaxGridDimension);
                var cells = new List<TextureCellResult>(rows * cols);
                for (int r = 0; r < rows; r++)
                for (int c = 0; c < cols; c++)
                {
                    var cell = ZoneGeometry.CellQuad(zone.Geometry, rows, cols, r, c, w, h);
                    var win  = ZoneGeometry.CellWindow(zone.Geometry, rows, cols, r, c, inset, w, h);
                    double score = Math.Round(Score(ctx.Gray, win, LongestSide(cell), w, h), 1);
                    bool passed = InRange(score);
                    cells.Add(new TextureCellResult { Row = r, Col = c, Index = r * cols + c, Score = score, Passed = passed });
                    windows.Add((win, passed, score));
                }
                return new TextureDetection
                {
                    Windows = windows,
                    Result = new TextureResult
                    {
                        InspectionId = insp.Id,
                        Name         = insp.Name,
                        Score        = cells.Count == 0 ? 0 : cells.Max(x => x.Score),
                        Passed       = cells.Any(x => x.Passed),
                        Cells        = cells,
                        CellsPassed  = cells.Count(x => x.Passed),
                    },
                };
            }

            // No grid: the whole zone (or frame) is one cell, measured without an inset.
            var whole = zone != null ? ZoneGeometry.FourCorners(zone.Geometry, w, h) : null;
            double s0;
            if (whole != null) s0 = Score(ctx.Gray, whole, LongestSide(whole), w, h);
            else
            {
                var b = zone != null ? ZoneGeometry.ZoneBounds(zone.Geometry, w, h) : new Rect(0, 0, w, h);
                using var zoneBox = new Mat(ctx.ZoneMask(zone), b);
                s0 = ScoreMasked(ctx.Gray, zoneBox, b, Math.Max(b.Width, b.Height));
                whole = new[] { new Point2f(b.X, b.Y), new Point2f(b.Right, b.Y), new Point2f(b.Right, b.Bottom), new Point2f(b.X, b.Bottom) };
            }
            s0 = Math.Round(s0, 1);
            windows.Add((whole, InRange(s0), s0));
            return new TextureDetection
            {
                Windows = windows,
                Result  = new TextureResult { InspectionId = insp.Id, Name = insp.Name, Score = s0, Passed = InRange(s0) },
            };
        }

        private static double LongestSide(Point2f[] q)
        {
            double m = 0;
            for (int i = 0; i < q.Length; i++)
            {
                var a = q[i]; var b = q[(i + 1) % q.Length];
                m = Math.Max(m, Math.Sqrt((b.X - a.X) * (b.X - a.X) + (b.Y - a.Y) * (b.Y - a.Y)));
            }
            return m;
        }

        /// <summary>Texture score inside a pixel quad; the blur scale comes from the cell size.</summary>
        internal static double Score(Mat gray, Point2f[] window, double cellSize, int w, int h)
        {
            var box = ZoneGeometry.QuadBounds(window, w, h);
            if (box.Width < 2 || box.Height < 2) return 0;
            using var mask = new Mat(box.Size, MatType.CV_8UC1, Scalar.Black);
            Cv2.FillPoly(mask, new[] { ZoneGeometry.ToIntPoints(window).Select(p => new Point(p.X - box.X, p.Y - box.Y)).ToArray() }, Scalar.White);
            return ScoreMasked(gray, mask, box, cellSize);
        }

        /// <summary>
        /// The score over the white pixels of <paramref name="maskBox"/>, a mask the size of
        /// <paramref name="box"/> (the window's pixel bounds).
        /// The high-pass is taken over a padded box so the blur near the window edge sees real
        /// pixels around it rather than a border.
        /// </summary>
        internal static double ScoreMasked(Mat gray, Mat maskBox, Rect box, double cellSize)
        {
            int w = gray.Width, h = gray.Height;
            double sigma = Math.Max(0.6, DetailFraction * cellSize);
            int pad = (int)Math.Ceiling(sigma * 3) + 1;
            var padded = ZoneGeometry.ClampRect(box.X - pad, box.Y - pad, box.Width + 2 * pad, box.Height + 2 * pad, w, h);
            if (padded.Width < 2 || padded.Height < 2) return 0;

            using var roi = new Mat();
            using (var src = new Mat(gray, padded)) src.ConvertTo(roi, MatType.CV_32FC1);
            using var blur = new Mat();
            Cv2.GaussianBlur(roi, blur, new Size(0, 0), sigma);
            using var hp = new Mat();
            Cv2.Subtract(roi, blur, hp);

            var inner = new Rect(box.X - padded.X, box.Y - padded.Y, box.Width, box.Height);
            using var hpBox   = new Mat(hp, inner);
            if (Cv2.CountNonZero(maskBox) < 4) return 0;
            Cv2.MeanStdDev(hpBox, out _, out var std, maskBox);
            return std.Val0;
        }

        public void Annotate(Mat annotated, TextureDetection d, TextureInspection insp, VisionZone? zone, LabelStack labels)
        {
            if (zone != null)
                VisionDrawing.DrawZoneBorder(annotated, zone.Geometry, annotated.Width, annotated.Height, insp.Name, VisionPalette.ZoneBorder);
            foreach (var (win, passed, score) in d.Windows)
            {
                var pts = ZoneGeometry.ToIntPoints(win);
                Cv2.Polylines(annotated, new[] { pts }, true, passed ? VisionPalette.Texture : VisionPalette.Fail, passed ? 2 : 1);
                var label = new Point((int)pts.Average(p => p.X) - 8, (int)pts.Average(p => p.Y) + 4);
                Cv2.PutText(annotated, $"{score:0}", label, VisionDrawing.Font, VisionDrawing.CellFontScale, VisionPalette.GridCellText, 1);
            }
            labels.DrawStatusLabel(annotated, insp.Name, d.Result.Passed);
        }
    }
}
