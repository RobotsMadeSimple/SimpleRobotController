using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision.Inspections
{
    using Point = OpenCvSharp.Point;

    /// <summary>Color coverage detection: the public result plus the matched-pixel mask the tint is drawn from.</summary>
    internal sealed class ColorCoverageDetection : IDisposable
    {
        public required ColorCoverageResult Result { get; init; }
        /// <summary>Pixels matching any color entry, restricted to the zone. Owned.</summary>
        public required Mat MatchMask { get; init; }
        /// <summary>Clamped grid dimensions the cells were measured on; null when not gridded.</summary>
        public (int Rows, int Cols)? Grid { get; init; }

        public void Dispose() => MatchMask.Dispose();
    }

    /// <summary>
    /// Percentage of a zone whose pixels fall within tolerance of any of a set of colors,
    /// optionally measured per cell of the zone's grid.
    /// </summary>
    internal sealed class ColorCoverageStrategy : IInspectionStrategy<ColorCoverageInspection, ColorCoverageDetection>
    {
        /// <summary>Converts a 0–100 tolerance to a 0–255 per-channel delta.</summary>
        public const double PercentToByte = 2.55;
        /// <summary>Largest rows or cols a zone grid is measured at.</summary>
        public const int MaxGridDimension = 64;

        private const double TintSourceWeight = 0.55;
        private const double TintColorWeight  = 0.45;

        public ColorCoverageDetection Run(FrameContext ctx, ColorCoverageInspection insp, VisionZone? zone)
        {
            int w = ctx.Width, h = ctx.Height;

            // White = area to measure; the whole frame when there is no zone.
            var zoneMask = ctx.ZoneMask(zone);

            using var colorMask = new Mat(ctx.Size, MatType.CV_8UC1, Scalar.Black);
            foreach (var ce in insp.Colors)
            {
                int delta = (int)Math.Round(ce.Tolerance * PercentToByte);
                var lower = new Scalar(
                    Math.Max(0, ce.B - delta),
                    Math.Max(0, ce.G - delta),
                    Math.Max(0, ce.R - delta));
                var upper = new Scalar(
                    Math.Min(255, ce.B + delta),
                    Math.Min(255, ce.G + delta),
                    Math.Min(255, ce.R + delta));
                using var oneMask = new Mat();
                Cv2.InRange(ctx.Source, lower, upper, oneMask);
                Cv2.BitwiseOr(colorMask, oneMask, colorMask);
            }

            var matchInZone = new Mat();
            try
            {
                Cv2.BitwiseAnd(colorMask, zoneMask, matchInZone);

                int total   = Cv2.CountNonZero(zoneMask);
                int matched = Cv2.CountNonZero(matchInZone);
                double coverage = total > 0 ? matched * 100.0 / total : 0;

                bool InRange(double c) => (!insp.MinCoverage.HasValue || c >= insp.MinCoverage.Value)
                                       && (!insp.MaxCoverage.HasValue || c <= insp.MaxCoverage.Value);

                bool passed = InRange(coverage);
                List<ColorCellResult>? cells = null;
                (int, int)? gridDims = null;

                var grid = zone?.Grid;
                if (grid != null && grid.Rows * grid.Cols > 1)
                {
                    int rows = Math.Clamp(grid.Rows, 1, MaxGridDimension);
                    int cols = Math.Clamp(grid.Cols, 1, MaxGridDimension);
                    gridDims = (rows, cols);
                    cells    = MeasureCells(zone!.Geometry, rows, cols, zoneMask, matchInZone, w, h, InRange);

                    // A zone-wide average hides half-full cells, so on a grid "passed" means
                    // every cell passed.
                    passed = cells.TrueForAll(cell => cell.Passed);
                }

                return new ColorCoverageDetection
                {
                    MatchMask = matchInZone,
                    Grid      = gridDims,
                    Result    = new ColorCoverageResult
                    {
                        InspectionId = insp.Id,
                        Name         = insp.Name,
                        Coverage     = Math.Round(coverage, 1),
                        Passed       = passed,
                        Cells        = cells,
                        CellsPassed  = cells?.Count(cell => cell.Passed),
                    },
                };
            }
            catch
            {
                matchInZone.Dispose();
                throw;
            }
        }

        /// <summary>
        /// Measures each cell on its own. The masks are already built, so an upright cell is
        /// just a sub-rect view of them — no extra allocation, and the zone shape still clips
        /// each cell because the zone mask carries it. A tilted cell is not a rect, so it needs
        /// a mask of its own; that mask is only the size of the cell's own bounding box, which
        /// keeps the cost close to the upright path.
        /// </summary>
        private static List<ColorCellResult> MeasureCells(
            VisionZoneGeometry geom, int rows, int cols, Mat zoneMask, Mat matchInZone,
            int w, int h, Func<double, bool> inRange)
        {
            bool tilted = ZoneGeometry.IsRotatedRect(geom);
            var bounds  = ZoneGeometry.ZoneBounds(geom, w, h);
            var cells   = new List<ColorCellResult>(rows * cols);

            double MeasureCell(int r, int c)
            {
                int cellTotal, cellMatched;
                if (tilted)
                {
                    var quad = ZoneGeometry.CellQuad(geom, rows, cols, r, c, w, h);
                    var box  = ZoneGeometry.QuadBounds(quad, w, h);
                    using var cellMask = new Mat(box.Size, MatType.CV_8UC1, Scalar.Black);
                    Cv2.FillPoly(cellMask, new[] { ZoneGeometry.ToIntPoints(quad)
                        .Select(p => new Point(p.X - box.X, p.Y - box.Y)).ToArray() },
                        Scalar.White);

                    using var zoneBox  = new Mat(zoneMask,    box);
                    using var matchBox = new Mat(matchInZone, box);
                    using var zoneIn   = new Mat();
                    using var matchIn  = new Mat();
                    Cv2.BitwiseAnd(zoneBox,  cellMask, zoneIn);
                    Cv2.BitwiseAnd(matchBox, cellMask, matchIn);
                    cellTotal   = Cv2.CountNonZero(zoneIn);
                    cellMatched = Cv2.CountNonZero(matchIn);
                }
                else
                {
                    var rect = ZoneGeometry.CellRect(bounds, rows, cols, r, c);
                    using var cellZone  = new Mat(zoneMask,    rect);
                    using var cellMatch = new Mat(matchInZone, rect);
                    cellTotal   = Cv2.CountNonZero(cellZone);
                    cellMatched = Cv2.CountNonZero(cellMatch);
                }
                return cellTotal > 0 ? cellMatched * 100.0 / cellTotal : 0;
            }

            for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
            {
                double cellCov = MeasureCell(r, c);
                cells.Add(new ColorCellResult
                {
                    Row      = r,
                    Col      = c,
                    Index    = r * cols + c,
                    Coverage = Math.Round(cellCov, 1),
                    Passed   = inRange(cellCov),
                });
            }
            return cells;
        }

        public void Annotate(Mat annotated, ColorCoverageDetection d, ColorCoverageInspection insp, VisionZone? zone, LabelStack labels)
        {
            int w = annotated.Width, h = annotated.Height;

            // Semi-transparent tint over matching pixels.
            using (var tint    = new Mat(annotated.Size(), annotated.Type(), VisionPalette.CoverageTint))
            using (var blended = new Mat())
            {
                Cv2.AddWeighted(annotated, TintSourceWeight, tint, TintColorWeight, 0, blended);
                blended.CopyTo(annotated, d.MatchMask);
            }

            if (zone != null)
                VisionDrawing.DrawZoneBorder(annotated, zone.Geometry, w, h, insp.Name, VisionPalette.ColorZoneBorder);

            if (zone != null && d.Grid is (int rows, int cols) && d.Result.Cells != null)
                DrawGridCells(annotated, zone.Geometry, rows, cols, d.Result.Cells, w, h);

            // Name label only — the coverage/pass values are reported as text in the app.
            labels.DrawStatusLabel(annotated, insp.Name, d.Result.Passed);
        }

        /// <summary>Outlines each grid cell green/red so the preview shows which ones failed.</summary>
        private static void DrawGridCells(
            Mat img, VisionZoneGeometry geom, int rows, int cols, List<ColorCellResult> cells, int w, int h)
        {
            bool tilted = ZoneGeometry.IsRotatedRect(geom);
            var bounds  = ZoneGeometry.ZoneBounds(geom, w, h);

            foreach (var cell in cells)
            {
                var color = cell.Passed ? VisionPalette.Pass : VisionPalette.Fail;
                Point label;

                if (tilted)
                {
                    var quad = ZoneGeometry.ToIntPoints(ZoneGeometry.CellQuad(geom, rows, cols, cell.Row, cell.Col, w, h));
                    Cv2.Polylines(img, new[] { quad }, true, color, 1);
                    // Anchor on the cell's own centre — on a tilt there is no reliable
                    // "top-left" corner, and a centred number stays inside its cell.
                    label = new Point((int)quad.Average(p => p.X) - 8, (int)quad.Average(p => p.Y) + 4);
                }
                else
                {
                    var rect = ZoneGeometry.CellRect(bounds, rows, cols, cell.Row, cell.Col);
                    Cv2.Rectangle(img, rect, color, 1);
                    label = new Point(rect.X + 3, rect.Y + 13);
                }

                Cv2.PutText(img, $"{cell.Coverage:0}", label,
                    VisionDrawing.Font, VisionDrawing.CellFontScale, VisionPalette.GridCellText, 1);
            }
        }
    }
}
