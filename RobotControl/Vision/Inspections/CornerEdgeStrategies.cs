using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision.Inspections
{
    using Point = OpenCvSharp.Point;

    internal sealed class CornerDetection
    {
        public required CornerResult Result { get; init; }
        public required Point2f[] Points { get; init; }
    }

    /// <summary>Corner (feature point) detection in the zone: Shi-Tomasi, or Harris.</summary>
    internal sealed class CornerStrategy : IInspectionStrategy<CornerInspection, CornerDetection>
    {
        public CornerDetection Run(FrameContext ctx, CornerInspection insp, VisionZone? zone)
        {
            int w = ctx.Width, h = ctx.Height;
            bool harris = string.Equals(insp.Method, "harris", StringComparison.OrdinalIgnoreCase);
            // GoodFeaturesToTrack returns the strongest first, which is the order we report.
            var pts = Cv2.GoodFeaturesToTrack(ctx.Gray, Math.Max(1, insp.MaxCorners), Math.Max(1e-4, insp.Quality),
                Math.Max(1, insp.MinDistance), ctx.ZoneMask(zone), Math.Max(2, insp.BlockSize), harris, 0.04);
            if (zone != null)
                pts = pts.Where(p => ZoneGeometry.IsInsideZone(zone.Geometry, p.X, p.Y, w, h)).ToArray();
            return new CornerDetection
            {
                Points = pts,
                Result = new CornerResult
                {
                    InspectionId = insp.Id,
                    Name         = insp.Name,
                    Count        = pts.Length,
                    Found        = pts.Length >= Math.Max(1, insp.MinCount),
                    Points       = pts.Select(p => new[] { Math.Round(p.X / (double)w, 4), Math.Round(p.Y / (double)h, 4) }).ToList(),
                },
            };
        }

        public void Annotate(Mat annotated, CornerDetection d, CornerInspection insp, VisionZone? zone, LabelStack labels)
        {
            if (zone != null)
                VisionDrawing.DrawZoneBorder(annotated, zone.Geometry, annotated.Width, annotated.Height, insp.Name, VisionPalette.ZoneBorder);
            foreach (var p in d.Points)
                Cv2.Circle(annotated, new Point((int)p.X, (int)p.Y), 4, VisionPalette.Corner, 2);
            labels.DrawStatusLabel(annotated, insp.Name, d.Result.Found);
        }
    }

    internal sealed class EdgeDetection : IDisposable
    {
        public required EdgeResult Result { get; init; }
        public required Mat Edges { get; init; }
        public LineSegmentPoint? Dominant { get; init; }
        public void Dispose() => Edges.Dispose();
    }

    /// <summary>Canny edges in the zone: edge density (% of zone pixels) and the dominant straight edge.</summary>
    internal sealed class EdgeStrategy : IInspectionStrategy<EdgeInspection, EdgeDetection>
    {
        public EdgeDetection Run(FrameContext ctx, EdgeInspection insp, VisionZone? zone)
        {
            int w = ctx.Width, h = ctx.Height;
            var mask  = ctx.ZoneMask(zone);
            var edges = new Mat();
            try
            {
                Cv2.BitwiseAnd(ctx.Canny(insp.CannyLow, insp.CannyHigh), mask, edges);
                int total = Cv2.CountNonZero(mask);
                double density = total > 0 ? Cv2.CountNonZero(edges) * 100.0 / total : 0;

                var bounds = zone != null ? ZoneGeometry.ZoneBounds(zone.Geometry, w, h) : new Rect(0, 0, w, h);
                double minLen = Math.Max(5, Math.Min(bounds.Width, bounds.Height) * 0.1);
                var segs = Cv2.HoughLinesP(edges, 1, Math.PI / 180, 30, minLen, 5);
                LineSegmentPoint? dominant = segs.Length == 0 ? null
                    : segs.OrderByDescending(s => Math.Pow(s.P2.X - s.P1.X, 2) + Math.Pow(s.P2.Y - s.P1.Y, 2)).First();

                bool passed = (!insp.MinDensity.HasValue || density >= insp.MinDensity.Value)
                           && (!insp.MaxDensity.HasValue || density <= insp.MaxDensity.Value);
                return new EdgeDetection
                {
                    Edges    = edges,
                    Dominant = dominant,
                    Result   = new EdgeResult
                    {
                        InspectionId  = insp.Id,
                        Name          = insp.Name,
                        Density       = Math.Round(density, 1),
                        Passed        = passed,
                        DominantAngle = dominant is { } d ? Math.Round(LineStrategy.SegmentAngle(d), 1) : null,
                    },
                };
            }
            catch
            {
                edges.Dispose();
                throw;
            }
        }

        public void Annotate(Mat annotated, EdgeDetection d, EdgeInspection insp, VisionZone? zone, LabelStack labels)
        {
            if (zone != null)
                VisionDrawing.DrawZoneBorder(annotated, zone.Geometry, annotated.Width, annotated.Height, insp.Name, VisionPalette.ZoneBorder);
            annotated.SetTo(VisionPalette.Edge, d.Edges);
            if (d.Dominant is { } s)
                Cv2.Line(annotated, s.P1, s.P2, VisionPalette.Frame, VisionDrawing.OverlayThickness + 1);
            labels.DrawStatusLabel(annotated, insp.Name, d.Result.Passed);
        }
    }
}
