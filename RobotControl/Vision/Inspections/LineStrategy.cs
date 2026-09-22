using OpenCvSharp;
using System;
using System.Collections.Generic;

namespace Controller.RobotControl.Vision.Inspections
{
    using Point = OpenCvSharp.Point;

    internal sealed class LineDetection
    {
        public required LineResult Result { get; init; }
        /// <summary>Pixel endpoints of each kept segment, parallel to <see cref="LineResult.Lines"/>.</summary>
        public required List<LineSegmentPoint> Segments { get; init; }
    }

    /// <summary>
    /// Canny edges (masked to the zone) → probabilistic Hough → optional angle filter. The
    /// pipeline lives in <see cref="Detect"/>, shared by <see cref="Run"/> and
    /// <see cref="RenderDebug"/>.
    /// </summary>
    internal sealed class LineStrategy : IInspectionStrategy<LineInspection, LineDetection>
    {
        private const double HoughRho   = 1;
        private const double HoughTheta = Math.PI / 180;
        private const double DebugLabelFontScale = 0.4;

        /// <summary>Edge image fed to Hough, and the raw segments it found.</summary>
        private sealed class Stage : IDisposable
        {
            private readonly Mat? _owned;
            public Stage(Mat edges, Mat? owned, LineSegmentPoint[] segments)
            {
                Edges = edges; _owned = owned; Segments = segments;
            }
            public Mat                Edges    { get; }
            public LineSegmentPoint[] Segments { get; }
            public void Dispose() => _owned?.Dispose();
        }

        private static Stage Detect(FrameContext ctx, LineInspection insp, VisionZone? zone)
        {
            // The cached Canny output is shared, so masking writes into a new Mat.
            var  edges  = ctx.Canny(insp.CannyThreshold1, insp.CannyThreshold2);
            Mat? masked = null;
            if (zone != null)
            {
                masked = new Mat();
                Cv2.BitwiseAnd(edges, ctx.ZoneMask(zone), masked);
                edges = masked;
            }

            try
            {
                var segments = Cv2.HoughLinesP(edges, HoughRho, HoughTheta,
                    insp.HoughThreshold, insp.MinLineLength, insp.MaxLineGap);
                return new Stage(edges, masked, segments);
            }
            catch
            {
                masked?.Dispose();
                throw;
            }
        }

        /// <summary>Undirected angle in degrees, 0–180: 0 = horizontal, 90 = vertical.</summary>
        internal static double SegmentAngle(LineSegmentPoint seg)
        {
            double dx = seg.P2.X - seg.P1.X, dy = seg.P2.Y - seg.P1.Y;
            double deg = Math.Atan2(dy, dx) * 180.0 / Math.PI;
            return ((deg % 180) + 180) % 180;
        }

        private static bool PassesAngle(LineInspection insp, double angleDeg) =>
            !insp.FilterByAngle || (angleDeg >= insp.MinAngle && angleDeg <= insp.MaxAngle);

        public LineDetection Run(FrameContext ctx, LineInspection insp, VisionZone? zone)
        {
            int w = ctx.Width, h = ctx.Height;
            using var stage = Detect(ctx, insp, zone);

            var lines = new List<LineSegment>();
            var kept  = new List<LineSegmentPoint>();

            foreach (var seg in stage.Segments)
            {
                double angleDeg = SegmentAngle(seg);
                if (!PassesAngle(insp, angleDeg)) continue;

                float mx = (seg.P1.X + seg.P2.X) / 2f, my = (seg.P1.Y + seg.P2.Y) / 2f;
                if (zone != null && !ZoneGeometry.IsInsideZone(zone.Geometry, mx, my, w, h)) continue;

                double dx = seg.P2.X - seg.P1.X, dy = seg.P2.Y - seg.P1.Y;
                lines.Add(new LineSegment
                {
                    X1     = Math.Round(seg.P1.X / (double)w, 4),
                    Y1     = Math.Round(seg.P1.Y / (double)h, 4),
                    X2     = Math.Round(seg.P2.X / (double)w, 4),
                    Y2     = Math.Round(seg.P2.Y / (double)h, 4),
                    Angle  = Math.Round(angleDeg, 2),
                    Length = Math.Round(Math.Sqrt(dx * dx + dy * dy), 1),
                });
                kept.Add(seg);
            }

            return new LineDetection
            {
                Segments = kept,
                Result   = new LineResult
                {
                    InspectionId = insp.Id,
                    Name         = insp.Name,
                    Count        = lines.Count,
                    Found        = lines.Count > 0,
                    Lines        = lines,
                },
            };
        }

        public void Annotate(Mat annotated, LineDetection d, LineInspection insp, VisionZone? zone, LabelStack labels)
        {
            foreach (var seg in d.Segments)
                Cv2.Line(annotated, seg.P1, seg.P2, VisionPalette.LineSegment, VisionDrawing.OverlayThickness);
            labels.DrawStatusLabel(annotated, insp.Name, d.Result.Found);
        }

        /// <summary>The masked edge image with every Hough segment: green passed the angle filter, orange did not.</summary>
        public byte[]? RenderDebug(FrameContext ctx, LineInspection insp, VisionZone? zone)
        {
            using var stage = Detect(ctx, insp, zone);

            using var debug = new Mat();
            Cv2.CvtColor(stage.Edges, debug, ColorConversionCodes.GRAY2BGR);

            int matched = 0;
            foreach (var seg in stage.Segments)
            {
                double angleDeg = SegmentAngle(seg);
                bool   pass     = PassesAngle(insp, angleDeg);
                var    color    = pass ? VisionPalette.DebugMatched : VisionPalette.DebugRejected;

                Cv2.Line(debug, seg.P1, seg.P2, color, VisionDrawing.OverlayThickness);

                int mx = (seg.P1.X + seg.P2.X) / 2, my = (seg.P1.Y + seg.P2.Y) / 2;
                VisionDrawing.DrawOutlinedText(debug, $"{angleDeg:F0}°", new Point(mx + 4, my), DebugLabelFontScale, color);

                if (pass) matched++;
            }

            string angleDesc = insp.FilterByAngle ? $"   Angle {insp.MinAngle:F0}°–{insp.MaxAngle:F0}°" : "";
            VisionDrawing.DrawInfoStrip(debug, new[]
            {
                $"Canny {insp.CannyThreshold1}/{insp.CannyThreshold2}   Threshold {insp.HoughThreshold}   MinLen {insp.MinLineLength}   MaxGap {insp.MaxLineGap}{angleDesc}",
                $"Segments: {stage.Segments.Length}   Matched: {matched}",
                insp.FilterByAngle ? "Orange=angle filtered   Green=matched" : "Green=all matched segments",
            });

            return VisionDrawing.EncodeJpeg(debug, VisionDrawing.DebugJpegQuality);
        }
    }
}
