using OpenCvSharp;
using System;
using System.Collections.Generic;

namespace Controller.RobotControl.Vision.Inspections
{
    using Point = OpenCvSharp.Point;

    /// <summary>A matched polygon as drawn: its approximated outline and orientation arrow.</summary>
    internal readonly record struct PolygonShape(Point[] Outline, Point Center, Point ArrowTip);

    internal sealed class PolygonDetection
    {
        public required PolygonResult      Result { get; init; }
        public required List<PolygonShape> Shapes { get; init; }
    }

    /// <summary>
    /// Threshold → external contours → area filter → ApproxPolyDP side count. The pipeline
    /// lives in <see cref="Threshold"/> and <see cref="Analyze"/>, which both
    /// <see cref="Run"/> and <see cref="RenderDebug"/> use, so the debug view always shows
    /// exactly what detection sees.
    /// </summary>
    internal sealed class PolygonStrategy : IInspectionStrategy<PolygonInspection, PolygonDetection>
    {
        /// <summary>Orientation arrow length as a fraction of the square root of the contour area.</summary>
        private const double ArrowLengthFactor = 0.4;
        private const double DebugLabelFontScale = 0.5;

        /// <summary>One external contour and how far it got through the filters.</summary>
        private readonly record struct Candidate(Point[] Contour, double Area, bool AreaPass, Point[]? Approx, Moments? Moments);

        /// <summary>Binary mask of blurred-gray pixels in the threshold band (inverted if asked). Caller owns it.</summary>
        private static Mat Threshold(FrameContext ctx, PolygonInspection insp)
        {
            var thresh = new Mat();
            Cv2.InRange(ctx.Blurred, new Scalar(insp.MinThreshold), new Scalar(insp.MaxThreshold), thresh);
            if (insp.InvertThreshold) Cv2.BitwiseNot(thresh, thresh);
            return thresh;
        }

        private static List<Candidate> Analyze(Mat thresh, PolygonInspection insp)
        {
            Cv2.FindContours(thresh, out var contours, out _, RetrievalModes.External, ContourApproximationModes.ApproxSimple);

            var candidates = new List<Candidate>(contours.Length);
            foreach (var contour in contours)
            {
                double area = Cv2.ContourArea(contour);
                if (area < insp.MinArea || area > insp.MaxArea)
                {
                    candidates.Add(new Candidate(contour, area, false, null, null));
                    continue;
                }

                double peri   = Cv2.ArcLength(contour, true);
                var    approx = Cv2.ApproxPolyDP(contour, insp.Epsilon * peri, true);
                candidates.Add(new Candidate(contour, area, true, approx, Cv2.Moments(contour)));
            }
            return candidates;
        }

        public PolygonDetection Run(FrameContext ctx, PolygonInspection insp, VisionZone? zone)
        {
            int w = ctx.Width, h = ctx.Height;

            List<Candidate> candidates;
            using (var thresh = Threshold(ctx, insp))
                candidates = Analyze(thresh, insp);

            var    shapes   = new List<PolygonShape>();
            double angle    = 0, centerX = 0, centerY = 0, bestArea = 0;

            foreach (var cand in candidates)
            {
                if (!cand.AreaPass || cand.Approx!.Length != insp.Sides) continue;

                // Centroid from moments for the zone check.
                var m = cand.Moments!;
                if (m.M00 == 0) continue;
                float cx = (float)(m.M10 / m.M00);
                float cy = (float)(m.M01 / m.M00);

                if (zone != null && !ZoneGeometry.IsInsideZone(zone.Geometry, cx, cy, w, h)) continue;

                // Orientation arrow from MinAreaRect.
                var    rect = Cv2.MinAreaRect(cand.Approx);
                double rad  = rect.Angle * Math.PI / 180.0;
                double len  = Math.Sqrt(cand.Area) * ArrowLengthFactor;
                shapes.Add(new PolygonShape(
                    cand.Approx,
                    new Point((int)cx, (int)cy),
                    new Point((int)(cx + Math.Cos(rad) * len), (int)(cy + Math.Sin(rad) * len))));

                if (cand.Area > bestArea)
                {
                    bestArea = cand.Area;
                    angle    = Math.Round(rect.Angle, 2);
                    centerX  = Math.Round(cx / w, 4);
                    centerY  = Math.Round(cy / h, 4);
                }
            }

            return new PolygonDetection
            {
                Shapes = shapes,
                Result = new PolygonResult
                {
                    InspectionId = insp.Id,
                    Name         = insp.Name,
                    Count        = shapes.Count,
                    Found        = shapes.Count > 0,
                    Angle        = angle,
                    CenterX      = centerX,
                    CenterY      = centerY,
                },
            };
        }

        public void Annotate(Mat annotated, PolygonDetection d, PolygonInspection insp, VisionZone? zone, LabelStack labels)
        {
            foreach (var s in d.Shapes)
            {
                Cv2.Polylines(annotated, new[] { s.Outline }, true, VisionPalette.Polygon, VisionDrawing.OverlayThickness);
                Cv2.ArrowedLine(annotated, s.Center, s.ArrowTip, VisionPalette.PolygonArrow, VisionDrawing.OverlayThickness);
            }
            labels.DrawStatusLabel(annotated, insp.Name, d.Result.Found);
        }

        /// <summary>
        /// The threshold mask with every contour classified: gray failed the area filter,
        /// orange has the wrong side count, green matched. Ignores the zone, as it always has,
        /// so the whole frame's thresholding can be tuned.
        /// </summary>
        public byte[]? RenderDebug(FrameContext ctx, PolygonInspection insp, VisionZone? zone)
        {
            using var thresh = Threshold(ctx, insp);

            // Debug canvas: threshold mask in color so overlays are visible.
            using var debug = new Mat();
            Cv2.CvtColor(thresh, debug, ColorConversionCodes.GRAY2BGR);

            var candidates = Analyze(thresh, insp);
            int areaPass = 0, matchCount = 0;

            foreach (var cand in candidates)
            {
                if (!cand.AreaPass)
                {
                    Cv2.DrawContours(debug, new[] { cand.Contour }, -1, VisionPalette.DebugIgnored, 1);
                    continue;
                }

                areaPass++;
                var m  = cand.Moments!;
                int cx = m.M00 > 0 ? (int)(m.M10 / m.M00) : 0;
                int cy = m.M00 > 0 ? (int)(m.M01 / m.M00) : 0;
                var at = new Point(cx + 4, cy);

                if (cand.Approx!.Length != insp.Sides)
                {
                    Cv2.DrawContours(debug, new[] { cand.Approx }, -1, VisionPalette.DebugRejected, VisionDrawing.OverlayThickness);
                    VisionDrawing.DrawOutlinedText(debug, $"{cand.Approx.Length}s", at, DebugLabelFontScale, VisionPalette.DebugRejected);
                }
                else
                {
                    Cv2.DrawContours(debug, new[] { cand.Approx }, -1, VisionPalette.DebugMatched, VisionDrawing.OverlayThickness);
                    var rect = Cv2.MinAreaRect(cand.Approx);
                    VisionDrawing.DrawOutlinedText(debug, $"{rect.Angle:F1}deg", at, DebugLabelFontScale, VisionPalette.DebugMatched);
                    matchCount++;
                }
            }

            VisionDrawing.DrawInfoStrip(debug, new[]
            {
                $"Thresh {insp.MinThreshold}-{insp.MaxThreshold}{(insp.InvertThreshold ? " (inverted)" : "")}   Eps {insp.Epsilon}   Need {insp.Sides} sides",
                $"Contours: {candidates.Count}   Area pass: {areaPass}   Matched: {matchCount}",
                "Gray=area fail   Orange=wrong sides   Green=matched",
            });

            return VisionDrawing.EncodeJpeg(debug, VisionDrawing.DebugJpegQuality);
        }
    }
}
