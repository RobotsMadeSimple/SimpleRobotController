using OpenCvSharp;
using System;
using System.Collections.Generic;

namespace Controller.RobotControl.Vision.Inspections
{
    using Point = OpenCvSharp.Point;

    /// <summary>SimpleBlobDetector keypoints, kept when their center lies inside the zone.</summary>
    internal sealed class BlobStrategy : IInspectionStrategy<BlobInspection, InspectionResult>
    {
        private const int CenterDotRadius = 3;

        public InspectionResult Run(FrameContext ctx, BlobInspection insp, VisionZone? zone)
        {
            var result = new InspectionResult
            {
                InspectionId = insp.Id,
                Name         = insp.Name,
                Blobs        = new List<BlobResult>(),
            };

            // SimpleBlobDetector converts a BGR input to exactly this grayscale internally,
            // so handing it the shared one skips a conversion without changing the result.
            foreach (var kp in DetectBlobs(ctx.Gray, insp.BlobParams))
            {
                float bx = kp.Pt.X, by = kp.Pt.Y;
                if (zone != null && !ZoneGeometry.IsInsideZone(zone.Geometry, bx, by, ctx.Width, ctx.Height))
                    continue;
                result.Blobs.Add(new BlobResult { X = bx, Y = by, Size = kp.Size });
            }
            return result;
        }

        public void Annotate(Mat annotated, InspectionResult result, BlobInspection insp, VisionZone? zone, LabelStack labels)
        {
            // Blob inspections have never had a status label; their circles are the feedback.
            foreach (var b in result.Blobs)
            {
                var pt = new Point((int)b.X, (int)b.Y);
                Cv2.Circle(annotated, pt, Math.Max(2, (int)(b.Size / 2)), VisionPalette.Blob, VisionDrawing.OverlayThickness);
                Cv2.Circle(annotated, pt, CenterDotRadius, VisionPalette.Blob, -1);
            }
        }

        /// <summary>The (empty) result recorded when the inspection throws, as it always has been.</summary>
        public static InspectionResult Empty(BlobInspection insp) => new()
        {
            InspectionId = insp.Id,
            Name         = insp.Name,
            Blobs        = new List<BlobResult>(),
        };

        private static KeyPoint[] DetectBlobs(Mat image, BlobDetectionParams p)
        {
            using var detector = SimpleBlobDetector.Create(new SimpleBlobDetector.Params
            {
                FilterByArea         = true,
                MinArea              = p.MinArea,
                MaxArea              = p.MaxArea,
                FilterByCircularity  = p.FilterByCircularity,
                MinCircularity       = p.MinCircularity,
                FilterByConvexity    = p.FilterByConvexity,
                MinConvexity         = p.MinConvexity,
                FilterByInertia      = p.FilterByInertia,
                MinInertiaRatio      = p.MinInertiaRatio,
                MinThreshold         = p.MinThreshold,
                MaxThreshold         = p.MaxThreshold,
                FilterByColor        = p.FilterByColor,
                BlobColor            = (byte)Math.Clamp(p.BlobColor, 0, 255),
            });

            return detector.Detect(image);
        }
    }
}
