using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using ZXing;

namespace Controller.RobotControl.Vision.Inspections
{
    using Point = OpenCvSharp.Point;

    internal sealed class BarcodeDetection
    {
        public required BarcodeResult Result   { get; init; }
        /// <summary>Full-frame pixel outline of each decoded code that reported ≥2 result points.</summary>
        public required List<Point[]> Outlines { get; init; }
    }

    /// <summary>
    /// ZXing multi-code decode over the zone's bounding box (the whole frame without a zone).
    /// The requested format names are parsed when the strategy is built for a program, not
    /// once per frame.
    /// </summary>
    internal sealed class BarcodeStrategy : IInspectionStrategy<BarcodeInspection, BarcodeDetection>
    {
        // Keyed by instance: a program update builds a new strategy for the new inspections.
        private readonly Dictionary<BarcodeInspection, List<BarcodeFormat>?> _formats =
            new(ReferenceEqualityComparer.Instance);

        public BarcodeStrategy(IEnumerable<BarcodeInspection> inspections)
        {
            foreach (var insp in inspections)
                _formats[insp] = ParseFormats(insp.Formats);
        }

        /// <summary>
        /// ZXing formats for a list of names; null (all formats) when the list is empty.
        /// Unrecognised names are dropped — a non-empty list of only unknown names yields an
        /// empty list, exactly as the per-frame parse used to.
        /// </summary>
        internal static List<BarcodeFormat>? ParseFormats(IReadOnlyCollection<string> names)
        {
            if (names.Count == 0) return null;
            var formats = new List<BarcodeFormat>(names.Count);
            foreach (var name in names)
                if (Enum.TryParse<BarcodeFormat>(name, out var f)) formats.Add(f);
            return formats;
        }

        public BarcodeDetection Run(FrameContext ctx, BarcodeInspection insp, VisionZone? zone)
        {
            int w = ctx.Width, h = ctx.Height;

            // ZoneBounds handles every shape (including a tilted rectangle's axis-aligned box)
            // and clamps to the frame, so the crop can never be negative or out of bounds.
            var roiRect = zone != null ? ZoneGeometry.ZoneBounds(zone.Geometry, w, h) : new Rect(0, 0, w, h);
            bool full   = roiRect.X == 0 && roiRect.Y == 0 && roiRect.Width == w && roiRect.Height == h;

            // Grayscale of a crop is the crop of the grayscale, so the shared gray frame is cut
            // rather than converting a colour crop. A crop is cloned to get a contiguous buffer.
            using var crop = full ? null : CloneRegion(ctx.Gray, roiRect);
            var gray = crop ?? ctx.Gray;

            var pixelBytes = new byte[gray.Width * gray.Height];
            Marshal.Copy(gray.Data, pixelBytes, 0, pixelBytes.Length);
            var luminance = new RGBLuminanceSource(pixelBytes, gray.Width, gray.Height,
                RGBLuminanceSource.BitmapFormat.Gray8);

            var reader = new BarcodeReaderGeneric { Options = { TryHarder = true, TryInverted = true } };
            var formats = _formats.TryGetValue(insp, out var cached) ? cached : ParseFormats(insp.Formats);
            if (formats != null)
                reader.Options.PossibleFormats = formats;

            var rawResults = reader.DecodeMultiple(luminance) ?? Array.Empty<ZXing.Result>();

            var codes    = new List<BarcodeCodeResult>();
            var outlines = new List<Point[]>();
            int roiX = roiRect.X, roiY = roiRect.Y;

            foreach (var r in rawResults)
            {
                if (r?.Text == null) continue;

                // Map result point centers back to full-image coordinates.
                double cx, cy;
                if (r.ResultPoints is { Length: > 0 })
                {
                    cx = roiX + r.ResultPoints.Where(p => p != null).Average(p => p.X);
                    cy = roiY + r.ResultPoints.Where(p => p != null).Average(p => p.Y);
                }
                else
                {
                    cx = roiX + roiRect.Width  / 2.0;
                    cy = roiY + roiRect.Height / 2.0;
                }

                codes.Add(new BarcodeCodeResult
                {
                    Value   = r.Text,
                    Format  = r.BarcodeFormat.ToString(),
                    CenterX = Math.Round(cx / w, 4),
                    CenterY = Math.Round(cy / h, 4),
                });

                if (r.ResultPoints is { Length: >= 2 })
                {
                    outlines.Add(r.ResultPoints
                        .Where(p => p != null)
                        .Select(p => new Point((int)(p.X + roiX), (int)(p.Y + roiY)))
                        .ToArray());
                }
            }

            return new BarcodeDetection
            {
                Outlines = outlines,
                Result   = new BarcodeResult
                {
                    InspectionId = insp.Id,
                    Name         = insp.Name,
                    Count        = codes.Count,
                    Found        = codes.Count > 0,
                    Codes        = codes,
                },
            };
        }

        private static Mat CloneRegion(Mat src, Rect region)
        {
            using var view = new Mat(src, region);
            return view.Clone();
        }

        public void Annotate(Mat annotated, BarcodeDetection d, BarcodeInspection insp, VisionZone? zone, LabelStack labels)
        {
            foreach (var pts in d.Outlines)
                Cv2.Polylines(annotated, new[] { pts }, isClosed: pts.Length > 2, VisionPalette.Barcode, VisionDrawing.OverlayThickness);
            labels.DrawStatusLabel(annotated, insp.Name, d.Result.Found);
        }
    }
}
