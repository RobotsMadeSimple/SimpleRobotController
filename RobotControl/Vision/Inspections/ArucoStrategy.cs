using OpenCvSharp;
using OpenCvSharp.Aruco;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision.Inspections
{
    using Point = OpenCvSharp.Point;

    internal sealed class ArucoDetection
    {
        public required ArucoResult   Result   { get; init; }
        /// <summary>Pixel corners of each kept marker, parallel to <see cref="ArucoResult.Markers"/>.</summary>
        public required List<Point[]> Outlines { get; init; }
    }

    /// <summary>
    /// ArUco marker detection in one predefined dictionary, or in every dictionary when the
    /// inspection's DictionaryId is <see cref="AllDictionaries"/>. Owns a cache of detectors,
    /// which are expensive native objects: one per dictionary, built on first use and kept
    /// until <see cref="ReleaseDetectors"/>. Stays usable afterwards (detectors rebuild lazily).
    /// </summary>
    internal sealed class ArucoStrategy : IInspectionStrategy<ArucoInspection, ArucoDetection>
    {
        /// <summary>The DictionaryId value meaning "search every predefined dictionary".</summary>
        public const int AllDictionaries = -1;

        /// <summary>The dictionaries scanned when DictionaryId is -1 ("all").</summary>
        // "All" means the 17 classic ArUco dictionaries (ids 0-16), as before the
        // strategy refactor. The AprilTag families (17+) are excluded on purpose:
        // 16h5 in particular produces false detections on ordinary scenes.
        internal static readonly PredefinedDictionaryType[] AllDictionaryTypes =
            Enum.GetValues<PredefinedDictionaryType>().Distinct().Where(t => (int)t <= 16).ToArray();

        private readonly Dictionary<PredefinedDictionaryType, (Dictionary Dict, ArucoDetector Detector)> _detectors = new();
        private readonly object _lock = new();

        private ArucoDetector GetDetector(PredefinedDictionaryType type)
        {
            lock (_lock)
            {
                if (_detectors.TryGetValue(type, out var cached))
                    return cached.Detector;

                var dict     = CvAruco.GetPredefinedDictionary(type);
                var detector = new ArucoDetector(dict, new DetectorParameters(), new RefineParameters());
                _detectors[type] = (dict, detector);
                return detector;
            }
        }

        public ArucoDetection Run(FrameContext ctx, ArucoInspection insp, VisionZone? zone)
        {
            int w = ctx.Width, h = ctx.Height;

            var markers     = new List<ArucoMarkerResult>();
            var outlines    = new List<Point[]>();
            var seenCenters = new HashSet<(int x, int y)>();

            IEnumerable<PredefinedDictionaryType> dictTypes = insp.DictionaryId == AllDictionaries
                ? AllDictionaryTypes
                : new[] { (PredefinedDictionaryType)insp.DictionaryId };

            foreach (var type in dictTypes)
            {
                // ArucoDetector converts a BGR input to exactly this grayscale internally, so
                // the shared one saves a conversion per dictionary without changing the result.
                GetDetector(type).DetectMarkers(ctx.Gray, out var corners, out var ids, out _);
                if (ids == null || ids.Length == 0) continue;

                for (int i = 0; i < ids.Length; i++)
                {
                    var c = corners[i];

                    float minX = c.Min(p => p.X), maxX = c.Max(p => p.X);
                    float minY = c.Min(p => p.Y), maxY = c.Max(p => p.Y);
                    float area = (maxX - minX) * (maxY - minY);
                    if (area < insp.MinMarkerArea || area > insp.MaxMarkerArea) continue;

                    float cx = c.Average(p => p.X);
                    float cy = c.Average(p => p.Y);
                    if (zone != null && !ZoneGeometry.IsInsideZone(zone.Geometry, cx, cy, w, h)) continue;

                    // Deduplicate by pixel center so a marker found in several dictionaries counts once.
                    if (!seenCenters.Add(((int)cx, (int)cy))) continue;

                    markers.Add(new ArucoMarkerResult
                    {
                        MarkerId = ids[i],
                        CenterX  = Math.Round(cx / w, 4),
                        CenterY  = Math.Round(cy / h, 4),
                    });
                    outlines.Add(c.Select(p => new Point((int)p.X, (int)p.Y)).ToArray());
                }
            }

            return new ArucoDetection
            {
                Outlines = outlines,
                Result   = new ArucoResult
                {
                    InspectionId = insp.Id,
                    Name         = insp.Name,
                    Count        = markers.Count,
                    Found        = markers.Count > 0,
                    Markers      = markers,
                },
            };
        }

        public void Annotate(Mat annotated, ArucoDetection d, ArucoInspection insp, VisionZone? zone, LabelStack labels)
        {
            foreach (var pts in d.Outlines)
                Cv2.Polylines(annotated, new[] { pts }, isClosed: true, VisionPalette.ArucoMarker, VisionDrawing.OverlayThickness);
            labels.DrawStatusLabel(annotated, insp.Name, d.Result.Found);
        }

        /// <summary>Releases every cached detector. Only call once no frame is being processed.</summary>
        public void ReleaseDetectors()
        {
            lock (_lock)
            {
                foreach (var entry in _detectors.Values)
                {
                    entry.Detector.Dispose();
                    entry.Dict.Dispose();
                }
                _detectors.Clear();
            }
        }
    }
}
