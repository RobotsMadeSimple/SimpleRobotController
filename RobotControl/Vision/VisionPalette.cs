using OpenCvSharp;

namespace Controller.RobotControl.Vision
{
    /// <summary>
    /// Every colour the vision overlays draw with, by role. OpenCV scalars are BGR, so the
    /// component order below is blue, green, red.
    /// </summary>
    internal static class VisionPalette
    {
        // ── Status ──────────────────────────────────────────────────────────────
        public static readonly Scalar Pass        = new(0, 220, 0);
        public static readonly Scalar Fail        = new(0, 0, 220);
        public static readonly Scalar TextOutline = new(0, 0, 0);

        // ── Zones ───────────────────────────────────────────────────────────────
        /// <summary>Cyan: a zone used by at least one enabled inspection.</summary>
        public static readonly Scalar ZoneBorder      = new(255, 255, 0);
        /// <summary>Magenta: the zone a color coverage inspection measured.</summary>
        public static readonly Scalar ColorZoneBorder = new(255, 0, 255);
        public static readonly Scalar GridCellText    = new(255, 255, 255);

        // ── Per-inspection overlays ─────────────────────────────────────────────
        public static readonly Scalar Blob           = new(0, 255, 0);
        /// <summary>Tint laid over pixels matched by a color coverage inspection.</summary>
        public static readonly Scalar CoverageTint   = new(0, 200, 60);
        /// <summary>Orange outline of a matched polygon.</summary>
        public static readonly Scalar Polygon        = new(0, 165, 255);
        /// <summary>Yellow orientation arrow on a matched polygon.</summary>
        public static readonly Scalar PolygonArrow   = new(0, 255, 255);
        /// <summary>Spring green.</summary>
        public static readonly Scalar ArucoMarker    = new(0, 255, 127);
        /// <summary>Violet-ish. Stored BGR (168, 85, 247) — kept as it has always rendered.</summary>
        public static readonly Scalar LineSegment    = new(168, 85, 247);
        /// <summary>Dodger blue.</summary>
        public static readonly Scalar Barcode        = new(30, 144, 255);

        // ── Debug frames ────────────────────────────────────────────────────────
        public static readonly Scalar DebugMatched   = new(0, 210, 0);
        public static readonly Scalar DebugRejected  = new(0, 140, 255);
        public static readonly Scalar DebugIgnored   = new(80, 80, 80);
        public static readonly Scalar DebugStripBack = new(18, 18, 18);
        public static readonly Scalar DebugStripText = new(180, 180, 180);
    }
}
