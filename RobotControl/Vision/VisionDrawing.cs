using OpenCvSharp;

namespace Controller.RobotControl.Vision
{
    using Point = OpenCvSharp.Point;

    /// <summary>Shared overlay drawing constants and encoding for the annotated and debug frames.</summary>
    internal static class VisionDrawing
    {
        public const HersheyFonts Font = HersheyFonts.HersheySimplex;

        /// <summary>Status labels stacked down the left edge.</summary>
        public const double StatusFontScale = 0.5;
        /// <summary>Per-cell values on a gridded tool.</summary>
        public const double CellFontScale   = 0.35;

        /// <summary>Thickness of the dark stroke under outlined text.</summary>
        public const int TextOutlineThickness = 3;
        public const int OverlayThickness     = 2;
        public const int LineHeight           = 22;
        public const int LeftMargin           = 6;

        public const int AnnotatedJpegQuality = 80;
        public const int DebugJpegQuality     = 88;

        public static byte[] EncodeJpeg(Mat img, int quality)
        {
            Cv2.ImEncode(".jpg", img, out var buf, new[] { (int)ImwriteFlags.JpegQuality, quality });
            return buf;
        }

        /// <summary>Text drawn over a thicker dark copy of itself, so it reads on any background.</summary>
        public static void DrawOutlinedText(Mat img, string text, Point at, double scale, Scalar color)
        {
            Cv2.PutText(img, text, at, Font, scale, VisionPalette.TextOutline, TextOutlineThickness);
            Cv2.PutText(img, text, at, Font, scale, color, 1);
        }
    }
}
