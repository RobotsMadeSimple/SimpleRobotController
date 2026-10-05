using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Controller.RobotControl.Vision
{
    using Point = OpenCvSharp.Point;

    /// <summary>Shared overlay drawing and encoding for the annotated and debug frames.</summary>
    internal static class VisionDrawing
    {
        public const HersheyFonts Font = HersheyFonts.HersheySimplex;

        /// <summary>Status labels stacked down the left edge.</summary>
        public const double StatusFontScale    = 0.5;
        /// <summary>Zone names written beside their outline.</summary>
        public const double ZoneFontScale      = 0.45;
        /// <summary>Per-cell coverage numbers on a gridded zone.</summary>
        public const double CellFontScale      = 0.35;
        /// <summary>The parameter summary strip along the bottom of a debug frame.</summary>
        public const double InfoStripFontScale = 0.42;

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

        /// <summary>Outlines a zone in <paramref name="color"/> and writes its label beside it.</summary>
        public static void DrawZoneBorder(Mat img, VisionZoneGeometry geom, int w, int h, string label, Scalar color)
        {
            if (ZoneGeometry.IsRotatedRect(geom))
            {
                // The label is anchored to whichever corner sits highest so it does not end
                // up written across the tilted shape.
                var pts = ZoneGeometry.ToIntPoints(ZoneGeometry.RectCorners(geom, w, h));
                Cv2.Polylines(img, new[] { pts }, true, color, OverlayThickness);
                var top = pts.OrderBy(p => p.Y).First();
                PutZoneLabel(img, label, new Point(top.X + 4, Math.Max(12, top.Y - 6)), color);
                return;
            }

            switch (geom.Shape)
            {
                case VisionZoneShape.Rectangle:
                {
                    var tl = new Point((int)(geom.X * w), (int)(geom.Y * h));
                    var br = new Point((int)((geom.X + geom.Width) * w), (int)((geom.Y + geom.Height) * h));
                    Cv2.Rectangle(img, tl, br, color, OverlayThickness);
                    PutZoneLabel(img, label, new Point(tl.X + 4, tl.Y - 6), color);
                    break;
                }
                case VisionZoneShape.Circle:
                {
                    var center = new Point((int)(geom.Cx * w), (int)(geom.Cy * h));
                    int radius = (int)(geom.Radius * Math.Min(w, h));
                    Cv2.Circle(img, center, radius, color, OverlayThickness);
                    PutZoneLabel(img, label, new Point(center.X + 4, center.Y - radius - 4), color);
                    break;
                }
                case VisionZoneShape.Polygon:
                {
                    if (geom.Points.Count >= 2)
                    {
                        var pts = geom.Points.Select(p => new Point((int)(p[0] * w), (int)(p[1] * h))).ToArray();
                        Cv2.Polylines(img, new[] { pts }, true, color, OverlayThickness);
                        PutZoneLabel(img, label, new Point(pts[0].X + 4, pts[0].Y - 6), color);
                    }
                    break;
                }
            }
        }

        private static void PutZoneLabel(Mat img, string label, Point at, Scalar color) =>
            Cv2.PutText(img, label, at, Font, ZoneFontScale, color, 1);

        /// <summary>Dark strip along the bottom of a debug frame listing the parameters and counts.</summary>
        public static void DrawInfoStrip(Mat img, IReadOnlyList<string> lines)
        {
            int w = img.Width, h = img.Height;
            int stripH = lines.Count * LineHeight + 8;
            int stripY = Math.Max(0, h - stripH);
            using (var stripRoi = img[new Rect(0, stripY, w, h - stripY)])
                stripRoi.SetTo(VisionPalette.DebugStripBack);

            int y = stripY + 18;
            foreach (var line in lines)
            {
                Cv2.PutText(img, line, new Point(LeftMargin, y), Font, InfoStripFontScale, VisionPalette.DebugStripText, 1);
                y += LineHeight;
            }
        }
    }
}
