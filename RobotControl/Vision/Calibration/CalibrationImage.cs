using OpenCvSharp;

namespace Controller.RobotControl.Vision.Calibration
{
    using Point = OpenCvSharp.Point;

    /// <summary>The wizard's annotated frame: every dot circled and numbered, taught dots highlighted, and the sheet axes at dot (0, 0).</summary>
    internal static class CalibrationImage
    {
        private static readonly Scalar DotColor    = VisionPalette.Blob;
        private static readonly Scalar TaughtColor = VisionPalette.Polygon;
        private static readonly Scalar AxisIColor  = VisionPalette.PolygonArrow;
        private static readonly Scalar AxisJColor  = VisionPalette.ZoneBorder;

        public static byte[]? Render(byte[] frameJpeg, DotGridResult grid, IReadOnlyCollection<int> taughtIndices)
        {
            using var img = Cv2.ImDecode(frameJpeg, ImreadModes.Color);
            if (img.Empty()) return null;
            Draw(img, grid, taughtIndices);
            return VisionDrawing.EncodeJpeg(img, VisionDrawing.DebugJpegQuality);
        }

        public static void Draw(Mat img, DotGridResult grid, IReadOnlyCollection<int> taughtIndices)
        {
            double spacing = grid.MedianSpacingPx > 0 ? grid.MedianSpacingPx : 40;
            double fontScale = Math.Clamp(spacing / 90.0, 0.3, 0.8);
            int thickness = spacing > 60 ? 2 : 1;

            // Sheet axes: arrows from lattice (0, 0) along +i and +j, drawn first so the dots sit on top.
            var h = grid.LatticeToPixel;
            var o  = ToPoint(Homography.Apply(h, 0, 0));
            var pi = ToPoint(Homography.Apply(h, 1.6, 0));
            var pj = ToPoint(Homography.Apply(h, 0, 1.6));
            Cv2.ArrowedLine(img, o, pi, AxisIColor, thickness + 1, LineTypes.AntiAlias, 0, 0.2);
            Cv2.ArrowedLine(img, o, pj, AxisJColor, thickness + 1, LineTypes.AntiAlias, 0, 0.2);
            VisionDrawing.DrawOutlinedText(img, "i", new Point(pi.X + 4, pi.Y - 4), fontScale, AxisIColor);
            VisionDrawing.DrawOutlinedText(img, "j", new Point(pj.X + 4, pj.Y + 12), fontScale, AxisJColor);

            foreach (var d in grid.Dots)
            {
                bool taught = taughtIndices.Contains(d.Index);
                var color = taught ? TaughtColor : DotColor;
                var c = new Point((int)Math.Round(d.X), (int)Math.Round(d.Y));
                int r = Math.Max(4, (int)Math.Round(Math.Sqrt(d.AreaPx / Math.PI)) + 3);
                Cv2.Circle(img, c, r, color, taught ? thickness + 1 : thickness, LineTypes.AntiAlias);
                Cv2.Circle(img, c, 2, color, -1);
                VisionDrawing.DrawOutlinedText(img, d.Index.ToString(System.Globalization.CultureInfo.InvariantCulture),
                    new Point(c.X + r / 2 + 2, c.Y - r / 2 - 2), fontScale, color);
            }

            VisionDrawing.DrawOutlinedText(img,
                $"{grid.Dots.Count} dots  {grid.Rows}x{grid.Cols}  rms {grid.RmsPx:0.00}px",
                new Point(VisionDrawing.LeftMargin, LabelStack.TopY), VisionDrawing.StatusFontScale, VisionPalette.DebugStripText);
        }

        private static Point ToPoint((double X, double Y) p) =>
            double.IsFinite(p.X) && double.IsFinite(p.Y)
                ? new Point((int)Math.Round(p.X), (int)Math.Round(p.Y))
                : new Point(0, 0);
    }
}
