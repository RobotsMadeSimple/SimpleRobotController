using OpenCvSharp;
using System;

namespace Controller.RobotControl.Vision
{
    using Point = OpenCvSharp.Point;

    /// <summary>
    /// Pass/fail name labels stacked down the left edge of the annotated frame, one per
    /// inspection. When the next label would fall off the bottom of the frame the stack
    /// starts a new column to the right of the widest label so far, instead of drawing
    /// labels nobody can see.
    /// </summary>
    internal sealed class LabelStack
    {
        public const int TopY         = 20;
        public const int ColumnGap    = 12;
        /// <summary>Room left under a baseline for descenders before a label counts as off-frame.</summary>
        public const int BottomMargin = 4;

        private readonly int _frameHeight;
        private int _x = VisionDrawing.LeftMargin;
        private int _y = TopY;
        private int _columnWidth;

        public LabelStack(int frameHeight) => _frameHeight = frameHeight;

        /// <summary>Where the next label's baseline will start, after any column wrap.</summary>
        public Point Next
        {
            get { WrapIfNeeded(); return new Point(_x, _y); }
        }

        public void DrawStatusLabel(Mat img, string text, bool passed)
        {
            var at = Next;
            VisionDrawing.DrawOutlinedText(img, text, at, VisionDrawing.StatusFontScale,
                                           passed ? VisionPalette.Pass : VisionPalette.Fail);

            var size = Cv2.GetTextSize(text, VisionDrawing.Font, VisionDrawing.StatusFontScale,
                                       VisionDrawing.TextOutlineThickness, out _);
            _columnWidth = Math.Max(_columnWidth, size.Width);
            _y += VisionDrawing.LineHeight;
        }

        private void WrapIfNeeded()
        {
            // Only wrap once the column holds something, so a frame too short for even one
            // label still gets it (clipped) rather than wrapping forever.
            if (_y <= _frameHeight - BottomMargin || _y == TopY) return;
            _x += _columnWidth + ColumnGap;
            _y  = TopY;
            _columnWidth = 0;
        }
    }
}
