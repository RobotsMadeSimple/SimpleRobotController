using Controller.RobotControl;
using Controller.RobotControl.Vision;
using OpenCvSharp;

namespace RobotControl.Tests;

/// <summary>
/// Tilted rectangle zones.
///
/// The trap this pins down is the coordinate space. Geometry is stored as 0–1 fractions,
/// which are scaled by width and height independently; rotating in that space shears the
/// rectangle into a parallelogram on any non-square frame, and the result still *looks*
/// plausible in a preview. So the frame used here is deliberately 640×480 rather than
/// square — a shear bug shows up as an unequal diagonal, and a square frame would hide it.
///
/// The other thing worth pinning is that rotation stays opt-in: a zone with Rotation = 0
/// must produce byte-identical results to one from before rotation existed, because every
/// untilted path is supposed to be the original code untouched.
/// </summary>
public class VisionZoneRotationTests
{
    private const int W = 640, H = 480;

    private static VisionZoneGeometry Rect(double rotation) => new()
    {
        Shape = VisionZoneShape.Rectangle,
        X = 0.25, Y = 0.25, Width = 0.5, Height = 0.5,
        Rotation = rotation,
    };

    private static double Dist(Point2f a, Point2f b)
    {
        double dx = a.X - b.X, dy = a.Y - b.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    // -- Corners -----------------------------------------------------------------

    [Fact]
    public void AnUntiltedRectangleIsNotTreatedAsRotated()
    {
        Assert.False(VisionProcessor.IsRotatedRect(Rect(0)));
        Assert.True(VisionProcessor.IsRotatedRect(Rect(30)));
    }

    [Fact]
    public void CirclesAndPolygonsAreNeverTreatedAsRotated()
    {
        // Rotation is meaningless on a circle and redundant on a polygon, whose points are
        // already absolute. Both must keep their original paths even if the field is set.
        Assert.False(VisionProcessor.IsRotatedRect(new VisionZoneGeometry
        {
            Shape = VisionZoneShape.Circle, Rotation = 45,
        }));
        Assert.False(VisionProcessor.IsRotatedRect(new VisionZoneGeometry
        {
            Shape = VisionZoneShape.Polygon, Rotation = 45, Points = [[0, 0], [1, 0], [1, 1]],
        }));
    }

    [Fact]
    public void UntiltedCornersAreTheRectangleItself()
    {
        var c = VisionProcessor.RectCorners(Rect(0), W, H);

        Assert.Equal(160, c[0].X, 3); Assert.Equal(120, c[0].Y, 3);   // top-left
        Assert.Equal(480, c[1].X, 3); Assert.Equal(120, c[1].Y, 3);   // top-right
        Assert.Equal(480, c[2].X, 3); Assert.Equal(360, c[2].Y, 3);   // bottom-right
        Assert.Equal(160, c[3].X, 3); Assert.Equal(360, c[3].Y, 3);   // bottom-left
    }

    [Fact]
    public void RotationPreservesSideLengths()
    {
        // This is the shear check. In pixels the rectangle is 320×240; if rotation were
        // applied in normalized space those sides would come out unequal to each other
        // after scaling, and the diagonals would stop matching.
        var c = VisionProcessor.RectCorners(Rect(37), W, H);

        Assert.Equal(320, Dist(c[0], c[1]), 2);   // top edge
        Assert.Equal(240, Dist(c[1], c[2]), 2);   // right edge
        Assert.Equal(320, Dist(c[2], c[3]), 2);   // bottom edge
        Assert.Equal(240, Dist(c[3], c[0]), 2);   // left edge
        Assert.Equal(Dist(c[0], c[2]), Dist(c[1], c[3]), 2);
    }

    [Fact]
    public void RotationIsAboutTheRectangleCenter()
    {
        foreach (var angle in new double[] { 0, 15, 90, 180, -42 })
        {
            var c = VisionProcessor.RectCorners(Rect(angle), W, H);
            Assert.Equal(320, c.Average(p => p.X), 2);
            Assert.Equal(240, c.Average(p => p.Y), 2);
        }
    }

    [Fact]
    public void NinetyDegreesSwapsTheSides()
    {
        // A quarter turn of a 320x240 box gives a 240x320 footprint about the same center.
        var b = VisionProcessor.ZoneBounds(Rect(90), W, H);

        Assert.Equal(320 - 120, b.X);
        Assert.Equal(240 - 160, b.Y);
        Assert.Equal(240, b.Width);
        Assert.Equal(320, b.Height);
    }

    // -- Bounds ------------------------------------------------------------------

    [Fact]
    public void UntiltedBoundsMatchThePreRotationResult()
    {
        Assert.Equal(new Rect(160, 120, 320, 240), VisionProcessor.ZoneBounds(Rect(0), W, H));
    }

    [Fact]
    public void TiltedBoundsGrowAndStayCentered()
    {
        var b = VisionProcessor.ZoneBounds(Rect(45), W, H);

        // A 45 degree turn of 320x240 has an extent of (320+240)/sqrt(2) on both axes.
        double expected = 560 / Math.Sqrt(2);
        Assert.Equal(expected, b.Width,  1);
        Assert.Equal(expected, b.Height, 1);
        Assert.Equal(320, b.X + b.Width  / 2.0, 1);
        Assert.Equal(240, b.Y + b.Height / 2.0, 1);
    }

    [Fact]
    public void TiltedBoundsAreClampedToTheFrame()
    {
        // Turned on the spot near a corner, the bounding box would run off the frame.
        var geom = new VisionZoneGeometry
        {
            Shape = VisionZoneShape.Rectangle,
            X = 0.0, Y = 0.0, Width = 0.4, Height = 0.4, Rotation = 45,
        };
        var b = VisionProcessor.ZoneBounds(geom, W, H);

        Assert.True(b.X >= 0 && b.Y >= 0);
        Assert.True(b.X + b.Width  <= W);
        Assert.True(b.Y + b.Height <= H);
    }

    // -- Grid cells --------------------------------------------------------------

    [Fact]
    public void CellQuadsTileTheRectangleWithoutOverlap()
    {
        // Every cell of a 2x2 has a quarter of the area, and together they add back up to
        // the whole rectangle — the check that the lattice neither gaps nor double-counts.
        const int rows = 2, cols = 2;
        double whole = 320.0 * 240.0;
        double sum = 0;

        for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
        {
            var q    = VisionProcessor.CellQuad(Rect(33), rows, cols, r, c, W, H);
            double a = ShoelaceArea(q);
            Assert.Equal(whole / (rows * cols), a, 1);
            sum += a;
        }

        Assert.Equal(whole, sum, 1);
    }

    [Fact]
    public void CellQuadsRotateWithTheRectangle()
    {
        // The lattice is built in the rectangle's own frame, so a cell's corners must be the
        // untilted cell's corners put through the same rotation — not a box re-derived from
        // the tilted bounding box, which is the mistake that makes cells drift off the zone.
        var untilted = VisionProcessor.CellQuad(Rect(0),  3, 4, 1, 2, W, H);
        var tilted   = VisionProcessor.CellQuad(Rect(25), 3, 4, 1, 2, W, H);

        double a = 25 * Math.PI / 180.0, cos = Math.Cos(a), sin = Math.Sin(a);
        for (int i = 0; i < 4; i++)
        {
            double dx = untilted[i].X - 320, dy = untilted[i].Y - 240;
            Assert.Equal(320 + dx * cos - dy * sin, tilted[i].X, 2);
            Assert.Equal(240 + dx * sin + dy * cos, tilted[i].Y, 2);
        }
    }

    [Fact]
    public void UntiltedCellQuadsAgreeWithTheRectLattice()
    {
        // The tilted and untilted paths are separate implementations, so they are only
        // trustworthy if they describe the same lattice at zero degrees.
        //
        // Agreement is to within a pixel, not exact: CellRect divides the bounds with integer
        // arithmetic while CellQuad works in doubles, so a 320px span over 3 columns lands on
        // 266 one way and 266.67 the other. That gap is harmless because the two paths never
        // both run on the same zone — the tolerance is here to catch a lattice that is
        // genuinely misplaced, such as cells offset by half a cell or indexed row/col swapped.
        var bounds = VisionProcessor.ZoneBounds(Rect(0), W, H);

        for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
        {
            var rect = VisionProcessor.CellRect(bounds, 3, 3, r, c);
            var quad = VisionProcessor.CellQuad(Rect(0), 3, 3, r, c, W, H);

            AssertWithinAPixel(rect.X,               quad.Min(p => p.X));
            AssertWithinAPixel(rect.Y,               quad.Min(p => p.Y));
            AssertWithinAPixel(rect.X + rect.Width,  quad.Max(p => p.X));
            AssertWithinAPixel(rect.Y + rect.Height, quad.Max(p => p.Y));
        }
    }

    private static void AssertWithinAPixel(double expected, double actual) =>
        Assert.True(Math.Abs(expected - actual) <= 1.0,
                    $"expected {expected} and {actual} to agree within a pixel");

    private static double ShoelaceArea(Point2f[] q)
    {
        double s = 0;
        for (int i = 0, j = q.Length - 1; i < q.Length; j = i++)
            s += (q[j].X + q[i].X) * (q[j].Y - q[i].Y);
        return Math.Abs(s / 2.0);
    }
}
