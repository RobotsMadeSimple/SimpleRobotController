using Controller.RobotControl;
using Controller.RobotControl.Vision;
using OpenCvSharp;

namespace RobotControl.Tests;

/// <summary>
/// Geometry behind gridded vision zones: where the lattice sits, and how it is divided.
/// The measurement itself is masked pixel counting inside VisionProcessor — what can go
/// wrong without anyone noticing is the tiling, so that is what is pinned here.
/// </summary>
public class VisionZoneGridTests
{
    private const int W = 640, H = 480;

    // -- Zone bounds -------------------------------------------------------------

    [Fact]
    public void RectangleBoundsAreTheRectangle()
    {
        var b = VisionProcessor.ZoneBounds(new VisionZoneGeometry
        {
            Shape = VisionZoneShape.Rectangle, X = 0.25, Y = 0.5, Width = 0.5, Height = 0.25,
        }, W, H);

        Assert.Equal(new Rect(160, 240, 320, 120), b);
    }

    [Fact]
    public void CircleBoundsAreTheSquareAroundIt()
    {
        // Radius is a fraction of min(w, h) = 480, so 0.25 -> 120px.
        var b = VisionProcessor.ZoneBounds(new VisionZoneGeometry
        {
            Shape = VisionZoneShape.Circle, Cx = 0.5, Cy = 0.5, Radius = 0.25,
        }, W, H);

        Assert.Equal(new Rect(320 - 120, 240 - 120, 240, 240), b);
    }

    [Fact]
    public void PolygonBoundsAreTheExtremes()
    {
        var b = VisionProcessor.ZoneBounds(new VisionZoneGeometry
        {
            Shape  = VisionZoneShape.Polygon,
            Points = [[0.2, 0.1], [0.6, 0.3], [0.4, 0.5]],
        }, W, H);

        Assert.Equal(new Rect(128, 48, 256, 192), b);
    }

    [Fact]
    public void BoundsAreClampedToTheFrame()
    {
        // A circle hanging off the top-left corner must not produce negative origins.
        var b = VisionProcessor.ZoneBounds(new VisionZoneGeometry
        {
            Shape = VisionZoneShape.Circle, Cx = 0.0, Cy = 0.0, Radius = 0.25,
        }, W, H);

        Assert.Equal(0, b.X);
        Assert.Equal(0, b.Y);
        Assert.True(b.Right <= W && b.Bottom <= H);
    }

    // -- Cell division -----------------------------------------------------------

    [Fact]
    public void CellsTileTheBoundsWithNoGapOrOverhang()
    {
        // 100 does not divide by 3 — the case where accumulating a cell width drifts.
        var bounds = new Rect(10, 20, 100, 100);
        const int rows = 3, cols = 3;

        int covered = 0;
        for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++)
            covered += VisionProcessor.CellRect(bounds, rows, cols, r, c).Width
                     * VisionProcessor.CellRect(bounds, rows, cols, r, c).Height;

        Assert.Equal(bounds.Width * bounds.Height, covered);

        var first = VisionProcessor.CellRect(bounds, rows, cols, 0, 0);
        var last  = VisionProcessor.CellRect(bounds, rows, cols, rows - 1, cols - 1);
        Assert.Equal(bounds.X, first.X);
        Assert.Equal(bounds.Y, first.Y);
        Assert.Equal(bounds.Right,  last.Right);
        Assert.Equal(bounds.Bottom, last.Bottom);
    }

    [Fact]
    public void AdjacentCellsMeetExactly()
    {
        var bounds = new Rect(0, 0, 101, 77);
        var a = VisionProcessor.CellRect(bounds, 4, 7, 1, 2);
        var b = VisionProcessor.CellRect(bounds, 4, 7, 1, 3);
        var below = VisionProcessor.CellRect(bounds, 4, 7, 2, 2);

        Assert.Equal(a.Right,  b.X);       // no horizontal seam
        Assert.Equal(a.Bottom, below.Y);   // no vertical seam
    }

    [Fact]
    public void OneByOneGridIsTheWholeBounds()
    {
        var bounds = new Rect(5, 6, 70, 90);
        Assert.Equal(bounds, VisionProcessor.CellRect(bounds, 1, 1, 0, 0));
    }

    [Fact]
    public void CellsAreOrderedRowMajor()
    {
        // The Index field of a ColorCellResult is row * cols + col, and grid editors lay
        // cells out left-to-right then top-to-bottom. Pin that the geometry agrees.
        var bounds = new Rect(0, 0, 100, 100);
        var topRight   = VisionProcessor.CellRect(bounds, 2, 2, 0, 1);
        var bottomLeft = VisionProcessor.CellRect(bounds, 2, 2, 1, 0);

        Assert.True(topRight.X > bottomLeft.X);
        Assert.True(bottomLeft.Y > topRight.Y);
    }
}
