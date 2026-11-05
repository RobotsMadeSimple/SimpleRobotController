using Controller.RobotControl.Vision;
using Controller.RobotControl.Vision.Inspections;
using OpenCvSharp;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

/// <summary>
/// Skewed (perspective) grid zones, frame detection, zone alignment, and the corner/edge
/// inspections — all on a synthetic 8x8 board rendered at an angle, with red pieces on known
/// squares, so the right answer is known exactly.
/// </summary>
public class VisionAlignmentTests
{
    private const int W = 640, H = 480, Board = 400;

    // Two camera views of the same board: corners TL, TR, BR, BL in pixels.
    private static readonly Point2f[] PoseA = { new(180, 120), new(460, 110), new(580, 400), new(70, 410) };
    private static readonly Point2f[] PoseB = { new(220, 90), new(520, 140), new(560, 430), new(110, 380) };

    private static readonly (int Row, int Col)[] Pieces = { (0, 1), (2, 3), (5, 4), (7, 6) };

    /// <summary>The board, flat: alternating light/dark squares, a red disc on each piece square.</summary>
    private static Mat FlatBoard()
    {
        var flat = new Mat(Board, Board, MatType.CV_8UC3, Scalar.All(0));
        int s = Board / 8;
        for (int r = 0; r < 8; r++)
        for (int c = 0; c < 8; c++)
            Cv2.Rectangle(flat, new Rect(c * s, r * s, s, s), (r + c) % 2 == 0 ? Scalar.All(225) : Scalar.All(150), -1);
        foreach (var (r, c) in Pieces)
            Cv2.Circle(flat, new Point(c * s + s / 2, r * s + s / 2), s * 2 / 5, new Scalar(0, 0, 255), -1);
        return flat;
    }

    /// <summary>The board seen from a camera: warped onto <paramref name="pose"/> on a black frame.</summary>
    private static Mat Render(Point2f[] pose)
    {
        using var flat = FlatBoard();
        var src = new[] { new Point2f(0, 0), new Point2f(Board, 0), new Point2f(Board, Board), new Point2f(0, Board) };
        using var hm = Cv2.GetPerspectiveTransform(src, pose);
        var img = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        Cv2.WarpPerspective(flat, img, hm, new Size(W, H), InterpolationFlags.Linear, BorderTypes.Constant, Scalar.All(0));
        return img;
    }

    private static List<double[]> Norm(Point2f[] q) => q.Select(p => new[] { p.X / (double)W, p.Y / (double)H }).ToList();

    private static VisionZone QuadZone(Point2f[] pose) => new()
    {
        Id = "board", Name = "Board", Grid = new VisionZoneGrid { Rows = 8, Cols = 8 },
        Geometry = new VisionZoneGeometry { Shape = VisionZoneShape.Quad, Points = Norm(pose) },
    };

    private static ColorCoverageInspection Red(string zoneId = "board") => new()
    {
        Id = "red", Name = "Red", ZoneId = zoneId, Colors = { new ColorEntry { R = 255, G = 0, B = 0, Tolerance = 40 } }, MinCoverage = 25,
    };

    private static HashSet<(int, int)> PassedCells(ColorCoverageResult r) =>
        r.Cells!.Where(c => c.Passed).Select(c => (c.Row, c.Col)).ToHashSet();

    private static void AssertNear(Point2f[] expected, List<double[]> actualNorm, double tolPx)
    {
        Assert.Equal(4, actualNorm.Count);
        for (int i = 0; i < 4; i++)
        {
            double dx = actualNorm[i][0] * W - expected[i].X, dy = actualNorm[i][1] * H - expected[i].Y;
            Assert.True(Math.Sqrt(dx * dx + dy * dy) <= tolPx,
                $"corner {i}: expected ({expected[i].X},{expected[i].Y}) got ({actualNorm[i][0] * W:0.0},{actualNorm[i][1] * H:0.0})");
        }
    }

    // -- Quad zones ---------------------------------------------------------------

    [Fact]
    public void AQuadGridFollowsThePerspectiveOfAnAngledBoard()
    {
        using var img = Render(PoseA);
        using var ctx = new FrameContext(img);
        using var d = InspectionPlan.Color.Run(ctx, Red(), QuadZone(PoseA));

        Assert.Equal(64, d.Result.Cells!.Count);
        Assert.Equal(Pieces.ToHashSet(), PassedCells(d.Result));

        // The same zone as a plain bounding rectangle reads the wrong cells — the point of the quad.
        var rect = new VisionZone
        {
            Id = "rect", Name = "Rect", Grid = new VisionZoneGrid { Rows = 8, Cols = 8 },
            Geometry = new VisionZoneGeometry { X = 70.0 / W, Y = 110.0 / H, Width = 510.0 / W, Height = 300.0 / H },
        };
        using var dr = InspectionPlan.Color.Run(ctx, Red("rect"), rect);
        Assert.NotEqual(Pieces.ToHashSet(), PassedCells(dr.Result));

        using var annotated = img.Clone();
        InspectionPlan.Color.Annotate(annotated, d, Red(), QuadZone(PoseA), new LabelStack(H));
    }

    [Fact]
    public void QuadCellsTileTheQuadAndAMalformedQuadIsEmpty()
    {
        var g = QuadZone(PoseA).Geometry;
        Assert.True(ZoneGeometry.IsQuad(g) && ZoneGeometry.HasQuadCells(g));
        var c00 = ZoneGeometry.CellQuad(g, 8, 8, 0, 0, W, H);
        var c77 = ZoneGeometry.CellQuad(g, 8, 8, 7, 7, W, H);
        Assert.True(Math.Abs(c00[0].X - PoseA[0].X) < 0.01 && Math.Abs(c00[0].Y - PoseA[0].Y) < 0.01);
        Assert.True(Math.Abs(c77[2].X - PoseA[2].X) < 0.01 && Math.Abs(c77[2].Y - PoseA[2].Y) < 0.01);
        // Neighbours share edges exactly.
        var c01 = ZoneGeometry.CellQuad(g, 8, 8, 0, 1, W, H);
        Assert.Equal(c00[1], c01[0]);

        var bad = new VisionZoneGeometry { Shape = VisionZoneShape.Quad, Points = Norm(PoseA).Take(3).ToList() };
        Assert.False(ZoneGeometry.IsInsideZone(bad, 300, 250, W, H));
        using var mask = new Mat(H, W, MatType.CV_8UC1, Scalar.Black);
        ZoneGeometry.FillZoneMask(mask, bad, W, H);
        Assert.Equal(0, Cv2.CountNonZero(mask));
        Assert.True(ZoneGeometry.IsInsideZone(g, 300, 250, W, H));
    }

    // -- Frame detection ----------------------------------------------------------

    [Theory]
    [InlineData("contour", 4.0)]
    [InlineData("lines", 6.0)]
    [InlineData("corners", 6.0)]
    public void FrameFindsTheBoardCornersInOrder(string method, double tolPx)
    {
        using var img = Render(PoseA);
        using var ctx = new FrameContext(img);
        var insp = new FrameInspection { Id = "f", Name = "Board frame", Method = method };
        var d = InspectionPlan.Frame.Run(ctx, insp, zone: null);

        Assert.True(d.Result.Found, $"{method} did not find the frame");
        AssertNear(PoseA, d.Result.Corners, tolPx);
        Assert.InRange(d.Result.Angle, -3, 0);   // TL→TR rises slightly to the right
        Assert.True(d.Result.AreaPct > 20);

        using var annotated = img.Clone();
        InspectionPlan.Frame.Annotate(annotated, d, insp, null, new LabelStack(H));
    }

    [Fact]
    public void LinesStillFindAFrameWithACornerCovered()
    {
        // A "hand" over the top-right corner: no corner to see, but most of both edges remain.
        using var img = Render(PoseB);
        Cv2.Circle(img, new Point((int)PoseB[1].X, (int)PoseB[1].Y), 45, new Scalar(60, 90, 140), -1);
        using var ctx = new FrameContext(img);
        var d = InspectionPlan.Frame.Run(ctx, new FrameInspection { Id = "f", Method = "lines" }, null);
        Assert.True(d.Result.Found);
        AssertNear(PoseB, d.Result.Corners, 8);
    }

    [Fact]
    public void NoFrameOnABlankImage()
    {
        using var img = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        using var ctx = new FrameContext(img);
        foreach (var m in new[] { "contour", "lines", "corners" })
        {
            var d = InspectionPlan.Frame.Run(ctx, new FrameInspection { Id = "f", Method = m }, null);
            Assert.False(d.Result.Found, m);
            Assert.Empty(d.Result.Corners);
        }
    }

    // -- Alignment ----------------------------------------------------------------

    /// <summary>A program whose grid zone was drawn on the board at pose A, aligned to a frame whose reference is pose A.</summary>
    private static VisionProgram AlignedProgram(bool aligned = true, List<double[]>? reference = null)
    {
        var zone = QuadZone(PoseA);
        zone.AlignTo = aligned ? "frame" : null;
        return new VisionProgram
        {
            Id = "p", Name = "Checkers",
            Zones = { zone },
            FrameInspections = { new FrameInspection { Id = "frame", Name = "Board frame", Method = "contour", ReferenceCorners = reference ?? Norm(PoseA) } },
            ColorInspections = { Red() },
            // Even listed first, the color inspection must run after the frame.
            InspectionOrder = { "red", "frame" },
        };
    }

    [Fact]
    public void AnAlignedGridFollowsTheBoardWhenItMoves()
    {
        using var img = Render(PoseB);
        var result = VisionProcessor.RunProgramOnce(AlignedProgram(), img, out _);

        Assert.Empty(result.Errors);
        Assert.True(result.FrameResults.Single().Found);
        Assert.Equal(Pieces.ToHashSet(), PassedCells(result.ColorResults.Single()));
        var outline = result.AlignedZones.Single();
        Assert.Equal("board", outline.ZoneId);
        AssertNear(PoseB, outline.Points, 5);

        // Without alignment the zone stays where it was drawn and misreads the moved board.
        var still = VisionProcessor.RunProgramOnce(AlignedProgram(aligned: false), img, out _);
        Assert.NotEqual(Pieces.ToHashSet(), PassedCells(still.ColorResults.Single()));
    }

    [Fact]
    public void AGriddedRectangleIsWarpedIntoAPerspectiveQuad()
    {
        // A rectangle zone drawn over a flat-on board, then the board seen at an angle.
        var front = new[] { new Point2f(120, 40), new Point2f(520, 40), new Point2f(520, 440), new Point2f(120, 440) };
        var zone = new VisionZone
        {
            Id = "board", Name = "Board", Grid = new VisionZoneGrid { Rows = 8, Cols = 8 }, AlignTo = "frame",
            Geometry = new VisionZoneGeometry { X = 120.0 / W, Y = 40.0 / H, Width = 400.0 / W, Height = 400.0 / H },
        };
        var prog = new VisionProgram
        {
            Id = "p", Zones = { zone }, ColorInspections = { Red() },
            FrameInspections = { new FrameInspection { Id = "frame", Name = "Frame", ReferenceCorners = Norm(front) } },
        };
        using var img = Render(PoseA);
        var result = VisionProcessor.RunProgramOnce(prog, img, out _);
        Assert.Equal(Pieces.ToHashSet(), PassedCells(result.ColorResults.Single()));
    }

    [Fact]
    public void AnAlignedInspectionIsSkippedWhenTheFrameIsNotFound()
    {
        using var img = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        var result = VisionProcessor.RunProgramOnce(AlignedProgram(), img, out _);
        Assert.Empty(result.ColorResults);
        Assert.Contains(result.Errors, e => e.StartsWith("red:") && e.Contains("not found"));
    }

    [Fact]
    public void WithoutAReferenceTheZoneIsUsedAsDrawnAndSaysSo()
    {
        using var img = Render(PoseA);
        var prog = AlignedProgram();
        prog.FrameInspections[0].ReferenceCorners = null;
        var result = VisionProcessor.RunProgramOnce(prog, img, out _);
        Assert.Equal(Pieces.ToHashSet(), PassedCells(result.ColorResults.Single()));
        Assert.Contains(result.Errors, e => e.Contains("no reference"));
        Assert.Empty(result.AlignedZones);
    }

    [Fact]
    public void WarpingByTheIdentityKeepsAZoneWhereItIs()
    {
        var id = new double[][] { [1, 0, 0], [0, 1, 0], [0, 0, 1] };
        var rect = new VisionZone { Id = "r", Geometry = new VisionZoneGeometry { X = 0.1, Y = 0.2, Width = 0.5, Height = 0.4 } };
        var w = ZoneGeometry.Warp(rect, id, W, H);
        Assert.Equal(VisionZoneShape.Quad, w.Geometry.Shape);
        Assert.Equal(new[] { 0.1, 0.2 }, w.Geometry.Points[0].Select(v => Math.Round(v, 6)));
        Assert.Equal(new[] { 0.6, 0.6 }, w.Geometry.Points[2].Select(v => Math.Round(v, 6)));
        Assert.Null(ZoneGeometry.Between(Norm(PoseA).Take(3).ToList(), Norm(PoseB)));
    }

    // -- Corner and edge inspections ----------------------------------------------

    [Fact]
    public void CornersFindTheFourCornersOfASquare()
    {
        using var img = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        Cv2.Rectangle(img, new Rect(200, 140, 200, 200), Scalar.All(255), -1);
        using var ctx = new FrameContext(img);
        var insp = new CornerInspection { Id = "c", Name = "Corners", MaxCorners = 10, Quality = 0.1, MinDistance = 20, MinCount = 4 };
        var d = InspectionPlan.Corner.Run(ctx, insp, null);

        Assert.Equal(4, d.Result.Count);
        Assert.True(d.Result.Found);
        foreach (var (x, y) in new[] { (200, 140), (399, 140), (399, 339), (200, 339) })
            Assert.Contains(d.Result.Points, p => Math.Abs(p[0] * W - x) <= 3 && Math.Abs(p[1] * H - y) <= 3);

        var harris = InspectionPlan.Corner.Run(ctx, new CornerInspection { Id = "h", Method = "harris", MaxCorners = 10, Quality = 0.1, MinDistance = 20 }, null);
        Assert.Equal(4, harris.Result.Count);

        // Zone-limited: only the top-left corner lies in this zone.
        var zone = new VisionZone { Id = "z", Geometry = new VisionZoneGeometry { X = 150.0 / W, Y = 100.0 / H, Width = 100.0 / W, Height = 100.0 / H } };
        Assert.Equal(1, InspectionPlan.Corner.Run(ctx, insp, zone).Result.Count);
    }

    [Fact]
    public void EdgesMeasureDensityAndTheDominantDirection()
    {
        using var img = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        Cv2.Rectangle(img, new Rect(0, 240, W, 240), Scalar.All(255), -1);   // one horizontal edge
        using var ctx = new FrameContext(img);
        var insp = new EdgeInspection { Id = "e", Name = "Edges", MinDensity = 0.1 };
        using var d = InspectionPlan.Edge.Run(ctx, insp, null);
        Assert.True(d.Result.Density > 0.1 && d.Result.Density < 2);
        Assert.True(d.Result.Passed);
        Assert.NotNull(d.Result.DominantAngle);
        Assert.True(d.Result.DominantAngle < 2 || d.Result.DominantAngle > 178);

        using var blank = new Mat(H, W, MatType.CV_8UC3, Scalar.All(0));
        using var bctx = new FrameContext(blank);
        using var bd = InspectionPlan.Edge.Run(bctx, insp, null);
        Assert.Equal(0, bd.Result.Density);
        Assert.False(bd.Result.Passed);
        Assert.Null(bd.Result.DominantAngle);
    }

    [Fact]
    public void NewInspectionsJoinAllInspectionsAndRunFramesFirst()
    {
        var prog = AlignedProgram();
        prog.CornerInspections.Add(new CornerInspection { Id = "corn" });
        prog.EdgeInspections.Add(new EdgeInspection { Id = "edge" });
        Assert.Equal(new[] { "red", "frame", "corn", "edge" }, prog.AllInspections().Select(i => i.Id));
        var plan = InspectionPlan.Build(prog, new ArucoStrategy());
        Assert.Equal("frame", plan.Steps[0].Id);
    }
}
