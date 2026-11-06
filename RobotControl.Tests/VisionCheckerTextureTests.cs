using Controller.RobotControl.Vision;
using Controller.RobotControl.Vision.Inspections;
using OpenCvSharp;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

/// <summary>
/// The checker-grid frame and the texture inspection, on two real photos of a checkers board
/// taken from the side (Fixtures/checkers-real-*.jpg, 1280×720): black pieces on black
/// squares, white (cream) pieces, the board's near corner cut off in photo 1. The pieces were
/// labelled by hand; squares are (row, col) with row 0 the far side.
/// </summary>
public class VisionCheckerTextureTests
{
    private const int W = 1280, H = 720;

    private sealed record Photo(string File, Point2f[] Squares, HashSet<(int, int)> White, HashSet<(int, int)> Black);

    // Squares: the playing area's corners TL, TR, BR, BL as fitted and checked by eye.
    private static readonly Photo One = new("checkers-real-1.jpg",
        new Point2f[] { new(428.2f, 221.0f), new(845.9f, 214.9f), new(1288.1f, 626.4f), new(17.5f, 645.0f) },
        new() { (0, 0), (0, 2), (1, 1), (2, 0), (2, 2), (3, 1), (4, 0), (4, 2), (5, 1), (6, 0), (6, 2), (7, 1) },
        new() { (0, 6), (1, 5), (1, 7), (2, 6), (3, 5), (3, 7), (4, 6), (5, 5), (5, 7), (6, 6), (7, 5), (7, 7) });
    private static readonly Photo Two = new("checkers-real-2.jpg",
        new Point2f[] { new(467.6f, 290.0f), new(839.0f, 288.4f), new(1203.2f, 618.6f), new(148.5f, 616.4f) },
        new() { (0, 0), (1, 1), (1, 3), (2, 0), (3, 1), (3, 3), (4, 0), (5, 1), (5, 3), (6, 0), (6, 2), (7, 1) },
        new() { (0, 4), (0, 6), (1, 7), (2, 6), (3, 5), (3, 7), (4, 4), (4, 6), (5, 7), (6, 6), (7, 5), (7, 7) });

    public static IEnumerable<object[]> Photos => new[] { new object[] { "one" }, new object[] { "two" } };
    private static Photo Get(string which) => which == "one" ? One : Two;

    private static Mat Load(Photo p) => Cv2.ImRead(Path.Combine(AppContext.BaseDirectory, "Fixtures", p.File));

    /// <summary>The corners as someone would draw them: each a fifth of its square off, in a fixed direction.</summary>
    private static Point2f[] Rough(Photo p)
    {
        var q = p.Squares;
        float top = Dist(q[0], q[1]) / 8, bottom = Dist(q[3], q[2]) / 8;
        var dirs = new[] { (0.7f, 0.7f), (-0.7f, 0.7f), (0.7f, -0.7f), (-0.7f, -0.7f) };
        return q.Select((c, i) =>
        {
            float s = (i < 2 ? top : bottom) * 0.2f;
            return new Point2f(c.X + dirs[i].Item1 * s, c.Y + dirs[i].Item2 * s);
        }).ToArray();
    }

    /// <summary>8% of the square at corner i: about 3 px far away, 12 px close up.</summary>
    private static float Tolerance(Point2f[] q, int i) => 0.08f * (i < 2 ? Dist(q[0], q[1]) : Dist(q[3], q[2])) / 8;

    private static float Dist(Point2f a, Point2f b) => (float)Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

    private static VisionZone BoardZone(Point2f[] corners) => new()
    {
        Id = "board", Name = "Board", Grid = new VisionZoneGrid { Rows = 8, Cols = 8 }, AlignTo = "squares",
        Geometry = new VisionZoneGeometry { Shape = VisionZoneShape.Quad, Points = corners.Select(c => new[] { c.X / (double)W, c.Y / (double)H }).ToList() },
    };

    /// <summary>The vision program checkers-vision.mjs builds, on the given drawn corners.</summary>
    private static VisionProgram CheckersProgram(Point2f[] drawn, string frameId) => new()
    {
        Id = "checkers", Name = "Checkers",
        Zones = { BoardZone(drawn) },
        FrameInspections = { new FrameInspection { Id = frameId, Name = "Squares", ZoneId = "board", Method = "checker" } },
        ColorInspections = { new ColorCoverageInspection { Id = "white", Name = "White", ZoneId = "board", MinCoverage = 30,
                             Colors = { new ColorEntry { R = 232, G = 230, B = 200, Tolerance = 20 } } } },
        TextureInspections = { new TextureInspection { Id = "black", Name = "Black", ZoneId = "board" } },
    };

    private static VisionProgram WithAlign(VisionProgram p, string frameId) { p.Zones[0].AlignTo = frameId; return p; }

    private static readonly HashSet<(int, int)> Dark =
        Enumerable.Range(0, 64).Select(i => (i / 8, i % 8)).Where(rc => (rc.Item1 + rc.Item2) % 2 == 0).ToHashSet();

    [Theory]
    [MemberData(nameof(Photos))]
    public void Checker_frame_snaps_a_rough_zone_onto_the_squares(string which)
    {
        var p = Get(which);
        using var img = Load(p);
        using var ctx = new FrameContext(img);
        var d = InspectionPlan.Frame.Run(ctx, new FrameInspection { Id = $"snap-{which}", Method = "checker" }, BoardZone(Rough(p)));

        Assert.True(d.Result.Found);
        Assert.True(d.Result.Score >= 8, $"edge ratio {d.Result.Score}");
        for (int i = 0; i < 4; i++)
            Assert.True(Dist(d.Corners![i], p.Squares[i]) < Tolerance(p.Squares, i),
                $"corner {i}: {d.Corners[i]} vs {p.Squares[i]} ({Dist(d.Corners[i], p.Squares[i]):0.0} px)");
    }

    [Theory]
    [MemberData(nameof(Photos))]
    public void Checkers_program_reads_every_piece(string which)
    {
        var p = Get(which);
        using var img = Load(p);
        var r = VisionProcessor.RunProgramOnce(WithAlign(CheckersProgram(Rough(p), $"read-{which}"), $"read-{which}"), img, out _);

        Assert.Empty(r.Errors);
        var white = r.ColorResults.Single().Cells!.Where(c => c.Passed).Select(c => (c.Row, c.Col)).Where(Dark.Contains).ToHashSet();
        var textured = r.TextureResults.Single().Cells!.Where(c => c.Passed).Select(c => (c.Row, c.Col)).Where(Dark.Contains).ToHashSet();
        textured.ExceptWith(white);   // white pieces pass texture too; detectMove's bothMeans=white
        Assert.Equal(p.White.OrderBy(x => x), white.OrderBy(x => x));
        Assert.Equal(p.Black.OrderBy(x => x), textured.OrderBy(x => x));

        // The margin, not just the answer: black pieces well above the threshold, empty squares well below.
        var scores = r.TextureResults.Single().Cells!.ToDictionary(c => (c.Row, c.Col), c => c.Score);
        Assert.True(p.Black.Min(k => scores[k]) >= 6, "black pieces score at least 6");
        Assert.True(Dark.Except(p.White).Except(p.Black).Max(k => scores[k]) <= 3, "empty dark squares score at most 3");
    }

    [Fact]
    public void Self_checker_zone_is_used_where_the_squares_are()
    {
        using var img = Load(Two);
        var r = VisionProcessor.RunProgramOnce(WithAlign(CheckersProgram(Rough(Two), "self"), "self"), img, out _);

        var aligned = r.AlignedZones.Single(a => a.ZoneId == "board").Points;
        var found = r.FrameResults.Single().Corners;
        for (int i = 0; i < 4; i++)
        {
            Assert.InRange(aligned[i][0], found[i][0] - 0.002, found[i][0] + 0.002);
            Assert.InRange(aligned[i][1], found[i][1] - 0.002, found[i][1] + 0.002);
        }
    }

    [Fact]
    public void Checker_keeps_corner_order_on_a_steep_board()
    {
        // Photo 2's TL and BL have almost the same x+y: sorting corners by x±y would swap TR/BR.
        using var img = Load(Two);
        using var ctx = new FrameContext(img);
        var d = InspectionPlan.Frame.Run(ctx, new FrameInspection { Id = "order", Method = "checker" }, BoardZone(Two.Squares));
        Assert.True(d.Result.Found);
        Assert.True(d.Corners![1].Y < d.Corners[2].Y - 200, "TR above BR");
        Assert.True(d.Corners[0].X > d.Corners[3].X + 200, "TL right of BL");
    }

    [Fact]
    public void Checker_not_found_skips_the_aligned_inspections()
    {
        // A zone drawn on photo 2's board, run on photo 1 (camera moved by more than a square).
        using var img = Load(One);
        var r = VisionProcessor.RunProgramOnce(WithAlign(CheckersProgram(Two.Squares, "moved"), "moved"), img, out _);

        Assert.False(r.FrameResults.Single().Found);
        Assert.True(r.FrameResults.Single().Score < 4);
        Assert.Empty(r.ColorResults);
        Assert.Empty(r.TextureResults);
        Assert.Contains(r.Errors, e => e.Contains("not found"));
    }

    [Fact]
    public void Checker_tracks_a_nudged_camera()
    {
        // Same board shifted 18 px right and 10 px down (a bumped camera); the zone stays put.
        using var img = Load(Two);
        using var moved = new Mat();
        using (var m = new Mat(2, 3, MatType.CV_64FC1))
        {
            m.Set(0, 0, 1.0); m.Set(0, 1, 0.0); m.Set(0, 2, 18.0);
            m.Set(1, 0, 0.0); m.Set(1, 1, 1.0); m.Set(1, 2, 10.0);
            Cv2.WarpAffine(img, moved, m, img.Size(), InterpolationFlags.Linear, BorderTypes.Replicate);
        }
        using var ctx = new FrameContext(moved);
        var d = InspectionPlan.Frame.Run(ctx, new FrameInspection { Id = "nudge", Method = "checker" }, BoardZone(Two.Squares));
        Assert.True(d.Result.Found);
        for (int i = 0; i < 4; i++)
        {
            var want = new Point2f(Two.Squares[i].X + 18, Two.Squares[i].Y + 10);
            Assert.True(Dist(d.Corners![i], want) < Tolerance(Two.Squares, i), $"corner {i}: {Dist(d.Corners[i], want):0.0} px off");
        }
    }

    [Fact]
    public void Texture_ignores_smooth_shading_but_sees_fine_rings()
    {
        // Left: a dark patch with a strong lighting gradient. Right: same dark, with thin rings.
        using var img = new Mat(200, 400, MatType.CV_8UC3, Scalar.All(30));
        for (int x = 0; x < 200; x++)
            Cv2.Line(img, new Point(x, 0), new Point(x, 199), Scalar.All(20 + x * 0.3), 1);
        for (int rad = 10; rad < 90; rad += 8)
            Cv2.Circle(img, new Point(300, 100), rad, Scalar.All(75), 1, LineTypes.AntiAlias);

        var zone = new VisionZone
        {
            Id = "z", Name = "z", Grid = new VisionZoneGrid { Rows = 1, Cols = 2 },
            Geometry = new VisionZoneGeometry { Shape = VisionZoneShape.Rectangle, X = 0, Y = 0, Width = 1, Height = 1 },
        };
        using var ctx = new FrameContext(img);
        var d = InspectionPlan.Texture.Run(ctx, new TextureInspection { Id = "t", Name = "t", ZoneId = "z" }, zone);
        var cells = d.Result.Cells!;
        Assert.True(cells[0].Score < 1.5, $"gradient scored {cells[0].Score}");
        Assert.True(cells[1].Score > 6, $"rings scored {cells[1].Score}");
        Assert.False(cells[0].Passed);
        Assert.True(cells[1].Passed);
        Assert.Equal(1, d.Result.CellsPassed);
    }
}
