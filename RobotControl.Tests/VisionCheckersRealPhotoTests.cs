using System.Text.Json;
using Controller.RobotControl.Vision;
using OpenCvSharp;
using Xunit.Abstractions;

namespace RobotControl.Tests;

/// <summary>
/// The checkers example's vision program (examples/dotnet/Checkers/program/checkers-vision.json,
/// built from generic tools: four findLine edges → a fourLines perspective fixture → per-square
/// pixelCount for white pieces and intensity stdDev on a background-removed layer for black
/// pieces) on two real photos of a board taken from the side (Fixtures/checkers-real-*.jpg,
/// 1280×720): black pieces on black squares, cream pieces, the board's near corner cut off in
/// photo 1. Pieces labelled by hand; squares are (row, col) with row 0 the far side.
/// </summary>
public class VisionCheckersRealPhotoTests
{
    private readonly ITestOutputHelper _out;
    public VisionCheckersRealPhotoTests(ITestOutputHelper output) => _out = output;

    private sealed record Photo(string File, Point2f[] Squares, HashSet<(int, int)> White, HashSet<(int, int)> Black);

    // Squares: the playing area's corners TL, TR, BR, BL, checked by eye.
    private static readonly Photo One = new("checkers-real-1.jpg",
        new Point2f[] { new(428.2f, 221.0f), new(845.9f, 214.9f), new(1288.1f, 626.4f), new(17.5f, 645.0f) },
        new() { (0, 0), (0, 2), (1, 1), (2, 0), (2, 2), (3, 1), (4, 0), (4, 2), (5, 1), (6, 0), (6, 2), (7, 1) },
        new() { (0, 6), (1, 5), (1, 7), (2, 6), (3, 5), (3, 7), (4, 6), (5, 5), (5, 7), (6, 6), (7, 5), (7, 7) });
    private static readonly Photo Two = new("checkers-real-2.jpg",
        new Point2f[] { new(467.6f, 290.0f), new(839.0f, 288.4f), new(1203.2f, 618.6f), new(148.5f, 616.4f) },
        new() { (0, 0), (1, 1), (1, 3), (2, 0), (3, 1), (3, 3), (4, 0), (5, 1), (5, 3), (6, 0), (6, 2), (7, 1) },
        new() { (0, 4), (0, 6), (1, 7), (2, 6), (3, 5), (3, 7), (4, 4), (4, 6), (5, 7), (6, 6), (7, 5), (7, 7) });

    private static readonly HashSet<(int, int)> Dark =
        Enumerable.Range(0, 64).Select(i => (i / 8, i % 8)).Where(rc => (rc.Item1 + rc.Item2) % 2 == 0).ToHashSet();

    private static Mat Load(Photo p) => Cv2.ImRead(Path.Combine(AppContext.BaseDirectory, "Fixtures", p.File));

    /// <summary>The example program as shipped (its regions are drawn on photo 2).</summary>
    internal static VisionProgram CheckersProgram() =>
        JsonSerializer.Deserialize<VisionProgram>(File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "Fixtures", "checkers-vision.json")))!;

    /// <summary>
    /// A findLine region drawn along an edge from A to B (clockwise round the board, so the
    /// probes run from outside the board inwards), keeping the shipped region's thickness.
    /// </summary>
    private static VisionRegion AlongEdge(VisionRegion shipped, double ax, double ay, double bx, double by)
    {
        const double W = 1280, H = 720;
        double cx = (ax + bx) / 2, cy = (ay + by) / 2, len = Math.Sqrt((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
        double thick = shipped.Height * H;
        return new VisionRegion
        {
            Shape = "rect", X = (cx - len / 2) / W, Y = (cy - thick / 2) / H, Width = len / W, Height = thick / H,
            Rotation = Math.Atan2(by - ay, bx - ax) * 180 / Math.PI,
        };
    }

    /// <summary>The four edge regions redrawn after the camera moved to photo 1's view.</summary>
    private static void UsePhotoOneLineRegions(VisionProgram p)
    {
        void Set(string id, double ax, double ay, double bx, double by) => p.Tool(id)!.Region = AlongEdge(p.Tool(id)!.Region!, ax, ay, bx, by);
        Set("top", 420, 205, 830, 205);
        Set("right", 959, 280, 1203, 480);
        Set("bottom", 1150, 628, 150, 643);
        Set("left", 52, 530, 331, 280);
    }

    private static (HashSet<(int, int)> White, HashSet<(int, int)> Black, Dictionary<(int, int), double> Texture) Read(VisionResult r)
    {
        var white = r.Tool("white")!.Cells.Where(c => c.Passed).Select(c => (c.Row, c.Col)).Where(Dark.Contains).ToHashSet();
        var black = r.Tool("black")!.Cells.Where(c => c.Passed).Select(c => (c.Row, c.Col)).Where(Dark.Contains).ToHashSet();
        black.ExceptWith(white);   // white pieces have texture too; detectMove's bothMeans=white
        return (white, black, r.Tool("black")!.Cells.ToDictionary(c => (c.Row, c.Col), c => c.Value));
    }

    private void Dump(string name, VisionProgram prog, VisionResult r, byte[] annotated)
    {
        foreach (var t in r.Tools)
            _out.WriteLine($"{t.Id}: {t.Status} {t.Message} {string.Join(" ", t.Values.Select(kv => $"{kv.Key}={kv.Value}"))}");
        foreach (var id in new[] { "white", "black" })
        {
            var cells = r.Tool(id)!.Cells;
            if (cells.Count == 0) continue;
            _out.WriteLine(id);
            for (int row = 0; row < 8; row++)
                _out.WriteLine(string.Join(" ", cells.Where(c => c.Row == row).Select(c => $"{c.Value,5:0.0}{(c.Passed ? "*" : " ")}")));
        }
        var dir = Environment.GetEnvironmentVariable("VISION_TEST_DUMP");
        if (!string.IsNullOrEmpty(dir)) File.WriteAllBytes(Path.Combine(dir, name + ".jpg"), annotated);
    }

    private static float Dist(Point2f a, PxPoint b) => (float)Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

    [Fact]
    public void Shipped_program_reads_every_piece_on_its_own_photo()
    {
        var prog = CheckersProgram();
        using var img = Load(Two);
        var r = VisionProcessor.RunProgramOnce(prog, img, out var annotated);
        Dump("photo2", prog, r, annotated);

        Assert.Empty(r.Errors);
        Assert.All(r.Tools, t => Assert.Equal(ToolStatus.Ok, t.Status));
        var (white, black, tex) = Read(r);
        _out.WriteLine($"MARGIN black min {Two.Black.Min(k => tex[k]):0.0} empty max {Dark.Except(Two.White).Except(Two.Black).Max(k => tex[k]):0.0}");
        Assert.Equal(Two.White.OrderBy(x => x), white.OrderBy(x => x));
        Assert.Equal(Two.Black.OrderBy(x => x), black.OrderBy(x => x));
        // The margin, not just the answer: black pieces well above the threshold, empty dark squares well below.
        Assert.True(Two.Black.Min(k => tex[k]) >= 6, $"black pieces score at least 6 (min {Two.Black.Min(k => tex[k]):0.0})");
        Assert.True(Dark.Except(Two.White).Except(Two.Black).Max(k => tex[k]) <= 3, "empty dark squares score at most 3");
        // The fixture's reference was captured on this first run.
        Assert.NotNull(prog.Tool("board")!.Reference?.Quad);
    }

    [Fact]
    public void Grid_follows_the_board_to_a_moved_camera_through_the_perspective_fixture()
    {
        // Taught on photo 2; the camera then moves to photo 1's view (far more than a nudge) and
        // only the four edge regions are redrawn — the 8×8 region and its reference stay as taught.
        var prog = CheckersProgram();
        using (var two = Load(Two)) VisionProcessor.RunProgramOnce(prog, two, out _);
        UsePhotoOneLineRegions(prog);

        using var img = Load(One);
        var r = VisionProcessor.RunProgramOnce(prog, img, out var annotated);
        Dump("photo1", prog, r, annotated);

        Assert.Empty(r.Errors);
        // The taught squares quad, carried by the homography, lands on photo 1's squares.
        var used = r.Tool("white")!.Region;
        _out.WriteLine("CORNERS " + string.Join(" ", Enumerable.Range(0, 4).Select(i => $"{Dist(One.Squares[i], used[i]):0.0}")));
        for (int i = 0; i < 4; i++)
        {
            float tol = 0.12f * (i < 2 ? 418f : 1270f) / 8;   // 12% of a square at that corner
            Assert.True(Dist(One.Squares[i], used[i]) < tol, $"corner {i}: {used[i].X:0.0},{used[i].Y:0.0} vs {One.Squares[i]} ({Dist(One.Squares[i], used[i]):0.0} px)");
        }
        var (white, black, _) = Read(r);
        Assert.Equal(One.White.OrderBy(x => x), white.OrderBy(x => x));
        Assert.Equal(One.Black.OrderBy(x => x), black.OrderBy(x => x));
    }

    [Fact]
    public void Grid_tracks_a_nudged_camera()
    {
        // Same board shifted 18 px right and 10 px down (a bumped camera); nothing is redrawn.
        var prog = CheckersProgram();
        using var img = Load(Two);
        VisionProcessor.RunProgramOnce(prog, img, out _);
        var reference = prog.Tool("board")!.Reference!.Quad!;

        using var moved = new Mat();
        using (var m = new Mat(2, 3, MatType.CV_64FC1))
        {
            m.Set(0, 0, 1.0); m.Set(0, 1, 0.0); m.Set(0, 2, 18.0);
            m.Set(1, 0, 0.0); m.Set(1, 1, 1.0); m.Set(1, 2, 10.0);
            Cv2.WarpAffine(img, moved, m, img.Size(), InterpolationFlags.Linear, BorderTypes.Replicate);
        }
        var r = VisionProcessor.RunProgramOnce(prog, moved, out var annotated);
        Dump("nudged", prog, r, annotated);

        var quad = r.Tool("board")!.Pose!.Quad!;
        for (int i = 0; i < 4; i++)
        {
            Assert.InRange(quad[i].X - reference[i].X, 18 - 3, 18 + 3);
            Assert.InRange(quad[i].Y - reference[i].Y, 10 - 3, 10 + 3);
        }
        var (white, black, _) = Read(r);
        Assert.Equal(Two.White.OrderBy(x => x), white.OrderBy(x => x));
        Assert.Equal(Two.Black.OrderBy(x => x), black.OrderBy(x => x));
    }

    [Fact]
    public void Board_edge_hidden_leaves_the_piece_tools_not_run()
    {
        // A hand over the board's top edge: the fixture is not found, so neither piece tool runs
        // on a guessed grid — they report notRun, naming the fixture.
        var prog = CheckersProgram();
        using var img = Load(Two);
        VisionProcessor.RunProgramOnce(prog, img, out _);
        using var hand = img.Clone();
        Cv2.Rectangle(hand, new Rect(440, 220, 420, 110), new Scalar(120, 150, 205), -1);

        var r = VisionProcessor.RunProgramOnce(prog, hand, out var annotated);
        Dump("hand", prog, r, annotated);

        Assert.NotEqual(ToolStatus.Ok, r.Tool("top")!.Status);
        Assert.Equal(ToolStatus.NotRun, r.Tool("board")!.Status);
        foreach (var id in new[] { "white", "black" })
        {
            var t = r.Tool(id)!;
            Assert.Equal(ToolStatus.NotRun, t.Status);
            Assert.False(t.Passed);
            Assert.Contains("Board", t.Message);
            Assert.Empty(t.Cells);
        }
        Assert.False(r.Passed);
    }
}
