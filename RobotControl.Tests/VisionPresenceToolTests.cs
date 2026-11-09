using Controller.RobotControl.Vision;
using OpenCvSharp;
using static RobotControl.Tests.VisionTestKit;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

// Presence and count tools (pixelCount, intensity, edgePixels, polygon): whole-region values,
// grids (rect, circle and perspective quad cells), limits, fixtures and input layers.

/// <summary>
/// A flat 200 × 200 board warped onto a perspective quad, with red dots in its corners that a
/// fourPoints fixture locates, so a quad region drawn on the board follows its perspective.
/// </summary>
internal static class Board
{
    public static readonly (double X, double Y)[] TeachQuad = { (110, 60), (290, 60), (290, 240), (110, 240) };
    public static readonly (double X, double Y)[] SeenQuad = { (130, 70), (280, 55), (320, 250), (90, 235) };

    /// <summary>Cells of the 4 × 4 grid: the board's inner square 30..170 split into 35 px cells.</summary>
    public const double Inner0 = 30, Cell = 35;

    public static Point2f[] Square => new[] { new Point2f(0, 0), new Point2f(200, 0), new Point2f(200, 200), new Point2f(0, 200) };

    /// <summary>A flat board of gray <paramref name="bg"/> with the four red corner dots.</summary>
    public static Mat Flat(int bg)
    {
        var flat = new Mat(200, 200, MatType.CV_8UC3, Scalar.All(bg));
        foreach (var (x, y) in new[] { (10, 10), (190, 10), (190, 190), (10, 190) }) Cv2.Circle(flat, new Point(x, y), 6, Red, -1);
        return flat;
    }

    /// <summary>The flat cell (row, col) rectangle, px.</summary>
    public static Rect CellRect(int row, int col, int margin = 0) =>
        new((int)(Inner0 + col * Cell) + margin, (int)(Inner0 + row * Cell) + margin, (int)Cell - 2 * margin, (int)Cell - 2 * margin);

    public static Mat Warp(Mat flat, (double X, double Y)[] quad)
    {
        var img = Canvas(40);
        using var m = Cv2.GetPerspectiveTransform(Square, quad.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray());
        Cv2.WarpPerspective(flat, img, m, img.Size(), InterpolationFlags.Linear, BorderTypes.Transparent);
        return img;
    }

    /// <summary>A flat board point as seen on a quad.</summary>
    public static (double X, double Y) Map((double X, double Y)[] quad, double u, double v)
    {
        using var m = Cv2.GetPerspectiveTransform(Square, quad.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray());
        var o = Cv2.PerspectiveTransform(new[] { new Point2f((float)u, (float)v) }, m)[0];
        return (o.X, o.Y);
    }

    /// <summary>The inner square (the gridded area) as seen on a quad.</summary>
    public static VisionRegion InnerQuad((double X, double Y)[] quad) => Quad(
        Map(quad, Inner0, Inner0), Map(quad, Inner0 + 4 * Cell, Inner0), Map(quad, Inner0 + 4 * Cell, Inner0 + 4 * Cell), Map(quad, Inner0, Inner0 + 4 * Cell));

    /// <summary>The red layer, four dot blobs (one per image quadrant) and a fourPoints fixture "fx", then the tool.</summary>
    public static VisionProgram Program(VisionTool tool)
    {
        var prog = VisionTestKit.Program(
            Tool("tl", "blob", Rect(0, 0, 200, 150), new { polarity = "light" }, input: "red"),
            Tool("tr", "blob", Rect(200, 0, 200, 150), new { polarity = "light" }, input: "red"),
            Tool("br", "blob", Rect(200, 150, 200, 150), new { polarity = "light" }, input: "red"),
            Tool("bl", "blob", Rect(0, 150, 200, 150), new { polarity = "light" }, input: "red"),
            Tool("fx", "makeFixture", null, new { mode = "fourPoints", sources = new[] { "tl", "tr", "br", "bl" } }),
            tool);
        prog.Layers.Add(RedLayer());
        return prog;
    }

    /// <summary>
    /// The tool gridded 4 × 4 over the board's inner square: taught on the square-on board, run
    /// on the perspective one, both fixtured and as a plain quad region drawn on the seen board.
    /// Asserts the passing cells are <paramref name="want"/> each time and returns the seen results.
    /// </summary>
    public static (ToolResult Fixtured, ToolResult Drawn) AssertGrid(Mat flat, VisionTool tool, ISet<(int, int)> want)
    {
        tool.Region = InnerQuad(TeachQuad);
        tool.FixtureId = "fx";
        tool.Grid ??= new VisionGrid { Rows = 4, Cols = 4, Inset = 0.2 };
        var prog = Program(tool);
        using (var teach = Warp(flat, TeachQuad))
        {
            var r0 = Run(prog, teach).Tool(tool.Id)!;
            Assert.True(r0.Ok, r0.Message);
            AssertCells(r0, want);
        }
        using var seen = Warp(flat, SeenQuad);
        var r1 = Run(prog, seen);
        Assert.Equal(4, r1.Tool("fx")!.Pose!.Quad!.Count);
        var fixtured = r1.Tool(tool.Id)!;
        Assert.True(fixtured.Ok, fixtured.Message);
        AssertCells(fixtured, want);
        // The region as used is the inner square as seen now.
        var inner = InnerQuad(SeenQuad).Points!;
        for (int i = 0; i < 4; i++) Near(fixtured.Region[i].X, fixtured.Region[i].Y, (inner[i].X * W, inner[i].Y * H), 1.5, $"region corner {i}");

        var drawn = new VisionTool
        {
            Id = tool.Id, Name = tool.Id, Type = tool.Type, Params = tool.Params, Region = InnerQuad(SeenQuad),
            Grid = tool.Grid, Limits = tool.Limits,
        };
        var r2 = RunOne(seen, drawn);
        AssertCells(r2, want);
        return (fixtured, r2);
    }

    public static void AssertCells(ToolResult r, ISet<(int, int)> want)
    {
        Assert.Equal(16, r.Cells.Count);
        Assert.Equal(want.OrderBy(x => x), r.Cells.Where(c => c.Passed).Select(c => (c.Row, c.Col)).OrderBy(x => x));
        Assert.Equal(want.Count, Num(r, "cellsPassed"));
        Assert.Equal(want.Count == 16, r.Passed);
        for (int i = 0; i < 16; i++)
        {
            Assert.Equal(i, r.Cells[i].Index);
            Assert.Equal(i / 4, r.Cells[i].Row);
            Assert.Equal(i % 4, r.Cells[i].Col);
            Assert.Equal(4, r.Cells[i].Quad!.Count);
        }
    }

    public static readonly HashSet<(int, int)> Some = new() { (0, 0), (0, 3), (1, 2), (2, 1), (3, 3) };
}

/// <summary>pixelCount: the percentage of matching pixels.</summary>
public class PixelCountToolTests
{
    /// <summary>Columns 0..99 red, the rest green (BGR).</summary>
    private static Mat RedGreen()
    {
        var img = Canvas();
        Box(img, 0, 0, 100, 300, new Scalar(0, 0, 255));
        Box(img, 100, 0, 300, 300, new Scalar(0, 255, 0));
        return img;
    }

    private static readonly object[] RedOnly = { new { r = 255, g = 0, b = 0, tolerance = 10 } };

    [Fact]
    public void Colour_percent_over_a_rect_is_exact()
    {
        using var img = RedGreen();
        var r = RunOne(img, Tool("p", "pixelCount", Rect(50, 50, 100, 100), new { mode = "color", colors = RedOnly }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(50, Num(r, "percent"), 2);               // pixels 50..149: half of them red
        Assert.Empty(r.Cells);
        Assert.False(r.Values.ContainsKey("cellsPassed"));
        var whole = RunOne(img, Tool("p", "pixelCount", null, new { colors = RedOnly }));   // color is the default mode
        Assert.Equal(25, Num(whole, "percent"), 2);
    }

    [Theory]
    // A (R 200, G 100, B 50) image against the target (210, 100, 50): ±round(2.55·tolerance) per channel.
    [InlineData(0, 0)]
    [InlineData(3, 0)]       // ±8: 10 away is out
    [InlineData(3.9, 100)]   // ±10
    [InlineData(4, 100)]
    [InlineData(100, 100)]
    public void Tolerance_widens_each_channel(double tolerance, double percent)
    {
        using var img = Canvas();
        img.SetTo(new Scalar(50, 100, 200));
        var r = RunOne(img, Tool("p", "pixelCount", null, new { mode = "color", colors = new[] { new { r = 210, g = 100, b = 50, tolerance } } }));
        Assert.Equal(percent, Num(r, "percent"), 2);
    }

    [Fact]
    public void An_exact_colour_matches_at_zero_tolerance_and_the_default_is_20()
    {
        using var img = Canvas();
        img.SetTo(new Scalar(50, 100, 200));
        Assert.Equal(100, Num(RunOne(img, Tool("p", "pixelCount", null, new { colors = new[] { new { r = 200, g = 100, b = 50, tolerance = 0 } } })), "percent"), 2);
        // No tolerance given: ±51.
        Assert.Equal(100, Num(RunOne(img, Tool("p", "pixelCount", null, new { colors = new[] { new { r = 250, g = 100, b = 50 } } })), "percent"), 2);
        Assert.Equal(0, Num(RunOne(img, Tool("p", "pixelCount", null, new { colors = new[] { new { r = 252, g = 100, b = 50 } } })), "percent"), 2);
    }

    [Theory]
    [InlineData(new[] { "red" }, 25)]
    [InlineData(new[] { "green" }, 75)]
    [InlineData(new[] { "red", "green" }, 100)]
    [InlineData(new[] { "blue" }, 0)]
    [InlineData(new string[0], 0)]
    public void Several_colours_add_up(string[] names, double percent)
    {
        using var img = RedGreen();
        var all = new Dictionary<string, object>
        {
            ["red"] = new { r = 255, g = 0, b = 0, tolerance = 5 },
            ["green"] = new { r = 0, g = 255, b = 0, tolerance = 5 },
            ["blue"] = new { r = 0, g = 0, b = 255, tolerance = 5 },
        };
        var r = RunOne(img, Tool("p", "pixelCount", null, new { mode = "color", colors = names.Select(n => all[n]).ToArray() }));
        Assert.Equal(percent, Num(r, "percent"), 2);
    }

    [Theory]
    // Stripes of gray 99, 100, 200, 201 — the bounds are inclusive.
    [InlineData(100, 200, 50)]
    [InlineData(99, 201, 100)]
    [InlineData(101, 199, 0)]
    [InlineData(0, 127, 50)]      // the default range
    public void Gray_mode_counts_levels_in_range(int grayMin, int grayMax, double percent)
    {
        using var img = Canvas();
        int[] levels = { 99, 100, 200, 201 };
        for (int i = 0; i < 4; i++) Box(img, i * 100, 0, 100, 300, Scalar.All(levels[i]));
        object p = grayMin == 0 && grayMax == 127 ? new { mode = "gray" } : new { mode = "gray", grayMin, grayMax };
        Assert.Equal(percent, Num(RunOne(img, Tool("p", "pixelCount", null, p)), "percent"), 2);
    }

    [Fact]
    public void Colour_on_a_single_channel_layer_compares_luminance()
    {
        using var img = Canvas(150);
        var prog = Program(
            Tool("same", "pixelCount", null, new { mode = "color", colors = new[] { new { r = 150, g = 150, b = 150, tolerance = 2 } } }, input: "g"),
            Tool("red", "pixelCount", null, new { mode = "color", colors = new[] { new { r = 255, g = 0, b = 0, tolerance = 2 } } }, input: "g"),
            Tool("lum", "pixelCount", null, new { mode = "color", colors = new[] { new { r = 0, g = 255, b = 0, tolerance = 2 } } }, input: "g"));
        prog.Layers.Add(new ImageLayer { Id = "g", Filters = { Filter("channel", new { channel = "gray" }) } });
        var r = Run(prog, img);
        Assert.Equal(100, Num(r.Tool("same")!, "percent"), 2);
        Assert.Equal(0, Num(r.Tool("red")!, "percent"), 2);     // luminance 76
        Assert.Equal(100, Num(r.Tool("lum")!, "percent"), 2);   // luminance 149.7: within ±5 of 150
    }

    [Fact]
    public void Circle_turned_rect_and_polygon_regions()
    {
        using var img = Canvas();
        Box(img, 0, 0, 200, 300, new Scalar(0, 0, 255));       // red left of x 200
        var red = new { mode = "color", colors = RedOnly };
        Assert.InRange(Num(RunOne(img, Tool("p", "pixelCount", Circle(200, 150, 50), red)), "percent"), 49, 51);
        Assert.InRange(Num(RunOne(img, Tool("p", "pixelCount", Centered(200, 150, 100, 60, 45), red)), "percent"), 49, 51);
        Assert.InRange(Num(RunOne(img, Tool("p", "pixelCount", Circle(100, 150, 50), red)), "percent"), 99.9, 100);
        // A triangle with its apex on x 200: all of it red.
        var tri = new VisionRegion { Shape = "polygon", Points = new() { new(100.0 / W, 50.0 / H), new(199.0 / W, 150.0 / H), new(100.0 / W, 250.0 / H) } };
        Assert.Equal(100, Num(RunOne(img, Tool("p", "pixelCount", tri, red)), "percent"), 1);
    }

    [Fact]
    public void A_grid_judges_each_cell_and_counts_the_passes()
    {
        using var img = Canvas(220);
        // Columns 0..3 of a 2 × 4 grid over the whole image, dark fill 0 %, 25 %, 50 %, 100 % on row 0.
        Box(img, 100, 0, 25, 150, Scalar.All(20));
        Box(img, 200, 0, 50, 150, Scalar.All(20));
        Box(img, 300, 0, 100, 150, Scalar.All(20));
        var limits = new Dictionary<string, VisionLimit> { ["percent"] = new() { Min = 40 } };
        var r = RunOne(img, Tool("p", "pixelCount", null, new { mode = "gray", grayMin = 0, grayMax = 100 }, grid: new VisionGrid { Rows = 2, Cols = 4 }, limits: limits));
        Assert.Equal(8, r.Cells.Count);
        Assert.Equal(new[] { 0.0, 25, 50, 100, 0, 0, 0, 0 }, r.Cells.Select(c => c.Value));
        Assert.Equal(new[] { false, false, true, true, false, false, false, false }, r.Cells.Select(c => c.Passed));
        Assert.Equal(2, Num(r, "cellsPassed"));
        Assert.Equal(new[] { 0, 1, 2, 3, 4, 5, 6, 7 }, r.Cells.Select(c => c.Index));
        Assert.Equal(new[] { 0, 0, 0, 0, 1, 1, 1, 1 }, r.Cells.Select(c => c.Row));
        Assert.Equal(21.875, Num(r, "percent"), 2);           // the whole region is still reported
        Assert.True(r.Ok);
        Assert.False(r.Passed);                               // the main-value limit means every cell
        // Cell quads: cell (1, 2) spans x 200..300, y 150..300.
        var q = r.Cells[6].Quad!;
        Assert.Equal((200.0, 150.0, 300.0, 300.0), (q[0].X, q[0].Y, q[2].X, q[2].Y));

        // All cells passing passes the tool.
        var easy = RunOne(img, Tool("p", "pixelCount", null, new { mode = "gray", grayMin = 0, grayMax = 255 }, grid: new VisionGrid { Rows = 2, Cols = 4 }, limits: limits));
        Assert.True(easy.Passed);
        Assert.Equal(8, Num(easy, "cellsPassed"));
        // Without a limit every cell passes.
        var free = RunOne(img, Tool("p", "pixelCount", null, new { mode = "gray", grayMin = 0, grayMax = 100 }, grid: new VisionGrid { Rows = 2, Cols = 4 }));
        Assert.All(free.Cells, c => Assert.True(c.Passed));
        Assert.True(free.Passed);
    }

    [Fact]
    public void Inset_trims_each_cell_before_measuring()
    {
        using var img = Canvas(220);
        Box(img, 0, 0, 10, 300, Scalar.All(20));               // a dark strip along the left edge of cell 0
        var plain = RunOne(img, Tool("p", "pixelCount", Rect(0, 0, 200, 100), new { mode = "gray", grayMax = 100 }, grid: new VisionGrid { Rows = 1, Cols = 2 }));
        var inset = RunOne(img, Tool("p", "pixelCount", Rect(0, 0, 200, 100), new { mode = "gray", grayMax = 100 }, grid: new VisionGrid { Rows = 1, Cols = 2, Inset = 0.2 }));
        Assert.Equal(10, plain.Cells[0].Value, 1);
        Assert.Equal(0, inset.Cells[0].Value, 1);              // 20 px trimmed off each side
        var q = inset.Cells[0].Quad!;
        Assert.Equal((20.0, 20.0, 80.0, 80.0), (q[0].X, q[0].Y, q[2].X, q[2].Y));
        // An inset beyond 0.45 is held at 0.45.
        var huge = RunOne(img, Tool("p", "pixelCount", Rect(0, 0, 200, 100), new { mode = "gray" }, grid: new VisionGrid { Rows = 1, Cols = 2, Inset = 0.9 }));
        Assert.Equal(45.0, huge.Cells[0].Quad![0].X, 1);
    }

    [Theory]
    [InlineData(1, 1, 0)]       // a 1 × 1 grid is no grid
    [InlineData(3, 2, 6)]
    [InlineData(100, 1, 64)]    // held at 64 per side
    public void Grid_size(int rows, int cols, int cells)
    {
        using var img = Canvas(220);
        var r = RunOne(img, Tool("p", "pixelCount", null, new { mode = "gray" }, grid: new VisionGrid { Rows = rows, Cols = cols }));
        Assert.Equal(cells, r.Cells.Count);
        Assert.Equal(cells > 0, r.Values.ContainsKey("cellsPassed"));
    }

    [Fact]
    public void A_circle_regions_cells_are_clipped_to_it()
    {
        using var img = Canvas(255);
        var r = RunOne(img, Tool("p", "pixelCount", Circle(200, 150, 100), new { mode = "gray", grayMin = 200, grayMax = 255 }, grid: new VisionGrid { Rows = 2, Cols = 2 }));
        Assert.All(r.Cells, c => Assert.InRange(c.Value, 99.9, 100));
        // On a dark image with a light ring just outside the circle, no cell sees the ring.
        using var ring = Canvas(0);
        Cv2.Circle(ring, new Point(200, 150), 120, Scalar.All(255), 30);
        var clipped = RunOne(ring, Tool("p", "pixelCount", Circle(200, 150, 100), new { mode = "gray", grayMin = 200, grayMax = 255 }, grid: new VisionGrid { Rows = 2, Cols = 2 }));
        Assert.All(clipped.Cells, c => Assert.Equal(0, c.Value));
    }

    [Fact]
    public void A_region_off_the_image_fails()
    {
        using var img = Canvas();
        var r = RunOne(img, Tool("p", "pixelCount", Rect(450, 350, 50, 50), new { mode = "gray" }));
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Contains("outside the image", r.Message);
        Assert.False(r.Passed);
    }

    [Fact]
    public void Limits_on_percent()
    {
        using var img = RedGreen();
        Assert.Equal(50, AssertLimitBoundaries(img, "p", "percent", Single(() => Tool("p", "pixelCount", Rect(50, 50, 100, 100), new { colors = RedOnly }))), 2);
    }

    [Fact]
    public void A_perspective_grid_follows_the_board()
    {
        using var flat = Board.Flat(220);
        foreach (var (row, col) in Board.Some)
        {
            var c = Board.CellRect(row, col);
            Cv2.Circle(flat, new Point(c.X + c.Width / 2, c.Y + c.Height / 2), 12, Scalar.All(30), -1);
        }
        var (fixtured, _) = Board.AssertGrid(flat, Tool("p", "pixelCount", null, new { mode = "gray", grayMin = 0, grayMax = 100 },
            limits: new() { ["percent"] = new() { Min = 50 } }), Board.Some);
        Assert.All(fixtured.Cells.Where(c => c.Passed), c => Assert.InRange(c.Value, 80, 100));   // the disc covers most of the inset window
        Assert.All(fixtured.Cells.Where(c => !c.Passed), c => Assert.InRange(c.Value, 0, 1));
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Canvas(220);
        Cv2.Circle(teach, new Point(200, 180), 20, Scalar.All(30), -1);
        Dots(teach);
        // Gray 0..125: the level halfway between the disc (30) and the background (220).
        var tool = Tool("p", "pixelCount", Centered(200, 180, 40, 40), new { mode = "gray", grayMin = 0, grayMax = 125 }, fixture: "fx");
        var still = Tool("still", "pixelCount", Centered(200, 180, 40, 40), new { mode = "gray", grayMin = 0, grayMax = 125 });
        var (r0, r1) = TeachThenMoved(FixturedProgram(tool, still), teach, 220);
        double p0 = Num(r0.Tool("p")!, "percent");
        Assert.InRange(p0, 76, 80);                            // π·20² of 40²
        // The moved scene is resampled, which softens the disc's rim by a fraction of a pixel.
        Assert.InRange(Num(r1.Tool("p")!, "percent"), p0 * 0.97, p0 * 1.02);
        Assert.True(Num(r1.Tool("still")!, "percent") < p0 - 20);
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("p", "pixelCount", Centered(200, 180, 40, 40), new { mode = "gray" }));

    [Fact]
    public void It_reads_its_input_layer()
    {
        using var img = Canvas(RedGray);
        Box(img, 0, 0, 100, 300, Red);
        var tool = Tool("p", "pixelCount", null, new { mode = "gray", grayMin = 200, grayMax = 255 });
        Assert.Equal(0, Num(RunOne(img, tool), "percent"), 2);
        tool.Input = "red";
        var prog = Program(tool);
        prog.Layers.Add(RedLayer());
        Assert.Equal(25, Num(Run(prog, img).Tool("p")!, "percent"), 1);
    }
}

/// <summary>intensity: gray statistics of the region or each cell.</summary>
public class IntensityToolTests
{
    [Fact]
    public void Reports_every_statistic()
    {
        using var img = Canvas(50);
        Box(img, 200, 0, 200, 300, Scalar.All(150));
        var r = RunOne(img, Tool("i", "intensity", Rect(100, 50, 200, 100)));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(100, Num(r, "mean"), 2);
        Assert.Equal(50, Num(r, "stdDev"), 2);
        Assert.Equal(50, Num(r, "min"));
        Assert.Equal(150, Num(r, "max"));
        Assert.Equal(100, Num(r, "contrast"));
        Assert.Empty(r.Cells);
    }

    /// <summary>Left cell uniform 50; right cell half 100, half 200 (a 1 × 2 grid over x 0..400).</summary>
    private static Mat TwoCells()
    {
        var img = Canvas(50);
        Box(img, 200, 0, 100, 300, Scalar.All(100));
        Box(img, 300, 0, 100, 300, Scalar.All(200));
        return img;
    }

    [Theory]
    [InlineData("mean", 50, 150)]
    [InlineData("stdDev", 0, 50)]
    [InlineData("min", 50, 100)]
    [InlineData("max", 50, 200)]
    [InlineData("contrast", 0, 100)]
    [InlineData("sparkle", 50, 150)]   // unknown: mean
    public void Cells_are_valued_and_judged_on_the_chosen_statistic(string statistic, double left, double right)
    {
        using var img = TwoCells();
        string key = statistic == "sparkle" ? "mean" : statistic;
        double cut = (left + right) / 2;
        var r = RunOne(img, Tool("i", "intensity", null, new { statistic }, grid: new VisionGrid { Rows = 1, Cols = 2 },
            limits: new() { [key] = new() { Max = cut } }));
        Assert.Equal(new[] { left, right }, r.Cells.Select(c => c.Value));
        Assert.Equal(new[] { true, false }, r.Cells.Select(c => c.Passed));
        Assert.Equal(1, Num(r, "cellsPassed"));
        Assert.False(r.Passed);
        // Every statistic is still reported for the whole region.
        Assert.Equal(100, Num(r, "mean"), 2);
        Assert.Equal(50, Num(r, "min"));
        Assert.Equal(200, Num(r, "max"));
        Assert.Equal(150, Num(r, "contrast"));
        // A limit on a statistic that is not the cells' main value judges the whole-region value.
        if (key != "mean")
        {
            var whole = RunOne(img, Tool("i", "intensity", null, new { statistic }, grid: new VisionGrid { Rows = 1, Cols = 2 },
                limits: new() { ["mean"] = new() { Min = 99, Max = 101 } }));
            Assert.True(whole.Passed);
            Assert.All(whole.Cells, c => Assert.True(c.Passed));
        }
    }

    /// <summary>Gray = column index (0..255) — or row index when <paramref name="rows"/>.</summary>
    private static Mat Ramp(bool rows = false)
    {
        var img = Canvas(0);
        if (rows) for (int y = 0; y < 256; y++) Cv2.Line(img, new Point(0, y), new Point(W - 1, y), Scalar.All(y));
        else for (int x = 0; x < 256; x++) Cv2.Line(img, new Point(x, 0), new Point(x, H - 1), Scalar.All(x));
        return img;
    }

    [Theory]
    [InlineData(10, 100, 3)]       // cell edges at 43.33 and 76.67
    [InlineData(10.3, 99.9, 7)]    // edges every 14.27 px, none on a whole pixel or a pixel centre
    [InlineData(20, 120, 4)]       // whole-pixel edges
    public void Cells_split_the_region_by_pixel_centres(double x0, double width, int cols)
    {
        // On a ramp a cell's min and max name its first and last column: a pixel belongs to the
        // cell holding its centre, so neighbours never share one.
        using var img = Ramp();
        var lo = RunOne(img, Tool("i", "intensity", Rect(x0, 10, width, 50), new { statistic = "min" }, grid: new VisionGrid { Rows = 1, Cols = cols }));
        var hi = RunOne(img, Tool("i", "intensity", Rect(x0, 10, width, 50), new { statistic = "max" }, grid: new VisionGrid { Rows = 1, Cols = cols }));
        Assert.Equal(Math.Ceiling(x0 - 0.5), Num(lo, "min"));
        Assert.Equal(Math.Ceiling(x0 + width - 0.5) - 1, Num(hi, "max"));
        for (int c = 0; c < cols; c++)
        {
            double left = x0 + width * c / cols, right = x0 + width * (c + 1) / cols;
            Assert.Equal(Math.Ceiling(left - 0.5), lo.Cells[c].Value);
            Assert.Equal(Math.Ceiling(right - 0.5) - 1, hi.Cells[c].Value);
        }
    }

    [Theory]
    [InlineData(100, 50)]     // 100 / 300 is not exact in binary
    [InlineData(70, 80)]
    [InlineData(33.3, 66.6)]
    public void A_rect_region_takes_exactly_the_rows_whose_centres_it_holds(double y0, double height)
    {
        using var img = Ramp(rows: true);
        var r = RunOne(img, Tool("i", "intensity", Rect(40, y0, 60, height)));
        Assert.Equal(Math.Ceiling(y0 - 0.5), Num(r, "min"));
        Assert.Equal(Math.Ceiling(y0 + height - 0.5) - 1, Num(r, "max"));
    }

    [Fact]
    public void A_circle_region_measures_only_its_disc()
    {
        using var img = Canvas(0);
        Cv2.Circle(img, new Point(200, 150), 60, Scalar.All(180), -1);
        var r = RunOne(img, Tool("i", "intensity", Circle(200, 150, 50)));
        Assert.Equal(180, Num(r, "mean"), 2);
        Assert.Equal(0, Num(r, "stdDev"), 2);
        Assert.Equal(0, Num(r, "contrast"));
    }

    [Fact]
    public void A_region_off_the_image_fails()
    {
        using var img = Canvas();
        var r = RunOne(img, Tool("i", "intensity", Rect(450, 350, 50, 50)));
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Contains("outside the image", r.Message);
        Assert.False(r.Values.ContainsKey("mean"));
    }

    [Fact]
    public void Limits_on_mean()
    {
        using var img = TwoCells();
        Assert.Equal(100, AssertLimitBoundaries(img, "i", "mean", Single(() => Tool("i", "intensity", null))), 2);
    }

    [Fact]
    public void A_perspective_grid_follows_the_board()
    {
        using var flat = Board.Flat(0);
        for (int row = 0; row < 4; row++)
            for (int col = 0; col < 4; col++)
                Cv2.Rectangle(flat, Board.CellRect(row, col), Scalar.All(40 + 10 * (row * 4 + col)), -1);
        var want = Enumerable.Range(8, 8).Select(i => (i / 4, i % 4)).ToHashSet();
        var (fixtured, drawn) = Board.AssertGrid(flat, Tool("i", "intensity", null, new { statistic = "mean" },
            limits: new() { ["mean"] = new() { Min = 115 } }), want);
        foreach (var r in new[] { fixtured, drawn })
            for (int i = 0; i < 16; i++) Assert.InRange(r.Cells[i].Value, 40 + 10 * i - 1.5, 40 + 10 * i + 1.5);
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Canvas(40);
        Box(teach, 180, 160, 40, 40, Scalar.All(200));
        Dots(teach);
        var (r0, r1) = TeachThenMoved(FixturedProgram(
            Tool("i", "intensity", Centered(199.5, 179.5, 30, 30), null, fixture: "fx"),
            Tool("still", "intensity", Centered(199.5, 179.5, 30, 30))), teach, 40);
        Assert.Equal(200, Num(r0.Tool("i")!, "mean"), 1);
        Assert.InRange(Num(r1.Tool("i")!, "mean"), 198, 200);
        Assert.InRange(Num(r1.Tool("i")!, "stdDev"), 0, 4);
        Assert.True(Num(r1.Tool("still")!, "mean") < 180);
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("i", "intensity", Centered(200, 180, 30, 30)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        using var img = Canvas(RedGray);
        Box(img, 0, 0, 200, 300, Red);
        var tool = Tool("i", "intensity", null);
        Assert.Equal(RedGray, Num(RunOne(img, tool), "mean"), 0);
        tool.Input = "red";
        var prog = Program(tool);
        prog.Layers.Add(RedLayer());
        Assert.Equal(127.5, Num(Run(prog, img).Tool("i")!, "mean"), 1);
    }
}

/// <summary>edgePixels: Canny edge density and the dominant edge direction.</summary>
public class EdgePixelsToolTests
{
    [Fact]
    public void Density_is_the_share_of_edge_pixels()
    {
        using var img = Canvas(40);
        Box(img, 200, 0, 200, 300, Scalar.All(200));           // one vertical step at x 199.5
        var r = RunOne(img, Tool("e", "edgePixels", Rect(150, 100, 100, 100)));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "density"), 2);                 // one column of 100 in 100 × 100
        Assert.Equal(90, Num(r, "angle"));
    }

    [Theory]
    [InlineData(0, 90)]      // a vertical edge runs at 90°
    [InlineData(90, 0)]      // a horizontal one at 0°
    [InlineData(45, 135)]
    [InlineData(-45, 45)]
    public void Angle_is_the_dominant_edge_direction(double turn, double want)
    {
        using var upright = Canvas(40);
        Box(upright, 200, -200, 400, 700, Scalar.All(200));
        using var img = Rotated(upright, turn, 200);
        var r = RunOne(img, Tool("e", "edgePixels", Centered(200, 150, 120, 120)));
        double a = Num(r, "angle");
        Assert.True(Math.Abs(AngleDiff(a * 2, want * 2)) / 2 <= 2, $"angle {a}, want {want}");
        Assert.InRange(a, 0, 179);
    }

    [Theory]
    [InlineData(50, 150, false)]   // a 40 → 60 step is too faint
    [InlineData(10, 30, true)]
    public void Canny_thresholds_decide_which_edges_count(double cannyLow, double cannyHigh, bool found)
    {
        using var img = Canvas(40);
        Box(img, 200, 0, 200, 300, Scalar.All(60));
        var r = RunOne(img, Tool("e", "edgePixels", Rect(150, 100, 100, 100), new { cannyLow, cannyHigh }));
        Assert.Equal(found ? 1 : 0, Num(r, "density"), 2);
        Assert.Equal(found, r.Values.ContainsKey("angle"));
    }

    [Fact]
    public void A_blank_region_has_no_edges_and_no_angle()
    {
        using var img = Canvas(90);
        var r = RunOne(img, Tool("e", "edgePixels"));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.Equal(0, Num(r, "density"));
        Assert.False(r.Values.ContainsKey("angle"));
        var off = RunOne(img, Tool("e", "edgePixels", Rect(450, 350, 50, 50)));
        Assert.Equal(ToolStatus.Fail, off.Status);
    }

    [Fact]
    public void A_grid_judges_each_cell_on_density()
    {
        using var img = Canvas(40);
        for (int x = 210; x < 400; x += 12) Box(img, x, 0, 6, 300, Scalar.All(200));   // stripes on the right half
        var r = RunOne(img, Tool("e", "edgePixels", null, null, grid: new VisionGrid { Rows = 2, Cols = 2 },
            limits: new() { ["density"] = new() { Min = 5 } }));
        Assert.Equal(new[] { 0.0, 0.0 }, new[] { r.Cells[0].Value, r.Cells[2].Value });
        Assert.All(new[] { r.Cells[1], r.Cells[3] }, c => Assert.InRange(c.Value, 15, 18));   // 32 edge columns of 200
        Assert.Equal(new[] { false, true, false, true }, r.Cells.Select(c => c.Passed));
        Assert.Equal(2, Num(r, "cellsPassed"));
        Assert.False(r.Passed);
        Assert.InRange(Num(r, "density"), 7.5, 9);
    }

    [Fact]
    public void Limits_on_density()
    {
        using var img = Canvas(40);
        Box(img, 200, 0, 200, 300, Scalar.All(200));
        Assert.Equal(1, AssertLimitBoundaries(img, "e", "density", Single(() => Tool("e", "edgePixels", Rect(150, 100, 100, 100)))), 2);
    }

    [Fact]
    public void A_perspective_grid_follows_the_board()
    {
        using var flat = Board.Flat(40);
        foreach (var (row, col) in Board.Some)
        {
            var c = Board.CellRect(row, col);
            for (int x = c.X + 2; x < c.Right - 2; x += 6) Cv2.Rectangle(flat, new Rect(x, c.Y, 3, c.Height), Scalar.All(220), -1);
        }
        Board.AssertGrid(flat, Tool("e", "edgePixels", null, null, limits: new() { ["density"] = new() { Min = 10 } }), Board.Some);
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Canvas(40);
        for (int x = 160; x < 240; x += 10) Box(teach, x, 140, 5, 80, Scalar.All(200));
        Dots(teach);
        var (r0, r1) = TeachThenMoved(FixturedProgram(
            Tool("e", "edgePixels", Centered(200, 180, 60, 60), null, fixture: "fx")), teach, 40);
        double d0 = Num(r0.Tool("e")!, "density");
        Assert.Equal(90, Num(r0.Tool("e")!, "angle"));
        var e = r1.Tool("e")!;
        Assert.InRange(Num(e, "density"), d0 * 0.8, d0 * 1.25);
        Assert.InRange(Num(e, "angle"), 90 + MoveDeg - 3, 90 + MoveDeg + 3);   // the stripes turned with the part
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("e", "edgePixels", Centered(200, 180, 60, 60)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        using var img = Canvas(RedGray);
        Box(img, 200, 0, 200, 300, Red);
        var tool = Tool("e", "edgePixels", Rect(150, 100, 100, 100));
        Assert.Equal(0, Num(RunOne(img, tool), "density"));
        tool.Input = "red";
        var prog = Program(tool);
        prog.Layers.Add(RedLayer());
        Assert.Equal(1, Num(Run(prog, img).Tool("e")!, "density"), 2);
    }
}

/// <summary>polygon: closed shapes with a given number of sides in a binary input.</summary>
public class PolygonToolTests
{
    private static (double X, double Y)[] Regular(double cx, double cy, double r, int n, double deg = -90) =>
        Enumerable.Range(0, n).Select(i => (cx + r * Math.Cos((deg + 360.0 * i / n) * Math.PI / 180), cy + r * Math.Sin((deg + 360.0 * i / n) * Math.PI / 180))).ToArray();

    /// <summary>A white triangle, square, pentagon and hexagon (radius 35) on black.</summary>
    private static Mat Shapes()
    {
        var img = Canvas(0);
        Poly(img, Scalar.All(255), Regular(70, 150, 35, 3));
        Poly(img, Scalar.All(255), Regular(160, 150, 35, 4, 45));
        Poly(img, Scalar.All(255), Regular(250, 150, 35, 5));
        Poly(img, Scalar.All(255), Regular(340, 150, 35, 6));
        return img;
    }

    [Theory]
    [InlineData(3, 70)]
    [InlineData(4, 160)]
    [InlineData(5, 250)]
    [InlineData(6, 340)]
    public void Counts_shapes_with_the_given_number_of_sides(int sides, double x)
    {
        using var img = Shapes();
        var r = RunOne(img, Tool("p", "polygon", null, new { sides }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "count"));
        var it = Assert.Single(r.Items);
        Assert.InRange(D(it["x"]), x - 1, x + 1);
        Assert.InRange(D(it["y"]), 148, 152);    // the triangle's centroid is its circumcentre too
        Assert.Equal(D(it["x"]), Num(r, "x"), 2);
        Assert.Null(r.Pose);
    }

    [Fact]
    public void Several_shapes_are_listed_largest_first()
    {
        using var img = Canvas(0);
        Box(img, 40, 40, 40, 40, Scalar.All(255));
        Box(img, 200, 100, 100, 100, Scalar.All(255));
        Box(img, 120, 200, 60, 60, Scalar.All(255));
        var r = RunOne(img, Tool("p", "polygon", null, new { sides = 4 }));
        Assert.Equal(3, Num(r, "count"));
        Assert.Equal(new[] { 99.0 * 99, 59 * 59, 39 * 39 }, r.Items.Select(i => D(i["area"])));
        Assert.Equal(249.5, Num(r, "x"), 2);
        Assert.Equal(149.5, Num(r, "y"), 2);
    }

    [Theory]
    [InlineData(20)]
    [InlineData(-30)]
    public void Angle_is_the_largest_shapes_turn_modulo_90(double deg)
    {
        using var img = Canvas(0);
        Poly(img, Scalar.All(255), Turned(200, 150, 100, 100, deg));
        var r = RunOne(img, Tool("p", "polygon", null, new { sides = 4 }));
        Assert.Equal(1, Num(r, "count"));
        double a = ((Num(r, "angle") % 90) + 90) % 90, want = ((deg % 90) + 90) % 90;
        Assert.InRange(a, want - 1, want + 1);
    }

    [Theory]
    [InlineData(0.04, 4)]
    [InlineData(0.002, 8)]   // fine enough to keep the chamfers
    public void Epsilon_sets_how_closely_the_outline_is_followed(double epsilon, int sides)
    {
        using var img = Canvas(0);
        Poly(img, Scalar.All(255), (110, 60), (290, 60), (300, 70), (300, 230), (290, 240), (110, 240), (100, 230), (100, 70));   // a square with 10 px chamfers
        Assert.Equal(1, Num(RunOne(img, Tool("p", "polygon", null, new { sides, epsilon })), "count"));
        Assert.Equal(0, Num(RunOne(img, Tool("p", "polygon", null, new { sides = sides == 4 ? 8 : 4, epsilon })), "count"));
    }

    [Theory]
    [InlineData(100.0, 1e9, 3)]
    [InlineData(2000.0, 1e9, 2)]
    [InlineData(100.0, 5000.0, 2)]
    [InlineData(2000.0, 5000.0, 1)]
    [InlineData(1e6, 1e9, 0)]
    public void Area_limits_filter_shapes(double minArea, double maxArea, int count)
    {
        using var img = Canvas(0);
        Box(img, 40, 40, 30, 30, Scalar.All(255));      // 841
        Box(img, 120, 40, 60, 60, Scalar.All(255));     // 3481
        Box(img, 220, 40, 100, 100, Scalar.All(255));   // 9801
        Assert.Equal(count, Num(RunOne(img, Tool("p", "polygon", null, new { sides = 4, minArea, maxArea })), "count"));
    }

    [Theory]
    [InlineData(127, 0)]
    [InlineData(128, 1)]
    [InlineData(255, 1)]
    public void Objects_are_pixels_brighter_than_127(int level, int count)
    {
        using var img = Canvas(0);
        Box(img, 150, 100, 100, 100, Scalar.All(level));
        Assert.Equal(count, Num(RunOne(img, Tool("p", "polygon", null, new { sides = 4 })), "count"));
    }

    [Fact]
    public void None_is_ok_with_a_zero_count()
    {
        using var img = Canvas(0);
        var r = RunOne(img, Tool("p", "polygon", null, new { sides = 4 }));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.True(r.Passed);
        Assert.Equal(0, Num(r, "count"));
        Assert.False(r.Values.ContainsKey("x"));
        Assert.Empty(r.Items);
        Assert.Equal(ToolStatus.Fail, RunOne(img, Tool("p", "polygon", Rect(450, 350, 50, 50))).Status);
    }

    [Fact]
    public void A_grid_counts_shapes_per_cell()
    {
        using var img = Canvas(0);
        Box(img, 40, 40, 60, 60, Scalar.All(255));
        Box(img, 100 + 200, 40, 60, 60, Scalar.All(255));
        Box(img, 100 + 200, 200, 30, 30, Scalar.All(255));
        Box(img, 340, 240, 30, 30, Scalar.All(255));
        var r = RunOne(img, Tool("p", "polygon", null, new { sides = 4 }, grid: new VisionGrid { Rows = 2, Cols = 2 },
            limits: new() { ["count"] = new() { Min = 1, Max = 1 } }));
        Assert.Equal(new[] { 1.0, 1, 0, 2 }, r.Cells.Select(c => c.Value));
        Assert.Equal(new[] { true, true, false, false }, r.Cells.Select(c => c.Passed));
        Assert.Equal(2, Num(r, "cellsPassed"));
        Assert.Equal(4, Num(r, "count"));                     // the whole region
        Assert.Equal(4, r.Items.Count);
        Assert.False(r.Passed);
    }

    [Fact]
    public void Limits_on_count()
    {
        using var img = Shapes();
        Assert.Equal(1, AssertLimitBoundaries(img, "p", "count", Single(() => Tool("p", "polygon", null, new { sides = 5 }))));
    }

    [Fact]
    public void A_perspective_grid_follows_the_board()
    {
        using var flat = Board.Flat(0);
        for (int row = 0; row < 4; row++)
            for (int col = 0; col < 4; col++)
            {
                var c = Board.CellRect(row, col, 9);                   // a 17 px shape in the middle of every cell
                if (Board.Some.Contains((row, col))) Cv2.Rectangle(flat, c, Scalar.All(255), -1);
                else Cv2.FillPoly(flat, new[] { new[] { new Point(c.X, c.Bottom), new Point(c.Right, c.Bottom), new Point(c.X + c.Width / 2, c.Y) } }, Scalar.All(255));
            }
        var (fixtured, _) = Board.AssertGrid(flat, Tool("p", "polygon", null, new { sides = 4, minArea = 30 },
            grid: new VisionGrid { Rows = 4, Cols = 4, Inset = 0.05 }, limits: new() { ["count"] = new() { Min = 1 } }), Board.Some);
        Assert.Equal(Board.Some.Count, Num(fixtured, "count"));
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Canvas(0);
        Box(teach, 170, 150, 60, 60, Scalar.All(255));
        Dots(teach);
        var (r0, r1) = TeachThenMoved(FixturedProgram(
            Tool("p", "polygon", Centered(200, 180, 90, 90), new { sides = 4 }, fixture: "fx"),
            Tool("still", "polygon", Centered(200, 180, 70, 70), new { sides = 4 })), teach, 0);
        Assert.Equal(1, Num(r0.Tool("still")!, "count"));
        var p = r1.Tool("p")!;
        Assert.Equal(1, Num(p, "count"));
        Near(Num(p, "x"), Num(p, "y"), Moved(199.5, 179.5), 1);
        // The unmoved region cuts the moved square: whatever it sees is not the square.
        var still = r1.Tool("still")!;
        var m = Moved(199.5, 179.5);
        Assert.False(Num(still, "count") == 1 && Math.Abs(Num(still, "x") - m.X) < 1 && Math.Abs(Num(still, "y") - m.Y) < 1);
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("p", "polygon", Centered(200, 180, 90, 90)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        // Dark squares on light: nothing until a layer inverts them.
        using var img = Canvas(230);
        Box(img, 60, 100, 80, 80, Scalar.All(20));
        Box(img, 240, 100, 80, 80, Scalar.All(20));
        var tool = Tool("p", "polygon", null, new { sides = 4, maxArea = 50000 });
        Assert.Equal(0, Num(RunOne(img, tool), "count"));
        tool.Input = "inv";
        var prog = Program(tool);
        prog.Layers.Add(new ImageLayer { Id = "inv", Filters = { Filter("invert") } });
        Assert.Equal(2, Num(Run(prog, img).Tool("p")!, "count"));
    }
}

/// <summary>Grid cells and region masks: every pixel of the region in exactly one cell.</summary>
public class GridCellTests
{
    public static IEnumerable<object[]> Regions() => new[]
    {
        new object[] { "rect", 3, 3 },
        new object[] { "turned", 4, 5 },
        new object[] { "quad", 4, 4 },
        new object[] { "fractional", 7, 3 },
    };

    private static PixelRegion Make(string kind) => kind switch
    {
        "rect" => PixelRegion.Rect(200, 150, 150, 90, 0),
        "turned" => PixelRegion.Rect(200, 150, 160, 100, 27),
        "quad" => PixelRegion.Quad(new[] { new Point2d(130, 70), new Point2d(280, 55), new Point2d(320, 250), new Point2d(90, 235) }),
        _ => PixelRegion.Rect(201.37, 149.71, 99.4, 61.3, 0),
    };

    [Theory]
    [MemberData(nameof(Regions))]
    public void Cells_cover_the_region_once(string kind, int rows, int cols)
    {
        var region = Make(kind);
        using var whole = new Mat(H, W, MatType.CV_8UC1, Scalar.Black);
        region.Fill(whole, new Point(0, 0));
        using var union = new Mat(H, W, MatType.CV_8UC1, Scalar.Black);
        int sum = 0;
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
            {
                var (m, box) = Geo.QuadMask(region.Cell(rows, cols, r, c, 0), W, H);
                using (m)
                {
                    sum += Cv2.CountNonZero(m);
                    using var roi = new Mat(union, box);
                    Cv2.BitwiseOr(roi, m, roi);
                }
            }
        int area = Cv2.CountNonZero(whole);
        Assert.Equal(area, sum);                          // no pixel in two cells…
        Assert.Equal(area, Cv2.CountNonZero(union));     // …and none left out
        using var diff = new Mat();
        Cv2.BitwiseXor(whole, union, diff);
        Assert.Equal(0, Cv2.CountNonZero(diff));
    }

    [Theory]
    [MemberData(nameof(Regions))]
    public void A_mask_does_not_depend_on_where_it_starts(string kind, int rows, int cols)
    {
        _ = rows; _ = cols;
        var region = Make(kind);
        using var full = new Mat(H, W, MatType.CV_8UC1, Scalar.Black);
        region.Fill(full, new Point(0, 0));
        var (cut, box) = Controller.RobotControl.Vision.Tools.ToolMath.RegionMask(region, W, H);
        using (cut)
        {
            using var same = new Mat(full, box);
            using var diff = new Mat();
            Cv2.BitwiseXor(same, cut, diff);
            Assert.Equal(0, Cv2.CountNonZero(diff));
            Assert.Equal(Cv2.CountNonZero(full), Cv2.CountNonZero(cut));
        }
    }

    [Fact]
    public void A_rect_mask_holds_the_pixels_whose_centres_are_inside()
    {
        // x 100.2..149.8, y 60.5..90.5: columns 100..149 (centre 149.5 < 149.8), rows 60..89.
        var region = PixelRegion.Rect(125, 75.5, 49.6, 30, 0);
        var (m, box) = Controller.RobotControl.Vision.Tools.ToolMath.RegionMask(region, W, H);
        using (m)
        {
            Assert.Equal(50 * 30, Cv2.CountNonZero(m));
            using var nz = new Mat();
            Cv2.FindNonZero(m, nz);
            nz.GetArray(out Point[] pts);
            Assert.Equal(100, pts.Min(p => p.X) + box.X);
            Assert.Equal(149, pts.Max(p => p.X) + box.X);
            Assert.Equal(60, pts.Min(p => p.Y) + box.Y);
            Assert.Equal(89, pts.Max(p => p.Y) + box.Y);
        }
    }
}
