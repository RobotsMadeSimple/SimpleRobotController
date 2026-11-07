using Controller.RobotControl.Vision;
using OpenCvSharp;
using static RobotControl.Tests.VisionTestKit;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

// Measuring tools (caliper, distance, angle, corners, lines) on synthetic images. Edges of an
// unsmoothed fill follow OpenCV's pixel convention: a fill over columns 180..209 has its edges at
// x = 179.5 and 209.5.

/// <summary>caliper: edges along a rect's width.</summary>
public class CaliperToolTests
{
    /// <summary>40 left of x 180, a 200 bar over 180..209, 120 right of it: steps +160 at 179.5, −80 at 209.5.</summary>
    private static Mat Bar()
    {
        var img = Canvas(40);
        Box(img, 180, 0, 30, 300, Scalar.All(200));
        Box(img, 210, 0, 190, 300, Scalar.All(120));
        return img;
    }

    [Theory]
    [InlineData("darkToLight", "strongest", 179.5, 160)]
    [InlineData("lightToDark", "strongest", 209.5, -80)]
    [InlineData("any", "strongest", 179.5, 160)]
    [InlineData("any", "first", 179.5, 160)]
    [InlineData("any", "last", 209.5, -80)]
    [InlineData("any", "center", 209.5, -80)]     // the region's middle is x 200
    [InlineData("darkToLight", "last", 179.5, 160)]
    public void Single_picks_one_edge(string polarity, string pick, double x, double contrast)
    {
        using var img = Bar();
        var r = RunOne(img, Tool("c", "caliper", Centered(200, 150, 200, 40), new { mode = "single", polarity, pick }));
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "x"), x - 0.5, x + 0.5);
        Assert.InRange(Num(r, "y"), 149.5, 150.5);
        Assert.InRange(Num(r, "contrast"), contrast - 4, contrast + 4);
        Assert.Equal(polarity == "any" ? 2 : 1, Num(r, "count"));
        var p = Assert.Single(r.Points);
        Assert.Equal(Num(r, "x"), p.X, 1);
        Assert.Null(r.Pose);                                    // a caliper is no fixture
    }

    [Fact]
    public void Single_is_the_default_mode_and_strongest_the_default_pick()
    {
        using var img = Bar();
        var r = RunOne(img, Tool("c", "caliper", Centered(200, 150, 200, 40)));
        Assert.InRange(Num(r, "x"), 179, 180);
        Assert.True(r.Values.ContainsKey("contrast"));
    }

    /// <summary>
    /// Light bars on 40: A x 30..39 (120), B 120..139 (230, the strongest), C 190..209 (100, round
    /// the middle), D 300..329 (150, the last).
    /// </summary>
    private static Mat Bars()
    {
        var img = Canvas(40);
        Box(img, 30, 0, 10, 300, Scalar.All(120));
        Box(img, 120, 0, 20, 300, Scalar.All(230));
        Box(img, 190, 0, 20, 300, Scalar.All(100));
        Box(img, 300, 0, 30, 300, Scalar.All(150));
        return img;
    }

    [Theory]
    [InlineData("first", 34.5, 10)]
    [InlineData("strongest", 129.5, 20)]
    [InlineData("center", 199.5, 20)]
    [InlineData("last", 314.5, 30)]
    public void Pair_measures_a_bar_and_pick_chooses_which(string pick, double x, double width)
    {
        using var img = Bars();
        var r = RunOne(img, Tool("c", "caliper", Centered(200, 150, 380, 40), new { mode = "pair", polarity = "darkToLight", pick }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(4, Num(r, "count"));
        Assert.InRange(Num(r, "x"), x - 0.5, x + 0.5);
        Assert.InRange(Num(r, "width"), width - 0.5, width + 0.5);
        Assert.Equal(2, r.Points.Count);                        // its two edges
        Assert.InRange(r.Points[0].X, x - width / 2 - 0.5, x - width / 2 + 0.5);
        Assert.InRange(r.Points[1].X, x + width / 2 - 0.5, x + width / 2 + 0.5);
    }

    [Theory]
    [InlineData(30, 3, "strongest", 314.5)]
    [InlineData(20, 3, "strongest", 129.5)]
    [InlineData(20, 3, "last", 199.5)]
    [InlineData(10, 1, "first", 34.5)]
    [InlineData(15, 2, "first", double.NaN)]   // no bar 13–17 wide
    public void PairWidth_keeps_bars_of_the_expected_width(double pairWidth, double pairTolerance, string pick, double x)
    {
        using var img = Bars();
        var r = RunOne(img, Tool("c", "caliper", Centered(200, 150, 380, 40), new { mode = "pair", polarity = "darkToLight", pick, pairWidth, pairTolerance }));
        if (double.IsNaN(x))
        {
            Assert.Equal(ToolStatus.Fail, r.Status);
            Assert.Equal("no edge pair found", r.Message);
            Assert.Equal(0, Num(r, "count"));
            return;
        }
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "x"), x - 0.5, x + 0.5);
        Assert.InRange(Num(r, "width"), pairWidth - pairTolerance, pairWidth + pairTolerance);
    }

    [Fact]
    public void Pair_polarity_sets_which_edge_opens_the_pair()
    {
        using var img = Canvas(200);
        Box(img, 100, 0, 25, 300, Scalar.All(40));             // a dark bar 100..124
        var region = Centered(150, 150, 200, 30);
        var dark = RunOne(img, Tool("c", "caliper", region, new { mode = "pair", polarity = "lightToDark" }));
        Assert.True(dark.Ok, dark.Message);
        Assert.InRange(Num(dark, "width"), 24.5, 25.5);
        Assert.InRange(Num(dark, "x"), 111.5, 112.5);
        // Rising first: the rising edge at 124.5 has no falling edge after it.
        var light = RunOne(img, Tool("c", "caliper", region, new { mode = "pair", polarity = "darkToLight" }));
        Assert.Equal(ToolStatus.Fail, light.Status);
        Assert.True(RunOne(img, Tool("c", "caliper", region, new { mode = "pair", polarity = "any" })).Ok);
    }

    [Fact]
    public void All_reports_every_edge_in_order()
    {
        using var img = Bars();
        var r = RunOne(img, Tool("c", "caliper", Centered(200, 150, 380, 40), new { mode = "all", polarity = "any" }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(8, Num(r, "count"));
        var want = new[] { 29.5, 39.5, 119.5, 139.5, 189.5, 209.5, 299.5, 329.5 };
        Assert.Equal(8, r.Points.Count);
        for (int i = 0; i < 8; i++) Assert.InRange(r.Points[i].X, want[i] - 0.5, want[i] + 0.5);
        Assert.Equal(r.Points[0].X, Num(r, "x"), 1);           // x/y = the first edge
        var rising = RunOne(img, Tool("c", "caliper", Centered(200, 150, 380, 40), new { mode = "all", polarity = "darkToLight" }));
        Assert.Equal(4, Num(rising, "count"));
    }

    [Theory]
    [InlineData(0, 179.5)]
    [InlineData(180, 209.5)]   // turned round, the profile runs right → left
    public void The_profile_runs_along_the_regions_width(double rotation, double firstX)
    {
        using var img = Bar();
        var r = RunOne(img, Tool("c", "caliper", Centered(195, 150, 100, 30, rotation), new { mode = "single", pick = "first" }));
        Assert.InRange(Num(r, "x"), firstX - 0.5, firstX + 0.5);
    }

    [Fact]
    public void A_turned_region_measures_a_turned_bar()
    {
        // A bar over columns 185..214 (centre 199.5, 30 wide) turned 30° about the image centre.
        using var upright = Canvas(40);
        Box(upright, 185, -200, 30, 700, Scalar.All(200));
        using var img = Rotated(upright, 30, 40);
        var centre = TurnPoint(199.5, 150, 200, 150, 30);
        var r = RunOne(img, Tool("c", "caliper", Centered(centre.X, centre.Y, 140, 30, 30), new { mode = "pair", polarity = "darkToLight" }));
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "width"), 29.6, 30.4);
        Near(Num(r, "x"), Num(r, "y"), centre, 0.6);

        using var flat = Canvas(40);
        Box(flat, 0, 140, 400, 20, Scalar.All(200));                   // rows 140..159
        var down = RunOne(flat, Tool("c", "caliper", Centered(200, 150, 100, 30, 90), new { mode = "pair", polarity = "darkToLight" }));
        Assert.InRange(Num(down, "width"), 19.5, 20.5);
        Near(Num(down, "x"), Num(down, "y"), (200, 149.5), 0.6);
    }

    [Theory]
    [InlineData(20, false)]
    [InlineData(10, true)]
    public void Contrast_sets_the_weakest_edge_taken(double contrast, bool found)
    {
        using var img = Canvas(40);
        Box(img, 200, 0, 200, 300, Scalar.All(55));
        var r = RunOne(img, Tool("c", "caliper", Centered(200, 150, 100, 30), new { contrast }));
        Assert.Equal(found ? ToolStatus.Ok : ToolStatus.Fail, r.Status);
        if (found) Assert.InRange(Num(r, "x"), 199, 200);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(3)]
    public void Smoothing_does_not_move_a_clean_edge(double smoothing)
    {
        using var img = Bar();
        var r = RunOne(img, Tool("c", "caliper", Centered(200, 150, 200, 40), new { polarity = "darkToLight", smoothing }));
        Assert.InRange(Num(r, "x"), 179, 180);
    }

    [Theory]
    [InlineData("single", "no edge found")]
    [InlineData("all", "no edge found")]
    [InlineData("pair", "no edge pair found")]
    public void A_blank_region_finds_nothing(string mode, string message)
    {
        using var img = Canvas(100);
        var r = RunOne(img, Tool("c", "caliper", Centered(200, 150, 200, 40), new { mode }));
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Equal(message, r.Message);
        Assert.Equal(0, Num(r, "count"));
        Assert.Empty(r.Points);
        Assert.False(r.Passed);
    }

    [Fact]
    public void Limits_on_width()
    {
        using var img = Bars();
        double v = AssertLimitBoundaries(img, "c", "width",
            Single(() => Tool("c", "caliper", Centered(200, 150, 380, 40), new { mode = "pair", polarity = "darkToLight" })));
        Assert.InRange(v, 19.5, 20.5);
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Canvas(40);
        Box(teach, 185, 100, 30, 160, Scalar.All(200));                // x 185..214: centre 199.5
        Dots(teach);
        var prog = FixturedProgram(Tool("c", "caliper", Centered(200, 180, 120, 30), new { mode = "pair", polarity = "darkToLight" }, fixture: "fx"));
        var (r0, r1) = TeachThenMoved(prog, teach, 40);
        Assert.InRange(Num(r0.Tool("c")!, "width"), 29.5, 30.5);
        var c = r1.Tool("c")!;
        Assert.True(c.Ok, c.Message);
        Assert.InRange(Num(c, "width"), 29.3, 30.7);
        Near(Num(c, "x"), Num(c, "y"), Moved(199.5, 180), 1);
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("c", "caliper", Centered(200, 180, 120, 30)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        using var img = Canvas(RedGray);
        Box(img, 180, 0, 30, 300, Red);
        var tool = Tool("c", "caliper", Centered(200, 150, 200, 40), new { mode = "pair", polarity = "darkToLight" });
        Assert.Equal(ToolStatus.Fail, RunOne(img, tool).Status);
        tool.Input = "red";
        var prog = Program(tool);
        prog.Layers.Add(RedLayer());
        var r = Run(prog, img).Tool("c")!;
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "width"), 29.5, 30.5);
    }
}

/// <summary>distance: between two tools' points, or from a point to a line.</summary>
public class DistanceToolTests
{
    /// <summary>Light dots at (100, 100) and (250, 180); light below y 250 (edge at 249.5).</summary>
    private static Mat Scene()
    {
        var img = Canvas(40);
        Cv2.Circle(img, new Point(100, 100), 8, Scalar.All(200), -1);
        Cv2.Circle(img, new Point(250, 180), 8, Scalar.All(200), -1);
        Box(img, 0, 250, 400, 50, Scalar.All(200));
        return img;
    }

    private static VisionTool[] Sources() => new[]
    {
        Tool("a", "blob", Centered(100, 100, 40, 40), new { polarity = "light" }),
        Tool("b", "blob", Centered(250, 180, 40, 40), new { polarity = "light" }),
        Tool("floor", "findLine", Centered(200, 250, 300, 30), new { polarity = "darkToLight" }),
        Tool("floorRight", "findLine", Centered(340, 250, 80, 30), new { polarity = "darkToLight" }),   // a short segment far from a
        Tool("ang", "angle", null, new { sources = new[] { "floor" } }),                             // lines only, no point
        Tool("cal", "caliper", Centered(250, 180, 60, 10), new { mode = "single", pick = "first" }),  // points only
        Tool("gone", "blob", Rect(150, 20, 60, 40), new { polarity = "light" }),                      // finds nothing (ok, no point)
        Tool("broken", "findLine", Centered(200, 120, 60, 20)),                                        // fails
    };

    private static ToolResult Measure(object p, out VisionResult all, Mat? img = null)
    {
        using var scene = Scene();
        var tools = Sources().ToList();
        tools.Add(Tool("d", "distance", null, p));
        all = Run(Program(tools.ToArray()), img ?? scene);
        return all.Tool("d")!;
    }

    [Fact]
    public void PointPoint_measures_between_two_points()
    {
        var r = Measure(new { mode = "pointPoint", sources = new[] { "a", "b" } }, out _);
        Assert.True(r.Ok, r.Message);
        Assert.Equal(170, Num(r, "distance"), 1);              // √(150² + 80²)
        Assert.Equal(150, Num(r, "dx"), 1);
        Assert.Equal(80, Num(r, "dy"), 1);
        Assert.Equal(2, r.Points.Count);
        var l = Assert.Single(r.Lines);
        Assert.Equal((100.0, 100.0, 250.0, 180.0), (Math.Round(l.X1), Math.Round(l.Y1), Math.Round(l.X2), Math.Round(l.Y2)));

        var back = Measure(new { sources = new[] { "b", "a" } }, out _);   // pointPoint is the default
        Assert.Equal(170, Num(back, "distance"), 1);
        Assert.Equal(-150, Num(back, "dx"), 1);
        Assert.Equal(-80, Num(back, "dy"), 1);
    }

    [Fact]
    public void PointLine_measures_to_the_infinite_line()
    {
        var r = Measure(new { mode = "pointLine", sources = new[] { "a", "floor" } }, out _);
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "distance"), 149, 150);          // 249.5 − 100
        Assert.InRange(Num(r, "dx"), -0.2, 0.2);
        Assert.InRange(Num(r, "dy"), 149, 150);
        Assert.InRange(r.Points[1].X, 99.8, 100.2);            // the foot of the perpendicular
        // A short segment far to the right measures the same: the line is extended.
        var far = Measure(new { mode = "pointLine", sources = new[] { "a", "floorRight" } }, out _);
        Assert.InRange(Num(far, "distance"), 149, 150);
    }

    [Fact]
    public void PointLine_to_a_slanted_line()
    {
        using var img = FindLineToolTests.Edge(20);
        Cv2.Circle(img, new Point(100, 60), 8, Scalar.All(200), -1);
        var r = Run(Program(
            Tool("a", "blob", Centered(100, 60, 40, 40), new { polarity = "light" }),
            Tool("edge", "findLine", Centered(220, 157, 160, 80, 20), new { polarity = "darkToLight" }),
            Tool("d", "distance", null, new { mode = "pointLine", sources = new[] { "a", "edge" } })), img).Tool("d")!;
        // The line through (200, 150) at 20°: |(−100, −90) × (cos 20°, sin 20°)| = 50.37.
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "distance"), 49.4, 51.4);
        // The step from the point to its foot is perpendicular to the line.
        Assert.InRange(Math.Atan2(Num(r, "dy"), Num(r, "dx")) * 180 / Math.PI, 110 - 1, 110 + 1);
    }

    [Fact]
    public void Points_come_from_a_pose_a_first_point_or_x_y_values()
    {
        var cal = Measure(new { sources = new[] { "a", "cal" } }, out var all);
        Assert.True(cal.Ok, cal.Message);
        var p = all.Tool("cal")!.Points[0];
        Assert.Equal(p.X - 100, Num(cal, "dx"), 1);
        using var img = Scene();
        Box(img, 300, 60, 40, 40, Scalar.All(200));            // a square: polygon gives x/y values only
        var tools = Sources().ToList();
        tools.Add(Tool("sq", "polygon", Rect(280, 40, 80, 80), new { sides = 4 }));
        tools.Add(Tool("d", "distance", null, new { sources = new[] { "a", "sq" } }));
        var r = Run(Program(tools.ToArray()), img).Tool("d")!;
        Assert.True(r.Ok, r.Message);
        Assert.Equal(219.5, Num(r, "dx"), 1);                  // the square's centre is (319.5, 79.5)
        Assert.Equal(-20.5, Num(r, "dy"), 1);
    }

    [Theory]
    [InlineData("pointPoint", new[] { "a" }, "needs two sources")]
    [InlineData("pointPoint", new string[0], "needs two sources")]
    [InlineData("pointPoint", new[] { "ang", "b" }, "source 1 has no point")]
    [InlineData("pointPoint", new[] { "a", "ang" }, "source 2 has no point")]
    [InlineData("pointPoint", new[] { "a", "gone" }, "source 2 has no point")]
    [InlineData("pointLine", new[] { "a", "b" }, "source 2 has no line")]
    public void Missing_geometry_fails(string mode, string[] sources, string message)
    {
        var r = Measure(new { mode, sources }, out _);
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Equal(message, r.Message);
        Assert.False(r.Passed);
    }

    [Fact]
    public void A_failed_or_unknown_source_leaves_it_not_run()
    {
        var failed = Measure(new { sources = new[] { "a", "broken" } }, out var all);
        Assert.Equal(ToolStatus.Fail, all.Tool("broken")!.Status);
        Assert.Equal(ToolStatus.NotRun, failed.Status);
        Assert.Contains("source 'broken' not found", failed.Message);
        Assert.Empty(failed.Values);
        var unknown = Measure(new { sources = new[] { "a", "nope" } }, out _);
        Assert.Equal(ToolStatus.NotRun, unknown.Status);
        Assert.Contains("not an earlier tool", unknown.Message);
    }

    [Fact]
    public void Limits_on_distance()
    {
        using var img = Scene();
        double v = AssertLimitBoundaries(img, "d", "distance", (lim, inc) =>
        {
            var tools = Sources().Where(t => t.Id != "broken").ToList();
            tools.Add(Tool("d", "distance", null, new { sources = new[] { "a", "b" } }, limits: lim, includeInPass: inc));
            return Program(tools.ToArray());
        });
        Assert.Equal(170, v, 1);
    }
}

/// <summary>angle: a line's angle, or the angle from line A to line B.</summary>
public class AngleToolTests
{
    [Theory]
    [InlineData(-30)]
    [InlineData(0)]
    [InlineData(25)]
    public void One_line_is_measured_against_the_x_axis(double deg)
    {
        using var img = FindLineToolTests.Edge(deg);
        var r = Run(Program(
            Tool("l", "findLine", Centered(200, 150, 200, 160), new { polarity = "darkToLight" }),
            Tool("a", "angle", null, new { sources = new[] { "l" } })), img).Tool("a")!;
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "angle"), deg - 0.3, deg + 0.3);
        Assert.Single(r.Lines);
    }

    /// <summary>Light below y 250, and light above a 30° edge through (200, 40 + 100·tan 30°).</summary>
    private static Mat TwoEdges()
    {
        var img = Canvas(40);
        double t = Math.Tan(30 * Math.PI / 180);
        Box(img, 0, 250, 400, 50, Scalar.All(200));
        Poly(img, Scalar.All(200), (100, 40), (300, 40 + 200 * t), (300, 0), (100, 0));
        return img;
    }

    private static VisionTool[] Lines(params VisionTool[] more) => new[]
    {
        Tool("floor", "findLine", Centered(200, 250, 300, 30), new { polarity = "darkToLight" }),
        Tool("slope", "findLine", Centered(200, 40 + 100 * Math.Tan(30 * Math.PI / 180), 120, 40, 30), new { polarity = "any" }),
        Tool("dot", "blob", Rect(0, 100, 50, 50)),
        Tool("broken", "findLine", Centered(200, 150, 60, 20)),
    }.Concat(more).ToArray();

    [Theory]
    [InlineData("floor", "slope", 30)]
    [InlineData("slope", "floor", -30)]
    [InlineData("floor", "floor", 0)]
    public void Two_lines_give_B_minus_A(string a, string b, double want)
    {
        using var img = TwoEdges();
        var r = Run(Program(Lines(Tool("a", "angle", null, new { sources = new[] { a, b } }))), img).Tool("a")!;
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "angle"), want - 0.5, want + 0.5);
        Assert.Equal(2, r.Lines.Count);
    }

    [Theory]
    [InlineData(new[] { "left", "right" }, -20)]     // 170 − (−170) = 340 → −20
    [InlineData(new[] { "right", "left" }, 20)]
    public void The_difference_wraps_into_minus_180_to_180(string[] sources, double want)
    {
        // A V: light below an edge at 10° on the left and at −10° on the right.
        using var img = Canvas(40);
        double t = Math.Tan(10 * Math.PI / 180);
        Poly(img, Scalar.All(200), (0, 150 - 100 * t), (200, 150 + 100 * t), (400, 150 - 100 * t), (400, 300), (0, 300));
        var r = Run(Program(
            Tool("left", "findLine", Centered(100, 150, 120, 60, 190), new { polarity = "any" }),    // points left: −170°
            Tool("right", "findLine", Centered(300, 150, 120, 60, 170), new { polarity = "any" }),   // points left: 170°
            Tool("a", "angle", null, new { sources })), img);
        Assert.InRange(Num(r.Tool("left")!, "angle"), -170.4, -169.6);
        Assert.InRange(Num(r.Tool("right")!, "angle"), 169.6, 170.4);
        Assert.InRange(Num(r.Tool("a")!, "angle"), want - 0.6, want + 0.6);
    }

    [Theory]
    [InlineData(new string[0], "needs a line source")]
    [InlineData(new[] { "dot" }, "source 1 has no line")]
    [InlineData(new[] { "floor", "dot" }, "source 2 has no line")]
    public void A_source_without_a_line_fails(string[] sources, string message)
    {
        using var img = TwoEdges();
        var r = Run(Program(Lines(Tool("a", "angle", null, new { sources }))), img).Tool("a")!;
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Equal(message, r.Message);
    }

    [Fact]
    public void A_failed_source_leaves_it_not_run()
    {
        using var img = TwoEdges();
        var r = Run(Program(Lines(Tool("a", "angle", null, new { sources = new[] { "floor", "broken" } }))), img).Tool("a")!;
        Assert.Equal(ToolStatus.NotRun, r.Status);
        Assert.Contains("broken", r.Message);
        Assert.False(r.Passed);
    }

    [Fact]
    public void Limits_on_angle()
    {
        using var img = TwoEdges();
        double v = AssertLimitBoundaries(img, "a", "angle", (lim, inc) =>
        {
            var tools = Lines(Tool("a", "angle", null, new { sources = new[] { "floor", "slope" } }, limits: lim, includeInPass: inc))
                .Where(t => t.Id != "broken").ToArray();
            return Program(tools);
        });
        Assert.InRange(v, 29.5, 30.5);
    }
}

/// <summary>corners: Shi-Tomasi / Harris feature corners.</summary>
public class CornersToolTests
{
    private static Mat Rectangle()
    {
        var img = Canvas(40);
        Box(img, 100, 80, 200, 140, Scalar.All(200));     // corner pixels (100,80) (299,80) (299,219) (100,219)
        return img;
    }

    private static readonly (double X, double Y)[] RectCorners = { (100, 80), (299, 80), (299, 219), (100, 219) };

    private static void AssertCorners(ToolResult r, IEnumerable<(double X, double Y)> want, double tol)
    {
        foreach (var w in want)
            Assert.True(r.Points.Any(p => Math.Abs(p.X - w.X) <= tol && Math.Abs(p.Y - w.Y) <= tol),
                $"no corner near ({w.X:F1}, {w.Y:F1}); got {string.Join(" ", r.Points.Select(p => $"({p.X},{p.Y})"))}");
    }

    [Theory]
    [InlineData("shiTomasi")]
    [InlineData("harris")]
    public void Finds_the_four_corners_of_a_rectangle(string method)
    {
        using var img = Rectangle();
        var r = RunOne(img, Tool("k", "corners", null, new { method, maxCorners = 10, quality = 0.1, minDistance = 20 }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(4, Num(r, "count"));
        Assert.Equal(4, r.Points.Count);
        AssertCorners(r, RectCorners, 1.5);
    }

    [Theory]
    [InlineData(30)]
    [InlineData(-15)]
    public void Finds_the_corners_of_a_turned_square(double deg)
    {
        using var img = Canvas(40);
        var k = Turned(200, 150, 120, 120, deg);
        Poly(img, Scalar.All(200), k);
        var r = RunOne(img, Tool("k", "corners", null, new { quality = 0.1, minDistance = 20 }));
        Assert.Equal(4, Num(r, "count"));
        AssertCorners(r, k, 2.5);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(2, 2)]
    [InlineData(50, 4)]
    public void MaxCorners_caps_the_count(int maxCorners, int count)
    {
        using var img = Rectangle();
        var r = RunOne(img, Tool("k", "corners", null, new { maxCorners, quality = 0.1, minDistance = 20 }));
        Assert.Equal(count, Num(r, "count"));
        Assert.All(r.Points, p => Assert.Contains(RectCorners, w => Math.Abs(p.X - w.X) <= 1.5 && Math.Abs(p.Y - w.Y) <= 1.5));
    }

    [Theory]
    [InlineData(10, 8)]
    [InlineData(40, 6)]    // the two inner pairs (16 px apart) lose one corner each
    public void MinDistance_thins_close_corners(double minDistance, int count)
    {
        using var img = Canvas(40);
        Box(img, 100, 100, 60, 60, Scalar.All(200));
        Box(img, 176, 100, 60, 60, Scalar.All(200));
        var r = RunOne(img, Tool("k", "corners", null, new { quality = 0.1, minDistance }));
        Assert.Equal(count, Num(r, "count"));
        for (int i = 0; i < r.Points.Count; i++)
            for (int j = i + 1; j < r.Points.Count; j++)
                Assert.True(Math.Sqrt(Math.Pow(r.Points[i].X - r.Points[j].X, 2) + Math.Pow(r.Points[i].Y - r.Points[j].Y, 2)) >= minDistance);
    }

    [Theory]
    [InlineData(0.01, 8)]
    [InlineData(0.2, 4)]   // the faint rectangle's corners are far weaker
    public void Quality_drops_weak_corners(double quality, int count)
    {
        using var img = Canvas(40);
        Box(img, 40, 80, 120, 120, Scalar.All(220));
        Box(img, 240, 80, 120, 120, Scalar.All(75));
        var r = RunOne(img, Tool("k", "corners", null, new { quality, minDistance = 10 }));
        Assert.Equal(count, Num(r, "count"));
        if (count == 4) Assert.All(r.Points, p => Assert.True(p.X < 200));
    }

    [Theory]
    [InlineData(3)]
    [InlineData(7)]
    public void BlockSize_still_finds_the_corners(int blockSize)
    {
        using var img = Rectangle();
        var r = RunOne(img, Tool("k", "corners", null, new { quality = 0.1, minDistance = 20, blockSize }));
        Assert.Equal(4, Num(r, "count"));
        AssertCorners(r, RectCorners, 2.5);
    }

    [Fact]
    public void Only_the_region_is_searched()
    {
        using var img = Rectangle();
        var rect = RunOne(img, Tool("k", "corners", Rect(50, 40, 120, 100), new { quality = 0.1 }));
        Assert.Equal(1, Num(rect, "count"));
        AssertCorners(rect, new[] { RectCorners[0] }, 1.5);
        var circle = RunOne(img, Tool("k", "corners", Circle(299, 219, 30), new { quality = 0.1 }));
        Assert.Equal(1, Num(circle, "count"));
        AssertCorners(circle, new[] { RectCorners[2] }, 1.5);
    }

    [Fact]
    public void None_is_ok_and_off_the_image_fails()
    {
        using var img = Canvas(90);
        var r = RunOne(img, Tool("k", "corners"));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.Equal(0, Num(r, "count"));
        Assert.True(r.Passed);
        var off = RunOne(img, Tool("k", "corners", Rect(450, 350, 40, 40)));
        Assert.Equal(ToolStatus.Fail, off.Status);
        Assert.Contains("outside the image", off.Message);
        Assert.Equal(0, Num(off, "count"));
    }

    [Fact]
    public void Limits_on_count()
    {
        using var img = Rectangle();
        Assert.Equal(4, AssertLimitBoundaries(img, "k", "count", Single(() => Tool("k", "corners", null, new { quality = 0.1, minDistance = 20 }))));
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Canvas(40);
        Box(teach, 160, 120, 80, 80, Scalar.All(200));
        Dots(teach);
        var prog = FixturedProgram(
            Tool("k", "corners", Centered(200, 160, 120, 120), new { quality = 0.1, minDistance = 20 }, fixture: "fx"));
        var (_, r1) = TeachThenMoved(prog, teach, 40);
        var k = r1.Tool("k")!;
        Assert.Equal(4, Num(k, "count"));
        AssertCorners(k, new[] { Moved(160, 120), Moved(239, 120), Moved(239, 199), Moved(160, 199) }, 2.5);
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("k", "corners", Centered(200, 160, 120, 120)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        using var img = Canvas(RedGray);
        Box(img, 100, 80, 200, 140, Red);
        Assert.Equal(0, Num(RunOne(img, Tool("k", "corners", null, new { quality = 0.1, minDistance = 20 })), "count"));
        var prog = Program(Tool("k", "corners", null, new { quality = 0.1, minDistance = 20 }, input: "red"));
        prog.Layers.Add(RedLayer());
        var r = Run(prog, img).Tool("k")!;
        Assert.Equal(4, Num(r, "count"));
        AssertCorners(r, RectCorners, 1.5);
    }
}

/// <summary>lines: Canny + probabilistic Hough segments, filtered by angle.</summary>
public class LinesToolTests
{
    private static double Undirected(double a) => ((a % 180) + 180) % 180;

    /// <summary>A light 200 × 100 rectangle, edges at x ≈ 100 / 299, y ≈ 100 / 199.</summary>
    private static Mat Box200x100()
    {
        var img = Canvas(40);
        Box(img, 100, 100, 200, 100, Scalar.All(200));
        return img;
    }

    private static object P(object? extra = null)
    {
        var d = new Dictionary<string, object> { ["houghThreshold"] = 40, ["minLineLength"] = 60, ["maxLineGap"] = 5 };
        if (extra != null) foreach (var prop in extra.GetType().GetProperties()) d[prop.Name] = prop.GetValue(extra)!;
        return d;
    }

    private static string Side(Dictionary<string, object> it)
    {
        double x1 = D(it["x1"]), y1 = D(it["y1"]), x2 = D(it["x2"]), y2 = D(it["y2"]);
        if (Math.Abs(y1 - 100) <= 1.5 && Math.Abs(y2 - 100) <= 1.5) return "top";
        if (Math.Abs(y1 - 199) <= 1.5 && Math.Abs(y2 - 199) <= 1.5) return "bottom";
        if (Math.Abs(x1 - 100) <= 1.5 && Math.Abs(x2 - 100) <= 1.5) return "left";
        if (Math.Abs(x1 - 299) <= 1.5 && Math.Abs(x2 - 299) <= 1.5) return "right";
        return $"stray ({x1},{y1})-({x2},{y2})";
    }

    [Fact]
    public void Finds_every_side_of_a_rectangle_longest_first()
    {
        using var img = Box200x100();
        var r = RunOne(img, Tool("l", "lines", null, P()));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(new[] { "bottom", "left", "right", "top" }, r.Items.Select(Side).Distinct().OrderBy(s => s));
        Assert.Equal(r.Lines.Count, Num(r, "count"));
        Assert.Equal(r.Items.Count, r.Lines.Count);
        var lengths = r.Items.Select(i => D(i["length"])).ToList();
        Assert.Equal(lengths.OrderByDescending(x => x), lengths);
        Assert.InRange(lengths[0], 190, 201);                  // a long side
        Assert.All(r.Items, i => Assert.True(D(i["length"]) >= 60));
        Assert.All(r.Items, i => Assert.True(Undirected(D(i["angle"])) < 1 || Math.Abs(D(i["angle"]) - 90) < 1));
    }

    [Theory]
    [InlineData(80, 100, new[] { "left", "right" })]
    [InlineData(170, 10, new[] { "bottom", "top" })]    // min > max wraps round 0
    [InlineData(-1, 1, new[] { "bottom", "top" })]
    [InlineData(30, 60, new string[0])]
    public void The_angle_filter_keeps_matching_segments(double minAngle, double maxAngle, string[] sides)
    {
        using var img = Box200x100();
        var r = RunOne(img, Tool("l", "lines", null, P(new { minAngle, maxAngle })));
        Assert.Equal(sides, r.Items.Select(Side).Distinct().OrderBy(s => s));
        Assert.Equal(r.Items.Count, Num(r, "count"));
    }

    [Theory]
    [InlineData(30, 25, 35, 30)]
    [InlineData(-45, 130, 140, 135)]    // −45° is 135° undirected
    [InlineData(45, 40, 50, 45)]
    public void Reports_angles_undirected_0_to_180(double deg, double lo, double hi, double want)
    {
        using var img = Canvas(40);
        Poly(img, Scalar.All(200), Turned(200, 150, 220, 100, deg));
        var r = RunOne(img, Tool("l", "lines", null, P(new { minAngle = lo, maxAngle = hi })));
        Assert.True(Num(r, "count") >= 1, "no segment in the angle window");
        Assert.All(r.Items, i => Assert.InRange(D(i["angle"]), want - 1.5, want + 1.5));
        Assert.All(r.Items, i => Assert.InRange(D(i["angle"]), 0, 180));
    }

    [Fact]
    public void MinLineLength_drops_short_segments()
    {
        using var img = Canvas(40);
        Box(img, 100, 130, 200, 40, Scalar.All(200));         // long sides 200, short sides 40
        var all = RunOne(img, Tool("l", "lines", null, new { houghThreshold = 20, minLineLength = 30, maxLineGap = 5 }));
        Assert.Contains(all.Items, i => Math.Abs(D(i["angle"]) - 90) < 1);
        var longOnly = RunOne(img, Tool("l", "lines", null, new { houghThreshold = 20, minLineLength = 60, maxLineGap = 5 }));
        Assert.True(Num(longOnly, "count") >= 2);
        Assert.All(longOnly.Items, i => Assert.True(Undirected(D(i["angle"])) < 1 || Undirected(D(i["angle"])) > 179));
    }

    [Theory]
    [InlineData(50, 150, 0)]    // a faint 40 → 60 step is below Canny's thresholds
    [InlineData(10, 30, 4)]
    public void Canny_thresholds_decide_which_edges_exist(double canny1, double canny2, int sides)
    {
        using var img = Canvas(40);
        Box(img, 100, 100, 200, 100, Scalar.All(60));
        var r = RunOne(img, Tool("l", "lines", null, P(new { canny1, canny2 })));
        Assert.Equal(sides, r.Items.Select(Side).Distinct().Count());
    }

    [Fact]
    public void Only_the_region_is_searched()
    {
        using var img = Box200x100();
        var r = RunOne(img, Tool("l", "lines", Centered(200, 100, 160, 30), P()));
        Assert.True(Num(r, "count") >= 1);
        Assert.All(r.Items, i => Assert.Equal("top", Side(i)));
        Assert.All(r.Lines, l => { Assert.InRange(l.X1, 119, 281); Assert.InRange(l.X2, 119, 281); });
    }

    [Fact]
    public void None_is_ok_and_off_the_image_fails()
    {
        using var img = Canvas(90);
        var r = RunOne(img, Tool("l", "lines", null, P()));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.Equal(0, Num(r, "count"));
        var off = RunOne(img, Tool("l", "lines", Rect(420, 0, 40, 40), P()));
        Assert.Equal(ToolStatus.Fail, off.Status);
        Assert.Contains("outside the image", off.Message);
    }

    [Fact]
    public void Limits_on_count()
    {
        using var img = Box200x100();
        AssertLimitBoundaries(img, "l", "count", Single(() => Tool("l", "lines", null, P(new { minAngle = 80, maxAngle = 100 }))));
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Box200x100();
        Dots(teach);
        var prog = FixturedProgram(Tool("l", "lines", Centered(200, 100, 160, 30), P(), fixture: "fx"));
        var (_, r1) = TeachThenMoved(prog, teach, 40);
        var l = r1.Tool("l")!;
        Assert.True(Num(l, "count") >= 1);
        Assert.All(l.Items, i => Assert.InRange(D(i["angle"]), MoveDeg - 1.5, MoveDeg + 1.5));
        // On the moved top side: the line through Moved(200, 99.5) at MoveDeg.
        var (mx, my) = Moved(200, 99.5);
        var dir = (Math.Cos(MoveDeg * Math.PI / 180), Math.Sin(MoveDeg * Math.PI / 180));
        var onTop = new PxLine(mx, my, mx + 100 * dir.Item1, my + 100 * dir.Item2);
        Assert.All(l.Lines, s => { Assert.True(DistToLine(onTop, s.X1, s.Y1) < 1.5); Assert.True(DistToLine(onTop, s.X2, s.Y2) < 1.5); });
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("l", "lines", Centered(200, 100, 160, 30), P()));

    [Fact]
    public void It_reads_its_input_layer()
    {
        using var img = Canvas(RedGray);
        Box(img, 100, 100, 200, 100, Red);
        Assert.Equal(0, Num(RunOne(img, Tool("l", "lines", null, P())), "count"));
        var prog = Program(Tool("l", "lines", null, P(), input: "red"));
        prog.Layers.Add(RedLayer());
        var r = Run(prog, img).Tool("l")!;
        Assert.Equal(new[] { "bottom", "left", "right", "top" }, r.Items.Select(Side).Distinct().OrderBy(s => s));
    }
}
