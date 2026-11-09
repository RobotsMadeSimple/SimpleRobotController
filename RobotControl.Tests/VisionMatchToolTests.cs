using Controller.RobotControl.Vision;
using Controller.RobotControl.Vision.Tools;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using static RobotControl.Tests.VisionTestKit;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

// Area-based locating tools (blob, contourMatch, patternMatch, aruco) on synthetic images.

/// <summary>blob: connected dark or light areas.</summary>
public class BlobToolTests
{
    [Fact]
    public void Reports_area_centroid_and_angle_of_each_blob()
    {
        using var img = Canvas(220);
        Box(img, 60, 100, 40, 20, Scalar.All(30));     // pixels x 60..99, y 100..119
        Cv2.Circle(img, new Point(250, 150), 25, Scalar.All(30), -1);
        var r = RunOne(img, Tool("b", "blob", null, new { polarity = "dark" }));
        Assert.True(r.Ok);
        Assert.Equal(2, Num(r, "count"));
        Assert.Equal(1, Num(r, "found"));
        var disc = r.Items[0];                          // biggest first
        Assert.Equal(250, D(disc["x"]), 1);
        Assert.Equal(150, D(disc["y"]), 1);
        Assert.InRange(D(disc["area"]), Math.PI * 24.5 * 24.5 * 0.97, Math.PI * 25 * 25);   // the contour runs through the rim pixels' centres
        Assert.True(D(disc["circularity"]) > 0.85);
        var box = r.Items[1];
        Assert.Equal(79.5, D(box["x"]), 2);
        Assert.Equal(109.5, D(box["y"]), 2);
        Assert.Equal(39 * 19, D(box["area"]), 1);      // contour through the outer pixels' centres
        Assert.InRange(D(box["angle"]), -0.5, 0.5);     // long axis horizontal
        Assert.InRange(D(box["circularity"]), 0.55, 0.7);   // 4π·741 / 116²
        Assert.Equal(250, Num(r, "x"), 1);
        Assert.Equal(150, r.Pose!.Y, 1);
    }

    [Theory]
    [InlineData(30)]
    [InlineData(-45)]
    [InlineData(70)]
    public void The_angle_is_the_long_axis(double deg)
    {
        using var img = Canvas(220);
        Poly(img, Scalar.All(30), Turned(200, 150, 120, 30, deg));
        var r = RunOne(img, Tool("b", "blob", null, new { polarity = "dark" }));
        Assert.Equal(1, Num(r, "count"));
        Assert.InRange(D(r.Items[0]["angle"]), deg - 0.5, deg + 0.5);
        Assert.InRange(r.Pose!.Angle, deg - 0.5, deg + 0.5);
        Near(Num(r, "x"), Num(r, "y"), (200, 150), 0.6);
    }

    /// <summary>A dark (30) disc left, a light (230) disc right, on mid gray 128.</summary>
    private static Mat DarkAndLight()
    {
        var img = Canvas(128);
        Cv2.Circle(img, new Point(100, 150), 30, Scalar.All(30), -1);
        Cv2.Circle(img, new Point(300, 150), 20, Scalar.All(230), -1);
        return img;
    }

    [Theory]
    [InlineData("dark", 80, 100)]
    [InlineData("light", 180, 300)]
    public void Polarity_picks_dark_or_light_objects(string polarity, int level, double x)
    {
        using var img = DarkAndLight();
        var r = RunOne(img, Tool("b", "blob", null, new { polarity, threshold = "fixed", level }));
        Assert.Equal(1, Num(r, "count"));
        Assert.Equal(x, Num(r, "x"), 1);
    }

    [Theory]
    // Discs of gray 30, 100 and 160 on 220: a fixed level only takes those below it.
    [InlineData(50, 1)]
    [InlineData(128, 2)]
    [InlineData(190, 3)]
    [InlineData(20, 0)]
    public void A_fixed_level_takes_what_is_darker(int level, int count)
    {
        using var img = Canvas(220);
        Cv2.Circle(img, new Point(80, 150), 25, Scalar.All(30), -1);
        Cv2.Circle(img, new Point(200, 150), 25, Scalar.All(100), -1);
        Cv2.Circle(img, new Point(320, 150), 25, Scalar.All(160), -1);
        var r = RunOne(img, Tool("b", "blob", null, new { polarity = "dark", threshold = "fixed", level }));
        Assert.True(r.Ok);
        Assert.Equal(count, Num(r, "count"));
        Assert.Equal(count > 0 ? 1 : 0, Num(r, "found"));
    }

    [Fact]
    public void Otsu_adapts_to_the_region_where_a_fixed_level_does_not()
    {
        using var img = Canvas(220);
        Cv2.Circle(img, new Point(200, 150), 30, Scalar.All(180), -1);   // faint: lighter than 128
        Assert.Equal(1, Num(RunOne(img, Tool("b", "blob", null, new { polarity = "dark" })), "count"));
        Assert.Equal(1, Num(RunOne(img, Tool("b", "blob", null, new { polarity = "dark", threshold = "otsu" })), "count"));
        Assert.Equal(0, Num(RunOne(img, Tool("b", "blob", null, new { polarity = "dark", threshold = "fixed", level = 128 })), "count"));
    }

    /// <summary>Dark discs r 10, 20, 30 at x 80, 200, 320 on light.</summary>
    private static Mat Three()
    {
        var img = Canvas(220);
        Cv2.Circle(img, new Point(80, 150), 10, Scalar.All(30), -1);
        Cv2.Circle(img, new Point(200, 150), 20, Scalar.All(30), -1);
        Cv2.Circle(img, new Point(320, 150), 30, Scalar.All(30), -1);
        return img;
    }

    [Theory]
    [InlineData(null, null, new double[] { 320, 200, 80 })]
    [InlineData(500.0, null, new double[] { 320, 200 })]
    [InlineData(null, 2000.0, new double[] { 200, 80 })]
    [InlineData(500.0, 2000.0, new double[] { 200 })]
    [InlineData(5000.0, null, new double[0])]
    public void Area_limits_filter_blobs(double? minArea, double? maxArea, double[] xs)
    {
        using var img = Three();
        var p = new Dictionary<string, object> { ["polarity"] = "dark" };
        if (minArea != null) p["minArea"] = minArea;
        if (maxArea != null) p["maxArea"] = maxArea;
        var r = RunOne(img, Tool("b", "blob", null, p));
        Assert.True(r.Ok);
        Assert.Equal(xs, r.Items.Select(i => Math.Round(D(i["x"]))));
        Assert.Equal(xs.Length, Num(r, "count"));
        if (xs.Length == 0) { Assert.Null(r.Pose); Assert.False(r.Values.ContainsKey("x")); }
    }

    [Fact]
    public void The_default_minimum_area_drops_specks()
    {
        using var img = Canvas(220);
        Box(img, 100, 100, 3, 3, Scalar.All(30));      // area 4 < 20
        Box(img, 200, 100, 8, 8, Scalar.All(30));      // area 49
        Assert.Equal(1, Num(RunOne(img, Tool("b", "blob", null, new { polarity = "dark" })), "count"));
        Assert.Equal(2, Num(RunOne(img, Tool("b", "blob", null, new { polarity = "dark", minArea = 1 })), "count"));
    }

    [Theory]
    [InlineData(0.0, 3)]
    [InlineData(0.5, 2)]     // the bar is out
    [InlineData(0.85, 1)]    // only the disc
    [InlineData(0.99, 0)]
    public void Circularity_filters_elongated_and_angular_blobs(double minCircularity, int count)
    {
        using var img = Canvas(220);
        Cv2.Circle(img, new Point(80, 150), 30, Scalar.All(30), -1);
        Box(img, 160, 120, 60, 60, Scalar.All(30));
        Box(img, 260, 140, 120, 12, Scalar.All(30));
        var r = RunOne(img, Tool("b", "blob", null, new { polarity = "dark", minCircularity }));
        Assert.Equal(count, Num(r, "count"));
        Assert.All(r.Items, i => Assert.True(D(i["circularity"]) >= minCircularity));
    }

    [Theory]
    [InlineData("area", new double[] { 320, 80, 200 })]
    [InlineData("x", new double[] { 80, 200, 320 })]
    [InlineData("y", new double[] { 200, 320, 80 })]
    public void Sort_orders_the_items_and_the_pose_is_the_first(string sort, double[] xs)
    {
        using var img = Canvas(220);
        Cv2.Circle(img, new Point(80, 250), 20, Scalar.All(30), -1);
        Cv2.Circle(img, new Point(200, 60), 10, Scalar.All(30), -1);
        Cv2.Circle(img, new Point(320, 150), 30, Scalar.All(30), -1);
        var r = RunOne(img, Tool("b", "blob", null, new { polarity = "dark", sort }));
        Assert.Equal(xs, r.Items.Select(i => Math.Round(D(i["x"]))));
        Assert.Equal(xs[0], r.Pose!.X, 0);
        Assert.Equal(xs[0], Num(r, "x"), 0);
    }

    [Theory]
    [InlineData(2, 2)]
    [InlineData(1, 1)]
    [InlineData(0, 5)]       // 0 = no cap
    [InlineData(100, 5)]
    public void MaxCount_keeps_the_first_blobs(int maxCount, int count)
    {
        using var img = Canvas(220);
        for (int i = 0; i < 5; i++) Cv2.Circle(img, new Point(50 + i * 75, 150), 8 + i * 4, Scalar.All(30), -1);
        var r = RunOne(img, Tool("b", "blob", null, new { polarity = "dark", maxCount }));
        Assert.Equal(count, Num(r, "count"));
        Assert.Equal(count, r.Items.Count);
        Assert.Equal(350, D(r.Items[0]["x"]), 1);       // the biggest is kept first
    }

    [Fact]
    public void Only_the_region_is_searched()
    {
        using var img = Three();
        var rect = RunOne(img, Tool("b", "blob", Rect(150, 100, 100, 100), new { polarity = "dark" }));
        Assert.Equal(1, Num(rect, "count"));
        Assert.Equal(200, Num(rect, "x"), 1);
        var circle = RunOne(img, Tool("b", "blob", Circle(320, 150, 45), new { polarity = "dark" }));
        Assert.Equal(1, Num(circle, "count"));
        Assert.Equal(320, Num(circle, "x"), 1);
        // A blob cut by the region edge counts only its part inside.
        var half = RunOne(img, Tool("b", "blob", Rect(320, 100, 80, 100), new { polarity = "dark" }));
        Assert.Equal(1, Num(half, "count"));
        Assert.InRange(Num(half, "x"), 330, 335);
        Assert.InRange(D(half.Items[0]["area"]), 1300, 1450);
    }

    [Fact]
    public void Nothing_found_is_ok_with_a_zero_count()
    {
        using var img = Canvas(220);
        var r = RunOne(img, Tool("b", "blob", null, new { polarity = "dark" }));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.True(r.Passed);
        Assert.Equal(0, Num(r, "count"));
        Assert.Equal(0, Num(r, "found"));
        Assert.Null(r.Pose);
        Assert.Empty(r.Items);
        // A region off the image is the same.
        var off = RunOne(img, Tool("b", "blob", Rect(500, 400, 50, 50), new { polarity = "dark" }));
        Assert.Equal(ToolStatus.Ok, off.Status);
        Assert.Equal(0, Num(off, "count"));
        // A limit on the count turns it into a failed check.
        var judged = RunOne(img, Tool("b", "blob", null, new { polarity = "dark" }, limits: new() { ["count"] = new() { Min = 1 } }));
        Assert.Equal(ToolStatus.Ok, judged.Status);
        Assert.False(judged.Passed);
    }

    [Fact]
    public void Limits_on_count()
    {
        using var img = Three();
        double v = AssertLimitBoundaries(img, "b", "count", Single(() => Tool("b", "blob", null, new { polarity = "dark" })));
        Assert.Equal(3, v);
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Canvas(220);
        Cv2.Circle(teach, new Point(200, 170), 18, Scalar.All(30), -1);
        Dots(teach);
        var prog = FixturedProgram(
            Tool("b", "blob", Centered(200, 170, 44, 44), new { polarity = "dark", threshold = "fixed", level = 128 }, fixture: "fx"),
            Tool("still", "blob", Centered(200, 170, 44, 44), new { polarity = "dark", threshold = "fixed", level = 128 }));
        var (r0, r1) = TeachThenMoved(prog, teach, 220);
        double area0 = D(r0.Tool("b")!.Items[0]["area"]);
        var b = r1.Tool("b")!;
        Assert.Equal(1, Num(b, "count"));
        Near(Num(b, "x"), Num(b, "y"), Moved(200, 170), 0.6);
        Assert.InRange(D(b.Items[0]["area"]), area0 * 0.97, area0 * 1.03);
        // The same region left where it was drawn cuts the moved blob.
        var still = r1.Tool("still")!;
        Assert.True(Num(still, "count") == 0 || D(still.Items[0]["area"]) < area0 * 0.9, "the unfixtured region still holds the whole blob");
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("b", "blob", Centered(200, 170, 60, 60)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        using var img = Canvas(RedGray);
        Cv2.Circle(img, new Point(150, 150), 20, Red, -1);
        Cv2.Circle(img, new Point(260, 150), 30, Red, -1);
        Assert.Equal(0, Num(RunOne(img, Tool("b", "blob", null, new { polarity = "light", threshold = "fixed", level = 128 })), "count"));
        var prog = Program(Tool("b", "blob", null, new { polarity = "light" }, input: "red"));
        prog.Layers.Add(RedLayer());
        var r = Run(prog, img).Tool("b")!;
        Assert.Equal(2, Num(r, "count"));
        Assert.Equal(260, Num(r, "x"), 1);
    }
}

/// <summary>contourMatch: shapes compared with a taught contour (Hu moments).</summary>
public class ContourMatchToolTests
{
    // An asymmetric triangle; its vertex mean (= its centroid) is (86.7, 133.3).
    private static readonly (double X, double Y)[] Taught = { (80, 60), (140, 180), (40, 160) };
    private static readonly double Mx = Taught.Average(p => p.X), My = Taught.Average(p => p.Y);

    private static (double X, double Y)[] Placed(double cx, double cy, double scale, double deg) =>
        Taught.Select(p => TurnPoint(cx + (p.X - Mx) * scale, cy + (p.Y - My) * scale, cx, cy, deg)).ToArray();

    private static VisionProgram Taught_(object? extra = null, int fg = 220, int bg = 30, string input = "camera", ImageLayer? layer = null)
    {
        using var teach = Canvas(bg);
        Poly(teach, fg == -1 ? Red : Scalar.All(fg), Taught);
        var p = new Dictionary<string, object> { ["polarity"] = fg > bg || fg == -1 ? "light" : "dark", ["minScore"] = 0.8 };
        if (extra != null) foreach (var prop in extra.GetType().GetProperties()) p[prop.Name] = prop.GetValue(extra)!;
        var prog = Program(Tool("m", "contourMatch", Rect(20, 40, 140, 160), p, input: input));
        if (layer != null) prog.Layers.Add(layer);
        using (var rt = new VisionRuntime()) Assert.Contains("contour of", VisionEngine.Teach(prog, "m", teach, rt));
        prog.Tools[0].Region = null;
        return prog;
    }

    private static double Wrap90(double a) { a %= 180; if (a > 90) a -= 180; if (a <= -90) a += 180; return a; }

    [Fact]
    public void Teaching_stores_the_largest_contour_of_the_region()
    {
        var prog = Taught_();
        var t = prog.Tools[0].P.Obj<ContourMatchTool.Template>("template")!;
        Assert.True(t.Points.Count >= 3);
        Assert.InRange(t.Width, 99, 102);              // x 40..140
        Assert.InRange(t.Height, 119, 122);            // y 60..180
        Assert.All(t.Points, q => { Assert.InRange(q.X, 0, 1); Assert.InRange(q.Y, 0, 1); });
    }

    [Theory]
    [InlineData(0, 1.0)]
    [InlineData(30, 1.3)]
    [InlineData(75, 0.8)]
    [InlineData(-45, 1.0)]
    [InlineData(120, 1.5)]
    public void Finds_the_shape_turned_and_scaled(double deg, double scale)
    {
        var prog = Taught_();
        using var img = Canvas(30);
        Poly(img, Scalar.All(220), Placed(250, 150, scale, deg));
        Cv2.Rectangle(img, new Rect(30, 40, 60, 60), Scalar.All(220), -1);      // distractors
        Cv2.Circle(img, new Point(80, 230), 35, Scalar.All(220), -1);
        var r = Run(prog, img).Tool("m")!;
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "count"));
        Near(r.Pose!.X, r.Pose.Y, (250, 150), 1.5);
        Assert.True(Num(r, "score") > 0.85, $"score {Num(r, "score")}");   // pixelisation moves the Hu moments a little
        // Orientation from the principal axis: the turn, modulo 180°.
        Assert.True(Math.Abs(Wrap90(Num(r, "angle") - deg)) < 3, $"angle {Num(r, "angle")} for {deg}");
        Assert.Equal(Num(r, "angle"), r.Pose.Angle, 1);
        Assert.Equal(Num(r, "score"), D(r.Items[0]["score"]), 2);
    }

    [Fact]
    public void Other_shapes_are_not_found()
    {
        var prog = Taught_();
        using var img = Canvas(30);
        Cv2.Rectangle(img, new Rect(30, 40, 60, 60), Scalar.All(220), -1);
        Cv2.Circle(img, new Point(250, 150), 50, Scalar.All(220), -1);
        var r = Run(prog, img).Tool("m")!;
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Equal(0, Num(r, "count"));
        Assert.Equal(0, Num(r, "score"));
        Assert.Null(r.Pose);
        Assert.Empty(r.Items);
        Assert.False(r.Passed);
    }

    [Fact]
    public void MinScore_is_the_acceptance_threshold()
    {
        // An equilateral-ish triangle: like the taught one, but not the same.
        using var img = Canvas(30);
        Poly(img, Scalar.All(220), (200, 80), (270, 200), (130, 200));
        var loose = Run(Taught_(new { minScore = 0.0 }), img).Tool("m")!;
        Assert.True(loose.Ok);
        double s = Num(loose, "score");
        Assert.InRange(s, 0.05, 0.97);
        Assert.Equal(ToolStatus.Fail, Run(Taught_(new { minScore = s + 0.02 }), img).Tool("m")!.Status);
        Assert.Equal(ToolStatus.Ok, Run(Taught_(new { minScore = s - 0.02 }), img).Tool("m")!.Status);
    }

    [Theory]
    [InlineData(10, 3)]
    [InlineData(2, 2)]
    [InlineData(1, 1)]
    public void MaxCount_keeps_the_best_matches(int maxCount, int count)
    {
        using var img = Canvas(30);
        Poly(img, Scalar.All(220), Placed(80, 90, 0.6, 0));
        Poly(img, Scalar.All(220), Placed(200, 200, 0.7, 40));
        Poly(img, Scalar.All(220), Placed(320, 100, 0.8, -70));
        var r = Run(Taught_(new { maxCount }), img).Tool("m")!;
        Assert.Equal(count, Num(r, "count"));
        Assert.Equal(count, r.Items.Count);
        var scores = r.Items.Select(i => D(i["score"])).ToList();
        Assert.Equal(scores.OrderByDescending(x => x), scores);
        Assert.Equal(scores[0], Num(r, "score"), 3);
    }

    [Fact]
    public void MinArea_drops_small_copies()
    {
        using var img = Canvas(30);
        Poly(img, Scalar.All(220), Placed(100, 150, 0.35, 0));   // about 660 px²
        Poly(img, Scalar.All(220), Placed(280, 150, 1.0, 0));    // about 6000 px²
        Assert.Equal(2, Num(Run(Taught_(new { minArea = 50 }), img).Tool("m")!, "count"));
        var big = Run(Taught_(new { minArea = 1000 }), img).Tool("m")!;
        Assert.Equal(1, Num(big, "count"));
        Assert.Equal(280, big.Pose!.X, 0);
    }

    [Fact]
    public void Dark_shapes_on_light_with_dark_polarity()
    {
        var prog = Taught_(fg: 30, bg: 220);
        Assert.Equal("dark", prog.Tools[0].P.Str("polarity", ""));
        using var img = Canvas(220);
        Poly(img, Scalar.All(30), Placed(260, 160, 1.2, 50));
        var r = Run(prog, img).Tool("m")!;
        Assert.True(r.Ok, r.Message);
        Near(r.Pose!.X, r.Pose.Y, (260, 160), 1.5);
    }

    [Fact]
    public void Without_a_usable_template_it_fails()
    {
        using var img = Canvas();
        var none = RunOne(img, Tool("m", "contourMatch"));
        Assert.Equal(ToolStatus.Fail, none.Status);
        Assert.Contains("taught", none.Message);
        var two = RunOne(img, Tool("m", "contourMatch", null, new { template = new { points = new[] { new { x = 0, y = 0 }, new { x = 1, y = 1 } }, width = 10, height = 10 } }));
        Assert.Equal(ToolStatus.Fail, two.Status);
        Assert.Equal(0, Num(two, "count"));
        // Teaching on an empty region is refused.
        var prog = Program(Tool("m", "contourMatch", Rect(20, 40, 140, 160), new { polarity = "light", threshold = "fixed", level = 128 }));
        using var rt = new VisionRuntime();
        using var blank = Canvas(30);
        Assert.Throws<InvalidOperationException>(() => VisionEngine.Teach(prog, "m", blank, rt));
    }

    [Fact]
    public void Limits_on_score()
    {
        using var img = Canvas(30);
        Poly(img, Scalar.All(220), Placed(250, 150, 1.1, 20));
        double v = AssertLimitBoundaries(img, "m", "score", (lim, inc) =>
        {
            var prog = Taught_();
            prog.Tools[0].Limits = lim;
            prog.Tools[0].IncludeInPass = inc;
            return prog;
        });
        Assert.True(v > 0.9);
    }

    [Fact]
    public void The_search_region_follows_the_fixture()
    {
        using var teach = Canvas(30);
        Poly(teach, Scalar.All(220), Placed(200, 170, 0.8, 0));
        Dots(teach);
        var tpl = Taught_().Tools[0].Params!["template"];
        var tool = Tool("m", "contourMatch", Centered(200, 170, 120, 120), new { polarity = "light", minScore = 0.8 }, fixture: "fx");
        tool.Params!["template"] = tpl;
        var prog = FixturedProgram(tool);
        var (r0, r1) = TeachThenMoved(prog, teach, 30);
        Assert.True(r0.Tool("m")!.Ok);
        var m = r1.Tool("m")!;
        Assert.True(m.Ok, m.Message);
        Near(m.Pose!.X, m.Pose.Y, Moved(200, 170), 1.2);
        Assert.True(Math.Abs(Wrap90(m.Pose.Angle - r0.Tool("m")!.Pose!.Angle - MoveDeg)) < 2);
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("m", "contourMatch", Centered(200, 170, 120, 120)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        var prog = Taught_(fg: -1, bg: RedGray, input: "red", layer: RedLayer());
        using var img = Canvas(RedGray);
        Poly(img, Red, Placed(250, 150, 1.0, 25));
        var r = Run(prog, img).Tool("m")!;
        Assert.True(r.Ok, r.Message);
        Near(r.Pose!.X, r.Pose.Y, (250, 150), 1.5);
        prog.Tools[0].Input = "camera";
        Assert.Equal(ToolStatus.Fail, Run(prog, img).Tool("m")!.Status);
    }
}

/// <summary>patternMatch: a taught gray patch found by normalized cross-correlation.</summary>
public class PatternMatchToolTests
{
    /// <summary>An asymmetric "F" with a little texture, on a 60 × 60 patch of <paramref name="bg"/>.</summary>
    private static Mat Pattern(Scalar? ink = null, int bg = 60)
    {
        var p = new Mat(60, 60, MatType.CV_8UC3, Scalar.All(bg));
        var c = ink ?? Scalar.All(210);
        Cv2.Rectangle(p, new Rect(15, 8, 10, 44), c, -1);
        Cv2.Rectangle(p, new Rect(15, 8, 32, 9), c, -1);
        Cv2.Rectangle(p, new Rect(15, 26, 22, 8), ink ?? Scalar.All(170), -1);
        Cv2.Circle(p, new Point(42, 42), 5, ink ?? Scalar.All(130), -1);
        return p;
    }

    /// <summary>Draws the pattern centred on (cx, cy) turned by deg (clockwise on screen); its centre pixel (30, 30) lands on (cx, cy).</summary>
    private static void Place(Mat img, double cx, double cy, double deg, Scalar? ink = null, int bg = 60)
    {
        using var pat = Pattern(ink, bg);
        using var m = Cv2.GetRotationMatrix2D(new Point2f(30, 30), -deg, 1);
        m.Set(0, 2, m.At<double>(0, 2) + cx - 30);
        m.Set(1, 2, m.At<double>(1, 2) + cy - 30);
        Cv2.WarpAffine(pat, img, m, img.Size(), InterpolationFlags.Linear, BorderTypes.Transparent);
    }

    private static Mat Scene(double cx, double cy, double deg)
    {
        var img = Canvas(60);
        Place(img, cx, cy, deg);
        return img;
    }

    /// <summary>A program taught on the pattern at (120, 100); the search region is then the whole image.</summary>
    private static VisionProgram Taught(object? p = null, string input = "camera", ImageLayer? layer = null, Mat? teachImg = null)
    {
        using var teach = Scene(120, 100, 0);
        var prog = Program(Tool("p", "patternMatch", Centered(120, 100, 60, 60), p ?? new { minScore = 0.6 }, input: input));
        if (layer != null) prog.Layers.Add(layer);
        using (var rt = new VisionRuntime()) Assert.Equal("pattern 60×60 px", VisionEngine.Teach(prog, "p", teachImg ?? teach, rt));
        prog.Tools[0].Region = null;
        return prog;
    }

    [Fact]
    public void Teaching_cuts_the_region_upright_with_the_origin_at_its_centre()
    {
        var t = Taught().Tools[0].P.Obj<PatternMatchTool.Template>("template")!;
        Assert.Equal(60, t.Width);
        Assert.Equal(60, t.Height);
        Assert.Equal(30, t.OriginX);
        Assert.Equal(30, t.OriginY);
        using var png = Cv2.ImDecode(Convert.FromBase64String(t.Png), ImreadModes.Grayscale);
        Assert.Equal(new Size(60, 60), png.Size());
    }

    [Theory]
    [InlineData(250, 170)]
    [InlineData(80, 230)]
    [InlineData(330.5, 60.5)]   // half-pixel placement: sub-pixel peak
    public void Finds_the_patch_where_it_moved(double cx, double cy)
    {
        var prog = Taught();
        using var img = Scene(cx, cy, 0);
        var r = Run(prog, img).Tool("p")!;
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "count"));
        Near(r.Pose!.X, r.Pose.Y, (cx, cy), 0.6);
        Assert.Equal(0, r.Pose.Angle);
        Assert.True(Num(r, "score") > 0.95, $"score {Num(r, "score")}");
        Near(Num(r, "x"), Num(r, "y"), (cx, cy), 0.6);
    }

    [Theory]
    [InlineData(20)]
    [InlineData(-15)]
    [InlineData(25)]
    [InlineData(12)]     // between search angles: refined past the step
    [InlineData(-7.5)]
    public void Finds_the_patch_turned_within_the_angle_range(double deg)
    {
        var prog = Taught(new { minScore = 0.5, angleRange = 30, angleStep = 5 });
        using var img = Scene(230, 160, deg);
        var r = Run(prog, img).Tool("p")!;
        Assert.True(r.Ok, r.Message);
        Assert.InRange(r.Pose!.Angle, deg - 0.3, deg + 0.3);
        Near(r.Pose.X, r.Pose.Y, (230, 160), 1.0);
        Assert.Equal(r.Pose.Angle, D(r.Items[0]["angle"]), 2);
        Assert.True(Num(r, "score") > 0.9, $"score {Num(r, "score")}");
    }

    [Fact]
    public void Without_an_angle_range_a_turned_patch_scores_lower()
    {
        using var img = Scene(230, 160, 25);
        var fixedAngle = Run(Taught(new { minScore = 0.1 }), img).Tool("p")!;
        var searched = Run(Taught(new { minScore = 0.1, angleRange = 30, angleStep = 5 }), img).Tool("p")!;
        Assert.True(Num(searched, "score") > 0.9);
        Assert.True(Num(fixedAngle, "score") < Num(searched, "score") - 0.2, $"{Num(fixedAngle, "score")} vs {Num(searched, "score")}");
        Assert.Equal(ToolStatus.Fail, Run(Taught(new { minScore = 0.8 }), img).Tool("p")!.Status);
    }

    [Fact]
    public void MinScore_rejects_a_damaged_copy()
    {
        using var img = Scene(230, 160, 0);
        Box(img, 200, 130, 60, 28, Scalar.All(60));      // the top half rubbed out
        var loose = Run(Taught(new { minScore = 0.05 }), img).Tool("p")!;
        Assert.True(loose.Ok);
        double s = Num(loose, "score");
        Assert.InRange(s, 0.1, 0.9);
        Assert.Equal(ToolStatus.Fail, Run(Taught(new { minScore = s + 0.02 }), img).Tool("p")!.Status);
        Assert.Equal(ToolStatus.Ok, Run(Taught(new { minScore = s - 0.02 }), img).Tool("p")!.Status);
    }

    [Theory]
    [InlineData(1, 1)]
    [InlineData(2, 2)]
    [InlineData(5, 2)]       // only two copies exist
    public void MaxCount_reports_several_copies_best_first(int maxCount, int count)
    {
        using var img = Canvas(60);
        Place(img, 100, 100, 0);
        Place(img, 290, 200, 0);
        Box(img, 270, 175, 12, 6, Scalar.All(60));      // a nick in the second copy: it scores lower
        var r = Run(Taught(new { minScore = 0.6, maxCount }), img).Tool("p")!;
        Assert.Equal(count, Num(r, "count"));
        Assert.Equal(count, r.Items.Count);
        Near(D(r.Items[0]["x"]), D(r.Items[0]["y"]), (100, 100), 0.6);
        if (count == 2)
        {
            Near(D(r.Items[1]["x"]), D(r.Items[1]["y"]), (290, 200), 0.6);
            Assert.True(D(r.Items[0]["score"]) > D(r.Items[1]["score"]));
        }
    }

    [Fact]
    public void Only_the_region_is_searched()
    {
        var prog = Taught();
        using var img = Scene(300, 200, 0);
        prog.Tools[0].Region = Rect(0, 0, 200, 300);
        var r = Run(prog, img).Tool("p")!;
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Equal("pattern not found", r.Message);
        prog.Tools[0].Region = Rect(220, 120, 160, 160);
        Assert.True(Run(prog, img).Tool("p")!.Ok);
        prog.Tools[0].Region = Rect(250, 150, 40, 40);
        var small = Run(prog, img).Tool("p")!;
        Assert.Equal(ToolStatus.Fail, small.Status);
        Assert.Contains("smaller than the pattern", small.Message);
    }

    [Fact]
    public void Without_a_usable_template_it_fails()
    {
        using var img = Scene(200, 150, 0);
        var none = RunOne(img, Tool("p", "patternMatch"));
        Assert.Equal(ToolStatus.Fail, none.Status);
        Assert.Contains("taught", none.Message);
        var junk = RunOne(img, Tool("p", "patternMatch", null, new { template = new { png = "not base64!", width = 10, height = 10 } }));
        Assert.Equal(ToolStatus.Fail, junk.Status);
        Assert.Contains("taught", junk.Message);
        Assert.Equal(0, Num(junk, "count"));
    }

    [Fact]
    public void A_blank_image_finds_nothing()
    {
        using var blank = Canvas(60);
        var r = Run(Taught(), blank).Tool("p")!;
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Equal(0, Num(r, "count"));
        Assert.Equal(0, Num(r, "score"));
        Assert.Null(r.Pose);
    }

    [Fact]
    public void Limits_on_score()
    {
        using var img = Scene(250, 170, 0);
        double v = AssertLimitBoundaries(img, "p", "score", (lim, inc) =>
        {
            var prog = Taught();
            prog.Tools[0].Limits = lim;
            prog.Tools[0].IncludeInPass = inc;
            return prog;
        });
        Assert.True(v > 0.95);
    }

    [Fact]
    public void The_search_region_follows_the_fixture()
    {
        using var teach = Scene(200, 170, 0);
        Dots(teach);
        var tpl = Taught().Tools[0].Params!["template"];
        var tool = Tool("p", "patternMatch", Centered(200, 170, 110, 110), new { minScore = 0.6, angleRange = 15, angleStep = 5 }, fixture: "fx");
        tool.Params!["template"] = tpl;
        var (_, r1) = TeachThenMoved(FixturedProgram(tool), teach, 60);
        var p = r1.Tool("p")!;
        Assert.True(p.Ok, p.Message);
        Near(p.Pose!.X, p.Pose.Y, Moved(200, 170), 1.5);
        Assert.InRange(p.Pose.Angle, MoveDeg - 0.3, MoveDeg + 0.3);
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("p", "patternMatch", Centered(200, 170, 110, 110)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        // The pattern in red on gray 76: flat in camera gray, a white "F" on the red layer.
        using var teach = Canvas(RedGray);
        Place(teach, 120, 100, 0, Red, RedGray);
        var prog = Taught(input: "red", layer: RedLayer(), teachImg: teach);
        using var img = Canvas(RedGray);
        Place(img, 260, 190, 0, Red, RedGray);
        var r = Run(prog, img).Tool("p")!;
        Assert.True(r.Ok, r.Message);
        Near(r.Pose!.X, r.Pose.Y, (260, 190), 0.8);
        prog.Tools[0].Input = "camera";
        Assert.Equal(ToolStatus.Fail, Run(prog, img).Tool("p")!.Status);
    }

    // ── Teaching from a rotated area: the pattern's own rotation ─────────────────

    private const double TaughtDeg = 30;

    /// <summary>
    /// Taught with a rotated rectangle drawn round the pattern lying at TaughtDeg — the tool's own
    /// region is the whole image (null), as in the app's teach flow.
    /// </summary>
    private static VisionProgram TaughtTurned(object? p = null)
    {
        using var teach = Scene(150, 140, TaughtDeg);
        var prog = Program(Tool("p", "patternMatch", null, p ?? new { minScore = 0.6, angleRange = 180, angleStep = 5 }));
        using (var rt = new VisionRuntime())
            Assert.Equal("pattern 60×60 px", VisionEngine.Teach(prog, "p", teach, rt, Centered(150, 140, 60, 60, TaughtDeg)));
        return prog;
    }

    [Fact]
    public void A_rotated_teach_area_is_cut_deskewed_and_its_angle_is_kept()
    {
        var t = TaughtTurned().Tools[0].P.Obj<PatternMatchTool.Template>("template")!;
        Assert.Equal(TaughtDeg, t.Angle, 3);
        Assert.Equal(60, t.Width);
        Assert.Equal(60, t.Height);
        Assert.NotNull(t.Region);
        Assert.Equal(TaughtDeg, t.Region!.Rotation);
        // The cut patch is the pattern upright: it correlates with the unturned original.
        using var patch = Cv2.ImDecode(Convert.FromBase64String(t.Png), ImreadModes.Grayscale);
        using var upright = Pattern();
        using var gray = new Mat();
        Cv2.CvtColor(upright, gray, ColorConversionCodes.BGR2GRAY);
        using var inner = new Mat(gray, new Rect(6, 6, 48, 48));
        using var res = new Mat();
        Cv2.MatchTemplate(patch, inner, res, TemplateMatchModes.CCoeffNormed);
        Cv2.MinMaxLoc(res, out _, out double best, out _, out Point at);
        Assert.True(best > 0.9, $"deskewed patch vs upright pattern {best}");
        Assert.InRange(at.X, 5, 7);
        Assert.InRange(at.Y, 5, 7);
        // Outlines of the "F" and its dot, in patch pixels.
        Assert.NotNull(t.Contours);
        Assert.InRange(t.Contours!.Count, 1, 24);
        Assert.All(t.Contours.SelectMany(c => c), q => { Assert.InRange(q.X, 0, 60); Assert.InRange(q.Y, 0, 60); });
    }

    [Theory]
    [InlineData(30, 0)]       // as taught
    [InlineData(75, 45)]
    [InlineData(-60, -90)]
    [InlineData(120, 90)]
    [InlineData(200, 170)]
    [InlineData(-150, 180)]   // upside down
    [InlineData(47.5, 17.5)]  // between search steps
    public void Angles_are_relative_to_the_taught_orientation(double sceneDeg, double expected)
    {
        var prog = TaughtTurned();
        using var img = Scene(260, 170, sceneDeg);
        var r = Run(prog, img).Tool("p")!;
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "count"));
        double err = Math.Abs(Geo.Wrap180(r.Pose!.Angle - expected));
        Assert.True(err < 0.4, $"angle {r.Pose.Angle}, expected {expected}");
        Assert.Equal(r.Pose.Angle, Num(r, "angle"), 2);
        Near(r.Pose.X, r.Pose.Y, (260, 170), 1.0);
        Assert.True(Num(r, "score") > 0.9, $"score {Num(r, "score")}");
    }

    [Fact]
    public void Teaching_from_a_rotated_tool_region_also_takes_its_angle()
    {
        using var teach = Scene(150, 140, -40);
        var prog = Program(Tool("p", "patternMatch", Centered(150, 140, 60, 60, -40), new { minScore = 0.6, angleRange = 60, angleStep = 5 }));
        using (var rt = new VisionRuntime()) VisionEngine.Teach(prog, "p", teach, rt);
        var t = prog.Tools[0].P.Obj<PatternMatchTool.Template>("template")!;
        Assert.Equal(-40, t.Angle, 3);
        Assert.Null(t.Region);
        prog.Tools[0].Region = null;
        using var img = Scene(250, 160, -40);
        var r = Run(prog, img).Tool("p")!;
        Assert.True(r.Ok, r.Message);
        Assert.InRange(r.Pose!.Angle, -0.4, 0.4);
    }

    [Theory]
    [InlineData(30)]
    [InlineData(115)]
    [InlineData(-80)]
    public void The_pattern_outlines_are_drawn_where_each_match_was_found(double sceneDeg)
    {
        var prog = TaughtTurned(new { minScore = 0.6, angleRange = 180, angleStep = 5, maxCount = 2 });
        using var img = Canvas(60);
        Place(img, 110, 100, sceneDeg);
        Place(img, 290, 200, sceneDeg + 50);
        var r = Run(prog, img).Tool("p")!;
        Assert.Equal(2, Num(r, "count"));
        var t = prog.Tools[0].P.Obj<PatternMatchTool.Template>("template")!;
        Assert.Equal(2 * t.Contours!.Count, r.Contours.Count);
        // Every outline point lies on an edge of the scene (within ~2 px).
        using var gray = new Mat();
        Cv2.CvtColor(img, gray, ColorConversionCodes.BGR2GRAY);
        using var edges = new Mat();
        Cv2.Canny(gray, edges, 40, 100);
        using var inv = new Mat();
        Cv2.BitwiseNot(edges, inv);
        using var dist = new Mat();
        Cv2.DistanceTransform(inv, dist, DistanceTypes.L2, DistanceTransformMasks.Mask3);
        var pts = r.Contours.SelectMany(c => c).ToList();
        int near = pts.Count(q => dist.At<float>((int)Math.Round(q.Y), (int)Math.Round(q.X)) <= 2.5f);
        Assert.True(near >= pts.Count * 0.9, $"{near}/{pts.Count} outline points on an edge");
        // One set of outlines per match, round its position.
        int per = t.Contours.Count;
        for (int m = 0; m < 2; m++)
        {
            var mine = r.Contours.Skip(m * per).Take(per).SelectMany(c => c).ToList();
            double mx = mine.Average(q => q.X), my = mine.Average(q => q.Y);
            Assert.True(Math.Abs(mx - D(r.Items[m]["x"])) < 20 && Math.Abs(my - D(r.Items[m]["y"])) < 20);
        }
    }

    [Fact]
    public void Placed_outlines_follow_the_pose()
    {
        var t = new PatternMatchTool.Template
        {
            OriginX = 10, OriginY = 5,
            Contours = new() { new() { new VisionPoint(20, 5), new VisionPoint(10, 15) } },
        };
        var c = PatternMatchTool.Placed(t, 100, 50, 90).Single();
        Assert.Equal(100, c[0].X, 6); Assert.Equal(60, c[0].Y, 6);     // +10 along x turns to +10 along y (clockwise)
        Assert.Equal(90, c[1].X, 6);  Assert.Equal(50, c[1].Y, 6);
    }

    [Fact]
    public void A_flat_patch_has_no_outlines()
    {
        using var flat = new Mat(40, 40, MatType.CV_8UC1, Scalar.All(90));
        Assert.Empty(PatternMatchTool.Outlines(flat));
    }

    [Fact]
    public void The_teach_preview_is_an_enlarged_png()
    {
        var t = TaughtTurned().Tools[0].P.Obj<PatternMatchTool.Template>("template")!;
        using var patch = Cv2.ImDecode(Convert.FromBase64String(t.Png), ImreadModes.Grayscale);
        using var png = Cv2.ImDecode(PatternMatchTool.PreviewPng(t, patch), ImreadModes.Color);
        Assert.Equal(360, png.Width);     // 60 px × 6
        Assert.Equal(3, png.Channels());
    }

    [Fact]
    public void A_full_turn_search_stays_quick()
    {
        var prog = TaughtTurned();
        using var img = Scene(260, 170, 133);
        Run(prog, img);   // warm up
        var sw = System.Diagnostics.Stopwatch.StartNew();
        var r = Run(prog, img).Tool("p")!;
        Assert.True(r.Ok, r.Message);
        Assert.True(sw.ElapsedMilliseconds < 1500, $"{sw.ElapsedMilliseconds} ms for a full turn on {W}x{H}");
    }

    [Fact]
    public void A_shape_match_draws_each_matched_outline()
    {
        using var img = Canvas(30);
        Cv2.Circle(img, new Point(100, 150), 30, Scalar.All(220), -1);
        Cv2.Circle(img, new Point(300, 150), 20, Scalar.All(220), -1);
        var tool = Tool("m", "contourMatch", null, new
        {
            polarity = "light", minScore = 0.8,
            template = new { width = 2, height = 2, points = Enumerable.Range(0, 32).Select(i => new { x = 0.5 + 0.5 * Math.Cos(i * Math.PI / 16), y = 0.5 + 0.5 * Math.Sin(i * Math.PI / 16) }) },
        });
        var r = RunOne(img, tool);
        Assert.Equal(2, Num(r, "count"));
        Assert.Equal(2, r.Contours.Count);
        Assert.All(r.Contours, c => Assert.True(c.Count >= 8));
    }
}

/// <summary>aruco: ArUco markers whose centre lies in the region.</summary>
public class ArucoToolTests
{
    private const int Dict = (int)PredefinedDictionaryType.Dict4X4_50;

    /// <summary>Marker <paramref name="id"/> (size px, plus a white quiet zone) centred on (cx, cy), turned by deg.</summary>
    private static void Marker(Mat img, int id, int cx, int cy, int size = 80, double deg = 0, PredefinedDictionaryType dict = PredefinedDictionaryType.Dict4X4_50)
    {
        using var d = CvAruco.GetPredefinedDictionary(dict);
        using var m = new Mat();
        d.GenerateImageMarker(id, size, m, 1);
        int pad = size / 4, side = size + 2 * pad;
        using var tile = new Mat(side, side, MatType.CV_8UC1, Scalar.All(255));
        m.CopyTo(new Mat(tile, new Rect(pad, pad, size, size)));
        using var turned = deg == 0 ? tile.Clone() : Rotated(tile, deg, 255);
        Paste(img, turned, cx - side / 2, cy - side / 2);
    }

    [Theory]
    [InlineData(150, 120, 0)]
    [InlineData(250, 170, 30)]
    [InlineData(200, 150, -45)]
    [InlineData(200, 150, 100)]
    public void Reads_id_centre_and_angle(int cx, int cy, double deg)
    {
        using var img = Canvas(255);
        Marker(img, 7, cx, cy, 80, deg);
        var r = RunOne(img, Tool("a", "aruco", null, new { dictionaryId = Dict }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "count"));
        Assert.Equal(7, Num(r, "id"));
        // GenerateImageMarker centres the marker on pixel (size−1)/2 of its tile.
        Near(Num(r, "x"), Num(r, "y"), (cx - 0.5, cy - 0.5), 1);
        Assert.True(Math.Abs(AngleDiff(Num(r, "angle"), deg)) < 1.5, $"angle {Num(r, "angle")}");
        Assert.True(Math.Abs(AngleDiff(r.Pose!.Angle, deg)) < 1.5);
        Assert.Equal(7, Convert.ToInt32(r.Items[0]["id"]));
        Assert.Equal(4, r.Lines.Count);                 // the marker outline
    }

    [Fact]
    public void Several_markers_and_the_markerId_filter()
    {
        using var img = Canvas(255);
        Marker(img, 3, 100, 150);
        Marker(img, 7, 300, 150);
        var all = RunOne(img, Tool("a", "aruco", null, new { dictionaryId = Dict }));
        Assert.Equal(2, Num(all, "count"));
        Assert.Equal(new[] { 3, 7 }, all.Items.Select(i => Convert.ToInt32(i["id"])).OrderBy(x => x));
        Assert.Equal(8, all.Lines.Count);

        var seven = RunOne(img, Tool("a", "aruco", null, new { dictionaryId = Dict, markerId = 7 }));
        Assert.Equal(1, Num(seven, "count"));
        Assert.Equal(7, Num(seven, "id"));
        Near(Num(seven, "x"), Num(seven, "y"), (299.5, 149.5), 1);

        var nine = RunOne(img, Tool("a", "aruco", null, new { dictionaryId = Dict, markerId = 9 }));
        Assert.Equal(ToolStatus.Fail, nine.Status);
        Assert.Equal(0, Num(nine, "count"));
        Assert.Null(nine.Pose);
    }

    [Fact]
    public void The_dictionary_must_match_unless_all_are_searched()
    {
        using var img = Canvas(255);
        Marker(img, 12, 200, 150, 96, 0, PredefinedDictionaryType.Dict6X6_250);
        Assert.Equal(ToolStatus.Fail, RunOne(img, Tool("a", "aruco", null, new { dictionaryId = Dict })).Status);
        var six = RunOne(img, Tool("a", "aruco", null, new { dictionaryId = (int)PredefinedDictionaryType.Dict6X6_250 }));
        Assert.True(six.Ok, six.Message);
        Assert.Equal(12, Num(six, "id"));
        var any = RunOne(img, Tool("a", "aruco", null, new { dictionaryId = -1 }));
        Assert.True(any.Ok, any.Message);
        Assert.Equal(1, Num(any, "count"));             // one marker, even if several dictionaries read it
    }

    [Theory]
    [InlineData(100.0, 1e9, true)]
    [InlineData(5000.0, 8000.0, true)]   // 80 px marker: ~6400 px²
    [InlineData(7000.0, 1e9, false)]
    [InlineData(100.0, 6000.0, false)]
    public void Area_limits(double minArea, double maxArea, bool found)
    {
        using var img = Canvas(255);
        Marker(img, 7, 200, 150);
        var r = RunOne(img, Tool("a", "aruco", null, new { dictionaryId = Dict, minArea, maxArea }));
        Assert.Equal(found ? ToolStatus.Ok : ToolStatus.Fail, r.Status);
    }

    [Fact]
    public void Only_markers_centred_in_the_region_count()
    {
        using var img = Canvas(255);
        Marker(img, 3, 100, 150);
        Marker(img, 7, 300, 150);
        var right = RunOne(img, Tool("a", "aruco", Rect(200, 0, 200, 300), new { dictionaryId = Dict }));
        Assert.Equal(1, Num(right, "count"));
        Assert.Equal(7, Num(right, "id"));
        // The region holds the marker's centre but not all of it: still read.
        var tight = RunOne(img, Tool("a", "aruco", Centered(300, 150, 20, 20), new { dictionaryId = Dict }));
        Assert.Equal(7, Num(tight, "id"));
        var none = RunOne(img, Tool("a", "aruco", Rect(150, 0, 100, 300), new { dictionaryId = Dict }));
        Assert.Equal(ToolStatus.Fail, none.Status);
        Assert.Equal("no marker found", none.Message);
    }

    [Fact]
    public void Limits_on_angle()
    {
        using var img = Canvas(255);
        Marker(img, 7, 200, 150, 80, 20);
        double v = AssertLimitBoundaries(img, "a", "angle", Single(() => Tool("a", "aruco", null, new { dictionaryId = Dict })));
        Assert.InRange(v, 18.5, 21.5);
    }

    [Fact]
    public void The_region_follows_the_fixture()
    {
        using var teach = Canvas(255);
        Marker(teach, 5, 200, 180, 70);
        Dots(teach);
        var prog = FixturedProgram(
            Tool("a", "aruco", Centered(200, 180, 16, 16), new { dictionaryId = Dict }, fixture: "fx"),
            Tool("still", "aruco", Centered(200, 180, 16, 16), new { dictionaryId = Dict }));
        var (r0, r1) = TeachThenMoved(prog, teach, 255);
        Assert.True(r0.Tool("still")!.Ok);
        var a = r1.Tool("a")!;
        Assert.True(a.Ok, a.Message);
        Near(Num(a, "x"), Num(a, "y"), Moved(199.5, 179.5), 1.2);
        Assert.True(Math.Abs(AngleDiff(Num(a, "angle"), MoveDeg)) < 1.5);
        Assert.Equal(ToolStatus.Fail, r1.Tool("still")!.Status);    // the moved marker's centre left the drawn region
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("a", "aruco", Centered(200, 180, 30, 30), new { dictionaryId = Dict }));

    [Fact]
    public void It_reads_its_input_layer()
    {
        // An inverted marker (white bits on black) is not read until a layer inverts it back.
        using var img = Canvas(255);
        Marker(img, 7, 200, 150);
        Cv2.BitwiseNot(img, img);
        Assert.Equal(ToolStatus.Fail, RunOne(img, Tool("a", "aruco", null, new { dictionaryId = Dict })).Status);
        var prog = Program(Tool("a", "aruco", null, new { dictionaryId = Dict }, input: "inv"));
        prog.Layers.Add(new ImageLayer { Id = "inv", Filters = { Filter("invert") } });
        var r = Run(prog, img).Tool("a")!;
        Assert.True(r.Ok, r.Message);
        Assert.Equal(7, Num(r, "id"));
    }
}
