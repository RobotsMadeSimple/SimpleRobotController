using Controller.RobotControl.Vision;
using OpenCvSharp;
using static RobotControl.Tests.VisionTestKit;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

// Edge-based locating tools (findLine, findCorner, findCircle) and makeFixture, on synthetic
// images drawn with OpenCV. Ground truth uses OpenCV's pixel convention (pixel i's centre at i),
// so a fill that starts at row 110 has its edge at y = 109.5.

/// <summary>findLine: caliper probes across a rect region, a line fitted through their edges.</summary>
public class FindLineToolTests
{
    /// <summary>Dark (40) above, light (200) below a line through (200, 150) at <paramref name="deg"/>.</summary>
    internal static Mat Edge(double deg, int dark = 40, int light = 200)
    {
        var img = Canvas(dark);
        double t = Math.Tan(deg * Math.PI / 180);
        Poly(img, Scalar.All(light), (-10, 150 - 210 * t), (410, 150 + 210 * t), (410, 400), (-10, 400));
        return img;
    }

    [Theory]
    [InlineData(-25)]
    [InlineData(-10)]
    [InlineData(0)]
    [InlineData(5)]
    [InlineData(15)]
    [InlineData(30)]
    public void Fits_an_edge_at_any_angle(double deg)
    {
        using var img = Edge(deg);
        var r = RunOne(img, Tool("l", "findLine", Centered(200, 150, 240, 200), new { probes = 12, polarity = "darkToLight" }));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.True(r.Passed);
        Assert.Equal(1, Num(r, "found"));
        Assert.InRange(Num(r, "angle"), deg - 0.3, deg + 0.3);
        Near(Num(r, "x"), Num(r, "y"), (200, 150), 1, "midpoint");   // the region centre's foot on the line
        Assert.True(Num(r, "fitError") < 0.3, $"fitError {Num(r, "fitError")}");
        Assert.Equal(12, Num(r, "pointsUsed"));
        Assert.Equal(12, r.Points.Count);
        var line = Assert.Single(r.Lines);
        Assert.True(DistToLine(line, 200, 150) < 1);
        // The segment spans the region's width: its ends are the feet of the left and right sides.
        Assert.InRange(Math.Abs(line.X2 - line.X1), 239 * Math.Cos(deg * Math.PI / 180) * Math.Cos(deg * Math.PI / 180) - 2, 242);
        Assert.NotNull(r.Pose);
        Assert.Equal(Math.Round(Num(r, "x"), 2), r.Pose!.X, 1);
        Assert.InRange(r.Pose.Angle, deg - 0.3, deg + 0.3);
        Assert.Equal(4, r.Region.Count);
    }

    [Theory]
    [InlineData(-30)]
    [InlineData(10)]
    [InlineData(45)]
    public void A_region_drawn_at_the_edges_tilt_finds_it(double deg)
    {
        using var img = Edge(deg);
        var r = RunOne(img, Tool("l", "findLine", Centered(200, 150, 200, 100, deg), new { probes = 10, polarity = "darkToLight" }));
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "angle"), deg - 0.3, deg + 0.3);
        Near(Num(r, "x"), Num(r, "y"), (200, 150), 1);
        Assert.True(Num(r, "fitError") < 0.3);
    }

    [Theory]
    // Region upright: probes run top → bottom, i.e. dark → light.
    [InlineData(0, "darkToLight", true)]
    [InlineData(0, "lightToDark", false)]
    [InlineData(0, "any", true)]
    // Region turned 180°: probes run bottom → top (light → dark) and the line points left.
    [InlineData(180, "darkToLight", false)]
    [InlineData(180, "lightToDark", true)]
    [InlineData(180, "any", true)]
    public void Polarity_is_judged_along_the_probe_direction(double rotation, string polarity, bool found)
    {
        using var img = Edge(0);
        var r = RunOne(img, Tool("l", "findLine", Centered(200, 150, 240, 100, rotation), new { polarity }));
        if (!found)
        {
            Assert.Equal(ToolStatus.Fail, r.Status);
            Assert.Equal(0, Num(r, "found"));
            Assert.Null(r.Pose);
            Assert.Empty(r.Lines);
            Assert.False(r.Passed);
            Assert.StartsWith("no line", r.Message);
            return;
        }
        Assert.True(r.Ok, r.Message);
        Assert.True(Math.Abs(AngleDiff(Num(r, "angle"), rotation)) < 0.3, $"angle {Num(r, "angle")}");
        Assert.InRange(Num(r, "y"), 149, 151);
    }

    /// <summary>Rows: 40 above y 110, 90 to y 140, 230 to y 170, 160 below — steps +50, +140, −70.</summary>
    private static Mat Steps()
    {
        var img = Canvas(40);
        Box(img, 0, 110, 400, 30, Scalar.All(90));
        Box(img, 0, 140, 400, 30, Scalar.All(230));
        Box(img, 0, 170, 400, 130, Scalar.All(160));
        return img;
    }

    [Theory]
    [InlineData("first", "any", 109.5)]
    [InlineData("strongest", "any", 139.5)]
    [InlineData("last", "any", 169.5)]
    [InlineData("last", "darkToLight", 139.5)]
    [InlineData("first", "lightToDark", 169.5)]
    [InlineData("strongest", "darkToLight", 139.5)]
    public void Pick_chooses_which_edge_each_probe_takes(string pick, string polarity, double y)
    {
        using var img = Steps();
        var r = RunOne(img, Tool("l", "findLine", Centered(200, 140, 200, 120), new { pick, polarity }));
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "y"), y - 0.6, y + 0.6);
        Assert.InRange(Num(r, "angle"), -0.2, 0.2);
        Assert.All(r.Points, p => Assert.InRange(p.Y, y - 0.6, y + 0.6));
    }

    [Theory]
    [InlineData(20, false)]
    [InlineData(16, false)]
    [InlineData(10, true)]
    public void Contrast_sets_the_weakest_step_taken(double contrast, bool found)
    {
        using var img = Edge(0, 40, 55);   // a 15-level step
        var r = RunOne(img, Tool("l", "findLine", Centered(200, 150, 240, 100), new { contrast }));
        Assert.Equal(found ? ToolStatus.Ok : ToolStatus.Fail, r.Status);
        if (found) Assert.InRange(Num(r, "y"), 149, 151);
    }

    [Fact]
    public void Ignore_drops_outliers_and_keeps_at_least_two_points()
    {
        using var img = Edge(0);
        // Two light specks above the edge, right where probes 1 and 7 of 10 run.
        Box(img, 62, 118, 16, 8, Scalar.All(200));
        Box(img, 242, 118, 16, 8, Scalar.All(200));
        object P(int ignore) => new { probes = 10, polarity = "darkToLight", pick = "first", ignore };
        var raw = RunOne(img, Tool("l", "findLine", Centered(200, 150, 300, 90), P(0)));
        var clean = RunOne(img, Tool("l", "findLine", Centered(200, 150, 300, 90), P(2)));
        Assert.True(Num(raw, "fitError") > 5, $"outliers bend the raw fit ({Num(raw, "fitError")})");
        Assert.Equal(10, Num(raw, "pointsUsed"));
        Assert.True(Num(clean, "fitError") < 0.3);
        Assert.InRange(Num(clean, "y"), 149, 151);
        Assert.InRange(Num(clean, "angle"), -0.2, 0.2);
        Assert.Equal(8, Num(clean, "pointsUsed"));
        Assert.Equal(8, clean.Points.Count);
        Assert.DoesNotContain(clean.Points, p => p.Y < 140);    // the specks are gone

        using var plain = Edge(0);
        var many = RunOne(plain, Tool("l", "findLine", Centered(200, 150, 300, 90), new { probes = 10, ignore = 50 }));
        Assert.True(many.Ok);
        Assert.Equal(2, Num(many, "pointsUsed"));
    }

    [Theory]
    [InlineData(1, 2)]       // clamped up to 2
    [InlineData(2, 2)]
    [InlineData(5, 5)]
    [InlineData(20, 20)]
    [InlineData(500, 200)]   // clamped down to 200
    public void Every_probe_contributes_a_point(int probes, int used)
    {
        using var img = Edge(8);
        var r = RunOne(img, Tool("l", "findLine", Centered(200, 150, 240, 120), new { probes }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(used, Num(r, "pointsUsed"));
        Assert.InRange(Num(r, "angle"), 7.5, 8.5);
    }

    [Fact]
    public void A_blank_region_finds_nothing()
    {
        using var img = Canvas(120);
        var r = RunOne(img, Tool("l", "findLine", Centered(200, 150, 240, 100)));
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Equal(0, Num(r, "found"));
        Assert.Contains("found 0 edge point(s), need 2", r.Message);
        Assert.Null(r.Pose);
        Assert.False(r.Passed);
    }

    [Fact]
    public void Limits_on_angle_are_inclusive_and_count_only_when_included()
    {
        using var img = Edge(10);
        double v = AssertLimitBoundaries(img, "l", "angle",
            Single(() => Tool("l", "findLine", Centered(200, 150, 240, 120), new { polarity = "darkToLight" })));
        Assert.InRange(v, 9.7, 10.3);
    }

    [Fact]
    public void The_region_follows_a_shifted_and_turned_fixture()
    {
        using var teach = Edge(5);
        Dots(teach);
        var prog = FixturedProgram(Tool("l", "findLine", Centered(200, 150, 200, 100), new { polarity = "darkToLight" }, fixture: "fx"));
        var (r0, r1) = TeachThenMoved(prog, teach, 40);
        Assert.InRange(Num(r0.Tool("l")!, "angle"), 4.7, 5.3);

        var l = r1.Tool("l")!;
        Assert.True(l.Ok, l.Message);
        Assert.InRange(Num(l, "angle"), 5 + MoveDeg - 0.4, 5 + MoveDeg + 0.4);
        // The midpoint is where the region centre's foot lands now: on the moved edge, at the moved centre.
        Near(Num(l, "x"), Num(l, "y"), Moved(200, 150), 1.2, "midpoint");
        Near(l.Region.Average(p => p.X), l.Region.Average(p => p.Y), Moved(200, 150), 0.8, "region centre");
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("l", "findLine", Centered(200, 150, 200, 100)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        // A red area below the line: invisible in camera gray, white on the red layer.
        using var img = Canvas(RedGray);
        Poly(img, Red, (-10, 150), (410, 150 + 420 * Math.Tan(12 * Math.PI / 180)), (410, 400), (-10, 400));
        var cam = RunOne(img, Tool("l", "findLine", Centered(200, 170, 240, 140), new { polarity = "darkToLight" }));
        Assert.Equal(ToolStatus.Fail, cam.Status);
        var prog = Program(Tool("l", "findLine", Centered(200, 170, 240, 140), new { polarity = "darkToLight" }, input: "red"));
        prog.Layers.Add(RedLayer());
        var r = Run(prog, img).Tool("l")!;
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "angle"), 11.6, 12.4);
    }
}

/// <summary>findCorner: two findLine sides intersected.</summary>
public class FindCornerToolTests
{
    /// <summary>A light 200 × 120 rectangle centred on (200, 150) turned by deg; its corners TL, TR, BR, BL.</summary>
    private static Mat Part(double deg, out (double X, double Y)[] corners)
    {
        var img = Canvas(40);
        corners = Turned(200, 150, 200, 120, deg);
        Poly(img, Scalar.All(200), corners);
        return img;
    }

    /// <summary>Side A on the top side (probes outside → in), side B on the left side (also outside → in).</summary>
    private static VisionTool Corner((double X, double Y)[] k, double deg, object? extra = null, string? fixture = null)
    {
        var topMid = ((k[0].X + k[1].X) / 2, (k[0].Y + k[1].Y) / 2);
        var leftMid = ((k[0].X + k[3].X) / 2, (k[0].Y + k[3].Y) / 2);
        var p = new Dictionary<string, object?>
        {
            ["polarity"] = "darkToLight",
            ["probes"] = 10,
            ["regionB"] = Centered(leftMid.Item1, leftMid.Item2, 80, 50, deg - 90),
        };
        if (extra != null)
            foreach (var prop in extra.GetType().GetProperties()) p[prop.Name] = prop.GetValue(extra);
        return Tool("c", "findCorner", Centered(topMid.Item1, topMid.Item2, 120, 50, deg), p, fixture: fixture);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(15)]
    [InlineData(-20)]
    public void Finds_the_corner_of_a_turned_part(double deg)
    {
        using var img = Part(deg, out var k);
        var r = RunOne(img, Corner(k, deg));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "found"));
        // The anti-aliased fill is drawn from integer vertices and spreads up to ~½ px past its
        // outline, so a turned corner is known to about 1.3 px.
        Near(Num(r, "x"), Num(r, "y"), k[0], 1.3, "corner");
        Assert.InRange(Num(r, "angle"), deg - 0.3, deg + 0.3);          // side A's direction
        Assert.InRange(Num(r, "cornerAngle"), 89.5, 90.5);
        Assert.Equal(2, r.Lines.Count);
        Near(r.Points[0].X, r.Points[0].Y, (Num(r, "x"), Num(r, "y")), 0.01, "corner point");
        Assert.Equal(1 + 20, r.Points.Count);                            // the corner, then both sides' edge points
        Near(r.Pose!.X, r.Pose.Y, (Num(r, "x"), Num(r, "y")), 0.01, "pose");
        Assert.InRange(r.Pose.Angle, deg - 0.3, deg + 0.3);
    }

    [Theory]
    // cornerAngle is the angle between the two sides' directions as their regions are drawn.
    [InlineData(-120, 120)]   // side B drawn from outside in (its probes run dark → light)
    [InlineData(60, 60)]      // side B drawn the other way round (light → dark)
    public void Measures_a_corner_that_is_not_square(double rotationB, double cornerAngle)
    {
        using var img = Canvas(40);
        // A triangle with a 60° corner at (100, 80): a top side going right, a side going down-right at 60°.
        var b = (100 + 160 * Math.Cos(Math.PI / 3), 80 + 160 * Math.Sin(Math.PI / 3));
        Poly(img, Scalar.All(200), (100, 80), (330, 80), b);
        var regionB = Centered((100 + b.Item1) / 2, (80 + b.Item2) / 2, 80, 40, rotationB);
        var r = RunOne(img, Tool("c", "findCorner", Centered(200, 80, 120, 40), new { polarity = "any", regionB }));
        Assert.True(r.Ok, r.Message);
        Near(Num(r, "x"), Num(r, "y"), (100, 80), 1.2, "corner");
        Assert.InRange(Num(r, "cornerAngle"), cornerAngle - 0.5, cornerAngle + 0.5);
        Assert.InRange(Num(r, "angle"), -0.3, 0.3);
        var strict = RunOne(img, Tool("c", "findCorner", Centered(200, 80, 120, 40), new { polarity = "darkToLight", regionB }));
        // Side A runs dark → light; side B only when drawn from outside in.
        Assert.Equal(rotationB == -120 ? ToolStatus.Ok : ToolStatus.Fail, strict.Status);
        if (!strict.Ok) Assert.StartsWith("side B", strict.Message);
    }

    [Fact]
    public void Without_side_B_it_fails()
    {
        using var img = Part(0, out var k);
        var r = RunOne(img, Tool("c", "findCorner", Centered(200, 90, 120, 50)));
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Contains("regionB", r.Message);
        Assert.Equal(0, Num(r, "found"));
        Assert.Null(r.Pose);
    }

    [Fact]
    public void A_side_without_an_edge_fails_and_names_the_side()
    {
        using var img = Part(0, out var k);
        // Side A over the part's plain middle.
        var a = RunOne(img, Tool("c", "findCorner", Centered(200, 150, 100, 40),
            new { polarity = "darkToLight", regionB = Centered(100, 150, 80, 50, -90) }));
        Assert.Equal(ToolStatus.Fail, a.Status);
        Assert.StartsWith("side A", a.Message);
        // Side B over the dark background.
        var b = RunOne(img, Tool("c", "findCorner", Centered(200, 90, 120, 50),
            new { polarity = "darkToLight", regionB = Centered(40, 150, 60, 30, -90) }));
        Assert.Equal(ToolStatus.Fail, b.Status);
        Assert.StartsWith("side B", b.Message);
        Assert.Equal(0, Num(b, "found"));
        // The wrong polarity for both: side A is reported first.
        var both = RunOne(img, Corner(k, 0, new { polarity = "lightToDark" }));
        Assert.StartsWith("side A", both.Message);
    }

    [Fact]
    public void Two_parallel_sides_have_no_corner()
    {
        using var img = Canvas(40);
        Box(img, 0, 150, 400, 150, Scalar.All(200));   // one straight horizontal edge
        var r = RunOne(img, Tool("c", "findCorner", Centered(100, 150, 120, 50),
            new { polarity = "darkToLight", regionB = Centered(300, 150, 120, 50) }));
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Contains("parallel", r.Message);
    }

    [Fact]
    public void Limits_on_cornerAngle()
    {
        using var img = Part(10, out var k);
        double v = AssertLimitBoundaries(img, "c", "cornerAngle", Single(() => Corner(k, 10)));
        Assert.InRange(v, 89.5, 90.5);
    }

    [Fact]
    public void Both_sides_follow_the_fixture()
    {
        using var teach = Part(0, out var k);
        Dots(teach);
        var prog = FixturedProgram(Corner(k, 0, fixture: "fx"));
        var (_, r1) = TeachThenMoved(prog, teach, 40);
        var c = r1.Tool("c")!;
        Assert.True(c.Ok, c.Message);
        Near(Num(c, "x"), Num(c, "y"), Moved(k[0].X, k[0].Y), 1.2, "corner");
        Assert.InRange(Num(c, "angle"), MoveDeg - 0.4, MoveDeg + 0.4);
        Assert.InRange(Num(c, "cornerAngle"), 89.4, 90.6);
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run()
    {
        var k = Turned(200, 150, 200, 120, 0);
        AssertNotRunWithoutFixture(Corner(k, 0));
    }

    [Fact]
    public void It_reads_its_input_layer()
    {
        using var img = Canvas(RedGray);
        var k = Turned(200, 150, 200, 120, 0);
        Poly(img, Red, k);
        Assert.Equal(ToolStatus.Fail, RunOne(img, Corner(k, 0)).Status);
        var t = Corner(k, 0);
        t.Input = "red";
        var prog = Program(t);
        prog.Layers.Add(RedLayer());
        var r = Run(prog, img).Tool("c")!;
        Assert.True(r.Ok, r.Message);
        Near(Num(r, "x"), Num(r, "y"), k[0], 1.2);
    }
}

/// <summary>findCircle: radial probes across an annulus and a circle fit.</summary>
public class FindCircleToolTests
{
    private static Mat Disc(double cx, double cy, double r, int bg = 40, int fg = 200)
    {
        var img = Canvas(bg);
        Cv2.Circle(img, new Point((int)cx, (int)cy), (int)r, Scalar.All(fg), -1, LineTypes.AntiAlias);
        return img;
    }

    [Theory]
    [InlineData(210, 140, 60)]
    [InlineData(190, 160, 45)]
    [InlineData(200, 150, 30)]
    public void Fits_centre_and_radius(int cx, int cy, int radius)
    {
        using var img = Disc(cx, cy, radius);
        var r = RunOne(img, Tool("c", "findCircle", Circle(200, 150, 85), new { innerRatio = 0.2, probes = 24, polarity = "darkToLight" }));
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "found"));
        Near(Num(r, "x"), Num(r, "y"), (cx, cy), 0.5, "centre");
        Assert.InRange(Num(r, "radius"), radius - 0.7, radius + 0.7);
        Assert.True(Num(r, "fitError") < 0.5, $"fitError {Num(r, "fitError")}");
        Assert.Equal(24, Num(r, "pointsUsed"));
        Assert.Equal(24, r.Points.Count);
        Assert.All(r.Points, p => Assert.InRange(Math.Sqrt((p.X - cx) * (p.X - cx) + (p.Y - cy) * (p.Y - cy)), radius - 1.2, radius + 1.2));
        Near(r.Pose!.X, r.Pose.Y, (cx, cy), 0.5, "pose");
        Assert.Equal(0, r.Pose.Angle);
    }

    [Theory]
    // A light disc on dark: inward probes rise (dark → light), outward probes fall.
    [InlineData(true, "inward", "darkToLight", true)]
    [InlineData(true, "inward", "lightToDark", false)]
    [InlineData(true, "inward", "any", true)]
    [InlineData(true, "outward", "lightToDark", true)]
    [InlineData(true, "outward", "darkToLight", false)]
    [InlineData(true, "outward", "any", true)]
    // A dark disc on light: the other way round.
    [InlineData(false, "inward", "lightToDark", true)]
    [InlineData(false, "inward", "darkToLight", false)]
    [InlineData(false, "outward", "darkToLight", true)]
    [InlineData(false, "outward", "lightToDark", false)]
    public void Direction_and_polarity(bool lightDisc, string direction, string polarity, bool found)
    {
        using var img = lightDisc ? Disc(205, 145, 50) : Disc(205, 145, 50, 200, 40);
        var r = RunOne(img, Tool("c", "findCircle", Circle(200, 150, 85), new { innerRatio = 0.25, direction, polarity }));
        if (!found)
        {
            Assert.Equal(ToolStatus.Fail, r.Status);
            Assert.Equal(0, Num(r, "found"));
            Assert.Null(r.Pose);
            return;
        }
        Assert.True(r.Ok, r.Message);
        Near(Num(r, "x"), Num(r, "y"), (205, 145), 0.6);
        Assert.InRange(Num(r, "radius"), 49.3, 50.7);
    }

    [Theory]
    // A light ring (r 70) round a mid-gray disc (r 40): steps +160 at 70 and −80 at 40.
    [InlineData("inward", "first", 70)]
    [InlineData("inward", "last", 40)]
    [InlineData("inward", "strongest", 70)]
    [InlineData("outward", "first", 40)]
    [InlineData("outward", "last", 70)]
    [InlineData("outward", "strongest", 70)]
    public void Pick_chooses_between_concentric_edges(string direction, string pick, double radius)
    {
        using var img = Disc(200, 150, 70);
        Cv2.Circle(img, new Point(200, 150), 40, Scalar.All(120), -1, LineTypes.AntiAlias);
        var r = RunOne(img, Tool("c", "findCircle", Circle(200, 150, 90), new { innerRatio = 0.2, direction, pick, polarity = "any" }));
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "radius"), radius - 0.8, radius + 0.8);
        Near(Num(r, "x"), Num(r, "y"), (200, 150), 0.5);
    }

    [Fact]
    public void Ignore_drops_outlier_points()
    {
        using var img = Disc(200, 150, 50);
        // Light specks outside the rim on the probes at 0° and 90°: with pick first they are taken.
        Box(img, 266, 146, 8, 9, Scalar.All(200));
        Box(img, 196, 216, 9, 8, Scalar.All(200));
        object P(int ignore) => new { probes = 16, innerRatio = 0.2, polarity = "darkToLight", pick = "first", ignore };
        var raw = RunOne(img, Tool("c", "findCircle", Circle(200, 150, 85), P(0)));
        var clean = RunOne(img, Tool("c", "findCircle", Circle(200, 150, 85), P(2)));
        Assert.True(Num(raw, "fitError") > 3, $"raw fitError {Num(raw, "fitError")}");
        Assert.Equal(16, Num(raw, "pointsUsed"));
        Assert.True(Num(clean, "fitError") < 0.5, $"clean fitError {Num(clean, "fitError")}");
        Assert.Equal(14, Num(clean, "pointsUsed"));
        Assert.InRange(Num(clean, "radius"), 49.3, 50.7);
        Near(Num(clean, "x"), Num(clean, "y"), (200, 150), 0.5);
    }

    [Theory]
    [InlineData(20, false)]
    [InlineData(8, true)]
    public void Contrast_sets_the_weakest_rim_taken(double contrast, bool found)
    {
        using var img = Disc(200, 150, 50, 40, 52);
        var r = RunOne(img, Tool("c", "findCircle", Circle(200, 150, 85), new { innerRatio = 0.2, contrast }));
        Assert.Equal(found ? ToolStatus.Ok : ToolStatus.Fail, r.Status);
        if (found) Assert.InRange(Num(r, "radius"), 49, 51);
    }

    [Fact]
    public void A_rect_region_probes_its_inscribed_circle()
    {
        using var img = Disc(210, 140, 60);
        var r = RunOne(img, Tool("c", "findCircle", Centered(210, 140, 200, 170), new { innerRatio = 0.3 }));
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "radius"), 59.3, 60.7);
        Near(Num(r, "x"), Num(r, "y"), (210, 140), 0.5);
    }

    [Fact]
    public void A_blank_region_finds_nothing()
    {
        using var img = Canvas(90);
        var r = RunOne(img, Tool("c", "findCircle", Circle(200, 150, 85)));
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Contains("found 0 edge point(s), need 3", r.Message);
        Assert.Equal(0, Num(r, "found"));
        Assert.Null(r.Pose);
        Assert.False(r.Passed);
    }

    [Fact]
    public void Limits_on_radius()
    {
        using var img = Disc(200, 150, 55);
        double v = AssertLimitBoundaries(img, "c", "radius", Single(() => Tool("c", "findCircle", Circle(200, 150, 85), new { innerRatio = 0.3 })));
        Assert.InRange(v, 54.3, 55.7);
    }

    [Fact]
    public void The_annulus_follows_the_fixture()
    {
        using var teach = Disc(220, 160, 40);
        Dots(teach);
        var prog = FixturedProgram(
            Tool("c", "findCircle", Circle(220, 160, 60), new { innerRatio = 0.3, polarity = "darkToLight" }, fixture: "fx"),
            Tool("still", "findCircle", Circle(220, 160, 60), new { innerRatio = 0.3, polarity = "darkToLight" }));
        var (_, r1) = TeachThenMoved(prog, teach, 40);
        var c = r1.Tool("c")!;
        Assert.True(c.Ok, c.Message);
        Near(Num(c, "x"), Num(c, "y"), Moved(220, 160), 0.8, "centre");
        Assert.InRange(Num(c, "radius"), 39.3, 40.7);
        Near(c.Region.Average(p => p.X), c.Region.Average(p => p.Y), Moved(220, 160), 0.5, "annulus centre");
        Assert.InRange(r1.Tool("still")!.Region.Average(p => p.X), 219.5, 220.5);   // unfixtured: stays where drawn
    }

    [Fact]
    public void Without_its_fixture_it_does_not_run() =>
        AssertNotRunWithoutFixture(Tool("c", "findCircle", Circle(220, 160, 60)));

    [Fact]
    public void It_reads_its_input_layer()
    {
        using var img = Canvas(RedGray);
        Cv2.Circle(img, new Point(200, 150), 50, Red, -1, LineTypes.AntiAlias);
        Assert.Equal(ToolStatus.Fail, RunOne(img, Tool("c", "findCircle", Circle(200, 150, 85), new { innerRatio = 0.2 })).Status);
        var prog = Program(Tool("c", "findCircle", Circle(200, 150, 85), new { innerRatio = 0.2 }, input: "red"));
        prog.Layers.Add(RedLayer());
        var r = Run(prog, img).Tool("c")!;
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "radius"), 49, 51);
    }
}

/// <summary>makeFixture: a pose (and quad) built from other tools' results.</summary>
public class MakeFixtureToolTests
{
    /// <summary>
    /// A light 200 × 140 rectangle (corners (100,80) (300,80) (300,220) (100,220) in fill
    /// coordinates) and four light dots (60,40) (340,40) (340,260) (60,260), with locating tools
    /// for its four sides and the dots.
    /// </summary>
    private static Mat Scene()
    {
        var img = Canvas(40);
        Box(img, 100, 80, 200, 140, Scalar.All(200));
        foreach (var (x, y) in Dots4) Cv2.Circle(img, new Point(x, y), 8, Scalar.All(200), -1);
        return img;
    }

    private static readonly (int X, int Y)[] Dots4 = { (60, 40), (340, 40), (340, 260), (60, 260) };

    // The rectangle's edges in OpenCV's pixel convention.
    private const double Left = 99.5, Right = 299.5, Top = 79.5, Bottom = 219.5;

    private static List<VisionTool> Locators() => new()
    {
        Tool("top",    "findLine", Centered(200, 80, 120, 40, 0),    new { polarity = "darkToLight" }),
        Tool("right",  "findLine", Centered(300, 150, 80, 40, 90),   new { polarity = "darkToLight" }),
        Tool("bottom", "findLine", Centered(200, 220, 120, 40, 180), new { polarity = "darkToLight" }),
        Tool("left",   "findLine", Centered(100, 150, 80, 40, -90),  new { polarity = "darkToLight" }),
        Tool("d1", "blob", Centered(60, 40, 40, 30),   new { polarity = "light" }),
        Tool("d2", "blob", Centered(340, 40, 40, 30),  new { polarity = "light" }),
        Tool("d3", "blob", Centered(340, 260, 40, 30), new { polarity = "light" }),
        Tool("d4", "blob", Centered(60, 260, 40, 30),  new { polarity = "light" }),
        Tool("cal", "caliper", Centered(200, 150, 260, 20), new { mode = "single", pick = "first" }),   // points only, no pose
        Tool("poly", "polygon", Rect(80, 60, 240, 180), new { sides = 4 }),                             // values x/y only
    };

    private static ToolResult RunFixture(Mat img, object p, out VisionResult all)
    {
        var tools = Locators();
        tools.Add(Tool("fx", "makeFixture", null, p));
        all = Run(Program(tools.ToArray()), img);
        return all.Tool("fx")!;
    }

    private static double Deg(double dx, double dy) => Math.Atan2(dy, dx) * 180 / Math.PI;

    [Fact]
    public void PointAngle_takes_the_origin_from_one_source_and_the_angle_from_another()
    {
        using var img = Scene();
        var r = RunFixture(img, new { mode = "pointAngle", sources = new[] { "d3", "left" } }, out _);
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "found"));
        Near(r.Pose!.X, r.Pose.Y, (340, 260), 0.1);
        Assert.InRange(r.Pose.Angle, -90.3, -89.7);         // the left side's region points up
        Assert.Null(r.Pose.Quad);
        Near(Num(r, "x"), Num(r, "y"), (340, 260), 0.1);
        Assert.Equal(r.Pose.Angle, Num(r, "angle"), 2);
        Near(r.Points.Single().X, r.Points.Single().Y, (340, 260), 0.1);

        // Mode defaults to pointAngle.
        var dflt = RunFixture(img, new { sources = new[] { "d3", "left" } }, out _);
        Assert.Equal(r.Pose.X, dflt.Pose!.X);
        // Source 2 must have a pose (a caliper has only points).
        var noPose = RunFixture(img, new { mode = "pointAngle", sources = new[] { "d1", "cal" } }, out _);
        Assert.Equal(ToolStatus.Fail, noPose.Status);
        Assert.Contains("no pose", noPose.Message);
        Assert.Null(noPose.Pose);
    }

    [Fact]
    public void TwoPoints_points_from_the_first_source_to_the_second()
    {
        using var img = Scene();
        var r = RunFixture(img, new { mode = "twoPoints", sources = new[] { "d1", "d3" } }, out _);
        Assert.True(r.Ok, r.Message);
        Near(r.Pose!.X, r.Pose.Y, (60, 40), 0.1);
        Assert.InRange(r.Pose.Angle, Deg(280, 220) - 0.1, Deg(280, 220) + 0.1);
        var back = RunFixture(img, new { mode = "twoPoints", sources = new[] { "d3", "d1" } }, out _);
        Near(back.Pose!.X, back.Pose.Y, (340, 260), 0.1);
        Assert.InRange(back.Pose.Angle, Deg(-280, -220) - 0.1, Deg(-280, -220) + 0.1);

        // Points may also come from a tool's first point or its x/y values.
        var mixed = RunFixture(img, new { mode = "twoPoints", sources = new[] { "cal", "poly" } }, out var all);
        Assert.True(mixed.Ok, mixed.Message);
        Near(mixed.Pose!.X, mixed.Pose.Y, (all.Tool("cal")!.Points[0].X, all.Tool("cal")!.Points[0].Y), 0.01);
        Assert.InRange(mixed.Pose.X, Left - 0.6, Left + 0.6);
        Assert.InRange(mixed.Pose.Angle, -0.5, 0.5);          // to the square's centre, (199.5, 149.5)
    }

    [Fact]
    public void LineIntersection_crosses_two_lines_and_takes_the_first_ones_angle()
    {
        using var img = Scene();
        var r = RunFixture(img, new { mode = "lineIntersection", sources = new[] { "top", "left" } }, out _);
        Assert.True(r.Ok, r.Message);
        Near(r.Pose!.X, r.Pose.Y, (Left, Top), 0.6);
        Assert.InRange(r.Pose.Angle, -0.2, 0.2);
        var other = RunFixture(img, new { mode = "lineIntersection", sources = new[] { "right", "bottom" } }, out _);
        Near(other.Pose!.X, other.Pose.Y, (Right, Bottom), 0.6);
        Assert.InRange(other.Pose.Angle, 89.8, 90.2);

        var parallel = RunFixture(img, new { mode = "lineIntersection", sources = new[] { "top", "bottom" } }, out _);
        Assert.Equal(ToolStatus.Fail, parallel.Status);
        Assert.Contains("parallel", parallel.Message);
        var noLine = RunFixture(img, new { mode = "lineIntersection", sources = new[] { "top", "d1" } }, out _);
        Assert.Equal(ToolStatus.Fail, noLine.Status);
        Assert.Contains("no line", noLine.Message);
    }

    [Fact]
    public void FourLines_builds_a_perspective_quad_from_the_corners()
    {
        using var img = Scene();
        var r = RunFixture(img, new { mode = "fourLines", sources = new[] { "top", "right", "bottom", "left" } }, out _);
        Assert.True(r.Ok, r.Message);
        var want = new[] { (Left, Top), (Right, Top), (Right, Bottom), (Left, Bottom) };
        Assert.Equal(4, r.Pose!.Quad!.Count);
        for (int i = 0; i < 4; i++) Near(r.Pose.Quad[i].X, r.Pose.Quad[i].Y, want[i], 0.6, $"corner {i}");
        Near(r.Pose.X, r.Pose.Y, ((Left + Right) / 2, (Top + Bottom) / 2), 0.4, "centroid");
        Assert.InRange(r.Pose.Angle, -0.2, 0.2);              // TL → TR
        Assert.Equal(4, r.Points.Count);
        Assert.Equal(4, r.Lines.Count);                       // the quad outline

        // Sources out of order make a bow-tie: not convex.
        var twisted = RunFixture(img, new { mode = "fourLines", sources = new[] { "left", "right", "bottom", "top" } }, out _);
        Assert.Equal(ToolStatus.Fail, twisted.Status);
        var parallel = RunFixture(img, new { mode = "fourLines", sources = new[] { "top", "bottom", "right", "left" } }, out _);
        Assert.Equal(ToolStatus.Fail, parallel.Status);
        Assert.Contains("parallel", parallel.Message);
    }

    [Fact]
    public void FourPoints_takes_the_corners_from_four_point_sources()
    {
        using var img = Scene();
        var r = RunFixture(img, new { mode = "fourPoints", sources = new[] { "d1", "d2", "d3", "d4" } }, out _);
        Assert.True(r.Ok, r.Message);
        for (int i = 0; i < 4; i++) Near(r.Pose!.Quad![i].X, r.Pose.Quad[i].Y, (Dots4[i].X, Dots4[i].Y), 0.1, $"corner {i}");
        Near(r.Pose!.X, r.Pose.Y, (200, 150), 0.1);
        Assert.InRange(r.Pose.Angle, -0.1, 0.1);

        var bowTie = RunFixture(img, new { mode = "fourPoints", sources = new[] { "d1", "d2", "d4", "d3" } }, out _);
        Assert.Equal(ToolStatus.Fail, bowTie.Status);
        Assert.Contains("convex", bowTie.Message);
        Assert.Null(bowTie.Pose);
        var unknown = RunFixture(img, new { mode = "fourPoints", sources = new[] { "d1", "d2", "d3", "top2" } }, out _);
        Assert.Equal(ToolStatus.NotRun, unknown.Status);      // an unknown source id
        // A source with no point (an angle tool has only lines).
        var tools = Locators();
        tools.Add(Tool("ang", "angle", null, new { sources = new[] { "top" } }));
        tools.Add(Tool("fx", "makeFixture", null, new { mode = "fourPoints", sources = new[] { "d1", "d2", "d3", "ang" } }));
        var noPoint = Run(Program(tools.ToArray()), img).Tool("fx")!;
        Assert.Equal(ToolStatus.Fail, noPoint.Status);
        Assert.Contains("no point", noPoint.Message);
    }

    [Theory]
    [InlineData("pointAngle", 1, 2)]
    [InlineData("twoPoints", 1, 2)]
    [InlineData("lineIntersection", 1, 2)]
    [InlineData("fourLines", 3, 4)]
    [InlineData("fourPoints", 2, 4)]
    public void Too_few_sources_fail(string mode, int given, int need)
    {
        using var img = Scene();
        var r = RunFixture(img, new { mode, sources = new[] { "top", "right", "bottom" }.Take(given).ToArray() }, out _);
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Contains($"needs {need} sources", r.Message);
        Assert.Equal(0, Num(r, "found"));
    }

    [Fact]
    public void A_failed_or_unknown_source_leaves_it_not_run()
    {
        using var img = Scene();
        var tools = Locators();
        tools.Add(Tool("gone", "findLine", Centered(200, 150, 100, 40), new { polarity = "darkToLight" }));   // plain middle: fails
        tools.Add(Tool("fx", "makeFixture", null, new { mode = "twoPoints", sources = new[] { "d1", "gone" } }));
        tools.Add(Tool("fy", "makeFixture", null, new { mode = "twoPoints", sources = new[] { "d1", "later" } }));
        tools.Add(Tool("later", "blob"));
        tools.Add(Tool("dep", "intensity", Centered(200, 150, 40, 40), null, fixture: "fx"));
        var r = Run(Program(tools.ToArray()), img);
        Assert.Equal(ToolStatus.Fail, r.Tool("gone")!.Status);
        Assert.Equal(ToolStatus.NotRun, r.Tool("fx")!.Status);
        Assert.Contains("source 'gone'", r.Tool("fx")!.Message);
        Assert.Equal(ToolStatus.NotRun, r.Tool("fy")!.Status);
        Assert.Contains("not an earlier tool", r.Tool("fy")!.Message);
        Assert.Equal(ToolStatus.NotRun, r.Tool("dep")!.Status);
        Assert.False(r.Passed);
    }

    [Fact]
    public void Limits_on_angle()
    {
        using var img = Scene();
        AssertLimitBoundaries(img, "fx", "angle", (lim, inc) =>
        {
            var tools = Locators();
            tools.Add(Tool("fx", "makeFixture", null, new { mode = "twoPoints", sources = new[] { "d1", "d3" } }, limits: lim, includeInPass: inc));
            return Program(tools.ToArray());
        });
    }

    [Theory]
    [InlineData("twoPoints")]
    [InlineData("fourPoints")]
    public void Its_pose_follows_a_moved_part(string mode)
    {
        using var teach = Scene();
        using var moved = MoveScene(teach, 40);
        // Dot regions big enough to catch the dots in both scenes.
        var tools = new List<VisionTool>();
        for (int i = 0; i < 4; i++)
        {
            var c = ((Dots4[i].X + Moved(Dots4[i].X, Dots4[i].Y).X) / 2, (Dots4[i].Y + Moved(Dots4[i].X, Dots4[i].Y).Y) / 2);
            tools.Add(Tool($"d{i + 1}", "blob", Centered(c.Item1, c.Item2, 60, 60), new { polarity = "light", minArea = 50, maxArea = 400 }));
        }
        tools.Add(Tool("fx", "makeFixture", null, new { mode, sources = mode == "twoPoints" ? new[] { "d1", "d2" } : new[] { "d1", "d2", "d3", "d4" } }));
        var r = Run(Program(tools.ToArray()), moved).Tool("fx")!;
        Assert.True(r.Ok, r.Message);
        Assert.InRange(r.Pose!.Angle, MoveDeg - 0.2, MoveDeg + 0.2);
        Near(r.Pose.X, r.Pose.Y, mode == "twoPoints" ? Moved(60, 40) : Moved(200, 150), 0.5);
        if (mode == "fourPoints")
            for (int i = 0; i < 4; i++) Near(r.Pose.Quad![i].X, r.Pose.Quad[i].Y, Moved(Dots4[i].X, Dots4[i].Y), 0.5, $"corner {i}");
    }
}
