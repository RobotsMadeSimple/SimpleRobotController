using Controller.RobotControl.Vision;
using Controller.RobotControl.Vision.Tools;
using OpenCvSharp;
using OpenCvSharp.Aruco;
using static RobotControl.Tests.VisionTestKit;

namespace RobotControl.Tests;

/// <summary>Every tool of the catalogue on synthetic images drawn with OpenCV.</summary>
public class VisionToolTests
{
    // ── findLine / findCorner / findCircle ──────────────────────────────────────

    /// <summary>Dark above, light below a line through (200, 150) at <paramref name="deg"/>.</summary>
    private static Mat Edge(double deg)
    {
        var img = Canvas(40);
        double t = Math.Tan(deg * Math.PI / 180);
        Poly(img, Scalar.All(200), (-10, 150 - 210 * t), (410, 150 + 210 * t), (410, 400), (-10, 400));
        return img;
    }

    [Fact]
    public void FindLine_fits_the_edge_its_probes_cross()
    {
        using var img = Edge(10);
        var r = RunOne(img, Tool("l", "findLine", Centered(200, 150, 300, 90), new { probes = 10, polarity = "darkToLight" }));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.InRange(Num(r, "angle"), 9.5, 10.5);
        Assert.InRange(Num(r, "y"), 149, 151);         // the midpoint: the region centre's foot on the line
        Assert.InRange(Num(r, "x"), 199, 201);
        Assert.True(Num(r, "fitError") < 0.5);
        Assert.Equal(10, Num(r, "pointsUsed"));
        Assert.Single(r.Lines);
        Assert.NotNull(r.Pose);
        Assert.InRange(r.Pose!.Angle, 9.5, 10.5);

        // A region drawn at the same tilt finds the same line.
        var tilted = RunOne(img, Tool("l", "findLine", Centered(200, 150, 300, 60, 10), new { probes = 10 }));
        Assert.InRange(Num(tilted, "angle"), 9.5, 10.5);

        // Wrong polarity: nothing found, and so no pose.
        var none = RunOne(img, Tool("l", "findLine", Centered(200, 150, 300, 90), new { polarity = "lightToDark" }));
        Assert.Equal(ToolStatus.Fail, none.Status);
        Assert.Equal(0, Num(none, "found"));
        Assert.Null(none.Pose);
        Assert.False(none.Passed);
    }

    [Fact]
    public void FindLine_drops_the_ignored_outliers()
    {
        using var img = Edge(0);
        // Two light specks above the edge, right where two probes run.
        Cv2.Rectangle(img, new Rect(62, 118, 16, 8), Scalar.All(200), -1);
        Cv2.Rectangle(img, new Rect(242, 118, 16, 8), Scalar.All(200), -1);
        object P(int ignore) => new { probes = 10, polarity = "darkToLight", pick = "first", ignore };
        var raw = RunOne(img, Tool("l", "findLine", Centered(200, 150, 300, 90), P(0)));
        var clean = RunOne(img, Tool("l", "findLine", Centered(200, 150, 300, 90), P(2)));
        Assert.True(Num(raw, "fitError") > 5, $"outliers bend the raw fit ({Num(raw, "fitError")})");
        Assert.True(Num(clean, "fitError") < 0.5);
        Assert.InRange(Num(clean, "y"), 149, 151);
        Assert.Equal(8, Num(clean, "pointsUsed"));
    }

    [Fact]
    public void FindCorner_intersects_two_sides()
    {
        using var img = Canvas(40);
        Cv2.Rectangle(img, new Rect(120, 100, 160, 100), Scalar.All(200), -1);
        var regionB = Centered(120, 150, 80, 50, 90);   // along the left side
        var r = RunOne(img, Tool("c", "findCorner", Centered(200, 100, 100, 50), new { polarity = "any", regionB }));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.InRange(Num(r, "x"), 119, 121);
        Assert.InRange(Num(r, "y"), 99, 101);
        Assert.InRange(Num(r, "cornerAngle"), 89, 91);
        Assert.Equal(2, r.Lines.Count);

        var noB = RunOne(img, Tool("c", "findCorner", Centered(200, 100, 100, 50)));
        Assert.Equal(ToolStatus.Fail, noB.Status);
    }

    [Fact]
    public void FindCircle_fits_the_rim_from_radial_probes()
    {
        using var img = Canvas(40);
        Cv2.Circle(img, new Point(210, 140), 60, Scalar.All(200), -1, LineTypes.AntiAlias);
        var r = RunOne(img, Tool("c", "findCircle", Circle(200, 150, 85), new { innerRatio = 0.4, probes = 24, direction = "inward", polarity = "darkToLight" }));
        Assert.True(r.Ok, r.Message);
        Assert.InRange(Num(r, "x"), 209, 211);
        Assert.InRange(Num(r, "y"), 139, 141);
        Assert.InRange(Num(r, "radius"), 59, 61);
        Assert.True(Num(r, "fitError") < 1);
        Assert.Equal(24, Num(r, "pointsUsed"));

        // Outward from the centre the rim falls light → dark too.
        var outward = RunOne(img, Tool("c", "findCircle", Circle(200, 150, 85), new { innerRatio = 0.1, direction = "outward", polarity = "lightToDark" }));
        Assert.InRange(Num(outward, "radius"), 59, 61);
    }

    // ── blob / contourMatch / patternMatch / aruco ──────────────────────────────

    [Fact]
    public void Blob_counts_sorts_and_filters()
    {
        using var img = Canvas(220);
        Cv2.Circle(img, new Point(80, 150), 10, Scalar.All(30), -1);
        Cv2.Circle(img, new Point(200, 150), 20, Scalar.All(30), -1);
        Cv2.Circle(img, new Point(320, 150), 30, Scalar.All(30), -1);
        var r = RunOne(img, Tool("b", "blob", null, new { polarity = "dark" }));
        Assert.Equal(3, Num(r, "count"));
        Assert.InRange((double)r.Items[0]["x"], 319, 321);                 // biggest first
        Assert.InRange((double)r.Items[0]["area"], 2700, 2900);
        Assert.True((double)r.Items[0]["circularity"] > 0.85);
        Assert.InRange(r.Pose!.X, 319, 321);

        var big = RunOne(img, Tool("b", "blob", null, new { polarity = "dark", minArea = 500, sort = "x" }));
        Assert.Equal(2, Num(big, "count"));
        Assert.InRange((double)big.Items[0]["x"], 199, 201);

        var light = RunOne(img, Tool("b", "blob", Rect(150, 100, 100, 100), new { polarity = "dark", threshold = "fixed", level = 128, minArea = 5000 }));
        Assert.Equal(ToolStatus.Ok, light.Status);                           // none is a valid count
        Assert.Equal(0, Num(light, "count"));
        Assert.Null(light.Pose);
    }

    [Fact]
    public void ContourMatch_finds_the_taught_shape_turned_and_scaled()
    {
        using var teach = Canvas(30);
        Poly(teach, Scalar.All(220), (80, 60), (140, 180), (40, 160));
        var prog = Program(Tool("m", "contourMatch", Rect(20, 40, 140, 160), new { polarity = "light", minScore = 0.8 }));
        using (var rt = new VisionRuntime()) VisionEngine.Teach(prog, "m", teach, rt);
        Assert.True(prog.Tools[0].P.Has("template"));

        // The same triangle 1.3× bigger, centred on (270, 150) and turned 30°, among a square and a circle.
        var taught = new[] { (80.0, 60.0), (140.0, 180.0), (40.0, 160.0) };
        double mx = taught.Average(p => p.Item1), my = taught.Average(p => p.Item2);
        var tri = taught.Select(p => TurnPoint(270 + (p.Item1 - mx) * 1.3, 150 + (p.Item2 - my) * 1.3, 270, 150, 30)).ToArray();
        using var img = Canvas(30);
        Poly(img, Scalar.All(220), tri);
        Cv2.Rectangle(img, new Rect(30, 40, 60, 60), Scalar.All(220), -1);
        Cv2.Circle(img, new Point(80, 220), 35, Scalar.All(220), -1);
        prog.Tools[0].Region = null;
        var r = Run(prog, img).Tool("m")!;
        Assert.True(r.Ok, r.Message);
        Assert.Equal(1, Num(r, "count"));
        Assert.InRange(r.Pose!.X, 267, 273);
        Assert.InRange(r.Pose.Y, 147, 153);
        Assert.True(Num(r, "score") >= 0.8);
    }

    [Fact]
    public void ContourMatch_without_a_template_fails()
    {
        using var img = Canvas();
        var r = RunOne(img, Tool("m", "contourMatch"));
        Assert.Equal(ToolStatus.Fail, r.Status);
        Assert.Contains("taught", r.Message);
    }

    /// <summary>An asymmetric "F" with a little texture, on a 60 × 60 gray patch.</summary>
    private static Mat Pattern()
    {
        var p = new Mat(60, 60, MatType.CV_8UC3, Scalar.All(60));
        Cv2.Rectangle(p, new Rect(15, 8, 10, 44), Scalar.All(210), -1);
        Cv2.Rectangle(p, new Rect(15, 8, 32, 9), Scalar.All(210), -1);
        Cv2.Rectangle(p, new Rect(15, 26, 22, 8), Scalar.All(170), -1);
        Cv2.Circle(p, new Point(42, 42), 5, Scalar.All(130), -1);
        return p;
    }

    /// <summary>The pattern on a gray-60 canvas, centred on (cx, cy), turned by deg (clockwise on screen).</summary>
    private static Mat Place(double cx, double cy, double deg)
    {
        var img = Canvas(60);
        using var pat = Pattern();
        using var m = Cv2.GetRotationMatrix2D(new Point2f(30, 30), -deg, 1);
        m.Set(0, 2, m.At<double>(0, 2) + cx - 30);
        m.Set(1, 2, m.At<double>(1, 2) + cy - 30);
        Cv2.WarpAffine(pat, img, m, img.Size(), InterpolationFlags.Linear, BorderTypes.Transparent);
        return img;
    }

    [Fact]
    public void PatternMatch_finds_the_taught_patch_moved_and_turned()
    {
        using var teach = Place(120, 100, 0);
        var prog = Program(Tool("p", "patternMatch", Centered(120, 100, 60, 60), new { minScore = 0.6, angleRange = 30, angleStep = 5 }));
        using (var rt = new VisionRuntime()) Assert.Contains("pattern", VisionEngine.Teach(prog, "p", teach, rt));

        prog.Tools[0].Region = null;
        using var same = Place(250, 170, 0);
        var r0 = Run(prog, same).Tool("p")!;
        Assert.InRange(r0.Pose!.X, 249, 251);
        Assert.InRange(r0.Pose.Y, 169, 171);
        Assert.InRange(r0.Pose.Angle, -0.1, 0.1);
        Assert.True(Num(r0, "score") > 0.95);

        using var turned = Place(260, 180, 20);
        var r = Run(prog, turned).Tool("p")!;
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.InRange(r.Pose!.X, 258, 262);
        Assert.InRange(r.Pose.Y, 178, 182);
        Assert.InRange(r.Pose.Angle, 17, 23);

        // Without the angle search a turned patch scores lower; a blank image finds nothing.
        using var blank = Canvas(60);
        var none = Run(prog, blank).Tool("p")!;
        Assert.Equal(ToolStatus.Fail, none.Status);
        Assert.Equal(0, Num(none, "count"));
    }

    [Fact]
    public void Aruco_reads_markers_inside_the_region()
    {
        using var img = Canvas(255);
        using (var dict = CvAruco.GetPredefinedDictionary(PredefinedDictionaryType.Dict4X4_50))
        using (var marker = new Mat())
        {
            dict.GenerateImageMarker(7, 100, marker, 1);
            using var bgr = new Mat();
            Cv2.CvtColor(marker, bgr, ColorConversionCodes.GRAY2BGR);
            bgr.CopyTo(new Mat(img, new Rect(150, 100, 100, 100)));
        }
        int dictId = (int)PredefinedDictionaryType.Dict4X4_50;
        var r = RunOne(img, Tool("a", "aruco", null, new { dictionaryId = dictId }));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.Equal(1, Num(r, "count"));
        Assert.Equal(7, Convert.ToInt32(r.Items[0]["id"]));
        Assert.InRange(r.Pose!.X, 198, 202);
        Assert.InRange(r.Pose.Y, 148, 152);
        Assert.InRange(r.Pose.Angle, -2, 2);

        Assert.Equal(ToolStatus.Fail, RunOne(img, Tool("a", "aruco", null, new { dictionaryId = dictId, markerId = 8 })).Status);
        Assert.Equal(ToolStatus.Fail, RunOne(img, Tool("a", "aruco", Rect(0, 0, 100, 100), new { dictionaryId = dictId })).Status);
        Assert.Equal(1, Num(RunOne(img, Tool("a", "aruco", null, new { dictionaryId = -1 })), "count"));
    }

    // ── makeFixture ─────────────────────────────────────────────────────────────

    [Fact]
    public void MakeFixture_builds_a_pose_in_every_mode()
    {
        using var img = Canvas(40);
        Cv2.Rectangle(img, new Rect(100, 80, 200, 140), Scalar.All(200), -1);        // corners (100,80) (300,80) (300,220) (100,220)
        foreach (var (x, y) in new[] { (60, 40), (340, 40), (340, 260), (60, 260) })
            Cv2.Circle(img, new Point(x, y), 8, Scalar.All(200), -1);

        var tools = new List<VisionTool>
        {
            Tool("top",    "findLine", Centered(200, 80, 120, 40, 0),    new { polarity = "darkToLight" }),
            Tool("right",  "findLine", Centered(300, 150, 80, 40, 90),   new { polarity = "darkToLight" }),
            Tool("bottom", "findLine", Centered(200, 220, 120, 40, 180), new { polarity = "darkToLight" }),
            Tool("left",   "findLine", Centered(100, 150, 80, 40, -90),  new { polarity = "darkToLight" }),
            Tool("d1", "blob", Centered(60, 40, 40, 30),   new { polarity = "light" }),
            Tool("d2", "blob", Centered(340, 40, 40, 30),  new { polarity = "light" }),
            Tool("d3", "blob", Centered(340, 260, 40, 30), new { polarity = "light" }),
            Tool("d4", "blob", Centered(60, 260, 40, 30),  new { polarity = "light" }),
            Tool("two",   "makeFixture", null, new { mode = "twoPoints", sources = new[] { "d1", "d3" } }),
            Tool("pa",    "makeFixture", null, new { mode = "pointAngle", sources = new[] { "d1", "top" } }),
            Tool("li",    "makeFixture", null, new { mode = "lineIntersection", sources = new[] { "top", "left" } }),
            Tool("four",  "makeFixture", null, new { mode = "fourLines", sources = new[] { "top", "right", "bottom", "left" } }),
            Tool("pts",   "makeFixture", null, new { mode = "fourPoints", sources = new[] { "d1", "d2", "d3", "d4" } }),
            Tool("short", "makeFixture", null, new { mode = "fourLines", sources = new[] { "top", "right" } }),
        };
        var r = Run(Program(tools.ToArray()), img);
        Assert.All(r.Tools.Where(t => t.Id != "short"), t => Assert.True(t.Ok, $"{t.Id}: {t.Message}"));

        var two = r.Tool("two")!.Pose!;
        Assert.InRange(two.X, 59, 61); Assert.InRange(two.Angle, Math.Atan2(220, 280) * 180 / Math.PI - 0.5, Math.Atan2(220, 280) * 180 / Math.PI + 0.5);
        var pa = r.Tool("pa")!.Pose!;
        Assert.InRange(pa.X, 59, 61); Assert.InRange(pa.Angle, -0.5, 0.5);
        var li = r.Tool("li")!.Pose!;
        Assert.InRange(li.X, 99, 101); Assert.InRange(li.Y, 79, 81);

        var four = r.Tool("four")!.Pose!;
        Assert.Equal(4, four.Quad!.Count);
        var want = new[] { (100.0, 80.0), (300.0, 80.0), (300.0, 220.0), (100.0, 220.0) };
        for (int i = 0; i < 4; i++)
        {
            Assert.InRange(four.Quad[i].X, want[i].Item1 - 1.5, want[i].Item1 + 1.5);
            Assert.InRange(four.Quad[i].Y, want[i].Item2 - 1.5, want[i].Item2 + 1.5);
        }
        Assert.InRange(four.X, 199, 201); Assert.InRange(four.Y, 149, 151);

        var pts = r.Tool("pts")!.Pose!;
        Assert.InRange(pts.Quad![2].X, 339, 341); Assert.InRange(pts.Quad[2].Y, 259, 261);

        Assert.Equal(ToolStatus.Fail, r.Tool("short")!.Status);
    }

    // ── caliper / distance / angle ──────────────────────────────────────────────

    [Fact]
    public void Caliper_finds_single_edges_pairs_and_all()
    {
        using var img = Canvas(40);
        Cv2.Rectangle(img, new Rect(180, 0, 30, 300), Scalar.All(200), -1);   // light bar x 180..209
        var region = Centered(200, 150, 200, 40);

        var pair = RunOne(img, Tool("c", "caliper", region, new { mode = "pair", polarity = "darkToLight" }));
        Assert.Equal(ToolStatus.Ok, pair.Status);
        Assert.InRange(Num(pair, "width"), 29, 31);
        Assert.InRange(Num(pair, "x"), 193.5, 195.5);
        Assert.InRange(Num(pair, "y"), 149, 151);
        Assert.Equal(2, pair.Points.Count);

        var first = RunOne(img, Tool("c", "caliper", region, new { mode = "single", polarity = "any", pick = "first" }));
        Assert.InRange(Num(first, "x"), 178.5, 180.5);
        var last = RunOne(img, Tool("c", "caliper", region, new { mode = "single", polarity = "any", pick = "last" }));
        Assert.InRange(Num(last, "x"), 208.5, 210.5);
        var all = RunOne(img, Tool("c", "caliper", region, new { mode = "all" }));
        Assert.Equal(2, Num(all, "count"));
        Assert.Equal(2, all.Points.Count);

        var wrongWidth = RunOne(img, Tool("c", "caliper", region, new { mode = "pair", pairWidth = 50, pairTolerance = 5 }));
        Assert.Equal(ToolStatus.Fail, wrongWidth.Status);
    }

    [Fact]
    public void Distance_and_angle_measure_between_tools()
    {
        using var img = Canvas(40);
        Cv2.Circle(img, new Point(100, 100), 8, Scalar.All(200), -1);
        Cv2.Circle(img, new Point(250, 180), 8, Scalar.All(200), -1);
        Poly(img, Scalar.All(200), (0, 250), (400, 250), (400, 300), (0, 300));            // horizontal edge y = 250
        double t = Math.Tan(30 * Math.PI / 180);
        var tools = new[]
        {
            Tool("a", "blob", Centered(100, 100, 40, 40), new { polarity = "light" }),
            Tool("b", "blob", Centered(250, 180, 40, 40), new { polarity = "light" }),
            Tool("floor", "findLine", Centered(200, 250, 300, 30), new { polarity = "darkToLight" }),
            Tool("pp", "distance", null, new { mode = "pointPoint", sources = new[] { "a", "b" } }),
            Tool("pl", "distance", null, new { mode = "pointLine", sources = new[] { "a", "floor" } }),
            Tool("ang1", "angle", null, new { sources = new[] { "floor" } }),
        };
        var r = Run(Program(tools), img);
        Assert.InRange(Num(r.Tool("pp")!, "distance"), 169, 171);       // √(150² + 80²) = 170
        Assert.InRange(Num(r.Tool("pp")!, "dx"), 149, 151);
        Assert.InRange(Num(r.Tool("pl")!, "distance"), 149, 151);
        Assert.InRange(Num(r.Tool("ang1")!, "angle"), -0.5, 0.5);

        // Angle between two lines: a second edge at 30°.
        using var img2 = Canvas(40);
        Poly(img2, Scalar.All(200), (0, 250), (400, 250), (400, 300), (0, 300));
        Poly(img2, Scalar.All(200), (100, 40), (300, 40 + 200 * t), (300, 0), (100, 0));
        var r2 = Run(Program(
            Tool("floor", "findLine", Centered(200, 250, 300, 30), new { polarity = "darkToLight" }),
            Tool("slope", "findLine", Centered(200, 40 + 100 * t, 120, 40, 30), new { polarity = "any" }),
            Tool("between", "angle", null, new { sources = new[] { "floor", "slope" } })), img2);
        Assert.InRange(Math.Abs(Num(r2.Tool("between")!, "angle")), 29, 31);
    }

    // ── corners / lines ─────────────────────────────────────────────────────────

    [Fact]
    public void Corners_finds_the_corners_of_a_rectangle()
    {
        using var img = Canvas(40);
        Cv2.Rectangle(img, new Rect(100, 80, 200, 140), Scalar.All(200), -1);
        var r = RunOne(img, Tool("k", "corners", null, new { maxCorners = 10, quality = 0.1, minDistance = 20 }));
        Assert.Equal(4, Num(r, "count"));
        foreach (var (x, y) in new[] { (100, 80), (299, 80), (299, 219), (100, 219) })
            Assert.Contains(r.Points, p => Math.Abs(p.X - x) < 3 && Math.Abs(p.Y - y) < 3);
        var harris = RunOne(img, Tool("k", "corners", Rect(50, 40, 120, 100), new { method = "harris", quality = 0.1, minDistance = 20 }));
        Assert.Equal(1, Num(harris, "count"));   // only the top-left corner is in the region
    }

    [Fact]
    public void Lines_detects_segments_and_filters_by_angle()
    {
        using var img = Canvas(0);
        Cv2.Line(img, new Point(40, 60), new Point(360, 60), Scalar.All(255), 2);
        Cv2.Line(img, new Point(40, 240), new Point(360, 240), Scalar.All(255), 2);
        Cv2.Line(img, new Point(200, 90), new Point(200, 210), Scalar.All(255), 2);
        var all = RunOne(img, Tool("l", "lines", null, new { houghThreshold = 40, minLineLength = 80, maxLineGap = 5 }));
        Assert.True(Num(all, "count") >= 3);
        var vertical = RunOne(img, Tool("l", "lines", null, new { houghThreshold = 40, minLineLength = 80, maxLineGap = 5, minAngle = 80, maxAngle = 100 }));
        Assert.True(Num(vertical, "count") >= 1);
        Assert.All(vertical.Lines, l => Assert.InRange(Math.Abs(l.X2 - l.X1), 0, 3));
    }

    // ── pixelCount / intensity / edgePixels / polygon ───────────────────────────

    [Fact]
    public void PixelCount_measures_colour_and_gray_with_a_judged_grid()
    {
        using var img = Canvas();
        Cv2.Rectangle(img, new Rect(0, 0, 200, 300), new Scalar(0, 0, 255), -1);     // red left
        Cv2.Rectangle(img, new Rect(200, 0, 200, 300), new Scalar(0, 255, 0), -1);   // green right
        var red = new[] { new { r = 255, g = 0, b = 0, tolerance = 10 } };
        var limits = new Dictionary<string, VisionLimit> { ["percent"] = new() { Min = 50 } };

        var whole = RunOne(img, Tool("p", "pixelCount", null, new { mode = "color", colors = red }));
        Assert.InRange(Num(whole, "percent"), 49.5, 50.5);
        Assert.Empty(whole.Cells);
        Assert.True(whole.Passed);   // no limits

        var grid = RunOne(img, Tool("p", "pixelCount", null, new { mode = "color", colors = red }, grid: new VisionGrid { Rows = 1, Cols = 2 }, limits: limits));
        Assert.Equal(2, grid.Cells.Count);
        Assert.InRange(grid.Cells[0].Value, 99, 100); Assert.True(grid.Cells[0].Passed);
        Assert.InRange(grid.Cells[1].Value, 0, 1);    Assert.False(grid.Cells[1].Passed);
        Assert.Equal(1, Num(grid, "cellsPassed"));
        Assert.False(grid.Passed);   // a gridded main-value limit means every cell

        var gray = RunOne(img, Tool("p", "pixelCount", null, new { mode = "gray", grayMin = 140, grayMax = 160 }));
        Assert.InRange(Num(gray, "percent"), 49.5, 50.5);   // green's gray is ~150, red's ~76

        // A circular region clips its cells.
        using var white = Canvas(255);
        var circ = RunOne(white, Tool("p", "pixelCount", Circle(200, 150, 100), new { mode = "gray", grayMin = 200, grayMax = 255 }, grid: new VisionGrid { Rows = 2, Cols = 2 }));
        Assert.InRange(Num(circ, "percent"), 99.9, 100);
        Assert.All(circ.Cells, c => Assert.InRange(c.Value, 99.9, 100));
    }

    [Fact]
    public void Intensity_reports_statistics_and_judges_cells_on_the_chosen_one()
    {
        using var img = Canvas(50);
        Cv2.Rectangle(img, new Rect(200, 0, 200, 300), Scalar.All(150), -1);
        var r = RunOne(img, Tool("i", "intensity", null, new { statistic = "mean" }, grid: new VisionGrid { Rows = 1, Cols = 2 },
            limits: new() { ["mean"] = new() { Max = 100 } }));
        Assert.InRange(Num(r, "mean"), 99.5, 100.5);
        Assert.InRange(Num(r, "stdDev"), 49.5, 50.5);
        Assert.Equal(50, Num(r, "min")); Assert.Equal(150, Num(r, "max")); Assert.Equal(100, Num(r, "contrast"));
        Assert.Equal(50, r.Cells[0].Value); Assert.True(r.Cells[0].Passed);
        Assert.Equal(150, r.Cells[1].Value); Assert.False(r.Cells[1].Passed);

        var std = RunOne(img, Tool("i", "intensity", null, new { statistic = "stdDev" }, grid: new VisionGrid { Rows = 1, Cols = 2, Inset = 0.2 }));
        Assert.All(std.Cells, c => Assert.Equal(0, c.Value));
        Assert.Equal(4, std.Cells[0].Quad!.Count);
    }

    [Fact]
    public void EdgePixels_measures_density_and_direction()
    {
        using var img = Canvas(40);
        for (int x = 210; x < 400; x += 12) Cv2.Rectangle(img, new Rect(x, 0, 6, 300), Scalar.All(200), -1);   // vertical stripes on the right
        var r = RunOne(img, Tool("e", "edgePixels", null, new { cannyLow = 50, cannyHigh = 150 }, grid: new VisionGrid { Rows = 1, Cols = 2 }));
        Assert.Equal(0, r.Cells[0].Value);
        Assert.True(r.Cells[1].Value > 10);
        Assert.InRange(Num(r, "angle"), 85, 95);
    }

    [Fact]
    public void Polygon_counts_shapes_by_their_sides()
    {
        using var img = Canvas(0);
        Poly(img, Scalar.All(255), (60, 60), (140, 60), (140, 140), (60, 140));
        Poly(img, Scalar.All(255), (260, 160), (340, 160), (340, 240), (260, 240));
        Poly(img, Scalar.All(255), (300, 30), (360, 120), (240, 120));
        var four = RunOne(img, Tool("p", "polygon", null, new { sides = 4, minArea = 1000 }, grid: new VisionGrid { Rows = 1, Cols = 2 }));
        Assert.Equal(2, Num(four, "count"));
        Assert.Equal(new[] { 1.0, 1.0 }, four.Cells.Select(c => c.Value));
        Assert.Equal(2, four.Items.Count);
        var three = RunOne(img, Tool("p", "polygon", null, new { sides = 3, minArea = 1000 }));
        Assert.Equal(1, Num(three, "count"));
        Assert.InRange((double)three.Items[0]["x"], 295, 305);
    }

    // ── barcode ─────────────────────────────────────────────────────────────────

    [Fact]
    public void Barcode_reads_a_qr_code()
    {
        var writer = new ZXing.BarcodeWriterPixelData
        {
            Format = ZXing.BarcodeFormat.QR_CODE,
            Options = new ZXing.Common.EncodingOptions { Width = 160, Height = 160, Margin = 2 },
        };
        var px = writer.Write("HELLO-42");
        using var bgra = new Mat(px.Height, px.Width, MatType.CV_8UC4);
        System.Runtime.InteropServices.Marshal.Copy(px.Pixels, 0, bgra.Data, px.Pixels.Length);
        using var img = Canvas(255);
        using (var bgr = new Mat())
        {
            Cv2.CvtColor(bgra, bgr, ColorConversionCodes.BGRA2BGR);
            bgr.CopyTo(new Mat(img, new Rect(120, 70, px.Width, px.Height)));
        }
        var r = RunOne(img, Tool("q", "barcode", null, new { formats = new[] { "QR_CODE" } }));
        Assert.Equal(ToolStatus.Ok, r.Status);
        Assert.Equal(1, Num(r, "count"));
        Assert.Equal("HELLO-42", r.Values["value"]);
        Assert.Equal("HELLO-42", r.Items[0]["value"]);
        Assert.InRange((double)r.Items[0]["x"], 180, 220);

        using var blank = Canvas(255);
        Assert.Equal(ToolStatus.Fail, RunOne(blank, Tool("q", "barcode")).Status);
    }

    [Fact]
    public void Every_tool_type_in_the_spec_is_registered()
    {
        var spec = new[] { "findLine", "findCorner", "findCircle", "caliper", "makeFixture", "distance", "angle", "contourMatch",
                           "patternMatch", "blob", "aruco", "corners", "lines", "pixelCount", "intensity", "edgePixels", "polygon", "barcode" };
        Assert.Equal(spec.OrderBy(x => x), ToolRegistry.Types.OrderBy(x => x));
        using var img = Canvas();
        var bad = RunOne(img, Tool("x", "sparkle"));
        Assert.Equal(ToolStatus.Error, bad.Status);
    }
}
