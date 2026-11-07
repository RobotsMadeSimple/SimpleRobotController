using System.Text.Json;
using Controller.RobotControl.Vision;
using OpenCvSharp;

namespace RobotControl.Tests;

/// <summary>Builders for vision programs and synthetic images used by the vision tests.</summary>
internal static class VisionTestKit
{
    public const int W = 400, H = 300;

    public static Mat Canvas(int gray = 40, int w = W, int h = H) => new(h, w, MatType.CV_8UC3, Scalar.All(gray));

    /// <summary>A rect region given in pixels of a w × h image (rotation in degrees about its centre).</summary>
    public static VisionRegion Rect(double x, double y, double w, double h, double rotation = 0, int iw = W, int ih = H) =>
        new() { Shape = "rect", X = x / iw, Y = y / ih, Width = w / iw, Height = h / ih, Rotation = rotation };

    /// <summary>A rect region of size w × h centred on (cx, cy), pixels.</summary>
    public static VisionRegion Centered(double cx, double cy, double w, double h, double rotation = 0, int iw = W, int ih = H) =>
        Rect(cx - w / 2, cy - h / 2, w, h, rotation, iw, ih);

    public static VisionRegion Circle(double cx, double cy, double r, int iw = W, int ih = H) =>
        new() { Shape = "circle", Cx = cx / iw, Cy = cy / ih, Radius = r / Math.Min(iw, ih) };

    public static VisionRegion Quad(params (double X, double Y)[] pts) =>
        new() { Shape = "quad", Points = pts.Select(p => new VisionPoint(p.X / W, p.Y / H)).ToList() };

    public static Dictionary<string, JsonElement> Params(object? p)
    {
        var d = new Dictionary<string, JsonElement>();
        if (p == null) return d;
        foreach (var prop in JsonSerializer.SerializeToElement(p).EnumerateObject()) d[prop.Name] = prop.Value.Clone();
        return d;
    }

    public static VisionTool Tool(string id, string type, VisionRegion? region = null, object? p = null,
                                  string input = "camera", string? fixture = null, VisionGrid? grid = null,
                                  Dictionary<string, VisionLimit>? limits = null, bool includeInPass = true) =>
        new()
        {
            Id = id, Name = id, Type = type, Region = region, Params = Params(p), Input = input, FixtureId = fixture,
            Grid = grid, Limits = limits, IncludeInPass = includeInPass,
        };

    public static VisionProgram Program(params VisionTool[] tools) => new() { Id = "test", Name = "test", Tools = tools.ToList() };

    public static VisionFilter Filter(string type, object? p = null) => new() { Id = type, Type = type, Params = Params(p) };

    public static VisionResult Run(VisionProgram prog, Mat img) => VisionProcessor.RunProgramOnce(prog, img, out _);

    public static ToolResult RunOne(Mat img, VisionTool tool) => Run(Program(tool), img).Tool(tool.Id)!;

    public static double Num(ToolResult r, string key) => r.TryGetNumber(key, out var v) ? v : double.NaN;

    /// <summary>Fills a polygon (pixel points) on an image.</summary>
    public static void Poly(Mat img, Scalar color, params (double X, double Y)[] pts) =>
        Cv2.FillPoly(img, new[] { pts.Select(p => new OpenCvSharp.Point((int)Math.Round(p.X), (int)Math.Round(p.Y))).ToArray() }, color, LineTypes.AntiAlias);

    /// <summary>The corners of a w × h rectangle centred on (cx, cy) turned by deg (clockwise on screen).</summary>
    public static (double X, double Y)[] Turned(double cx, double cy, double w, double h, double deg)
    {
        double a = deg * Math.PI / 180, c = Math.Cos(a), s = Math.Sin(a);
        (double, double) P(double lx, double ly) => (cx + lx * c - ly * s, cy + lx * s + ly * c);
        return new[] { P(-w / 2, -h / 2), P(w / 2, -h / 2), P(w / 2, h / 2), P(-w / 2, h / 2) };
    }

    /// <summary>A point turned by deg about (cx, cy).</summary>
    public static (double X, double Y) TurnPoint(double x, double y, double cx, double cy, double deg)
    {
        double a = deg * Math.PI / 180, c = Math.Cos(a), s = Math.Sin(a);
        return (cx + (x - cx) * c - (y - cy) * s, cy + (x - cx) * s + (y - cy) * c);
    }

    // ── Moved scenes and the two-dot fixture ────────────────────────────────────

    /// <summary>How far the "moved" scene is shifted and turned (clockwise on screen) about the image centre.</summary>
    public const double MoveDx = 15, MoveDy = 10, MoveDeg = 10;

    /// <summary>Where a teach-image point lands in the moved scene.</summary>
    public static (double X, double Y) Moved(double x, double y)
    {
        var (tx, ty) = TurnPoint(x, y, W / 2.0, H / 2.0, MoveDeg);
        return (tx + MoveDx, ty + MoveDy);
    }

    /// <summary>The teach image turned by <see cref="MoveDeg"/> about its centre and shifted by (MoveDx, MoveDy); uncovered pixels get <paramref name="bg"/>.</summary>
    public static Mat MoveScene(Mat teach, double bg)
    {
        var img = new Mat(teach.Size(), teach.Type(), Scalar.All(bg));
        using var m = Cv2.GetRotationMatrix2D(new Point2f(W / 2f, H / 2f), -MoveDeg, 1);
        m.Set(0, 2, m.At<double>(0, 2) + MoveDx);
        m.Set(1, 2, m.At<double>(1, 2) + MoveDy);
        Cv2.WarpAffine(teach, img, m, img.Size(), InterpolationFlags.Linear, BorderTypes.Constant, Scalar.All(bg));
        return img;
    }

    /// <summary>Pure red (BGR) — its gray level is <see cref="RedGray"/>, so on a gray-76 canvas it is invisible to gray tools.</summary>
    public static readonly Scalar Red = new(0, 0, 255);
    public const int RedGray = 76;

    /// <summary>A layer that keeps only the red pixels (255) and drops everything else (0).</summary>
    public static ImageLayer RedLayer(string id = "red") =>
        new() { Id = id, Input = "camera", Filters = { Filter("colorMask", new { hMin = 170, hMax = 10, sMin = 120, sMax = 255, vMin = 120, vMax = 255 }) } };

    /// <summary>The two red fixture dots of a teach image (keep tool regions clear of the top corners).</summary>
    public static readonly (double X, double Y) DotA = (50, 30), DotB = (350, 30);

    public static void Dots(Mat img)
    {
        Cv2.Circle(img, new OpenCvSharp.Point((int)DotA.X, (int)DotA.Y), 7, Red, -1);
        Cv2.Circle(img, new OpenCvSharp.Point((int)DotB.X, (int)DotB.Y), 7, Red, -1);
    }

    /// <summary>
    /// A twoPoints fixture "fx" from the two red dots (blobs "dotA"/"dotB" on the red layer),
    /// followed by <paramref name="tools"/>; the red layer is added to the program.
    /// </summary>
    public static VisionProgram FixturedProgram(params VisionTool[] tools)
    {
        var all = new List<VisionTool>
        {
            Tool("dotA", "blob", Rect(0, 0, 200, 120), new { polarity = "light" }, input: "red"),
            Tool("dotB", "blob", Rect(200, 0, 200, 120), new { polarity = "light" }, input: "red"),
            Tool("fx", "makeFixture", null, new { mode = "twoPoints", sources = new[] { "dotA", "dotB" } }),
        };
        all.AddRange(tools);
        var prog = Program(all.ToArray());
        prog.Layers.Add(RedLayer());
        return prog;
    }

    /// <summary>
    /// Runs a fixtured program on the teach image (capturing the fixture's reference) and then
    /// on the moved scene; returns both results.
    /// </summary>
    public static (VisionResult Teach, VisionResult Moved) TeachThenMoved(VisionProgram prog, Mat teach, double bg)
    {
        var r0 = Run(prog, teach);
        Assert.True(r0.Tool("fx")!.Ok, r0.Tool("fx")!.Message);
        Assert.NotNull(prog.Tool("fx")!.Reference);
        using var moved = MoveScene(teach, bg);
        var r1 = Run(prog, moved);
        Assert.True(r1.Tool("fx")!.Ok, r1.Tool("fx")!.Message);
        Assert.InRange(r1.Tool("fx")!.Pose!.Angle, MoveDeg - 0.3, MoveDeg + 0.3);
        return (r0, r1);
    }

    /// <summary>
    /// With a blank image the fixture is not found, so the tool is notRun: not passed, no values,
    /// no pose, the overall result failed.
    /// </summary>
    public static void AssertNotRunWithoutFixture(VisionTool tool)
    {
        tool.FixtureId = "fx";
        var prog = FixturedProgram(tool);
        using var blank = Canvas(RedGray);
        var r = Run(prog, blank);
        Assert.Equal(ToolStatus.Fail, r.Tool("fx")!.Status);
        var t = r.Tool(tool.Id)!;
        Assert.Equal(ToolStatus.NotRun, t.Status);
        Assert.Contains("fixture 'fx'", t.Message);
        Assert.False(t.Passed);
        Assert.Empty(t.Values);
        Assert.Null(t.Pose);
        Assert.False(r.Passed);
    }

    // ── Limits ──────────────────────────────────────────────────────────────────

    /// <summary>
    /// Limits on <paramref name="key"/> at its measured value v: [v, v] passes (bounds are
    /// inclusive), min v + 0.01 or max v − 0.01 fails; and the tool's pass counts for the program
    /// only with includeInPass. <paramref name="build"/> makes a fresh program with the given limits
    /// and includeInPass on the tool under test (the program's other tools must pass).
    /// </summary>
    public static double AssertLimitBoundaries(Mat img, string toolId, string key,
                                               Func<Dictionary<string, VisionLimit>?, bool, VisionProgram> build)
    {
        var free = Run(build(null, true), img);
        var t0 = free.Tool(toolId)!;
        Assert.True(t0.Ok, t0.Message);
        Assert.True(t0.Passed);
        Assert.True(free.Passed);
        double v = Num(t0, key);
        Assert.False(double.IsNaN(v), $"{toolId} has no value '{key}'");

        void Check(double? min, double? max, bool want)
        {
            var lim = new Dictionary<string, VisionLimit> { [key] = new() { Min = min, Max = max } };
            var inc = Run(build(lim, true), img);
            Assert.True(want == inc.Tool(toolId)!.Passed, $"{key} = {v}, limit [{min}, {max}]: expected passed {want}");
            Assert.Equal(want, inc.Passed);
            var exc = Run(build(lim, false), img);
            Assert.Equal(want, exc.Tool(toolId)!.Passed);
            Assert.True(exc.Passed, "a tool left out of the pass does not fail the program");
        }
        Check(v, v, true);
        Check(v - 1, null, true);
        Check(null, v + 1, true);
        Check(v + 0.01, null, false);
        Check(null, v - 0.01, false);
        // A limit on a value the tool does not produce fails.
        var missing = Run(build(new() { ["noSuchValue"] = new() { Min = 0 } }, true), img);
        Assert.False(missing.Tool(toolId)!.Passed);
        return v;
    }

    /// <summary>A one-tool program builder for <see cref="AssertLimitBoundaries"/>.</summary>
    public static Func<Dictionary<string, VisionLimit>?, bool, VisionProgram> Single(Func<VisionTool> make) => (lim, inc) =>
    {
        var t = make();
        t.Limits = lim;
        t.IncludeInPass = inc;
        return Program(t);
    };

    // ── Drawing ─────────────────────────────────────────────────────────────────

    /// <summary>Fills an axis-aligned pixel rectangle (x, y, w, h) — no anti-aliasing.</summary>
    public static void Box(Mat img, int x, int y, int w, int h, Scalar color) => Cv2.Rectangle(img, new Rect(x, y, w, h), color, -1);

    /// <summary>Draws an image (BGR or gray) onto a canvas with its top-left at (x, y).</summary>
    public static void Paste(Mat canvas, Mat img, int x, int y)
    {
        using var bgr = new Mat();
        if (img.Channels() == 1) Cv2.CvtColor(img, bgr, ColorConversionCodes.GRAY2BGR); else img.CopyTo(bgr);
        using var roi = new Mat(canvas, new Rect(x, y, bgr.Width, bgr.Height));
        bgr.CopyTo(roi);
    }

    /// <summary>An image turned by deg (clockwise on screen) about its centre on a canvas of the same size, uncovered pixels bg.</summary>
    public static Mat Rotated(Mat img, double deg, double bg)
    {
        var dst = new Mat(img.Size(), img.Type(), Scalar.All(bg));
        using var m = Cv2.GetRotationMatrix2D(new Point2f(img.Width / 2f, img.Height / 2f), -deg, 1);
        Cv2.WarpAffine(img, dst, m, img.Size(), InterpolationFlags.Linear, BorderTypes.Constant, Scalar.All(bg));
        return dst;
    }

    /// <summary>Distance from (x, y) to the infinite line through a PxLine.</summary>
    public static double DistToLine(PxLine l, double x, double y)
    {
        double dx = l.X2 - l.X1, dy = l.Y2 - l.Y1;
        return Math.Abs((x - l.X1) * dy - (y - l.Y1) * dx) / Math.Sqrt(dx * dx + dy * dy);
    }

    /// <summary>a − b wrapped into (−180, 180].</summary>
    public static double AngleDiff(double a, double b)
    {
        double d = (a - b) % 360;
        if (d > 180) d -= 360;
        if (d <= -180) d += 360;
        return d;
    }

    /// <summary>Asserts a result point is within tol px of (x, y).</summary>
    public static void Near(double ax, double ay, (double X, double Y) want, double tol, string what = "point")
    {
        Assert.True(Math.Abs(ax - want.X) <= tol && Math.Abs(ay - want.Y) <= tol,
            $"{what} ({ax:F2}, {ay:F2}) is not within {tol} px of ({want.X:F2}, {want.Y:F2})");
    }

    public static double D(object o) => Convert.ToDouble(o);
}
