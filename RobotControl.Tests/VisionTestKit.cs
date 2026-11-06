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
}
