using System.Text.Json;
using Controller.RobotControl.Vision;
using OpenCvSharp;
using static RobotControl.Tests.VisionTestKit;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

/// <summary>
/// A fixed skewed (four-corner) grid with no fixture, and <c>regionFrom</c>: a tool that uses an
/// earlier tool's region (and fixture), so one area drawn once serves several tools.
/// </summary>
public class VisionSharedRegionTests
{
    /// <summary>A light 4 × 4 board (dark dots in some cells) warped onto a perspective quad.</summary>
    private static Mat Board((double X, double Y)[] quad, HashSet<(int, int)> dots)
    {
        using var flat = new Mat(200, 200, MatType.CV_8UC3, Scalar.All(220));
        foreach (var (row, col) in dots) Cv2.Circle(flat, new Point(col * 50 + 25, row * 50 + 25), 14, Scalar.All(30), -1);
        var img = Canvas(40);
        using var m = Cv2.GetPerspectiveTransform(
            new[] { new Point2f(0, 0), new Point2f(200, 0), new Point2f(200, 200), new Point2f(0, 200) },
            quad.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray());
        Cv2.WarpPerspective(flat, img, m, img.Size(), InterpolationFlags.Linear, BorderTypes.Transparent);
        return img;
    }

    private static readonly HashSet<(int, int)> Dots = new() { (0, 0), (0, 3), (1, 2), (2, 1), (3, 3) };
    private static readonly (double X, double Y)[] Seen = { (130, 70), (280, 55), (320, 250), (90, 235) };

    private static IEnumerable<(int, int)> PassedCells(ToolResult r) =>
        r.Cells.Where(c => c.Passed).Select(c => (c.Row, c.Col)).OrderBy(d => d);

    [Fact]
    public void A_fixed_skewed_grid_reads_a_perspective_board_without_a_fixture()
    {
        var prog = Program(
            Tool("dark", "pixelCount", Quad(Seen), new { mode = "gray", grayMin = 0, grayMax = 100 },
                 grid: new VisionGrid { Rows = 4, Cols = 4, Inset = 0.2 }, limits: new() { ["percent"] = new() { Min = 20 } }));
        using var img = Board(Seen, Dots);
        var r = Run(prog, img).Tool("dark")!;
        Assert.True(r.Ok, r.Message);
        Assert.Equal(16, r.Cells.Count);
        Assert.Equal(Dots.OrderBy(d => d), PassedCells(r));
        for (int i = 0; i < 4; i++)
        {
            Assert.InRange(r.Region[i].X, Seen[i].X - 1, Seen[i].X + 1);
            Assert.InRange(r.Region[i].Y, Seen[i].Y - 1, Seen[i].Y + 1);
        }
    }

    [Fact]
    public void A_tool_with_regionFrom_uses_the_earlier_tools_area_and_keeps_its_own_grid()
    {
        var area = Tool("area", "pixelCount", Quad(Seen), new { mode = "gray", grayMin = 0, grayMax = 100 },
                        grid: new VisionGrid { Rows = 4, Cols = 4, Inset = 0.2 }, limits: new() { ["percent"] = new() { Min = 20 } });
        // Its own region is somewhere else entirely: it must be ignored.
        var shared = Tool("shared", "intensity", Centered(30, 30, 20, 20), new { statistic = "mean" },
                          grid: new VisionGrid { Rows = 4, Cols = 4, Inset = 0.2 }, limits: new() { ["mean"] = new() { Max = 120 } });
        shared.RegionFrom = "area";
        var whole = Tool("whole", "intensity", Centered(30, 30, 20, 20), new { statistic = "mean" });
        whole.RegionFrom = "area";
        var prog = Program(area, shared, whole);

        using var img = Board(Seen, Dots);
        var r = Run(prog, img);
        var s = r.Tool("shared")!;
        Assert.True(s.Ok, s.Message);
        Assert.Equal(Dots.OrderBy(d => d), PassedCells(s));
        Assert.Equal(r.Tool("area")!.Region.Select(p => (p.X, p.Y)), s.Region.Select(p => (p.X, p.Y)));
        // No grid of its own → the whole shared area as one measurement.
        var w = r.Tool("whole")!;
        Assert.Empty(w.Cells);
        Assert.Equal(r.Tool("area")!.Region.Select(p => (p.X, p.Y)), w.Region.Select(p => (p.X, p.Y)));
    }

    [Fact]
    public void A_shared_region_also_follows_its_owners_fixture()
    {
        var teachQuad = new[] { (110.0, 60.0), (290.0, 60.0), (290.0, 240.0), (110.0, 240.0) };
        var shared = Tool("shared", "pixelCount", null, new { mode = "gray", grayMin = 0, grayMax = 100 },
                          grid: new VisionGrid { Rows = 4, Cols = 4, Inset = 0.2 }, limits: new() { ["percent"] = new() { Min = 20 } });
        shared.RegionFrom = "cells";
        var prog = Program(
            Tool("top",    "findLine", Centered(200, 60, 120, 60, 0),    new { polarity = "darkToLight", pick = "first" }),
            Tool("right",  "findLine", Centered(290, 150, 120, 60, 90),  new { polarity = "darkToLight", pick = "first" }),
            Tool("bottom", "findLine", Centered(200, 240, 120, 60, 180), new { polarity = "darkToLight", pick = "first" }),
            Tool("left",   "findLine", Centered(110, 150, 120, 60, -90), new { polarity = "darkToLight", pick = "first" }),
            Tool("board",  "makeFixture", null, new { mode = "fourLines", sources = new[] { "top", "right", "bottom", "left" } }),
            // Disabled: its area is still borrowed.
            Tool("cells",  "intensity", Quad(teachQuad), null, fixture: "board"),
            shared);
        prog.Tool("cells")!.Enabled = false;

        using (var teach = Board(teachQuad, Dots)) Run(prog, teach);
        using var img = Board(Seen, Dots);
        var r = Run(prog, img);
        var s = r.Tool("shared")!;
        Assert.True(s.Ok, s.Message);
        Assert.Equal(Dots.OrderBy(d => d), PassedCells(s));
        for (int i = 0; i < 4; i++)
        {
            Assert.InRange(s.Region[i].X, Seen[i].X - 2, Seen[i].X + 2);
            Assert.InRange(s.Region[i].Y, Seen[i].Y - 2, Seen[i].Y + 2);
        }
    }

    [Fact]
    public void A_region_source_that_is_not_earlier_is_an_error()
    {
        var a = Tool("a", "intensity", Centered(100, 100, 40, 40), null);
        a.RegionFrom = "b";
        var self = Tool("self", "intensity", Centered(100, 100, 40, 40), null);
        self.RegionFrom = "self";
        var prog = Program(a, Tool("b", "intensity", Centered(100, 100, 40, 40), null), self);
        using var img = Canvas(40);
        var r = Run(prog, img);
        Assert.Equal(ToolStatus.Error, r.Tool("a")!.Status);
        Assert.Contains("not an earlier tool", r.Tool("a")!.Message);
        Assert.Equal(ToolStatus.Error, r.Tool("self")!.Status);
        Assert.True(r.Tool("b")!.Ok);
    }

    [Fact]
    public void RegionFrom_round_trips_in_program_json()
    {
        var t = Tool("x", "intensity", null, null);
        t.RegionFrom = "area";
        var json = JsonSerializer.Serialize(Program(t));
        Assert.Contains("\"regionFrom\":\"area\"", json);
        var back = JsonSerializer.Deserialize<VisionProgram>(json)!;
        Assert.Equal("area", back.Tools[0].RegionFrom);
        var none = JsonSerializer.Deserialize<VisionProgram>("""{"id":"p","tools":[{"id":"y","type":"intensity"}]}""")!;
        Assert.Null(none.Tools[0].RegionFrom);
    }
}
