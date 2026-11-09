using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Vision;
using Controller.RobotControl.Vision.Calibration;
using OpenCvSharp;
using static RobotControl.Tests.VisionTestKit;
using Point = OpenCvSharp.Point;

namespace RobotControl.Tests;

/// <summary>
/// The tool runner: fixtures (similarity and perspective), notRun propagation, limits and
/// pass, reference capture, the program JSON shape, and RunVision's variable outputs.
/// </summary>
public class VisionEngineTests
{
    // ── Fixtures ────────────────────────────────────────────────────────────────

    /// <summary>A light 160 × 100 part centred on (cx, cy), turned deg, with a dark hole at part-local (+50, +20).</summary>
    private static Mat Part(double cx, double cy, double deg, out (double X, double Y) hole)
    {
        var img = Canvas(40);
        Poly(img, Scalar.All(200), Turned(cx, cy, 160, 100, deg));
        hole = TurnPoint(cx + 50, cy + 20, cx, cy, deg);
        Cv2.Circle(img, new Point((int)Math.Round(hole.X), (int)Math.Round(hole.Y)), 9, Scalar.All(20), -1, LineTypes.AntiAlias);
        return img;
    }

    private static VisionProgram PartProgram() => Program(
        // Fixture: the part's top-left corner (side A = top edge, side B = left edge).
        Tool("corner", "findCorner", Centered(200, 90, 120, 60), new { polarity = "any", probes = 10, regionB = Centered(100, 140, 80, 100, 90) }),
        // Taught on the part at (180, 140), 0°: its hole is at (230, 160).
        Tool("hole", "pixelCount", Centered(230, 160, 14, 14), new { mode = "gray", grayMin = 0, grayMax = 90 }, fixture: "corner",
             limits: new() { ["percent"] = new() { Min = 60 } }),
        Tool("holeFixed", "pixelCount", Centered(230, 160, 14, 14), new { mode = "gray", grayMin = 0, grayMax = 90 },
             limits: new() { ["percent"] = new() { Min = 60 } }, includeInPass: false));

    [Fact]
    public void Regions_follow_a_shifted_and_turned_part()
    {
        var prog = PartProgram();
        using (var teach = Part(180, 140, 0, out _))
        {
            var r0 = Run(prog, teach);
            Assert.True(r0.Passed);
            Assert.InRange(prog.Tool("corner")!.Reference!.X, 99, 101);     // captured on the first good run
            Assert.InRange(prog.Tool("corner")!.Reference!.Y, 89, 91);
        }

        using var moved = Part(215, 155, 10, out var hole);
        var r = Run(prog, moved);
        var corner = r.Tool("corner")!;
        Assert.True(corner.Ok, corner.Message);
        Assert.InRange(corner.Pose!.Angle, 9.3, 10.7);

        var followed = r.Tool("hole")!;
        Assert.True(followed.Passed, $"followed region covers {Num(followed, "percent")}% hole");
        double cx = followed.Region.Average(p => p.X), cy = followed.Region.Average(p => p.Y);
        Assert.InRange(cx, hole.X - 1.5, hole.X + 1.5);
        Assert.InRange(cy, hole.Y - 1.5, hole.Y + 1.5);

        var stayed = r.Tool("holeFixed")!;
        Assert.False(stayed.Passed);                  // the unfixtured region missed the hole…
        Assert.True(r.Passed);                        // …but it is not part of the overall pass
    }

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

    [Fact]
    public void A_fourLines_fixture_carries_a_grid_through_perspective()
    {
        var dots = new HashSet<(int, int)> { (0, 0), (0, 3), (1, 2), (2, 1), (3, 3) };
        var teachQuad = new[] { (110.0, 60.0), (290.0, 60.0), (290.0, 240.0), (110.0, 240.0) };
        var prog = Program(
            // Drawn clockwise round the board, so every probe runs from outside in.
            Tool("top",    "findLine", Centered(200, 60, 120, 60, 0),    new { polarity = "darkToLight", pick = "first" }),
            Tool("right",  "findLine", Centered(290, 150, 120, 60, 90),  new { polarity = "darkToLight", pick = "first" }),
            Tool("bottom", "findLine", Centered(200, 240, 120, 60, 180), new { polarity = "darkToLight", pick = "first" }),
            Tool("left",   "findLine", Centered(110, 150, 120, 60, -90), new { polarity = "darkToLight", pick = "first" }),
            Tool("board",  "makeFixture", null, new { mode = "fourLines", sources = new[] { "top", "right", "bottom", "left" } }),
            Tool("cells",  "pixelCount", Quad(teachQuad), new { mode = "gray", grayMin = 0, grayMax = 100 }, fixture: "board",
                 grid: new VisionGrid { Rows = 4, Cols = 4, Inset = 0.2 }, limits: new() { ["percent"] = new() { Min = 20 } }, includeInPass: false));

        using (var teach = Board(teachQuad, dots))
        {
            var r0 = Run(prog, teach);
            Assert.All(r0.Tools, t => Assert.True(t.Ok, $"{t.Id}: {t.Message}"));
            Assert.Equal(dots.OrderBy(d => d), r0.Tool("cells")!.Cells.Where(c => c.Passed).Select(c => (c.Row, c.Col)).OrderBy(d => d));
        }

        var seen = new[] { (130.0, 70.0), (280.0, 55.0), (320.0, 250.0), (90.0, 235.0) };
        using var img = Board(seen, dots);
        var r = Run(prog, img);
        var cells = r.Tool("cells")!;
        Assert.True(cells.Ok, cells.Message);
        Assert.Equal(dots.OrderBy(d => d), cells.Cells.Where(c => c.Passed).Select(c => (c.Row, c.Col)).OrderBy(d => d));
        // The region as used is the board as seen now (a perspective quad, not a similarity guess).
        for (int i = 0; i < 4; i++)
        {
            Assert.InRange(cells.Region[i].X, seen[i].Item1 - 2, seen[i].Item1 + 2);
            Assert.InRange(cells.Region[i].Y, seen[i].Item2 - 2, seen[i].Item2 + 2);
        }
        Assert.Equal(4, r.Tool("board")!.Pose!.Quad!.Count);
    }

    // ── notRun propagation ──────────────────────────────────────────────────────

    [Fact]
    public void A_missing_fixture_leaves_every_dependent_not_run()
    {
        var prog = Program(
            Tool("edge", "findLine", Centered(200, 150, 300, 60), new { polarity = "darkToLight" }),
            Tool("fix",  "makeFixture", null, new { mode = "pointAngle", sources = new[] { "edge", "edge" } }),
            Tool("a",    "pixelCount", Centered(200, 150, 40, 40), new { mode = "gray", grayMin = 0, grayMax = 255 }, fixture: "edge"),
            Tool("b",    "intensity", Centered(200, 150, 40, 40), null, fixture: "fix"),
            Tool("free", "intensity", Centered(200, 150, 40, 40), null));
        prog.Tools[0].Name = "Edge";

        using var blank = Canvas(40);
        var r = Run(prog, blank);
        Assert.Equal(ToolStatus.Fail, r.Tool("edge")!.Status);
        Assert.Equal(ToolStatus.NotRun, r.Tool("fix")!.Status);           // its source is missing
        Assert.Contains("Edge", r.Tool("fix")!.Message);
        Assert.Equal(ToolStatus.NotRun, r.Tool("a")!.Status);
        Assert.Contains("fixture 'Edge' not found", r.Tool("a")!.Message);
        Assert.Equal(ToolStatus.NotRun, r.Tool("b")!.Status);              // through the chain
        Assert.Contains("did not run", r.Tool("b")!.Message);
        Assert.All(new[] { "edge", "fix", "a", "b" }, id => Assert.False(r.Tool(id)!.Passed));
        Assert.Empty(r.Tool("a")!.Values);                                // never ran at the untransformed region
        Assert.Equal(ToolStatus.Ok, r.Tool("free")!.Status);
        Assert.False(r.Passed);
        Assert.Null(prog.Tool("edge")!.Reference);                         // nothing to capture

        // A disabled fixture, and a fixture that comes later in the list, also stop a tool.
        prog.Tools[0].Enabled = false;
        var r2 = Run(prog, blank);
        Assert.Null(r2.Tool("edge"));
        Assert.Contains("disabled", r2.Tool("a")!.Message);
        var later = Program(Tool("x", "intensity", null, null, fixture: "y"), Tool("y", "blob"));
        Assert.Contains("not an earlier tool", Run(later, blank).Tool("x")!.Message);
    }

    // ── Limits and pass ─────────────────────────────────────────────────────────

    [Fact]
    public void Limits_judge_values_and_the_program_passes_on_its_included_tools()
    {
        using var img = Canvas(50);
        Cv2.Rectangle(img, new Rect(200, 0, 200, 300), Scalar.All(150), -1);
        VisionTool Mean(string id, double? min, double? max, bool include = true) =>
            Tool(id, "intensity", null, new { statistic = "mean" }, limits: new() { ["mean"] = new() { Min = min, Max = max } }, includeInPass: include);

        var r = Run(Program(Mean("in", 90, 110), Mean("low", 120, null, include: false)), img);
        Assert.True(r.Tool("in")!.Passed);
        Assert.False(r.Tool("low")!.Passed);
        Assert.True(r.Passed);                               // "low" is not included

        r = Run(Program(Mean("in", 90, 110), Mean("low", 120, null)), img);
        Assert.False(r.Passed);

        var missing = Tool("m", "intensity", null, null, limits: new() { ["nonsense"] = new() { Min = 0 } });
        Assert.False(RunOne(img, missing).Passed);           // a limit on a value the tool does not have fails

        Assert.True(Run(Program(), img).Passed);             // nothing to fail
    }

    [Fact]
    public void A_captured_reference_is_kept_until_cleared()
    {
        var prog = PartProgram();
        using var teach = Part(180, 140, 0, out _);
        using var moved = Part(215, 155, 10, out _);
        var r1 = VisionProcessor.RunProgramOnce(prog, teach, out _, out bool captured1);
        Assert.True(captured1);
        var reference = prog.Tool("corner")!.Reference!;
        VisionProcessor.RunProgramOnce(prog, moved, out _, out bool captured2);
        Assert.False(captured2);
        Assert.Same(reference, prog.Tool("corner")!.Reference);
        Assert.NotNull(r1.Tool("corner")!.Pose);
    }

    // ── Program JSON ────────────────────────────────────────────────────────────

    [Fact]
    public void Program_json_keeps_flat_filter_params_and_tool_params()
    {
        const string json = """
        {
          "id": "p", "name": "P", "cameraId": "cam",
          "layers": [ { "id": "soft", "name": "Soft", "input": "camera",
                        "filters": [ { "id": "f1", "type": "blur", "enabled": true, "size": 7 },
                                     { "id": "f2", "type": "threshold", "mode": "otsu", "invert": true } ] } ],
          "tools": [ { "id": "t1", "name": "Top", "type": "findLine", "input": "soft", "fixtureId": null,
                       "region": { "shape": "rect", "x": 0.1, "y": 0.2, "width": 0.5, "height": 0.1, "rotation": 5 },
                       "grid": { "rows": 8, "cols": 8, "inset": 0.25 },
                       "limits": { "percent": { "min": 20, "max": null } }, "includeInPass": false,
                       "reference": { "x": 1, "y": 2, "angle": 3, "scale": 1, "quad": [ {"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1},{"x":0,"y":1} ] },
                       "params": { "probes": 12, "polarity": "darkToLight", "sources": ["a", "b"] } } ]
        }
        """;
        var p = JsonSerializer.Deserialize<VisionProgram>(json)!;
        Assert.Equal(7, p.Layers[0].Filters[0].P.Int("size", 0));
        Assert.True(p.Layers[0].Filters[1].P.Bool("invert", false));
        Assert.Equal(12, p.Tools[0].P.Int("probes", 0));
        Assert.Equal(new[] { "a", "b" }, p.Tools[0].P.StrList("sources"));
        Assert.Equal(4, p.Tools[0].Reference!.Quad!.Count);
        Assert.False(p.Tools[0].IncludeInPass);

        var back = JsonDocument.Parse(JsonSerializer.Serialize(p)).RootElement;
        var f1 = back.GetProperty("layers")[0].GetProperty("filters")[0];
        Assert.Equal(7, f1.GetProperty("size").GetInt32());           // still flat on the filter
        Assert.Equal("darkToLight", back.GetProperty("tools")[0].GetProperty("params").GetProperty("polarity").GetString());
    }

    [Fact]
    public void Old_zone_programs_are_recognised_and_not_loaded()
    {
        Assert.True(VisionProgramRepository.IsOldFormat("""{"id":"x","zones":[],"inspections":[]}"""));
        Assert.True(VisionProgramRepository.IsOldFormat("""{"id":"x","colorInspections":[]}"""));
        Assert.False(VisionProgramRepository.IsOldFormat("""{"id":"x","layers":[],"tools":[]}"""));

        var dir = Path.Combine(Path.GetTempPath(), "vision-repo-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        try
        {
            File.WriteAllText(Path.Combine(dir, "old.json"), """{"id":"old","name":"Old","zones":[{"id":"z"}],"inspections":[]}""");
            File.WriteAllText(Path.Combine(dir, "new.json"), """{"id":"new","name":"New","layers":[],"tools":[{"id":"t","type":"blob"}]}""");
            var repo = new VisionProgramRepository(dir);
            Assert.Null(repo.Get("old"));
            Assert.Equal("blob", repo.Get("new")!.Tools[0].Type);
            Assert.True(File.Exists(Path.Combine(dir, "old.json")));      // left alone on disk
        }
        finally { Directory.Delete(dir, true); }
    }

    // ── RunVision outputs ───────────────────────────────────────────────────────

    private static VisionResult SampleResult()
    {
        var line = new ToolResult { Id = "line", Type = "findLine", Status = ToolStatus.Ok, Passed = true, Pose = new VisionPose { X = 320, Y = 240, Angle = 12.5 } };
        line.Set("angle", 12.5); line.Set("x", 320); line.Set("y", 240); line.Set("found", true);
        line.Points.Add(new PxPoint(64, 48));
        line.Lines.Add(new PxLine(0, 0, 640, 480));
        var cells = new ToolResult { Id = "cells", Type = "pixelCount", Status = ToolStatus.Ok, Passed = false };
        cells.Cells.Add(new CellResult { Row = 0, Col = 1, Index = 1, Value = 42.5, Passed = true, Quad = new() { new PxPoint(1, 1) } });
        var code = new ToolResult { Id = "code", Type = "barcode", Status = ToolStatus.Ok, Passed = true };
        code.Set("value", "ABC"); code.Set("count", 1);
        code.Items.Add(new Dictionary<string, object> { ["value"] = "ABC", ["x"] = 160.0, ["y"] = 120.0 });
        var gone = new ToolResult { Id = "gone", Type = "blob", Status = ToolStatus.NotRun, Message = "fixture 'F' not found" };
        return new VisionResult { ProgramId = "p", ImageWidth = 640, ImageHeight = 480, Passed = true, Tools = { line, cells, code, gone } };
    }

    [Fact]
    public void RunVision_writes_pass_and_tool_outputs_in_the_output_frame()
    {
        var step = new ProgramStep
        {
            Type = StepType.RunVision, PassVariable = "$visionOk",
            ToolOutputs =
            [
                new() { ToolId = "line",  Value = "angle",  Variable = "edgeAngle" },
                new() { ToolId = "line",  Value = "x",      Variable = "edgeX" },
                new() { ToolId = "line",  Value = "passed", Variable = "edgeOk" },
                new() { ToolId = "line",  Value = "points", Variable = "pts" },
                new() { ToolId = "line",  Value = "pose",   Variable = "pose" },
                new() { ToolId = "line",  Value = "lines",  Variable = "segs" },
                new() { ToolId = "cells", Value = "cells",  Variable = "whiteCells" },
                new() { ToolId = "cells", Value = "passed", Variable = "cellsOk" },
                new() { ToolId = "code",  Value = "value",  Variable = "text" },
                new() { ToolId = "code",  Value = "items",  Variable = "codes" },
                new() { ToolId = "gone",  Value = "items",  Variable = "goneItems" },
                new() { ToolId = "gone",  Value = "count",  Variable = "goneCount" },
                new() { ToolId = "nope",  Value = "found",  Variable = "nopeFound" },
            ],
        };
        var vars = new VariableScope();
        vars.SetList("goneItems", ListVar.OfRecords([new ObjectRecord { ["x"] = 1 }]));   // last run's
        var frame = new VisionOutputFrame(OutputFrameKind.Normalized, 640, 480);
        VisionStepOutputs.WriteOutputs(step, SampleResult(), vars, frame);

        var v = vars.MergedVars();
        Assert.Equal(1, v["visionOk"]);
        Assert.Equal(12.5, v["edgeAngle"]);
        Assert.Equal(0.5, v["edgeX"]);                       // x converted with y by the output frame
        Assert.Equal(1, v["edgeOk"]);
        Assert.Equal(0, v["cellsOk"]);
        Assert.Equal(0, v["goneCount"]);
        Assert.Equal(0, v["nopeFound"]);
        Assert.True(vars.TryGetString("text", out var text));
        Assert.Equal("ABC", text);

        var pts = vars.Lists["pts"];
        Assert.Equal(ListElementType.Point, pts.ElementType);
        Assert.Equal(0.1, pts.Items[0]["x"], 6);
        Assert.Equal(0.1, pts.Items[0]["y"], 6);
        var pose = vars.Lists["pose"].Items.Single();
        Assert.Equal(0.5, pose["x"], 6); Assert.Equal(0.5, pose["y"], 6); Assert.Equal(12.5, pose["angle"]);
        Assert.Equal(1.0, vars.Lists["segs"].Items[0]["x2"], 6);
        var cell = vars.Lists["whiteCells"].Items.Single();
        Assert.Equal(new[] { "row", "col", "index", "value", "passed" }.OrderBy(k => k), cell.Keys.OrderBy(k => k));
        Assert.Equal(42.5, cell["value"]); Assert.Equal(1, cell["passed"]); Assert.Equal(1, cell["index"]);
        var codeItem = vars.Lists["codes"].Items.Single();
        Assert.False(codeItem.ContainsKey("value"));         // records are numeric: the text is dropped
        Assert.Equal(0.25, codeItem["x"], 6);
        Assert.Empty(vars.Lists["goneItems"].Items);         // a tool that did not run empties its lists
    }

    [Fact]
    public void RunVision_defaults_to_pixels()
    {
        var step = new ProgramStep
        {
            Type = StepType.RunVision,
            ToolOutputs = [new() { ToolId = "line", Value = "x", Variable = "x" }, new() { ToolId = "line", Value = "pose", Variable = "p" }],
        };
        var vars = new VariableScope();
        VisionStepOutputs.WriteOutputs(step, SampleResult(), vars);
        Assert.Equal(320, vars.MergedVars()["x"]);
        Assert.Equal(240, vars.Lists["p"].Items[0]["y"]);
    }
}
