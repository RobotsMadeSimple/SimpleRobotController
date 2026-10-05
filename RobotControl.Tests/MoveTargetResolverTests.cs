using Controller.RobotControl;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Persistence;

namespace RobotControl.Tests;

public class MoveTargetResolverTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "rms-mtr-" + Guid.NewGuid().ToString("N"));
    private readonly MoveTargetSources _src;
    private readonly VariableScope _vars = new();
    private static readonly Func<Vector6> Live = () => new Vector6(500, 600, 700, 0, 0, 0);

    public MoveTargetResolverTests()
    {
        Directory.CreateDirectory(_dir);
        _src = new MoveTargetSources(
            new PointRepository(P("points.json"), P("pointsHistory.json")),
            new GridRepository(P("grids.json")),
            new StackRepository(P("stacks.json")),
            new LocalRepository(P("locals.json"), P("localsHistory.json")));
        _src.Points.SavePoint("base", new Vector6(100, 200, 50, 0, 0, 90));
    }

    public void Dispose() { try { Directory.Delete(_dir, recursive: true); } catch { } }

    private string P(string name) => Path.Combine(_dir, name);

    private Vector6 Resolve(ProgramStep step, Vector6? activeLocal = null)
    {
        Assert.True(MoveTargetResolver.TryResolve(step, _src, _vars, activeLocal, Live, out var t, out var err), err);
        return t;
    }

    private static void Near(double expected, double actual) => Assert.Equal(expected, actual, 6);

    [Fact]
    public void NamedPoint_PlusOffsets()
    {
        var t = Resolve(new ProgramStep { Type = StepType.MoveL, PointName = "base", OffsetX = 1, OffsetZ = -5 });
        Near(101, t.X); Near(200, t.Y); Near(45, t.Z); Near(90, t.RZ);
    }

    [Fact]
    public void Offsets_FromExpressions()
    {
        _vars.Set("dz", 10);
        var t = Resolve(new ProgramStep
        {
            Type = StepType.MoveL, PointName = "base",
            Expressions = new() { ["offsetZ"] = "$dz * 2" },
        });
        Near(70, t.Z);
    }

    [Fact]
    public void Overrides_ReplaceTheComputedAxis()
    {
        var t = Resolve(new ProgramStep
        {
            Type = StepType.MoveL, PointName = "base", OffsetX = 5,
            OverrideX = 1, OverrideZ = 2,
        });
        Near(1, t.X); Near(200, t.Y); Near(2, t.Z);
    }

    [Fact]
    public void NoTarget_IsRelativeToLivePosition()
    {
        var t = Resolve(new ProgramStep { Type = StepType.MoveL, OffsetY = 10 });
        Near(500, t.X); Near(610, t.Y); Near(700, t.Z);
    }

    [Fact]
    public void NoTarget_UsesSuppliedCurrentPositionInsteadOfLive()
    {
        Assert.True(MoveTargetResolver.TryResolve(new ProgramStep { Type = StepType.MoveL, OffsetX = 1 },
            _src, _vars, null, () => throw new InvalidOperationException("live read"),
            out var t, out _, currentPos: new Vector6(1, 2, 3, 0, 0, 0)));
        Near(2, t.X);
    }

    [Fact]
    public void Grid_RowColumnWithRotation()
    {
        var g = _src.Grids.Upsert(new Grid
        {
            Name = "g", BasePointName = "base",
            RowOffsetX = 10, ColOffsetY = 20, RowOffsetZ = 1, Rotation = 90,
        });
        var t = Resolve(new ProgramStep
        {
            Type = StepType.MoveL,
            GridPoint = new GridPointRef { GridId = g.Id, RowIndex = 2, ColIndex = 1 },
        });
        // raw (x=20, y=20) rotated 90° → (-20, 20)
        Near(80, t.X); Near(220, t.Y); Near(52, t.Z);
    }

    [Fact]
    public void Grid_LinearIndexNeedsColCount()
    {
        var g = _src.Grids.Upsert(new Grid { Name = "g", BasePointName = "base", RowOffsetX = 10, ColOffsetY = 1, ColCount = 3 });
        var t = Resolve(new ProgramStep
        {
            Type = StepType.MoveL,
            GridPoint = new GridPointRef { GridId = g.Id, UseGridIndex = true, GridIndex = 4 }, // row 1, col 1
        });
        Near(110, t.X); Near(201, t.Y);

        var bad = _src.Grids.Upsert(new Grid { Name = "nocols", BasePointName = "base" });
        Assert.False(MoveTargetResolver.TryResolve(new ProgramStep
        {
            Type = StepType.MoveL,
            GridPoint = new GridPointRef { GridId = bad.Id, UseGridIndex = true, GridIndex = 1 },
        }, _src, _vars, null, Live, out _, out var err));
        Assert.Contains("requires colCount", err);
    }

    [Fact]
    public void Stack_IndexWrapsAtMaxCount()
    {
        var s = _src.Stacks.Upsert(new RobotStack { Name = "s", BasePointName = "base", OffsetZ = 10, MaxCount = 3 });
        var step = new ProgramStep { Type = StepType.MoveL, StackPoint = new StackPointRef { StackId = s.Id, Index = 4 } };
        Near(60, Resolve(step).Z); // 4 mod 3 = 1

        step.StackPoint.Index = -1;
        Near(70, Resolve(step).Z); // -1 wraps to 2
    }

    [Fact]
    public void PointsListVariable_ClampedIndex()
    {
        _vars.SetList("pts", ListVar.OfPoints([new Vector6Val { X = 1 }, new Vector6Val { X = 2 }]));
        _vars.Set("i", 9);
        Near(2, Resolve(new ProgramStep { Type = StepType.MoveL, VarPointName = "pts", VarPointIndex = "$i" }).X);
        Near(1, Resolve(new ProgramStep { Type = StepType.MoveL, PointNameExpr = "$pts[0]" }).X);
    }

    [Fact]
    public void PointNameExpression_InterpolatesToASavedPoint()
    {
        _src.Points.SavePoint("bin2", new Vector6(7, 8, 9, 0, 0, 0));
        _vars.Set("n", 2);
        Near(7, Resolve(new ProgramStep { Type = StepType.MoveL, PointNameExpr = "bin$n" }).X);
    }

    [Fact]
    public void Errors_ComeBackAsMessages()
    {
        Assert.False(MoveTargetResolver.TryResolve(new ProgramStep { Type = StepType.MoveL, PointName = "nope" },
            _src, _vars, null, Live, out _, out var err));
        Assert.Equal("Point not found: nope", err);

        Assert.False(MoveTargetResolver.TryResolve(new ProgramStep
        {
            Type = StepType.MoveL, StackPoint = new StackPointRef { StackId = "missing" },
        }, _src, _vars, null, Live, out _, out err));
        Assert.Equal("Stack not found: missing", err);
    }

    [Fact]
    public void UnknownVariable_Propagates()
    {
        Assert.Throws<UnknownVariableException>(() => MoveTargetResolver.TryResolve(new ProgramStep
        {
            Type = StepType.MoveL, PointName = "base", Expressions = new() { ["offsetX"] = "$typo" },
        }, _src, _vars, null, Live, out _, out _));
    }

    [Fact]
    public void ActiveLocal_TranslatesAbsoluteTargetsButNotRelativeAxes()
    {
        var local = new Vector6(10, 20, 30, 0, 0, 0);
        var abs = Resolve(new ProgramStep { Type = StepType.MoveL, PointName = "base" }, local);
        Near(110, abs.X); Near(220, abs.Y); Near(80, abs.Z);

        // Relative move with only Z overridden: X/Y ride the live position, Z gets the local.
        var rel = Resolve(new ProgramStep { Type = StepType.MoveL, OverrideZ = 5 }, local);
        Near(500, rel.X); Near(600, rel.Y); Near(35, rel.Z);
    }

    [Fact]
    public void StepLocalName_OverridesActiveLocal()
    {
        _src.Locals.SaveLocal("L", new Vector6(1, 0, 0, 0, 0, 0));
        var t = Resolve(new ProgramStep { Type = StepType.MoveL, PointName = "base", LocalName = "L" },
                        new Vector6(1000, 0, 0, 0, 0, 0));
        Near(101, t.X);
    }

    [Fact]
    public void ToolOffset_NullWhenUnsetOtherwiseEvaluated()
    {
        Assert.Null(MoveTargetResolver.ResolveToolOffset(new ProgramStep(), _vars.Eval));
        var off = MoveTargetResolver.ResolveToolOffset(new ProgramStep
        {
            ToolOffsetZ = 5, Expressions = new() { ["toolOffsetX"] = "2 + 1" },
        }, _vars.Eval)!;
        // Any literal turns the offset on; then every axis evaluates (expression first).
        Near(3, off.X); Near(0, off.Y); Near(5, off.Z);
    }
}
