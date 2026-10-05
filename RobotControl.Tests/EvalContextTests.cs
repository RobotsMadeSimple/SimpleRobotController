using Controller.RobotControl;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

public class EvalContextTests
{
    // An IO source that reports a different input value every time it is read, so a
    // rebuilt dictionary is distinguishable from a reused one.
    private static VariableScope ScopeWithCountingIo(Func<int> reads, Action bump) =>
        new(ioSource: d => { bump(); d["stb.in1"] = reads(); });

    private static ProgramStep MoveWith(params (string Field, string Expr)[] exprs) => new()
    {
        Type = StepType.MoveL,
        Expressions = exprs.ToDictionary(e => e.Field, e => e.Expr),
    };

    [Fact]
    public void TwoFieldsInOneStep_SeeTheSameSnapshot()
    {
        int reads = 0;
        var scope = ScopeWithCountingIo(() => reads, () => reads++);
        var step = MoveWith(("offsetX", "$stb.in1"), ("offsetY", "$stb.in1"), ("offsetZ", "$time_ms"), ("offsetRZ", "$time_ms"));

        scope.Eval.BeginTick();
        double x  = scope.Eval.EvalField(step, "offsetX", 0);
        double y  = scope.Eval.EvalField(step, "offsetY", 0);
        double z  = scope.Eval.EvalField(step, "offsetZ", 0);
        Thread.Sleep(5);
        double rz = scope.Eval.EvalField(step, "offsetRZ", 0);
        scope.Eval.EndTick();

        Assert.Equal(x, y);          // one IO read served both fields
        Assert.Equal(1, reads);
        Assert.Equal(z, rz);         // and one time_ms
    }

    [Fact]
    public void Snapshot_IsReusedAcrossCallsWithinATick()
    {
        var scope = new VariableScope();
        scope.Set("a", 1);
        scope.Eval.BeginTick();
        var first = scope.Eval.Vars;
        Assert.Same(first, scope.Eval.Vars);
        scope.Eval.EndTick();
    }

    [Fact]
    public void WriteWithinATick_InvalidatesTheSnapshot()
    {
        var scope = new VariableScope();
        scope.Set("a", 1);
        var step = MoveWith(("offsetX", "$a"));

        scope.Eval.BeginTick();
        Assert.Equal(1, scope.Eval.EvalField(step, "offsetX", 0));
        scope.Set("a", 7);
        Assert.Equal(7, scope.Eval.EvalField(step, "offsetX", 0));
        scope.Eval.EndTick();
    }

    [Fact]
    public void OutsideATick_EveryAccessIsFresh()
    {
        int reads = 0;
        var scope = ScopeWithCountingIo(() => reads, () => reads++);
        var a = scope.Eval.Vars;
        var b = scope.Eval.Vars;
        Assert.NotSame(a, b);
        Assert.Equal(2, reads);
    }

    [Fact]
    public void NewTick_RebuildsTheSnapshot()
    {
        int reads = 0;
        var scope = ScopeWithCountingIo(() => reads, () => reads++);
        var step = MoveWith(("offsetX", "$stb.in1"));

        scope.Eval.BeginTick();
        double first = scope.Eval.EvalField(step, "offsetX", 0);
        scope.Eval.EndTick();
        scope.Eval.BeginTick();
        double second = scope.Eval.EvalField(step, "offsetX", 0);
        scope.Eval.EndTick();

        Assert.NotEqual(first, second);
    }

    [Fact]
    public void OptionalField_NullWhenNeitherLiteralNorExpression()
    {
        var scope = new VariableScope();
        var step = MoveWith(("speed", "2 * 5"));
        Assert.Null(scope.Eval.OptionalField(step, "accel", null));
        Assert.Equal(3, scope.Eval.OptionalField(step, "accel", 3));
        Assert.Equal(10, scope.Eval.OptionalField(step, "speed", null));
    }

    [Fact]
    public void UnknownVariable_Propagates()
    {
        var scope = new VariableScope();
        var step = MoveWith(("offsetX", "$typo"));
        Assert.Throws<UnknownVariableException>(() => scope.Eval.EvalField(step, "offsetX", 0));
    }
}
