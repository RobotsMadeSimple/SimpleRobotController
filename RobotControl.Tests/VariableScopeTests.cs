using Controller.RobotControl;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

public class VariableScopeTests
{
    private static BuiltProgram Program(params ProgramVariable[] vars) =>
        new() { Id = "p1", Name = "P", Variables = [.. vars] };

    [Fact]
    public void SetThenGet_ScalarIsVisibleInMergedVars()
    {
        var scope = new VariableScope();
        scope.Set("count", 3);
        var merged = scope.MergedVars();
        Assert.Equal(3, merged["count"]);
        Assert.True(scope.TryGetLocal("count", out var v));
        Assert.Equal(3, v);
    }

    [Fact]
    public void MergedVars_AlwaysCarriesTimeMs()
    {
        var merged = new VariableScope().MergedVars();
        Assert.True(merged.TryGetValue("time_ms", out var t));
        Assert.True(t > 0);
    }

    [Fact]
    public void Names_AreCaseInsensitive()
    {
        var scope = new VariableScope();
        scope.Set("Speed", 10);
        scope.Set("SPEED", 20); // same variable
        Assert.Equal(20, scope.MergedVars()["speed"]);
        Assert.True(scope.TryGetLocal("sPeEd", out var v));
        Assert.Equal(20, v);

        scope.Initialize(Program(new ProgramVariable { Name = "Label", IsString = true, StringValue = "a" }));
        Assert.True(scope.IsString("LABEL"));
    }

    [Fact]
    public void GlobalVariable_WritesGoToSharedStoreAndWinOverLocal()
    {
        var globals = new GlobalVariableStore();
        var a = new VariableScope(globals);
        var b = new VariableScope(globals);
        var prog = Program(new ProgramVariable { Name = "shared", Value = 5, IsGlobal = true });

        a.Initialize(prog);
        b.Initialize(prog);                      // first writer wins: stays 5
        Assert.Equal(5, b.MergedVars()["shared"]);

        a.Set("shared", 42);                     // routed to the store, not a local
        Assert.False(a.TryGetLocal("shared", out _));
        Assert.Equal(42, b.MergedVars()["shared"]);
    }

    [Fact]
    public void GlobalValue_TakesPrecedenceOverSameNamedLocal()
    {
        var globals = new GlobalVariableStore();
        globals.Set("x", 99);
        var scope = new VariableScope(globals);
        scope.Set("x", 1); // not registered as global → a local
        Assert.Equal(99, scope.MergedVars()["x"]);
    }

    [Fact]
    public void EvalVars_IncludesIoFromSource()
    {
        var scope = new VariableScope(ioSource: d => d["stb.in1"] = 1);
        Assert.Equal(1, scope.EvalVars()["stb.in1"]);
        Assert.False(scope.MergedVars().ContainsKey("stb.in1"));
    }

    [Fact]
    public void Initialize_ResolvesExpressionsInDeclarationOrder()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(
            new ProgramVariable { Name = "a", Value = 2 },
            new ProgramVariable { Name = "b", ValueExpression = "$a * 3" },
            new ProgramVariable { Name = "flag", ValueExpression = "$b", IsBoolean = true }));
        var m = scope.MergedVars();
        Assert.Equal(6, m["b"]);
        Assert.Equal(1, m["flag"]);
    }

    [Fact]
    public void Initialize_UnknownVariableInExpressionThrows()
    {
        var scope = new VariableScope();
        Assert.Throws<UnknownVariableException>(() =>
            scope.Initialize(Program(new ProgramVariable { Name = "a", ValueExpression = "$nope + 1" })));
    }

    [Fact]
    public void Interpolate_ScalarsBooleansStringsAndExpressions()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(
            new ProgramVariable { Name = "i", Value = 2 },
            new ProgramVariable { Name = "ok", Value = 1, IsBoolean = true },
            new ProgramVariable { Name = "name", IsString = true, StringValue = "bin" }));

        Assert.Equal("i=2 ok=True name=bin", scope.Interpolate("i=$i ok=$ok name=$name"));
        Assert.Equal("bin3", scope.Interpolate("{$name}{$i + 1}"));
        Assert.Equal("$missing", scope.Interpolate("$missing"));
        Assert.Equal("{i}", scope.Interpolate("{i}")); // forgotten sigil left as written
    }

    [Fact]
    public void Interpolate_ListElementsAndComponents()
    {
        var scope = new VariableScope();
        scope.SetList("pts", ListVar.OfPoints([new Vector6Val { X = 1, Y = 2, Z = 3 }]));
        scope.SetList("nums", ListVar.OfNumbers([10, 20]));
        scope.Set("k", 1);

        Assert.Equal("20", scope.Interpolate("$nums[$k]"));
        Assert.Equal("3", scope.Interpolate("$pts[0].z"));
        Assert.StartsWith("(x=1, y=2, z=3", scope.Interpolate("$pts[0]"));
    }

    [Fact]
    public void TryGetPointList_OnlyAcceptsPointLists()
    {
        var scope = new VariableScope();
        scope.SetList("pts", ListVar.OfPoints([new Vector6Val()]));
        scope.SetList("nums", ListVar.OfNumbers([1]));
        Assert.True(scope.TryGetPointList("PTS", out _));
        Assert.False(scope.TryGetPointList("nums", out _));
        Assert.False(scope.TryGetPointList("nope", out _));
    }

    [Fact]
    public void Images_CountRevisionsAndSurviveClearAsRevisions()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(new ProgramVariable { Name = "img", IsImage = true }));
        scope.SetImage("img", "AAA");
        scope.SetImage("img", "BBB");
        Assert.Equal("BBB", scope.GetImage("img"));
        Assert.Equal(2, scope.ImageRevision("img"));

        scope.Clear();
        Assert.False(scope.IsImage("img"));
        Assert.Equal(2, scope.ImageRevision("img"));
    }

    [Fact]
    public void Conditions_NumericAndString()
    {
        var scope = new VariableScope();
        scope.Initialize(Program(
            new ProgramVariable { Name = "n", Value = 5 },
            new ProgramVariable { Name = "s", IsString = true, StringValue = "hello world" }));
        var vars = scope.EvalVars();

        var all = new ConditionGroup
        {
            Combinator = "ALL",
            Items =
            [
                new ConditionItem { Left = "$n", Operator = ">", Right = "3" },
                new ConditionItem { Left = "$s", Operator = "contains", Right = "world" },
            ],
        };
        Assert.True(scope.EvaluateCondition(all, vars));

        var any = new ConditionGroup
        {
            Combinator = "ANY",
            Items = [new ConditionItem { Left = "$n", Operator = "==", Right = "4" }],
        };
        Assert.False(scope.EvaluateCondition(any, vars));
    }
}
