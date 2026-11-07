using Controller.RobotControl;
using Controller.RobotControl.Validation;
using static RobotControl.Tests.FlowTestKit;

namespace RobotControl.Tests;

/// <summary>
/// Routine parameters (docs/program-language.md §4.2): in / out / inOut, arguments evaluated in
/// the caller's scope before the call, outputs copied back on Return or at the routine's end.
/// </summary>
[Collection("RobotController")]
public sealed class RoutineParamTests : IDisposable
{
    private readonly FlowTestKit _k = new();
    public void Dispose() => _k.Dispose();

    private static List<PluginOutputMapping> Out(params (string Key, string Var)[] maps) =>
        maps.Select(m => new PluginOutputMapping { Key = m.Key, VariableName = m.Var }).ToList();

    private ValidationContext Ctx() =>
        new() { FindProgram = (id, name) => (id != null ? _k.Repo.GetById(id) : null) ?? _k.Repo.Get(name ?? "") };

    [Fact]
    public void InParamsTakeTheirArgumentsAndOutParamsComeBack()
    {
        var r = _k.Routine("Scale", [Num("a", 1, ParamModes.In), Num("factor", 2, ParamModes.In), Num("result", 0, ParamModes.Out)],
            Set("result", "$a * $factor"));
        var call = Call(r, s =>
        {
            s.RoutineArgs    = new() { ["a"] = "$x + 1" }; // factor keeps its declared 2
            s.RoutineOutputs = Out(("result", "y"));
        });
        _k.RunToEnd(Prog([Num("x", 4), Num("y")], call));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(10, _k.Value("$y"));
    }

    [Fact]
    public void ArgumentsAreEvaluatedInTheCallersScopeBeforeTheRoutineInitialises()
    {
        // The routine's own "n" starts at 100, but the argument reads the caller's n (3).
        var r = _k.Routine("R", [Num("n", 100), Num("p", 0, ParamModes.In), Num("got", 0, ParamModes.Out)],
            Set("got", "$p"));
        var call = Call(r, s => { s.RoutineArgs = new() { ["p"] = "$n * 2" }; s.RoutineOutputs = Out(("got", "y")); });
        _k.RunToEnd(Prog([Num("n", 3), Num("y")], call));
        Assert.Equal(6, _k.Value("$y"));
    }

    [Fact]
    public void InOutGoesBothWays()
    {
        var r = _k.Routine("Inc", [Num("count", 0, ParamModes.InOut)], Set("count", "$count + 1"));
        var call = Call(r, s => { s.RoutineArgs = new() { ["count"] = "$x" }; s.RoutineOutputs = Out(("count", "x")); });
        _k.RunToEnd(Prog([Num("x", 7)], call, call));
        Assert.Equal(9, _k.Value("$x"));
    }

    [Fact]
    public void OutputsAreCopiedOnReturn()
    {
        var r = _k.Routine("Find", [Num("found", 0, ParamModes.Out)],
            Loop(0, Set("found", "$found + 1"), If("$found >= 3", [Step(StepType.Return)])));
        var call = Call(r, s => s.RoutineOutputs = Out(("found", "y")));
        _k.RunToEnd(Prog([Num("y")], call, Set("y", "$y * 10")));
        Assert.Equal(30, _k.Value("$y"));
    }

    [Fact]
    public void TextAndBooleanParams()
    {
        var r = _k.Routine("Label", [Text("label", "", ParamModes.In), Bool("ok", ParamModes.Out), Text("echo", "", ParamModes.Out)],
            Set("ok", "1"), Set("echo", "<{$label}>"));
        var call = Call(r, s =>
        {
            s.RoutineArgs = new() { ["label"] = "Bin {$x}" };
            s.RoutineOutputs = Out(("ok", "flag"), ("echo", "said"));
        });
        _k.RunToEnd(Prog([Num("x", 2), Bool("flag"), Text("said")], call));
        Assert.Equal(1, _k.Value("$flag"));
        Assert.Equal("<Bin 2>", _k.Text("said"));
    }

    [Fact]
    public void ABooleanArgumentIsStoredAsZeroOrOne()
    {
        var r = _k.Routine("B", [Bool("b", ParamModes.In), Num("seen", 0, ParamModes.Out)], Set("seen", "$b"));
        var call = Call(r, s => { s.RoutineArgs = new() { ["b"] = "5" }; s.RoutineOutputs = Out(("seen", "y")); });
        _k.RunToEnd(Prog([Num("y")], call));
        Assert.Equal(1, _k.Value("$y"));
    }

    [Fact]
    public void AListParamIsCopiedIn()
    {
        var listParam = new ProgramVariable { Id = "pts", Name = "pts", Items = [], ElementType = ListElementType.Number, ParamMode = ParamModes.In };
        var r = _k.Routine("Count", [listParam, Num("len", 0, ParamModes.Out)], Set("len", "$pts.length"));
        var call = Call(r, s => { s.RoutineArgs = new() { ["pts"] = "$mine" }; s.RoutineOutputs = Out(("len", "y")); });
        var mine = new ProgramVariable
        {
            Id = "mine", Name = "mine", ElementType = ListElementType.Number,
            Items = [ObjectRecord.FromScalar(1), ObjectRecord.FromScalar(2), ObjectRecord.FromScalar(3)],
        };
        _k.RunToEnd(Prog([mine, Num("y")], call));
        Assert.Equal(3, _k.Value("$y"));
    }

    [Fact]
    public void AnEmptyRoutineStillCopiesItsOutputs()
    {
        var r = _k.Routine("Nothing", [Num("v", 5, ParamModes.Out)]);
        _k.RunToEnd(Prog([Num("y")], Call(r, s => s.RoutineOutputs = Out(("v", "y")))));
        Assert.Equal(5, _k.Value("$y"));
    }

    [Fact]
    public void AnInParamIsNotCopiedBack()
    {
        var r = _k.Routine("R", [Num("p", 0, ParamModes.In)], Set("p", "99"));
        _k.RunToEnd(Prog([Num("y", 1)], Call(r, s => { s.RoutineArgs = new() { ["p"] = "1" }; s.RoutineOutputs = Out(("p", "y")); })));
        Assert.Equal(1, _k.Value("$y"));
    }

    // ── Validator ─────────────────────────────────────────────────────────────

    [Fact]
    public void ValidatorChecksArgumentsAndOutputs()
    {
        var r = _k.Routine("R", [Num("a", 0, ParamModes.In), Num("b", 0, ParamModes.In), Text("t", "", ParamModes.Out), Num("o", 0, ParamModes.Out)]);
        var call = Call(r, s =>
        {
            s.RoutineArgs = new() { ["a"] = "$x", ["nope"] = "1", ["o"] = "2" };
            s.RoutineOutputs = Out(("t", "x"), ("a", "y"), ("missing", "y"));
        });
        var problems = ProgramValidator.Validate(Prog([Num("x"), Num("y")], call), Ctx());

        Assert.Contains(problems, p => p.Code == ValidationCodes.UnknownParam && p.Field == "routineArgs.nope" && p.Severity == ValidationSeverity.Error);
        Assert.Contains(problems, p => p.Code == ValidationCodes.UnknownParam && p.Field == "routineArgs.o" && p.Severity == ValidationSeverity.Warning);
        Assert.Contains(problems, p => p.Code == ValidationCodes.ParamMissing && p.Field == "routineArgs.b");
        Assert.DoesNotContain(problems, p => p.Code == ValidationCodes.ParamMissing && p.Field == "routineArgs.a");
        Assert.Contains(problems, p => p.Code == ValidationCodes.ParamOutputType && p.Field == "routineOutputs[0].variableName");
        Assert.Contains(problems, p => p.Code == ValidationCodes.UnknownParam && p.Field == "routineOutputs[1].key");
        Assert.Contains(problems, p => p.Code == ValidationCodes.UnknownParam && p.Field == "routineOutputs[2].key");
    }

    [Fact]
    public void ValidatorChecksArgumentExpressions()
    {
        var r = _k.Routine("R", [Num("a", 0, ParamModes.In)]);
        var problems = ProgramValidator.Validate(Prog(Call(r, s => s.RoutineArgs = new() { ["a"] = "$typo + 1" })), Ctx());
        Assert.Contains(problems, p => p.Code == ValidationCodes.UnknownVariable && p.Field == "routineArgs.a");
    }

    [Fact]
    public void AnUndeclaredOutputTargetIsCreatedWithTheParamsKind()
    {
        var r = _k.Routine("R", [Text("name", "", ParamModes.Out)]);
        var call = Call(r, s => s.RoutineOutputs = Out(("name", "who")));
        var status = Step(StepType.StatusUpdate, s => s.StatusMessage = "hello {$who}");
        var problems = ProgramValidator.Validate(Prog(call, status), Ctx());
        Assert.DoesNotContain(problems, p => p.Code == ValidationCodes.UnknownVariable);
    }
}
