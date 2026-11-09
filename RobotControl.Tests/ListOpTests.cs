using Controller.RobotControl;

namespace RobotControl.Tests;

/// <summary>
/// ListOp (docs/program-language.md §4.3): every action — append, insert, removeAt, clear, set,
/// setField, sort, reverse, copyFrom — on every element type (Number, Boolean, Point, Record),
/// with index expressions, out-of-range indexes (indexOutOfRange), steps that do not fit the
/// list (badListOperation), and the program's declared list left untouched.
/// </summary>
[Collection("RobotController")]
public sealed class ListOpTests : IDisposable
{
    private readonly StepsRig _rig = new();
    public void Dispose() => _rig.Dispose();

    private static ProgramStep Op(string list, string action, Action<ProgramStep>? init = null) =>
        StepsRig.Step(StepType.ListOp, s => { s.VariableName = list; s.ListAction = action; init?.Invoke(s); });

    private static ProgramStep Value(string list, string action, string value, int? index = null) =>
        Op(list, action, s => { s.ListValueExpr = value; s.ListIndex = index; });

    private static ProgramStep Fields(string list, string action, int? index, params (string Field, string Expr)[] fields) =>
        Op(list, action, s =>
        {
            s.ListIndex  = index;
            s.ListFields = fields.ToDictionary(f => f.Field, f => f.Expr);
        });

    private double[] Values(string list) =>
        Enumerable.Range(0, _rig.Count(list)).Select(i => _rig.Live($"${list}[{i}]")).ToArray();

    private double[] Field(string list, string field) =>
        Enumerable.Range(0, _rig.Count(list)).Select(i => _rig.Live($"${list}[{i}].{field}")).ToArray();

    private void RunOk(IEnumerable<ProgramVariable> vars, params ProgramStep[] steps)
    {
        _rig.Run(StepsRig.Prog(vars, steps));
        _rig.AssertComplete();
    }

    // ── Number lists ──────────────────────────────────────────────────────────

    [Fact]
    public void NumberAppendInsertSetRemove()
    {
        RunOk([StepsRig.Numbers("v", 1, 2, 3), StepsRig.Num("x", 10)],
            Value("v", "append", "$x * 2"),          // 1 2 3 20
            Value("v", "insert", "-1", 0),           // -1 1 2 3 20
            Value("v", "set", "$v[1] + 100", 1),     // -1 101 2 3 20
            Op("v", "removeAt", s => s.ListIndex = 3)); // -1 101 2 20
        Assert.Equal([-1, 101, 2, 20], Values("v"));
    }

    [Fact]
    public void DefaultActionIsAppend()
    {
        RunOk([StepsRig.Numbers("v", 1)], Op("v", "", s => s.ListValueExpr = "5"));
        Assert.Equal([1, 5], Values("v"));
    }

    [Fact]
    public void InsertAtTheEndIsAllowed()
    {
        RunOk([StepsRig.Numbers("v", 1, 2)], Value("v", "insert", "9", 2));
        Assert.Equal([1, 2, 9], Values("v"));
    }

    [Fact]
    public void IndexComesFromItsExpression()
    {
        var step = Value("v", "set", "0", 0);
        step.Expressions = new() { ["listIndex"] = "len($v) - 1" };
        RunOk([StepsRig.Numbers("v", 1, 2, 3)], step);
        Assert.Equal([1, 2, 0], Values("v"));
    }

    [Fact]
    public void ClearSortReverse()
    {
        RunOk([StepsRig.Numbers("a", 3, 1, 2), StepsRig.Numbers("b", 3, 1, 2), StepsRig.Numbers("c", 3, 1, 2), StepsRig.Numbers("d", 4, 5)],
            Op("a", "sort"),
            Op("b", "sort", s => s.ListSortDescending = true),
            Op("c", "reverse"),
            Op("d", "clear"));
        Assert.Equal([1, 2, 3], Values("a"));
        Assert.Equal([3, 2, 1], Values("b"));
        Assert.Equal([2, 1, 3], Values("c"));
        Assert.Equal(0, _rig.Count("d"));
    }

    [Fact]
    public void SortByValueFieldNameIsAccepted()
    {
        RunOk([StepsRig.Numbers("a", 2, 1)], Op("a", "sort", s => s.ListSortField = "value"));
        Assert.Equal([1, 2], Values("a"));
    }

    [Theory]
    [InlineData("insert", 4)]
    [InlineData("insert", -1)]
    [InlineData("set", 3)]
    [InlineData("removeAt", 3)]
    [InlineData("removeAt", -2)]
    [InlineData("setField", 5)]
    public void OutOfRangeIndexFailsAndLeavesTheListAlone(string action, int index)
    {
        var step = action == "setField"
            ? Fields("r", action, index, ("w", "1"))
            : Op(action == "setField" ? "r" : "v", action, s => { s.ListIndex = index; s.ListValueExpr = "7"; });
        _rig.Run(StepsRig.Prog([StepsRig.Numbers("v", 1, 2, 3), StepsRig.Records("r", StepsRig.Rec(("w", 0)))],
            step, StepsRig.Set("after", "1")));
        _rig.AssertError($"index {index} is outside");
        Assert.Equal([1, 2, 3], Values("v"));
    }

    [Fact]
    public void RemoveFromAnEmptyListSaysItIsEmpty()
    {
        _rig.Run(StepsRig.Prog([StepsRig.Numbers("v")], Op("v", "removeAt", s => s.ListIndex = 0)));
        _rig.AssertError("the list is empty");
    }

    [Fact]
    public void ANonNumberIndexIsOutOfRange()
    {
        var step = Op("v", "removeAt", s => s.Expressions = new() { ["listIndex"] = "sqrt(-1)" });
        _rig.Run(StepsRig.Prog([StepsRig.Numbers("v", 1)], step));
        _rig.AssertError("index -1 is outside");
    }

    [Fact]
    public void NumberListMisfits()
    {
        _rig.Run(StepsRig.Prog([StepsRig.Numbers("v", 1)], Op("v", "append")));
        _rig.AssertError("no value is set");

        _rig.Run(StepsRig.Prog([StepsRig.Numbers("v", 1)], Fields("v", "setField", 0, ("x", "1"))));
        _rig.AssertError("its elements have no fields");

        _rig.Run(StepsRig.Prog([StepsRig.Numbers("v", 1)], Fields("v", "append", null, ("x", "1"))));
        _rig.AssertError("no value is set");

        _rig.Run(StepsRig.Prog([StepsRig.Numbers("v", 1)], Op("v", "sort", s => s.ListSortField = "x")));
        _rig.AssertError("sorts by value");

        _rig.Run(StepsRig.Prog([StepsRig.Numbers("v", 1)], Op("v", "shuffle")));
        _rig.AssertError("unknown list action 'shuffle'");
    }

    [Fact]
    public void AValueFieldStandsInForTheValue()
    {
        RunOk([StepsRig.Numbers("v")], Fields("v", "append", null, ("value", "4")));
        Assert.Equal([4], Values("v"));
    }

    // ── Boolean lists ─────────────────────────────────────────────────────────

    [Fact]
    public void BooleanElementsAreStoredAsZeroOrOne()
    {
        RunOk([StepsRig.Booleans("b", true), StepsRig.Num("n", 3)],
            Value("b", "append", "$n > 2"),
            Value("b", "append", "5"),
            Value("b", "append", "0"),
            Value("b", "set", "-2", 0),
            Op("b", "sort"));
        Assert.Equal([0, 1, 1, 1], Values("b"));
    }

    // ── Point lists ───────────────────────────────────────────────────────────

    [Fact]
    public void PointAppendWithNoFieldsIsHere()
    {
        RunOk([StepsRig.PointList("p")], Op("p", "append"));
        var h = _rig.Robot.GetCurrentPosition();
        Assert.Equal(h.X, _rig.Live("$p[0].x"), 9);
        Assert.Equal(h.Z, _rig.Live("$p[0].z"), 9);
        Assert.Equal(h.RZ, _rig.Live("$p[0].rz"), 9);
    }

    [Fact]
    public void PointFieldsAreAxes()
    {
        RunOk([StepsRig.PointList("p", StepsRig.P(1, 2, 3, rz: 4)), StepsRig.Num("h", 7)],
            Fields("p", "append", null, ("X", "$h"), ("y", "8")),            // missing axes start at 0
            Fields("p", "insert", 0, ("z", "-1")),
            Fields("p", "set", 2, ("z", "99")),                              // set keeps the axes it does not name
            Fields("p", "setField", 1, ("rz", "45")));
        Assert.Equal([0, 1, 7], Field("p", "x"));
        Assert.Equal([0, 2, 8], Field("p", "y"));
        Assert.Equal([-1, 3, 99], Field("p", "z"));
        Assert.Equal([0, 45, 0], Field("p", "rz"));
        Assert.Equal(7, _rig.Live("$p[2][0]"));   // positional access still works on the new element
    }

    [Fact]
    public void PointSortByAxis()
    {
        RunOk([StepsRig.PointList("p", StepsRig.P(3, 0, 0), StepsRig.P(1, 1, 0), StepsRig.P(2, 2, 0))],
            Op("p", "sort", s => { s.ListSortField = "X"; s.ListSortDescending = true; }));
        Assert.Equal([3, 2, 1], Field("p", "x"));
        Assert.Equal([0, 2, 1], Field("p", "y")); // whole elements moved, not one axis
    }

    [Fact]
    public void PointListMisfits()
    {
        _rig.Run(StepsRig.Prog([StepsRig.PointList("p", StepsRig.P(0, 0, 0))], Fields("p", "append", null, ("w", "1"))));
        _rig.AssertError("'w' is not a point axis");

        _rig.Run(StepsRig.Prog([StepsRig.PointList("p", StepsRig.P(0, 0, 0))], Value("p", "append", "5")));
        _rig.AssertError("not a single value");

        _rig.Run(StepsRig.Prog([StepsRig.PointList("p", StepsRig.P(0, 0, 0))], Op("p", "set", s => s.ListIndex = 0)));
        _rig.AssertError("no axes are set");

        _rig.Run(StepsRig.Prog([StepsRig.PointList("p", StepsRig.P(0, 0, 0))], Op("p", "insert", s => s.ListIndex = 0)));
        _rig.AssertError("no axes are set"); // only append means "here"

        _rig.Run(StepsRig.Prog([StepsRig.PointList("p", StepsRig.P(0, 0, 0))], Op("p", "sort")));
        _rig.AssertError("choose the field to sort by");

        _rig.Run(StepsRig.Prog([StepsRig.PointList("p", StepsRig.P(0, 0, 0))], Op("p", "sort", s => s.ListSortField = "w")));
        _rig.AssertError("'w' is not a point axis");
    }

    // ── Record lists ──────────────────────────────────────────────────────────

    [Fact]
    public void RecordFieldsAreFree()
    {
        RunOk([StepsRig.Records("r", StepsRig.Rec(("w", 5), ("ok", 1)))],
            Fields("r", "append", null, ("w", "7"), ("score", "0.5")),
            Fields("r", "set", 0, ("w", "6")),             // set merges: ok stays
            Fields("r", "setField", 1, ("ok", "1")));       // a field the element did not have
        Assert.Equal([6, 7], Field("r", "w"));
        Assert.Equal([1, 1], Field("r", "ok"));
        Assert.Equal([0, 0.5], Field("r", "score"));
    }

    [Fact]
    public void RecordSortIsStableAndMissingFieldsReadZero()
    {
        RunOk([StepsRig.Records("r",
                StepsRig.Rec(("id", 1), ("w", 2)), StepsRig.Rec(("id", 2)), StepsRig.Rec(("id", 3), ("w", 2)), StepsRig.Rec(("id", 4), ("w", -1)))],
            Op("r", "sort", s => s.ListSortField = "w"));
        Assert.Equal([4, 2, 1, 3], Field("r", "id"));
    }

    [Fact]
    public void RecordListMisfits()
    {
        _rig.Run(StepsRig.Prog([StepsRig.Records("r")], Op("r", "append")));
        _rig.AssertError("no fields are set");

        _rig.Run(StepsRig.Prog([StepsRig.Records("r", StepsRig.Rec(("w", 1)))], Op("r", "setField", s => s.ListIndex = 0)));
        _rig.AssertError("no field is set");

        _rig.Run(StepsRig.Prog([StepsRig.Records("r", StepsRig.Rec(("w", 1)))], Fields("r", "append", null, (" ", "1"))));
        _rig.AssertError("a field has no name");
    }

    // ── copyFrom ──────────────────────────────────────────────────────────────

    [Fact]
    public void CopyFromReplacesTheContentsWithACopy()
    {
        RunOk([StepsRig.Numbers("src", 1, 2), StepsRig.Numbers("dst", 9, 9, 9)],
            Op("dst", "copyFrom", s => s.ListSourceVariable = "$src"),
            Value("src", "append", "3"));           // later changes to the source do not reach the copy
        Assert.Equal([1, 2], Values("dst"));
        Assert.Equal([1, 2, 3], Values("src"));
    }

    [Fact]
    public void CopyBetweenCompatibleTypes()
    {
        RunOk([StepsRig.Numbers("n", 0, 4), StepsRig.Booleans("b"), StepsRig.PointList("p", StepsRig.P(1, 2, 3)), StepsRig.Records("r")],
            Op("b", "copyFrom", s => s.ListSourceVariable = "n"),   // numbers → 0/1
            Op("r", "copyFrom", s => s.ListSourceVariable = "p"));  // points → records of x…rz
        Assert.Equal([0, 1], Values("b"));
        Assert.Equal([2], Field("r", "y"));
    }

    [Fact]
    public void CopyMisfits()
    {
        _rig.Run(StepsRig.Prog([StepsRig.Records("r", StepsRig.Rec(("x", 1))), StepsRig.PointList("p")],
            Op("p", "copyFrom", s => s.ListSourceVariable = "r")));
        _rig.AssertError("cannot copy a Record list into '$p'");

        _rig.Run(StepsRig.Prog([StepsRig.PointList("p")], Op("p", "copyFrom", s => s.ListSourceVariable = "nope")));
        _rig.AssertError("'$nope' is not a list variable");

        _rig.Run(StepsRig.Prog([StepsRig.PointList("p")], Op("p", "copyFrom")));
        _rig.AssertError("no list to copy from");
    }

    // ── The list variable itself ──────────────────────────────────────────────

    [Fact]
    public void UnknownOrNonListVariableFails()
    {
        _rig.Run(StepsRig.Prog([StepsRig.Num("x")], Value("x", "append", "1")));
        _rig.AssertError("'$x' is not a list variable");

        _rig.Run(StepsRig.Prog(Value("missing", "append", "1")));
        _rig.AssertError("'$missing' is not a list variable");

        _rig.Run(StepsRig.Prog(Value("", "append", "1")));
        _rig.AssertError("no list variable is selected");
    }

    [Fact]
    public void AnUnknownVariableInAValueStillStopsTheProgram()
    {
        _rig.Run(StepsRig.Prog([StepsRig.Numbers("v")], Value("v", "append", "$typo + 1")));
        _rig.AssertError("Unknown variable '$typo'");
        Assert.Equal(0, _rig.Count("v"));
    }

    [Fact]
    public void TheDeclaredListIsNeverChanged()
    {
        var program = StepsRig.Prog([StepsRig.Records("r", StepsRig.Rec(("w", 1)))],
            Fields("r", "setField", 0, ("w", "50")), Fields("r", "append", null, ("w", "2")));
        _rig.Run(program);
        _rig.AssertComplete();
        Assert.Equal([50, 2], Field("r", "w"));
        // The program definition still declares one element with w = 1, so a re-run starts there.
        Assert.Single(program.Variables[0].Items!);
        Assert.Equal(1, program.Variables[0].Items![0]["w"]);
        _rig.Run(program);
        Assert.Equal([50, 2], Field("r", "w"));
    }

    [Fact]
    public void ListOpsInALoopKeepWorkingOnTheSameList()
    {
        var loop = StepsRig.Step(StepType.Loop, s =>
        {
            s.LoopCount = 5;
            s.ForEachIndexVariableName = "i";
            s.LoopSteps = [Value("v", "insert", "$i", 0)];
        });
        RunOk([StepsRig.Numbers("v"), StepsRig.Num("i")], loop);
        Assert.Equal([4, 3, 2, 1, 0], Values("v"));
    }
}
