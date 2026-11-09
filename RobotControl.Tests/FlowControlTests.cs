using Controller.RobotControl;
using Controller.RobotControl.Validation;
using static RobotControl.Tests.FlowTestKit;

namespace RobotControl.Tests;

/// <summary>
/// Break / Next iteration, Return, End Program, Switch, the routine boundary for GoToLabel, and
/// the SetOutput toggle / expression value (docs/program-language.md §4.2) — executor and
/// validator.
/// </summary>
[Collection("RobotController")]
public sealed class FlowControlTests : IDisposable
{
    private readonly FlowTestKit _k = new();
    public void Dispose() => _k.Dispose();

    // ── LoopControl ───────────────────────────────────────────────────────────

    [Fact]
    public void BreakLeavesTheInnermostLoop()
    {
        // $i counts passes; break on the 3rd. The step after the loop still runs.
        var loop = Loop(10, Set("i", "$i + 1"), If("$i == 3", [Break()]), Set("x", "$x + 1"));
        _k.RunToEnd(Prog(loop, Set("y", "99")));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(3, _k.Value("$i"));
        Assert.Equal(2, _k.Value("$x")); // the 3rd pass broke before its x += 1
        Assert.Equal(99, _k.Value("$y"));
    }

    [Fact]
    public void ContinueSkipsTheRestOfThePass()
    {
        // Odd passes skip "x += 1": 5 passes → x counts the 2 even ones (i = 2, 4).
        var loop = Loop(5, Set("i", "$i + 1"), If("$i % 2 == 1", [Continue()]), Set("x", "$x + 1"));
        _k.RunToEnd(Prog(loop));
        Assert.Equal(5, _k.Value("$i"));
        Assert.Equal(2, _k.Value("$x"));
    }

    [Fact]
    public void ContinueInAWhileLoopRechecksTheCondition()
    {
        var loop = While("$i < 4", Set("i", "$i + 1"), Continue(), Set("x", "99"));
        _k.RunToEnd(Prog(loop));
        Assert.Equal(4, _k.Value("$i"));
        Assert.Equal(0, _k.Value("$x"));
    }

    [Fact]
    public void BreakOnlyLeavesTheInnerOfTwoLoops()
    {
        var inner = Loop(10, Set("n", "$n + 1"), Break());
        var outer = Loop(3, inner, Set("x", "$x + 1"));
        _k.RunToEnd(Prog(outer));
        Assert.Equal(3, _k.Value("$n"));
        Assert.Equal(3, _k.Value("$x"));
    }

    [Fact]
    public void BreakInsideATryInsideALoopLeavesTheLoop()
    {
        var loop = Loop(5, Set("i", "$i + 1"), Try([Break()]));
        _k.RunToEnd(Prog(loop, Set("y", "1")));
        Assert.Equal(1, _k.Value("$i"));
        Assert.Equal(1, _k.Value("$y"));
    }

    [Fact]
    public void BreakNeverCrossesARoutineBoundary()
    {
        var r = _k.Routine("Inner", [], Break());
        _k.Run(Prog(Loop(3, Call(r))));
        Assert.Equal("Error", _k.Status());
        Assert.Contains("not inside a loop in this routine", _k.Error());
    }

    [Fact]
    public void BreakOutsideAnyLoopIsAnError()
    {
        _k.Run(Prog(Break(), Set("x", "1")));
        Assert.Equal("Error", _k.Status());
        Assert.Equal(0, _k.Value("$x"));
    }

    // ── Return / EndProgram ───────────────────────────────────────────────────

    [Fact]
    public void ReturnLeavesTheRoutineFromInsideALoop()
    {
        var r = _k.Routine("Early", [], Loop(0, Set("n", "$n + 1"), If("$n == 4", [Step(StepType.Return)])));
        _k.RunToEnd(Prog(Call(r), Set("x", "$n * 10")));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(4, _k.Value("$n"));
        Assert.Equal(40, _k.Value("$x"));
    }

    [Fact]
    public void ReturnInTheMainProgramCompletesIt()
    {
        _k.RunToEnd(Prog(Set("x", "1"), Step(StepType.Return), Set("x", "2")));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(1, _k.Value("$x"));
    }

    [Fact]
    public void EndProgramFinishesFromAnyDepthWithItsMessage()
    {
        var r = _k.Routine("Deep", [],
            Loop(5, Set("n", "$n + 1"),
                If("$n == 2", [Step(StepType.EndProgram, s => s.StatusMessage = "Done after {$n}")])));
        _k.RunToEnd(Prog(Call(r), Set("x", "1")));
        var summary = _k.Summary();
        Assert.Equal("Complete", summary.GetProperty("status").GetString());
        Assert.Equal("Done after 2", summary.GetProperty("currentStepDescription").GetString());
        Assert.Equal(0, _k.Value("$x"));
    }

    // ── Switch ────────────────────────────────────────────────────────────────

    private static ProgramStep Switch(string expr, ProgramStep[]? def, params (string[] Values, ProgramStep[] Steps)[] cases) =>
        Step(StepType.Switch, s =>
        {
            s.SwitchExpr    = expr;
            s.SwitchCases   = cases.Select((c, i) => new SwitchCase { Id = $"c{i}", Values = c.Values.ToList(), Steps = c.Steps.ToList() }).ToList();
            s.SwitchDefault = def?.ToList();
        });

    [Theory]
    [InlineData(1, 10)]
    [InlineData(2, 20)]
    [InlineData(3, 20)]   // second value of case 2
    [InlineData(7, -1)]   // default
    public void SwitchRunsTheFirstMatchingCaseOrTheDefault(int n, int expected)
    {
        var sw = Switch("$n", [Set("x", "-1")],
            (["1"], [Set("x", "10")]),
            (["2", "1 + 2"], [Set("x", "20")]),
            (["3"], [Set("x", "30")])); // shadowed by case 2's "1 + 2"
        _k.RunToEnd(Prog([Num("x"), Num("n", n)], sw, Set("x", "$x")));
        Assert.Equal(expected, _k.Value("$x"));
    }

    [Fact]
    public void SwitchWithNoMatchAndNoDefaultDoesNothing()
    {
        _k.RunToEnd(Prog([Num("x", 5), Num("n", 9)], Switch("$n", null, (["1"], [Set("x", "1")])), Set("x", "$x + 1")));
        Assert.Equal(6, _k.Value("$x"));
    }

    [Fact]
    public void SwitchComparesWithinTolerance()
    {
        _k.RunToEnd(Prog([Num("x"), Num("n")], Set("n", "0.1 + 0.2"), Switch("$n", null, (["0.3"], [Set("x", "1")]))));
        Assert.Equal(1, _k.Value("$x"));
    }

    // ── GoToLabel boundary ────────────────────────────────────────────────────

    [Fact]
    public void GoToLabelDoesNotLeaveARoutine()
    {
        var r = _k.Routine("Jumper", [], Step(StepType.GoToLabel, s => { s.LabelId = "L"; s.LabelName = "outer"; }));
        _k.Run(Prog(Step(StepType.Label, s => { s.LabelId = "L"; s.LabelName = "outer"; }), Call(r)));
        Assert.Equal("Error", _k.Status());
        Assert.Contains("not found in this routine", _k.Error());
    }

    // ── SetOutput toggle / expression ─────────────────────────────────────────

    private static ProgramStep Out(int n, Action<ProgramStep> init) =>
        Step(StepType.SetOutput, s => { s.OutputCard = "stb"; s.OutputNumber = n; init(s); });

    [Fact]
    public void SetOutputToggleFlipsTheCurrentState()
    {
        _k.Robot.stb.SetOutput(3, false);
        _k.RunToEnd(Prog(Out(3, s => s.OutputAction = "toggle"), Set("x", "$stb.out3"),
                         Out(3, s => s.OutputAction = "toggle"), Set("y", "$stb.out3")));
        Assert.Equal(1, _k.Value("$x"));
        Assert.Equal(0, _k.Value("$y"));
    }

    [Fact]
    public void SetOutputValueCanBeAnExpression()
    {
        _k.Robot.stb.SetOutput(2, false);
        _k.RunToEnd(Prog([Num("x"), Num("n", 5)],
            Out(2, s => { s.OutputValue = false; s.Expressions = new() { ["outputValue"] = "$n > 3" }; }),
            Set("x", "$stb.out2")));
        Assert.Equal(1, _k.Value("$x"));
        _k.Robot.stb.SetOutput(2, false);
    }

    [Fact]
    public void BlockingTogglePulseRevertsToTheStateItStartedFrom()
    {
        _k.Robot.stb.SetOutput(4, true);
        _k.RunToEndTimed(Prog(Out(4, s => { s.OutputAction = "toggle"; s.PulseMs = 1; s.PulseBlocking = true; }),
                              Set("x", "$stb.out4")));
        Assert.Equal(1, _k.Value("$x")); // toggled off for the pulse, then back on
        _k.Robot.stb.SetOutput(4, false);
    }

    // ── Wait error-on-timeout ─────────────────────────────────────────────────

    private static ProgramStep WaitUntil(string cond, int timeoutMs, string? action, string? flag = null) =>
        Step(StepType.Wait, s =>
        {
            s.WaitMode = "condition";
            s.WaitCondition = Cond(cond);
            s.WaitTimeoutMs = timeoutMs;
            s.WaitTimeoutAction = action;
            s.WaitTimeoutVariableName = flag;
        });

    [Fact]
    public void WaitTimeoutContinuesByDefault()
    {
        _k.RunToEndTimed(Prog(WaitUntil("0", 5, null, "y"), Set("x", "1")));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(1, _k.Value("$y"));
        Assert.Equal(1, _k.Value("$x"));
    }

    [Fact]
    public void WaitTimeoutCanFailTheProgram()
    {
        _k.RunToEndTimed(Prog(WaitUntil("$x == 7", 5, "error", "y"), Set("x", "1")));
        Assert.Equal("Error", _k.Status());
        Assert.Contains("timed out after 5 ms waiting for $x == 7", _k.Error());
        Assert.Equal(1, _k.Value("$y"));  // the flag is still written
        Assert.Equal(0, _k.Value("$x"));
    }

    [Fact]
    public void AMetConditionNeverFailsOnTimeout()
    {
        _k.RunToEnd(Prog(WaitUntil("1", 1, "error", "y"), Set("x", "1")), maxTicks: 100_000);
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(0, _k.Value("$y"));
    }

    // ── Validator ─────────────────────────────────────────────────────────────

    private static List<ValidationProblem> Validate(BuiltProgram p, ValidationContext? ctx = null) =>
        ProgramValidator.Validate(p, ctx ?? ValidationContext.Offline());

    [Fact]
    public void ValidatorFlagsBreakOutsideALoop()
    {
        var inLoop = Break();
        var outside = Continue();
        var problems = Validate(Prog(Loop(2, If("1", [inLoop])), outside));
        var p = Assert.Single(problems, x => x.Code == ValidationCodes.LoopControlOutsideLoop);
        Assert.Equal(outside.Id, p.StepId);
        Assert.Equal(ValidationSeverity.Error, p.Severity);
    }

    [Fact]
    public void ValidatorFlagsBreakInARoutineCalledFromALoop()
    {
        var r = _k.Routine("R", [], Break());
        var ctx = new ValidationContext { FindProgram = (id, name) => _k.Repo.GetById(id ?? "") ?? _k.Repo.Get(name ?? "") };
        var problems = Validate(Prog(Loop(2, Call(r))), ctx);
        Assert.Contains(problems, x => x.Code == ValidationCodes.LoopControlOutsideLoop && x.StepPath.Contains("routine(R)"));
    }

    [Fact]
    public void ValidatorChecksSwitch()
    {
        var sw = Switch("$n", null,
            (["1", "2"], [Set("x", "1")]),
            (["2.0"], [Set("x", "2")]),        // duplicate of 2
            ([], [Set("x", "3")]),             // no value
            (["$nope"], []));
        var problems = Validate(Prog(sw));
        var dup = Assert.Single(problems, x => x.Code == ValidationCodes.DuplicateCaseValue);
        Assert.Equal("switchCases[1].values[0]", dup.Field);
        Assert.Equal(ValidationSeverity.Warning, dup.Severity);
        Assert.Contains(problems, x => x.Code == ValidationCodes.MissingField && x.Field == "switchCases[2].values");
        Assert.Contains(problems, x => x.Code == ValidationCodes.UnknownVariable && x.Field == "switchCases[3].values[0]");

        var empty = Validate(Prog(Switch("$n", null, (["1"], []))));
        Assert.Contains(empty, x => x.Code == ValidationCodes.EmptyBranch);
        var noExpr = Validate(Prog(Switch("", [Set("x", "1")])));
        Assert.Contains(noExpr, x => x.Code == ValidationCodes.MissingField && x.Field == "switchExpr");
    }

    [Fact]
    public void ValidatorChecksRaiseAndEnd()
    {
        var problems = Validate(Prog(
            Raise("", "bad code!"),
            Raise("x is {$x}", "motionFault"),
            Step(StepType.EndProgram, s => s.StatusMessage = "bye {$missing}")));
        Assert.Contains(problems, x => x.Code == ValidationCodes.MissingField && x.Field == "errorMessage");
        Assert.Contains(problems, x => x.Field == "errorCode" && x.Severity == ValidationSeverity.Error);
        Assert.Contains(problems, x => x.Field == "errorCode" && x.Severity == ValidationSeverity.Warning && x.Message.Contains("fatal"));
        Assert.Contains(problems, x => x.Code == ValidationCodes.UnknownVariable && x.Field == "statusMessage");
    }

    [Fact]
    public void ValidatorKeepsGoToInsideItsRoutine()
    {
        var r = _k.Routine("J", [], Step(StepType.GoToLabel, s => { s.LabelId = "L"; s.LabelName = "top"; }));
        var ctx = new ValidationContext { FindProgram = (id, name) => _k.Repo.GetById(id ?? "") ?? _k.Repo.Get(name ?? "") };
        var problems = Validate(Prog(Step(StepType.Label, s => { s.LabelId = "L"; s.LabelName = "top"; }), Call(r)), ctx);
        Assert.Contains(problems, x => x.Code == ValidationCodes.UnknownLabel && x.StepPath.Contains("routine(J)"));
    }
}
