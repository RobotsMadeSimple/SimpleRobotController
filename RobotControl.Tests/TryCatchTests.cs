using System.Diagnostics;
using Controller.RobotControl;
using Controller.RobotControl.Validation;
using static RobotControl.Tests.FlowTestKit;

namespace RobotControl.Tests;

/// <summary>
/// Try / Catch / retry and RaiseError (docs/program-language.md §4.2): <c>ctx.Fail</c> unwinds
/// to the innermost Try that catches the code, at any depth; fatal codes are never caught.
/// </summary>
[Collection("RobotController")]
public sealed class TryCatchTests : IDisposable
{
    private readonly FlowTestKit _k = new();
    public void Dispose() => _k.Dispose();

    private static BuiltProgram P(params ProgramStep[] steps) =>
        Prog([Num("x"), Num("y"), Num("n"), Text("msg"), Text("code")], steps);

    [Fact]
    public void UncaughtRaiseEndsTheProgramWithItsMessage()
    {
        _k.RunToEnd(P(Set("n", "4"), Raise("No part at {$n}"), Set("x", "1")));
        Assert.Equal("Error", _k.Status());
        Assert.Equal("No part at 4", _k.Error());
        Assert.Equal(0, _k.Value("$x"));
    }

    [Fact]
    public void CatchRunsWithTheErrorVariablesThenTheProgramContinues()
    {
        var t = Try([Set("x", "1"), Raise("bad {$x}", "noPart"), Set("x", "2")],
                    [Set("y", "5")],
                    s => { s.ErrorVariableName = "msg"; s.ErrorCodeVariableName = "code"; });
        _k.RunToEnd(P(t, Set("n", "9")));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(1, _k.Value("$x"));
        Assert.Equal(5, _k.Value("$y"));
        Assert.Equal(9, _k.Value("$n"));
        Assert.Equal("bad 1", _k.Text("msg"));
        Assert.Equal("noPart", _k.Text("code"));
    }

    [Fact]
    public void AnEmptyCatchIgnoresTheError()
    {
        _k.RunToEnd(P(Try([Raise("ignored"), Set("x", "1")]), Set("y", "1")));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(0, _k.Value("$x"));
        Assert.Equal(1, _k.Value("$y"));
    }

    [Fact]
    public void RetriesRunTheBodyAgainBeforeCatching()
    {
        // Fails on attempts 1 and 2, passes on 3: two retries are enough, the catch never runs.
        var body = new[] { Set("n", "$n + 1"), If("$n < 3", [Raise("try again")]), Set("x", "1") };
        _k.RunToEnd(P(Try(body, [Set("y", "1")], s => s.TryRetries = 2)));
        Assert.Equal(3, _k.Value("$n"));
        Assert.Equal(1, _k.Value("$x"));
        Assert.Equal(0, _k.Value("$y"));
    }

    [Fact]
    public void RetriesRunOutThenTheCatchRuns()
    {
        var body = new[] { Set("n", "$n + 1"), Raise("always") };
        _k.RunToEnd(P(Try(body, [Set("y", "1")], s => s.TryRetries = 2)));
        Assert.Equal(3, _k.Value("$n")); // 1 attempt + 2 retries
        Assert.Equal(1, _k.Value("$y"));
    }

    [Fact]
    public void RetryWaitsTheDelay()
    {
        var sw = Stopwatch.StartNew();
        _k.RunToEnd(P(Try([Set("n", "$n + 1"), Raise("x")], null, s => { s.TryRetries = 1; s.TryRetryDelayMs = 40; })),
                    maxTicks: 1_000_000);
        Assert.Equal(2, _k.Value("$n"));
        Assert.True(sw.ElapsedMilliseconds >= 35, $"retry did not wait ({sw.ElapsedMilliseconds} ms)");
    }

    [Fact]
    public void EachEntryToATryStartsWithFreshRetries()
    {
        var t = Try([Set("n", "$n + 1"), Raise("x")], [Set("y", "$y + 1")], s => s.TryRetries = 1);
        _k.RunToEnd(P(Loop(2, t)));
        Assert.Equal(4, _k.Value("$n"));
        Assert.Equal(2, _k.Value("$y"));
    }

    [Fact]
    public void ACodeNotInCatchCodesGoesToTheOuterTry()
    {
        var inner = Try([Raise("jammed", "jam")], [Set("x", "1")], s => s.CatchCodes = ["waitTimeout"]);
        var outer = Try([inner, Set("x", "2")], [Set("y", "1")], s => s.ErrorCodeVariableName = "code");
        _k.RunToEnd(P(outer));
        Assert.Equal(0, _k.Value("$x"));
        Assert.Equal(1, _k.Value("$y"));
        Assert.Equal("jam", _k.Text("code"));
    }

    [Fact]
    public void AMatchingCatchCodeCatches()
    {
        _k.RunToEnd(P(Try([Raise("jammed", "jam")], [Set("x", "1")], s => s.CatchCodes = ["other", "jam"])));
        Assert.Equal(1, _k.Value("$x"));
    }

    [Fact]
    public void AFailureInTheCatchGoesOutward()
    {
        var inner = Try([Raise("first")], [Raise("second")]);
        var outer = Try([inner], [Set("x", "1")], s => s.ErrorVariableName = "msg");
        _k.RunToEnd(P(outer));
        Assert.Equal(1, _k.Value("$x"));
        Assert.Equal("second", _k.Text("msg"));
    }

    [Fact]
    public void FatalErrorsAreNeverCaught()
    {
        _k.RunToEnd(P(Try([Set("x", "$typo + 1")], [Set("y", "1")])));
        Assert.Equal("Error", _k.Status());
        Assert.Contains("Unknown variable '$typo'", _k.Error());
        Assert.Equal(0, _k.Value("$y"));
    }

    [Fact]
    public void ARaisedFatalCodeIsNotCaught()
    {
        _k.RunToEnd(P(Try([Raise("hardware", "motionFault")], [Set("y", "1")])));
        Assert.Equal("Error", _k.Status());
        Assert.Equal(0, _k.Value("$y"));
    }

    [Fact]
    public void AFailureInsideARoutineIsCaughtByTheCallersTry()
    {
        var r = _k.Routine("Pick", [Num("result", 0, ParamModes.Out)],
            Set("result", "42"), Raise("no part", "noPart"));
        var call = Call(r, s => s.RoutineOutputs = [new PluginOutputMapping { Key = "result", VariableName = "x" }]);
        _k.RunToEnd(P(Try([call, Set("y", "1")], [Set("n", "1")]), Set("y", "$y + 10")));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(0, _k.Value("$x")); // the routine did not finish: outputs not copied
        Assert.Equal(1, _k.Value("$n"));
        Assert.Equal(10, _k.Value("$y"));
    }

    [Fact]
    public void AWaitTimeoutIsCatchable()
    {
        var wait = Step(StepType.Wait, s =>
        {
            s.WaitMode = "condition"; s.WaitCondition = Cond("0"); s.WaitTimeoutMs = 2; s.WaitTimeoutAction = "error";
        });
        _k.RunToEnd(P(Try([wait, Set("x", "1")], [Set("y", "1")], s => s.ErrorCodeVariableName = "code")), maxTicks: 1_000_000);
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(0, _k.Value("$x"));
        Assert.Equal(1, _k.Value("$y"));
        Assert.Equal("waitTimeout", _k.Text("code"));
    }

    [Fact]
    public void APointTemplateNamingNoPointIsCatchable()
    {
        // Resolution fails before anything is dispatched, so nothing moves.
        var move = Step(StepType.MoveL, s => s.PointNameExpr = "Slot{$n}");
        _k.RunToEnd(P(Try([move], [Set("y", "1")], s => { s.ErrorCodeVariableName = "code"; s.ErrorVariableName = "msg"; })));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal("pointNotFound", _k.Text("code"));
        Assert.Contains("Slot0", _k.Text("msg"));
    }

    [Fact]
    public void AFailedHttpRequestIsCatchable()
    {
        // Port 1 on loopback refuses the connection at once.
        var req = Step(StepType.HttpRequest, s => { s.JsonUrl = "http://127.0.0.1:1/x"; s.JsonTimeoutMs = 3000; });
        _k.Exec.Start(P(Try([req, Set("x", "1")], [Set("y", "1")], s => s.ErrorCodeVariableName = "code")));
        var sw = Stopwatch.StartNew();
        while (_k.Exec.IsRunning && sw.ElapsedMilliseconds < 5000) { _k.Exec.Update(); Thread.Sleep(1); }
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(0, _k.Value("$x"));
        Assert.Equal(1, _k.Value("$y"));
        Assert.Equal("httpFailed", _k.Text("code"));
    }

    [Fact]
    public void AMissingPickedPointIsStillFatal()
    {
        var move = Step(StepType.MoveL, s => s.PointName = "Gone");
        _k.RunToEnd(P(Try([move], [Set("y", "1")])));
        Assert.Equal("Error", _k.Status());
        Assert.Equal(0, _k.Value("$y"));
    }

    [Fact]
    public void ACaughtErrorIsLogged()
    {
        _k.RunToEnd(P(Try([Raise("oops", "e1")], [Set("x", "1")])));
        var (_, _, logs) = _k.Programs.GetProgramLogs("Main", 0, 1000);
        Assert.Contains(logs, l => System.Text.Json.JsonSerializer.Serialize(l).Contains("[Caught e1] oops"));
    }

    // ── Error status: code and failing step ──────────────────────────────────

    private string? ErrorCode() =>
        _k.Summary().GetProperty("errorCode") is { ValueKind: System.Text.Json.JsonValueKind.String } c ? c.GetString() : null;

    private string? CurrentStepId() => _k.Summary().GetProperty("currentStepId").GetString();

    [Fact]
    public void AnUncaughtFailureReportsItsCodeAndStep()
    {
        var wait  = Step(StepType.Wait, s => s.WaitMs = 0);
        var raise = Raise("no part", "noPart");
        _k.RunToEnd(P(wait, raise, Set("x", "1")), maxTicks: 10_000);
        Assert.Equal("Error", _k.Status());
        Assert.Equal("noPart", ErrorCode());
        Assert.Equal(raise.Id, CurrentStepId()); // not the Wait before it
    }

    [Fact]
    public void AFatalErrorReportsItsCodeAndStep()
    {
        var bad = Set("x", "$typo");
        _k.RunToEnd(P(Set("y", "1"), bad));
        Assert.Equal("unknownVariable", ErrorCode());
        Assert.Equal(bad.Id, CurrentStepId());
    }

    [Fact]
    public void AnErrorWithoutACodeStillMarksItsStep()
    {
        var brk = Break(); // outside a loop: a plain error
        _k.RunToEnd(P(Set("y", "1"), brk));
        Assert.Equal("Error", _k.Status());
        Assert.Null(ErrorCode());
        Assert.Equal(brk.Id, CurrentStepId());
    }

    [Fact]
    public void ANewRunClearsTheErrorCode()
    {
        _k.RunToEnd(P(Raise("x", "first")));
        Assert.Equal("first", ErrorCode());
        _k.RunToEnd(P(Set("x", "1")));
        Assert.Equal("Complete", _k.Status());
        Assert.Null(ErrorCode());
    }

    [Fact]
    public void ACaughtFailureSetsNoErrorCode()
    {
        _k.RunToEnd(P(Try([Raise("x", "handled")]), Set("x", "1")));
        Assert.Equal("Complete", _k.Status());
        Assert.Null(ErrorCode());
    }

    // ── Validator ─────────────────────────────────────────────────────────────

    [Fact]
    public void ValidatorChecksTry()
    {
        var t = Try([], null, s =>
        {
            s.TryRetries = -1;
            s.CatchCodes = ["unknownVariable", "not a code"];
            s.ErrorVariableName = "x"; // a number variable
        });
        var problems = ProgramValidator.Validate(P(t), ValidationContext.Offline());
        Assert.Contains(problems, p => p.Code == ValidationCodes.EmptyBranch && p.Field == "trySteps");
        Assert.Contains(problems, p => p.Field == "tryRetries");
        Assert.Contains(problems, p => p.Field == "catchCodes[0]" && p.Severity == ValidationSeverity.Warning);
        Assert.Contains(problems, p => p.Field == "catchCodes[1]" && p.Severity == ValidationSeverity.Error);
        Assert.Contains(problems, p => p.Field == "errorVariableName" && p.Code == ValidationCodes.ParamOutputType);
    }

    [Fact]
    public void ValidatorTreatsUndeclaredErrorVariablesAsText()
    {
        var t = Try([Raise("x")], [Step(StepType.StatusUpdate, s => s.StatusMessage = "failed: {$why} ({$what})")],
                    s => { s.ErrorVariableName = "why"; s.ErrorCodeVariableName = "what"; });
        var problems = ProgramValidator.Validate(P(t), ValidationContext.Offline());
        Assert.DoesNotContain(problems, p => p.Code == ValidationCodes.UnknownVariable);
    }
}
