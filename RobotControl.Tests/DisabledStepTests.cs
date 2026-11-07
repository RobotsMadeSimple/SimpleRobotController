using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.Validation;

namespace RobotControl.Tests;

/// <summary>
/// Step enable/disable (<c>enabled: false</c>, docs/program-language.md §4.1 #1) end to end:
/// the executor skips a disabled step with its whole body (it still counts as a step), the
/// validator reports it once and nothing inside it, and Notes are never counted. Also checks
/// that a block whose behaviour has not landed yet ends the run with a clear error.
/// </summary>
[Collection("RobotController")]
public sealed class DisabledStepTests : IDisposable
{
    private readonly string              _tmp;
    private readonly RobotController     _robot = new();
    private readonly ProgramCycleManager _programs = new();
    private readonly ProgramExecutor     _exec;

    public DisabledStepTests()
    {
        _tmp = Path.Combine(Path.GetTempPath(), "rms-disabled-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(_tmp);
        var points = new PointRepository(Path.Combine(_tmp, "points.json"), Path.Combine(_tmp, "pointsHistory.json"));
        var grids  = new GridRepository(Path.Combine(_tmp, "grids.json"));
        var stacks = new StackRepository(Path.Combine(_tmp, "stacks.json"));
        _exec = new ProgramExecutor(_robot, _programs, points, _robot.toolRepo, _robot.localRepo,
                                    _robot.builtProgramRepo, grids, stacks);
    }

    public void Dispose()
    {
        try { Directory.Delete(_tmp, recursive: true); } catch { /* best effort */ }
    }

    private static int _ids;
    private static ProgramStep Step(StepType type, Action<ProgramStep>? init = null)
    {
        var s = new ProgramStep { Id = $"d{++_ids}", Type = type };
        init?.Invoke(s);
        return s;
    }

    private static ProgramStep Set(string name, string expr, bool? enabled = null) =>
        Step(StepType.SetVariable, s => { s.VariableName = name; s.VariableExpr = expr; s.Enabled = enabled; });

    private static ProgramVariable Num(string name) => new() { Id = name, Name = name };

    private static BuiltProgram Prog(params ProgramStep[] steps) =>
        new() { Id = "main", Name = "Main", Steps = steps.ToList(), Variables = [Num("x"), Num("y")] };

    private void Run(BuiltProgram p, int maxTicks = 50)
    {
        _exec.Start(p);
        for (int i = 0; i < maxTicks && _exec.IsRunning; i++) _exec.Update();
        Assert.False(_exec.IsRunning, "program did not finish");
    }

    private JsonElement Summary() =>
        JsonSerializer.SerializeToElement(_programs.GetProgramsSummary())
            .EnumerateArray().First(p => p.GetProperty("name").GetString() == "Main");

    // ── Executor ──────────────────────────────────────────────────────────────

    [Fact]
    public void DisabledStepIsSkippedButCounted()
    {
        Run(Prog(Set("x", "1"), Set("x", "$x + 10", enabled: false), Set("y", "$x")));
        Assert.Equal(1, _exec.EvaluateLive("$x"));
        Assert.Equal(1, _exec.EvaluateLive("$y"));
        var summary = Summary();
        Assert.Equal("Complete", summary.GetProperty("status").GetString());
        Assert.Equal(3, summary.GetProperty("currentStepNumber").GetInt32());
        Assert.Equal(3, summary.GetProperty("maxStepCount").GetInt32());
    }

    [Fact]
    public void DisabledBlockSkipsItsWholeBody()
    {
        var iff = Step(StepType.IfCondition, s =>
        {
            s.Enabled   = false;
            s.Condition = new ConditionGroup { Items = [new ConditionItem { Id = "c", Expr = "1" }] };
            s.IfSteps   = [Set("x", "99")];
        });
        Run(Prog(iff, Set("y", "5")));
        Assert.Equal(0, _exec.EvaluateLive("$x"));
        Assert.Equal(5, _exec.EvaluateLive("$y"));
        // The disabled If counts once, its body not at all.
        Assert.Equal(2, Summary().GetProperty("maxStepCount").GetInt32());
    }

    [Fact]
    public void NotesAreNotStepsEvenWhenDisabled()
    {
        var note    = Step(StepType.Note, s => { s.Comment = "Setup"; s.NoteStyle = "section"; });
        var offNote = Step(StepType.Note, s => { s.Comment = "old"; s.Enabled = false; });
        Run(Prog(note, Set("x", "2"), offNote));
        Assert.Equal(2, _exec.EvaluateLive("$x"));
        var summary = Summary();
        Assert.Equal("Complete", summary.GetProperty("status").GetString());
        Assert.Equal(1, summary.GetProperty("maxStepCount").GetInt32());
        Assert.Equal(1, summary.GetProperty("currentStepNumber").GetInt32());
    }

    [Fact]
    public void StatusCarriesTheCurrentStepId()
    {
        var last = Set("y", "3");
        Run(Prog(Set("x", "1"), last));
        Assert.Equal(last.Id, Summary().GetProperty("currentStepId").GetString());
        Assert.Equal(JsonValueKind.Null, Summary().GetProperty("pauseReason").ValueKind);
        Assert.Equal(JsonValueKind.Null, Summary().GetProperty("prompt").ValueKind);
    }

    [Fact]
    public void BlockWithoutAHandlerYetEndsTheRunWithAClearError()
    {
        // Every new block but Note is a placeholder until packages B and C land; once one gets
        // its handler this test moves on to the next placeholder, and is deleted with the last.
        var placeholder = new[]
        {
            StepType.LoopControl, StepType.Return, StepType.EndProgram, StepType.RaiseError, StepType.Switch,
            StepType.Try, StepType.SavePosition, StepType.SetSpeedFactor, StepType.ListOp, StepType.Prompt, StepType.MoveC,
        }.FirstOrDefault(t => StepHandlers.TryGet(t, out var h) && h is NotImplementedStep);
        if (placeholder == default) return; // all landed

        Run(Prog(Set("x", "1"), Step(placeholder), Set("x", "2")));
        var summary = Summary();
        Assert.Equal("Error", summary.GetProperty("status").GetString());
        Assert.Contains("not implemented", summary.GetProperty("errorDescription").GetString());
        Assert.Equal(1, _exec.EvaluateLive("$x"));
    }

    // ── Validator ─────────────────────────────────────────────────────────────

    [Fact]
    public void ValidatorWarnsOnceAndIgnoresProblemsInsideADisabledStep()
    {
        var loop = Step(StepType.Loop, s =>
        {
            s.Enabled   = false;
            s.LoopCount = 2;
            s.LoopSteps = [Set("x", "$nope + 1")];
        });
        var problems = ProgramValidator.Validate(Prog(loop, Set("y", "$x")), ValidationContext.Offline());
        var disabled = Assert.Single(problems, p => p.Code == ValidationCodes.DisabledStep);
        Assert.Equal(loop.Id, disabled.StepId);
        Assert.Equal(ValidationSeverity.Warning, disabled.Severity);
        Assert.Equal("enabled", disabled.Field);
        Assert.DoesNotContain(problems, p => p.Code == ValidationCodes.UnknownVariable);
    }

    [Fact]
    public void DisabledGoToDoesNotMakeTheRestUnreachable()
    {
        var label = Step(StepType.Label, s => { s.LabelId = "L"; s.LabelName = "top"; });
        var go = Step(StepType.GoToLabel, s => { s.LabelId = "L"; s.LabelName = "top"; s.Enabled = false; });
        var problems = ProgramValidator.Validate(Prog(label, go, Set("x", "1")), ValidationContext.Offline());
        Assert.DoesNotContain(problems, p => p.Code == ValidationCodes.UnreachableStep);
    }

    [Fact]
    public void ValidatorWalksSwitchAndTryBodies()
    {
        var sw = Step(StepType.Switch, s =>
        {
            s.SwitchExpr    = "$x";
            s.SwitchCases   = [new SwitchCase { Id = "c1", Values = ["1"], Steps = [Set("y", "$missing1")] }];
            s.SwitchDefault = [Set("y", "$missing2")];
        });
        var tr = Step(StepType.Try, s => { s.TrySteps = [Set("y", "$missing3")]; s.CatchSteps = [Set("y", "$missing4")]; });
        var problems = ProgramValidator.Validate(Prog(sw, tr), ValidationContext.Offline());
        var paths = problems.Where(p => p.Code == ValidationCodes.UnknownVariable).Select(p => p.StepPath).ToList();
        Assert.Contains("steps[0].switchCases[0].steps[0]", paths);
        Assert.Contains("steps[0].switchDefault[0]", paths);
        Assert.Contains("steps[1].trySteps[0]", paths);
        Assert.Contains("steps[1].catchSteps[0]", paths);
    }

    // ── Step count ────────────────────────────────────────────────────────────

    [Fact]
    public void CountStepsCoversNewBodiesAndSkipsNotes()
    {
        var steps = new List<ProgramStep>
        {
            Step(StepType.Note),
            Step(StepType.Switch, s => { s.SwitchCases = [new SwitchCase { Steps = [Set("x", "1"), Set("x", "2")] }]; s.SwitchDefault = [Set("x", "3")]; }),
            Step(StepType.Try, s => { s.TrySteps = [Set("x", "1")]; s.CatchSteps = [Set("x", "2")]; }),
            Step(StepType.Try, s => { s.Enabled = false; s.TrySteps = [Set("x", "1")]; }),
            Step(StepType.Loop, s => { s.LoopCount = 3; s.LoopSteps = [Set("x", "1")]; }),
        };
        // switch 1 + 3 body, try 1 + 2 body, disabled try 1, loop 1 (bodies of loops never count)
        Assert.Equal(9, ProgramExecutor.CountSteps(steps));
    }
}
