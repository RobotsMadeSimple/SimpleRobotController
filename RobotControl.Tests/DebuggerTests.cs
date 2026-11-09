using Controller.RobotControl;
using Controller.RobotControl.Commands;
using static RobotControl.Tests.FlowTestKit;

namespace RobotControl.Tests;

/// <summary>
/// The program debugger (docs/program-language.md §4.2 "Debugging"): breakpoints pause before a
/// step, step mode and StepBuiltProgram run one step at a time, Continue runs on, and Run from
/// step builds the frame stack down the step's path.
/// </summary>
[Collection("RobotController")]
public sealed class DebuggerTests : IDisposable
{
    private readonly FlowTestKit _k = new();
    public void Dispose() => _k.Dispose();

    private static ProgramExecutor.DebugOptions Debug(string? startAt = null, bool step = false, params string[] breakpoints) =>
        new(startAt, breakpoints, step);

    [Fact]
    public void ABreakpointPausesBeforeItsStepAndContinueRunsOn()
    {
        var a = Set("x", "1");
        var b = Set("x", "2");
        var c = Set("y", "3");
        _k.Run(Prog(a, b, c), debug: Debug(breakpoints: b.Id));

        Assert.True(_k.Exec.IsPaused);
        Assert.Equal("Stopped", _k.Status());
        Assert.Equal("breakpoint", _k.PauseReason());
        Assert.Equal(b.Id, _k.PausedAt());
        Assert.Equal(1, _k.Value("$x")); // b has not run

        _k.Exec.Resume();
        _k.Tick();
        Assert.False(_k.Exec.IsRunning);
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(2, _k.Value("$x"));
        Assert.Equal(3, _k.Value("$y"));
        Assert.Null(_k.PauseReason());
    }

    [Fact]
    public void ABreakpointInALoopPausesEveryPass()
    {
        var body = Set("n", "$n + 1");
        _k.Run(Prog(Loop(3, body)), debug: Debug(breakpoints: body.Id));
        int pauses = 0;
        while (_k.Exec.IsPaused)
        {
            pauses++;
            Assert.Equal(pauses - 1, _k.Value("$n"));
            _k.Exec.Resume();
            _k.Tick();
        }
        Assert.Equal(3, pauses);
        Assert.Equal(3, _k.Value("$n"));
    }

    [Fact]
    public void StepModePausesBeforeEveryStepAndStepRunsOne()
    {
        var a = Set("x", "1");
        var b = Set("x", "2");
        var c = Set("x", "3");
        _k.Run(Prog(a, b, c), debug: Debug(step: true));
        Assert.Equal("step", _k.PauseReason());
        Assert.Equal(a.Id, _k.PausedAt());
        Assert.Equal(0, _k.Value("$x"));

        Assert.True(_k.Exec.StepOnce());
        _k.Tick();
        Assert.Equal(b.Id, _k.PausedAt());
        Assert.Equal(1, _k.Value("$x"));

        Assert.True(_k.Exec.StepOnce());
        _k.Tick();
        Assert.Equal(c.Id, _k.PausedAt());
        Assert.Equal(2, _k.Value("$x"));

        // Continue leaves step mode: the rest runs to the end.
        _k.Exec.Resume();
        _k.Tick();
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(3, _k.Value("$x"));
    }

    [Fact]
    public void StepFromABreakpointStopsAtTheNextStep()
    {
        var a = Set("x", "1");
        var b = Set("x", "2");
        var c = Set("x", "3");
        _k.Run(Prog(a, b, c), debug: Debug(breakpoints: a.Id));
        Assert.Equal(a.Id, _k.PausedAt());
        _k.Exec.StepOnce();
        _k.Tick();
        Assert.Equal("step", _k.PauseReason());
        Assert.Equal(b.Id, _k.PausedAt());
    }

    [Fact]
    public void AMultiTickStepIsNotPausedHalfway()
    {
        var wait = Step(StepType.Wait, s => { s.WaitMs = 5; });
        var after = Set("x", "1");
        _k.Run(Prog(wait, after), maxTicks: 100_000, debug: Debug(step: true));
        Assert.Equal(wait.Id, _k.PausedAt());
        _k.Exec.StepOnce();
        _k.Tick(100_000);
        // The wait ran to its end across many ticks and the program paused before the next step.
        Assert.Equal(after.Id, _k.PausedAt());
        Assert.Equal(0, _k.Value("$x"));
    }

    [Fact]
    public void StepModeSkipsNotesAndDisabledSteps()
    {
        var note = Step(StepType.Note, s => s.Comment = "hello");
        var off  = Set("x", "5");
        off.Enabled = false;
        var a = Set("y", "1");
        _k.Run(Prog(note, off, a), debug: Debug(step: true));
        Assert.Equal(a.Id, _k.PausedAt());
    }

    [Fact]
    public void SetBreakpointsReplacesThemMidRun()
    {
        var a = Set("x", "1");
        var b = Set("x", "2");
        var c = Set("x", "3");
        _k.Run(Prog(a, b, c), debug: Debug(breakpoints: a.Id));
        Assert.Equal(a.Id, _k.PausedAt());
        Assert.Equal(1, _k.Exec.SetBreakpoints([c.Id]));
        _k.Exec.Resume();
        _k.Tick();
        Assert.Equal(c.Id, _k.PausedAt());
        Assert.Equal(2, _k.Value("$x"));
    }

    [Fact]
    public void APlainStartClearsTheDebugState()
    {
        var a = Set("x", "1");
        _k.Run(Prog(a), debug: Debug(breakpoints: a.Id));
        Assert.True(_k.Exec.IsPaused);
        _k.Exec.Reset();
        _k.RunToEnd(Prog(a));
        Assert.Equal("Complete", _k.Status());
        Assert.Empty(_k.Exec.Breakpoints);
    }

    [Fact]
    public void ARetriedStepPausesAgain()
    {
        var bump  = Set("n", "$n + 1");
        var raise = If("$n < 2", [Raise("again")]);
        _k.Run(Prog(Try([bump, raise], null, s => s.TryRetries = 1)), debug: Debug(breakpoints: bump.Id));
        Assert.Equal(bump.Id, _k.PausedAt());
        _k.Exec.Resume();
        _k.Tick();
        Assert.Equal(bump.Id, _k.PausedAt()); // the retry runs it again
        Assert.Equal(1, _k.Value("$n"));
        _k.Exec.Resume();
        _k.Tick();
        Assert.Equal("Complete", _k.Status());
    }

    // ── Run from step ─────────────────────────────────────────────────────────

    [Fact]
    public void RunFromATopLevelStepSkipsTheStepsBefore()
    {
        var b = Set("y", "$x + 5");
        _k.RunToEnd(Prog(Set("x", "100"), b, Set("n", "1")), debug: Debug(startAt: b.Id));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(0, _k.Value("$x"));
        Assert.Equal(5, _k.Value("$y"));
        Assert.Equal(1, _k.Value("$n"));
    }

    [Fact]
    public void RunFromInsideALoopStartsAtTheFirstPass()
    {
        var target = Set("n", "$n + 1");
        var loop = Step(StepType.Loop, s =>
        {
            s.LoopCount = 3;
            s.ForEachIndexVariableName = "i";
            s.LoopSteps = [Set("x", "$x + 100"), target, Set("y", "$y + $i")];
        });
        _k.RunToEnd(Prog(loop, Set("x", "$x + 1")), debug: Debug(startAt: target.Id));
        // First pass from the target (no x += 100), then two full passes; then the step after.
        Assert.Equal(3, _k.Value("$n"));
        Assert.Equal(201, _k.Value("$x"));
        Assert.Equal(0 + 1 + 2, _k.Value("$y"));
    }

    [Fact]
    public void RunFromInsideABranchEntersItWithoutEvaluating()
    {
        var target = Set("x", "7");
        var iff = If("$typo == 1", [Set("y", "1"), target], [Set("y", "2")]); // the condition is never evaluated
        _k.RunToEnd(Prog(iff, Set("n", "1")), debug: Debug(startAt: target.Id));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(7, _k.Value("$x"));
        Assert.Equal(0, _k.Value("$y"));
        Assert.Equal(1, _k.Value("$n"));
    }

    [Fact]
    public void RunFromInsideATryKeepsTheTry()
    {
        var target = Raise("boom");
        var t = Try([Set("y", "1"), target], [Set("x", "1")]);
        _k.RunToEnd(Prog(t), debug: Debug(startAt: target.Id));
        Assert.Equal("Complete", _k.Status());
        Assert.Equal(1, _k.Value("$x"));
        Assert.Equal(0, _k.Value("$y"));
    }

    [Fact]
    public void RunFromInsideASwitchCase()
    {
        var target = Set("x", "9");
        var sw = Step(StepType.Switch, s =>
        {
            s.SwitchExpr = "1";
            s.SwitchCases = [new SwitchCase { Id = "a", Values = ["1"], Steps = [Set("y", "1")] },
                             new SwitchCase { Id = "b", Values = ["2"], Steps = [target] }];
        });
        _k.RunToEnd(Prog(sw), debug: Debug(startAt: target.Id));
        Assert.Equal(9, _k.Value("$x"));
        Assert.Equal(0, _k.Value("$y"));
    }

    [Fact]
    public void RunFromAStepThatIsNotThereIsRefused()
    {
        var p = Prog(Set("x", "1"));
        Assert.NotNull(ProgramExecutor.CheckStartAt(p, "nope"));
        var off = Step(StepType.Loop, s => { s.Enabled = false; s.LoopCount = 1; s.LoopSteps = [Set("x", "1")]; });
        Assert.Contains("disabled", ProgramExecutor.CheckStartAt(Prog(off), off.LoopSteps![0].Id));

        _k.Run(p, debug: Debug(startAt: "nope"));
        Assert.Equal("Error", _k.Status());
        Assert.Contains("Cannot run from this step", _k.Error());
        Assert.Equal(0, _k.Value("$x"));
    }

    [Fact]
    public void RunFromAndABreakpointTogether()
    {
        var a = Set("x", "1");
        var b = Set("x", "2");
        var c = Set("x", "3");
        _k.Run(Prog(a, b, c), debug: Debug(b.Id, false, b.Id));
        Assert.Equal(b.Id, _k.PausedAt()); // starts paused before the start step
        Assert.Equal(0, _k.Value("$x"));
    }

    // ── Commands ──────────────────────────────────────────────────────────────

    [Fact]
    public void DebugCommandsDriveTheExecutor()
    {
        var d = new CommandDispatcher();
        new DebugCommands(_k.Exec).Register(d);
        Assert.True(d.TryGet("StepBuiltProgram", out var step));
        Assert.True(d.TryGet("SetBreakpoints", out var setBp));

        // Nothing running: Step fails.
        Assert.Throws<InvalidOperationException>(() => step(new CommandMessage()).GetAwaiter().GetResult());

        var a = Set("x", "1");
        var b = Set("x", "2");
        _k.Run(Prog(a, b), debug: Debug(step: true));
        var msg = new CommandMessage { Params = System.Text.Json.JsonSerializer.SerializeToElement(new { stepIds = new[] { b.Id, "", "z" } }) };
        var res = System.Text.Json.JsonSerializer.SerializeToElement(setBp(msg).GetAwaiter().GetResult());
        Assert.Equal(2, res.GetProperty("count").GetInt32());

        step(new CommandMessage()).GetAwaiter().GetResult();
        _k.Tick();
        Assert.Equal(b.Id, _k.PausedAt());
    }
}
