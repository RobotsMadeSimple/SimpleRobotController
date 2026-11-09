using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

/// <summary>
/// The Prompt step and AnswerPrompt (docs/program-language.md §4.3): what is published in the
/// program status, button / number / text answers into variables, first answer wins
/// (stalePrompt after), bad answers, timeouts (a default button or a promptTimeout failure),
/// Stop and Continue (refused while stopped, re-shown under a fresh id), and a reset run.
/// </summary>
[Collection("RobotController")]
public sealed class PromptTests : IDisposable
{
    private readonly StepsRig _rig = new();
    private readonly CommandDispatcher _commands = new();
    private long _now = 1_000_000;

    public PromptTests()
    {
        new OperatorCommands(_rig.Programs).Register(_commands);
        PromptRegistry.Clock = () => _now;
    }

    public void Dispose()
    {
        PromptRegistry.Clock = () => Environment.TickCount64;
        PromptRegistry.Clear();
        _rig.Dispose();
    }

    private static ProgramStep Prompt(Action<ProgramStep>? init = null) =>
        StepsRig.Step(StepType.Prompt, s => { s.PromptMessage = "Load the tray"; init?.Invoke(s); });

    private ProgramPromptInfo Shown() => Assert.IsType<ProgramPromptInfo>(_rig.Programs.GetPrompt("Main"));

    /// <summary>Sends AnswerPrompt; null on success, else the error text.</summary>
    private string? Answer(string id, int button, string? value = null)
    {
        Assert.True(_commands.TryGet("AnswerPrompt", out var handler));
        var msg = new CommandMessage { Params = JsonSerializer.SerializeToElement(new { promptId = id, button, value }) };
        try { return handler(msg).GetAwaiter().GetResult() == null ? null : "unexpected payload"; }
        catch (InvalidOperationException ex) { return ex.Message; }
    }

    private void Tick(int times = 1) { for (int i = 0; i < times; i++) { _now += 5; _rig.Tick(); } }

    private void FinishRun() { for (int i = 0; i < 50 && _rig.Exec.IsRunning; i++) Tick(); Assert.False(_rig.Exec.IsRunning); }

    // ── Publishing ────────────────────────────────────────────────────────────

    [Fact]
    public void PublishesThePromptInTheStatusAndWaits()
    {
        var step = Prompt(s => { s.PromptTitle = "Setup {$n}"; s.PromptMessage = "Load tray $n"; });
        _rig.Start(StepsRig.Prog([StepsRig.Num("n", 3)], step, StepsRig.Set("after", "1")));
        Tick(3);

        var p = Shown();
        Assert.Equal(32, p.Id.Length);
        Assert.Equal(step.Id, p.StepId);
        Assert.Equal("Main", p.ProgramName);
        Assert.Equal("Setup 3", p.Title);
        Assert.Equal("Load tray 3", p.Message);
        Assert.Equal(["OK"], p.Buttons);
        Assert.Equal("none", p.Input);
        Assert.Null(p.Default);
        Assert.Equal(0, p.TimeoutMs);
        Assert.True(p.ShownUnixMs > 0);
        Assert.True(_rig.Exec.IsRunning);
        Assert.Equal(p.Id, _rig.Summary().GetProperty("prompt").GetProperty("id").GetString());
        Assert.Throws<UnknownVariableException>(() => _rig.Live("$after")); // still waiting
    }

    [Fact]
    public void ButtonsAreTrimmedToOneToFour()
    {
        _rig.Start(StepsRig.Prog(Prompt(s => s.PromptButtons = ["", " Go ", "Skip", "A", "B", "C"])));
        Tick();
        Assert.Equal(["Go", "Skip", "A", "B"], Shown().Buttons);
    }

    [Fact]
    public void AnEmptyMessageAndNoTitleStillShow()
    {
        _rig.Start(StepsRig.Prog(StepsRig.Step(StepType.Prompt)));
        Tick();
        Assert.Equal("", Shown().Message);
        Assert.Null(Shown().Title);
    }

    // ── Answers ───────────────────────────────────────────────────────────────

    [Fact]
    public void AButtonAnswerContinuesAndWritesTheIndex()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Num("choice", -1)],
            Prompt(s => { s.PromptButtons = ["Go", "Skip"]; s.PromptResultVariableName = "$choice"; }),
            StepsRig.Set("after", "$choice + 10")));
        Tick();
        string id = Shown().Id;
        Assert.Null(Answer(id, 1));
        FinishRun();
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$choice"));
        Assert.Equal(11, _rig.Live("$after"));
        Assert.Null(_rig.Programs.GetPrompt("Main"));
        // The first answer won; a later one is stale.
        Assert.StartsWith("stalePrompt", Answer(id, 0));
    }

    [Fact]
    public void ATextResultVariableGetsTheLabel()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Str("label"), StepsRig.Num("isSkip")],
            Prompt(s => { s.PromptButtons = ["Go", "Skip"]; s.PromptResultVariableName = "label"; }),
            StepsRig.Set("isSkip", "$label.contains(\"Skip\") and $label.length == 4")));
        Tick();
        Assert.Null(Answer(Shown().Id, 1));
        FinishRun();
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$isSkip"));
    }

    [Fact]
    public void AnUndeclaredResultVariableIsCreatedAsANumber()
    {
        _rig.Start(StepsRig.Prog(Prompt(s => { s.PromptButtons = ["a", "b", "c"]; s.PromptResultVariableName = "pick"; })));
        Tick();
        Assert.Null(Answer(Shown().Id, 2));
        FinishRun();
        Assert.Equal(2, _rig.Live("$pick"));
    }

    [Fact]
    public void TheSecondOfTwoQuickAnswersIsStale()
    {
        _rig.Start(StepsRig.Prog(Prompt(s => s.PromptButtons = ["a", "b"])));
        Tick();
        var id = Shown().Id;
        Assert.Null(Answer(id, 0));
        Assert.StartsWith("stalePrompt", Answer(id, 1)); // before the program has even consumed the first
    }

    [Theory]
    [InlineData(2)]
    [InlineData(-1)]
    public void ANonexistentButtonIsRefusedAndThePromptStays(int button)
    {
        _rig.Start(StepsRig.Prog(Prompt(s => s.PromptButtons = ["a", "b"])));
        Tick();
        var id = Shown().Id;
        Assert.StartsWith("badButton", Answer(id, button));
        Tick(3);
        Assert.Equal(id, Shown().Id);
        Assert.Null(Answer(id, 1));
    }

    [Fact]
    public void AnUnknownIdIsStale()
    {
        Assert.StartsWith("stalePrompt", Answer("nope", 0));
        Assert.StartsWith("stalePrompt", Answer("", 0));
    }

    // ── Inputs ────────────────────────────────────────────────────────────────

    private ProgramStep NumberPrompt() => Prompt(s =>
    {
        s.PromptInput = "number"; s.PromptDefault = "$n * 2"; s.PromptMin = 1; s.PromptMax = 50;
        s.PromptInputVariableName = "qty"; s.PromptResultVariableName = "button";
    });

    [Fact]
    public void ANumberInputIsPublishedWithItsEvaluatedDefaultAndRange()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Num("n", 5), StepsRig.Num("qty"), StepsRig.Num("button")], NumberPrompt()));
        Tick();
        var p = Shown();
        Assert.Equal("number", p.Input);
        Assert.Equal("10", p.Default);
        Assert.Equal(1, p.Min);
        Assert.Equal(50, p.Max);

        Assert.StartsWith("badValue", Answer(p.Id, 0, "abc"));
        Assert.StartsWith("badValue", Answer(p.Id, 0, "0.5"));   // below the minimum
        Assert.StartsWith("badValue", Answer(p.Id, 0, "51"));    // above the maximum
        Assert.Null(Answer(p.Id, 0, " 7.5 "));
        FinishRun();
        Assert.Equal(7.5, _rig.Live("$qty"));
        Assert.Equal(0, _rig.Live("$button"));
    }

    [Fact]
    public void AMissingNumberUsesTheDefault()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Num("n", 4), StepsRig.Num("qty"), StepsRig.Num("button")], NumberPrompt()));
        Tick();
        Assert.Null(Answer(Shown().Id, 0));
        FinishRun();
        Assert.Equal(8, _rig.Live("$qty"));
    }

    [Fact]
    public void ANumberIsRequiredWhenThereIsNoDefault()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Num("qty")], Prompt(s => { s.PromptInput = "number"; s.PromptInputVariableName = "qty"; })));
        Tick();
        Assert.StartsWith("badValue: a number is required", Answer(Shown().Id, 0));
    }

    [Fact]
    public void ATextInputGoesIntoATextVariable()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Str("who"), StepsRig.Str("name", "Ada"), StepsRig.Num("len"), StepsRig.Num("gr")],
            Prompt(s => { s.PromptInput = "text"; s.PromptDefault = "{$name} L."; s.PromptInputVariableName = "who"; }),
            StepsRig.Set("len", "$who.length"),
            StepsRig.Set("gr", "$who.startsWith(\"Gr\")")));
        Tick();
        var p = Shown();
        Assert.Equal("Ada L.", p.Default);
        Assert.Null(p.Min);
        Assert.Null(Answer(p.Id, 0, "Grace"));
        FinishRun();
        _rig.AssertComplete();
        Assert.Equal(5, _rig.Live("$len"));
        Assert.Equal(1, _rig.Live("$gr"));
    }

    [Fact]
    public void TextIntoANumberVariableIsReadAsANumber()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Num("v", 9)], Prompt(s => { s.PromptInput = "text"; s.PromptInputVariableName = "v"; })));
        Tick();
        Assert.Null(Answer(Shown().Id, 0, "not a number"));
        FinishRun();
        Assert.Equal(0, _rig.Live("$v"));
    }

    [Fact]
    public void AnInputVariableIsIgnoredWithoutAnInput()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Num("v", 9)], Prompt(s => s.PromptInputVariableName = "v")));
        Tick();
        Assert.Null(Answer(Shown().Id, 0, "3"));
        FinishRun();
        Assert.Equal(9, _rig.Live("$v"));
    }

    // ── Timeouts ──────────────────────────────────────────────────────────────

    [Fact]
    public void TimeoutPressesTheTimeoutButton()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Num("choice", -1), StepsRig.Num("qty")],
            Prompt(s =>
            {
                s.PromptButtons = ["Go", "Skip"]; s.PromptTimeoutMs = 200; s.PromptTimeoutButton = 1;
                s.PromptResultVariableName = "choice";
                s.PromptInput = "number"; s.PromptDefault = "3"; s.PromptInputVariableName = "qty";
            })));
        Tick();
        Assert.Equal(200, Shown().TimeoutMs);
        var id = Shown().Id;
        Tick(30); // 150 ms
        Assert.True(_rig.Exec.IsRunning);
        Tick(15); // past 200 ms
        FinishRun();
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$choice"));
        Assert.Equal(3, _rig.Live("$qty"));      // the default stands in for the input
        Assert.StartsWith("stalePrompt", Answer(id, 0));
    }

    [Fact]
    public void TimeoutWithoutAButtonFailsPromptTimeout()
    {
        _rig.Start(StepsRig.Prog(Prompt(s => s.PromptTimeoutMs = 100), StepsRig.Set("after", "1")));
        Tick();
        var id = Shown().Id;
        Tick(30);
        Assert.False(_rig.Exec.IsRunning);
        _rig.AssertError("no answer within 100 ms");
        Assert.Null(_rig.Programs.GetPrompt("Main"));
        Assert.StartsWith("stalePrompt", Answer(id, 0));
    }

    [Fact]
    public void AnOutOfRangeTimeoutButtonFailsInstead()
    {
        _rig.Start(StepsRig.Prog(Prompt(s => { s.PromptTimeoutMs = 50; s.PromptTimeoutButton = 3; })));
        Tick(20);
        _rig.AssertError("no answer within");
    }

    // ── Stop / Continue / reset ───────────────────────────────────────────────

    [Fact]
    public void AStoppedProgramRefusesAnswersAndReShowsOnContinue()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Num("choice", -1)],
            Prompt(s => { s.PromptButtons = ["a", "b"]; s.PromptTimeoutMs = 1000; s.PromptResultVariableName = "choice"; })));
        Tick();
        var first = Shown();
        Tick(100); // 500 ms of the 1000 ms timeout used

        _rig.Exec.Stop();
        Assert.Equal("Stopped", _rig.Status());
        Assert.Equal(first.Id, Shown().Id);      // the status keeps showing it while stopped…
        _now += 5_000;                           // …for a long while
        Assert.StartsWith("programPaused", Answer(first.Id, 1));

        _rig.Exec.Resume();
        Tick();
        var again = Shown();
        Assert.NotEqual(first.Id, again.Id);     // re-shown under a fresh id
        Assert.True(again.ShownUnixMs >= first.ShownUnixMs);
        Assert.StartsWith("stalePrompt", Answer(first.Id, 1));

        Tick(150);                               // 750 ms — the timeout restarted, so still waiting
        Assert.True(_rig.Exec.IsRunning);
        Assert.Null(Answer(again.Id, 1));
        FinishRun();
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$choice"));
    }

    [Fact]
    public void AnAnswerGivenJustBeforeAStopIsKept()
    {
        _rig.Start(StepsRig.Prog([StepsRig.Num("choice", -1)],
            Prompt(s => { s.PromptButtons = ["a", "b"]; s.PromptResultVariableName = "choice"; })));
        Tick();
        Assert.Null(Answer(Shown().Id, 1));
        _rig.Exec.Stop();
        _now += 5_000;
        _rig.Exec.Resume();
        FinishRun();
        Assert.Equal(1, _rig.Live("$choice"));
    }

    [Fact]
    public void AResetRunMakesThePromptStaleAndClearsIt()
    {
        _rig.Start(StepsRig.Prog(Prompt()));
        Tick();
        var id = Shown().Id;
        _rig.Exec.Reset();
        Assert.StartsWith("stalePrompt", Answer(id, 0));
        Assert.Null(_rig.Programs.GetPrompt("Main"));
    }

    [Fact]
    public void APromptInALoopIsShownAfreshEachPass()
    {
        var loop = StepsRig.Step(StepType.Loop, s => { s.LoopCount = 2; s.LoopSteps = [Prompt()]; });
        _rig.Start(StepsRig.Prog(loop));
        Tick(2);
        var first = Shown().Id;
        Assert.Null(Answer(first, 0));
        Tick(3);
        var second = Shown().Id;
        Assert.NotEqual(first, second);
        Assert.Null(Answer(second, 0));
        FinishRun();
        _rig.AssertComplete();
    }

    [Fact]
    public void AProgramWithAPromptValidatesTheStatusPlumbing()
    {
        // The prompt reaches the status through the progress reporter's program manager (read by
        // reflection in PromptStep): if that field is ever renamed this is the test that says so.
        _rig.Start(StepsRig.Prog(Prompt()));
        Tick();
        Assert.NotNull(_rig.Programs.GetPrompt("Main"));
        Assert.Single(_rig.Programs.GetPrompts());
    }
}
