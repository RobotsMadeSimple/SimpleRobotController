using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Execution;
using static RobotControl.Tests.StepsRig;

namespace RobotControl.Tests;

/// <summary>
/// The expression tester and helper panel see what running programs see: EvaluateLive uses the
/// scope's string resolvers (text helpers work), GetExpressionSymbols lists the text helpers,
/// and the Prompt step reaches the program status through ProgressReporter.Programs.
/// </summary>
[Collection("RobotController")]
public class ExpressionToolsFollowupTests
{
    [Fact]
    public void EvaluateLiveResolvesTextHelpersOnStringVariables()
    {
        using var rig = new StepsRig();
        rig.Run(Prog([Str("s", "hello 42"), Str("n", " 3.5 ")], Step(StepType.Note, n => n.Comment = "nothing to do")));
        rig.AssertComplete();

        Assert.Equal(8, rig.Live("$s.length"));
        Assert.Equal(6, rig.Live("$s.indexOf(\"42\")"));
        Assert.Equal(-1, rig.Live("$s.indexOf(\"zz\")"));
        Assert.Equal(1, rig.Live("$s.contains(\"lo 4\")"));
        Assert.Equal(1, rig.Live("$s.startsWith(\"hell\")"));
        Assert.Equal(0, rig.Live("$s.endsWith(\"hell\")"));
        Assert.Equal(3.5, rig.Live("$n.toNumber()"));
        Assert.Equal(0, rig.Live("$s.toNumber()"));
        Assert.Equal(4.5, rig.Live("$n.toNumber() + $s.contains(\"42\")"));
    }

    [Fact]
    public void EvaluateLiveMatchesARunningSetVariable()
    {
        // The same expression assigned by a step and evaluated live gives the same value.
        using var rig = new StepsRig();
        rig.Run(Prog([Str("s", "abc,def"), Num("k")], Set("k", "$s.indexOf(\",\") * 10 + $s.length")));
        rig.AssertComplete();
        Assert.Equal(37, rig.Live("$k"));
        Assert.Equal(rig.Live("$k"), rig.Live("$s.indexOf(\",\") * 10 + $s.length"));
    }

    private static readonly Lazy<CommandDispatcher> Dispatcher = new(() =>
    {
        var robot      = new RobotController();
        var programs   = new ProgramCycleManager();
        var background = new BackgroundProgramManager(robot, programs, robot.pointRepo, robot.toolRepo,
            robot.localRepo, robot.builtProgramRepo, robot.gridRepo, robot.stackRepo);
        var d = new CommandDispatcher();
        new BuiltProgramCommands(robot, programs, null, background).Register(d);
        return d;
    });

    [Fact]
    public async Task GetExpressionSymbolsListsTheTextHelpersAfterTheFunctions()
    {
        Assert.True(Dispatcher.Value.TryGet("GetExpressionSymbols", out var handler));
        var msg = new CommandMessage { Type = "Command", Id = "t", Command = "GetExpressionSymbols",
                                       Params = JsonSerializer.SerializeToElement(new { }) };
        var r = JsonSerializer.SerializeToElement(await handler(msg));
        var functions = r.GetProperty("functions").EnumerateArray().ToList();
        var names = functions.Select(f => f.GetProperty("name").GetString()).ToList();

        Assert.Contains("inBox", names);
        Assert.Contains("inCircle", names);
        foreach (var m in ExpressionEvaluator.StringMethods)
        {
            var f = functions.Single(x => x.GetProperty("name").GetString() == "." + m.Name);
            Assert.Equal(m.Signature, f.GetProperty("signature").GetString());
            Assert.Equal(m.Description, f.GetProperty("description").GetString());
            Assert.Equal("textMethod", f.GetProperty("kind").GetString());
        }
        // Built-in functions first, then the text helpers.
        Assert.True(names.IndexOf("inCircle") < names.IndexOf(".contains"));
        Assert.Equal(names.Count, names.Distinct().Count());
    }

    [Fact]
    public void ProgressReporterExposesItsProgramManager()
    {
        var programs = new ProgramCycleManager();
        var reporter = new ProgressReporter(programs, new FrameStack(), new VariableScope());
        Assert.Same(programs, reporter.Programs);
    }
}
