using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;

namespace RobotControl.Tests;

/// <summary>
/// The three program-editor commands through the dispatcher, the way the app calls them:
/// ValidateBuiltProgram, EvaluateExpression, GetExpressionSymbols. A RobotController is
/// constructed but never started — no devices, threads or motion.
/// </summary>
public class ExpressionCommandsTests
{
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

    private static JsonElement Send(string command, object parameters)
    {
        Assert.True(Dispatcher.Value.TryGet(command, out var handler));
        var msg = new CommandMessage
        {
            Type = "Command", Id = "t", Command = command,
            Params = JsonSerializer.SerializeToElement(parameters),
        };
        var result = handler(msg).GetAwaiter().GetResult();
        return JsonSerializer.SerializeToElement(result);
    }

    [Fact]
    public void EvaluateExpressionReturnsValueAndBooleanFlag()
    {
        var r = Send("EvaluateExpression", new { expression = "max(2, 3) ^ 2 > 8" });
        Assert.True(r.GetProperty("ok").GetBoolean());
        Assert.Equal(1, r.GetProperty("value").GetDouble());
        Assert.True(r.GetProperty("isBoolean").GetBoolean());

        r = Send("EvaluateExpression", new { expression = "$time.hour + $stb.in1 * 0 + $robot.speedOverride * 0" });
        Assert.True(r.GetProperty("ok").GetBoolean(), r.ToString());
        Assert.False(r.GetProperty("isBoolean").GetBoolean());
    }

    [Fact]
    public void EvaluateExpressionReportsErrors()
    {
        var r = Send("EvaluateExpression", new { expression = "1 +* 2" });
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.Equal("expressionSyntax", r.GetProperty("code").GetString());
        Assert.Equal(3, r.GetProperty("position").GetInt32());

        r = Send("EvaluateExpression", new { expression = "$nope + 1", programName = "NotRunning" });
        Assert.False(r.GetProperty("ok").GetBoolean());
        Assert.Contains("nope", r.GetProperty("error").GetString());

        r = Send("EvaluateExpression", new { expression = "sqrt(-1)" });
        Assert.False(r.GetProperty("ok").GetBoolean());
    }

    [Fact]
    public void ValidateBuiltProgramAcceptsAnUnsavedProgramWithUnknownStepTypes()
    {
        var program = new
        {
            id = "tmp", name = "Tmp",
            steps = new object[]
            {
                new { id = "a", type = "Teleport" },
                new { id = "b", type = "SetVariable", variableName = "robot.x", variableExpr = "1" },
                new { id = "c", type = "Loop", loopCount = 1, loopSteps = new object[]
                {
                    new { id = "d", type = "MoveL", pointName = "__no_such_point__", expressions = new { offsetZ = "abs(" } },
                } },
            },
        };
        var r = Send("ValidateBuiltProgram", new { program });
        var problems = r.GetProperty("problems").EnumerateArray().ToList();
        string Code(string id) => string.Join(",", problems.Where(p => p.GetProperty("stepId").GetString() == id)
                                                           .Select(p => p.GetProperty("code").GetString()));
        Assert.Equal("unknownStepType", Code("a"));
        Assert.Equal("readOnlyProperty", Code("b"));
        Assert.Contains("unknownPoint", Code("d"));
        Assert.Contains("expressionSyntax", Code("d"));
        var d = problems.First(p => p.GetProperty("code").GetString() == "expressionSyntax");
        Assert.Equal("steps[2].loopSteps[0]", d.GetProperty("stepPath").GetString());
        Assert.Equal("expressions.offsetZ", d.GetProperty("field").GetString());
        Assert.Equal("error", d.GetProperty("severity").GetString());

        // A JSON string of the program works too.
        var r2 = Send("ValidateBuiltProgram", new { program = JsonSerializer.Serialize(program) });
        Assert.Equal(problems.Count, r2.GetProperty("problems").GetArrayLength());
    }

    [Fact]
    public void GetExpressionSymbolsListsPropertiesFunctionsAndIo()
    {
        var r = Send("GetExpressionSymbols", new { });
        var props = r.GetProperty("properties").EnumerateArray().Select(p => p.GetProperty("name").GetString()).ToList();
        Assert.Contains("robot.x", props);
        Assert.Contains("program.runCount", props);
        Assert.Contains("time.dayOfYear", props);

        var fn = r.GetProperty("functions").EnumerateArray().First(f => f.GetProperty("name").GetString() == "clamp");
        Assert.Equal("clamp(x, lo, hi)", fn.GetProperty("signature").GetString());
        Assert.False(string.IsNullOrEmpty(fn.GetProperty("description").GetString()));

        var io = r.GetProperty("io").EnumerateArray().Select(p => p.GetProperty("name").GetString()).ToList();
        Assert.Contains("stb.in1", io);
        Assert.Contains("relay.4", io);
        Assert.Equal(JsonValueKind.Array, r.GetProperty("variables").ValueKind);
    }
}
