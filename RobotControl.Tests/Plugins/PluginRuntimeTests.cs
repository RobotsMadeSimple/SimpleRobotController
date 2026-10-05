using System.Diagnostics;
using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.Plugins;
using Controller.RobotControl.Validation;

namespace RobotControl.Tests.Plugins;

/// <summary>
/// The plugin tests that touch process-wide state (the expression engine's dynamic function
/// provider and parse cache, the shared program run counters) run alone.
/// </summary>
[CollectionDefinition("PluginRuntime", DisableParallelization = true)]
public sealed class PluginRuntimeCollection { }

/// <summary>
/// The plugin system inside the program runtime (docs/plugins.md §5/§6): plugin expression
/// functions and properties, the Plugin step end to end against an in-memory plugin, the
/// validator codes, GetExpressionSymbols, program/step events and variables.get/set.
/// </summary>
[Collection("PluginRuntime")]
public sealed class PluginRuntimeTests : IAsyncLifetime
{
    private PluginTestEnv _env = null!;
    private TestPluginClient _client = null!;
    private string _tmp = null!;
    private readonly List<JsonElement> _executeRequests = new();

    private PluginManager Manager => _env.Manager;

    private static object ScaleManifest() => new Dictionary<string, object?>
    {
        ["id"] = "scale", ["name"] = "Bench Scale", ["version"] = "1.0.0", ["protocolVersion"] = 1, ["runtime"] = "external",
        ["steps"] = new object[]
        {
            new
            {
                id = "weigh", label = "Weigh item",
                @params = new object[]
                {
                    new { key = "samples", label = "Samples", type = "number", @default = 5, required = true },
                    new { key = "unit",    label = "Unit",    type = "enum",   @default = "g", options = new[] { "g", "oz" } },
                    new { key = "label",   label = "Label",   type = "string", @default = "" },
                    new { key = "stable",  label = "Stable",  type = "boolean", @default = true },
                    new { key = "target",  label = "Target",  type = "point" },
                    new { key = "weights", label = "History", type = "list" },
                    new { key = "photo",   label = "Photo",   type = "image" },
                    new { key = "var",     label = "Var",     type = "variable" },
                },
                outputs = new object[]
                {
                    new { key = "grams",  type = "number" },
                    new { key = "ok",     type = "boolean" },
                    new { key = "text",   type = "string" },
                    new { key = "where",  type = "point" },
                    new { key = "series", type = "list" },
                    new { key = "snap",   type = "image" },
                    new { key = "extra",  type = "number" },
                },
            },
            new
            {
                id = "strict", label = "Strict",
                @params = new object[] { new { key = "count", type = "number", required = true } },
                outputs = Array.Empty<object>(),
            },
        },
        ["functions"] = new object[]
        {
            new { name = "twice", signature = "scale.twice(x)", description = "x * 2", minArgs = 1, maxArgs = 1, timeoutMs = 1000 },
            new { name = "fail",  minArgs = 0, maxArgs = 0, timeoutMs = 1000 },
        },
        ["properties"] = new object[] { new { name = "weight", description = "Live weight", type = "number" } },
    };

    private static object IdleManifest() => new Dictionary<string, object?>
    {
        ["id"] = "idle", ["name"] = "Idle", ["version"] = "1.0.0", ["protocolVersion"] = 1, ["runtime"] = "external",
        ["steps"] = new object[] { new { id = "go", label = "Go" } },
        ["functions"] = new object[] { new { name = "f", minArgs = 0, maxArgs = 0 } },
        ["properties"] = new object[] { new { name = "p", type = "number" } },
    };

    public async Task InitializeAsync()
    {
        _env = new PluginTestEnv();
        _tmp = Path.Combine(_env.DataDir, "repos");
        Directory.CreateDirectory(_tmp);
        _env.WritePlugin("scale", ScaleManifest());
        _env.WritePlugin("idle", IdleManifest());
        _env.Boot();
        _client = await _env.ConnectReadyAsync("scale");
        _client.Handlers["function.call"] = p =>
        {
            var name = p.GetProperty("name").GetString();
            if (name == "fail") throw new PluginProtocolException("boom", "scale exploded");
            return new { value = p.GetProperty("args")[0].GetDouble() * 2 };
        };
        _client.Handlers["step.execute"] = p =>
        {
            lock (_executeRequests) _executeRequests.Add(p.Clone());
            return new
            {
                outputs = new Dictionary<string, object>
                {
                    ["grams"] = 12.5, ["ok"] = true, ["text"] = "hello",
                    ["where"] = new { x = 1, y = 2, z = 3, rx = 0, ry = 0, rz = 90 },
                    ["series"] = new[] { 4.0, 5.0 }, ["snap"] = "QUJD",
                },
            };
        };
        await Wait.UntilAsync(() => Manager.Get("scale")!.IsRunning, "scale running");
    }

    public Task DisposeAsync()
    {
        ExpressionEvaluator.SetDynamicFunctions(null);
        _env.Dispose();
        return Task.CompletedTask;
    }

    // ── Harness ──────────────────────────────────────────────────────────────

    private sealed class Rig
    {
        public required RobotController Robot;
        public required ProgramCycleManager Programs;
        public required BackgroundProgramManager Background;
        public required ProgramExecutor Executor;
        public required PointRepository Points;
        public required GridRepository Grids;
    }

    private Rig NewRig()
    {
        var robot = new RobotController { PluginManager = Manager };
        var programs = new ProgramCycleManager();
        var points = new PointRepository(Path.Combine(_tmp, "points.json"), Path.Combine(_tmp, "pointsHistory.json"));
        var grids  = new GridRepository(Path.Combine(_tmp, "grids.json"));
        var stacks = new StackRepository(Path.Combine(_tmp, "stacks.json"));
        var bg = new BackgroundProgramManager(robot, programs, points, robot.toolRepo, robot.localRepo,
            robot.builtProgramRepo, grids, stacks);
        var exec = new ProgramExecutor(robot, programs, points, robot.toolRepo, robot.localRepo, robot.builtProgramRepo,
            grids, stacks, isBackground: false, globalVars: bg.GlobalVars, globalImages: bg.GlobalImages, backgroundManager: bg);
        return new Rig { Robot = robot, Programs = programs, Background = bg, Executor = exec, Points = points, Grids = grids };
    }

    private static void RunToEnd(ProgramExecutor exec, int timeoutMs = 5000)
    {
        var sw = Stopwatch.StartNew();
        while (exec.IsRunning)
        {
            if (sw.ElapsedMilliseconds > timeoutMs) throw new TimeoutException("program did not finish");
            exec.Update();
            Thread.Sleep(1);
        }
    }

    private static JsonElement Summary(ProgramCycleManager programs, string name = "Main") =>
        JsonSerializer.SerializeToElement(programs.GetProgramsSummary())
            .EnumerateArray().First(p => p.GetProperty("name").GetString() == name);

    private static int _ids;
    private static ProgramStep PluginStep(string pluginId, string stepId, Dictionary<string, string>? p = null,
                                          params (string Key, string Var)[] outputs) => new()
    {
        Id = $"p{++_ids}", Type = StepType.Plugin, PluginId = pluginId, PluginStepId = stepId,
        PluginParams = p,
        PluginOutputs = outputs.Select(o => new PluginOutputMapping { Key = o.Key, VariableName = o.Var }).ToList(),
    };

    private static ProgramVariable Num(string name, double v = 0) => new() { Id = name, Name = name, Value = v };
    private static ProgramVariable Str(string name) => new() { Id = name, Name = name, IsString = true, StringValue = "" };
    private static ProgramVariable Img(string name) => new() { Id = name, Name = name, IsImage = true };
    private static ProgramVariable Lst(string name, ListElementType t, params ObjectRecord[] items) =>
        new() { Id = name, Name = name, ElementType = t, Items = items.ToList() };

    private static BuiltProgram Prog(IEnumerable<ProgramStep> steps, params ProgramVariable[] vars) =>
        new() { Id = "main", Name = "Main", Steps = steps.ToList(), Variables = vars.ToList() };

    private static ObjectRecord Pt(double x, double y, double z) =>
        ObjectRecord.FromPoint(new Vector6Val { X = x, Y = y, Z = z });

    // ── Expression functions ─────────────────────────────────────────────────

    [Fact]
    public void DottedCallsParseAndReportThePluginFunction()
    {
        Assert.True(ExpressionEvaluator.TryParse("scale.twice(1) + abs(-2)", out var err), err);
        var refs = ExpressionEvaluator.References("scale.twice($n + 1)");
        var fn = Assert.Single(refs, r => r.Kind == ExprRefKind.Function);
        Assert.Equal("scale.twice", fn.Name);
        Assert.Equal(1, fn.ArgCount);
        Assert.Contains("n", ExpressionEvaluator.ReferencedNames("scale.twice($n + 1)"));
        // still a syntax error: a dotted name that is not called
        Assert.False(ExpressionEvaluator.TryParse("scale.twice + 1", out _));
    }

    [Fact]
    public void DynamicFunctionsResolveThroughTheProviderAndTheCacheIsCleared()
    {
        var vars = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
        ExpressionEvaluator.SetDynamicFunctions(null);
        var ex = Assert.Throws<ExpressionParseException>(() => ExpressionEvaluator.Evaluate("scale.twice(21)", vars));
        Assert.Equal("unknownFunction", ex.Code);
        Assert.False(ExpressionEvaluator.IsFunctionName("scale.twice"));

        ExpressionEvaluator.SetDynamicFunctions(new PluginExpressionFunctions(Manager));
        Assert.True(ExpressionEvaluator.IsFunctionName("scale.twice"));
        Assert.False(ExpressionEvaluator.IsFunctionName("scale")); // ids never clash with dotted names
        Assert.Equal(42, ExpressionEvaluator.Evaluate("scale.twice(21)", vars));
        Assert.Equal(43, ExpressionEvaluator.Evaluate("SCALE.Twice(20) + 3", vars));

        ex = Assert.Throws<ExpressionParseException>(() => ExpressionEvaluator.Evaluate("scale.nope()", vars));
        Assert.Equal("unknownPluginFunction", ex.Code);
        ex = Assert.Throws<ExpressionParseException>(() => ExpressionEvaluator.Evaluate("other.nope()", vars));
        Assert.Equal("unknownFunction", ex.Code);
        ex = Assert.Throws<ExpressionParseException>(() => ExpressionEvaluator.Evaluate("scale.twice(1, 2)", vars));
        Assert.Equal("badArity", ex.Code);

        var fail = Assert.Throws<PluginFunctionCallException>(() => ExpressionEvaluator.Evaluate("scale.fail()", vars));
        Assert.Equal("pluginFunctionFailed", fail.Code);
        Assert.Contains("scale exploded", fail.Message);
        var idle = Assert.Throws<PluginFunctionCallException>(() => ExpressionEvaluator.Evaluate("idle.f()", vars));
        Assert.Equal("pluginNotRunning", idle.Code);
    }

    [Fact]
    public void PluginFunctionFailureIsAProgramErrorWithItsMessage()
    {
        ExpressionEvaluator.SetDynamicFunctions(new PluginExpressionFunctions(Manager));
        var rig = NewRig();
        rig.Executor.Start(Prog(
        [
            new ProgramStep { Id = "a", Type = StepType.SetVariable, VariableName = "n", VariableExpr = "scale.twice(4)" },
            new ProgramStep { Id = "b", Type = StepType.SetVariable, VariableName = "n", VariableExpr = "scale.fail()" },
        ], Num("n")));
        RunToEnd(rig.Executor);
        var s = Summary(rig.Programs);
        Assert.Equal("Error", s.GetProperty("status").GetString());
        Assert.Contains("scale exploded", s.GetProperty("errorDescription").GetString());
        Assert.Equal(8, rig.Executor.PluginSnapshot().Variables["n"]);
    }

    // ── Properties ───────────────────────────────────────────────────────────

    [Fact]
    public async Task PluginPropertiesResolveThroughTheScopeChainAndComputedVariables()
    {
        await _client.EventAsync("properties.set", new { values = new { weight = 12.5 } });
        await Wait.UntilAsync(() => Manager.TryGetProperty("scale.weight", out _), "property set");

        var scope = new VariableScope { Properties = new RobotPropertySource(null, null, Manager.Properties) };
        scope.Initialize(Prog([], new ProgramVariable { Id = "d", Name = "doubled", IsComputed = true, ValueExpression = "$scale.weight * 2" }));
        Assert.Equal(25, ExpressionEvaluator.Evaluate("$scale.weight * 2", scope.EvalVars(), scope.Lists, scope.Properties));
        Assert.True(scope.TryEvaluateComputed("doubled", out var d));
        Assert.Equal(25, d);
        Assert.Throws<UnknownVariableException>(() =>
            ExpressionEvaluator.Evaluate("$scale.missing", scope.EvalVars(), scope.Lists, scope.Properties));

        // Through a controller: RobotPropertySource reads the robot's PluginManager.
        var robot = new RobotController { PluginManager = Manager };
        Assert.True(new RobotPropertySource(robot, null).TryGet("scale.weight", out var w));
        Assert.Equal(12.5, w);
        Assert.True(new RobotPropertySource(robot, null).TryGet("time.hour", out _));
    }

    // ── The Plugin step ──────────────────────────────────────────────────────

    [Fact]
    public void PluginStepResolvesEveryParamTypeAndWritesEveryOutputType()
    {
        var rig = NewRig();
        rig.Points.SavePoint("P1", new Vector6(10, 20, 30, 0, 0, 45));
        rig.Grids.Upsert(new Grid { Id = "g1", Name = "Tray", BasePointName = "P1", RowOffsetX = 5, ColOffsetY = 7, ColCount = 3 });

        var first = PluginStep("scale", "weigh", new()
        {
            ["samples"] = "$n * 2", ["unit"] = "oz", ["label"] = "Bin {$i}", ["stable"] = "false",
            ["target"] = "$pts[1]", ["weights"] = "history", ["var"] = "$weight",
        }, ("grams", "weight"), ("ok", "okFlag"), ("text", "msg"), ("where", "pickPts"), ("series", "seriesList"),
           ("snap", "img"), ("extra", "untouched"));
        var second = PluginStep("scale", "weigh", new()
        {
            ["target"] = "grid:Tray[1, 2]", ["photo"] = "img", ["weights"] = "$pickPts",
        });
        var third = PluginStep("scale", "weigh", new() { ["target"] = "P1" });

        rig.Executor.Start(Prog([first, second, third],
            Num("n", 3), Num("i", 2), Num("weight"), new ProgramVariable { Id = "okFlag", Name = "okFlag", IsBoolean = true },
            Str("msg"), Img("img"), Num("untouched", 7),
            Lst("history", ListElementType.Number, ObjectRecord.FromScalar(1), ObjectRecord.FromScalar(2)),
            Lst("pts", ListElementType.Point, Pt(1, 1, 1), Pt(4, 5, 6)),
            Lst("pickPts", ListElementType.Point), Lst("seriesList", ListElementType.Number)));
        RunToEnd(rig.Executor);

        var s = Summary(rig.Programs);
        Assert.Equal("Complete", s.GetProperty("status").GetString());
        Assert.Equal(3, _executeRequests.Count);

        var p1 = _executeRequests[0];
        Assert.Equal("weigh", p1.GetProperty("stepId").GetString());
        Assert.Equal("Main", p1.GetProperty("programName").GetString());
        Assert.False(p1.GetProperty("isBackground").GetBoolean());
        var prm = p1.GetProperty("params");
        Assert.Equal(6, prm.GetProperty("samples").GetDouble());
        Assert.Equal("oz", prm.GetProperty("unit").GetString());
        Assert.Equal("Bin 2", prm.GetProperty("label").GetString());
        Assert.False(prm.GetProperty("stable").GetBoolean());
        Assert.Equal(5, prm.GetProperty("target").GetProperty("y").GetDouble());
        Assert.Equal([1.0, 2.0], prm.GetProperty("weights").EnumerateArray().Select(e => e.GetDouble()).ToArray());
        Assert.Equal("weight", prm.GetProperty("var").GetString());
        Assert.False(prm.TryGetProperty("photo", out _)); // no text, no default: not sent

        var prm2 = _executeRequests[1].GetProperty("params");
        Assert.Equal(5, prm2.GetProperty("samples").GetDouble());   // manifest default
        Assert.Equal("g", prm2.GetProperty("unit").GetString());
        Assert.True(prm2.GetProperty("stable").GetBoolean());
        Assert.Equal(10 + 5, prm2.GetProperty("target").GetProperty("x").GetDouble()); // row 1
        Assert.Equal(20 + 14, prm2.GetProperty("target").GetProperty("y").GetDouble()); // col 2
        Assert.Equal("QUJD", prm2.GetProperty("photo").GetString()); // written by the first step's output
        Assert.Equal(3, prm2.GetProperty("weights")[0].GetProperty("z").GetDouble()); // points list as objects

        var prm3 = _executeRequests[2].GetProperty("params");
        Assert.Equal(45, prm3.GetProperty("target").GetProperty("rz").GetDouble());

        var snap = rig.Executor.PluginSnapshot();
        Assert.Equal(12.5, snap.Variables["weight"]);
        Assert.Equal(1, snap.Variables["okFlag"]);
        Assert.Equal("hello", snap.Strings["msg"]);
        Assert.Equal(7, snap.Variables["untouched"]); // missing output skipped
        var series = JsonSerializer.SerializeToElement(snap.Lists["seriesList"]);
        Assert.Equal([4.0, 5.0], series.EnumerateArray().Select(e => e.GetDouble()).ToArray());
        var pick = JsonSerializer.SerializeToElement(snap.Lists["pickPts"]);
        Assert.Equal(1, pick.GetArrayLength());
        Assert.Equal(90, pick[0].GetProperty("rz").GetDouble());
    }

    [Fact]
    public void OutputTypeMismatchFailsTheProgram()
    {
        var rig = NewRig();
        rig.Executor.Start(Prog([PluginStep("scale", "weigh", null, ("text", "weight"))], Num("weight")));
        RunToEnd(rig.Executor);
        var s = Summary(rig.Programs);
        Assert.Equal("Error", s.GetProperty("status").GetString());
        Assert.Contains("pluginOutputType", s.GetProperty("errorDescription").GetString());
    }

    [Fact]
    public void RequiredParamWithoutValueAndPluginNotRunningFailTheProgram()
    {
        var rig = NewRig();
        rig.Executor.Start(Prog([PluginStep("scale", "strict")]));
        RunToEnd(rig.Executor);
        Assert.Contains("has no value", Summary(rig.Programs).GetProperty("errorDescription").GetString());

        rig.Executor.Start(Prog([PluginStep("idle", "go")]));
        RunToEnd(rig.Executor);
        var s = Summary(rig.Programs);
        Assert.Equal("Error", s.GetProperty("status").GetString());
        Assert.Equal("Plugin 'idle' is not running", s.GetProperty("errorDescription").GetString());
    }

    [Fact]
    public async Task TimeoutCancelsTheStepAndFailsTheProgram()
    {
        _client.Handlers["step.execute"] = _ => TestPluginClient.NoReply;
        var rig = NewRig();
        var step = PluginStep("scale", "weigh");
        step.PluginTimeoutMs = 50;
        rig.Executor.Start(Prog([step]));
        RunToEnd(rig.Executor);
        var s = Summary(rig.Programs);
        Assert.Equal("Error", s.GetProperty("status").GetString());
        Assert.Equal("Plugin step 'Weigh item' timed out after 50 ms", s.GetProperty("errorDescription").GetString());
        var cancel = await _client.WaitForRequestAsync("step.cancel");
        Assert.Equal("timeout", cancel.GetProperty("params").GetProperty("reason").GetString());
        Assert.Equal(0, Manager.OutstandingSteps);
    }

    [Fact]
    public async Task ReplyAfterStopIsDiscardedAndProgressUpdatesTheMonitor()
    {
        _client.Handlers["step.execute"] = _ => TestPluginClient.NoReply;
        var rig = NewRig();
        rig.Executor.Start(Prog([PluginStep("scale", "weigh", null, ("grams", "weight"))], Num("weight", 1)));
        rig.Executor.Update();
        var exec = await _client.WaitForRequestAsync("step.execute");
        string invocation = exec.GetProperty("params").GetProperty("invocationId").GetString()!;
        Assert.Equal("Bench Scale: Weigh item", Summary(rig.Programs).GetProperty("currentStepDescription").GetString());

        await _client.EventAsync("step.progress", new { invocationId = invocation, message = "settling", percent = 40 });
        var sw = Stopwatch.StartNew();
        while (Summary(rig.Programs).GetProperty("currentStepDescription").GetString() != "Bench Scale: Weigh item: settling (40%)")
        {
            Assert.True(sw.ElapsedMilliseconds < 5000, "progress text never arrived");
            rig.Executor.Update();
            Thread.Sleep(2);
        }
        Assert.Equal("Bench Scale: Weigh item: settling (40%)", rig.Executor.CurrentStepDescription);

        rig.Executor.Stop();
        var cancel = await _client.WaitForRequestAsync("step.cancel");
        Assert.Equal("stopped", cancel.GetProperty("params").GetProperty("reason").GetString());
        Assert.Equal(invocation, cancel.GetProperty("params").GetProperty("invocationId").GetString());

        // The late reply is discarded: nothing is written, even after Continue.
        await _client.SendRawAsync(new { t = "res", id = exec.GetProperty("id").GetString(), ok = true,
                                         result = new { outputs = new { grams = 99 } } });
        await Task.Delay(50);
        _client.Handlers["step.execute"] = _ => new { outputs = new { grams = 5 } };
        rig.Executor.Resume();
        RunToEnd(rig.Executor);
        Assert.Equal("Complete", Summary(rig.Programs).GetProperty("status").GetString());
        Assert.Equal(5, rig.Executor.PluginSnapshot().Variables["weight"]); // re-sent on Continue, 99 never applied
    }

    [Fact]
    public async Task ResetCancelsAnOutstandingStepWithReasonReset()
    {
        _client.Handlers["step.execute"] = _ => TestPluginClient.NoReply;
        var rig = NewRig();
        rig.Executor.Start(Prog([PluginStep("scale", "weigh")]));
        rig.Executor.Update();
        await _client.WaitForRequestAsync("step.execute");
        rig.Executor.Reset();
        var cancel = await _client.WaitForRequestAsync("step.cancel");
        Assert.Equal("reset", cancel.GetProperty("params").GetProperty("reason").GetString());
    }

    // ── Events ───────────────────────────────────────────────────────────────

    [Fact]
    public async Task ProgramAndStepEventsArePublishedInOrder()
    {
        Assert.False(Manager.HasSubscribers("program.started"));
        var sub = await _client.RequestAsync("events.subscribe", new { events = new[] { "program.*", "step.*" } });
        Assert.True(sub.GetProperty("ok").GetBoolean());
        Assert.True(Manager.HasSubscribers("step.completed"));
        Assert.False(Manager.HasSubscribers("io.changed"));

        var rig = NewRig();
        rig.Executor.Start(Prog(
        [
            new ProgramStep { Id = "set", Type = StepType.SetVariable, VariableName = "n", VariableExpr = "1" },
            new ProgramStep { Id = "off", Type = StepType.SetVariable, VariableName = "n", VariableExpr = "2", Enabled = false },
            PluginStep("scale", "weigh"),
        ], Num("n")));
        RunToEnd(rig.Executor);

        var events = new List<(string Name, JsonElement Data)>();
        var sw = Stopwatch.StartNew();
        while (!events.Any(e => e.Name == "program.finished"))
        {
            Assert.True(sw.ElapsedMilliseconds < 5000, "events never arrived: " + string.Join(", ", events.Select(e => e.Name)));
            var el = await _client.WaitForAsync(e => e.GetProperty("t").GetString() == "evt");
            events.Add((el.GetProperty("event").GetString()!, el.GetProperty("data")));
        }
        Assert.Equal(new[] { "program.started", "step.completed", "step.skipped", "step.started", "step.completed", "program.finished" },
                     events.Select(e => e.Name).ToArray());
        var started = events[0].Data;
        Assert.Equal("Main", started.GetProperty("programName").GetString());
        Assert.False(started.GetProperty("isBackground").GetBoolean());
        Assert.True(started.GetProperty("runCount").GetInt32() >= 1);
        Assert.False(started.TryGetProperty("error", out _));
        Assert.Equal("off", events[2].Data.GetProperty("stepId").GetString());
        var plugin = events[3].Data;
        Assert.Equal("Plugin", plugin.GetProperty("stepType").GetString());
        Assert.Equal("Bench Scale: Weigh item", plugin.GetProperty("description").GetString());
        Assert.Equal(2, plugin.GetProperty("stepIndex").GetInt32());
    }

    [Fact]
    public async Task ProgramErrorEventCarriesTheError()
    {
        await _client.RequestAsync("events.subscribe", new { events = new[] { "program.error" } });
        var rig = NewRig();
        rig.Executor.Start(Prog([PluginStep("idle", "go")]));
        RunToEnd(rig.Executor);
        var ev = await _client.WaitForEventAsync("program.error");
        Assert.Equal("Plugin 'idle' is not running", ev.GetProperty("data").GetProperty("error").GetString());
    }

    // ── variables.get / variables.set ────────────────────────────────────────

    [Fact]
    public async Task VariablesGetAndSetGoThroughTheOwningExecutorOrTheGlobals()
    {
        var rig = NewRig();
        Manager.VariablesGetter = n => PluginVariables.Get(n, rig.Executor, rig.Background);
        Manager.VariablesSetter = (n, v) => PluginVariables.Set(n, v, rig.Executor, rig.Background);
        rig.Executor.Start(Prog(
            [new ProgramStep { Id = "w", Type = StepType.Wait, WaitMs = 60_000 }],
            Num("n", 3), Str("msg"), Lst("history", ListElementType.Number, ObjectRecord.FromScalar(1)),
            Lst("pickPts", ListElementType.Point),
            new ProgramVariable { Id = "c", Name = "calc", IsComputed = true, ValueExpression = "$n + 1" }));
        rig.Executor.Update();

        var get = await _client.RequestAsync("variables.get", new { programName = "Main" });
        Assert.True(get.GetProperty("ok").GetBoolean(), get.ToString());
        var r = get.GetProperty("result");
        Assert.Equal(3, r.GetProperty("variables").GetProperty("n").GetDouble());
        Assert.Equal(4, r.GetProperty("variables").GetProperty("calc").GetDouble());
        Assert.Equal(1, r.GetProperty("lists").GetProperty("history")[0].GetDouble());
        Assert.Equal("", r.GetProperty("strings").GetProperty("msg").GetString());

        var set = await _client.RequestAsync("variables.set", new
        {
            programName = "Main",
            values = new Dictionary<string, object>
            {
                ["n"] = 7, ["msg"] = "hi", ["history"] = new[] { 9, 8 },
                ["pickPts"] = new { x = 1, y = 2, z = 3, rx = 0, ry = 0, rz = 0 },
            },
        });
        Assert.True(set.GetProperty("ok").GetBoolean(), set.ToString());
        rig.Executor.Update(); // applied on the loop thread
        var snap = rig.Executor.PluginSnapshot();
        Assert.Equal(7, snap.Variables["n"]);
        Assert.Equal("hi", snap.Strings["msg"]);
        Assert.Equal(2, JsonSerializer.SerializeToElement(snap.Lists["history"]).GetArrayLength());
        Assert.Equal(3, JsonSerializer.SerializeToElement(snap.Lists["pickPts"])[0].GetProperty("z").GetDouble());

        var computed = await _client.RequestAsync("variables.set", new { programName = "Main", values = new { calc = 1 } });
        Assert.Equal("computedVariable", computed.GetProperty("error").GetString());
        var bad = await _client.RequestAsync("variables.set", new { programName = "Main", values = new { n = (object?)null } });
        Assert.Equal("badValue", bad.GetProperty("error").GetString());
        var unknown = await _client.RequestAsync("variables.get", new { programName = "Nope" });
        Assert.Equal("unknownProgram", unknown.GetProperty("error").GetString());
        var unknownSet = await _client.RequestAsync("variables.set", new { programName = "Nope", values = new { n = 1 } });
        Assert.Equal("unknownProgram", unknownSet.GetProperty("error").GetString());

        var globalSet = await _client.RequestAsync("variables.set", new { values = new { g = 5 } });
        Assert.True(globalSet.GetProperty("ok").GetBoolean(), globalSet.ToString());
        Assert.True(rig.Background.GlobalVars.TryGet("g", out var g) && g == 5);
        var globals = await _client.RequestAsync("variables.get", new { });
        Assert.Equal(5, globals.GetProperty("result").GetProperty("variables").GetProperty("g").GetDouble());

        rig.Executor.Reset();
    }

    // ── Validation ───────────────────────────────────────────────────────────

    private ValidationContext VCtx() => new()
    {
        PointExists    = n => n == "P1",
        GridNameExists = n => n == "Tray",
        PluginLookup   = id => ValidationPlugin.From(Manager.Get(id)),
    };

    private static List<ValidationProblem> Validate(BuiltProgram p, ValidationContext ctx) => ProgramValidator.Validate(p, ctx);

    private static void Has(List<ValidationProblem> problems, string code, string severity = ValidationSeverity.Error, string? field = null) =>
        Assert.True(problems.Any(p => p.Code == code && p.Severity == severity && (field == null || p.Field == field)),
            $"expected {severity} {code}{(field != null ? " @" + field : "")}, got: {string.Join(" | ", problems)}");

    [Fact]
    public async Task ValidatorAcceptsAWellFormedPluginProgram()
    {
        await _client.EventAsync("properties.set", new { values = new { weight = 1 } });
        var problems = Validate(Prog(
        [
            PluginStep("scale", "weigh", new() { ["samples"] = "$n * 2", ["unit"] = "oz", ["target"] = "grid:Tray[1, $n]", ["weights"] = "history" },
                ("grams", "weight"), ("ok", "flag"), ("text", "msg"), ("where", "pts")),
            new ProgramStep { Id = "s", Type = StepType.SetVariable, VariableName = "n", VariableExpr = "scale.twice($weight) + $scale.weight" },
        ], Num("n"), Str("msg"), Lst("history", ListElementType.Number), Lst("pts", ListElementType.Point)), VCtx());
        Assert.True(problems.Count == 0, string.Join(" | ", problems));
    }

    [Fact]
    public void ValidatorReportsEveryPluginCode()
    {
        var problems = Validate(Prog(
        [
            PluginStep("nope", "weigh"),
            PluginStep("scale", "nope"),
            PluginStep("idle", "go"),
            PluginStep("scale", "strict"),
            PluginStep("scale", "weigh", new() { ["unit"] = "kg", ["target"] = "grid:Nowhere[0]" }, ("text", "n"), ("where", "n")),
            new ProgramStep { Id = "f1", Type = StepType.SetVariable, VariableName = "n", VariableExpr = "scale.nope()" },
            new ProgramStep { Id = "f2", Type = StepType.SetVariable, VariableName = "n", VariableExpr = "scale.twice(1, 2)" },
            new ProgramStep { Id = "f3", Type = StepType.SetVariable, VariableName = "n", VariableExpr = "idle.f() + $idle.p" },
            new ProgramStep { Id = "f4", Type = StepType.SetVariable, VariableName = "n", VariableExpr = "$scale.nothing + other.fn()" },
        ], Num("n")), VCtx());

        Has(problems, ValidationCodes.UnknownPlugin, field: "pluginId");
        Has(problems, ValidationCodes.UnknownPluginStep, field: "pluginStepId");
        Has(problems, ValidationCodes.PluginNotRunning, ValidationSeverity.Warning, "pluginId");
        Has(problems, ValidationCodes.PluginParamMissing, field: "pluginParams.count");
        Has(problems, ValidationCodes.PluginParamEnum, field: "pluginParams.unit");
        Has(problems, ValidationCodes.UnknownGrid, field: "pluginParams.target");
        Has(problems, ValidationCodes.PluginOutputType, field: "pluginOutputs[0].variableName");
        Has(problems, ValidationCodes.PluginOutputType, field: "pluginOutputs[1].variableName");
        Has(problems, ValidationCodes.UnknownPluginFunction);
        Has(problems, ValidationCodes.BadArity);
        Assert.Equal(2, problems.Count(p => p.Code == ValidationCodes.PluginNotRunning && p.Field == "variableExpr"));
        Has(problems, ValidationCodes.UnknownProperty);
        Has(problems, ValidationCodes.UnknownFunction);

        // Without a plugin lookup nothing plugin-related is reported (cannot check).
        var offline = Validate(Prog([PluginStep("nope", "weigh")]), ValidationContext.Offline());
        Assert.DoesNotContain(offline, p => p.Code == ValidationCodes.UnknownPlugin);
    }

    // ── GetExpressionSymbols ─────────────────────────────────────────────────

    [Fact]
    public async Task GetExpressionSymbolsListsPluginFunctionsPropertiesAndPlugins()
    {
        await _client.EventAsync("properties.set", new { values = new { weight = 3.5 } });
        await Wait.UntilAsync(() => Manager.TryGetProperty("scale.weight", out _), "property set");
        var rig = NewRig();
        var d = new CommandDispatcher();
        new BuiltProgramCommands(rig.Robot, rig.Programs, null, rig.Background).Register(d);
        Assert.True(d.TryGet("GetExpressionSymbols", out var handler));
        var result = await handler(new CommandMessage { Type = "Command", Id = "t", Command = "GetExpressionSymbols",
                                                  Params = JsonSerializer.SerializeToElement(new { }) });
        var r = JsonSerializer.SerializeToElement(result);

        var fn = r.GetProperty("functions").EnumerateArray().First(f => f.GetProperty("name").GetString() == "scale.twice");
        Assert.Equal("scale.twice(x)", fn.GetProperty("signature").GetString());
        Assert.Equal("scale", fn.GetProperty("pluginId").GetString());
        Assert.Contains(r.GetProperty("functions").EnumerateArray(), f => f.GetProperty("name").GetString() == "clamp");

        var prop = r.GetProperty("properties").EnumerateArray().First(p => p.GetProperty("name").GetString() == "scale.weight");
        Assert.Equal("scale", prop.GetProperty("pluginId").GetString());
        Assert.Equal(3.5, prop.GetProperty("value").GetDouble());
        Assert.Equal("number", prop.GetProperty("type").GetString());

        var plugins = r.GetProperty("plugins").EnumerateArray().ToList();
        var scale = plugins.First(p => p.GetProperty("id").GetString() == "scale");
        Assert.Equal("Bench Scale", scale.GetProperty("name").GetString());
        Assert.True(scale.GetProperty("running").GetBoolean());
        var twice = scale.GetProperty("functions").EnumerateArray().First();
        Assert.Equal("twice", twice.GetProperty("name").GetString());
        Assert.Equal("scale.twice", twice.GetProperty("fullName").GetString());
        Assert.Equal(1, twice.GetProperty("minArgs").GetInt32());
        var weight = scale.GetProperty("properties").EnumerateArray().First();
        Assert.Equal("weight", weight.GetProperty("name").GetString());
        Assert.Equal("scale.weight", weight.GetProperty("fullName").GetString());
        var idle = plugins.First(p => p.GetProperty("id").GetString() == "idle");
        Assert.False(idle.GetProperty("running").GetBoolean());
    }
}
