using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Plugins;

namespace RobotControl.Tests.Plugins;

/// <summary>The plugin protocol over the in-memory transport, end to end through PluginManager.</summary>
public class PluginSessionTests : IDisposable
{
    private readonly PluginTestEnv _env = new();

    public PluginSessionTests()
    {
        _env.WritePlugin("demo", PluginTestEnv.Manifest("demo"));
        _env.WritePlugin("other", PluginTestEnv.Manifest("other"));
        _env.Boot();
    }

    public void Dispose() => _env.Dispose();

    private PluginHost Demo => _env.Manager.Get("demo")!;

    [Fact]
    public async Task ReadyHandshakeAcceptsValidTokenOnly()
    {
        Assert.Equal(PluginState.Starting, Demo.State); // external: waiting for connection

        var (bad, _, serveBad) = await _env.ConnectAsync("wrong-token");
        await serveBad;
        Assert.Equal(4401, bad.Transport.PeerCloseCode);

        var (client, reply, _) = await _env.ConnectAsync(Demo.Token, new { sdk = new { name = "test", version = "1" } });
        Assert.True(reply.GetProperty("ok").GetBoolean());
        var result = reply.GetProperty("result");
        Assert.Equal("demo", result.GetProperty("pluginId").GetString());
        Assert.Equal("9.9.9-test", result.GetProperty("controllerVersion").GetString());
        Assert.Equal("COM3", result.GetProperty("config").GetProperty("port").GetString()); // default merged in
        Assert.Equal(_env.Manager.DataDir, result.GetProperty("dataDir").GetString());
        Assert.Equal(PluginState.Running, Demo.State);
        Assert.True(Demo.Connected && Demo.IsRunning);
    }

    [Fact]
    public async Task FirstMessageMustBePluginReady()
    {
        var (controller, plugin) = InMemoryPluginTransport.CreatePair();
        var serve = _env.Manager.AcceptConnectionAsync(controller);
        await plugin.SendAsync(JsonSerializer.Serialize(new { t = "req", id = "1", method = "config.get", @params = new { token = Demo.Token } }), default);
        await serve.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.Equal(4401, controller.CloseCode);
        Assert.False(Demo.Connected);
    }

    [Fact]
    public async Task SecondConnectionReplacesTheFirstWith4409()
    {
        var first = await _env.ConnectReadyAsync("demo");
        var second = await _env.ConnectReadyAsync("demo");
        await first.Closed.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.Equal(4409, first.Transport.PeerCloseCode);
        Assert.Equal(PluginState.Running, Demo.State);
        var r = await second.RequestAsync("plugin.state");
        Assert.Equal("running", r.GetProperty("result").GetProperty("state").GetString());
    }

    [Fact]
    public async Task PluginToControllerRequests()
    {
        var c = await _env.ConnectReadyAsync("demo");

        var cfg = await c.RequestAsync("config.get");
        Assert.Equal(5, cfg.GetProperty("result").GetProperty("config").GetProperty("samples").GetInt32());

        var st = await c.RequestAsync("plugin.state");
        Assert.True(st.GetProperty("result").GetProperty("enabled").GetBoolean());
        Assert.Equal(0, st.GetProperty("result").GetProperty("restartCount").GetInt32());

        var unknown = await c.RequestAsync("does.not.exist");
        Assert.False(unknown.GetProperty("ok").GetBoolean());
        Assert.Equal("unknownMethod", unknown.GetProperty("error").GetString());

        var bad = await c.RequestAsync("controller.command", new { });
        Assert.Equal("badParams", bad.GetProperty("error").GetString());

        CommandMessage? seen = null;
        _env.CommandHandler = m =>
        {
            seen = m;
            return Task.FromResult<object?>(m.Command == "Fail" ? new { ok = false, error = "nope" } : new { value = 5 });
        };
        var ok = await c.RequestAsync("controller.command", new { command = "GetThing", @params = new { a = 1 } });
        Assert.True(ok.GetProperty("ok").GetBoolean());
        Assert.Equal(5, ok.GetProperty("result").GetProperty("value").GetInt32());
        Assert.Equal("GetThing", seen!.Command);
        Assert.Equal(1, seen.Params!.Value.GetProperty("a").GetInt32());

        var fail = await c.RequestAsync("controller.command", new { command = "Fail" });
        Assert.False(fail.GetProperty("ok").GetBoolean());
        Assert.Equal("nope", fail.GetProperty("error").GetString());

        var vars = await c.RequestAsync("variables.get");
        Assert.Equal("unsupported", vars.GetProperty("error").GetString());

        _env.Manager.VariablesGetter = name => name == null
            ? new VariablesSnapshot { Variables = { ["count"] = 3 }, Strings = { ["s"] = "x" } }
            : null;
        vars = await c.RequestAsync("variables.get");
        Assert.Equal(3, vars.GetProperty("result").GetProperty("variables").GetProperty("count").GetDouble());
        vars = await c.RequestAsync("variables.get", new { programName = "Nope" });
        Assert.Equal("unknownProgram", vars.GetProperty("error").GetString());

        _env.Manager.VariablesSetter = (prog, values) => values.ContainsKey("computed") ? "computedVariable" : null;
        var set = await c.RequestAsync("variables.set", new { values = new { count = 4 } });
        Assert.True(set.GetProperty("ok").GetBoolean());
        set = await c.RequestAsync("variables.set", new { values = new { computed = 1 } });
        Assert.Equal("computedVariable", set.GetProperty("error").GetString());
    }

    [Fact]
    public async Task PropertiesSetClearAndDisconnect()
    {
        var c = await _env.ConnectReadyAsync("demo");
        await c.EventAsync("properties.set", new { values = new { weight = 12.5, stable = true, text = "ignored" } });
        await Wait.UntilAsync(() => _env.Manager.TryGetProperty("demo.stable", out _), "properties.set");

        Assert.True(_env.Manager.TryGetProperty("DEMO.Weight", out var w));
        Assert.Equal(12.5, w);
        Assert.True(_env.Manager.TryGetProperty("demo.stable", out var s));
        Assert.Equal(1, s);
        Assert.False(_env.Manager.TryGetProperty("demo.text", out _));

        var listed = _env.Manager.Properties.List().ToList();
        Assert.Contains(listed, p => p.Name == "demo.weight" && p.Description == "Live weight");
        Assert.Contains(listed, p => p.Name == "demo.stable" && p.Description == PluginPropertySource.Undocumented);

        var contrib = _env.Manager.GetContributions();
        Assert.Contains(contrib.Properties, p => p.Property.Name == "weight" && p.Value == 12.5 && p.Running);

        await c.EventAsync("properties.clear", new { });
        await Wait.UntilAsync(() => !_env.Manager.TryGetProperty("demo.weight", out _), "properties.clear");

        await c.EventAsync("properties.set", new { values = new { weight = 1 } });
        await Wait.UntilAsync(() => _env.Manager.TryGetProperty("demo.weight", out _), "set again");
        await c.CloseAsync();
        await Wait.UntilAsync(() => Demo.State == PluginState.Starting, "disconnect");
        Assert.False(_env.Manager.TryGetProperty("demo.weight", out _));
        Assert.Empty(Demo.Properties);
        Assert.Equal(PluginState.Starting, Demo.State); // external plugins wait for a reconnect
    }

    [Fact]
    public async Task LogAndStatusEvents()
    {
        var c = await _env.ConnectReadyAsync("demo");
        await c.EventAsync("log", new { level = "warn", message = "hello from plugin" });
        await c.EventAsync("status", new { state = "degraded", message = "no scale" });
        await Wait.UntilAsync(() => Demo.State == PluginState.Degraded, "degraded");
        Assert.Equal("degraded", Demo.StatusState);
        Assert.Equal("no scale", Demo.StatusMessage);
        Assert.Contains(Demo.Log.Tail(50), l => l.Contains("[warn] hello from plugin"));
        Assert.True(Demo.IsRunning);

        await c.EventAsync("status", new { state = "ok" });
        await Wait.UntilAsync(() => Demo.State == PluginState.Running, "ok again");
        Assert.Null(Demo.StatusMessage);
    }

    [Fact]
    public async Task CallFunctionSuccessTimeoutFailureAndUnknown()
    {
        var c = await _env.ConnectReadyAsync("demo");
        c.Handlers["function.call"] = p =>
        {
            var name = p.GetProperty("name").GetString();
            return name switch
            {
                "twice" => new { value = p.GetProperty("args")[0].GetDouble() * 2 },
                "slow"  => TestPluginClient.NoReply,
                _       => throw new PluginProtocolException("bang", "it broke"),
            };
        };

        Assert.Equal(6, _env.Manager.CallFunction("demo.twice", new double[] { 3 }));

        var timeout = Assert.Throws<PluginFunctionException>(() => _env.Manager.CallFunction("demo.slow", ReadOnlySpan<double>.Empty));
        Assert.Equal("pluginFunctionTimeout", timeout.Code);

        var failed = Assert.Throws<PluginFunctionException>(() => _env.Manager.CallFunction("demo.boom", ReadOnlySpan<double>.Empty));
        Assert.Equal("pluginFunctionFailed", failed.Code);
        Assert.Contains("it broke", failed.Message);

        var unknown = Assert.Throws<PluginFunctionException>(() => _env.Manager.CallFunction("demo.nope", ReadOnlySpan<double>.Empty));
        Assert.Equal("unknownPluginFunction", unknown.Code);

        var notRunning = Assert.Throws<PluginFunctionException>(() => _env.Manager.CallFunction("other.twice", new double[] { 1 }));
        Assert.Equal("pluginNotRunning", notRunning.Code);

        var fns = _env.Manager.Functions;
        var twice = fns.Single(f => f.FullName == "demo.twice");
        Assert.True(twice.Running);
        Assert.Equal(1, twice.MinArgs);
        Assert.Equal(150, twice.TimeoutMs);
        Assert.Equal("demo.twice(…)", twice.Signature);
        Assert.False(fns.Single(f => f.FullName == "other.twice").Running);
        Assert.Equal(250, fns.Single(f => f.FullName == "demo.boom").TimeoutMs);
    }

    [Fact]
    public async Task ExecuteStepRepliesWithOutputsAndReportsProgress()
    {
        var c = await _env.ConnectReadyAsync("demo");
        var progress = new List<(string, string?, double?)>();
        _env.Manager.StepProgress += (id, m, p) => { lock (progress) progress.Add((id, m, p)); };
        var reply = new TaskCompletionSource<PluginStepReply>();

        string inv = _env.Manager.ExecuteStep("demo", "weigh", new PluginStepRequest
        {
            ProgramName = "Main", StepName = "Weigh it", IsBackground = false,
            Params = new() { ["samples"] = 5.0, ["unit"] = "g" },
        }, r => reply.TrySetResult(r));

        var req = await c.WaitForRequestAsync("step.execute");
        var p = req.GetProperty("params");
        Assert.Equal(inv, p.GetProperty("invocationId").GetString());
        Assert.Equal("weigh", p.GetProperty("stepId").GetString());
        Assert.Equal("Main", p.GetProperty("programName").GetString());
        Assert.Equal(5, p.GetProperty("params").GetProperty("samples").GetDouble());
        Assert.False(p.GetProperty("isBackground").GetBoolean());
        Assert.Equal(1, _env.Manager.OutstandingSteps);

        await c.EventAsync("step.progress", new { invocationId = inv, message = "settling", percent = 40 });
        await c.SendRawAsync(new { t = "res", id = req.GetProperty("id").GetString(), ok = true, result = new { outputs = new { grams = 12.5 } } });

        var r = await reply.Task.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.True(r.Ok);
        Assert.Equal(12.5, r.Outputs["grams"].GetDouble());
        Assert.Equal(0, _env.Manager.OutstandingSteps);
        lock (progress) Assert.Contains((inv, "settling", 40.0), progress);
    }

    [Fact]
    public async Task ExecuteStepFailureCancelAndDisconnect()
    {
        Assert.Throws<PluginNotRunningException>(() =>
            _env.Manager.ExecuteStep("other", "weigh", new PluginStepRequest(), _ => { }));

        var c = await _env.ConnectReadyAsync("demo");

        // ok:false → Error/Message from the plugin
        var failed = new TaskCompletionSource<PluginStepReply>();
        _env.Manager.ExecuteStep("demo", "weigh", new PluginStepRequest { ProgramName = "P" }, r => failed.TrySetResult(r));
        var req = await c.WaitForRequestAsync("step.execute");
        await c.SendRawAsync(new { t = "res", id = req.GetProperty("id").GetString(), ok = false, error = "scaleTimeout", message = "scale timeout" });
        var f = await failed.Task.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.False(f.Ok);
        Assert.Equal("scaleTimeout", f.Error);
        Assert.Equal("scale timeout", f.Message);

        // cancel → step.cancel sent, late reply discarded
        int calls = 0;
        string inv = _env.Manager.ExecuteStep("demo", "weigh", new PluginStepRequest { ProgramName = "P" }, _ => Interlocked.Increment(ref calls));
        req = await c.WaitForRequestAsync("step.execute");
        _env.Manager.CancelStep(inv, "stopped");
        var cancel = await c.WaitForRequestAsync("step.cancel");
        Assert.Equal(inv, cancel.GetProperty("params").GetProperty("invocationId").GetString());
        Assert.Equal("stopped", cancel.GetProperty("params").GetProperty("reason").GetString());
        await c.SendRawAsync(new { t = "res", id = req.GetProperty("id").GetString(), ok = true, result = new { outputs = new { } } });
        await c.RequestAsync("plugin.state"); // round trip: the late reply has been processed
        Assert.Equal(0, calls);

        // disconnect while outstanding → pluginDisconnected
        var lost = new TaskCompletionSource<PluginStepReply>();
        _env.Manager.ExecuteStep("demo", "weigh", new PluginStepRequest { ProgramName = "P" }, r => lost.TrySetResult(r));
        await c.WaitForRequestAsync("step.execute");
        await c.CloseAsync();
        var l = await lost.Task.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.False(l.Ok);
        Assert.Equal("pluginDisconnected", l.Error);
    }

    [Fact]
    public async Task SubscriptionGlobsFilterPublishedEvents()
    {
        var c = await _env.ConnectReadyAsync("demo");
        var sub = await c.RequestAsync("events.subscribe", new { events = new[] { "program.*", "plugin.*" } });
        Assert.Equal(2, sub.GetProperty("result").GetProperty("subscribed").GetArrayLength());

        _env.Manager.PublishEvent("step.started", new { programName = "Main" });
        _env.Manager.PublishEvent(PluginEvents.ProgramStarted, new { programName = "Main", isBackground = false, runCount = 1 });
        var e = await c.WaitForAsync(x => x.GetProperty("t").GetString() == "evt");
        Assert.Equal("program.started", e.GetProperty("event").GetString());
        Assert.Equal("Main", e.GetProperty("data").GetProperty("programName").GetString());

        // plugin.started for *other* plugins only
        var other = await _env.ConnectReadyAsync("other");
        var started = await c.WaitForEventAsync("plugin.started");
        Assert.Equal("other", started.GetProperty("data").GetProperty("pluginId").GetString());

        await other.CloseAsync();
        var stopped = await c.WaitForEventAsync("plugin.stopped");
        Assert.Equal("other", stopped.GetProperty("data").GetProperty("pluginId").GetString());

        await c.RequestAsync("events.subscribe", new { events = new[] { "*" } });
        _env.Manager.PublishEvent("step.completed", new { stepIndex = 2 });
        Assert.Equal(2, (await c.WaitForEventAsync("step.completed")).GetProperty("data").GetProperty("stepIndex").GetInt32());

        await c.RequestAsync("events.unsubscribe");
        _env.Manager.PublishEvent("step.completed", new { stepIndex = 3 });
        await c.RequestAsync("plugin.state");
        Assert.DoesNotContain(c.Received, x => x.TryGetProperty("data", out var d) && d.TryGetProperty("stepIndex", out var i) && i.GetInt32() == 3);

        Assert.True(PluginEvents.Matches("*", "status"));
        Assert.True(PluginEvents.Matches("robot.*", "robot.position"));
        Assert.False(PluginEvents.Matches("robot.*", "robotx"));
        Assert.True(PluginEvents.Matches("io.changed", "io.changed"));
        Assert.False(PluginEvents.Matches("io.changed", "io.changedX"));
    }

    [Fact]
    public async Task PollSendsPositionStatusAndOnlyChangedIo()
    {
        var c = await _env.ConnectReadyAsync("demo");
        await c.RequestAsync("events.subscribe", new { events = new[] { "robot.*", "io.changed", "status" }, positionIntervalMs = 5, ioIntervalMs = 20 });

        _env.Io["stb.in1"] = 0;
        _env.Io["stb.in2"] = 1;
        _env.Manager.PollOnce();
        var pos = await c.WaitForEventAsync("robot.position");
        Assert.Equal(3, pos.GetProperty("data").GetProperty("z").GetDouble());
        // sent in order: position, status, io
        Assert.Equal("9.9.9-test", (await c.WaitForEventAsync("status")).GetProperty("data").GetProperty("version").GetString());
        var io = await c.WaitForEventAsync("io.changed");
        Assert.Equal(2, io.GetProperty("data").GetProperty("changes").EnumerateObject().Count()); // first poll: full snapshot

        // within the interval: nothing; after it: only the changed key
        _env.Io["stb.in1"] = 1;
        _env.Manager.PollOnce();
        _env.Clock.Advance(20);
        _env.Robot = _env.Robot with { Homed = true };
        _env.Manager.PollOnce();
        io = await c.WaitForEventAsync("io.changed");
        var changes = io.GetProperty("data").GetProperty("changes");
        Assert.Single(changes.EnumerateObject());
        Assert.Equal(1, changes.GetProperty("stb.in1").GetDouble());
        await c.WaitForEventAsync("robot.homed");
    }

    [Fact]
    public void OutboundQueueDropsOldestPeriodicEventAndDisconnectsWhenProgramEventsBackUp()
    {
        // Session without a running send loop: nothing drains, so the queue fills deterministically.
        var (controller, _) = InMemoryPluginTransport.CreatePair();
        var s = new PluginSession(controller, "x");
        for (int i = 0; i < PluginSession.OutboundCapacity; i++)
            Assert.True(s.SendEvent(PluginEvents.RobotPosition, $"{{\"i\":{i}}}"));
        Assert.Equal(0, s.DroppedEvents);

        Assert.True(s.SendEvent(PluginEvents.RobotPosition, "{}"));      // drops the oldest position
        Assert.True(s.SendEvent(PluginEvents.ProgramStarted, "{}"));     // makes room by dropping another
        Assert.Equal(2, s.DroppedEvents);
        Assert.Equal(PluginSession.OutboundCapacity, s.QueuedCount);
        Assert.False(s.IsClosed);

        var (controller2, _) = InMemoryPluginTransport.CreatePair();
        var s2 = new PluginSession(controller2, "y");
        for (int i = 0; i < PluginSession.OutboundCapacity; i++)
            Assert.True(s2.SendEvent(PluginEvents.StepStarted, "{}"));
        Assert.False(s2.SendEvent(PluginEvents.RobotPosition, "{}"));     // periodic: just dropped
        Assert.False(s2.IsClosed);
        Assert.False(s2.SendEvent(PluginEvents.StepCompleted, "{}"));     // never dropped: connection goes
        SpinWait.SpinUntil(() => controller2.CloseCode != null, 2000);
        Assert.True(s2.IsClosed);
        Assert.Equal("slow", s2.CloseReason);
        Assert.Equal(PluginSession.SlowCloseCode, controller2.CloseCode);
    }
}
