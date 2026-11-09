using System.Text.Json;

namespace SimpleRobot.PluginSdk.Tests;

/// <summary>Starts a fake controller and a plugin host connected to it.</summary>
internal sealed class Harness : IAsyncDisposable
{
    private readonly CancellationTokenSource _cts = new();
    public FakeController Server { get; private set; } = null!;
    public PluginHost Host { get; private set; } = null!;
    public Task Run { get; private set; } = null!;

    public static async Task<Harness> StartAsync(Action<PluginHost>? configure = null, string token = FakeController.Token, bool run = true)
    {
        var h = new Harness { Server = await FakeController.StartAsync() };
        h.Host = new PluginHost(new PluginHostOptions
        {
            Url = h.Server.Url, Token = token, PluginId = "demo",
            MinReconnectDelay = TimeSpan.FromMilliseconds(50), MaxReconnectDelay = TimeSpan.FromMilliseconds(200),
        });
        configure?.Invoke(h.Host);
        if (run) h.Run = h.Host.RunAsync(h._cts.Token);
        return h;
    }

    public async Task<FakeConnection> ConnectedAsync()
    {
        var c = await Server.NextConnectionAsync();
        Assert.True(await c.Ready.Task.WaitAsync(TimeSpan.FromSeconds(5)));
        return c;
    }

    public async Task ShutdownAsync(FakeConnection c)
    {
        var res = await c.RequestAsync("shutdown", new { reason = "test" });
        Assert.True(res.GetProperty("ok").GetBoolean());
        await Run.WaitAsync(TimeSpan.FromSeconds(5));
    }

    public async ValueTask DisposeAsync()
    {
        _cts.Cancel();
        try { if (Run != null) await Run.WaitAsync(TimeSpan.FromSeconds(3)); } catch { }
        await Server.DisposeAsync();
    }
}

public class PluginHostTests
{
    [Fact]
    public async Task RunAsync_returns_when_the_parent_process_is_gone()
    {
        var psi = OperatingSystem.IsWindows()
            ? new System.Diagnostics.ProcessStartInfo("ping", "-n 2 127.0.0.1")
            : new System.Diagnostics.ProcessStartInfo("sleep", "1");
        psi.RedirectStandardOutput = true;
        psi.CreateNoWindow = true;
        using var parent = System.Diagnostics.Process.Start(psi)!;
        _ = parent.StandardOutput.ReadToEndAsync();
        var host = new PluginHost(new PluginHostOptions
        {
            Url = "ws://127.0.0.1:1/plugin", Token = "t", PluginId = "demo",
            MinReconnectDelay = TimeSpan.FromMilliseconds(50), MaxReconnectDelay = TimeSpan.FromMilliseconds(200),
            ParentPid = parent.Id,
        });
        await host.RunAsync().WaitAsync(TimeSpan.FromSeconds(8));
        Assert.True(parent.WaitForExit(5000));
    }

    private static JsonElement J(string json) => JsonDocument.Parse(json).RootElement.Clone();

    [Fact]
    public async Task Handshake_sends_ready_first_with_token_and_sdk_and_runs_OnReady()
    {
        var gotCtx = new TaskCompletionSource<(string cv, string id, string data, string greeting)>();
        await using var h = await Harness.StartAsync(p => p.OnReady(ctx =>
        {
            gotCtx.TrySetResult((ctx.ControllerVersion, ctx.PluginId, ctx.DataDir, ctx.Config.GetProperty("greeting").GetString()!));
            return Task.CompletedTask;
        }));
        var c = await h.ConnectedAsync();
        Assert.Equal(FakeController.Token, c.ReadyParams.GetProperty("token").GetString());
        Assert.Equal("SimpleRobot.PluginSdk", c.ReadyParams.GetProperty("sdk").GetProperty("name").GetString());
        Assert.False(string.IsNullOrEmpty(c.ReadyParams.GetProperty("sdk").GetProperty("version").GetString()));
        Assert.Equal(("9.9.9", "demo", "/data", "hi"), await gotCtx.Task.WaitAsync(TimeSpan.FromSeconds(5)));
        await h.ShutdownAsync(c);
    }

    [Fact]
    public async Task Manifest_override_is_sent_in_ready()
    {
        var server = await FakeController.StartAsync();
        await using var _ = server;
        var host = new PluginHost(new PluginHostOptions
        {
            Url = server.Url, Token = FakeController.Token, PluginId = "demo",
            ManifestOverride = new { steps = new[] { new { id = "x" } } },
        });
        var run = host.RunAsync();
        var c = await server.NextConnectionAsync();
        await c.Ready.Task.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.Equal("x", c.ReadyParams.GetProperty("manifest").GetProperty("steps")[0].GetProperty("id").GetString());
        await c.RequestAsync("shutdown");
        await run.WaitAsync(TimeSpan.FromSeconds(5));
    }

    [Fact]
    public async Task Bad_token_throws_PluginAuthException()
    {
        await using var h = await Harness.StartAsync(token: "wrong");
        await Assert.ThrowsAsync<PluginAuthException>(() => h.Run.WaitAsync(TimeSpan.FromSeconds(5)));
    }

    [Fact]
    public async Task Step_execute_round_trip_with_typed_params_and_outputs()
    {
        var png = new byte[] { 1, 2, 3, 250, 251 };
        await using var h = await Harness.StartAsync(p => p.Step("weigh", async (ctx, prm) =>
        {
            var step = (IStepContext)ctx;
            Assert.Equal("inv-1", step.InvocationId);
            Assert.Equal("prog", step.ProgramName);
            ctx.Progress("settling", 25);
            await Task.Yield();
            var pt = prm.GetPoint("target");
            return new StepResult
            {
                ["grams"] = prm.GetDouble("samples") * 2,
                ["count"] = prm.GetInt("samples"),
                ["stable"] = prm.GetBool("stable"),
                ["text"] = prm.GetString("label"),
                ["where"] = new PluginPoint(pt.X + 1, pt.Y, pt.Z, pt.RX, pt.RY, pt.RZ),
                ["series"] = prm.GetList<double>("weights").Select(w => w * 10).ToArray(),
                ["snap"] = prm.GetImageBytes("photo")!,
            };
        }));
        var c = await h.ConnectedAsync();

        var res = await c.RequestAsync("step.execute", new
        {
            invocationId = "inv-1", stepId = "weigh", programName = "prog", isBackground = false,
            @params = new
            {
                samples = 5, stable = true, label = "item",
                target = new { x = 1.5, y = 2, z = 3, rx = 4, ry = 5, rz = 6 },
                weights = new[] { 1.0, 2.0, 3.5 },
                photo = Convert.ToBase64String(png),
            },
        });

        Assert.True(res.GetProperty("ok").GetBoolean());
        var o = res.GetProperty("result").GetProperty("outputs");
        Assert.Equal(10, o.GetProperty("grams").GetDouble());
        Assert.Equal(5, o.GetProperty("count").GetInt32());
        Assert.True(o.GetProperty("stable").GetBoolean());
        Assert.Equal("item", o.GetProperty("text").GetString());
        var w = o.GetProperty("where");
        Assert.Equal(2.5, w.GetProperty("x").GetDouble());
        Assert.Equal(6, w.GetProperty("rz").GetDouble());
        Assert.Equal(new[] { 10.0, 20.0, 35.0 }, o.GetProperty("series").EnumerateArray().Select(e => e.GetDouble()).ToArray());
        Assert.Equal(png, Convert.FromBase64String(o.GetProperty("snap").GetString()!));

        var prog = await c.WaitForEventAsync("step.progress");
        Assert.Equal("inv-1", prog.GetProperty("data").GetProperty("invocationId").GetString());
        Assert.Equal("settling", prog.GetProperty("data").GetProperty("message").GetString());
        Assert.Equal(25, prog.GetProperty("data").GetProperty("percent").GetDouble());
        await h.ShutdownAsync(c);
    }

    [Fact]
    public async Task Step_failures_map_to_error_codes()
    {
        await using var h = await Harness.StartAsync(p => p
            .Step("known", (ctx, prm) => throw new StepException("scale timeout", "scaleTimeout"))
            .Step("default", (ctx, prm) => throw new StepException("nope"))
            .Step("crash", (ctx, prm) => throw new InvalidOperationException("kaboom"))
            .Step("badparam", (ctx, prm) => Task.FromResult(new StepResult { ["x"] = prm.GetPoint("missing") })));
        var c = await h.ConnectedAsync();

        async Task<JsonElement> Exec(string id) => await c.RequestAsync("step.execute",
            new { invocationId = "i-" + id, stepId = id, programName = "p", isBackground = false, @params = new { } });

        var r1 = await Exec("known");
        Assert.False(r1.GetProperty("ok").GetBoolean());
        Assert.Equal("scaleTimeout", r1.GetProperty("error").GetString());
        Assert.Equal("scale timeout", r1.GetProperty("message").GetString());

        Assert.Equal("stepFailed", (await Exec("default")).GetProperty("error").GetString());

        var r3 = await Exec("crash");
        Assert.Equal("exception", r3.GetProperty("error").GetString());
        Assert.Equal("kaboom", r3.GetProperty("message").GetString());

        Assert.Equal("badParams", (await Exec("badparam")).GetProperty("error").GetString());
        Assert.Equal("unknownStep", (await Exec("nothere")).GetProperty("error").GetString());
        await h.ShutdownAsync(c);
    }

    [Fact]
    public async Task Function_call_returns_value_and_unknown_function_fails()
    {
        await using var h = await Harness.StartAsync(p => p.Function("toOz", (ctx, a) => Task.FromResult(a[0] / 28.0)));
        var c = await h.ConnectedAsync();
        var ok = await c.RequestAsync("function.call", new { name = "toOz", args = new[] { 56.0 } });
        Assert.Equal(2.0, ok.GetProperty("result").GetProperty("value").GetDouble());
        var bad = await c.RequestAsync("function.call", new { name = "nope", args = Array.Empty<double>() });
        Assert.False(bad.GetProperty("ok").GetBoolean());
        Assert.Equal("unknownFunction", bad.GetProperty("error").GetString());
        var unknown = await c.RequestAsync("does.not.exist");
        Assert.Equal("unknownMethod", unknown.GetProperty("error").GetString());
        await h.ShutdownAsync(c);
    }

    [Fact]
    public async Task Step_cancel_cancels_the_steps_token()
    {
        var started = new TaskCompletionSource();
        string? reason = null;
        await using var h = await Harness.StartAsync(p => p.Step("slow", async (ctx, prm) =>
        {
            var step = (IStepContext)ctx;
            started.SetResult();
            try { await Task.Delay(Timeout.Infinite, step.CancellationToken); }
            finally { reason = step.CancelReason; }
            return new StepResult();
        }));
        var c = await h.ConnectedAsync();

        var exec = c.RequestAsync("step.execute", new { invocationId = "inv-9", stepId = "slow", programName = "p", isBackground = false, @params = new { } });
        await started.Task.WaitAsync(TimeSpan.FromSeconds(5));
        var cancel = await c.RequestAsync("step.cancel", new { invocationId = "inv-9", reason = "stopped" });
        Assert.True(cancel.GetProperty("ok").GetBoolean());

        var res = await exec;
        Assert.False(res.GetProperty("ok").GetBoolean());
        Assert.Equal("cancelled", res.GetProperty("error").GetString());
        Assert.Equal("stopped", reason);
        await h.ShutdownAsync(c);
    }

    [Fact]
    public async Task Config_changed_updates_context_and_calls_handler()
    {
        var seen = new TaskCompletionSource<string>();
        IPluginContext? ctxRef = null;
        await using var h = await Harness.StartAsync(p => p
            .OnReady(ctx => { ctxRef = ctx; return Task.CompletedTask; })
            .OnConfigChanged((ctx, cfg) => { seen.TrySetResult(cfg.GetProperty("greeting").GetString()!); return Task.CompletedTask; }));
        var c = await h.ConnectedAsync();
        var res = await c.RequestAsync("config.changed", new { config = new { greeting = "yo" } });
        Assert.True(res.GetProperty("ok").GetBoolean());
        Assert.Equal("yo", await seen.Task.WaitAsync(TimeSpan.FromSeconds(5)));
        Assert.Equal("yo", ctxRef!.Config.GetProperty("greeting").GetString());
        await h.ShutdownAsync(c);
    }

    [Fact]
    public async Task Events_are_subscribed_automatically_and_dispatched()
    {
        var got = new TaskCompletionSource<(string, string)>();
        var io = new TaskCompletionSource<int>();
        await using var h = await Harness.StartAsync(p => p
            .SubscribeIntervals(positionIntervalMs: 50)
            .On("program.started", (ctx, e) => { got.TrySetResult(("program.started", e.GetProperty("programName").GetString()!)); return Task.CompletedTask; })
            .On("io.changed", (ctx, e) => { io.TrySetResult(e.GetProperty("changes").GetProperty("stb.in1").GetInt32()); return Task.CompletedTask; })
            .On("robot.*", (ctx, e) => Task.CompletedTask));
        var c = await h.ConnectedAsync();

        var sub = await c.WaitForRequestAsync("events.subscribe");
        var pr = sub.GetProperty("params");
        Assert.Equal(new[] { "io.changed", "program.started", "robot.*" }, pr.GetProperty("events").EnumerateArray().Select(e => e.GetString()).ToArray());
        Assert.Equal(50, pr.GetProperty("positionIntervalMs").GetInt32());
        Assert.False(pr.TryGetProperty("ioIntervalMs", out _));

        await c.SendEventAsync("program.started", new { programName = "main", isBackground = false, runCount = 1 });
        await c.SendEventAsync("io.changed", new { changes = new Dictionary<string, int> { ["stb.in1"] = 1 } });
        Assert.Equal(("program.started", "main"), await got.Task.WaitAsync(TimeSpan.FromSeconds(5)));
        Assert.Equal(1, await io.Task.WaitAsync(TimeSpan.FromSeconds(5)));
        await h.ShutdownAsync(c);
    }

    [Fact]
    public async Task Context_commands_properties_status_variables_and_subscribe()
    {
        IPluginContext? ctx = null;
        await using var h = await Harness.StartAsync(p => p.OnReady(x => { ctx = x; return Task.CompletedTask; }));
        var c = await h.ConnectedAsync();
        await Task.Delay(50); // OnReady runs right after the handshake
        Assert.NotNull(ctx);

        var echo = await ctx!.CommandAsync("SetSTBOutput", new { pin = 1, value = true });
        Assert.Equal("SetSTBOutput", echo.GetProperty("echo").GetString());
        var req = await c.WaitForRequestAsync("controller.command");
        Assert.Equal(1, req.GetProperty("params").GetProperty("params").GetProperty("pin").GetInt32());

        var ex = await Assert.ThrowsAsync<CommandException>(() => ctx.CommandAsync("Fail"));
        Assert.Equal("boom", ex.Code);
        Assert.Equal("it broke", ex.Message);

        ctx.SetProperties(new { weight = 12.5, stable = 1 });
        var ps = await c.WaitForEventAsync("properties.set");
        Assert.Equal(12.5, ps.GetProperty("data").GetProperty("values").GetProperty("weight").GetDouble());
        ctx.SetProperties(new Dictionary<string, object> { ["connected"] = true });
        await c.WaitForEventAsync("properties.set", d => d.GetProperty("values").TryGetProperty("connected", out _));
        ctx.ClearProperties();
        await c.WaitForEventAsync("properties.clear");
        ctx.SetStatus("degraded", "no scale");
        var st = await c.WaitForEventAsync("status");
        Assert.Equal("degraded", st.GetProperty("data").GetProperty("state").GetString());
        Assert.Equal("no scale", st.GetProperty("data").GetProperty("message").GetString());
        ctx.Log("careful", LogLevel.Warn);
        var log = await c.WaitForEventAsync("log");
        Assert.Equal("warn", log.GetProperty("data").GetProperty("level").GetString());

        var vars = await ctx.GetVariablesAsync();
        Assert.Equal(1, vars.Variables["a"].GetInt32());
        Assert.Equal("x", vars.Strings["s"].GetString());
        Assert.Equal(2, vars.Lists["l"].GetArrayLength());
        await ctx.SetVariablesAsync(new { a = 2 }, "main");
        var set = await c.WaitForRequestAsync("variables.set");
        Assert.Equal("main", set.GetProperty("params").GetProperty("programName").GetString());

        await ctx.SubscribeAsync(new[] { "status" }, statusIntervalMs: 200);
        var sub = await c.WaitForAsync(f => f.TryGetProperty("method", out var m) && m.GetString() == "events.subscribe");
        Assert.Equal(200, sub.GetProperty("params").GetProperty("statusIntervalMs").GetInt32());

        Assert.Throws<InvalidOperationException>(() => ctx.Progress("outside a step"));
        await h.ShutdownAsync(c);
    }

    [Fact]
    public async Task Background_runs_while_connected_and_is_cancelled_on_shutdown()
    {
        var started = new TaskCompletionSource();
        var stopped = new TaskCompletionSource();
        await using var h = await Harness.StartAsync(p => p.Background(async (ctx, ct) =>
        {
            started.TrySetResult();
            try { await Task.Delay(Timeout.Infinite, ct); }
            finally { stopped.TrySetResult(); }
        }));
        var c = await h.ConnectedAsync();
        await started.Task.WaitAsync(TimeSpan.FromSeconds(5));
        await h.ShutdownAsync(c);
        await stopped.Task.WaitAsync(TimeSpan.FromSeconds(5));
    }

    [Fact]
    public async Task Shutdown_is_acknowledged_and_RunAsync_completes()
    {
        await using var h = await Harness.StartAsync();
        var c = await h.ConnectedAsync();
        var res = await c.RequestAsync("shutdown", new { reason = "stopped" });
        Assert.True(res.GetProperty("ok").GetBoolean());
        await h.Run.WaitAsync(TimeSpan.FromSeconds(5));
        Assert.True(h.Run.IsCompletedSuccessfully);
    }

    [Fact]
    public async Task Reconnects_after_the_server_drops_the_socket()
    {
        int ready = 0;
        await using var h = await Harness.StartAsync(p => p.OnReady(_ => { Interlocked.Increment(ref ready); return Task.CompletedTask; }));
        var first = await h.ConnectedAsync();
        await first.DropAsync();

        var second = await h.ConnectedAsync();
        Assert.NotSame(first, second);
        Assert.Equal(FakeController.Token, second.ReadyParams.GetProperty("token").GetString());
        await h.ShutdownAsync(second);
        Assert.Equal(2, ready);
    }

    [Fact]
    public async Task Cancellation_token_stops_RunAsync()
    {
        var server = await FakeController.StartAsync();
        await using var _ = server;
        using var cts = new CancellationTokenSource();
        var host = new PluginHost(new PluginHostOptions { Url = server.Url, Token = FakeController.Token });
        var run = host.RunAsync(cts.Token);
        var c = await server.NextConnectionAsync();
        await c.Ready.Task.WaitAsync(TimeSpan.FromSeconds(5));
        cts.Cancel();
        await run.WaitAsync(TimeSpan.FromSeconds(5));
    }

    // ---- attribute registration ------------------------------------------------------------

    private sealed class AttrPlugin
    {
        public readonly TaskCompletionSource<string> Event = new();

        [PluginStep("echo")]
        public Task<StepResult> Echo(IPluginContext ctx, StepParams p) => Task.FromResult(new StepResult { ["text"] = p.GetString("text") });

        [PluginFunction("twice")]
        public double Twice(IPluginContext ctx, double[] args) => args[0] * 2;

        [PluginEvent("program.stopped")]
        public Task Stopped(IPluginContext ctx, JsonElement e) { Event.TrySetResult(e.GetProperty("programName").GetString()!); return Task.CompletedTask; }

        [PluginStep("boom")]
        public StepResult Boom(IPluginContext ctx, StepParams p) => throw new StepException("attr boom", "attrCode");
    }

    [Fact]
    public async Task Register_scans_attributed_methods()
    {
        var plugin = new AttrPlugin();
        await using var h = await Harness.StartAsync(p => p.Register(plugin));
        var c = await h.ConnectedAsync();

        var step = await c.RequestAsync("step.execute", new { invocationId = "a", stepId = "echo", programName = "p", isBackground = false, @params = new { text = "hey" } });
        Assert.Equal("hey", step.GetProperty("result").GetProperty("outputs").GetProperty("text").GetString());
        var boom = await c.RequestAsync("step.execute", new { invocationId = "b", stepId = "boom", programName = "p", isBackground = false, @params = new { } });
        Assert.Equal("attrCode", boom.GetProperty("error").GetString());
        var fn = await c.RequestAsync("function.call", new { name = "twice", args = new[] { 4.0 } });
        Assert.Equal(8, fn.GetProperty("result").GetProperty("value").GetDouble());

        var sub = await c.WaitForRequestAsync("events.subscribe");
        Assert.Equal("program.stopped", sub.GetProperty("params").GetProperty("events")[0].GetString());
        await c.SendEventAsync("program.stopped", new { programName = "main" });
        Assert.Equal("main", await plugin.Event.Task.WaitAsync(TimeSpan.FromSeconds(5)));
        await h.ShutdownAsync(c);
    }

    private sealed class BadSignature { [PluginStep("x")] public void Nope(int a) { } }

    [Fact]
    public void Register_rejects_wrong_signatures_and_duplicates()
    {
        var host = new PluginHost(new PluginHostOptions { Url = "ws://x/plugin", Token = "t" });
        Assert.Throws<ArgumentException>(() => host.Register(new BadSignature()));
        host.Step("dup", (c, p) => Task.FromResult(new StepResult()));
        Assert.Throws<ArgumentException>(() => host.Step("dup", (c, p) => Task.FromResult(new StepResult())));
    }

    // ---- StepParams / StepResult -----------------------------------------------------------------

    [Fact]
    public void StepParams_typed_getters_and_defaults()
    {
        var p = new StepParams(J("""
            { "n": 3.6, "b": true, "s": "text", "numStr": "2.5", "zero": 0, "nul": null,
              "pt": [1,2,3,4,5,6], "ptObj": {"x":1,"y":2,"z":3,"rx":4,"ry":5,"rz":6},
              "list": [{"x":1,"y":2,"z":3,"rx":0,"ry":0,"rz":0}], "img": "data:image/jpeg;base64,AQID" }
            """));
        Assert.Equal(3.6, p.GetDouble("n"));
        Assert.Equal(4, p.GetInt("n"));
        Assert.True(p.GetBool("b"));
        Assert.False(p.GetBool("zero", true));
        Assert.Equal(2.5, p.GetDouble("numStr"));
        Assert.Equal(7, p.GetInt("missing", 7));
        Assert.Equal(9, p.GetDouble("nul", 9));
        Assert.Equal("text", p.GetString("s"));
        Assert.Equal("dflt", p.GetString("missing", "dflt"));
        Assert.Equal(new PluginPoint(1, 2, 3, 4, 5, 6), p.GetPoint("pt"));
        Assert.Equal(new PluginPoint(1, 2, 3, 4, 5, 6), p.GetPoint("ptObj"));
        Assert.Single(p.GetList<PluginPoint>("list"));
        Assert.Empty(p.GetList<double>("missing"));
        Assert.Equal(new byte[] { 1, 2, 3 }, p.GetImageBytes("img"));
        Assert.Null(p.GetImageBytes("missing"));
        Assert.False(p.TryGetPoint("missing", out _));
        Assert.Equal(JsonValueKind.Object, p.Raw.ValueKind);
        var ex = Assert.Throws<StepException>(() => p.GetDouble("s"));
        Assert.Equal("badParams", ex.Code);
    }

    [Fact]
    public void StepResult_accepts_supported_values_only()
    {
        var r = new StepResult { ["a"] = 1, ["b"] = true, ["c"] = "s", ["d"] = new PluginPoint(1, 2, 3, 4, 5, 6), ["e"] = new[] { 1.0, 2.0 }, ["f"] = new byte[] { 1 } };
        Assert.Equal(6, r.Count);
        Assert.Throws<ArgumentException>(() => r["x"] = new object());
        Assert.Throws<ArgumentException>(() => r["y"] = new[] { new object() });
        var json = JsonSerializer.Serialize(r.ToDictionary());
        Assert.Contains("\"f\":\"AQ==\"", json);
        Assert.Contains("\"rx\":4", json);
    }
}
