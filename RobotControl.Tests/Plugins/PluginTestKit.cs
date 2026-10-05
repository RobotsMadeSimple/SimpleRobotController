using System.Collections.Concurrent;
using System.IO.Compression;
using System.Text.Json;
using System.Threading.Channels;
using Controller.RobotControl;
using Controller.RobotControl.Plugins;

namespace RobotControl.Tests.Plugins;

/// <summary>Manual clock: time moves only through <see cref="Advance"/>, which runs due timers in order.</summary>
internal sealed class FakeClock : IPluginClock
{
    private readonly object _lock = new();
    private readonly List<(long Due, long Seq, Action Callback, Handle Handle)> _timers = new();
    private long _seq;

    public long NowMs { get; private set; } = 1_000_000;
    public long UnixMs => 1_700_000_000_000 + NowMs;

    public sealed class Handle : IDisposable
    {
        public bool Cancelled;
        public void Dispose() => Cancelled = true;
    }

    public IDisposable Schedule(int delayMs, Action callback)
    {
        var h = new Handle();
        lock (_lock) _timers.Add((NowMs + delayMs, _seq++, callback, h));
        return h;
    }

    public int PendingTimers { get { lock (_lock) return _timers.Count(t => !t.Handle.Cancelled); } }

    public void Advance(long ms)
    {
        long target = NowMs + ms;
        while (true)
        {
            (long Due, long Seq, Action Callback, Handle Handle) next;
            lock (_lock)
            {
                _timers.RemoveAll(t => t.Handle.Cancelled);
                var due = _timers.Where(t => t.Due <= target).OrderBy(t => t.Due).ThenBy(t => t.Seq).ToList();
                if (due.Count == 0) break;
                next = due[0];
                _timers.Remove(next);
            }
            NowMs = Math.Max(NowMs, next.Due);
            next.Callback();
        }
        NowMs = target;
    }
}

/// <summary>A process whose exit the test controls. Continuations of <see cref="Exited"/> run synchronously.</summary>
internal sealed class FakeProcess : IPluginProcess
{
    private static int _nextPid = 4000;
    private readonly TaskCompletionSource<int> _exit = new();
    public int Pid { get; } = Interlocked.Increment(ref _nextPid);
    public Task<int> Exited => _exit.Task;
    public bool Killed { get; private set; }
    public bool ExitOnKill { get; set; } = true;
    public void Exit(int code) => _exit.TrySetResult(code);
    public void Kill()
    {
        Killed = true;
        if (ExitOnKill) _exit.TrySetResult(-9);
    }
}

/// <summary>Records launches; returns <see cref="FakeProcess"/>es or throws what the test configured.</summary>
internal sealed class FakeLauncher : IPluginProcessLauncher
{
    public List<PluginLaunchContext> Launches { get; } = new();
    public List<FakeProcess> Processes { get; } = new();
    public Exception? Throw { get; set; }
    public bool SimulateInstall { get; set; }

    public FakeProcess Last => Processes[^1];

    /// <summary>When set, launches wait for it (to observe the <c>installing</c> state).</summary>
    public TaskCompletionSource? Gate { get; set; }

    public async Task<IPluginProcess> LaunchAsync(PluginLaunchContext context, CancellationToken ct)
    {
        lock (Launches)
        {
            Launches.Add(context);
            if (SimulateInstall) context.OnInstalling?.Invoke();
        }
        if (Gate != null) await Gate.Task;
        lock (Launches)
        {
            if (Throw != null) throw Throw;
            var p = new FakeProcess();
            Processes.Add(p);
            return p;
        }
    }
}

/// <summary>The plugin side of an in-memory connection: sends frames, collects incoming ones, auto-answers requests.</summary>
internal sealed class TestPluginClient
{
    private readonly InMemoryPluginTransport _transport;
    private readonly Channel<JsonElement> _incoming = Channel.CreateUnbounded<JsonElement>();
    private readonly ConcurrentDictionary<string, TaskCompletionSource<JsonElement>> _pending = new();
    private int _nextId;

    /// <summary>Auto-replies: method → handler returning the result (throw <see cref="PluginProtocolException"/> for ok:false; return <see cref="NoReply"/> to stay silent).</summary>
    public ConcurrentDictionary<string, Func<JsonElement, object?>> Handlers { get; } = new();
    public static readonly object NoReply = new();

    /// <summary>Every request/event frame received (responses are routed to awaiting calls).</summary>
    public ConcurrentQueue<JsonElement> Received { get; } = new();

    public TestPluginClient(InMemoryPluginTransport transport)
    {
        _transport = transport;
        _ = Task.Run(ReceiveLoop);
    }

    public InMemoryPluginTransport Transport => _transport;
    public Task Closed => _closedTcs.Task;
    private readonly TaskCompletionSource _closedTcs = new(TaskCreationOptions.RunContinuationsAsynchronously);

    private async Task ReceiveLoop()
    {
        while (true)
        {
            var text = await _transport.ReceiveAsync(CancellationToken.None);
            if (text is null) break;
            var el = JsonDocument.Parse(text).RootElement.Clone();
            var t = el.GetProperty("t").GetString();
            if (t == "res")
            {
                if (_pending.TryRemove(el.GetProperty("id").GetString()!, out var tcs)) tcs.TrySetResult(el);
                continue;
            }
            Received.Enqueue(el);
            await _incoming.Writer.WriteAsync(el);
            if (t == "req")
            {
                var method = el.GetProperty("method").GetString()!;
                var id = el.GetProperty("id").GetString()!;
                var p = el.GetProperty("params");
                if (Handlers.TryGetValue(method, out var h))
                {
                    try
                    {
                        var result = h(p);
                        if (!ReferenceEquals(result, NoReply))
                            await SendRawAsync(new { t = "res", id, ok = true, result = result ?? new { } });
                    }
                    catch (PluginProtocolException ex)
                    {
                        await SendRawAsync(new { t = "res", id, ok = false, error = ex.Code, message = ex.Message });
                    }
                }
                else if (method == "shutdown")
                    await SendRawAsync(new { t = "res", id, ok = true, result = new { } });
            }
        }
        _incoming.Writer.TryComplete();
        _closedTcs.TrySetResult();
    }

    public Task SendRawAsync(object frame) =>
        _transport.SendAsync(JsonSerializer.Serialize(frame, PluginJson.Options), CancellationToken.None);

    /// <summary>Sends a request and returns the whole <c>res</c> frame.</summary>
    public async Task<JsonElement> RequestAsync(string method, object? parameters = null)
    {
        string id = "p" + Interlocked.Increment(ref _nextId);
        var tcs = new TaskCompletionSource<JsonElement>(TaskCreationOptions.RunContinuationsAsynchronously);
        _pending[id] = tcs;
        await SendRawAsync(new { t = "req", id, method, @params = parameters ?? new { } });
        return await tcs.Task.WaitAsync(TimeSpan.FromSeconds(5));
    }

    public Task EventAsync(string name, object data) => SendRawAsync(new { t = "evt", @event = name, data });

    /// <summary>Waits for the next incoming request/event matching <paramref name="match"/> (others are skipped).</summary>
    public async Task<JsonElement> WaitForAsync(Func<JsonElement, bool> match, int timeoutMs = 5000)
    {
        using var cts = new CancellationTokenSource(timeoutMs);
        while (true)
        {
            var el = await _incoming.Reader.ReadAsync(cts.Token);
            if (match(el)) return el;
        }
    }

    public Task<JsonElement> WaitForRequestAsync(string method, int timeoutMs = 5000) =>
        WaitForAsync(e => e.GetProperty("t").GetString() == "req" && e.GetProperty("method").GetString() == method, timeoutMs);

    public Task<JsonElement> WaitForEventAsync(string name, int timeoutMs = 5000) =>
        WaitForAsync(e => e.GetProperty("t").GetString() == "evt" && e.GetProperty("event").GetString() == name, timeoutMs);

    public Task CloseAsync() => _transport.CloseAsync(1000, "bye");
}

/// <summary>A temp data directory with a PluginManager on a fake clock/launcher.</summary>
internal sealed class PluginTestEnv : IDisposable
{
    public string DataDir { get; }
    public FakeClock Clock { get; } = new();
    public FakeLauncher Launcher { get; } = new();
    public PluginManager Manager { get; private set; }
    public List<string> ConsoleLines { get; } = new();
    public Func<CommandMessage, Task<object?>>? CommandHandler { get; set; }
    public Dictionary<string, double> Io { get; } = new();
    public PluginRobotSnapshot Robot { get; set; } = new(1, 2, 3, 0, 0, 90, false, false, false, 0, null);

    public PluginTestEnv()
    {
        DataDir = Path.Combine(Path.GetTempPath(), "srcplugins-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(DataDir);
        Manager = NewManager();
    }

    public PluginManager NewManager() => Manager = new PluginManager(DataDir, new PluginManagerOptions
    {
        Launcher          = Launcher,
        Clock             = Clock,
        RunPollThread     = false,
        StopGraceMs       = 100,
        Console           = l => { lock (ConsoleLines) ConsoleLines.Add(l); },
        ControllerVersion = "9.9.9-test",
        CommandHandler    = m => CommandHandler != null ? CommandHandler(m) : Task.FromResult<object?>(new { echo = m.Command }),
        StatusProvider    = () => new { moving = false, version = "9.9.9-test" },
        RobotProvider     = () => Robot,
        IoProvider        = io => { foreach (var kv in Io) io[kv.Key] = kv.Value; },
    });

    public string PluginsDir => Path.Combine(DataDir, "plugins");

    /// <summary>Writes plugins/&lt;folder&gt;/plugin.json from <paramref name="manifest"/> (an anonymous object or JSON string).</summary>
    public string WritePlugin(string folder, object manifest)
    {
        var dir = Path.Combine(PluginsDir, folder);
        Directory.CreateDirectory(dir);
        File.WriteAllText(Path.Combine(dir, "plugin.json"),
            manifest as string ?? JsonSerializer.Serialize(manifest, PluginJson.Options));
        return dir;
    }

    public static object Manifest(string id, string runtime = "external", object? extra = null)
    {
        var d = new Dictionary<string, object?>
        {
            ["id"] = id, ["name"] = id.ToUpperInvariant(), ["version"] = "1.0.0", ["protocolVersion"] = 1,
            ["runtime"] = runtime,
            ["configSchema"] = new object[]
            {
                new { key = "port", label = "Port", type = "string", @default = "COM3" },
                new { key = "samples", label = "Samples", type = "number", @default = 5, min = 1, max = 100 },
            },
            ["steps"] = new object[] { new { id = "weigh", label = "Weigh", outputs = new object[] { new { key = "grams", type = "number" } } } },
            ["functions"] = new object[]
            {
                new { name = "twice", minArgs = 1, maxArgs = 1, timeoutMs = 150 },
                new { name = "slow", minArgs = 0, maxArgs = 0, timeoutMs = 60 },
                new { name = "boom", minArgs = 0, maxArgs = 0 },
            },
            ["properties"] = new object[] { new { name = "weight", description = "Live weight", type = "number" } },
        };
        if (runtime != "external") d["entry"] = "main.py";
        if (extra != null)
            foreach (var p in JsonSerializer.SerializeToElement(extra, PluginJson.Options).EnumerateObject())
                d[p.Name] = p.Value;
        return d;
    }

    /// <summary>Discovers and starts (external plugins wait for a connection).</summary>
    public void Boot(int port = 9123)
    {
        Manager.Discover();
        Manager.StartAll(port);
    }

    /// <summary>Opens an in-memory connection and sends plugin.ready; returns the client and the ready reply frame.</summary>
    public async Task<(TestPluginClient Client, JsonElement Reply, Task Serve)> ConnectAsync(string token, object? readyExtra = null)
    {
        var (controller, plugin) = InMemoryPluginTransport.CreatePair();
        var serve = Task.Run(() => Manager.AcceptConnectionAsync(controller));
        var client = new TestPluginClient(plugin);
        var p = new Dictionary<string, object?> { ["token"] = token };
        if (readyExtra != null)
            foreach (var prop in JsonSerializer.SerializeToElement(readyExtra, PluginJson.Options).EnumerateObject())
                p[prop.Name] = prop.Value;
        var replyTask = client.RequestAsync("plugin.ready", p);
        var done = await Task.WhenAny(replyTask, client.Closed);
        return (client, done == replyTask ? await replyTask : default, serve);
    }

    public async Task<TestPluginClient> ConnectReadyAsync(string id)
    {
        var (client, reply, _) = await ConnectAsync(Manager.Get(id)!.Token);
        Assert.True(reply.GetProperty("ok").GetBoolean(), reply.ToString());
        return client;
    }

    public static byte[] Zip(params (string Path, string Content)[] files)
    {
        using var ms = new MemoryStream();
        using (var zip = new ZipArchive(ms, ZipArchiveMode.Create, leaveOpen: true))
            foreach (var (path, content) in files)
            {
                var e = zip.CreateEntry(path);
                using var w = new StreamWriter(e.Open());
                w.Write(content);
            }
        return ms.ToArray();
    }

    public void Dispose()
    {
        try { Manager.StopAll(2000); } catch { }
        try { Directory.Delete(DataDir, recursive: true); } catch { }
    }
}

internal static class Wait
{
    /// <summary>Polls <paramref name="condition"/> (every 5 ms, up to 5 s) — for state reached on another thread.</summary>
    public static async Task UntilAsync(Func<bool> condition, string what = "condition", int timeoutMs = 5000)
    {
        var sw = System.Diagnostics.Stopwatch.StartNew();
        while (!condition())
        {
            if (sw.ElapsedMilliseconds > timeoutMs) throw new TimeoutException("Timed out waiting for " + what);
            await Task.Delay(5);
        }
    }
}
