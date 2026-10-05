using System.Collections.Concurrent;
using System.Net.WebSockets;
using System.Reflection;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Threading.Channels;

namespace SimpleRobot.PluginSdk;

/// <summary>One WebSocket connection to the controller (docs/plugins.md §4). Not reused after it ends.</summary>
internal sealed class Session
{
    private static readonly JsonElement EmptyObject = JsonDocument.Parse("{}").RootElement.Clone();
    private static readonly JsonSerializerOptions Json = new() { };

    private readonly PluginHost _host;
    private readonly ClientWebSocket _ws = new();
    private readonly Channel<string> _outbound = Channel.CreateUnbounded<string>(new UnboundedChannelOptions { SingleReader = true });
    private readonly Channel<(string Name, JsonElement Data)> _events = Channel.CreateUnbounded<(string, JsonElement)>(new UnboundedChannelOptions { SingleReader = true });
    private readonly ConcurrentDictionary<string, TaskCompletionSource<JsonElement>> _pending = new();
    private readonly ConcurrentDictionary<string, StepContext> _steps = new();
    private readonly TaskCompletionSource _shutdown = new(TaskCreationOptions.RunContinuationsAsynchronously);
    private long _nextId;
    private volatile Exception? _failure;
    private CancellationTokenSource _cts = null!;

    internal JsonElement Config = EmptyObject;
    internal string ControllerVersion = "";
    internal string PluginId = "";
    internal string PluginDir = "";
    internal string DataDir = "";
    public bool WasReady { get; private set; }

    public Session(PluginHost host)
    {
        _host = host;
        PluginId = host.Options.PluginId;
        PluginDir = host.Options.PluginDir ?? Environment.CurrentDirectory;
        DataDir = host.Options.DataDir ?? "";
    }

    public async Task<SessionEnd> RunAsync(CancellationToken stop)
    {
        await _ws.ConnectAsync(new Uri(_host.Options.Url), stop).ConfigureAwait(false);
        _cts = CancellationTokenSource.CreateLinkedTokenSource(stop);
        var ct = _cts.Token;
        var writer = Task.Run(() => WriterLoopAsync(ct), CancellationToken.None);
        var reader = Task.Run(() => ReaderLoopAsync(ct), CancellationToken.None);
        var dispatcher = Task.Run(() => EventLoopAsync(ct), CancellationToken.None);
        var ctx = new PluginContext(this);
        var background = new List<Task>();
        try
        {
            // plugin.ready is always the first message.
            var readyParams = new Dictionary<string, object?>
            {
                ["token"] = _host.Options.Token,
                ["sdk"] = new { name = "SimpleRobot.PluginSdk", version = SdkVersion },
            };
            if (ManifestNode(_host.Options.ManifestOverride) is { } manifest) readyParams["manifest"] = manifest;
            var ready = await RequestAsync("plugin.ready", readyParams, ct).ConfigureAwait(false);
            if (ready.ValueKind == JsonValueKind.Object)
            {
                if (ready.TryGetProperty("controllerVersion", out var cv) && cv.ValueKind == JsonValueKind.String) ControllerVersion = cv.GetString()!;
                if (ready.TryGetProperty("pluginId", out var pid) && pid.ValueKind == JsonValueKind.String) PluginId = pid.GetString()!;
                if (ready.TryGetProperty("dataDir", out var dd) && dd.ValueKind == JsonValueKind.String) DataDir = dd.GetString()!;
                if (ready.TryGetProperty("config", out var cfg)) Config = cfg;
            }
            WasReady = true;

            if (_host.TryBuildSubscription(out var sub))
                await RequestAsync("events.subscribe", sub, ct).ConfigureAwait(false);

            foreach (var h in _host.ReadyHandlers)
            {
                try { await h(ctx).ConfigureAwait(false); }
                catch (Exception e) when (e is not OperationCanceledException)
                {
                    ctx.Log($"OnReady handler failed: {e.Message}", LogLevel.Error);
                    ctx.SetStatus("error", e.Message);
                }
            }

            foreach (var h in _host.BackgroundHandlers)
                background.Add(Task.Run(async () =>
                {
                    try { await h(ctx, ct).ConfigureAwait(false); }
                    catch (OperationCanceledException) when (ct.IsCancellationRequested) { }
                    catch (Exception e) { ctx.Log($"Background task failed: {e.Message}", LogLevel.Error); }
                }, CancellationToken.None));

            var done = await Task.WhenAny(_shutdown.Task, reader).ConfigureAwait(false);
            if (done == _shutdown.Task)
            {
                // Flush the shutdown reply, then close.
                _outbound.Writer.TryComplete();
                await Task.WhenAny(writer, Task.Delay(TimeSpan.FromSeconds(3))).ConfigureAwait(false);
                await CloseAsync().ConfigureAwait(false);
                return SessionEnd.Shutdown;
            }

            if (stop.IsCancellationRequested) { await CloseAsync().ConfigureAwait(false); return SessionEnd.Cancelled; }
            if (_failure is PluginAuthException or PluginReplacedException) throw _failure;
            return SessionEnd.Lost;
        }
        finally
        {
            _cts.Cancel();
            FailPending(_failure ?? new PluginConnectionException("Connection closed"));
            _outbound.Writer.TryComplete();
            _events.Writer.TryComplete();
            if (background.Count > 0) await Task.WhenAny(Task.WhenAll(background), Task.Delay(TimeSpan.FromSeconds(2))).ConfigureAwait(false);
            _ws.Dispose();
            _cts.Dispose();
        }
    }

    private static string SdkVersion { get; } =
        typeof(Session).Assembly.GetCustomAttribute<AssemblyInformationalVersionAttribute>()?.InformationalVersion?.Split('+')[0] ?? "1.0.0";

    private static JsonNode? ManifestNode(object? m) => m switch
    {
        null => null,
        string s => JsonNode.Parse(s),
        JsonNode n => n.DeepClone(),
        JsonElement e => JsonNode.Parse(e.GetRawText()),
        _ => JsonSerializer.SerializeToNode(m),
    };

    private async Task CloseAsync()
    {
        try
        {
            if (_ws.State == WebSocketState.Open)
            {
                using var t = new CancellationTokenSource(TimeSpan.FromSeconds(2));
                await _ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "bye", t.Token).ConfigureAwait(false);
            }
        }
        catch { /* best effort */ }
    }

    // ---- sending -----------------------------------------------------------------------

    internal void Enqueue(Dictionary<string, object?> frame)
    {
        _outbound.Writer.TryWrite(JsonSerializer.Serialize(frame, Json));
    }

    internal void SendEvent(string name, object? data) =>
        Enqueue(new() { ["t"] = "evt", ["event"] = name, ["data"] = data ?? new { } });

    internal Task<JsonElement> RequestAsync(string method, object? @params, CancellationToken ct = default)
    {
        var id = Interlocked.Increment(ref _nextId).ToString();
        var tcs = new TaskCompletionSource<JsonElement>(TaskCreationOptions.RunContinuationsAsynchronously);
        _pending[id] = tcs;
        if (_failure != null) { _pending.TryRemove(id, out _); return Task.FromException<JsonElement>(_failure); }
        try { Enqueue(new() { ["t"] = "req", ["id"] = id, ["method"] = method, ["params"] = @params ?? new { } }); }
        catch { _pending.TryRemove(id, out _); throw; }
        if (ct.CanBeCanceled)
        {
            var reg = ct.Register(() => { if (_pending.TryRemove(id, out var p)) p.TrySetCanceled(ct); });
            tcs.Task.ContinueWith(_ => reg.Dispose(), TaskScheduler.Default);
        }
        return tcs.Task;
    }

    private void Reply(string id, bool ok, object? result = null, string? error = null, string? message = null)
    {
        var f = new Dictionary<string, object?> { ["t"] = "res", ["id"] = id, ["ok"] = ok };
        if (ok) f["result"] = result ?? new { };
        else { f["error"] = error ?? "exception"; f["message"] = message ?? ""; }
        Enqueue(f);
    }

    private async Task WriterLoopAsync(CancellationToken ct)
    {
        try
        {
            await foreach (var s in _outbound.Reader.ReadAllAsync(CancellationToken.None).ConfigureAwait(false))
            {
                if (_ws.State != WebSocketState.Open) break;
                using var t = CancellationTokenSource.CreateLinkedTokenSource(ct);
                t.CancelAfter(TimeSpan.FromSeconds(30));
                await _ws.SendAsync(Encoding.UTF8.GetBytes(s), WebSocketMessageType.Text, true, t.Token).ConfigureAwait(false);
            }
        }
        catch (Exception e)
        {
            _failure ??= new PluginConnectionException("Send failed: " + e.Message, e);
            _cts.Cancel();
        }
    }

    // ---- receiving ---------------------------------------------------------------------

    private async Task ReaderLoopAsync(CancellationToken ct)
    {
        var buf = new byte[64 * 1024];
        using var ms = new MemoryStream();
        try
        {
            while (!ct.IsCancellationRequested)
            {
                ms.SetLength(0);
                WebSocketReceiveResult r;
                do
                {
                    r = await _ws.ReceiveAsync(buf, ct).ConfigureAwait(false);
                    if (r.MessageType == WebSocketMessageType.Close)
                    {
                        int code = (int?)r.CloseStatus ?? 0;
                        var desc = r.CloseStatusDescription;
                        _failure ??= code switch
                        {
                            4401 => new PluginAuthException($"Controller rejected the plugin token (4401){Suffix(desc)}"),
                            4409 => new PluginReplacedException($"Connection replaced by another instance (4409){Suffix(desc)}"),
                            _ => new PluginConnectionException($"Connection closed by controller ({code}){Suffix(desc)}"),
                        };
                        FailPending(_failure);
                        try { await _ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "", CancellationToken.None).ConfigureAwait(false); } catch { }
                        return;
                    }
                    ms.Write(buf, 0, r.Count);
                } while (!r.EndOfMessage);

                Dispatch(ms.GetBuffer().AsMemory(0, (int)ms.Length));
            }
        }
        catch (Exception e)
        {
            _failure ??= new PluginConnectionException("Receive failed: " + e.Message, e);
        }
        finally
        {
            FailPending(_failure ?? new PluginConnectionException("Connection closed"));
        }
    }

    private static string Suffix(string? d) => string.IsNullOrEmpty(d) ? "" : ": " + d;

    private void FailPending(Exception e)
    {
        _failure ??= e;
        foreach (var id in _pending.Keys)
            if (_pending.TryRemove(id, out var tcs)) tcs.TrySetException(_failure);
    }

    private void Dispatch(ReadOnlyMemory<byte> frame)
    {
        JsonElement root;
        try { using var doc = JsonDocument.Parse(frame); root = doc.RootElement.Clone(); }
        catch (JsonException) { return; } // ignore garbage
        if (root.ValueKind != JsonValueKind.Object) return;
        string? t = Str(root, "t");
        switch (t)
        {
            case "res":
            {
                var id = Str(root, "id");
                if (id == null || !_pending.TryRemove(id, out var tcs)) return;
                if (root.TryGetProperty("ok", out var ok) && ok.ValueKind == JsonValueKind.True)
                    tcs.TrySetResult(root.TryGetProperty("result", out var res) ? res : EmptyObject);
                else
                    tcs.TrySetException(new CommandException(Str(root, "error") ?? "error", Str(root, "message") ?? ""));
                break;
            }
            case "req":
            {
                var id = Str(root, "id") ?? "";
                var method = Str(root, "method") ?? "";
                var p = root.TryGetProperty("params", out var pp) ? pp : EmptyObject;
                _ = Task.Run(() => HandleRequestAsync(id, method, p));
                break;
            }
            case "evt":
            {
                var name = Str(root, "event");
                if (name != null) _events.Writer.TryWrite((name, root.TryGetProperty("data", out var d) ? d : EmptyObject));
                break;
            }
        }
    }

    private static string? Str(JsonElement e, string name) =>
        e.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() : null;

    private async Task EventLoopAsync(CancellationToken ct)
    {
        var ctx = new PluginContext(this);
        try
        {
            await foreach (var (name, data) in _events.Reader.ReadAllAsync(ct).ConfigureAwait(false))
            {
                foreach (var h in _host.EventHandlersFor(name))
                {
                    try { await h(ctx, data).ConfigureAwait(false); }
                    catch (Exception e) when (e is not OperationCanceledException)
                    {
                        ctx.Log($"Event handler for '{name}' failed: {e.Message}", LogLevel.Error);
                    }
                }
            }
        }
        catch (OperationCanceledException) { }
    }

    // ---- inbound requests --------------------------------------------------------------

    private async Task HandleRequestAsync(string id, string method, JsonElement p)
    {
        try
        {
            switch (method)
            {
                case "step.execute": await ExecuteStepAsync(id, p).ConfigureAwait(false); return;
                case "step.cancel":
                {
                    if (Str(p, "invocationId") is { } inv && _steps.TryGetValue(inv, out var sc))
                        sc.Cancel(Str(p, "reason"));
                    Reply(id, true);
                    return;
                }
                case "function.call":
                {
                    var name = Str(p, "name") ?? "";
                    if (!_host.TryGetFunction(name, out var fn)) { Reply(id, false, error: "unknownFunction", message: $"No function '{name}'"); return; }
                    var args = p.TryGetProperty("args", out var a) && a.ValueKind == JsonValueKind.Array
                        ? a.EnumerateArray().Select(x => x.ValueKind switch
                        {
                            JsonValueKind.True => 1.0,
                            JsonValueKind.False => 0.0,
                            _ => x.GetDouble(),
                        }).ToArray()
                        : Array.Empty<double>();
                    var value = await fn!(new PluginContext(this), args).ConfigureAwait(false);
                    Reply(id, true, new { value });
                    return;
                }
                case "config.changed":
                {
                    if (p.TryGetProperty("config", out var cfg)) Config = cfg;
                    var ctx = new PluginContext(this);
                    foreach (var h in _host.ConfigHandlers) await h(ctx, Config).ConfigureAwait(false);
                    Reply(id, true);
                    return;
                }
                case "shutdown":
                    Reply(id, true);
                    _shutdown.TrySetResult();
                    return;
                default:
                    Reply(id, false, error: "unknownMethod", message: $"Unknown method '{method}'");
                    return;
            }
        }
        catch (Exception e)
        {
            Reply(id, false, error: "exception", message: e.Message);
        }
    }

    private async Task ExecuteStepAsync(string id, JsonElement p)
    {
        var stepId = Str(p, "stepId") ?? "";
        var inv = Str(p, "invocationId") ?? "";
        if (!_host.TryGetStep(stepId, out var handler))
        {
            Reply(id, false, error: "unknownStep", message: $"No step '{stepId}'");
            return;
        }
        var sc = new StepContext(this, inv, stepId, Str(p, "programName") ?? "", Str(p, "stepName"),
            p.TryGetProperty("isBackground", out var bg) && bg.ValueKind == JsonValueKind.True, _cts.Token);
        _steps[inv] = sc;
        try
        {
            var prm = new StepParams(p.TryGetProperty("params", out var pp) ? pp : EmptyObject);
            var result = await handler!(sc, prm).ConfigureAwait(false);
            Reply(id, true, new { outputs = result?.ToDictionary() ?? new Dictionary<string, object?>() });
        }
        catch (StepException e)
        {
            Reply(id, false, error: e.Code, message: e.Message);
        }
        catch (OperationCanceledException) when (sc.IsCancelled)
        {
            Reply(id, false, error: "cancelled", message: "Step cancelled");
        }
        catch (Exception e)
        {
            sc.Log($"Step '{stepId}' threw {e.GetType().Name}: {e.Message}", LogLevel.Error);
            Reply(id, false, error: "exception", message: e.Message);
        }
        finally
        {
            _steps.TryRemove(inv, out _);
            sc.Dispose();
        }
    }

    // ---- context implementation ----------------------------------------------------------------

    private class PluginContext : IPluginContext
    {
        protected readonly Session S;
        public PluginContext(Session s) { S = s; }

        public JsonElement Config => S.Config;
        public string ControllerVersion => S.ControllerVersion;
        public string PluginId => S.PluginId;
        public string PluginDir => S.PluginDir;
        public string DataDir => S.DataDir;

        public void Log(string message, LogLevel level = LogLevel.Info) =>
            S.SendEvent("log", new { level = level.ToString().ToLowerInvariant(), message });

        public void SetProperties(object values) => S.SendEvent("properties.set", new { values });
        public void ClearProperties() => S.SendEvent("properties.clear", null);

        public void SetStatus(string state, string? message = null) =>
            S.SendEvent("status", message == null ? new { state } : (object)new { state, message });

        public virtual void Progress(string? message, double? percent = null) =>
            throw new InvalidOperationException("Progress is only valid inside a step handler.");

        public async Task<JsonElement> CommandAsync(string command, object? @params = null) =>
            await S.RequestAsync("controller.command", @params == null ? new { command } : new { command, @params }).ConfigureAwait(false);

        public async Task<VariablesSnapshot> GetVariablesAsync(string? program = null)
        {
            var r = await S.RequestAsync("variables.get", program == null ? new { } : new { programName = program }).ConfigureAwait(false);
            return new VariablesSnapshot { Variables = Map(r, "variables"), Lists = Map(r, "lists"), Strings = Map(r, "strings") };
        }

        public Task SetVariablesAsync(object values, string? program = null) =>
            S.RequestAsync("variables.set", program == null ? new { values } : new { programName = program, values });

        public async Task SubscribeAsync(IEnumerable<string> events, int? positionIntervalMs = null, int? statusIntervalMs = null, int? ioIntervalMs = null)
        {
            S._host.AddSubscription(events, positionIntervalMs, statusIntervalMs, ioIntervalMs);
            S._host.TryBuildSubscription(out var payload);
            await S.RequestAsync("events.subscribe", payload).ConfigureAwait(false);
        }

        private static Dictionary<string, JsonElement> Map(JsonElement r, string name)
        {
            var d = new Dictionary<string, JsonElement>();
            if (r.ValueKind == JsonValueKind.Object && r.TryGetProperty(name, out var o) && o.ValueKind == JsonValueKind.Object)
                foreach (var p in o.EnumerateObject()) d[p.Name] = p.Value;
            return d;
        }
    }

    private sealed class StepContext : PluginContext, IStepContext, IDisposable
    {
        private readonly CancellationTokenSource _cts;
        public StepContext(Session s, string invocationId, string stepId, string program, string? stepName, bool isBackground, CancellationToken sessionCt) : base(s)
        {
            InvocationId = invocationId; StepId = stepId; ProgramName = program; StepName = stepName; IsBackground = isBackground;
            _cts = CancellationTokenSource.CreateLinkedTokenSource(sessionCt);
        }
        public string InvocationId { get; }
        public string StepId { get; }
        public string ProgramName { get; }
        public string? StepName { get; }
        public bool IsBackground { get; }
        public bool IsCancelled => _cts.IsCancellationRequested;
        public CancellationToken CancellationToken => _cts.Token;
        public string? CancelReason { get; private set; }

        public void Cancel(string? reason) { CancelReason = reason; try { _cts.Cancel(); } catch (ObjectDisposedException) { } }
        public void Dispose() => _cts.Dispose();

        public override void Progress(string? message, double? percent = null)
        {
            var d = new Dictionary<string, object?> { ["invocationId"] = InvocationId };
            if (message != null) d["message"] = message;
            if (percent != null) d["percent"] = percent;
            S.SendEvent("step.progress", d);
        }
    }
}
