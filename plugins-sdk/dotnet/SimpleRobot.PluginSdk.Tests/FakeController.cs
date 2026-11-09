using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading.Channels;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.AspNetCore.Hosting.Server;
using Microsoft.AspNetCore.Hosting.Server.Features;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;

namespace SimpleRobot.PluginSdk.Tests;

/// <summary>A minimal controller: Kestrel on a random loopback port exposing <c>/plugin</c> per docs/plugins.md §4.</summary>
public sealed class FakeController : IAsyncDisposable
{
    public const string Token = "tok-123";
    private readonly WebApplication _app;
    private readonly Channel<FakeConnection> _connections = Channel.CreateUnbounded<FakeConnection>();

    public string Url { get; private set; } = "";
    public JsonElement Config { get; set; } = JsonDocument.Parse("""{ "greeting": "hi" }""").RootElement.Clone();

    private FakeController(WebApplication app) { _app = app; }

    public static async Task<FakeController> StartAsync()
    {
        var builder = WebApplication.CreateSlimBuilder();
        builder.Logging.ClearProviders();
        builder.WebHost.UseUrls("http://127.0.0.1:0");
        var app = builder.Build();
        var fake = new FakeController(app);
        app.UseWebSockets();
        app.Map("/plugin", async (HttpContext http) =>
        {
            if (!http.WebSockets.IsWebSocketRequest) { http.Response.StatusCode = 400; return; }
            using var ws = await http.WebSockets.AcceptWebSocketAsync();
            var conn = new FakeConnection(ws, fake);
            fake._connections.Writer.TryWrite(conn);
            await conn.RunAsync(http.RequestAborted);
        });
        await app.StartAsync();
        var addr = app.Services.GetRequiredService<IServer>().Features.Get<IServerAddressesFeature>()!.Addresses.First();
        fake.Url = addr.Replace("http://", "ws://") + "/plugin";
        return fake;
    }

    public Task<FakeConnection> NextConnectionAsync() => _connections.Reader.ReadAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(5));

    public async ValueTask DisposeAsync()
    {
        try { await _app.StopAsync(TimeSpan.FromSeconds(1)); } catch { }
        await _app.DisposeAsync();
    }
}

public sealed class FakeConnection
{
    private readonly WebSocket _ws;
    private readonly FakeController _owner;
    private readonly SemaphoreSlim _sendLock = new(1, 1);
    private readonly Dictionary<string, TaskCompletionSource<JsonElement>> _pending = new();
    private readonly List<JsonElement> _received = new();
    private readonly object _lock = new();
    private TaskCompletionSource _changed = new(TaskCreationOptions.RunContinuationsAsynchronously);
    private int _nextId;

    public TaskCompletionSource<bool> Ready { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
    public JsonElement ReadyParams { get; private set; }

    public FakeConnection(WebSocket ws, FakeController owner) { _ws = ws; _owner = owner; }

    public async Task RunAsync(CancellationToken ct)
    {
        var buf = new byte[16 * 1024];
        try
        {
            bool first = true;
            while (_ws.State == WebSocketState.Open)
            {
                using var ms = new MemoryStream();
                WebSocketReceiveResult r;
                do
                {
                    r = await _ws.ReceiveAsync(buf, ct);
                    if (r.MessageType == WebSocketMessageType.Close)
                    {
                        try { await _ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "", CancellationToken.None); } catch { }
                        return;
                    }
                    ms.Write(buf, 0, r.Count);
                } while (!r.EndOfMessage);

                var msg = JsonDocument.Parse(ms.ToArray()).RootElement.Clone();
                if (first)
                {
                    first = false;
                    bool ok = msg.GetProperty("method").GetString() == "plugin.ready"
                              && msg.GetProperty("params").GetProperty("token").GetString() == FakeController.Token;
                    if (!ok)
                    {
                        Ready.TrySetResult(false);
                        await _ws.CloseAsync((WebSocketCloseStatus)4401, "bad token", CancellationToken.None);
                        return;
                    }
                    ReadyParams = msg.GetProperty("params");
                    Record(msg);
                    await SendAsync(new
                    {
                        t = "res", id = msg.GetProperty("id").GetString(), ok = true,
                        result = new { controllerVersion = "9.9.9", pluginId = "demo", config = _owner.Config, dataDir = "/data" },
                    });
                    Ready.TrySetResult(true);
                    continue;
                }

                switch (msg.GetProperty("t").GetString())
                {
                    case "res":
                        TaskCompletionSource<JsonElement>? tcs;
                        lock (_lock) _pending.Remove(msg.GetProperty("id").GetString()!, out tcs);
                        tcs?.TrySetResult(msg);
                        Record(msg);
                        break;
                    case "req":
                        Record(msg);
                        await AutoReplyAsync(msg);
                        break;
                    default:
                        Record(msg);
                        break;
                }
            }
        }
        catch (Exception e) when (e is WebSocketException or OperationCanceledException or IOException) { }
    }

    private Task AutoReplyAsync(JsonElement req)
    {
        var id = req.GetProperty("id").GetString();
        var p = req.GetProperty("params");
        switch (req.GetProperty("method").GetString())
        {
            case "controller.command":
                var cmd = p.GetProperty("command").GetString();
                return cmd == "Fail"
                    ? SendAsync(new { t = "res", id, ok = false, error = "boom", message = "it broke" })
                    : SendAsync(new { t = "res", id, ok = true, result = new { echo = cmd, args = p.TryGetProperty("params", out var a) ? (object?)a : null } });
            case "events.subscribe":
                return SendAsync(new { t = "res", id, ok = true, result = new { subscribed = p.GetProperty("events") } });
            case "variables.get":
                return SendAsync(new { t = "res", id, ok = true, result = new { variables = new { a = 1 }, lists = new { l = new[] { 1, 2 } }, strings = new { s = "x" } } });
            default:
                return SendAsync(new { t = "res", id, ok = true, result = new { } });
        }
    }

    private void Record(JsonElement e)
    {
        lock (_lock) { _received.Add(e); var old = _changed; _changed = new(TaskCreationOptions.RunContinuationsAsynchronously); old.TrySetResult(); }
    }

    public async Task SendAsync(object frame)
    {
        var bytes = JsonSerializer.SerializeToUtf8Bytes(frame);
        await _sendLock.WaitAsync();
        try { await _ws.SendAsync(bytes, WebSocketMessageType.Text, true, CancellationToken.None); }
        finally { _sendLock.Release(); }
    }

    /// <summary>Sends a request to the plugin and returns the full <c>res</c> frame.</summary>
    public async Task<JsonElement> RequestAsync(string method, object? @params = null)
    {
        var id = "s" + Interlocked.Increment(ref _nextId);
        var tcs = new TaskCompletionSource<JsonElement>(TaskCreationOptions.RunContinuationsAsynchronously);
        lock (_lock) _pending[id] = tcs;
        await SendAsync(new { t = "req", id, method, @params = @params ?? new { } });
        return await tcs.Task.WaitAsync(TimeSpan.FromSeconds(5));
    }

    public Task SendEventAsync(string name, object data) => SendAsync(new { t = "evt", @event = name, data });

    /// <summary>Waits for a frame the plugin sent (any time, past or future) matching the predicate.</summary>
    public async Task<JsonElement> WaitForAsync(Func<JsonElement, bool> predicate)
    {
        var deadline = Task.Delay(TimeSpan.FromSeconds(5));
        while (true)
        {
            Task changed;
            lock (_lock)
            {
                foreach (var f in _received) if (predicate(f)) return f;
                changed = _changed.Task;
            }
            if (await Task.WhenAny(changed, deadline) == deadline) throw new TimeoutException("Expected frame never arrived");
        }
    }

    public Task<JsonElement> WaitForRequestAsync(string method) =>
        WaitForAsync(f => f.GetProperty("t").GetString() == "req" && f.GetProperty("method").GetString() == method);

    public Task<JsonElement> WaitForEventAsync(string name, Func<JsonElement, bool>? where = null) =>
        WaitForAsync(f => f.GetProperty("t").GetString() == "evt" && f.GetProperty("event").GetString() == name
                          && (where == null || where(f.GetProperty("data"))));

    public Task DropAsync() => _ws.CloseAsync(WebSocketCloseStatus.EndpointUnavailable, "drop", CancellationToken.None);
}
