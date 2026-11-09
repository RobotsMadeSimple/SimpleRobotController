using System.Collections.Concurrent;
using System.Text.Json;

namespace Controller.RobotControl.Plugins;

/// <summary>
/// An error with a protocol code: thrown by a request handler to answer
/// <c>ok:false, error:&lt;code&gt;</c>, and thrown by <see cref="PluginSession.RequestAsync"/>
/// when the peer answers <c>ok:false</c> (or disconnects: <c>disconnected</c>).
/// </summary>
public sealed class PluginProtocolException : Exception
{
    public string Code { get; }
    public PluginProtocolException(string code, string? message = null) : base(message ?? code) => Code = code;
}

/// <summary>What a <see cref="PluginSession"/> calls for inbound plugin→controller traffic.</summary>
public interface IPluginSessionHandler
{
    /// <summary>
    /// Handles a request; the returned object is the <c>result</c> (null → <c>{}</c>).
    /// Throw <see cref="PluginProtocolException"/> for <c>ok:false</c>.
    /// </summary>
    Task<object?> HandleRequestAsync(PluginSession session, string method, JsonElement parameters);

    /// <summary>Handles a notification. Runs on the receive loop (in order); must not block.</summary>
    void HandleEvent(PluginSession session, string name, JsonElement data);
}

/// <summary>A plugin's current <c>events.subscribe</c> request.</summary>
public sealed record PluginSubscription(
    IReadOnlyList<string> Patterns,
    int PositionIntervalMs,
    int StatusIntervalMs,
    int IoIntervalMs)
{
    public const int DefaultPositionMs = 100, MinPositionMs = 20;
    public const int DefaultStatusMs   = 500, MinStatusMs   = 100;
    public const int DefaultIoMs       = 20,  MinIoMs       = 10;

    public static readonly PluginSubscription None = new(Array.Empty<string>(), DefaultPositionMs, DefaultStatusMs, DefaultIoMs);

    public bool Wants(string eventName) => PluginEvents.MatchesAny(Patterns, eventName);
}

/// <summary>
/// One live connection to a plugin (docs/plugins.md §4.2): JSON text frames, request /
/// response correlation with timeouts in both directions, inbound dispatch to an
/// <see cref="IPluginSessionHandler"/>, and a bounded outbound queue (§5): 1024 frames,
/// oldest droppable event (<c>robot.position/status/io.changed</c>) dropped first, and a
/// full queue for anything else closes the connection as <c>slow</c>.
/// </summary>
public sealed class PluginSession
{
    public const int OutboundCapacity = 1024;
    /// <summary>Close code used when the outbound queue overflows with non-droppable frames.</summary>
    public const int SlowCloseCode = 4408;

    private sealed record OutItem(string Json, bool Droppable);

    private readonly IPluginTransport _transport;
    private readonly PluginLog? _log;
    private readonly ConcurrentDictionary<string, TaskCompletionSource<JsonElement>> _pending = new();
    private readonly LinkedList<OutItem> _queue = new();
    private readonly object _queueLock = new();
    private readonly SemaphoreSlim _queueSignal = new(0);
    private readonly CancellationTokenSource _cts = new();
    private long _nextId;
    private int _closed;
    private int _dropped;

    public PluginSession(IPluginTransport transport, string pluginId, PluginLog? log = null, IPluginSessionHandler? handler = null)
    {
        _transport = transport;
        PluginId   = pluginId;
        _log       = log;
        Handler    = handler;
    }

    public string PluginId { get; }
    public IPluginSessionHandler? Handler { get; set; }
    public IPluginTransport Transport => _transport;

    /// <summary>The current event subscription (replaced atomically by <c>events.subscribe</c>).</summary>
    public PluginSubscription Subscription { get; set; } = PluginSubscription.None;

    /// <summary>Per-subscriber IO snapshot last sent (owned by the manager's poll thread).</summary>
    internal Dictionary<string, double>? LastIo { get; set; }
    internal long NextPositionDueMs { get; set; }
    internal long NextStatusDueMs   { get; set; }
    internal long NextIoDueMs       { get; set; }

    public bool IsClosed => Volatile.Read(ref _closed) != 0;
    public string? CloseReason { get; private set; }
    /// <summary>Droppable events discarded because the outbound queue was full.</summary>
    public int DroppedEvents => Volatile.Read(ref _dropped);
    /// <summary>Frames waiting to be sent.</summary>
    public int QueuedCount { get { lock (_queueLock) return _queue.Count; } }

    /// <summary>Raised once when the connection ends (peer closed, transport error, <see cref="CloseAsync"/>).</summary>
    public event Action<PluginSession>? Closed;

    /// <summary>Runs the receive and send loops until the connection ends.</summary>
    public async Task RunAsync(CancellationToken ct = default)
    {
        using var linked = CancellationTokenSource.CreateLinkedTokenSource(ct, _cts.Token);
        var sendLoop = Task.Run(() => SendLoopAsync(linked.Token), CancellationToken.None);
        try
        {
            while (!linked.IsCancellationRequested)
            {
                string? frame;
                try { frame = await _transport.ReceiveAsync(linked.Token); }
                catch (Exception) { frame = null; }
                if (frame is null) break;
                HandleFrame(frame);
            }
        }
        finally
        {
            MarkClosed(CloseReason ?? "disconnected");
            try { await sendLoop; } catch { /* ignored */ }
        }
    }

    // ── outbound ─────────────────────────────────────────────────────────────

    /// <summary>
    /// Sends a request and awaits its reply's <c>result</c>. <paramref name="timeoutMs"/> 0 = no timeout.
    /// Throws <see cref="PluginProtocolException"/> for <c>ok:false</c> or a disconnect, and
    /// <see cref="TimeoutException"/> when no reply arrives in time.
    /// </summary>
    public async Task<JsonElement> RequestAsync(string method, object? parameters, int timeoutMs = 0, CancellationToken ct = default)
    {
        if (IsClosed) throw new PluginProtocolException("disconnected", "Plugin is not connected");
        string id = "c" + Interlocked.Increment(ref _nextId);
        var tcs = new TaskCompletionSource<JsonElement>(TaskCreationOptions.RunContinuationsAsynchronously);
        _pending[id] = tcs;
        try
        {
            string json = $"{{\"t\":\"req\",\"id\":{Quote(id)},\"method\":{Quote(method)},\"params\":{Serialize(parameters ?? new { })}}}";
            if (!Enqueue(new OutItem(json, false)))
                throw new PluginProtocolException("disconnected", "Plugin connection closed");
            if (timeoutMs > 0) return await tcs.Task.WaitAsync(TimeSpan.FromMilliseconds(timeoutMs), ct);
            return await tcs.Task.WaitAsync(ct);
        }
        finally { _pending.TryRemove(id, out _); }
    }

    /// <summary>Sends a request without waiting for (or caring about) the reply.</summary>
    public void Notify(string method, object? parameters)
    {
        _ = RequestAsync(method, parameters).ContinueWith(t => _ = t.Exception, TaskScheduler.Default);
    }

    /// <summary>
    /// Queues an event. <paramref name="dataJson"/> is the already-serialized <c>data</c>.
    /// Returns false when the event (or the connection, for a non-droppable event) was dropped.
    /// </summary>
    public bool SendEvent(string name, string dataJson)
    {
        string json = $"{{\"t\":\"evt\",\"event\":{Quote(name)},\"data\":{dataJson}}}";
        return Enqueue(new OutItem(json, PluginEvents.IsDroppable(name)));
    }

    /// <summary>Queues a reply to a plugin request.</summary>
    public void Reply(string id, object? result) =>
        Enqueue(new OutItem($"{{\"t\":\"res\",\"id\":{Quote(id)},\"ok\":true,\"result\":{Serialize(result ?? new { })}}}", false));

    /// <summary>Queues an error reply to a plugin request.</summary>
    public void ReplyError(string id, string code, string? message) =>
        Enqueue(new OutItem($"{{\"t\":\"res\",\"id\":{Quote(id)},\"ok\":false,\"error\":{Quote(code)},\"message\":{Quote(message ?? code)}}}", false));

    /// <summary>Flushes nothing further and closes the transport with <paramref name="code"/>.</summary>
    public async Task CloseAsync(int code, string reason)
    {
        CloseReason ??= reason;
        MarkClosed(reason);
        try { await _transport.CloseAsync(code, reason); } catch { /* already gone */ }
    }

    private bool Enqueue(OutItem item)
    {
        if (IsClosed) return false;
        lock (_queueLock)
        {
            if (_queue.Count >= OutboundCapacity)
            {
                // Make room by dropping the oldest periodic event (position/status/io) …
                var oldest = FirstDroppable();
                if (oldest != null)
                {
                    _queue.Remove(oldest);
                    Interlocked.Increment(ref _dropped);
                    _queue.AddLast(item);
                    return true; // count unchanged: no new signal
                }
                if (item.Droppable) { Interlocked.Increment(ref _dropped); return false; }
                // … but a program/step event or a request/reply cannot be dropped: the plugin is too slow.
                _ = Task.Run(() => CloseAsync(SlowCloseCode, "slow"));
                _log?.Append("warn", "Outbound queue full (plugin too slow) — disconnecting");
                return false;
            }
            _queue.AddLast(item);
        }
        _queueSignal.Release();
        return true;
    }

    private LinkedListNode<OutItem>? FirstDroppable()
    {
        for (var n = _queue.First; n != null; n = n.Next)
            if (n.Value.Droppable) return n;
        return null;
    }

    private async Task SendLoopAsync(CancellationToken ct)
    {
        try
        {
            while (!ct.IsCancellationRequested)
            {
                await _queueSignal.WaitAsync(ct);
                OutItem? item;
                lock (_queueLock)
                {
                    item = _queue.First?.Value;
                    if (item != null) _queue.RemoveFirst();
                }
                if (item is null) continue;
                await _transport.SendAsync(item.Json, ct);
            }
        }
        catch (OperationCanceledException) { }
        catch (Exception ex)
        {
            MarkClosed("send failed: " + ex.Message);
            try { await _transport.CloseAsync(1011, "send failed"); } catch { }
        }
    }

    // ── inbound ──────────────────────────────────────────────────────────────

    private void HandleFrame(string frame)
    {
        JsonElement root;
        try
        {
            using var doc = JsonDocument.Parse(frame);
            root = doc.RootElement.Clone();
        }
        catch (JsonException)
        {
            _log?.Append("warn", "Ignored a frame that is not valid JSON");
            return;
        }
        if (root.ValueKind != JsonValueKind.Object || !root.TryGetProperty("t", out var tEl)) return;

        switch (tEl.GetString())
        {
            case "req":
            {
                string? id = root.TryGetProperty("id", out var idEl) ? IdText(idEl) : null;
                string method = root.TryGetProperty("method", out var m) && m.ValueKind == JsonValueKind.String ? m.GetString()! : "";
                var parameters = root.TryGetProperty("params", out var p) ? p : PluginJson.EmptyObject;
                if (id is null) return;
                _ = Task.Run(() => DispatchRequestAsync(id, method, parameters));
                break;
            }
            case "res":
            {
                string? id = root.TryGetProperty("id", out var idEl) ? IdText(idEl) : null;
                if (id is null || !_pending.TryRemove(id, out var tcs)) return; // late or unknown reply
                bool ok = root.TryGetProperty("ok", out var okEl) && okEl.ValueKind == JsonValueKind.True;
                if (ok)
                    tcs.TrySetResult(root.TryGetProperty("result", out var r) ? r : PluginJson.EmptyObject);
                else
                {
                    string code = root.TryGetProperty("error", out var e) && e.ValueKind == JsonValueKind.String ? e.GetString()! : "error";
                    string? msg = root.TryGetProperty("message", out var me) && me.ValueKind == JsonValueKind.String ? me.GetString() : null;
                    tcs.TrySetException(new PluginProtocolException(code, msg ?? code));
                }
                break;
            }
            case "evt":
            {
                string name = root.TryGetProperty("event", out var n) && n.ValueKind == JsonValueKind.String ? n.GetString()! : "";
                var data = root.TryGetProperty("data", out var d) ? d : PluginJson.EmptyObject;
                try { Handler?.HandleEvent(this, name, data); }
                catch (Exception ex) { _log?.Append("warn", $"Event '{name}' ignored: {ex.Message}"); }
                break;
            }
        }
    }

    private async Task DispatchRequestAsync(string id, string method, JsonElement parameters)
    {
        try
        {
            if (Handler is null) throw new PluginProtocolException("unknownMethod", $"Unknown method '{method}'");
            var result = await Handler.HandleRequestAsync(this, method, parameters);
            Reply(id, result);
        }
        catch (PluginProtocolException ex) { ReplyError(id, ex.Code, ex.Message); }
        catch (Exception ex) when (ex is JsonException or KeyNotFoundException or InvalidOperationException or FormatException or ArgumentException)
        {
            ReplyError(id, "badParams", ex.Message);
        }
        catch (Exception ex)
        {
            _log?.Append("error", $"Request '{method}' failed: {ex.Message}");
            ReplyError(id, "internalError", ex.Message);
        }
    }

    private void MarkClosed(string reason)
    {
        if (Interlocked.Exchange(ref _closed, 1) != 0) return;
        CloseReason ??= reason;
        foreach (var kv in _pending)
            if (_pending.TryRemove(kv.Key, out var tcs))
                tcs.TrySetException(new PluginProtocolException("disconnected", "Plugin disconnected"));
        try { _cts.Cancel(); } catch (ObjectDisposedException) { }
        try { Closed?.Invoke(this); }
        catch (Exception ex) { _log?.Append("warn", $"Close handler failed: {ex.Message}"); }
    }

    private static string? IdText(JsonElement el) => el.ValueKind switch
    {
        JsonValueKind.String => el.GetString(),
        JsonValueKind.Number => el.GetRawText(),
        _ => null,
    };

    internal static string Quote(string s) => JsonSerializer.Serialize(s);

    internal static string Serialize(object? value) => value switch
    {
        null              => "null",
        JsonElement el    => el.GetRawText(),
        RawJson raw       => raw.Json,
        _                 => JsonSerializer.Serialize(value, PluginJson.Options),
    };
}

/// <summary>Pre-serialized JSON to embed verbatim in a frame.</summary>
public sealed record RawJson(string Json);
