using System.Net.WebSockets;
using System.Text;
using System.Threading.Channels;

namespace Controller.RobotControl.Plugins;

/// <summary>
/// A message-oriented duplex text channel between the controller and one plugin.
/// The production implementation wraps a WebSocket; tests use <see cref="InMemoryPluginTransport"/>.
/// </summary>
public interface IPluginTransport
{
    /// <summary>The next text frame, or null when the peer closed the connection.</summary>
    Task<string?> ReceiveAsync(CancellationToken ct);

    /// <summary>Sends one text frame. Throws when the connection is gone.</summary>
    Task SendAsync(string text, CancellationToken ct);

    /// <summary>Closes the connection with an application close code (4401, 4409, …).</summary>
    Task CloseAsync(int code, string reason);

    /// <summary>The close code this side sent, if any.</summary>
    int? CloseCode { get; }
}

/// <summary>Plugin protocol transport over an ASP.NET Core WebSocket (JSON text frames, 16 MB limit).</summary>
public sealed class WebSocketPluginTransport : IPluginTransport
{
    /// <summary>Maximum frame size (images travel as base64).</summary>
    public const int MaxMessageBytes = 16 * 1024 * 1024;

    private readonly WebSocket _socket;
    private readonly SemaphoreSlim _sendLock = new(1, 1);

    public WebSocketPluginTransport(WebSocket socket) => _socket = socket;

    public int? CloseCode { get; private set; }

    public async Task<string?> ReceiveAsync(CancellationToken ct)
    {
        var buffer = new byte[8192];
        using var ms = new MemoryStream();
        try
        {
            while (true)
            {
                var result = await _socket.ReceiveAsync(new ArraySegment<byte>(buffer), ct);
                if (result.MessageType == WebSocketMessageType.Close) return null;
                ms.Write(buffer, 0, result.Count);
                if (ms.Length > MaxMessageBytes)
                {
                    await CloseAsync((int)WebSocketCloseStatus.MessageTooBig, "message too big");
                    return null;
                }
                if (result.EndOfMessage) break;
            }
        }
        catch (WebSocketException) { return null; }
        catch (OperationCanceledException) { return null; }
        return Encoding.UTF8.GetString(ms.GetBuffer(), 0, (int)ms.Length);
    }

    public async Task SendAsync(string text, CancellationToken ct)
    {
        var bytes = Encoding.UTF8.GetBytes(text);
        await _sendLock.WaitAsync(ct);
        try { await _socket.SendAsync(bytes, WebSocketMessageType.Text, true, ct); }
        finally { _sendLock.Release(); }
    }

    public async Task CloseAsync(int code, string reason)
    {
        CloseCode ??= code;
        if (_socket.State is not (WebSocketState.Open or WebSocketState.CloseReceived)) return;
        using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(2));
        try { await _socket.CloseOutputAsync((WebSocketCloseStatus)code, reason, cts.Token); }
        catch { _socket.Abort(); }
    }
}

/// <summary>
/// One end of an in-process transport pair (tests). Each end's sends appear on the
/// other end's receives. <see cref="PauseSends"/> blocks this end's sends so tests can
/// fill an outbound queue.
/// </summary>
public sealed class InMemoryPluginTransport : IPluginTransport
{
    private readonly Channel<string> _inbox;
    private InMemoryPluginTransport _peer = null!;
    private readonly ManualResetEventSlim _sendGate = new(true);

    private InMemoryPluginTransport(Channel<string> inbox) => _inbox = inbox;

    /// <summary>Creates a connected pair: (controller side, plugin side).</summary>
    public static (InMemoryPluginTransport Controller, InMemoryPluginTransport Plugin) CreatePair()
    {
        var a = new InMemoryPluginTransport(Channel.CreateUnbounded<string>());
        var b = new InMemoryPluginTransport(Channel.CreateUnbounded<string>());
        a._peer = b;
        b._peer = a;
        return (a, b);
    }

    public int? CloseCode { get; private set; }

    /// <summary>The close code the peer sent (what this end "received").</summary>
    public int? PeerCloseCode => _peer.CloseCode;

    public bool IsClosed => CloseCode != null || _peer.CloseCode != null;

    /// <summary>Blocks (true) or releases (false) this end's sends.</summary>
    public bool PauseSends
    {
        get => !_sendGate.IsSet;
        set { if (value) _sendGate.Reset(); else _sendGate.Set(); }
    }

    public async Task<string?> ReceiveAsync(CancellationToken ct)
    {
        try
        {
            if (await _inbox.Reader.WaitToReadAsync(ct) && _inbox.Reader.TryRead(out var msg)) return msg;
            return null;
        }
        catch (OperationCanceledException) { return null; }
        catch (ChannelClosedException) { return null; }
    }

    public Task SendAsync(string text, CancellationToken ct)
    {
        return Task.Run(() =>
        {
            _sendGate.Wait(ct);
            if (IsClosed) throw new InvalidOperationException("transport closed");
            if (!_peer._inbox.Writer.TryWrite(text)) throw new InvalidOperationException("transport closed");
        }, ct);
    }

    public Task CloseAsync(int code, string reason)
    {
        CloseCode ??= code;
        _inbox.Writer.TryComplete();
        _peer._inbox.Writer.TryComplete();
        _sendGate.Set();
        return Task.CompletedTask;
    }
}
