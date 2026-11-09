using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Net.WebSockets;
using System.Text;
using Controller.RobotControl.Camera;
using Controller.RobotControl.Hosting;

namespace RobotControl.Tests;

/// <summary>
/// The live frame socket (<c>/camera/{id}/ws</c>, <c>/vision/{id}/ws</c>) must never stay stuck:
/// status messages while frames stop (only when asked), the source looked up again so a new one is
/// picked up, a viewer's close ending the loop, and frame stamps on the camera.
/// </summary>
public sealed class LiveFrameStreamTests
{
    /// <summary>A connected server/client WebSocket pair over loopback TCP.</summary>
    private static async Task<(WebSocket Server, WebSocket Client, IDisposable Cleanup)> PairAsync()
    {
        var listener = new TcpListener(IPAddress.Loopback, 0);
        listener.Start();
        var port = ((IPEndPoint)listener.LocalEndpoint).Port;
        var client = new TcpClient();
        var acceptTask = listener.AcceptTcpClientAsync();
        await client.ConnectAsync(IPAddress.Loopback, port);
        var server = await acceptTask;
        listener.Stop();
        var s = WebSocket.CreateFromStream(server.GetStream(), isServer: true, subProtocol: null, keepAliveInterval: TimeSpan.Zero);
        var c = WebSocket.CreateFromStream(client.GetStream(), isServer: false, subProtocol: null, keepAliveInterval: TimeSpan.Zero);
        return (s, c, new Cleanup(() => { s.Dispose(); c.Dispose(); server.Dispose(); client.Dispose(); }));
    }

    private sealed class Cleanup(Action a) : IDisposable { public void Dispose() => a(); }

    private static async Task<string?> ReceiveTextAsync(WebSocket ws, int timeoutMs)
    {
        using var cts = new CancellationTokenSource(timeoutMs);
        var buf = new byte[1 << 20];
        using var ms = new MemoryStream();
        try
        {
            WebSocketReceiveResult r;
            do
            {
                r = await ws.ReceiveAsync(buf, cts.Token);
                if (r.MessageType == WebSocketMessageType.Close) return null;
                ms.Write(buf, 0, r.Count);
            } while (!r.EndOfMessage);
            return Encoding.UTF8.GetString(ms.ToArray());
        }
        catch (OperationCanceledException) { return null; }
    }

    [Fact]
    public async Task MetaViewerGetsFramesThenStatusWhileTheSourceIsStill()
    {
        var (server, client, cleanup) = await PairAsync();
        using var _ = cleanup;
        var frame = new byte[] { 1, 2, 3 };
        using var cts = new CancellationTokenSource();
        var run = LiveFrameStream.RunAsync(server, () => new FrameFeed(() => frame, () => 4321), "/t", meta: true, pollMs: 10, cts.Token);

        var first = await ReceiveTextAsync(client, 3000);
        Assert.Equal("data:image/jpeg;base64," + Convert.ToBase64String(frame), first);

        var status = await ReceiveTextAsync(client, 3000);
        Assert.Equal(LiveFrameStream.StatusJson(1, 4321, true), status);

        cts.Cancel();
        await run;
    }

    [Fact]
    public async Task PlainViewerOnlyEverSeesDataUris()
    {
        var (server, client, cleanup) = await PairAsync();
        using var _ = cleanup;
        using var cts = new CancellationTokenSource();
        var frame = new byte[] { 9 };
        var run = LiveFrameStream.RunAsync(server, () => new FrameFeed(() => frame), "/t", meta: false, pollMs: 10, cts.Token);

        Assert.StartsWith("data:image/jpeg;base64,", await ReceiveTextAsync(client, 3000));
        Assert.Null(await ReceiveTextAsync(client, 1500));   // a repeat frame: nothing, not even a status

        cts.Cancel();
        await run;
    }

    [Fact]
    public async Task ASourceThatComesBackIsPickedUpWithoutANewSocket()
    {
        var (server, client, cleanup) = await PairAsync();
        using var _ = cleanup;
        var oldFrame = new byte[] { 1 };
        var newFrame = new byte[] { 2 };
        FrameFeed? current = new(() => oldFrame, null, new object());
        var logs = new ConcurrentQueue<string>();
        using var cts = new CancellationTokenSource();
        var run = LiveFrameStream.RunAsync(server, () => current, "/t", meta: true, pollMs: 10, cts.Token, logs.Enqueue);

        Assert.EndsWith(Convert.ToBase64String(oldFrame), await ReceiveTextAsync(client, 3000));
        current = null;                                          // camera removed
        var gone = await ReceiveTextAsync(client, 3000);
        Assert.Contains("\"source\":false", gone);
        current = new FrameFeed(() => newFrame, null, new object());   // re-added (a new device object)

        string? msg;
        do msg = await ReceiveTextAsync(client, 3000);
        while (msg != null && msg.StartsWith("{"));
        Assert.EndsWith(Convert.ToBase64String(newFrame), msg);
        Assert.Contains(logs, l => l.Contains("re-attached"));

        cts.Cancel();
        await run;
    }

    [Fact]
    public async Task AViewerCloseEndsTheLoopEvenWithNoFramesFlowing()
    {
        var (server, client, cleanup) = await PairAsync();
        using var _ = cleanup;
        var run = LiveFrameStream.RunAsync(server, () => new FrameFeed(() => null), "/t", meta: false, pollMs: 10, CancellationToken.None);

        await client.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, null, CancellationToken.None);
        var done = await Task.WhenAny(run, Task.Delay(5000));
        Assert.Same(run, done);
    }

    [Fact]
    public async Task StallAndRecoveryAreLoggedOnce()
    {
        var (server, client, cleanup) = await PairAsync();
        using var _ = cleanup;
        byte[] frame = { 1 };
        var logs = new ConcurrentQueue<string>();
        using var cts = new CancellationTokenSource();
        var run = LiveFrameStream.RunAsync(server, () => new FrameFeed(() => frame), "/t", meta: false, pollMs: 10, cts.Token, logs.Enqueue);
        Assert.NotNull(await ReceiveTextAsync(client, 3000));

        await Task.Delay(LiveFrameStream.StallLogMs + 500);
        frame = new byte[] { 2 };
        Assert.NotNull(await ReceiveTextAsync(client, 3000));

        Assert.Single(logs, l => l.Contains("no new frame"));
        Assert.Single(logs, l => l.Contains("frames resumed"));
        cts.Cancel();
        await run;
    }

    [Fact]
    public void CameraStampsEachFrame()
    {
        var cam = new CameraDevice(new CameraConfig { Id = "P", Name = "P", SourceType = "phone" });
        Assert.Equal(0, cam.FrameSeq);
        Assert.Equal(-1, cam.FrameAgeMs);
        cam.PushFrame(new byte[] { 0xFF, 0xD8 }, 0, 0);
        cam.PushFrame(new byte[] { 0xFF, 0xD8 }, 0, 0);
        Assert.Equal(2, cam.FrameSeq);
        Assert.InRange(cam.FrameAgeMs, 0, 1000);
    }
}
