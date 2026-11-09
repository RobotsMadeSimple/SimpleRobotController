using System.Net.WebSockets;
using System.Text;

namespace Controller.RobotControl.Hosting
{
    /// <summary>
    /// One live frame source as seen by a viewer socket: the newest JPEG and, when the source knows
    /// it, how long ago that frame was made (ms, -1 before the first). A new <c>byte[]</c> per frame,
    /// so reference equality tells a new frame from a repeat.
    /// </summary>
    internal sealed record FrameFeed(Func<byte[]?> Frame, Func<long>? AgeMs = null, object? Owner = null);

    /// <summary>
    /// The send loop behind <c>/camera/{id}/ws</c> and <c>/vision/{id}/ws</c>: pushes each new frame
    /// as a base64 JPEG data URI (text message), and keeps going through every way the feed used to get
    /// stuck for good:
    /// <list type="bullet">
    /// <item>The source is looked up again every second, so a camera that was removed and re-added
    /// or a vision program that was stopped and started again is picked up without a new socket.</item>
    /// <item>The socket is read too, so a viewer that closes (or vanishes) ends the loop instead of it
    /// polling forever for a frame that never comes.</item>
    /// <item>A send that a viewer does not drain within <see cref="SendTimeoutMs"/> aborts the socket
    /// (the viewer reconnects) rather than waiting forever.</item>
    /// <item>With <c>?meta=1</c> the viewer also gets <c>{"type":"status",…}</c> text messages once a
    /// second while no frame went out, carrying the frame count, the source's frame age and whether
    /// the source exists. A viewer can then tell "the camera stopped" (status keeps coming) from "my
    /// socket died" (nothing comes) and say so over the picture. Old viewers never ask for it, so
    /// they only ever see data URIs.</item>
    /// </list>
    /// Stalls and recoveries are logged once each.
    /// </summary>
    internal static class LiveFrameStream
    {
        internal const int ResolveEveryMs = 1000;
        internal const int StatusEveryMs  = 1000;
        internal const int SendTimeoutMs  = 10_000;
        internal const int StallLogMs     = 3000;

        /// <summary>Runs until the viewer goes away or <paramref name="ct"/> fires. Never throws.</summary>
        /// <param name="resolve">Finds the current source (null = it doesn't exist right now).</param>
        /// <param name="label">For log lines, e.g. <c>/camera/CAM_0/ws</c>.</param>
        /// <param name="meta">Send status messages (the viewer asked with <c>?meta=1</c>).</param>
        internal static async Task RunAsync(WebSocket ws, Func<FrameFeed?> resolve, string label, bool meta,
                                            int pollMs, CancellationToken ct, Action<string>? log = null)
        {
            log ??= Console.WriteLine;
            using var life = CancellationTokenSource.CreateLinkedTokenSource(ct);
            var reader = DrainAsync(ws, life);   // ends the loop when the viewer closes

            FrameFeed? feed = null;
            object? owner = null;
            bool everResolved = false;
            long nextResolve = 0, lastSentMs = Environment.TickCount64, lastStatusMs = 0, seq = 0;
            long lastNewFrameMs = Environment.TickCount64;
            bool stalledLogged = false;
            byte[]? last = null;
            try
            {
                while (!life.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    long now = Environment.TickCount64;
                    if (now >= nextResolve)
                    {
                        nextResolve = now + ResolveEveryMs;
                        var next = SafeResolve(resolve);
                        object? nextOwner = next?.Owner ?? next?.Frame.Target;
                        if (everResolved && !ReferenceEquals(nextOwner, owner))
                            log(next == null
                                ? $"[Camera] {label}: the source went away; waiting for it to come back"
                                : $"[Camera] {label}: re-attached to the {(owner == null ? "returned" : "new")} source");
                        if (next != null) everResolved = true;
                        feed = next; owner = nextOwner;
                    }

                    var jpeg = feed == null ? null : SafeFrame(feed);
                    if (jpeg != null && !ReferenceEquals(jpeg, last))
                    {
                        if (last != null && stalledLogged)
                            log($"[Camera] {label}: frames resumed after {(now - lastNewFrameMs) / 1000.0:F1} s");
                        stalledLogged = false;
                        last = jpeg;
                        lastNewFrameMs = now;
                        seq++;
                        var buf = Encoding.ASCII.GetBytes("data:image/jpeg;base64," + Convert.ToBase64String(jpeg));
                        if (!await SendAsync(ws, buf, life.Token, label, log)) break;
                        lastSentMs = Environment.TickCount64;
                    }
                    else
                    {
                        if (!stalledLogged && last != null && now - lastNewFrameMs >= StallLogMs)
                        {
                            stalledLogged = true;
                            log($"[Camera] {label}: no new frame for {StallLogMs / 1000} s " +
                                (feed == null ? "(source gone)" : "(the source stopped producing frames)"));
                        }
                        if (meta && now - lastSentMs >= StatusEveryMs && now - lastStatusMs >= StatusEveryMs)
                        {
                            lastStatusMs = now;
                            long age = feed?.AgeMs is { } a ? SafeAge(a) : (last == null ? -1 : now - lastNewFrameMs);
                            var status = StatusJson(seq, age, feed != null);
                            if (!await SendAsync(ws, Encoding.UTF8.GetBytes(status), life.Token, label, log)) break;
                        }
                    }
                    await Task.Delay(pollMs, life.Token);
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception) { /* client went away mid-send */ }
            finally
            {
                life.Cancel();
                try { await reader; } catch { }
                // Answer a viewer's close here, where no send can be in flight.
                try
                {
                    if (ws.State == WebSocketState.CloseReceived)
                        await ws.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, null, CancellationToken.None);
                }
                catch { }
            }
        }

        /// <summary><c>{"type":"status","seq":N,"frameAgeMs":ms,"source":bool}</c> — seq is frames sent on this socket.</summary>
        internal static string StatusJson(long seq, long frameAgeMs, bool source) =>
            $"{{\"type\":\"status\",\"seq\":{seq},\"frameAgeMs\":{frameAgeMs},\"source\":{(source ? "true" : "false")}}}";

        private static async Task<bool> SendAsync(WebSocket ws, byte[] buf, CancellationToken ct, string label, Action<string> log)
        {
            using var sendCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            sendCts.CancelAfter(SendTimeoutMs);
            try
            {
                await ws.SendAsync(buf, WebSocketMessageType.Text, true, sendCts.Token);
                return true;
            }
            catch (OperationCanceledException) when (!ct.IsCancellationRequested)
            {
                log($"[Camera] {label}: the viewer stopped reading for {SendTimeoutMs / 1000} s; dropping it so it reconnects");
                try { ws.Abort(); } catch { }
                return false;
            }
        }

        /// <summary>Reads (and ignores) whatever the viewer sends; a close or a dead socket cancels the stream.</summary>
        private static async Task DrainAsync(WebSocket ws, CancellationTokenSource life)
        {
            var buf = new byte[1024];
            try
            {
                while (!life.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    var r = await ws.ReceiveAsync(buf, life.Token);
                    if (r.MessageType == WebSocketMessageType.Close) break;   // the send loop answers it
                }
            }
            catch { /* aborted or cancelled */ }
            finally { try { life.Cancel(); } catch (ObjectDisposedException) { } }
        }

        private static FrameFeed? SafeResolve(Func<FrameFeed?> resolve) { try { return resolve(); } catch { return null; } }
        private static byte[]? SafeFrame(FrameFeed f) { try { return f.Frame(); } catch { return null; } }
        private static long SafeAge(Func<long> a) { try { return a(); } catch { return -1; } }
    }
}
