using System.Net.WebSockets;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;
using SIPSorcery.Net;

namespace Controller.RobotControl.Hosting
{
    /// <summary>
    /// The plain-HTTP side of the controller: webhook ingress, camera and vision
    /// frames (snapshot and streamed), and DXF/SVG file storage. The WebSocket
    /// command channel lives in <see cref="RobotWebSocketServer"/>.
    /// </summary>
    internal static class HttpEndpoints
    {
        public static void MapRobotEndpoints(this WebApplication app, RobotController robot, string vectorFileDir)
        {
            MapWebhook(app, robot);
            MapCamera(app, robot);
            MapDevices(app, robot);
            MapIoHistory(app, robot);
            MapVision(app, robot);
            MapVectorFiles(app, vectorFileDir);
        }

        // ── Webhook ───────────────────────────────────────────────────────────
        // Any HTTP POST to /webhook/{name} delivers the JSON body to all HttpReceive
        // steps currently listening on that name (including other robots).
        private static void MapWebhook(WebApplication app, RobotController robot)
        {
            app.MapPost("/webhook/{name}", async (string name, HttpContext context) =>
            {
                Dictionary<string, JsonElement>? payload;
                try
                {
                    payload = await JsonSerializer.DeserializeAsync<Dictionary<string, JsonElement>>(context.Request.Body);
                }
                catch
                {
                    context.Response.StatusCode = 400;
                    return;
                }
                if (payload == null) { context.Response.StatusCode = 400; return; }
                robot.WebhookManager.Deliver(name, payload);
                AllowAnyOrigin(context);
                context.Response.StatusCode = 200;
                await context.Response.WriteAsync("ok");
            });
        }

        // ── Camera ────────────────────────────────────────────────────────────
        private static void MapCamera(WebApplication app, RobotController robot)
        {
            // Live stream: base64 data-URI frames as WebSocket text messages (works in
            // web, Android and Electron <Image source={{ uri }}>).
            // Poll fast and send only when a new frame actually arrives (the camera stores a fresh
            // byte[] per frame, so reference-equality detects it). This lets the preview track the
            // true stream rate up to ~60fps instead of the old fixed ~20fps, without resending dupes.
            app.MapGet("/camera/{id}/ws", (string id, HttpContext context) =>
                StreamFramesAsync(context, robot.CameraManager.GetCamera(id) is { } cam ? cam.GetLatestPreviewFrame : null, 15));

            // Single JPEG snapshot — lightweight still image for thumbnails / testing.
            app.MapGet("/camera/{id}/snapshot", (string id, HttpContext context) =>
            {
                var camera = robot.CameraManager.GetCamera(id);
                return WriteJpegAsync(context, camera == null ? null : (camera.GetLatestFrame, 204));
            });

            // Ingest: a phone (sourceType "phone") pushes binary JPEG frames IN over this socket;
            // each becomes the camera's latest frame, so snapshot / ws / vision / calibration all
            // see it unchanged. Open on the LAN (no token) — same posture as the read sockets.
            app.MapGet("/camera/{id}/ingest", (string id, HttpContext context) =>
                IngestFramesAsync(context, robot.CameraManager.GetCamera(id)));

            // WebRTC signaling: a phone negotiates an H.264 peer connection over this socket, then
            // streams video the controller decodes to JPEG and pushes into the same "phone" camera
            // (higher frame rate than the JPEG ingest above). JSON text frames both ways:
            //   phone → {type:"offer",sdp} / {type:"ice",candidate:{…}}
            //   ctrl  → {type:"answer",sdp} / {type:"ice",candidate:{…}}
            // Open on the LAN (no token) — same posture as the other camera sockets.
            app.MapGet("/camera/{id}/webrtc", (string id, HttpContext context) =>
                WebRtcSignalingAsync(context, robot.CameraManager.GetCamera(id)));
        }

        // ── Devices (phone / client data channels) ─────────────────────────────
        private static void MapDevices(WebApplication app, RobotController robot)
        {
            // Ingest: a phone pushes named channel values IN over this socket as JSON text
            // frames (single {channel,value,unit?,unixMs?} or batch {values:{k:v,…},unixMs?}).
            // Values surface as $device.<id>.<channel>. Open on the LAN (no token) for v1.
            app.MapGet("/devices/{id}/ingest", (string id, HttpContext context) =>
                IngestDeviceAsync(context, robot.DeviceManager, id));
        }

        // ── IO value history ────────────────────────────────────────────────────
        private static readonly JsonSerializerOptions IoHistoryJson =
            new() { PropertyNamingPolicy = JsonNamingPolicy.CamelCase };

        private static void MapIoHistory(WebApplication app, RobotController robot)
        {
            // Live sample stream for one channel (?channel=<key>&window=<ms>). On connect the
            // server backfills the last `window` ms from the rolling buffer, then pushes new
            // samples as they arrive. Text JSON frames: { channelKey, unit, samples:[{t,v,r}] }.
            // Push-out only — the command channel (/control) stays request/ack.
            app.MapGet("/io/history/ws", (HttpContext context) => StreamIoHistoryAsync(context, robot));
        }

        private static async Task StreamIoHistoryAsync(HttpContext context, RobotController robot)
        {
            if (!context.WebSockets.IsWebSocketRequest) { context.Response.StatusCode = 426; return; }

            string channel = context.Request.Query["channel"].ToString();
            if (string.IsNullOrEmpty(channel)) { context.Response.StatusCode = 400; return; }

            long window = 20_000;
            if (long.TryParse(context.Request.Query["window"], out var w) && w > 0)
                window = Math.Clamp(w, 1_000, 3_600_000);

            using var ws = await context.WebSockets.AcceptWebSocketAsync();
            var ct = context.RequestAborted;
            // Backfill from `window` ms ago (exclusive lower bound), then stream everything newer.
            long lastTs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() - window;
            try
            {
                while (!ct.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    var batch = robot.IoHistory.GetSince(channel, lastTs, 4000);
                    if (batch.Count > 0)
                    {
                        lastTs = batch[^1].T;
                        var payload = JsonSerializer.SerializeToUtf8Bytes(
                            new { channelKey = channel, unit = robot.IoHistory.ChannelUnit(channel), samples = batch },
                            IoHistoryJson);
                        await ws.SendAsync(payload, WebSocketMessageType.Text, true, ct);
                    }
                    await Task.Delay(50, ct);
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception) { /* client went away mid-send */ }
        }

        // ── Vision ────────────────────────────────────────────────────────────
        private static void MapVision(WebApplication app, RobotController robot)
        {
            // Stream of annotated frames for a running vision program.
            app.MapGet("/vision/{id}/ws", (string id, HttpContext context) =>
                StreamFramesAsync(context, robot.VisionManager.GetProcessor(id) is { } proc ? proc.GetLatestAnnotated : null));

            // Raw (unannotated) snapshot for the vision editor zone-drawing canvas.
            app.MapGet("/vision/{id}/snapshot", (string id, HttpContext context) =>
            {
                var proc = robot.VisionManager.GetProcessor(id);
                return WriteJpegAsync(context, proc == null ? null : (proc.GetLatestRaw, 204));
            });

            // Latest annotated frame (blobs + zone borders drawn) for live polling.
            app.MapGet("/vision/{id}/annotated", (string id, HttpContext context) =>
            {
                var proc = robot.VisionManager.GetProcessor(id);
                return WriteJpegAsync(context, proc == null ? null : (proc.GetLatestAnnotated, 204));
            });

            // Polygon inspection debug frame — threshold mask with color-coded contours.
            app.MapGet("/vision/{id}/debug/polygon/{inspId}", (string id, string inspId, HttpContext context) =>
            {
                var proc = robot.VisionManager.GetProcessor(id);
                return WriteJpegAsync(context, proc == null ? null : (() => proc.GetPolygonDebugFrame(inspId), 204));
            });

            // Line inspection debug frame — Canny edge map with matched and angle-filtered segments.
            app.MapGet("/vision/{id}/debug/line/{inspId}", (string id, string inspId, HttpContext context) =>
            {
                var proc = robot.VisionManager.GetProcessor(id);
                return WriteJpegAsync(context, proc == null ? null : (() => proc.GetLineDebugFrame(inspId), 204));
            });

            // Annotated snapshot captured at the end of the most recent RunVision step.
            app.MapGet("/program-vision-snapshot/{visionProgramId}", (string visionProgramId, HttpContext context) =>
                WriteJpegAsync(context, (() => robot.GetProgramVisionSnapshot(visionProgramId), 404)));

            // Calibration wizard frame: detected dots numbered, taught dots highlighted.
            // 404 for an unknown or expired session, 204 while it has no image.
            app.MapGet("/calibration/{sessionId}/image", (string sessionId, HttpContext context) =>
            {
                var session = robot.CalibrationSessions.Get(sessionId);
                return WriteJpegAsync(context, session == null ? null : (() => { lock (session.Sync) return session.AnnotatedJpeg; }, 204));
            });
        }

        // ── Vector files (DXF + SVG) ──────────────────────────────────────────
        // Relative to the data directory, so each robot instance has its own folder.
        private static void MapVectorFiles(WebApplication app, string dir)
        {
            Directory.CreateDirectory(dir);

            // Upload — body is raw text, query param ?name=filename.dxf|.svg
            app.MapPost("/dxf", async (HttpContext context) =>
            {
                var name = context.Request.Query["name"].ToString();
                bool validExt = name.EndsWith(".dxf", StringComparison.OrdinalIgnoreCase)
                             || name.EndsWith(".svg", StringComparison.OrdinalIgnoreCase);
                if (string.IsNullOrWhiteSpace(name) || !validExt)
                {
                    context.Response.StatusCode = 400;
                    await context.Response.WriteAsync("Missing or invalid ?name= query parameter (.dxf or .svg).");
                    return;
                }
                name = Path.GetFileName(name); // strip any path components
                var path = Path.Combine(dir, name);
                using (var fs = File.Create(path))
                    await context.Request.Body.CopyToAsync(fs);
                context.Response.StatusCode = 200;
                await context.Response.WriteAsync(name);
            });

            app.MapGet("/dxf", (HttpContext context) =>
            {
                var files = Directory.GetFiles(dir, "*.dxf")
                                     .Concat(Directory.GetFiles(dir, "*.svg"))
                                     .Select(Path.GetFileName)
                                     .OrderBy(f => f)
                                     .ToArray();
                AllowAnyOrigin(context);
                return Results.Json(files);
            });

            app.MapGet("/dxf/{name}", async (string name, HttpContext context) =>
            {
                var path = Path.Combine(dir, Path.GetFileName(name));
                if (!File.Exists(path)) { context.Response.StatusCode = 404; return; }
                context.Response.ContentType = "application/octet-stream";
                AllowAnyOrigin(context);
                context.Response.Headers["Cache-Control"] = "no-cache";
                await context.Response.SendFileAsync(path);
            });

            app.MapDelete("/dxf/{name}", (string name, HttpContext context) =>
            {
                var path = Path.Combine(dir, Path.GetFileName(name));
                if (!File.Exists(path)) { context.Response.StatusCode = 404; return; }
                File.Delete(path);
                context.Response.StatusCode = 200;
            });
        }

        // ── Helpers ───────────────────────────────────────────────────────────

        private static void AllowAnyOrigin(HttpContext context) =>
            context.Response.Headers["Access-Control-Allow-Origin"] = "*";

        /// <summary>
        /// Writes one JPEG. <paramref name="source"/> is null when the owning device or
        /// program does not exist (404); its <c>EmptyStatus</c> is used when the device
        /// exists but has no frame yet.
        /// </summary>
        private static async Task WriteJpegAsync(HttpContext context, (Func<byte[]?> Frame, int EmptyStatus)? source)
        {
            if (source == null) { context.Response.StatusCode = 404; return; }

            var jpeg = source.Value.Frame();
            if (jpeg == null) { context.Response.StatusCode = source.Value.EmptyStatus; return; }

            context.Response.ContentType = "image/jpeg";
            context.Response.Headers["Cache-Control"] = "no-cache";
            AllowAnyOrigin(context);
            await context.Response.Body.WriteAsync(jpeg);
        }

        private const int MaxIngestFrameBytes = 8 * 1024 * 1024;

        /// <summary>
        /// Receive loop for a phone-push camera: each binary WebSocket message is one JPEG
        /// frame, stored via <see cref="Camera.CameraDevice.PushFrame"/>. 404 for an unknown
        /// camera or one that is not a "phone" source (we never let a push clobber a real
        /// capture device). Oversized messages are dropped; text messages are ignored.
        /// </summary>
        private static async Task IngestFramesAsync(HttpContext context, Camera.CameraDevice? cam)
        {
            if (!context.WebSockets.IsWebSocketRequest) { context.Response.StatusCode = 426; return; }
            if (cam is not { IsPhone: true }) { context.Response.StatusCode = 404; return; }

            using var ws = await context.WebSockets.AcceptWebSocketAsync();
            var ct = context.RequestAborted;
            var buffer = new byte[64 * 1024];
            using var msg = new MemoryStream();
            try
            {
                while (!ct.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    msg.SetLength(0);
                    WebSocketReceiveResult result;
                    bool oversized = false;
                    do
                    {
                        result = await ws.ReceiveAsync(buffer, ct);
                        if (result.MessageType == WebSocketMessageType.Close)
                        {
                            await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, null, CancellationToken.None);
                            return;
                        }
                        if (!oversized) msg.Write(buffer, 0, result.Count);
                        if (msg.Length > MaxIngestFrameBytes) { oversized = true; msg.SetLength(0); }
                    } while (!result.EndOfMessage);

                    if (oversized || result.MessageType != WebSocketMessageType.Binary || msg.Length == 0) continue;
                    var jpeg = msg.ToArray();
                    var (w, h) = TryReadJpegSize(jpeg);
                    cam.PushFrame(jpeg, w, h);
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception) { /* client went away mid-receive */ }
        }

        /// <summary>
        /// WebRTC signaling + receive for a phone camera: negotiates one H.264 peer connection over
        /// this socket and feeds decoded JPEGs into the "phone" camera via
        /// <see cref="Camera.WebRtc.WebRtcCameraReceiver"/>. JSON text frames both ways —
        /// phone sends <c>{type:"offer",sdp}</c> / <c>{type:"ice",candidate:{…}}</c>; the controller
        /// replies <c>{type:"answer",sdp}</c> / <c>{type:"ice",candidate:{…}}</c>. 404 for an unknown
        /// or non-"phone" camera (a push transport must never clobber a real capture device).
        /// </summary>
        private static async Task WebRtcSignalingAsync(HttpContext context, Camera.CameraDevice? cam)
        {
            if (!context.WebSockets.IsWebSocketRequest) { context.Response.StatusCode = 426; return; }
            if (cam is not { IsPhone: true }) { context.Response.StatusCode = 404; return; }

            using var ws = await context.WebSockets.AcceptWebSocketAsync();
            var ct = context.RequestAborted;

            // SendAsync is not re-entrant: local ICE callbacks (SIPSorcery thread) and the answer
            // (receive loop) both send, so serialize behind a semaphore.
            var sendLock = new SemaphoreSlim(1, 1);
            async Task Send(string json)
            {
                var bytes = System.Text.Encoding.UTF8.GetBytes(json);
                await sendLock.WaitAsync(ct);
                try { if (ws.State == WebSocketState.Open) await ws.SendAsync(bytes, WebSocketMessageType.Text, true, ct); }
                catch { /* client went away */ }
                finally { sendLock.Release(); }
            }

            // Bind WebRTC to the exact local address the phone reached us on (e.g. the Tailscale IP).
            // On a tailnet the phone can't gather its own VPN candidate, but it CAN send UDP to this
            // address (Tailscale routes by destination), so advertising just this one host candidate
            // lets ICE form a working pair instead of offering unreachable LAN/other-subnet addresses.
            var bind = context.Connection.LocalIpAddress;
            if (bind != null && bind.IsIPv4MappedToIPv6) bind = bind.MapToIPv4();
            var config = new RTCConfiguration { X_BindAddress = bind };
            using var receiver = new Camera.WebRtc.WebRtcCameraReceiver(cam, config);
            // cand is already a JSON object string from RTCIceCandidate.toJSON().
            receiver.OnLocalIceCandidate += cand => _ = Send($"{{\"type\":\"ice\",\"candidate\":{cand}}}");

            var buffer = new byte[16 * 1024];
            using var msg = new MemoryStream();
            try
            {
                while (!ct.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    msg.SetLength(0);
                    WebSocketReceiveResult result;
                    bool oversized = false;
                    do
                    {
                        result = await ws.ReceiveAsync(buffer, ct);
                        if (result.MessageType == WebSocketMessageType.Close)
                        {
                            await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, null, CancellationToken.None);
                            return;
                        }
                        if (!oversized) msg.Write(buffer, 0, result.Count);
                        if (msg.Length > MaxIngestJsonBytes) { oversized = true; msg.SetLength(0); }
                    } while (!result.EndOfMessage);

                    if (oversized || result.MessageType != WebSocketMessageType.Text || msg.Length == 0) continue;
                    await HandleWebRtcSignal(receiver, msg.GetBuffer().AsSpan(0, (int)msg.Length).ToArray(), Send);
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception) { /* client went away mid-session */ }
            finally { sendLock.Dispose(); }
        }

        /// <summary>Applies one signaling frame: an offer yields an answer; an ICE candidate is added.</summary>
        private static async Task HandleWebRtcSignal(Camera.WebRtc.WebRtcCameraReceiver receiver, byte[] utf8, Func<string, Task> send)
        {
            string type = ""; string? sdp = null; string? candidate = null;
            try
            {
                using var doc = JsonDocument.Parse(utf8);
                var root = doc.RootElement;
                if (root.ValueKind != JsonValueKind.Object) return;
                if (root.TryGetProperty("type", out var t) && t.ValueKind == JsonValueKind.String) type = t.GetString() ?? "";
                if (root.TryGetProperty("sdp", out var s) && s.ValueKind == JsonValueKind.String) sdp = s.GetString();
                if (root.TryGetProperty("candidate", out var c) && c.ValueKind == JsonValueKind.Object) candidate = c.GetRawText();
            }
            catch { return; }

            if (type == "offer" && sdp != null)
            {
                try
                {
                    var answer = await receiver.AnswerAsync(sdp);
                    await send($"{{\"type\":\"answer\",\"sdp\":{JsonSerializer.Serialize(answer)}}}");
                }
                catch (Exception ex) { Console.WriteLine($"[WebRTC] answer failed: {ex.Message}"); }
            }
            else if (type == "ice" && candidate != null)
            {
                receiver.AddRemoteIceCandidate(candidate);
            }
        }

        private const int MaxIngestJsonBytes = 256 * 1024;

        /// <summary>
        /// Receive loop for a phone data device: each JSON text message pushes one or more
        /// channel values into <see cref="Devices.DeviceManager"/>. 404 for an unknown device.
        /// </summary>
        private static async Task IngestDeviceAsync(HttpContext context, Devices.DeviceManager mgr, string id)
        {
            if (!context.WebSockets.IsWebSocketRequest) { context.Response.StatusCode = 426; return; }
            if (!mgr.Exists(id)) { context.Response.StatusCode = 404; return; }

            using var ws = await context.WebSockets.AcceptWebSocketAsync();
            var ct = context.RequestAborted;
            var buffer = new byte[16 * 1024];
            using var msg = new MemoryStream();

            try
            {
                while (!ct.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    msg.SetLength(0);
                    WebSocketReceiveResult result;
                    bool oversized = false;
                    do
                    {
                        result = await ws.ReceiveAsync(buffer, ct);
                        if (result.MessageType == WebSocketMessageType.Close)
                        {
                            await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, null, CancellationToken.None);
                            return;
                        }
                        if (!oversized) msg.Write(buffer, 0, result.Count);
                        if (msg.Length > MaxIngestJsonBytes) { oversized = true; msg.SetLength(0); }
                    } while (!result.EndOfMessage);

                    if (oversized || result.MessageType != WebSocketMessageType.Text || msg.Length == 0) continue;
                    IngestDeviceJson(mgr, id, msg.GetBuffer().AsSpan(0, (int)msg.Length));
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception) { /* client went away mid-receive */ }
        }

        /// <summary>Parses one ingest frame — single <c>{channel,value,unit?}</c> or batch <c>{values:{…}}</c>.</summary>
        private static void IngestDeviceJson(Devices.DeviceManager mgr, string id, ReadOnlySpan<byte> utf8)
        {
            try
            {
                using var doc = JsonDocument.Parse(utf8.ToArray());
                var root = doc.RootElement;
                if (root.ValueKind != JsonValueKind.Object) return;

                long unixMs = root.TryGetProperty("unixMs", out var t) && t.ValueKind == JsonValueKind.Number
                    ? t.GetInt64() : DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

                if (root.TryGetProperty("values", out var vals) && vals.ValueKind == JsonValueKind.Object)
                {
                    foreach (var p in vals.EnumerateObject())
                        if (TryNum(p.Value, out double v)) mgr.Ingest(id, p.Name, v, null, unixMs);
                    return;
                }

                if (root.TryGetProperty("channel", out var ch) && ch.ValueKind == JsonValueKind.String
                    && root.TryGetProperty("value", out var valEl) && TryNum(valEl, out double single))
                {
                    string? unit = root.TryGetProperty("unit", out var u) && u.ValueKind == JsonValueKind.String ? u.GetString() : null;
                    mgr.Ingest(id, ch.GetString()!, single, unit, unixMs);
                }
            }
            catch { /* ignore a malformed frame */ }
        }

        private static bool TryNum(JsonElement e, out double v)
        {
            switch (e.ValueKind)
            {
                case JsonValueKind.Number: v = e.GetDouble(); return true;
                case JsonValueKind.True:   v = 1; return true;
                case JsonValueKind.False:  v = 0; return true;
                default:                   v = 0; return false;
            }
        }

        /// <summary>
        /// Reads a JPEG's pixel size from its SOFn marker without decoding the image.
        /// Returns (0, 0) if it can't be found, which leaves the camera's last known size.
        /// </summary>
        private static (int W, int H) TryReadJpegSize(byte[] d)
        {
            if (d.Length < 4 || d[0] != 0xFF || d[1] != 0xD8) return (0, 0);
            int i = 2;
            while (i + 9 < d.Length)
            {
                if (d[i] != 0xFF) { i++; continue; }
                byte marker = d[i + 1];
                if (marker == 0xFF) { i++; continue; }                         // padding
                if (marker == 0xD8 || marker == 0xD9 ||
                    (marker >= 0xD0 && marker <= 0xD7)) { i += 2; continue; }  // SOI/EOI/RSTn: no length
                int len = (d[i + 2] << 8) | d[i + 3];
                // SOFn (baseline/progressive/…), excluding DHT(C4), JPG(C8), DAC(CC).
                if (marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 && marker != 0xCC)
                {
                    int h = (d[i + 5] << 8) | d[i + 6];
                    int w = (d[i + 7] << 8) | d[i + 8];
                    return (w, h);
                }
                if (len < 2) break;
                i += 2 + len;
            }
            return (0, 0);
        }

        /// <summary>
        /// Upgrades to a WebSocket and pushes the latest frame as a base64 data URI at
        /// roughly 20 fps until the client disconnects. A null <paramref name="frame"/>
        /// means the owning device/program does not exist (404).
        /// </summary>
        private static async Task StreamFramesAsync(HttpContext context, Func<byte[]?>? frame, int pollMs = 50)
        {
            if (!context.WebSockets.IsWebSocketRequest) { context.Response.StatusCode = 426; return; }
            if (frame == null) { context.Response.StatusCode = 404; return; }

            using var ws = await context.WebSockets.AcceptWebSocketAsync();
            var ct = context.RequestAborted;
            byte[]? last = null;
            try
            {
                while (!ct.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    var jpeg = frame();
                    // Skip resending the same frame reference — a fast poll then tracks the real
                    // stream rate (up to ~1000/pollMs fps) without flooding the socket with dupes.
                    if (jpeg != null && !ReferenceEquals(jpeg, last))
                    {
                        last = jpeg;
                        var buf = System.Text.Encoding.ASCII.GetBytes("data:image/jpeg;base64," + Convert.ToBase64String(jpeg));
                        await ws.SendAsync(buf, WebSocketMessageType.Text, true, ct);
                    }
                    await Task.Delay(pollMs, ct);
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception) { /* client went away mid-send */ }
        }
    }
}
