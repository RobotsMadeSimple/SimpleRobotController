using System.Net.WebSockets;
using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;

namespace Controller.RobotControl.Hosting
{
    /// <summary>
    /// The plain-HTTP side of the controller: webhook ingress, camera and vision
    /// frames (snapshot and streamed), and DXF/SVG file storage. The WebSocket
    /// command channel lives in <see cref="RobotWebSocketServer"/>.
    /// </summary>
    internal static class HttpEndpoints
    {
        public static void MapRobotEndpoints(this WebApplication app, RobotController robot, string vectorFileDir, string gcodeFileDir)
        {
            MapWebhook(app, robot);
            MapCamera(app, robot);
            MapVision(app, robot);
            MapVectorFiles(app, vectorFileDir);
            MapGcodeFiles(app, gcodeFileDir);
            MapGcodeStream(app, robot);
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
            app.MapGet("/camera/{id}/ws", (string id, HttpContext context) =>
                StreamFramesAsync(context, robot.CameraManager.GetCamera(id) is { } cam ? cam.GetLatestFrame : null));

            // Single JPEG snapshot — lightweight still image for thumbnails / testing.
            app.MapGet("/camera/{id}/snapshot", (string id, HttpContext context) =>
            {
                var camera = robot.CameraManager.GetCamera(id);
                return WriteJpegAsync(context, camera == null ? null : (camera.GetLatestFrame, 204));
            });
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

        // ── G-code files ──────────────────────────────────────────────────────
        // Same shape as /dxf: text upload/list/get/delete, stored per-robot in the data dir.
        private static readonly string[] GcodeExts = { ".nc", ".gcode", ".tap", ".ngc", ".txt" };

        private static void MapGcodeFiles(WebApplication app, string dir)
        {
            Directory.CreateDirectory(dir);

            app.MapPost("/gcode", async (HttpContext context) =>
            {
                var name = context.Request.Query["name"].ToString();
                bool validExt = GcodeExts.Any(e => name.EndsWith(e, StringComparison.OrdinalIgnoreCase));
                if (string.IsNullOrWhiteSpace(name) || !validExt)
                {
                    context.Response.StatusCode = 400;
                    await context.Response.WriteAsync("Missing or invalid ?name= (.nc .gcode .tap .ngc .txt).");
                    return;
                }
                name = Path.GetFileName(name);
                using (var fs = File.Create(Path.Combine(dir, name)))
                    await context.Request.Body.CopyToAsync(fs);
                context.Response.StatusCode = 200;
                await context.Response.WriteAsync(name);
            });

            app.MapGet("/gcode", (HttpContext context) =>
            {
                var files = GcodeExts
                    .SelectMany(e => Directory.GetFiles(dir, "*" + e))
                    .Select(Path.GetFileName)
                    .OrderBy(f => f)
                    .ToArray();
                AllowAnyOrigin(context);
                return Results.Json(files);
            });

            app.MapGet("/gcode/{name}", async (string name, HttpContext context) =>
            {
                var path = Path.Combine(dir, Path.GetFileName(name));
                if (!File.Exists(path)) { context.Response.StatusCode = 404; return; }
                context.Response.ContentType = "text/plain";
                AllowAnyOrigin(context);
                context.Response.Headers["Cache-Control"] = "no-cache";
                await context.Response.SendFileAsync(path);
            });

            app.MapDelete("/gcode/{name}", (string name, HttpContext context) =>
            {
                var path = Path.Combine(dir, Path.GetFileName(name));
                if (!File.Exists(path)) { context.Response.StatusCode = 404; return; }
                File.Delete(path);
                context.Response.StatusCode = 200;
            });
        }

        // ── G-code streaming (WebSocket) ──────────────────────────────────────
        // Text frames in (one or more newline-separated lines), a reply line per G-code
        // line ("ok" / "error:…"). Single-char realtime frames: '?' status, '!' / 0x18 stop.
        private static void MapGcodeStream(WebApplication app, RobotController robot)
        {
            app.Map("/gcode/stream", async context =>
            {
                if (!context.WebSockets.IsWebSocketRequest) { context.Response.StatusCode = 426; return; }
                using var ws = await context.WebSockets.AcceptWebSocketAsync();

                if (!Gcode.GcodeStreamSession.TryCreate(robot, out var session, out var err))
                {
                    await SendWsText(ws, "error:" + err);
                    try { await ws.CloseAsync(WebSocketCloseStatus.PolicyViolation, err, CancellationToken.None); } catch { }
                    return;
                }

                var ct = context.RequestAborted;
                using (var s = session!)
                {
                    await SendWsText(ws, Gcode.GcodeStreamSession.Banner);
                    var buffer = new byte[4096];
                    var pending = new System.Text.StringBuilder();
                    try
                    {
                        while (ws.State == WebSocketState.Open && !ct.IsCancellationRequested)
                        {
                            var result = await ws.ReceiveAsync(buffer, ct);
                            if (result.MessageType == WebSocketMessageType.Close) break;
                            var chunk = System.Text.Encoding.UTF8.GetString(buffer, 0, result.Count);

                            // Whole-frame realtime controls.
                            if (chunk.Length == 1)
                            {
                                if (chunk[0] == '?') { await SendWsText(ws, s.Status()); continue; }
                                if (chunk[0] == '!' || chunk[0] == '\x18') { s.Stop(); await SendWsText(ws, "ok"); continue; }
                            }

                            pending.Append(chunk);
                            int nl;
                            while ((nl = IndexOfNewline(pending)) >= 0)
                            {
                                var line = pending.ToString(0, nl);
                                pending.Remove(0, nl + 1);
                                await SendWsText(ws, s.Feed(line, ct));
                            }
                        }
                    }
                    catch (OperationCanceledException) { }
                    catch (WebSocketException) { }
                }
            });
        }

        private static int IndexOfNewline(System.Text.StringBuilder sb)
        {
            for (int i = 0; i < sb.Length; i++) if (sb[i] == '\n') return i;
            return -1;
        }

        private static async Task SendWsText(WebSocket ws, string text)
        {
            if (ws.State != WebSocketState.Open) return;
            var bytes = System.Text.Encoding.UTF8.GetBytes(text + "\n");
            try { await ws.SendAsync(bytes, WebSocketMessageType.Text, true, CancellationToken.None); }
            catch { /* client went away */ }
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

        /// <summary>
        /// Upgrades to a WebSocket and pushes the latest frame as a base64 data URI at
        /// roughly 20 fps until the client disconnects. A null <paramref name="frame"/>
        /// means the owning device/program does not exist (404).
        /// </summary>
        private static async Task StreamFramesAsync(HttpContext context, Func<byte[]?>? frame)
        {
            if (!context.WebSockets.IsWebSocketRequest) { context.Response.StatusCode = 426; return; }
            if (frame == null) { context.Response.StatusCode = 404; return; }

            using var ws = await context.WebSockets.AcceptWebSocketAsync();
            var ct = context.RequestAborted;
            try
            {
                while (!ct.IsCancellationRequested && ws.State == WebSocketState.Open)
                {
                    var jpeg = frame();
                    if (jpeg != null)
                    {
                        var buf = System.Text.Encoding.ASCII.GetBytes("data:image/jpeg;base64," + Convert.ToBase64String(jpeg));
                        await ws.SendAsync(buf, WebSocketMessageType.Text, true, ct);
                    }
                    await Task.Delay(50, ct); // ~20fps
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception) { /* client went away mid-send */ }
        }
    }
}
