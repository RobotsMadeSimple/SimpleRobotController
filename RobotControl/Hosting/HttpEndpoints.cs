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
        public static void MapRobotEndpoints(this WebApplication app, RobotController robot, string vectorFileDir)
        {
            MapWebhook(app, robot);
            MapCamera(app, robot);
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
            app.MapGet("/camera/{id}/ws", (string id, HttpContext context) =>
                StreamFramesAsync(context, robot.CameraManager.GetCamera(id)?.GetLatestFrame));

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
                StreamFramesAsync(context, robot.VisionManager.GetProcessor(id)?.GetLatestAnnotated));

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
