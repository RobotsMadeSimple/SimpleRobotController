using Controller.RobotControl;
using Makaretu.Dns;
using Microsoft.AspNetCore.Builder;
using Microsoft.Extensions.Hosting;
using System.Diagnostics;

class Program
{
    static void Main(string[] args)
    {
        // ── Parse CLI args ─────────────────────────────────────────────────────
        int    port    = 9000;
        string dataDir = AppContext.BaseDirectory;

        for (int i = 0; i < args.Length - 1; i++)
        {
            if (args[i] == "--port" && int.TryParse(args[i + 1], out int p)) port = p;
            if (args[i] == "--data") dataDir = Path.GetFullPath(args[i + 1]);
        }

        Directory.CreateDirectory(dataDir);
        // All relative-path file operations (repos, config, identity, relay) resolve
        // against this directory, so each robot instance gets its own isolated data.
        Directory.SetCurrentDirectory(dataDir);

        var version = Controller.RobotControl.RobotController.Version;
        Console.WriteLine($"[Boot] Simple Robot Controller {version}");
        Console.WriteLine($"[Boot] Port: {port}  Data: {dataDir}");
        Console.WriteLine($"[Boot] OS: {System.Runtime.InteropServices.RuntimeInformation.OSDescription}");
        Console.WriteLine($"[Boot] Runtime: {System.Runtime.InteropServices.RuntimeInformation.FrameworkDescription}");

        var identity = RobotIdentityService.Load();
        var config   = RobotConfigService.Load();

        var robotController = new Controller.RobotControl.RobotController();
        robotController.SetIdentity(identity);
        robotController.SetConfig(config);

        // ---- Web server ----
        var builder = WebApplication.CreateBuilder(args);
        var app = builder.Build();
        app.UseWebSockets();

        var wsServer = new RobotWebSocketServer(
            "/control",
            async command =>
            {
                return await robotController.AddCommand(command);
            }
        );

        ServiceProfile BuildProfile(RobotIdentity id)
        {
            var p = new ServiceProfile(id.SerialNumber, "_robot._tcp", (ushort)port);
            p.AddProperty("ControlEndpoint", "/control");
            p.AddProperty("SerialNumber",    id.SerialNumber);
            p.AddProperty("RobotType",       id.RobotType);
            p.AddProperty("RobotName",       id.RobotName);
            return p;
        }

        var sd = new ServiceDiscovery();
        var currentIdentity = identity;
        var service = BuildProfile(identity);
        sd.Advertise(service);

        Console.WriteLine($"[mDNS] Advertising as '{identity.SerialNumber}._robot._tcp' " +
                          $"(Type: '{identity.RobotType}', Name: '{identity.RobotName}')");

        robotController.OnIdentityChanged = updated =>
        {
            _ = Task.Run(async () =>
            {
                currentIdentity = updated;
                sd.Unadvertise(service);
                await Task.Delay(250);
                service = BuildProfile(updated);
                sd.Advertise(service);
                Console.WriteLine($"[mDNS] Re-advertising with Type: '{updated.RobotType}', Name: '{updated.RobotName}'");
            });
        };

        System.Net.NetworkInformation.NetworkChange.NetworkAddressChanged += (_, _) =>
        {
            _ = Task.Run(async () =>
            {
                await Task.Delay(2000); // wait for DHCP to assign the new IP
                sd.Unadvertise(service);
                await Task.Delay(250);
                service = BuildProfile(currentIdentity);
                sd.Advertise(service);
                Console.WriteLine("[mDNS] Network address changed — re-advertising");
            });
        };

        var lifetime = app.Services.GetRequiredService<IHostApplicationLifetime>();

        // Wire shutdown token so active WebSocket receive loops unblock immediately
        wsServer.SetShutdownToken(lifetime.ApplicationStopping);

        // On shutdown: unadvertise mDNS so peers stop seeing this robot instantly
        lifetime.ApplicationStopping.Register(() =>
        {
            Console.WriteLine("[mDNS] Unadvertising on shutdown…");
            try { sd.Unadvertise(service); } catch { }
            try { sd.Dispose(); } catch { }
        });

        // Periodic re-announce — exits cleanly when ApplicationStopping fires
        _ = Task.Run(async () =>
        {
            try
            {
                while (true)
                {
                    await Task.Delay(3000, lifetime.ApplicationStopping);
                    try { sd.Announce(service); } catch { }
                }
            }
            catch (OperationCanceledException) { }
        });

        wsServer.Map(app);

        // ── Camera endpoints ───────────────────────────────────────────────────
        // WebSocket stream: sends base64 data-URI frames as text messages.
        // Works on web, Android, and Electron (React Native <Image source={{ uri }}>).
        app.MapGet("/camera/{id}/ws", async (string id, HttpContext context) =>
        {
            if (!context.WebSockets.IsWebSocketRequest)
            {
                context.Response.StatusCode = 426;
                return;
            }

            var camera = robotController.CameraManager.GetCamera(id);
            if (camera == null) { context.Response.StatusCode = 404; return; }

            using var ws = await context.WebSockets.AcceptWebSocketAsync();
            var ct = context.RequestAborted;
            try
            {
                while (!ct.IsCancellationRequested && ws.State == System.Net.WebSockets.WebSocketState.Open)
                {
                    var jpeg = camera.GetLatestFrame();
                    if (jpeg != null)
                    {
                        var b64  = "data:image/jpeg;base64," + Convert.ToBase64String(jpeg);
                        var buf  = System.Text.Encoding.ASCII.GetBytes(b64);
                        await ws.SendAsync(buf, System.Net.WebSockets.WebSocketMessageType.Text, true, ct);
                    }
                    await Task.Delay(50, ct); // ~20fps
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception) { }
        });

        // Single JPEG snapshot — lightweight still image for thumbnails / testing
        app.MapGet("/camera/{id}/snapshot", async (string id, HttpContext context) =>
        {
            var camera = robotController.CameraManager.GetCamera(id);
            if (camera == null) { context.Response.StatusCode = 404; return; }

            var jpeg = camera.GetLatestFrame();
            if (jpeg == null) { context.Response.StatusCode = 204; return; }

            context.Response.ContentType        = "image/jpeg";
            context.Response.Headers["Cache-Control"] = "no-cache";
            context.Response.Headers["Access-Control-Allow-Origin"] = "*";
            await context.Response.Body.WriteAsync(jpeg);
        });

        // ── Vision endpoints ───────────────────────────────────────────────────
        // WebSocket stream of annotated frames for a running vision program.
        app.MapGet("/vision/{id}/ws", async (string id, HttpContext context) =>
        {
            if (!context.WebSockets.IsWebSocketRequest)
            {
                context.Response.StatusCode = 426;
                return;
            }

            var proc = robotController.VisionManager.GetProcessor(id);
            if (proc == null) { context.Response.StatusCode = 404; return; }

            using var ws = await context.WebSockets.AcceptWebSocketAsync();
            var ct = context.RequestAborted;
            try
            {
                while (!ct.IsCancellationRequested && ws.State == System.Net.WebSockets.WebSocketState.Open)
                {
                    var jpeg = proc.GetLatestAnnotated();
                    if (jpeg != null)
                    {
                        var b64  = "data:image/jpeg;base64," + Convert.ToBase64String(jpeg);
                        var buf  = System.Text.Encoding.ASCII.GetBytes(b64);
                        await ws.SendAsync(buf, System.Net.WebSockets.WebSocketMessageType.Text, true, ct);
                    }
                    await Task.Delay(50, ct);
                }
            }
            catch (OperationCanceledException) { }
            catch (Exception) { }
        });

        // Raw (unannotated) snapshot for the vision editor zone-drawing canvas
        app.MapGet("/vision/{id}/snapshot", async (string id, HttpContext context) =>
        {
            var proc = robotController.VisionManager.GetProcessor(id);
            if (proc == null) { context.Response.StatusCode = 404; return; }

            var jpeg = proc.GetLatestRaw();
            if (jpeg == null) { context.Response.StatusCode = 204; return; }

            context.Response.ContentType = "image/jpeg";
            context.Response.Headers["Cache-Control"] = "no-cache";
            context.Response.Headers["Access-Control-Allow-Origin"] = "*";
            await context.Response.Body.WriteAsync(jpeg);
        });

        // Latest annotated frame (blobs + zone borders drawn) for live polling
        app.MapGet("/vision/{id}/annotated", async (string id, HttpContext context) =>
        {
            var proc = robotController.VisionManager.GetProcessor(id);
            if (proc == null) { context.Response.StatusCode = 404; return; }

            var jpeg = proc.GetLatestAnnotated();
            if (jpeg == null) { context.Response.StatusCode = 204; return; }

            context.Response.ContentType = "image/jpeg";
            context.Response.Headers["Cache-Control"] = "no-cache";
            context.Response.Headers["Access-Control-Allow-Origin"] = "*";
            await context.Response.Body.WriteAsync(jpeg);
        });

        // Polygon inspection debug frame — threshold mask with color-coded contours
        app.MapGet("/vision/{id}/debug/polygon/{inspId}", async (string id, string inspId, HttpContext context) =>
        {
            var proc = robotController.VisionManager.GetProcessor(id);
            if (proc == null) { context.Response.StatusCode = 404; return; }

            var jpeg = proc.GetPolygonDebugFrame(inspId);
            if (jpeg == null) { context.Response.StatusCode = 204; return; }

            context.Response.ContentType = "image/jpeg";
            context.Response.Headers["Cache-Control"] = "no-cache";
            context.Response.Headers["Access-Control-Allow-Origin"] = "*";
            await context.Response.Body.WriteAsync(jpeg);
        });

        // Line inspection debug frame — Canny edge map with matched and angle-filtered segments
        app.MapGet("/vision/{id}/debug/line/{inspId}", async (string id, string inspId, HttpContext context) =>
        {
            var proc = robotController.VisionManager.GetProcessor(id);
            if (proc == null) { context.Response.StatusCode = 404; return; }

            var jpeg = proc.GetLineDebugFrame(inspId);
            if (jpeg == null) { context.Response.StatusCode = 204; return; }

            context.Response.ContentType = "image/jpeg";
            context.Response.Headers["Cache-Control"] = "no-cache";
            context.Response.Headers["Access-Control-Allow-Origin"] = "*";
            await context.Response.Body.WriteAsync(jpeg);
        });

        // Annotated snapshot captured at the end of the most recent RunVision step for this vision program
        app.MapGet("/program-vision-snapshot/{visionProgramId}", async (string visionProgramId, HttpContext context) =>
        {
            var jpeg = robotController.GetProgramVisionSnapshot(visionProgramId);
            if (jpeg == null) { context.Response.StatusCode = 404; return; }

            context.Response.ContentType = "image/jpeg";
            context.Response.Headers["Cache-Control"] = "no-cache";
            context.Response.Headers["Access-Control-Allow-Origin"] = "*";
            await context.Response.Body.WriteAsync(jpeg);
        });

        // ── DXF file endpoints ─────────────────────────────────────────────────
        // Relative to dataDir (already the CWD), so each robot instance has its own dxf/ folder.
        const string DxfDir = "dxf";
        Directory.CreateDirectory(DxfDir);

        // Upload a DXF file — body is raw text (DXF), query param ?name=filename.dxf
        app.MapPost("/dxf", async (HttpContext context) =>
        {
            var name = context.Request.Query["name"].ToString();
            if (string.IsNullOrWhiteSpace(name) || !name.EndsWith(".dxf", StringComparison.OrdinalIgnoreCase))
            {
                context.Response.StatusCode = 400;
                await context.Response.WriteAsync("Missing or invalid ?name= query parameter.");
                return;
            }
            // Sanitize: strip path separators
            name = Path.GetFileName(name);
            var path = Path.Combine(DxfDir, name);
            using var fs = File.Create(path);
            await context.Request.Body.CopyToAsync(fs);
            context.Response.StatusCode = 200;
            await context.Response.WriteAsync(name);
        });

        // List available DXF files
        app.MapGet("/dxf", (HttpContext context) =>
        {
            var files = Directory.GetFiles(DxfDir, "*.dxf")
                                 .Select(f => Path.GetFileName(f))
                                 .OrderBy(f => f)
                                 .ToArray();
            context.Response.Headers["Access-Control-Allow-Origin"] = "*";
            return Results.Json(files);
        });

        // Download a specific DXF file by name
        app.MapGet("/dxf/{name}", async (string name, HttpContext context) =>
        {
            name = Path.GetFileName(name);
            var path = Path.Combine(DxfDir, name);
            if (!File.Exists(path)) { context.Response.StatusCode = 404; return; }
            context.Response.ContentType = "application/octet-stream";
            context.Response.Headers["Access-Control-Allow-Origin"] = "*";
            context.Response.Headers["Cache-Control"] = "no-cache";
            await context.Response.SendFileAsync(path);
        });

        // Delete a DXF file
        app.MapDelete("/dxf/{name}", (string name, HttpContext context) =>
        {
            name = Path.GetFileName(name);
            var path = Path.Combine(DxfDir, name);
            if (!File.Exists(path)) { context.Response.StatusCode = 404; return; }
            File.Delete(path);
            context.Response.StatusCode = 200;
        });

        app.Run($"http://0.0.0.0:{port}");
    }
}
