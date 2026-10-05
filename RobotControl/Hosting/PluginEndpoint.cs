using System.Text.Json;
using Controller.RobotControl.Plugins;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Http.Features;

namespace Controller.RobotControl.Hosting;

/// <summary>
/// Plugin HTTP surface (docs/plugins.md §4.1, §7): the <c>GET /plugin</c> WebSocket plugins
/// connect back on, <c>POST /plugins/install</c> (raw zip body, <c>?replace=true</c>, 200 MB cap)
/// and <c>GET /plugins/{id}/download</c>.
/// </summary>
internal static class PluginEndpoint
{
    public static void MapPluginEndpoints(this WebApplication app, PluginManager manager, CancellationToken shutdown)
    {
        app.Map("/plugin", async (HttpContext context) =>
        {
            if (!context.WebSockets.IsWebSocketRequest) { context.Response.StatusCode = 400; return; }
            using var socket = await context.WebSockets.AcceptWebSocketAsync();
            using var cts = CancellationTokenSource.CreateLinkedTokenSource(context.RequestAborted, shutdown);
            try
            {
                await manager.AcceptConnectionAsync(new WebSocketPluginTransport(socket), cts.Token);
            }
            catch (Exception ex) when (ex is OperationCanceledException or System.Net.WebSockets.WebSocketException) { }
        });

        app.MapPost("/plugins/install", async (HttpContext context) =>
        {
            AllowAnyOrigin(context);
            if (context.Features.Get<IHttpMaxRequestBodySizeFeature>() is { IsReadOnly: false } limit)
                limit.MaxRequestBodySize = PluginManager.MaxZipBytes;
            if (context.Request.ContentLength is > PluginManager.MaxZipBytes)
            {
                await WriteJson(context, 413, new { error = "tooLarge", message = "Plugin zip must be at most 200 MB" });
                return;
            }

            byte[] body;
            try
            {
                using var ms = new MemoryStream();
                var buffer = new byte[81920];
                int n;
                while ((n = await context.Request.Body.ReadAsync(buffer, context.RequestAborted)) > 0)
                {
                    ms.Write(buffer, 0, n);
                    if (ms.Length > PluginManager.MaxZipBytes)
                    {
                        await WriteJson(context, 413, new { error = "tooLarge", message = "Plugin zip must be at most 200 MB" });
                        return;
                    }
                }
                body = ms.ToArray();
            }
            catch (BadHttpRequestException ex)
            {
                await WriteJson(context, ex.StatusCode, new { error = "tooLarge", message = ex.Message });
                return;
            }

            bool replace = string.Equals(context.Request.Query["replace"], "true", StringComparison.OrdinalIgnoreCase);
            var result = await manager.InstallAsync(body, replace);
            if (!result.Ok)
                await WriteJson(context, 400, new { error = result.Error, message = result.Message });
            else
                await WriteJson(context, 200, new { id = result.Id, plugin = result.Host!.ToSummary() });
        });

        app.MapGet("/plugins/{id}/download", async (string id, HttpContext context) =>
        {
            AllowAnyOrigin(context);
            var zip = manager.DownloadZip(id);
            if (zip is null) { context.Response.StatusCode = 404; return; }
            context.Response.ContentType = "application/zip";
            context.Response.Headers["Content-Disposition"] = $"attachment; filename=\"{manager.Get(id)?.Id ?? id}.zip\"";
            await context.Response.Body.WriteAsync(zip);
        });
    }

    private static async Task WriteJson(HttpContext context, int status, object body)
    {
        context.Response.StatusCode  = status;
        context.Response.ContentType = "application/json";
        await context.Response.WriteAsync(JsonSerializer.Serialize(body, PluginJson.Options));
    }

    private static void AllowAnyOrigin(HttpContext context) =>
        context.Response.Headers["Access-Control-Allow-Origin"] = "*";
}
