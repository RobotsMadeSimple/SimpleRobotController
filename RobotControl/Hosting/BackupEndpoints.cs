using System.Text.Json;
using Controller.RobotControl.Backup;
using Controller.RobotControl.Commands;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Http.Features;

namespace Controller.RobotControl.Hosting;

/// <summary>
/// Whole-controller backup and restore over HTTP (docs/websocket-api.md "Backup &amp; restore"):
/// <c>GET /backup</c> downloads one zip; <c>POST /backup/restore</c> validates an uploaded zip and
/// returns a preview, and with <c>?apply=true</c> applies it and restarts the controller so every
/// repository reloads. CORS is open like <c>/cells</c> (headers on errors too).
/// </summary>
internal static class BackupEndpoints
{
    private static readonly JsonSerializerOptions Json = new() { PropertyNamingPolicy = JsonNamingPolicy.CamelCase };

    /// <summary>How the controller restarts after a restore. Swappable for tests.</summary>
    internal static Action RestartAfterRestore = () =>
    {
        try { RestartPlan.Execute(); }
        catch (Exception ex) { Console.WriteLine($"[Backup] Restart helper failed: {ex.Message}"); }
        Console.WriteLine("[Backup] Exiting to reload the restored data.");
        Environment.Exit(0);
    };

    internal static BackupIdentity IdentityOf(RobotController robot) => new(
        RobotController.Version,
        robot.Config.RobotType,
        robot.Identity.RobotName,
        robot.Identity.SerialNumber,
        (robot.PluginManager?.Plugins ?? [])
            .Select(h => new BackupPlugin(h.Id, h.Manifest?.Name, h.Manifest?.Version))
            .ToList());

    public static void MapBackupEndpoints(this WebApplication app, RobotController robot)
    {
        BackupService Service() => new(Directory.GetCurrentDirectory());

        app.MapMethods("/backup/{**rest}", new[] { "OPTIONS" }, Preflight);
        app.MapMethods("/backup", new[] { "OPTIONS" }, Preflight);

        app.MapGet("/backup", async (HttpContext context) =>
        {
            AllowAnyOrigin(context);
            byte[] zip;
            try { zip = Service().Create(IdentityOf(robot)); }
            catch (Exception ex)
            {
                await WriteJson(context, 500, new { error = "backupFailed", message = ex.Message });
                return;
            }
            var name = SafeFileName(robot.Identity.RobotName);
            context.Response.StatusCode  = 200;
            context.Response.ContentType = "application/zip";
            context.Response.Headers["Cache-Control"] = "no-store";
            context.Response.Headers["Content-Disposition"] =
                $"attachment; filename=\"{name}-backup-{DateTime.Now:yyyyMMdd-HHmmss}.zip\"";
            await context.Response.Body.WriteAsync(zip);
        });

        app.MapPost("/backup/restore", async (HttpContext context) =>
        {
            AllowAnyOrigin(context);
            if (context.Features.Get<IHttpMaxRequestBodySizeFeature>() is { IsReadOnly: false } limit)
                limit.MaxRequestBodySize = BackupService.MaxZipBytes;
            if (context.Request.ContentLength is > BackupService.MaxZipBytes)
            {
                await WriteJson(context, 413, new { error = "tooLarge", message = "A backup must be at most 500 MB." });
                return;
            }

            byte[] body;
            try
            {
                using var ms = new MemoryStream();
                await context.Request.Body.CopyToAsync(ms, context.RequestAborted);
                if (ms.Length > BackupService.MaxZipBytes)
                {
                    await WriteJson(context, 413, new { error = "tooLarge", message = "A backup must be at most 500 MB." });
                    return;
                }
                body = ms.ToArray();
            }
            catch (BadHttpRequestException ex)
            {
                await WriteJson(context, ex.StatusCode, new { error = "tooLarge", message = ex.Message });
                return;
            }

            bool apply = string.Equals(context.Request.Query["apply"], "true", StringComparison.OrdinalIgnoreCase);
            var service  = Service();
            var identity = IdentityOf(robot);
            bool marked = false;
            try
            {
                var preview = service.Preview(body, identity);
                if (!apply)
                {
                    await WriteJson(context, 200, new { ok = true, applied = false, preview });
                    return;
                }

                if (robot.IsBusyForRestore)
                {
                    await WriteJson(context, 409, new { error = "busy",
                        message = "Stop every program and wait for the robot to stop moving before restoring." });
                    return;
                }
                if (!robot.TryMarkRestorePending())
                {
                    await WriteJson(context, 409, new { error = "restoring",
                        message = "A restore was just applied and the controller is restarting." });
                    return;
                }
                marked = true;
                var result = service.Apply(body, identity);
                Console.WriteLine($"[Backup] Restored {result.FilesWritten} file(s); pre-restore backup {result.PreRestoreBackup}. Restarting.");
                await WriteJson(context, 200, new
                {
                    ok = true,
                    applied = true,
                    preview,
                    filesWritten     = result.FilesWritten,
                    preRestoreBackup = result.PreRestoreBackup,
                    restarting       = true,
                });
                await context.Response.CompleteAsync();
                _ = Task.Run(async () => { await Task.Delay(750); RestartAfterRestore(); });
            }
            catch (BackupException ex)
            {
                if (marked) robot.ClearRestorePending();
                await WriteJson(context, ex.Code == "tooLarge" ? 413 : 400, new { error = ex.Code, message = ex.Message });
            }
            catch (Exception ex)
            {
                if (marked) robot.ClearRestorePending();
                Console.WriteLine($"[Backup] Restore failed: {ex}");
                await WriteJson(context, 500, new { error = "restoreFailed", message = ex.Message });
            }
        });
    }

    private static void Preflight(HttpContext context)
    {
        AllowAnyOrigin(context);
        context.Response.Headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS";
        context.Response.Headers["Access-Control-Allow-Headers"] = "Content-Type";
        context.Response.StatusCode = 204;
    }

    private static string SafeFileName(string? name)
    {
        var s = new string((name ?? "").Select(c => char.IsLetterOrDigit(c) || c is '-' or '_' ? c : '-').ToArray()).Trim('-');
        return string.IsNullOrEmpty(s) ? "controller" : s;
    }

    private static async Task WriteJson(HttpContext context, int status, object body)
    {
        context.Response.StatusCode  = status;
        context.Response.ContentType = "application/json; charset=utf-8";
        await context.Response.WriteAsync(JsonSerializer.Serialize(body, Json));
    }

    private static void AllowAnyOrigin(HttpContext context)
    {
        context.Response.Headers["Access-Control-Allow-Origin"]   = "*";
        context.Response.Headers["Access-Control-Expose-Headers"] = "Content-Disposition";
    }
}
