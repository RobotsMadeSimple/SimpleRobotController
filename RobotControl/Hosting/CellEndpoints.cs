using System.Text.Json;
using Controller.RobotControl.Cells;
using Controller.RobotControl.Commands;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Http.Features;

namespace Controller.RobotControl.Hosting;

/// <summary>
/// Cell package HTTP surface (docs/cell-package.md §4): upload, list, the resolved cell.json (active
/// or by id), package files (meshes) and delete. CORS open like <c>/dxf</c>, plus a preflight answer
/// so a browser can POST/DELETE too.
/// </summary>
internal static class CellEndpoints
{
    private static readonly JsonSerializerOptions Json = new() { PropertyNamingPolicy = JsonNamingPolicy.CamelCase };

    public static void MapCellEndpoints(this WebApplication app, RobotController robot)
    {
        CellRepository Cells() => robot.Cells;

        app.MapMethods("/cells/{**rest}", new[] { "OPTIONS" }, (HttpContext context) =>
        {
            AllowAnyOrigin(context);
            context.Response.Headers["Access-Control-Allow-Methods"] = "GET, POST, DELETE, OPTIONS";
            context.Response.Headers["Access-Control-Allow-Headers"] = "Content-Type";
            context.Response.StatusCode = 204;
        });

        // Upload — raw zip body; id comes from its cell.json. 400 bad package, 409 exists without ?replace=true.
        app.MapPost("/cells", async (HttpContext context) =>
        {
            AllowAnyOrigin(context);
            if (context.Features.Get<IHttpMaxRequestBodySizeFeature>() is { IsReadOnly: false } limit)
                limit.MaxRequestBodySize = CellRepository.MaxZipBytes;
            if (context.Request.ContentLength is > CellRepository.MaxZipBytes)
            {
                await WriteJson(context, 413, new { error = "tooLarge", message = "Cell package must be at most 200 MB" });
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
                    if (ms.Length > CellRepository.MaxZipBytes)
                    {
                        await WriteJson(context, 413, new { error = "tooLarge", message = "Cell package must be at most 200 MB" });
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
            var result = Cells().Install(body, replace);
            if (result.Ok)
                await WriteJson(context, 200, new { id = result.Id, cell = result.Cell is { } s ? CellCommands.ToJson(s) : null });
            else
                await WriteJson(context, result.Error switch { "idExists" => 409, "tooLarge" => 413, _ => 400 },
                                new { error = result.Error, message = result.Message });
        });

        app.MapGet("/cells", async (HttpContext context) =>
        {
            AllowAnyOrigin(context);
            await WriteJson(context, 200, Cells().List().Select(CellCommands.ToJson).ToList());
        });

        // Literal segment: takes precedence over /cells/{id} ("active" is a reserved id).
        app.MapGet("/cells/active", async (HttpContext context) =>
        {
            AllowAnyOrigin(context);
            var cells = Cells();
            var active = cells.ActiveId is { } id ? cells.Get(id) : null;
            await WriteNode(context, CellResolver.Resolve(active, robot.Config));
        });

        app.MapGet("/cells/{id}", async (string id, HttpContext context) =>
        {
            AllowAnyOrigin(context);
            var cell = Cells().Get(id);
            if (cell is null) { await WriteJson(context, 404, new { error = "unknownCell", message = $"No cell package '{id}'" }); return; }
            await WriteNode(context, CellResolver.Resolve(cell, robot.Config));
        });

        app.MapGet("/cells/{id}/{**path}", async (string id, string? path, HttpContext context) =>
        {
            AllowAnyOrigin(context);
            var full = Cells().FilePath(id, path);
            if (full is null) { context.Response.StatusCode = 404; return; }
            context.Response.ContentType = ContentTypeFor(full);
            context.Response.Headers["Cache-Control"] = "no-cache";
            await context.Response.SendFileAsync(full);
        });

        app.MapDelete("/cells/{id}", async (string id, HttpContext context) =>
        {
            AllowAnyOrigin(context);
            if (!Cells().Delete(id)) { await WriteJson(context, 404, new { error = "unknownCell", message = $"No cell package '{id}'" }); return; }
            context.Response.StatusCode = 200;
        });
    }

    internal static string ContentTypeFor(string file) => Path.GetExtension(file).ToLowerInvariant() switch
    {
        ".glb"            => "model/gltf-binary",
        ".gltf"           => "model/gltf+json",
        ".bin"            => "application/octet-stream",
        ".json"           => "application/json; charset=utf-8",
        ".png"            => "image/png",
        ".jpg" or ".jpeg" => "image/jpeg",
        ".webp"           => "image/webp",
        ".ktx2"           => "image/ktx2",
        ".stl"            => "model/stl",
        _                 => "application/octet-stream",
    };

    private static async Task WriteNode(HttpContext context, System.Text.Json.Nodes.JsonNode node)
    {
        context.Response.StatusCode  = 200;
        context.Response.ContentType = "application/json; charset=utf-8";
        context.Response.Headers["Cache-Control"] = "no-cache";
        await context.Response.WriteAsync(node.ToJsonString());
    }

    private static async Task WriteJson(HttpContext context, int status, object body)
    {
        context.Response.StatusCode  = status;
        context.Response.ContentType = "application/json; charset=utf-8";
        await context.Response.WriteAsync(JsonSerializer.Serialize(body, Json));
    }

    private static void AllowAnyOrigin(HttpContext context) =>
        context.Response.Headers["Access-Control-Allow-Origin"] = "*";
}
