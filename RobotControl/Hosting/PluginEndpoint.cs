using System.Net.WebSockets;
using System.Text;
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

        // ── Custom plugin GUI (docs/plugins.md §4) ─────────────────────────────
        // Static files from the plugin's ui folder, plus the canonical bridge script, plus the
        // push/invoke WebSocket. Served from the controller so the page is same-origin with the WS.
        app.MapGet("/plugins/{id}/ui/{**path}", async (string id, string? path, HttpContext context) =>
        {
            AllowAnyOrigin(context);
            context.Response.Headers["Content-Security-Policy"] =
                "default-src 'self'; connect-src 'self' ws: wss:; img-src 'self' data:; style-src 'self' 'unsafe-inline'; script-src 'self' 'unsafe-inline'; base-uri 'none'";

            // Reserved: always the controller's canonical bridge, never a plugin-supplied file.
            if (string.Equals(path, "ui-bridge.js", StringComparison.Ordinal))
            {
                context.Response.ContentType = "text/javascript; charset=utf-8";
                await context.Response.WriteAsync(UiBridgeJs);
                return;
            }

            var host = manager.Get(id);
            var entry = host?.Manifest?.Ui?.Entry;
            if (host is null || string.IsNullOrWhiteSpace(entry)) { context.Response.StatusCode = 404; return; }

            // The ui root is the entry file's folder (e.g. "ui" for "ui/index.html"); an empty
            // request path serves the entry itself.
            string uiRoot = (Path.GetDirectoryName(entry) ?? "").Replace('\\', '/');
            string rel = string.IsNullOrEmpty(path) ? entry : CombineUnderUi(uiRoot, path);
            if (!PluginManifest.IsInsideFolder(rel)) { context.Response.StatusCode = 404; return; }

            string full = Path.GetFullPath(Path.Combine(host.Folder, rel));
            string root = Path.GetFullPath(host.Folder);
            if (!full.StartsWith(root, StringComparison.Ordinal) || !File.Exists(full)) { context.Response.StatusCode = 404; return; }

            context.Response.ContentType = ContentTypeFor(full);
            await context.Response.SendFileAsync(full);
        });

        app.Map("/plugins/{id}/ui", async (string id, HttpContext context) =>
        {
            if (!context.WebSockets.IsWebSocketRequest) { context.Response.StatusCode = 400; return; }
            using var socket = await context.WebSockets.AcceptWebSocketAsync();
            using var cts = CancellationTokenSource.CreateLinkedTokenSource(context.RequestAborted, shutdown);
            var host = manager.Get(id);
            if (host is null || host.Manifest?.HasUi != true)
            {
                try { await socket.CloseAsync(WebSocketCloseStatus.NormalClosure, "unknownPlugin", CancellationToken.None); } catch { }
                return;
            }
            try { await RunUiSocketAsync(socket, host, cts.Token); }
            catch (Exception ex) when (ex is OperationCanceledException or WebSocketException) { }
        });
    }

    /// <summary>Joins a request path under the ui root; the caller rejects any result that escapes the folder.</summary>
    private static string CombineUnderUi(string uiRoot, string path)
    {
        path = path.Replace('\\', '/').TrimStart('/');
        return string.IsNullOrEmpty(uiRoot) ? path : $"{uiRoot}/{path}";
    }

    private static string ContentTypeFor(string file) => Path.GetExtension(file).ToLowerInvariant() switch
    {
        ".html" or ".htm" => "text/html; charset=utf-8",
        ".js" or ".mjs"   => "text/javascript; charset=utf-8",
        ".css"            => "text/css; charset=utf-8",
        ".json"           => "application/json; charset=utf-8",
        ".svg"            => "image/svg+xml",
        ".png"            => "image/png",
        ".jpg" or ".jpeg" => "image/jpeg",
        ".gif"            => "image/gif",
        ".webp"           => "image/webp",
        ".ico"            => "image/x-icon",
        ".woff2"          => "font/woff2",
        ".woff"           => "font/woff",
        ".ttf"            => "font/ttf",
        ".map"            => "application/json; charset=utf-8",
        _                 => "application/octet-stream",
    };

    // ── UI WebSocket: push view state, forward invoke → ui.action ──────────────
    private static async Task RunUiSocketAsync(WebSocket socket, PluginHost host, CancellationToken ct)
    {
        var sendLock = new SemaphoreSlim(1, 1);

        async Task SendAsync(string json)
        {
            await sendLock.WaitAsync(ct).ConfigureAwait(false);
            try
            {
                if (socket.State == WebSocketState.Open)
                    await socket.SendAsync(Encoding.UTF8.GetBytes(json), WebSocketMessageType.Text, true, ct).ConfigureAwait(false);
            }
            finally { sendLock.Release(); }
        }

        static string StateFrame(long _, bool connected, string stateJson) =>
            $"{{\"t\":\"state\",\"connected\":{(connected ? "true" : "false")},\"state\":{stateJson}}}";

        // Push loop: send the current snapshot, then again whenever (version, connected) changes.
        var push = Task.Run(async () =>
        {
            long lastVer = long.MinValue; bool lastConn = false; bool first = true;
            while (!ct.IsCancellationRequested && socket.State == WebSocketState.Open)
            {
                var (ver, conn, json) = host.ViewSnapshot();
                if (first || ver != lastVer || conn != lastConn)
                {
                    await SendAsync(StateFrame(ver, conn, json)).ConfigureAwait(false);
                    lastVer = ver; lastConn = conn; first = false;
                }
                try { await Task.Delay(80, ct).ConfigureAwait(false); } catch (OperationCanceledException) { break; }
            }
        }, ct);

        // Receive loop: {t:"invoke", id, action, args} → ui.action → {t:"result", id, ok, result|error}.
        var buf = new byte[32 * 1024];
        using var ms = new MemoryStream();
        try
        {
            while (!ct.IsCancellationRequested)
            {
                ms.SetLength(0);
                WebSocketReceiveResult r;
                do
                {
                    r = await socket.ReceiveAsync(buf, ct).ConfigureAwait(false);
                    if (r.MessageType == WebSocketMessageType.Close) return;
                    ms.Write(buf, 0, r.Count);
                    if (ms.Length > 256 * 1024) return; // guard
                } while (!r.EndOfMessage);

                JsonElement root;
                try { using var doc = JsonDocument.Parse(ms.GetBuffer().AsMemory(0, (int)ms.Length)); root = doc.RootElement.Clone(); }
                catch (JsonException) { continue; }
                if (root.ValueKind != JsonValueKind.Object) continue;
                if (StrOf(root, "t") != "invoke") continue;

                string reqId = StrOf(root, "id") ?? "";
                string action = StrOf(root, "action") ?? "";
                JsonElement args = root.TryGetProperty("args", out var a) ? a : default;
                try
                {
                    var result = await host.InvokeUiActionAsync(action, args, ct).ConfigureAwait(false);
                    await SendAsync($"{{\"t\":\"result\",\"id\":{Quote(reqId)},\"ok\":true,\"result\":{result.GetRawText()}}}").ConfigureAwait(false);
                }
                catch (PluginProtocolException ex)
                {
                    await SendAsync($"{{\"t\":\"result\",\"id\":{Quote(reqId)},\"ok\":false,\"error\":{Quote(ex.Code)},\"message\":{Quote(ex.Message)}}}").ConfigureAwait(false);
                }
                catch (Exception ex) when (ex is not OperationCanceledException)
                {
                    await SendAsync($"{{\"t\":\"result\",\"id\":{Quote(reqId)},\"ok\":false,\"error\":\"actionFailed\",\"message\":{Quote(ex.Message)}}}").ConfigureAwait(false);
                }
            }
        }
        finally
        {
            try { await push.ConfigureAwait(false); } catch { }
        }
    }

    private static string? StrOf(JsonElement e, string name) =>
        e.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() : null;

    private static string Quote(string s) => JsonSerializer.Serialize(s);

    /// <summary>
    /// The canonical UI bridge served at <c>/plugins/{id}/ui/ui-bridge.js</c>. A plugin GUI does
    /// <c>&lt;script src="ui-bridge.js"&gt;</c> then uses <c>SimpleRobot.onState / invoke / onStatus</c>.
    /// It derives the plugin id + socket URL from <c>location</c> and reconnects on drop.
    /// </summary>
    private const string UiBridgeJs =
"""
(function () {
  var m = location.pathname.match(/\/plugins\/([^/]+)\/ui(\/|$)/);
  var pluginId = m ? m[1] : "";
  var wsProto = location.protocol === "https:" ? "wss:" : "ws:";
  var wsUrl = wsProto + "//" + location.host + "/plugins/" + pluginId + "/ui";

  var stateCbs = [], statusCbs = [], readyCbs = [];
  var lastState = {}, connected = false, socketOpen = false;
  var pending = {}, nextId = 1, ws = null, backoff = 500;

  function emit(list, arg) { for (var i = 0; i < list.length; i++) { try { list[i](arg); } catch (e) {} } }

  function connect() {
    try { ws = new WebSocket(wsUrl); } catch (e) { setTimeout(connect, backoff); return; }
    ws.onopen = function () { socketOpen = true; backoff = 500; emit(readyCbs); };
    ws.onclose = function () {
      socketOpen = false; connected = false; emit(statusCbs, false);
      setTimeout(connect, backoff); backoff = Math.min(backoff * 2, 5000);
    };
    ws.onerror = function () { try { ws.close(); } catch (e) {} };
    ws.onmessage = function (ev) {
      var msg; try { msg = JSON.parse(ev.data); } catch (e) { return; }
      if (msg.t === "state") {
        lastState = msg.state || {};
        connected = !!msg.connected;
        emit(stateCbs, lastState);
        emit(statusCbs, connected);
      } else if (msg.t === "result") {
        var p = pending[msg.id]; if (!p) return; delete pending[msg.id];
        if (msg.ok) p.resolve(msg.result); else p.reject(new Error(msg.message || msg.error || "action failed"));
      }
    };
  }
  connect();

  window.SimpleRobot = {
    pluginId: pluginId,
    getState: function () { return lastState; },
    isConnected: function () { return connected; },
    onState: function (cb) {
      stateCbs.push(cb);
      if (socketOpen) { try { cb(lastState); } catch (e) {} }
      return function () { var i = stateCbs.indexOf(cb); if (i >= 0) stateCbs.splice(i, 1); };
    },
    onStatus: function (cb) {
      statusCbs.push(cb);
      try { cb(connected); } catch (e) {}
      return function () { var i = statusCbs.indexOf(cb); if (i >= 0) statusCbs.splice(i, 1); };
    },
    onReady: function (cb) { readyCbs.push(cb); if (socketOpen) { try { cb(); } catch (e) {} } },
    invoke: function (action, args) {
      return new Promise(function (resolve, reject) {
        if (!ws || ws.readyState !== 1) { reject(new Error("not connected")); return; }
        var id = String(nextId++);
        pending[id] = { resolve: resolve, reject: reject };
        try { ws.send(JSON.stringify({ t: "invoke", id: id, action: action, args: args || {} })); }
        catch (e) { delete pending[id]; reject(e); }
      });
    }
  };
})();
""";

    private static async Task WriteJson(HttpContext context, int status, object body)
    {
        context.Response.StatusCode  = status;
        context.Response.ContentType = "application/json";
        await context.Response.WriteAsync(JsonSerializer.Serialize(body, PluginJson.Options));
    }

    private static void AllowAnyOrigin(HttpContext context) =>
        context.Response.Headers["Access-Control-Allow-Origin"] = "*";
}
