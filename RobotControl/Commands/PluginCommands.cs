using System.Text.Json;
using Controller.RobotControl.Plugins;

namespace Controller.RobotControl.Commands;

/// <summary>
/// Plugin management (docs/plugins.md §7): list/detail/contributions, enable, start/stop/restart,
/// config, logs, uninstall, reload, token rotation. Responses are serialized camelCase through
/// <see cref="PluginJson"/> (enum states as strings) and returned as JSON elements so the
/// WebSocket ACK carries them verbatim.
/// </summary>
internal sealed class PluginCommands
{
    private readonly Func<PluginManager> _manager;

    public PluginCommands(RobotController robot) : this(() => robot.PluginManager) { }

    /// <summary>For tests: commands over a standalone manager.</summary>
    internal PluginCommands(PluginManager manager) : this(() => manager) { }

    private PluginCommands(Func<PluginManager> manager) => _manager = manager;

    private PluginManager Manager => _manager();

    public void Register(CommandDispatcher d)
    {
        d.Add("GetPlugins",                GetPlugins);
        d.Add("GetPlugin",                 GetPlugin);
        d.Add("GetPluginContributions",    GetPluginContributions);
        d.AddAsync("SetPluginEnabled",     SetPluginEnabled);
        d.Add("StartPlugin",               StartPlugin);
        d.Add("StopPlugin",                StopPlugin);
        d.Add("RestartPlugin",             RestartPlugin);
        d.Add("SetPluginConfig",           SetPluginConfig);
        d.Add("GetPluginLogs",             GetPluginLogs);
        d.Add("ClearPluginLogs",           ClearPluginLogs);
        d.AddAsync("UninstallPlugin",      UninstallPlugin);
        d.AddAsync("ReloadPlugins",        ReloadPlugins);
        d.Add("RotatePluginToken",         RotatePluginToken);
    }

    private object? GetPlugins(CommandMessage msg) =>
        Json(new { plugins = Manager.Plugins.Select(h => h.ToSummary()).ToList() });

    private object? GetPlugin(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        return Json(new { plugin = h.ToDetail() });
    }

    private object? GetPluginContributions(CommandMessage msg) => Json(Manager.GetContributions());

    private async Task<object?> SetPluginEnabled(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        if (!TryBool(msg, "enabled", out bool enabled)) return Fail("badParams", "enabled is required");
        // Disabling waits for the stop; enabling returns once the start is requested.
        await h.SetEnabledAsync(enabled);
        return Summary(h);
    }

    private object? StartPlugin(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        if (h.Start(manual: true) is { } code)
            return Fail(code, code == "pluginDisabled" ? $"Plugin '{h.Id}' is disabled" : $"Plugin '{h.Id}' has errors: {h.Message}");
        return Summary(h);
    }

    private object? StopPlugin(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        _ = h.StopAsync("stopped");
        return Summary(h);
    }

    private object? RestartPlugin(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        if (h.Problems.Count > 0) return Fail("pluginError", $"Plugin '{h.Id}' has errors: {h.Message}");
        if (!h.Enabled) return Fail("pluginDisabled", $"Plugin '{h.Id}' is disabled");
        _ = h.RestartAsync();
        return Summary(h);
    }

    private object? SetPluginConfig(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        var p = Params(msg);
        if (!p.TryGetProperty("config", out var cfg) || cfg.ValueKind != JsonValueKind.Object)
            return Fail("badParams", "config must be an object");
        var config = cfg.EnumerateObject().ToDictionary(x => x.Name, x => x.Value.Clone(), StringComparer.Ordinal);
        if (h.SetConfig(config) is { } problem)
            return new { ok = false, error = "badConfig", message = problem.Message, field = problem.Field };
        return Summary(h);
    }

    private object? GetPluginLogs(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        var p = Params(msg);
        int? start = p.TryGetProperty("start", out var s) && s.ValueKind == JsonValueKind.Number ? s.GetInt32() : null;
        int? end   = p.TryGetProperty("end", out var e) && e.ValueKind == JsonValueKind.Number ? e.GetInt32() : null;
        var (total, first, logs) = h.Log.Get(start, end);
        return new { totalCount = total, start = first, logs };
    }

    private object? ClearPluginLogs(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        h.Log.Clear();
        return null;
    }

    private async Task<object?> UninstallPlugin(CommandMessage msg)
    {
        string? id = Id(msg);
        if (string.IsNullOrEmpty(id)) return Fail("badParams", "id is required");
        return await Manager.UninstallAsync(id) ? null : Fail("unknownPlugin", $"No plugin '{id}'");
    }

    private async Task<object?> ReloadPlugins(CommandMessage msg)
    {
        var hosts = await Manager.ReloadAsync();
        return Json(new { plugins = hosts.Select(h => h.ToSummary()).ToList() });
    }

    private object? RotatePluginToken(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        h.RotateToken();
        return Summary(h);
    }

    // ── helpers ──────────────────────────────────────────────────────────────

    private static JsonElement Json(object value) => PluginJson.ToElement(value);

    private static JsonElement Summary(PluginHost h) => Json(new { plugin = h.ToSummary() });

    private static object Fail(string code, string message) => new { ok = false, error = code, message };

    private static JsonElement Params(CommandMessage msg) =>
        msg.Params is { ValueKind: JsonValueKind.Object } p ? p : PluginJson.EmptyObject;

    private static string? Id(CommandMessage msg) =>
        Params(msg).TryGetProperty("id", out var id) && id.ValueKind == JsonValueKind.String ? id.GetString() : null;

    private static bool TryBool(CommandMessage msg, string name, out bool value)
    {
        value = false;
        if (!Params(msg).TryGetProperty(name, out var v)) return false;
        if (v.ValueKind is JsonValueKind.True or JsonValueKind.False) { value = v.GetBoolean(); return true; }
        return false;
    }

    /// <summary>Resolves params.id → host; returns an error payload when missing/unknown.</summary>
    private object? Host(CommandMessage msg, out PluginHost host)
    {
        host = null!;
        string? id = Id(msg);
        if (string.IsNullOrEmpty(id)) return Fail("badParams", "id is required");
        var h = Manager.Get(id);
        if (h is null) return Fail("unknownPlugin", $"No plugin '{id}'");
        host = h;
        return null;
    }
}
