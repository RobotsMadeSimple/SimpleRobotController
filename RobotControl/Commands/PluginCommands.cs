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
        d.AddAsync("InvokePluginAction",   InvokePluginAction);
        d.AddAsync("CallPluginFunction",   CallPluginFunction);
        d.AddAsync("RunPluginStep",        RunPluginStep);
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

    /// <summary>Invokes a plugin UI action without a UI socket (docs/plugins.md §7); the GUI uses the socket instead.</summary>
    private async Task<object?> InvokePluginAction(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        var p = Params(msg);
        string? action = p.TryGetProperty("action", out var a) && a.ValueKind == JsonValueKind.String ? a.GetString() : null;
        if (string.IsNullOrEmpty(action)) return Fail("badParams", "action is required");
        JsonElement args = p.TryGetProperty("args", out var ar) ? ar : PluginJson.EmptyObject;
        try { return await h.InvokeUiActionAsync(action, args); }     // plugin replies { result: … } → merged into the ack
        catch (PluginProtocolException ex) { return Fail(ex.Code, ex.Message); }
        catch (Exception ex) { return Fail("actionFailed", ex.Message); }
    }

    /// <summary>
    /// Calls an expression function <c>id.name(args)</c> directly from a client (docs/plugins.md §7),
    /// so the plugin detail page can exercise a function without building a program. Args are numbers
    /// (booleans accepted as 0/1); returns <c>{ value }</c>. Runs off the dispatcher thread because
    /// the call blocks up to the function's timeout.
    /// </summary>
    private async Task<object?> CallPluginFunction(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        var p = Params(msg);
        string? name = p.TryGetProperty("name", out var n) && n.ValueKind == JsonValueKind.String ? n.GetString() : null;
        if (string.IsNullOrEmpty(name)) return Fail("badParams", "name is required");

        var def = h.Manifest?.Functions.FirstOrDefault(f => string.Equals(f.Name, name, StringComparison.OrdinalIgnoreCase));
        if (def is null) return Fail("unknownPluginFunction", $"Plugin '{h.Id}' has no function '{name}'");

        var args = new List<double>();
        if (p.TryGetProperty("args", out var arr) && arr.ValueKind == JsonValueKind.Array)
        {
            foreach (var a in arr.EnumerateArray())
            {
                switch (a.ValueKind)
                {
                    case JsonValueKind.Number: args.Add(a.GetDouble()); break;
                    case JsonValueKind.True:   args.Add(1); break;
                    case JsonValueKind.False:  args.Add(0); break;
                    default: return Fail("badParams", "args must be numbers");
                }
            }
        }
        if (args.Count < def.MinArgs || args.Count > def.EffectiveMaxArgs)
        {
            string want = def.MinArgs == def.EffectiveMaxArgs ? $"{def.MinArgs}" : $"{def.MinArgs}–{def.EffectiveMaxArgs}";
            return Fail("badParams", $"{h.Id}.{def.Name}() expects {want} argument(s), got {args.Count}");
        }

        try
        {
            double value = await Task.Run(() => Manager.CallFunction($"{h.Id}.{def.Name}", args.ToArray()));
            return new { value };
        }
        catch (PluginFunctionException ex) { return Fail(ex.Code, ex.Message); }
        catch (Exception ex) { return Fail("pluginFunctionFailed", ex.Message); }
    }

    /// <summary>
    /// Runs a plugin step once directly from a client (docs/plugins.md §7), so the plugin detail page
    /// can test a step without building a program. <c>params</c> is an object of already-resolved
    /// param values (numbers/booleans/strings/etc.); returns <c>{ outputs }</c>. Bridges the
    /// callback-based <see cref="PluginManager.ExecuteStep"/> to a single awaited reply, bounded by
    /// the step's own timeout (or a default), cancelling the invocation if it overruns.
    /// </summary>
    private async Task<object?> RunPluginStep(CommandMessage msg)
    {
        if (Host(msg, out var h) is { } err) return err;
        var p = Params(msg);
        string? stepId = p.TryGetProperty("stepId", out var s) && s.ValueKind == JsonValueKind.String ? s.GetString() : null;
        if (string.IsNullOrEmpty(stepId)) return Fail("badParams", "stepId is required");

        var def = h.Manifest?.Steps.FirstOrDefault(x => string.Equals(x.Id, stepId, StringComparison.Ordinal));
        if (def is null) return Fail("unknownPluginStep", $"Plugin '{h.Id}' has no step '{stepId}'");

        // Pass the client's param values straight through (JsonElement values serialize as-is on the wire).
        var request = new PluginStepRequest { ProgramName = "Test run", StepName = def.Label ?? def.Id, IsBackground = false };
        if (p.TryGetProperty("params", out var pp) && pp.ValueKind == JsonValueKind.Object)
            foreach (var prop in pp.EnumerateObject()) request.Params[prop.Name] = prop.Value.Clone();

        var tcs = new TaskCompletionSource<PluginStepReply>(TaskCreationOptions.RunContinuationsAsynchronously);
        string invocationId;
        try { invocationId = Manager.ExecuteStep(h.Id, stepId, request, reply => tcs.TrySetResult(reply)); }
        catch (PluginNotRunningException) { return Fail("pluginNotRunning", $"Plugin '{h.Id}' is not running"); }

        int timeoutMs = def.TimeoutMs > 0 ? def.TimeoutMs : 30000;
        if (await Task.WhenAny(tcs.Task, Task.Delay(timeoutMs)) != tcs.Task)
        {
            Manager.CancelStep(invocationId, "timeout");
            return Fail("pluginStepTimeout", $"Step '{def.Label ?? def.Id}' timed out");
        }

        var r = await tcs.Task;
        if (!r.Ok) return Fail(r.Error ?? "pluginStepFailed", r.Message ?? "Step failed");
        return Json(new { outputs = r.Outputs });
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
