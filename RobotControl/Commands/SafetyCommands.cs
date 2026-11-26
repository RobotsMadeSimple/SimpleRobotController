using System.Text.Json;
using Controller.RobotControl.Alarms;
using Controller.RobotControl.Safety;

namespace Controller.RobotControl.Commands;

/// <summary>
/// The hardwired e-stop (GetEstopConfig / SetEstopConfig / ResetEstop, docs/estop.md) and the alarm
/// history (GetAlarmCatalog / GetAlarms / AcknowledgeAlarm / ClearAlarms).
/// </summary>
internal sealed class SafetyCommands
{
    private readonly RobotController _robot;

    public SafetyCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetEstopConfig",   GetEstopConfig);
        d.Add("SetEstopConfig",   SetEstopConfig);
        d.Add("ResetEstop",       ResetEstop);
        d.Add("GetAlarmCatalog",  _ => new { alarms = AlarmCatalog.All.Select(CatalogJson).ToList() });
        d.Add("GetAlarms",        GetAlarms);
        d.Add("AcknowledgeAlarm", AcknowledgeAlarm);
        d.Add("ClearAlarms",      ClearAlarms);
    }

    // ── E-stop ───────────────────────────────────────────────────────────────

    internal static object ConfigJson(EstopConfig c) => new
    {
        enabled  = c.Enabled,
        source   = c.Source,
        stbInput = c.StbInput,
        deviceId = c.DeviceId,
        pin      = c.Pin,
        polarity = c.Polarity,
    };

    internal static object StateJson(EstopMonitor.Snapshot s) => new
    {
        enabled       = s.Enabled,
        active        = s.Active,
        latched       = s.Latched,
        sourceOnline  = s.SourceOnline,
        input         = s.Raw,
        inputHealthy  = s.Healthy,
        source        = s.Label,
        reason        = s.Reason,
        trippedUnixMs = s.TrippedUnixMs == 0 ? (long?)null : s.TrippedUnixMs,
    };

    private object? GetEstopConfig(CommandMessage msg)
    {
        var cfg = _robot.Config.Estop ?? new EstopConfig();
        // A fresh read so the live input state is current even before the next safety tick.
        _robot.SafetyTick();
        return new { config = ConfigJson(cfg), state = StateJson(_robot.Estop.State) };
    }

    /// <summary>Patch: only the fields present change. Validated as a whole before anything is saved.</summary>
    private object? SetEstopConfig(CommandMessage msg)
    {
        var next = (_robot.Config.Estop ?? new EstopConfig()).Clone();
        if (msg.Params is { ValueKind: JsonValueKind.Object } p)
        {
            foreach (var prop in p.EnumerateObject())
            {
                var v = prop.Value;
                switch (prop.Name.ToLowerInvariant())
                {
                    case "enabled":
                        if (v.ValueKind is not (JsonValueKind.True or JsonValueKind.False)) return Bad("enabled must be true or false");
                        next.Enabled = v.GetBoolean();
                        break;
                    case "source":
                        next.Source = v.ValueKind == JsonValueKind.String ? v.GetString()! : "";
                        break;
                    case "stbinput":
                        if (v.ValueKind != JsonValueKind.Number || !v.TryGetInt32(out var n)) return Bad("stbInput must be 1..4");
                        next.StbInput = n;
                        break;
                    case "deviceid":
                        next.DeviceId = v.ValueKind == JsonValueKind.String && !string.IsNullOrWhiteSpace(v.GetString()) ? v.GetString() : null;
                        break;
                    case "pin":
                        if (v.ValueKind == JsonValueKind.Null) { next.Pin = null; break; }
                        if (v.ValueKind != JsonValueKind.Number || !v.TryGetInt32(out var pin) || pin < 0) return Bad("pin must be a pin number");
                        next.Pin = pin;
                        break;
                    case "polarity":
                        next.Polarity = v.ValueKind == JsonValueKind.String ? v.GetString()! : "";
                        break;
                }
            }
        }
        if (next.Validate() is { } why) return Bad(why);

        _robot.ApplyEstopConfig(next);
        return new { ok = true, config = ConfigJson(next), state = StateJson(_robot.Estop.State) };
    }

    private object? ResetEstop(CommandMessage msg)
    {
        if (_robot.ResetEstop() is { } why)
            return new { ok = false, error = "estopNotHealthy", message = why, state = StateJson(_robot.Estop.State) };
        return new
        {
            ok = true,
            state = StateJson(_robot.Estop.State),
            message = "E-stop reset. Home the robot before moving it.",
        };
    }

    private static object Bad(string message) => new { ok = false, error = "badConfig", message };

    // ── Alarms ───────────────────────────────────────────────────────────────

    internal static object CatalogJson(AlarmCatalogEntry e) => new
    {
        code      = e.Code,
        title     = e.Title,
        severity  = e.Severity,
        cause     = e.Cause,
        remedy    = e.Remedy,
        catchable = e.Catchable,
    };

    private object? GetAlarms(CommandMessage msg)
    {
        int offset = 0, limit = AlarmHistory.DefaultPageSize;
        bool activeOnly = false;
        if (msg.Params is { ValueKind: JsonValueKind.Object } p)
        {
            if (TryInt(p, "offset", out var o)) offset = o;
            if (TryInt(p, "limit",  out var l)) limit  = l;
            if (p.TryGetProperty("activeOnly", out var a) && a.ValueKind == JsonValueKind.True) activeOnly = true;
        }
        var (total, active, page) = _robot.Alarms.GetPage(offset, limit, activeOnly);
        return new
        {
            total,
            activeCount     = active,
            offset          = Math.Max(0, offset),
            lastAlarmUpdate = _robot.Alarms.LastUpdatedUnixMs,
            alarms          = page.Select(e => new
            {
                id                 = e.Id,
                code               = e.Code,
                severity           = e.Severity,
                title              = e.Title,
                message            = e.Message,
                source             = e.Source,
                timestampUnixMs    = e.TimestampUnixMs,
                acknowledged       = e.Acknowledged,
                acknowledgedUnixMs = e.AcknowledgedUnixMs,
                cleared            = e.Cleared,
                clearedUnixMs      = e.ClearedUnixMs,
                active             = e.Active,
            }).ToList(),
        };
    }

    /// <summary><c>{ id }</c> acknowledges one alarm; <c>{ all: true }</c> (or no id) every alarm.</summary>
    private object? AcknowledgeAlarm(CommandMessage msg)
    {
        string? id = null;
        if (msg.Params is { ValueKind: JsonValueKind.Object } p
            && p.TryGetProperty("id", out var idEl) && idEl.ValueKind == JsonValueKind.String)
            id = idEl.GetString();
        int n = _robot.Alarms.Acknowledge(string.IsNullOrEmpty(id) ? null : id);
        if (n < 0) return new { ok = false, error = "unknownAlarm", message = $"No alarm '{id}'." };
        return new { ok = true, acknowledged = n, activeCount = _robot.Alarms.ActiveCount };
    }

    /// <summary>Removes every alarm whose condition has gone; ones still present stay.</summary>
    private object? ClearAlarms(CommandMessage msg)
    {
        int n = _robot.Alarms.ClearHistory();
        return new { ok = true, removed = n, remaining = _robot.Alarms.Count, activeCount = _robot.Alarms.ActiveCount };
    }

    private static bool TryInt(JsonElement obj, string name, out int value)
    {
        value = 0;
        foreach (var prop in obj.EnumerateObject())
            if (string.Equals(prop.Name, name, StringComparison.OrdinalIgnoreCase)
                && prop.Value.ValueKind == JsonValueKind.Number && prop.Value.TryGetInt32(out value))
                return true;
        return false;
    }
}
