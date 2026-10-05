using System.Security.Cryptography;
using System.Text.Json;
using Controller.RobotControl.Persistence;

namespace Controller.RobotControl.Plugins;

/// <summary>Contents of <c>pluginConfigs/&lt;id&gt;.json</c>.</summary>
public sealed class PluginConfigFile
{
    public bool Enabled { get; set; } = true;
    public string Token { get; set; } = "";
    public Dictionary<string, JsonElement> Config { get; set; } = new(StringComparer.Ordinal);
}

/// <summary>
/// Per-plugin persisted settings (enabled flag, connect token, config values) in
/// <c>&lt;data dir&gt;/pluginConfigs/&lt;id&gt;.json</c>. Written atomically; survives reinstall;
/// deleted on uninstall. Thread-safe.
/// </summary>
public sealed class PluginConfigStore
{
    private readonly string _dir;
    private readonly object _lock = new();

    public PluginConfigStore(string directory) => _dir = directory;

    public string Directory => _dir;

    public string PathFor(string id) => Path.Combine(_dir, id + ".json");

    /// <summary>Loads the file, creating it (enabled, fresh token) when absent or unreadable.</summary>
    public PluginConfigFile LoadOrCreate(string id)
    {
        lock (_lock)
        {
            var file = JsonFiles.Load<PluginConfigFile>(PathFor(id), PluginJson.Indented, "Plugins");
            bool dirty = false;
            if (file == null) { file = new PluginConfigFile(); dirty = true; }
            file.Config ??= new(StringComparer.Ordinal);
            if (string.IsNullOrEmpty(file.Token)) { file.Token = NewToken(); dirty = true; }
            if (dirty) SaveLocked(id, file);
            return file;
        }
    }

    /// <summary>Writes the file atomically.</summary>
    public void Save(string id, PluginConfigFile file)
    {
        lock (_lock) SaveLocked(id, file);
    }

    /// <summary>Deletes the file (uninstall).</summary>
    public void Delete(string id)
    {
        lock (_lock)
        {
            try { File.Delete(PathFor(id)); } catch (IOException) { } catch (UnauthorizedAccessException) { }
        }
    }

    /// <summary>Random 32 bytes, base64url without padding.</summary>
    public static string NewToken()
    {
        var bytes = RandomNumberGenerator.GetBytes(32);
        return Convert.ToBase64String(bytes).TrimEnd('=').Replace('+', '-').Replace('/', '_');
    }

    private void SaveLocked(string id, PluginConfigFile file)
    {
        System.IO.Directory.CreateDirectory(_dir);
        JsonFiles.Save(PathFor(id), file, PluginJson.Indented);
    }
}

/// <summary>Validation of config values against a manifest's <c>configSchema</c>.</summary>
public static class PluginConfigValidator
{
    /// <summary>Null when <paramref name="value"/> fits <paramref name="field"/>, else a human reason.</summary>
    public static string? CheckValue(PluginConfigField field, JsonElement value)
    {
        switch (field.Type)
        {
            case "string":
            case "password":
                if (value.ValueKind != JsonValueKind.String) return "must be a string";
                if (field.Required && string.IsNullOrEmpty(value.GetString())) return "is required";
                return null;
            case "enum":
                if (value.ValueKind != JsonValueKind.String) return "must be a string";
                if (field.Options is { Count: > 0 } opts && !opts.Contains(value.GetString()!)) return $"must be one of {string.Join(", ", opts)}";
                return null;
            case "number":
                if (value.ValueKind != JsonValueKind.Number) return "must be a number";
                double d = value.GetDouble();
                if (field.Min is { } min && d < min) return $"must be >= {min}";
                if (field.Max is { } max && d > max) return $"must be <= {max}";
                return null;
            case "boolean":
                return value.ValueKind is JsonValueKind.True or JsonValueKind.False ? null : "must be true or false";
            default:
                return null;
        }
    }

    /// <summary>
    /// Validates a full config object. Returns (field, message) of the first problem, or null when valid.
    /// Unknown keys are rejected; a missing required field is reported unless it has a default.
    /// </summary>
    public static (string Field, string Message)? Validate(IReadOnlyList<PluginConfigField> schema, IReadOnlyDictionary<string, JsonElement> config)
    {
        var byKey = schema.ToDictionary(f => f.Key, StringComparer.Ordinal);
        foreach (var (key, value) in config)
        {
            if (!byKey.TryGetValue(key, out var field)) return (key, $"'{key}' is not a config field");
            if (value.ValueKind == JsonValueKind.Null) continue; // null = use the default
            if (CheckValue(field, value) is { } err) return (key, $"'{field.Label ?? key}' {err}");
        }
        foreach (var f in schema)
        {
            if (!f.Required) continue;
            bool present = config.TryGetValue(f.Key, out var v) && v.ValueKind != JsonValueKind.Null;
            if (!present && f.Default is null) return (f.Key, $"'{f.Label ?? f.Key}' is required");
        }
        return null;
    }

    /// <summary>The stored config with schema defaults filled in for absent keys.</summary>
    public static Dictionary<string, JsonElement> MergeDefaults(IReadOnlyList<PluginConfigField> schema, IReadOnlyDictionary<string, JsonElement> stored)
    {
        var merged = new Dictionary<string, JsonElement>(StringComparer.Ordinal);
        foreach (var f in schema)
            if (f.Default is { } d) merged[f.Key] = d.Clone();
        foreach (var (k, v) in stored)
            if (v.ValueKind != JsonValueKind.Null) merged[k] = v.Clone();
        return merged;
    }
}
