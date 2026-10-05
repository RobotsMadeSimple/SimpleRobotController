using System.Text.Json;
using System.Text.Json.Serialization;
using System.Text.RegularExpressions;

namespace Controller.RobotControl.Plugins;

/// <summary>One problem found while loading or validating a manifest (docs/plugins.md §2).</summary>
/// <param name="Code">Machine-readable code, e.g. <c>badId</c>, <c>unsupportedProtocol</c>.</param>
/// <param name="Message">Human text.</param>
/// <param name="Field">The manifest path the problem is about (<c>steps[0].id</c>), when there is one.</param>
public sealed record ManifestProblem(string Code, string Message, string? Field = null)
{
    public override string ToString() => Field is null ? $"{Code}: {Message}" : $"{Code}: {Message} ({Field})";
}

/// <summary>python-runtime options of a manifest.</summary>
public sealed class PluginPythonOptions
{
    /// <summary>Minimum interpreter version, e.g. "3.9". Null = any.</summary>
    public string? MinVersion { get; set; }
    /// <summary>Requirements file relative to the plugin folder; default <c>requirements.txt</c>.</summary>
    public string? Requirements { get; set; }
}

/// <summary>Restart policy of a manifest.</summary>
public sealed class PluginRestartPolicy
{
    /// <summary><c>always</c> (default), <c>onFailure</c> or <c>never</c>.</summary>
    public string Mode { get; set; } = "always";
    /// <summary>Restarts allowed within 10 minutes before the plugin is left <c>crashed</c>.</summary>
    public int MaxRestarts { get; set; } = 5;
    /// <summary>First backoff delay; doubles per consecutive crash, capped at 30 s.</summary>
    public int BackoffMs { get; set; } = 2000;
}

/// <summary>One field of a plugin's config form.</summary>
public sealed class PluginConfigField
{
    public string Key { get; set; } = "";
    public string? Label { get; set; }
    /// <summary><c>string</c>, <c>password</c>, <c>number</c>, <c>boolean</c> or <c>enum</c>.</summary>
    public string Type { get; set; } = "string";
    public JsonElement? Default { get; set; }
    public string? Help { get; set; }
    public double? Min { get; set; }
    public double? Max { get; set; }
    public double? Step { get; set; }
    public List<string>? Options { get; set; }
    public bool Required { get; set; }
}

/// <summary>One parameter of a plugin step.</summary>
public sealed class PluginStepParam
{
    public string Key { get; set; } = "";
    public string? Label { get; set; }
    /// <summary><c>number boolean string enum point list image variable</c>.</summary>
    public string Type { get; set; } = "string";
    public JsonElement? Default { get; set; }
    public string? Help { get; set; }
    public double? Min { get; set; }
    public double? Max { get; set; }
    public double? Step { get; set; }
    public List<string>? Options { get; set; }
    public bool Required { get; set; }
}

/// <summary>One output of a plugin step.</summary>
public sealed class PluginStepOutput
{
    public string Key { get; set; } = "";
    public string? Label { get; set; }
    /// <summary><c>number boolean string point list image</c>.</summary>
    public string Type { get; set; } = "number";
}

/// <summary>A program step a plugin provides.</summary>
public sealed class PluginStepDef
{
    public string Id { get; set; } = "";
    public string? Label { get; set; }
    public string? Description { get; set; }
    public List<PluginStepParam> Params { get; set; } = new();
    public List<PluginStepOutput> Outputs { get; set; } = new();
    /// <summary>0 = no timeout.</summary>
    public int TimeoutMs { get; set; }
    public bool Cancellable { get; set; } = true;
}

/// <summary>An expression function a plugin provides (called as <c>&lt;id&gt;.&lt;name&gt;(…)</c>).</summary>
public sealed class PluginFunctionDef
{
    public string Name { get; set; } = "";
    public string? Signature { get; set; }
    public string? Description { get; set; }
    public int MinArgs { get; set; }
    /// <summary>Null = same as <see cref="MinArgs"/>.</summary>
    public int? MaxArgs { get; set; }
    /// <summary>Null = default 250 ms; max 5000.</summary>
    public int? TimeoutMs { get; set; }

    [JsonIgnore] public int EffectiveMaxArgs => MaxArgs ?? MinArgs;
    [JsonIgnore] public int EffectiveTimeoutMs => TimeoutMs is > 0 ? TimeoutMs.Value : PluginManifest.DefaultFunctionTimeoutMs;
}

/// <summary>A live property a plugin publishes (read as <c>$&lt;id&gt;.&lt;name&gt;</c>).</summary>
public sealed class PluginPropertyDef
{
    public string Name { get; set; } = "";
    public string? Description { get; set; }
    /// <summary><c>number</c> or <c>boolean</c>.</summary>
    public string Type { get; set; } = "number";
}

/// <summary>
/// The parsed <c>plugin.json</c> (docs/plugins.md §2). <see cref="Validate"/> returns every
/// rule violation; a manifest with problems leaves its plugin in state <c>error</c>.
/// </summary>
public sealed class PluginManifest
{
    public const int SupportedProtocolVersion = 1;
    public const int DefaultFunctionTimeoutMs = 250;
    public const int MaxFunctionTimeoutMs     = 5000;
    public const int DefaultReadyTimeoutMs    = 15000;

    /// <summary>Expression roots a plugin id may not take.</summary>
    public static readonly IReadOnlySet<string> ReservedIds = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
    {
        "robot", "program", "time", "aux", "camera", "stb", "relay", "nano", "plugin", "global", "local", "list", "time_ms",
    };

    public static readonly IReadOnlySet<string> Runtimes        = Set("python", "dotnet", "exe", "external");
    public static readonly IReadOnlySet<string> RestartModes    = Set("always", "onFailure", "never");
    public static readonly IReadOnlySet<string> ConfigTypes     = Set("string", "password", "number", "boolean", "enum");
    public static readonly IReadOnlySet<string> ParamTypes      = Set("number", "boolean", "string", "enum", "point", "list", "image", "variable");
    public static readonly IReadOnlySet<string> OutputTypes     = Set("number", "boolean", "string", "point", "list", "image");
    public static readonly IReadOnlySet<string> PropertyTypes   = Set("number", "boolean");

    private static readonly Regex IdRegex     = new("^[a-z][a-z0-9_]{1,31}$", RegexOptions.CultureInvariant);
    private static readonly Regex MemberRegex = new("^[a-z][A-Za-z0-9_]{0,31}$", RegexOptions.CultureInvariant);

    public string Id { get; set; } = "";
    public string? Name { get; set; }
    public string? Version { get; set; }
    public string? Description { get; set; }
    public string? Author { get; set; }
    public int ProtocolVersion { get; set; }

    public string Runtime { get; set; } = "";
    public string? Entry { get; set; }
    public List<string> Args { get; set; } = new();
    public PluginPythonOptions? Python { get; set; }

    public bool AutoStart { get; set; } = true;
    public PluginRestartPolicy Restart { get; set; } = new();
    public int ReadyTimeoutMs { get; set; } = DefaultReadyTimeoutMs;

    public List<PluginConfigField> ConfigSchema { get; set; } = new();
    public List<PluginStepDef> Steps { get; set; } = new();
    public List<PluginFunctionDef> Functions { get; set; } = new();
    public List<PluginPropertyDef> Properties { get; set; } = new();

    /// <summary>The display name: <see cref="Name"/>, or the id when absent.</summary>
    [JsonIgnore] public string DisplayName => string.IsNullOrWhiteSpace(Name) ? Id : Name!;

    /// <summary>True when the controller launches nothing (development mode).</summary>
    [JsonIgnore] public bool IsExternal => string.Equals(Runtime, "external", StringComparison.Ordinal);

    /// <summary>Parses manifest JSON. Throws <see cref="JsonException"/> on malformed input.</summary>
    public static PluginManifest Parse(string json) =>
        JsonSerializer.Deserialize<PluginManifest>(json, PluginJson.Options)
        ?? throw new JsonException("Manifest is null");

    /// <summary>A deep copy (via JSON).</summary>
    public PluginManifest Clone() => Parse(JsonSerializer.Serialize(this, PluginJson.Options));

    /// <summary>
    /// Every rule violation of docs/plugins.md §2. <paramref name="isFunctionName"/> answers whether a
    /// name is an existing expression function (defaults to <see cref="ExpressionEvaluator.IsFunctionName"/>).
    /// </summary>
    public List<ManifestProblem> Validate(Func<string, bool>? isFunctionName = null)
    {
        isFunctionName ??= ExpressionEvaluator.IsFunctionName;
        var problems = new List<ManifestProblem>();
        void Add(string code, string message, string? field = null) => problems.Add(new ManifestProblem(code, message, field));

        // ── id ──
        if (string.IsNullOrEmpty(Id) || !IdRegex.IsMatch(Id))
            Add("badId", $"Plugin id '{Id}' must match ^[a-z][a-z0-9_]{{1,31}}$", "id");
        else if (ReservedIds.Contains(Id))
            Add("reservedId", $"Plugin id '{Id}' is a reserved expression root", "id");
        else if (isFunctionName(Id))
            Add("reservedId", $"Plugin id '{Id}' is an existing expression function name", "id");

        // ── protocol / runtime ──
        if (ProtocolVersion != SupportedProtocolVersion)
            Add("unsupportedProtocol", $"protocolVersion {ProtocolVersion} is not supported (expected {SupportedProtocolVersion})", "protocolVersion");

        if (!Runtimes.Contains(Runtime ?? ""))
            Add("badRuntime", $"runtime '{Runtime}' must be one of python, dotnet, exe, external", "runtime");
        else if (!IsExternal)
        {
            if (string.IsNullOrWhiteSpace(Entry))
                Add("missingEntry", "entry is required unless runtime is external", "entry");
            else if (!IsInsideFolder(Entry))
                Add("entryOutsideFolder", $"entry '{Entry}' must stay inside the plugin folder", "entry");
        }

        if (Python?.Requirements is { Length: > 0 } req && !IsInsideFolder(req))
            Add("entryOutsideFolder", $"python.requirements '{req}' must stay inside the plugin folder", "python.requirements");
        if (Python?.MinVersion is { Length: > 0 } mv && !System.Version.TryParse(NormalizeVersion(mv), out _))
            Add("badPythonVersion", $"python.minVersion '{mv}' is not a version", "python.minVersion");

        // ── restart / timing ──
        if (Restart is null)
            Restart = new PluginRestartPolicy();
        if (!RestartModes.Contains(Restart.Mode ?? ""))
            Add("badRestart", $"restart.mode '{Restart.Mode}' must be always, onFailure or never", "restart.mode");
        if (Restart.MaxRestarts < 0)
            Add("badRestart", "restart.maxRestarts must be >= 0", "restart.maxRestarts");
        if (Restart.BackoffMs < 0)
            Add("badRestart", "restart.backoffMs must be >= 0", "restart.backoffMs");
        if (ReadyTimeoutMs <= 0)
            Add("badReadyTimeout", "readyTimeoutMs must be > 0", "readyTimeoutMs");

        // ── config schema ──
        var configKeys = new HashSet<string>(StringComparer.Ordinal);
        for (int i = 0; i < (ConfigSchema?.Count ?? 0); i++)
        {
            var f = ConfigSchema![i];
            string at = $"configSchema[{i}]";
            if (string.IsNullOrWhiteSpace(f.Key))
                Add("badConfigField", "config field key is required", $"{at}.key");
            else if (!configKeys.Add(f.Key))
                Add("duplicateConfigField", $"config field '{f.Key}' is declared twice", $"{at}.key");
            if (!ConfigTypes.Contains(f.Type ?? ""))
                Add("badConfigField", $"config field type '{f.Type}' must be string, password, number, boolean or enum", $"{at}.type");
            if (f.Type == "enum" && (f.Options is null || f.Options.Count == 0))
                Add("badConfigField", $"enum config field '{f.Key}' needs options", $"{at}.options");
            if (f.Default is { } d && PluginConfigValidator.CheckValue(f, d) is { } err)
                Add("badConfigField", $"default of '{f.Key}': {err}", $"{at}.default");
        }

        // ── steps ──
        var stepIds = new HashSet<string>(StringComparer.Ordinal);
        for (int i = 0; i < (Steps?.Count ?? 0); i++)
        {
            var s = Steps![i];
            string at = $"steps[{i}]";
            if (string.IsNullOrEmpty(s.Id) || !MemberRegex.IsMatch(s.Id))
                Add("badStepId", $"step id '{s.Id}' must match ^[a-z][A-Za-z0-9_]{{0,31}}$", $"{at}.id");
            else if (!stepIds.Add(s.Id))
                Add("duplicateStepId", $"step id '{s.Id}' is declared twice", $"{at}.id");
            if (s.TimeoutMs < 0)
                Add("badStepTimeout", "step timeoutMs must be >= 0", $"{at}.timeoutMs");

            var paramKeys = new HashSet<string>(StringComparer.Ordinal);
            for (int j = 0; j < (s.Params?.Count ?? 0); j++)
            {
                var p = s.Params![j];
                string pat = $"{at}.params[{j}]";
                if (string.IsNullOrWhiteSpace(p.Key))
                    Add("badParam", "param key is required", $"{pat}.key");
                else if (!paramKeys.Add(p.Key))
                    Add("duplicateParam", $"param '{p.Key}' is declared twice", $"{pat}.key");
                if (!ParamTypes.Contains(p.Type ?? ""))
                    Add("badParamType", $"param type '{p.Type}' is not one of {string.Join(", ", ParamTypes)}", $"{pat}.type");
                if (p.Type == "enum" && (p.Options is null || p.Options.Count == 0))
                    Add("badParam", $"enum param '{p.Key}' needs options", $"{pat}.options");
            }

            var outputKeys = new HashSet<string>(StringComparer.Ordinal);
            for (int j = 0; j < (s.Outputs?.Count ?? 0); j++)
            {
                var o = s.Outputs![j];
                string oat = $"{at}.outputs[{j}]";
                if (string.IsNullOrWhiteSpace(o.Key))
                    Add("badOutput", "output key is required", $"{oat}.key");
                else if (!outputKeys.Add(o.Key))
                    Add("duplicateOutput", $"output '{o.Key}' is declared twice", $"{oat}.key");
                if (!OutputTypes.Contains(o.Type ?? ""))
                    Add("badOutputType", $"output type '{o.Type}' is not one of {string.Join(", ", OutputTypes)}", $"{oat}.type");
            }
        }

        // ── functions ──
        var fnNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        for (int i = 0; i < (Functions?.Count ?? 0); i++)
        {
            var f = Functions![i];
            string at = $"functions[{i}]";
            if (string.IsNullOrEmpty(f.Name) || !MemberRegex.IsMatch(f.Name))
                Add("badFunctionName", $"function name '{f.Name}' must match ^[a-z][A-Za-z0-9_]{{0,31}}$", $"{at}.name");
            else if (!fnNames.Add(f.Name))
                Add("duplicateFunction", $"function '{f.Name}' is declared twice", $"{at}.name");
            if (f.MinArgs < 0 || f.EffectiveMaxArgs < f.MinArgs)
                Add("badArity", $"function '{f.Name}' needs 0 <= minArgs <= maxArgs", $"{at}.maxArgs");
            if (f.TimeoutMs is { } t && (t <= 0 || t > MaxFunctionTimeoutMs))
                Add("badFunctionTimeout", $"function timeoutMs must be 1..{MaxFunctionTimeoutMs}", $"{at}.timeoutMs");
        }

        // ── properties ──
        var propNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        for (int i = 0; i < (Properties?.Count ?? 0); i++)
        {
            var p = Properties![i];
            string at = $"properties[{i}]";
            if (string.IsNullOrEmpty(p.Name) || !MemberRegex.IsMatch(p.Name))
                Add("badPropertyName", $"property name '{p.Name}' must match ^[a-z][A-Za-z0-9_]{{0,31}}$", $"{at}.name");
            else if (!propNames.Add(p.Name))
                Add("duplicateProperty", $"property '{p.Name}' is declared twice", $"{at}.name");
            if (!PropertyTypes.Contains(p.Type ?? ""))
                Add("badPropertyType", $"property type '{p.Type}' must be number or boolean", $"{at}.type");
        }

        return problems;
    }

    /// <summary>
    /// Applies a <c>plugin.ready</c> manifest override: its <c>steps/functions/properties</c>
    /// lists (when present) replace this manifest's. Returns a new manifest.
    /// </summary>
    public PluginManifest WithOverride(JsonElement overrideManifest)
    {
        var copy = Clone();
        if (overrideManifest.ValueKind != JsonValueKind.Object) return copy;
        foreach (var prop in overrideManifest.EnumerateObject())
        {
            switch (prop.Name.ToLowerInvariant())
            {
                case "steps":      copy.Steps      = prop.Value.Deserialize<List<PluginStepDef>>(PluginJson.Options) ?? new(); break;
                case "functions":  copy.Functions  = prop.Value.Deserialize<List<PluginFunctionDef>>(PluginJson.Options) ?? new(); break;
                case "properties": copy.Properties = prop.Value.Deserialize<List<PluginPropertyDef>>(PluginJson.Options) ?? new(); break;
            }
        }
        return copy;
    }

    /// <summary>A relative path with no rooted prefix and no <c>..</c> segment.</summary>
    internal static bool IsInsideFolder(string relative)
    {
        if (string.IsNullOrWhiteSpace(relative) || Path.IsPathRooted(relative)) return false;
        if (relative.StartsWith('/') || relative.StartsWith('\\')) return false;
        var parts = relative.Split('/', '\\');
        return !parts.Any(p => p == "..");
    }

    /// <summary>"3" → "3.0" so <see cref="System.Version"/> can parse it.</summary>
    internal static string NormalizeVersion(string v)
    {
        v = v.Trim();
        return v.Contains('.') ? v : v + ".0";
    }

    private static IReadOnlySet<string> Set(params string[] items) => new HashSet<string>(items, StringComparer.Ordinal);
}
