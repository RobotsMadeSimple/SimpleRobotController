using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Plugins;

/// <summary>
/// Serializer settings for everything plugin-related: manifests on disk, the plugin
/// protocol frames, and the WebSocket command payloads. camelCase both ways, enums as
/// camelCase strings.
/// </summary>
public static class PluginJson
{
    /// <summary>camelCase, case-insensitive read, enums as camelCase strings, nulls written.</summary>
    public static readonly JsonSerializerOptions Options = new()
    {
        PropertyNamingPolicy        = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        NumberHandling              = JsonNumberHandling.AllowNamedFloatingPointLiterals,
        ReadCommentHandling         = JsonCommentHandling.Skip,
        AllowTrailingCommas         = true,
        Converters                  = { new JsonStringEnumConverter(JsonNamingPolicy.CamelCase) },
    };

    /// <summary>Same as <see cref="Options"/> but indented — used for files written to disk.</summary>
    public static readonly JsonSerializerOptions Indented = new(Options) { WriteIndented = true };

    /// <summary>Serializes <paramref name="value"/> to a detached <see cref="JsonElement"/> with <see cref="Options"/>.</summary>
    public static JsonElement ToElement(object? value) => JsonSerializer.SerializeToElement(value, Options);

    /// <summary>An empty JSON object element.</summary>
    public static JsonElement EmptyObject => JsonSerializer.SerializeToElement(new { });
}
