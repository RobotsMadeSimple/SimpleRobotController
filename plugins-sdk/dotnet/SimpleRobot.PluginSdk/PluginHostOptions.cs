namespace SimpleRobot.PluginSdk;

/// <summary>Explicit connection settings, mainly for development against a <c>runtime: external</c> plugin.</summary>
public sealed class PluginHostOptions
{
    /// <summary>e.g. <c>ws://192.168.1.50:5000/plugin</c>.</summary>
    public string Url { get; set; } = "";
    public string Token { get; set; } = "";
    public string PluginId { get; set; } = "";
    public string? PluginDir { get; set; }
    public string? DataDir { get; set; }
    /// <summary>
    /// Sent as <c>manifest</c> in <c>plugin.ready</c> to override the file's steps/functions/properties.
    /// Accepts a JSON string, a JsonElement/JsonNode, or any object (serialized).
    /// </summary>
    public object? ManifestOverride { get; set; }
    /// <summary>First reconnect delay (default 1 s); doubles up to <see cref="MaxReconnectDelay"/>.</summary>
    public TimeSpan MinReconnectDelay { get; set; } = TimeSpan.FromSeconds(1);
    public TimeSpan MaxReconnectDelay { get; set; } = TimeSpan.FromSeconds(30);
    /// <summary>Stop (RunAsync returns) when <see cref="ParentPid"/> exits. Default true.</summary>
    public bool WatchParent { get; set; } = true;
    /// <summary>The controller's pid; defaults to <c>SRC_PARENT_PID</c> when constructed from the environment. Null = no watchdog.</summary>
    public int? ParentPid { get; set; }
}
