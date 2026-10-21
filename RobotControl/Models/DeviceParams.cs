using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── Device command params (phone data devices) ─────────────────────────────────

public class AddDeviceParams
{
    [JsonPropertyName("name")]    public string Name    { get; set; } = "Phone";
    [JsonPropertyName("enabled")] public bool   Enabled { get; set; } = true;
}

public class RemoveDeviceParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class SetDeviceConfigParams
{
    [JsonPropertyName("id")]      public string  Id      { get; set; } = "";
    // null (absent) keeps the device's current value.
    [JsonPropertyName("name")]    public string? Name    { get; set; }
    [JsonPropertyName("enabled")] public bool?   Enabled { get; set; }
    /// <summary>App-owned provider settings JSON (stored verbatim); null keeps the current blob.</summary>
    [JsonPropertyName("providerConfig")] public string? ProviderConfig { get; set; }
}
