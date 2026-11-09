using System.Collections.Generic;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Devices
{
    /// <summary>
    /// A phone (or other client) that pushes named data channels into the controller over
    /// <c>/devices/{id}/ingest</c>. Only the identity (id/name/enabled) is persisted; the live
    /// channel values are in-memory and arrive at runtime. Channels surface in expressions as
    /// <c>$device.&lt;id&gt;.&lt;channel&gt;</c> (see <c>RobotPropertySource</c>).
    /// </summary>
    public sealed class DeviceConfig
    {
        [JsonPropertyName("id")]      public string Id      { get; set; } = "";
        [JsonPropertyName("name")]    public string Name    { get; set; } = "";
        [JsonPropertyName("enabled")] public bool   Enabled { get; set; } = true;
        /// <summary>
        /// Opaque provider settings owned by the app (a JSON blob: capture transport, resolution,
        /// frame rate, orientation lock, which sensors stream). The controller stores and echoes it
        /// verbatim so any phone that provides for this device pulls the same config on connect — it
        /// does not interpret the contents. Null/empty until the app first saves it.
        /// </summary>
        [JsonPropertyName("providerConfig")] public string? ProviderConfig { get; set; }
    }

    public sealed class DeviceManagerConfig
    {
        [JsonPropertyName("devices")] public List<DeviceConfig> Devices { get; set; } = new();
    }

    /// <summary>The latest value of one pushed channel.</summary>
    public sealed class DeviceChannelState
    {
        [JsonPropertyName("key")]    public string  Key    { get; set; } = "";
        [JsonPropertyName("value")]  public double  Value  { get; set; }
        /// <summary>Optional unit label from the phone (e.g. "m/s²", "lux"); informational only.</summary>
        [JsonPropertyName("unit")]   public string? Unit   { get; set; }
        /// <summary>Phone-supplied timestamp of the sample (Unix ms).</summary>
        [JsonPropertyName("unixMs")] public long    UnixMs { get; set; }
    }

    public sealed class DeviceState
    {
        [JsonPropertyName("id")]        public string Id        { get; set; } = "";
        [JsonPropertyName("name")]      public string Name      { get; set; } = "";
        [JsonPropertyName("enabled")]   public bool   Enabled   { get; set; }
        /// <summary>True while channel data has arrived recently (push freshness).</summary>
        [JsonPropertyName("connected")] public bool   Connected { get; set; }
        /// <summary>App-owned provider settings blob (see <see cref="DeviceConfig.ProviderConfig"/>); null until saved.</summary>
        [JsonPropertyName("providerConfig")] public string? ProviderConfig { get; set; }
        [JsonPropertyName("channels")]  public List<DeviceChannelState> Channels { get; set; } = new();
    }
}
