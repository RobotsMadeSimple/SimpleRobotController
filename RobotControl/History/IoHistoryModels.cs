using System.Text.Json.Serialization;

namespace Controller.RobotControl.History;

/// <summary>
/// One timestamped reading on a history channel. <see cref="Value"/> is the calibrated /
/// primary number (grams for a load cell); <see cref="Raw"/> is the uncalibrated reading
/// (HX711 counts) so the app can plot raw while a cell is uncalibrated and switch to grams
/// once it is calibrated. Kept as a struct so the per-channel ring buffer stays compact.
/// </summary>
public readonly struct IoSample
{
    public long   UnixMs { get; }
    public double Value  { get; }
    public double Raw    { get; }
    public IoSample(long unixMs, double value, double raw) { UnixMs = unixMs; Value = value; Raw = raw; }
}

/// <summary>Wire form of a sample — short keys keep high-rate (10–80 Hz) batches small.</summary>
public sealed class IoSampleDto
{
    [JsonPropertyName("t")] public long   T { get; set; }   // unix ms
    [JsonPropertyName("v")] public double V { get; set; }   // value (grams for a load cell)
    [JsonPropertyName("r")] public double R { get; set; }   // raw reading
}

/// <summary>Metadata for one recording (an explicit start→stop capture of a channel).</summary>
public sealed class IoRecordingMeta
{
    [JsonPropertyName("id")]          public string Id          { get; set; } = "";
    [JsonPropertyName("channelKey")]  public string ChannelKey  { get; set; } = "";
    [JsonPropertyName("name")]        public string Name        { get; set; } = "";
    [JsonPropertyName("unit")]        public string Unit        { get; set; } = "g";
    [JsonPropertyName("label")]       public string Label       { get; set; } = "";
    [JsonPropertyName("startUnixMs")] public long   StartUnixMs { get; set; }
    [JsonPropertyName("endUnixMs")]   public long?  EndUnixMs   { get; set; }
    [JsonPropertyName("sampleCount")] public int    SampleCount { get; set; }
    [JsonPropertyName("active")]      public bool   Active      { get; set; }
    [JsonPropertyName("saved")]       public bool   Saved       { get; set; }
}

/// <summary>A span of samples for a channel or recording, server-side decimated to the
/// requested point budget. Used by GetIoHistoryWindow / GetIoRecordingWindow.</summary>
public sealed class IoHistoryWindow
{
    [JsonPropertyName("channelKey")] public string          Key     { get; set; } = "";
    [JsonPropertyName("unit")]       public string          Unit    { get; set; } = "g";
    [JsonPropertyName("fromUnixMs")] public long            From    { get; set; }
    [JsonPropertyName("toUnixMs")]   public long            To      { get; set; }
    [JsonPropertyName("samples")]    public List<IoSampleDto> Samples { get; set; } = new();
}

/// <summary>Persisted service settings (how long the rolling in-RAM history is kept).</summary>
public sealed class IoHistoryConfig
{
    [JsonPropertyName("maxRetentionMs")] public long MaxRetentionMs { get; set; } = 600_000; // 10 min
}

/// <summary>On-disk form of a saved recording (meta + its full sample list).</summary>
public sealed class StoredRecording
{
    [JsonPropertyName("meta")]    public IoRecordingMeta  Meta    { get; set; } = new();
    [JsonPropertyName("samples")] public List<IoSampleDto> Samples { get; set; } = new();
}
