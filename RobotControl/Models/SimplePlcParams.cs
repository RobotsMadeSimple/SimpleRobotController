using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── SimplePLC device command params ─────────────────────────────────────────────

public class AddSimplePlcDeviceParams
{
    [JsonPropertyName("name")]       public string  Name       { get; set; } = "";
    [JsonPropertyName("host")]       public string  Host       { get; set; } = "";
    [JsonPropertyName("port")]       public int     Port       { get; set; } = SimplePlc.SimplePlcProtocol.DefaultTcpPort;
    [JsonPropertyName("mac")]        public string? Mac        { get; set; }
    [JsonPropertyName("watchdogMs")] public int?    WatchdogMs { get; set; }
    [JsonPropertyName("filterMs")]   public int?    FilterMs   { get; set; }
    [JsonPropertyName("onTimeout")]  public string? OnTimeout  { get; set; }
    [JsonPropertyName("autoConnect")] public bool?  AutoConnect { get; set; }
}

public class SimplePlcDeviceIdParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class SetSimplePlcDeviceConfigParams
{
    [JsonPropertyName("id")]          public string  Id          { get; set; } = "";
    [JsonPropertyName("name")]        public string? Name        { get; set; }
    [JsonPropertyName("host")]        public string? Host        { get; set; }
    [JsonPropertyName("port")]        public int?    Port        { get; set; }
    [JsonPropertyName("mac")]         public string? Mac         { get; set; }
    [JsonPropertyName("autoConnect")] public bool?   AutoConnect { get; set; }
    [JsonPropertyName("watchdogMs")]  public int?    WatchdogMs  { get; set; }
    [JsonPropertyName("filterMs")]    public int?    FilterMs    { get; set; }
    [JsonPropertyName("onTimeout")]   public string? OnTimeout   { get; set; }
}

public class SetSimplePlcDeviceConnectedParams
{
    [JsonPropertyName("id")]        public string Id        { get; set; } = "";
    [JsonPropertyName("connected")] public bool   Connected { get; set; }
}

public class DiscoverSimplePlcParams
{
    [JsonPropertyName("timeoutMs")] public int? TimeoutMs { get; set; }
}

public class IdentifySimplePlcParams
{
    [JsonPropertyName("id")]      public string Id      { get; set; } = "";
    /// <summary>Module slot to blink; 0 = the head's own LED.</summary>
    [JsonPropertyName("slot")]    public int    Slot    { get; set; }
    [JsonPropertyName("seconds")] public int    Seconds { get; set; } = 5;
}

public class SetSimplePlcHeadNameParams
{
    [JsonPropertyName("id")]   public string Id   { get; set; } = "";
    [JsonPropertyName("name")] public string Name { get; set; } = "";
}
