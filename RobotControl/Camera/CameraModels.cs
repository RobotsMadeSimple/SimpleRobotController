using System.Collections.Generic;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Camera
{
    public class CameraConfig
    {
        [JsonPropertyName("id")]         public string Id          { get; set; } = "";
        [JsonPropertyName("name")]       public string Name        { get; set; } = "";
        [JsonPropertyName("deviceIndex")]public int    DeviceIndex { get; set; } = 0;
        [JsonPropertyName("enabled")]    public bool   Enabled     { get; set; } = true;
        [JsonPropertyName("width")]      public int    Width       { get; set; } = 640;
        [JsonPropertyName("height")]     public int    Height      { get; set; } = 480;
        [JsonPropertyName("targetFps")]          public int                  TargetFps          { get; set; } = 15;
        [JsonPropertyName("supportedResolutions")] public List<CameraResolution> SupportedResolutions { get; set; } = new();

        // Network (RTSP/HTTP) sources — docs/network-cameras.md. Absent in older files = USB.
        /// <summary>"usb" | "network".</summary>
        [JsonPropertyName("sourceType")] public string SourceType { get; set; } = "usb";
        /// <summary>Stream URL for network cameras, without credentials.</summary>
        [JsonPropertyName("url")]        public string Url        { get; set; } = "";
        [JsonPropertyName("username")]   public string Username   { get; set; } = "";
        [JsonPropertyName("password")]   public string Password   { get; set; } = "";
        /// <summary>RTSP transport: "tcp" | "udp".</summary>
        [JsonPropertyName("transport")]  public string Transport  { get; set; } = "tcp";
    }

    public class CameraManagerConfig
    {
        [JsonPropertyName("cameras")]
        public List<CameraConfig> Cameras { get; set; } = new();
    }

    public class CameraState
    {
        [JsonPropertyName("id")]          public string Id          { get; set; } = "";
        [JsonPropertyName("name")]        public string Name        { get; set; } = "";
        [JsonPropertyName("connected")]   public bool   Connected   { get; set; }
        [JsonPropertyName("deviceIndex")] public int    DeviceIndex { get; set; }
        [JsonPropertyName("width")]       public int    Width       { get; set; }
        [JsonPropertyName("height")]      public int    Height      { get; set; }
        [JsonPropertyName("targetFps")]   public int    TargetFps   { get; set; }
        [JsonPropertyName("enabled")]              public bool                 Enabled              { get; set; }
        [JsonPropertyName("supportedResolutions")] public List<CameraResolution> SupportedResolutions { get; set; } = new();
        /// <summary>A camera-to-robot calibration is saved for this camera (docs/camera-calibration.md).</summary>
        [JsonPropertyName("calibrated")]           public bool                 Calibrated           { get; set; }
        /// <summary>When the saved calibration was made; null when not calibrated.</summary>
        [JsonPropertyName("calibratedUnixMs")]     public long?                CalibratedUnixMs     { get; set; }

        [JsonPropertyName("sourceType")]   public string SourceType   { get; set; } = "usb";
        [JsonPropertyName("url")]          public string Url          { get; set; } = "";
        [JsonPropertyName("username")]     public string Username     { get; set; } = "";
        [JsonPropertyName("password")]     public string Password     { get; set; } = "";
        [JsonPropertyName("transport")]    public string Transport    { get; set; } = "tcp";
        /// <summary>Size the stream actually delivers (0 until the first frame).</summary>
        [JsonPropertyName("streamWidth")]  public int    StreamWidth  { get; set; }
        [JsonPropertyName("streamHeight")] public int    StreamHeight { get; set; }
        /// <summary>Best-effort decode latency estimate for network streams; 0 when unknown.</summary>
        [JsonPropertyName("latencyMs")]    public int    LatencyMs    { get; set; }
    }

    public class CameraResolution
    {
        [JsonPropertyName("width")]  public int Width  { get; set; }
        [JsonPropertyName("height")] public int Height { get; set; }
    }
}


