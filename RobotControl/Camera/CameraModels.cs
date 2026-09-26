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
        /// <summary>"usb" | "network" | "sofia".</summary>
        [JsonPropertyName("sourceType")] public string SourceType { get; set; } = "usb";
        /// <summary>Stream URL for network cameras, without credentials.</summary>
        [JsonPropertyName("url")]        public string Url        { get; set; } = "";
        [JsonPropertyName("username")]   public string Username   { get; set; } = "";
        [JsonPropertyName("password")]   public string Password   { get; set; } = "";
        /// <summary>RTSP transport: "tcp" | "udp".</summary>
        [JsonPropertyName("transport")]  public string Transport  { get; set; } = "tcp";

        // Sofia / DVRIP (XMeye) sources — docs/network-cameras.md. username/password above are
        // the Sofia login for these (an empty username logs in as "admin").
        /// <summary>Camera IP / hostname for "sofia" cameras.</summary>
        [JsonPropertyName("host")]       public string Host       { get; set; } = "";
        /// <summary>DVRIP port.</summary>
        [JsonPropertyName("port")]       public int    Port       { get; set; } = 34567;
        /// <summary>"Main" | "Extra1" (sub-stream).</summary>
        [JsonPropertyName("stream")]     public string Stream     { get; set; } = "Main";
        /// <summary>"h264" | "hevc" — what the camera streams (the first frame's detected codec wins).</summary>
        [JsonPropertyName("codec")]      public string Codec      { get; set; } = "h264";
        /// <summary>"opencv" (in-process, loopback socket) | "ffmpeg" (external process).</summary>
        [JsonPropertyName("decoder")]    public string Decoder    { get; set; } = "opencv";
        /// <summary>Executable for the "ffmpeg" decoder.</summary>
        [JsonPropertyName("ffmpegPath")] public string FfmpegPath { get; set; } = "ffmpeg";
        /// <summary>"ffmpeg" decoder only: "" | "auto" | "d3d11va" | ….</summary>
        [JsonPropertyName("hwaccel")]    public string Hwaccel    { get; set; } = "";
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

        [JsonPropertyName("host")]         public string Host         { get; set; } = "";
        [JsonPropertyName("port")]         public int    Port         { get; set; } = 34567;
        [JsonPropertyName("stream")]       public string Stream       { get; set; } = "Main";
        [JsonPropertyName("codec")]        public string Codec        { get; set; } = "h264";
        [JsonPropertyName("decoder")]      public string Decoder      { get; set; } = "opencv";
        [JsonPropertyName("ffmpegPath")]   public string FfmpegPath   { get; set; } = "ffmpeg";
        [JsonPropertyName("hwaccel")]      public string Hwaccel      { get; set; } = "";
    }

    public class CameraResolution
    {
        [JsonPropertyName("width")]  public int Width  { get; set; }
        [JsonPropertyName("height")] public int Height { get; set; }
    }
}


