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
    }

    public class CameraResolution
    {
        [JsonPropertyName("width")]  public int Width  { get; set; }
        [JsonPropertyName("height")] public int Height { get; set; }
    }
}


