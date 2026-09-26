using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── Camera command params ─────────────────────────────────────────────────────

public class AddCameraParams
{
    [JsonPropertyName("name")]        public string Name        { get; set; } = "Camera";
    [JsonPropertyName("deviceIndex")] public int    DeviceIndex { get; set; } = 0;
    [JsonPropertyName("enabled")]     public bool   Enabled     { get; set; } = true;
    [JsonPropertyName("width")]       public int    Width       { get; set; } = 640;
    [JsonPropertyName("height")]      public int    Height      { get; set; } = 480;
    [JsonPropertyName("targetFps")]   public int    TargetFps   { get; set; } = 15;
    [JsonPropertyName("sourceType")]  public string SourceType  { get; set; } = "usb";
    [JsonPropertyName("url")]         public string Url         { get; set; } = "";
    [JsonPropertyName("username")]    public string Username    { get; set; } = "";
    [JsonPropertyName("password")]    public string Password    { get; set; } = "";
    [JsonPropertyName("transport")]   public string Transport   { get; set; } = "tcp";
    // Sofia / DVRIP source fields (sourceType "sofia")
    [JsonPropertyName("host")]        public string Host        { get; set; } = "";
    [JsonPropertyName("port")]        public int    Port        { get; set; } = 34567;
    [JsonPropertyName("stream")]      public string Stream      { get; set; } = "Main";
    [JsonPropertyName("codec")]       public string Codec       { get; set; } = "h264";
    [JsonPropertyName("decoder")]     public string Decoder     { get; set; } = "opencv";
    [JsonPropertyName("ffmpegPath")]  public string FfmpegPath  { get; set; } = "ffmpeg";
    [JsonPropertyName("hwaccel")]     public string Hwaccel     { get; set; } = "";
}

public class RemoveCameraParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class SetCameraConfigParams
{
    [JsonPropertyName("id")]          public string Id          { get; set; } = "";
    [JsonPropertyName("name")]        public string Name        { get; set; } = "Camera";
    [JsonPropertyName("deviceIndex")] public int    DeviceIndex { get; set; } = 0;
    [JsonPropertyName("enabled")]     public bool   Enabled     { get; set; } = true;
    [JsonPropertyName("width")]       public int    Width       { get; set; } = 640;
    [JsonPropertyName("height")]      public int    Height      { get; set; } = 480;
    [JsonPropertyName("targetFps")]   public int    TargetFps   { get; set; } = 15;
    // Network source fields: null (absent) keeps the camera's current value, so an app
    // that predates network cameras cannot turn one back into a USB camera by saving it.
    [JsonPropertyName("sourceType")]  public string? SourceType { get; set; }
    [JsonPropertyName("url")]         public string? Url        { get; set; }
    [JsonPropertyName("username")]    public string? Username   { get; set; }
    [JsonPropertyName("password")]    public string? Password   { get; set; }
    [JsonPropertyName("transport")]   public string? Transport  { get; set; }
    // Sofia source fields: same rule, absent keeps the current value.
    [JsonPropertyName("host")]        public string? Host       { get; set; }
    [JsonPropertyName("port")]        public int?    Port       { get; set; }
    [JsonPropertyName("stream")]      public string? Stream     { get; set; }
    [JsonPropertyName("codec")]       public string? Codec      { get; set; }
    [JsonPropertyName("decoder")]     public string? Decoder    { get; set; }
    [JsonPropertyName("ffmpegPath")]  public string? FfmpegPath { get; set; }
    [JsonPropertyName("hwaccel")]     public string? Hwaccel    { get; set; }
}

public class GetCameraResolutionsParams
{
    [JsonPropertyName("deviceIndex")] public int     DeviceIndex { get; set; } = 0;
    /// <summary>Optional camera id; a network camera answers [] (it has no resolution list).</summary>
    [JsonPropertyName("id")]          public string? Id          { get; set; }
}

public class TestCameraSourceParams
{
    /// <summary>"network" (default when absent) or "sofia".</summary>
    [JsonPropertyName("sourceType")] public string? SourceType { get; set; }
    [JsonPropertyName("url")]       public string  Url       { get; set; } = "";
    [JsonPropertyName("username")]  public string? Username  { get; set; }
    [JsonPropertyName("password")]  public string? Password  { get; set; }
    [JsonPropertyName("transport")] public string? Transport { get; set; }
    [JsonPropertyName("timeoutMs")] public int     TimeoutMs { get; set; } = 8000;
    // Sofia (sourceType "sofia")
    [JsonPropertyName("host")]       public string? Host       { get; set; }
    [JsonPropertyName("port")]       public int?    Port       { get; set; }
    [JsonPropertyName("stream")]     public string? Stream     { get; set; }
    [JsonPropertyName("codec")]      public string? Codec      { get; set; }
    [JsonPropertyName("decoder")]    public string? Decoder    { get; set; }
    [JsonPropertyName("ffmpegPath")] public string? FfmpegPath { get; set; }
    [JsonPropertyName("hwaccel")]    public string? Hwaccel    { get; set; }
}


