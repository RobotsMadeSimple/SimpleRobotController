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
}

public class GetCameraResolutionsParams
{
    [JsonPropertyName("deviceIndex")] public int DeviceIndex { get; set; } = 0;
}


