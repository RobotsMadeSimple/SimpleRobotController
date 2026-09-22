using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

// ── Stack ─────────────────────────────────────────────────────────────────────

/// <summary>A 1-D positional array — position = basePoint + index × offset.</summary>
public class RobotStack : Controller.RobotControl.Persistence.IStoredItem
{
    [JsonPropertyName("id")]             public string Id             { get; set; } = "";
    [JsonPropertyName("name")]           public string Name           { get; set; } = "";
    [JsonPropertyName("basePointName")]  public string BasePointName  { get; set; } = "";
    [JsonPropertyName("offsetX")]        public double OffsetX        { get; set; }
    [JsonPropertyName("offsetY")]        public double OffsetY        { get; set; }
    [JsonPropertyName("offsetZ")]        public double OffsetZ        { get; set; }
    /// <summary>When set, index wraps via modulo (round-robin).</summary>
    [JsonPropertyName("maxCount")]       public int?   MaxCount       { get; set; }
    [JsonPropertyName("lastUpdatedUnixMs")] public long LastUpdatedUnixMs { get; set; }
}

/// <summary>Identifies an entry in a named stack — used as the target in a MoveL/MoveJ/JumpL/JumpJ step.</summary>
public class StackPointRef
{
    [JsonPropertyName("stackId")] public string  StackId { get; set; } = "";
    [JsonPropertyName("index")]   public double? Index   { get; set; }
}

public class SaveStackParams
{
    [JsonPropertyName("id")]             public string  Id             { get; set; } = "";
    [JsonPropertyName("name")]           public string  Name           { get; set; } = "";
    [JsonPropertyName("basePointName")]  public string  BasePointName  { get; set; } = "";
    [JsonPropertyName("offsetX")]        public double  OffsetX        { get; set; }
    [JsonPropertyName("offsetY")]        public double  OffsetY        { get; set; }
    [JsonPropertyName("offsetZ")]        public double  OffsetZ        { get; set; }
    [JsonPropertyName("maxCount")]       public int?    MaxCount       { get; set; }
}

public class StackIdParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}


