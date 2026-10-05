using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── Nano I/O command params ────────────────────────────────────────────────────

public class SetNanoOutputParams
{
    [JsonPropertyName("nanoId")] public string NanoId { get; set; } = "";
    [JsonPropertyName("pin")]    public int    Pin    { get; set; }
    [JsonPropertyName("value")]  public bool   Value  { get; set; }
}

public class SetNeoPixelParams
{
    [JsonPropertyName("nanoId")] public string NanoId { get; set; } = "";
    [JsonPropertyName("pin")]    public int    Pin    { get; set; }

    /// <summary>Array of { r, g, b } objects, one per pixel.</summary>
    [JsonPropertyName("colors")]
    public List<NeoPixelColorParams> Colors { get; set; } = new();
}

public class NeoPixelColorParams
{
    [JsonPropertyName("r")] public byte R { get; set; }
    [JsonPropertyName("g")] public byte G { get; set; }
    [JsonPropertyName("b")] public byte B { get; set; }
}

public class RenameNanoPinParams
{
    [JsonPropertyName("nanoId")]  public string NanoId  { get; set; } = "";
    [JsonPropertyName("pin")]     public int    Pin     { get; set; }
    [JsonPropertyName("name")]    public string Name    { get; set; } = "";
}

public class ConfigureNanoPinParams
{
    [JsonPropertyName("nanoId")]     public string NanoId     { get; set; } = "";
    [JsonPropertyName("pin")]        public int    Pin        { get; set; }
    [JsonPropertyName("type")]       public string Type       { get; set; } = "Input";
    [JsonPropertyName("pixelCount")] public int    PixelCount { get; set; } = 8;
}

// ── USB Relay ─────────────────────────────────────────────────────────────────

public class SetRelayParams
{
    [JsonPropertyName("relay")] public int  Relay { get; set; }  // 1–4
    [JsonPropertyName("value")] public bool Value { get; set; }
}

public class RenameRelayParams
{
    [JsonPropertyName("relay")] public int    Relay { get; set; }  // 1–4
    [JsonPropertyName("name")]  public string Name  { get; set; } = "";
}


/// <summary>Relay board state included in GetIO responses.</summary>
public class UsbRelayState
{
    [JsonPropertyName("connected")] public bool     Connected { get; set; }
    [JsonPropertyName("serial")]    public string?  Serial    { get; set; }
    [JsonPropertyName("relays")]    public bool[]?  Relays    { get; set; }  // index 0 = relay 1
    [JsonPropertyName("names")]     public string[] Names     { get; set; } = [];
}


