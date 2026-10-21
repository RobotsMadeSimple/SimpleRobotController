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

// ── Load cell (HX711) command params ──────────────────────────────────────────

public class ConfigureLoadCellParams
{
    [JsonPropertyName("nanoId")] public string NanoId { get; set; } = "";
    [JsonPropertyName("dtPin")]  public int    DtPin  { get; set; }  // HX711 data pin
    [JsonPropertyName("sckPin")] public int    SckPin { get; set; }  // HX711 clock pin
    [JsonPropertyName("unit")]   public string Unit   { get; set; } = "g";
}

public class LoadCellPinParams
{
    [JsonPropertyName("nanoId")] public string NanoId { get; set; } = "";
    [JsonPropertyName("pin")]    public int    Pin    { get; set; }  // the DT pin
}

public class CalibrateLoadCellParams
{
    [JsonPropertyName("nanoId")]     public string NanoId     { get; set; } = "";
    [JsonPropertyName("pin")]        public int    Pin        { get; set; }
    [JsonPropertyName("knownGrams")] public double KnownGrams { get; set; }
}

public class SetLoadCellUnitParams
{
    [JsonPropertyName("nanoId")] public string NanoId { get; set; } = "";
    [JsonPropertyName("pin")]    public int    Pin    { get; set; }
    [JsonPropertyName("unit")]   public string Unit   { get; set; } = "g";
}

// ── IO value history / recordings command params ───────────────────────────────

public class IoHistoryWindowParams
{
    [JsonPropertyName("channelKey")] public string ChannelKey { get; set; } = "";
    [JsonPropertyName("fromUnixMs")] public long?  FromUnixMs { get; set; }
    [JsonPropertyName("toUnixMs")]   public long?  ToUnixMs   { get; set; }
    [JsonPropertyName("maxPoints")]  public int?   MaxPoints  { get; set; }
}

public class SetIoHistoryRetentionParams
{
    [JsonPropertyName("maxRetentionMs")] public long MaxRetentionMs { get; set; }
}

public class StartIoRecordingParams
{
    [JsonPropertyName("channelKey")] public string  ChannelKey { get; set; } = "";
    [JsonPropertyName("name")]       public string? Name       { get; set; }
}

public class IoRecordingIdParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class IoRecordingWindowParams
{
    [JsonPropertyName("id")]         public string Id         { get; set; } = "";
    [JsonPropertyName("fromUnixMs")] public long?  FromUnixMs { get; set; }
    [JsonPropertyName("toUnixMs")]   public long?  ToUnixMs   { get; set; }
    [JsonPropertyName("maxPoints")]  public int?   MaxPoints  { get; set; }
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


