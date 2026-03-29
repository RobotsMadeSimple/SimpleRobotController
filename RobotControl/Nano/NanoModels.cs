using System.Text.Json.Serialization;

namespace Controller.RobotControl.Nano
{
    // ── Enums ─────────────────────────────────────────────────────────────────

    [JsonConverter(typeof(JsonStringEnumConverter))]
    public enum PinType { Input, Output, Neopixel, Unconfigured }

    // ── Config file models ────────────────────────────────────────────────────

    /// <summary>Top-level nano_config.json structure.</summary>
    public class NanoConfig
    {
        [JsonPropertyName("nanos")]
        public List<NanoDeviceConfig> Nanos { get; set; } = new();
    }

    /// <summary>Configuration for a single Arduino Nano device.</summary>
    public class NanoDeviceConfig
    {
        /// <summary>Identifier string burned into the firmware as DEVICE_ID.</summary>
        [JsonPropertyName("id")]
        public string Id { get; set; } = "";

        /// <summary>Human-readable label shown in the UI.</summary>
        [JsonPropertyName("name")]
        public string Name { get; set; } = "";

        [JsonPropertyName("pins")]
        public List<NanoPinConfig> Pins { get; set; } = new();
    }

    /// <summary>Configuration for a single pin on a Nano device.</summary>
    public class NanoPinConfig
    {
        /// <summary>Arduino digital pin number (0–19).</summary>
        [JsonPropertyName("pin")]
        public int Pin { get; set; }

        [JsonPropertyName("type")]
        public PinType Type { get; set; }

        /// <summary>Human-readable label shown in the UI.</summary>
        [JsonPropertyName("name")]
        public string Name { get; set; } = "";

        /// <summary>Number of pixels for Neopixel strips (ignored for other types).</summary>
        [JsonPropertyName("pixelCount")]
        public int PixelCount { get; set; } = 8;
    }

    // ── Runtime state models ──────────────────────────────────────────────────

    /// <summary>Live state of a single pin, sent to the WebSocket client.</summary>
    public class NanoPinState
    {
        public int     Pin        { get; set; }
        public PinType Type       { get; set; }
        public bool    Value      { get; set; }
        public string  Name       { get; set; } = "";
        public string  NanoId     { get; set; } = "";
        public string  NanoName   { get; set; } = "";
        public int     PixelCount { get; set; }
    }

    /// <summary>Connection + pin state for a single Nano, sent to the WebSocket client.</summary>
    public class NanoState
    {
        public string            Id        { get; set; } = "";
        public string            Name      { get; set; } = "";
        public bool              Connected { get; set; }
        public List<NanoPinState> Pins     { get; set; } = new();
    }

    // ── Colour helper ─────────────────────────────────────────────────────────

    public class NeoPixelColor
    {
        public byte R { get; set; }
        public byte G { get; set; }
        public byte B { get; set; }

        public NeoPixelColor() { }
        public NeoPixelColor(byte r, byte g, byte b) { R = r; G = g; B = b; }

        public static NeoPixelColor Off    => new(0, 0, 0);
        public static NeoPixelColor Green  => new(0, 200, 0);
        public static NeoPixelColor Blue   => new(0, 0, 255);
        public static NeoPixelColor Red    => new(255, 0, 0);
        public static NeoPixelColor Yellow => new(255, 200, 0);
        public static NeoPixelColor Orange => new(255, 100, 0);
        public static NeoPixelColor Purple => new(80, 0, 80);
    }
}
