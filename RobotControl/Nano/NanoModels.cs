using System.Text.Json.Serialization;

namespace Controller.RobotControl.Nano
{
    // ── Enums ─────────────────────────────────────────────────────────────────

    [JsonConverter(typeof(JsonStringEnumConverter))]
    public enum PinType { Input, Output, Neopixel, Unconfigured, LoadCell }

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

        // ── Load cell (HX711) — only used when Type == LoadCell ────────────────
        /// <summary>HX711 clock (SCK) pin. The <see cref="Pin"/> field is the HX711 data (DT) pin.</summary>
        [JsonPropertyName("sckPin")]
        public int SckPin { get; set; } = -1;

        /// <summary>Raw count read with no load — subtracted from every reading (the tare / zero offset).</summary>
        [JsonPropertyName("tareOffset")]
        public long TareOffset { get; set; }

        /// <summary>Raw counts per gram, from the 2-point calibration. grams = (raw - tareOffset) / countsPerGram.
        /// 0 means uncalibrated (grams reports 0).</summary>
        [JsonPropertyName("countsPerGram")]
        public double CountsPerGram { get; set; }

        /// <summary>Display unit for this load cell (grams is the master behind the scenes): g, kg, oz, lb, N.</summary>
        [JsonPropertyName("unit")]
        public string Unit { get; set; } = "g";
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

        // ── Load cell runtime (Type == LoadCell) ──────────────────────────────
        public int     SckPin        { get; set; }
        /// <summary>Latest raw 24-bit count streamed by the HX711.</summary>
        public long    Raw           { get; set; }
        /// <summary>Calibrated weight in grams (the master unit): (raw - tareOffset) / countsPerGram.</summary>
        public double  Grams         { get; set; }
        /// <summary>Display unit (g/kg/oz/lb/N); grams is converted to it client-side.</summary>
        public string  Unit          { get; set; } = "g";
        public long    TareOffset    { get; set; }
        public double  CountsPerGram { get; set; }
        /// <summary>Measured HX711 sample rate (Hz) — reflects the RATE-pin wiring (10 or 80 SPS).</summary>
        public double  SampleRateHz  { get; set; }
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
