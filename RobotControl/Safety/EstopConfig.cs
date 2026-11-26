using System.Text.Json.Serialization;

namespace Controller.RobotControl.Safety;

/// <summary>
/// Where the controller reads its hardwired e-stop sense input from (docs/estop.md). Persisted as
/// the <c>estop</c> object of robot-config.json. Off by default.
/// </summary>
/// <remarks>
/// This is a monitoring input only: the e-stop button itself cuts the stepper drivers' power in
/// hardware. The controller learns about it from this input and reacts (stops, aborts, latches),
/// but it is not a certified safety function.
/// </remarks>
public sealed class EstopConfig
{
    public const string SourceStb = "stb";
    public const string SourceIo  = "io";

    public const string PolarityNormallyClosed = "normallyClosed";
    public const string PolarityNormallyOpen   = "normallyOpen";

    /// <summary>Master switch. Off by default: nothing is monitored until the operator turns it on.</summary>
    [JsonPropertyName("enabled")] public bool Enabled { get; set; }

    /// <summary><c>stb</c> (an STB4100 input) or <c>io</c> (an edge-device pin, by device id + pin).</summary>
    [JsonPropertyName("source")] public string Source { get; set; } = SourceStb;

    /// <summary>STB4100 input number 1..4 when <see cref="Source"/> is <c>stb</c>. Inputs 1–3 are the
    /// homing switches on ASTRO, so the default is input 4.</summary>
    [JsonPropertyName("stbInput")] public int StbInput { get; set; } = 4;

    /// <summary>The edge device's stable id (io_devices.json / aux_config.json) when <see cref="Source"/> is <c>io</c>.</summary>
    [JsonPropertyName("deviceId")] public string? DeviceId { get; set; }

    /// <summary>The device pin when <see cref="Source"/> is <c>io</c>.</summary>
    [JsonPropertyName("pin")] public int? Pin { get; set; }

    /// <summary>
    /// <c>normallyClosed</c> (default, fail-safe): the input reads on while the circuit is healthy, so
    /// an open circuit — the button pressed, or a broken wire — is an e-stop. <c>normallyOpen</c>: the
    /// input reads on when the e-stop is pressed.
    /// </summary>
    [JsonPropertyName("polarity")] public string Polarity { get; set; } = PolarityNormallyClosed;

    [JsonIgnore] public bool NormallyClosed => !string.Equals(Polarity, PolarityNormallyOpen, StringComparison.OrdinalIgnoreCase);

    public EstopConfig Clone() => new()
    {
        Enabled  = Enabled,
        Source   = Source,
        StbInput = StbInput,
        DeviceId = DeviceId,
        Pin      = Pin,
        Polarity = Polarity,
    };

    /// <summary>Why this config cannot be used, or null when it is valid.</summary>
    public string? Validate()
    {
        if (Source != SourceStb && Source != SourceIo)
            return "source must be \"stb\" or \"io\"";
        if (Polarity != PolarityNormallyClosed && Polarity != PolarityNormallyOpen)
            return "polarity must be \"normallyClosed\" or \"normallyOpen\"";
        if (Source == SourceStb && StbInput is < 1 or > 4)
            return "stbInput must be 1..4";
        if (Source == SourceIo && Enabled && (string.IsNullOrWhiteSpace(DeviceId) || Pin is null))
            return "an io source needs deviceId and pin";
        return null;
    }
}

/// <summary>One read of the e-stop input.</summary>
/// <param name="Available">The source device is connected and the pin is known.</param>
/// <param name="Raw">The input's logical level (true = on), or null when unavailable.</param>
/// <param name="Label">Where it was read, for messages ("STB input 4", "Cell IO pin 7").</param>
public readonly record struct EstopReading(bool Available, bool? Raw, string Label);
