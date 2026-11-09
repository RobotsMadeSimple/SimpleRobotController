using System.Text.Json;
using SimpleRobot.PluginSdk;

// Simulated bench scale: a slow sine wave around 500 g plus a little noise.
var scale = new SimScale();
var plugin = new PluginHost();   // reads SRC_PLUGIN_* from the environment (set by the controller)

plugin.OnReady(ctx =>
{
    scale.Noise = NoiseFrom(ctx.Config, scale.Noise);
    ctx.Log($"scale ready (controller {ctx.ControllerVersion})");
    return Task.CompletedTask;
});

plugin.OnConfigChanged((ctx, cfg) =>
{
    scale.Noise = NoiseFrom(cfg, scale.Noise);
    ctx.Log($"noise is now {scale.Noise} g");
    return Task.CompletedTask;
});

plugin.Step("weigh", async (ctx, p) =>
{
    var step = (IStepContext)ctx;
    int n = Math.Clamp(p.GetInt("samples", 5), 1, 100);
    bool needStable = p.GetBool("stable", true);
    string unit = p.GetString("unit", "g");

    var readings = new List<double>();
    for (int attempt = 0; attempt < 3; attempt++)
    {
        readings.Clear();
        for (int i = 0; i < n; i++)
        {
            step.CancellationToken.ThrowIfCancellationRequested();
            readings.Add(scale.Read());
            ctx.Progress($"sample {i + 1}/{n}", 100.0 * (i + 1) / n);
            await Task.Delay(20, step.CancellationToken);
        }
        if (!needStable || SimScale.IsStable(readings)) break;
        if (attempt == 2) throw new StepException("Reading never settled", "scaleUnstable");
    }

    double grams = readings.Average();
    bool stable = SimScale.IsStable(readings);
    string text = unit == "oz" ? $"{grams / SimScale.GramsPerOunce:F2} oz" : $"{grams:F1} g";
    return new StepResult { ["grams"] = grams, ["stable"] = stable, ["text"] = text };
});

plugin.Function("tare", (ctx, args) => { scale.Tare(); return Task.FromResult(1.0); });
plugin.Function("toOz", (ctx, args) =>
    Task.FromResult(args.Length > 0 ? args[0] / SimScale.GramsPerOunce : throw new ArgumentException("toOz needs 1 argument")));

plugin.On("program.*", (ctx, e) =>
{
    ctx.Log($"program event: {e}", LogLevel.Debug);
    return Task.CompletedTask;
});

plugin.Background(async (ctx, ct) =>
{
    var window = new Queue<double>();
    while (!ct.IsCancellationRequested)
    {
        double w = scale.Read();
        window.Enqueue(w);
        if (window.Count > 10) window.Dequeue();
        ctx.SetProperties(new { weight = w, stable = SimScale.IsStable(window) ? 1 : 0, connected = 1 });
        await Task.Delay(100, ct);
    }
});

await plugin.RunAsync();

static double NoiseFrom(JsonElement cfg, double fallback) =>
    cfg.ValueKind == JsonValueKind.Object && cfg.TryGetProperty("noise", out var n) && n.ValueKind == JsonValueKind.Number
        ? n.GetDouble() : fallback;

sealed class SimScale
{
    public const double GramsPerOunce = 28.349523125;
    private readonly Random _rng = new();
    private double _tare;
    public double Noise = 0.2;

    private static double Raw() => 500 + 5 * Math.Sin(Environment.TickCount64 / 3000.0);
    public double Read() => Raw() - _tare + (_rng.NextDouble() * 2 - 1) * Noise;
    public void Tare() => _tare = Raw();

    public static bool IsStable(IEnumerable<double> values)
    {
        var a = values.ToArray();
        if (a.Length < 2) return true;
        double mean = a.Average();
        return Math.Sqrt(a.Sum(v => (v - mean) * (v - mean)) / a.Length) < 0.5;
    }
}
