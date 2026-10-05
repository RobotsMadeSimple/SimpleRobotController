using System.Globalization;
using System.Text.Json;

namespace SimpleRobot.PluginSdk;

/// <summary>The resolved parameters of one <c>step.execute</c> (values are already typed per the manifest, see docs/plugins.md §6).</summary>
public sealed class StepParams
{
    internal static readonly JsonSerializerOptions JsonOptions = new() { PropertyNameCaseInsensitive = true };

    public StepParams(JsonElement raw) { Raw = raw; }

    /// <summary>The params object exactly as received.</summary>
    public JsonElement Raw { get; }

    public bool Has(string key) => TryGet(key, out _);

    public bool TryGet(string key, out JsonElement value)
    {
        value = default;
        if (Raw.ValueKind != JsonValueKind.Object || !Raw.TryGetProperty(key, out var v)) return false;
        if (v.ValueKind is JsonValueKind.Null or JsonValueKind.Undefined) return false;
        value = v;
        return true;
    }

    /// <summary>Numbers, booleans (0/1) and numeric strings are accepted. Missing → <paramref name="defaultValue"/>.</summary>
    public double GetDouble(string key, double defaultValue = 0)
    {
        if (!TryGet(key, out var v)) return defaultValue;
        switch (v.ValueKind)
        {
            case JsonValueKind.Number: return v.GetDouble();
            case JsonValueKind.True: return 1;
            case JsonValueKind.False: return 0;
            case JsonValueKind.String when double.TryParse(v.GetString(), NumberStyles.Float, CultureInfo.InvariantCulture, out var d): return d;
        }
        throw Bad(key, "a number");
    }

    public int GetInt(string key, int defaultValue = 0)
        => TryGet(key, out _) ? (int)Math.Round(GetDouble(key)) : defaultValue;

    /// <summary>Booleans, numbers (non-zero = true) and "true"/"false" strings are accepted.</summary>
    public bool GetBool(string key, bool defaultValue = false)
    {
        if (!TryGet(key, out var v)) return defaultValue;
        switch (v.ValueKind)
        {
            case JsonValueKind.True: return true;
            case JsonValueKind.False: return false;
            case JsonValueKind.Number: return v.GetDouble() != 0;
            case JsonValueKind.String when bool.TryParse(v.GetString(), out var b): return b;
        }
        throw Bad(key, "a boolean");
    }

    public string GetString(string key, string defaultValue = "")
    {
        if (!TryGet(key, out var v)) return defaultValue;
        return v.ValueKind == JsonValueKind.String ? v.GetString() ?? defaultValue : v.GetRawText();
    }

    /// <summary>A <c>{x,y,z,rx,ry,rz}</c> object or an array of 6 numbers. Throws a <see cref="StepException"/> (badParams) when absent.</summary>
    public PluginPoint GetPoint(string key)
        => TryGetPoint(key, out var p) ? p : throw Bad(key, "a point");

    public bool TryGetPoint(string key, out PluginPoint point)
    {
        point = default;
        if (!TryGet(key, out var v)) return false;
        try
        {
            if (v.ValueKind == JsonValueKind.Array && v.GetArrayLength() == 6)
            {
                var a = v.EnumerateArray().Select(e => e.GetDouble()).ToArray();
                point = new PluginPoint(a[0], a[1], a[2], a[3], a[4], a[5]);
                return true;
            }
            if (v.ValueKind == JsonValueKind.Object)
            {
                point = v.Deserialize<PluginPoint>(JsonOptions);
                return true;
            }
        }
        catch (Exception e) when (e is JsonException or InvalidOperationException or FormatException) { }
        throw Bad(key, "a point");
    }

    /// <summary>Deserializes a list param (numbers, <see cref="PluginPoint"/>s, records…). Missing → empty list.</summary>
    public List<T> GetList<T>(string key)
    {
        if (!TryGet(key, out var v)) return new List<T>();
        if (v.ValueKind != JsonValueKind.Array) throw Bad(key, "a list");
        try { return v.Deserialize<List<T>>(JsonOptions) ?? new List<T>(); }
        catch (JsonException) { throw Bad(key, "a list of " + typeof(T).Name); }
    }

    /// <summary>Decodes an <c>image</c> param (base64 JPEG; a <c>data:…;base64,</c> prefix is tolerated). Missing → null.</summary>
    public byte[]? GetImageBytes(string key)
    {
        if (!TryGet(key, out var v)) return null;
        if (v.ValueKind != JsonValueKind.String) throw Bad(key, "a base64 image");
        var s = v.GetString() ?? "";
        int comma = s.IndexOf("base64,", StringComparison.OrdinalIgnoreCase);
        if (comma >= 0) s = s[(comma + 7)..];
        if (s.Length == 0) return null;
        try { return Convert.FromBase64String(s); }
        catch (FormatException) { throw Bad(key, "a base64 image"); }
    }

    private static StepException Bad(string key, string expected)
        => new($"Parameter '{key}' is not {expected}", "badParams");
}
