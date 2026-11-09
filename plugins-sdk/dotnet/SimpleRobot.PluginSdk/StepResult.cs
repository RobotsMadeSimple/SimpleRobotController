using System.Collections;
using System.Text.Json;

namespace SimpleRobot.PluginSdk;

/// <summary>
/// Outputs of a step, keyed by the manifest's output keys. Accepted values: numbers, bool, string, <see cref="PluginPoint"/>,
/// IEnumerable of those (lists), <c>byte[]</c> (image, sent as base64), dictionaries (records) and JsonElement.
/// </summary>
public sealed class StepResult : IEnumerable<KeyValuePair<string, object?>>
{
    private readonly Dictionary<string, object?> _values = new();

    public object? this[string key]
    {
        get => _values[key];
        set => _values[key] = Validate(key, value);
    }

    public int Count => _values.Count;
    public bool ContainsKey(string key) => _values.ContainsKey(key);
    public IEnumerable<string> Keys => _values.Keys;

    /// <summary>Collection-initializer support: <c>new StepResult { { "grams", 1.5 } }</c>.</summary>
    public void Add(string key, object? value) => this[key] = value;

    public IEnumerator<KeyValuePair<string, object?>> GetEnumerator() => _values.GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    internal Dictionary<string, object?> ToDictionary() => new(_values);

    private static object? Validate(string key, object? v)
    {
        switch (v)
        {
            case null or bool or string or byte[] or PluginPoint or JsonElement:
            case sbyte or byte or short or ushort or int or uint or long or ulong or float or double or decimal:
                return v;
            case IDictionary d:
                foreach (DictionaryEntry e in d) Validate(key, e.Value);
                return v;
            case IEnumerable en:
                foreach (var item in en) Validate(key, item);
                return v;
            default:
                throw new ArgumentException($"Output '{key}': unsupported value type {v.GetType().Name}", nameof(v));
        }
    }
}
