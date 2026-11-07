using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Language;

// The parser's internal tree. A step is kept as the JSON its text produced (ProgramStep JSON
// names), plus its nested bodies and the @with object that is merged over it last — the JSON
// shape is what makes "every field round-trips" mechanical: anything the sugar does not say
// travels in @with.

internal sealed class StepNode
{
    public StepType Type;
    /// <summary>Fields from the statement text (no nested bodies).</summary>
    public JsonObject Json = new();
    /// <summary>The merged <c>@with {…}</c> objects (null when none).</summary>
    public JsonObject? With;
    public string? ExplicitId;
    public SrsSpan? IdSpan;
    public readonly List<StepBody> Bodies = [];
    /// <summary>The statement line (first physical line) and the block's last line (its end).</summary>
    public int Line, EndLine;
    public SrsSpan Span;
    /// <summary>Catalog field key → the span of its value, for mapping validator problems.</summary>
    public readonly Dictionary<string, SrsSpan> FieldSpans = new(StringComparer.Ordinal);
    /// <summary>SetVariable: the right-hand side was a string literal.</summary>
    public bool RhsIsString;
    public SrsSpan RhsSpan;
}

internal sealed record StepBody(string Role, int? Index, List<StepNode> Steps);

internal sealed class VarNode
{
    public JsonObject Json = new();
    public JsonObject? With;
    public string? ExplicitId;
    public string Name = "";
    public int Line;
    public SrsSpan Span;
}

internal sealed class SrsDocument
{
    public readonly List<VarNode> Vars = [];
    public readonly List<StepNode> Steps = [];
    public JsonObject? ProgramHeader;
    public readonly List<TextProblem> Problems = [];
}

/// <summary>JSON plumbing for the language: one serializer setup, deep merge and diff.</summary>
internal static class SrsJson
{
    public static readonly JsonSerializerOptions Options = new()
    {
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        Converters = { new JsonStringEnumConverter() },
        PropertyNameCaseInsensitive = false,
    };

    /// <summary>A compact one-line JSON writer that keeps non-ASCII text readable.</summary>
    public static readonly JsonSerializerOptions Compact = new()
    {
        Encoder = System.Text.Encodings.Web.JavaScriptEncoder.UnsafeRelaxedJsonEscaping,
    };

    public static JsonObject ToJson(ProgramStep s) => (JsonObject)JsonSerializer.SerializeToNode(s, Options)!;
    public static JsonObject ToJson(ProgramVariable v) => (JsonObject)JsonSerializer.SerializeToNode(v, Options)!;
    public static JsonObject ToJson(BuiltProgram p) => (JsonObject)JsonSerializer.SerializeToNode(p, Options)!;

    public static ProgramStep Step(JsonObject o) => o.Deserialize<ProgramStep>(Options)!;
    public static ProgramVariable Var(JsonObject o) => o.Deserialize<ProgramVariable>(Options)!;

    public static T Clone<T>(T value) =>
        JsonSerializer.Deserialize<T>(JsonSerializer.Serialize(value, Options), Options)!;

    public static string Write(JsonNode node) => node.ToJsonString(Compact);

    /// <summary>Deep merge <paramref name="over"/> into <paramref name="target"/>: objects merge,
    /// everything else replaces, a JSON null removes the property.</summary>
    public static void Merge(JsonObject target, JsonObject over)
    {
        foreach (var (k, v) in over.ToList())
        {
            if (v == null) { target.Remove(k); continue; }
            if (v is JsonObject vo && target[k] is JsonObject to) { Merge(to, vo); continue; }
            target[k] = v.DeepClone();
        }
    }

    /// <summary>The object <c>R</c> with <c>Merge(have, R) == want</c> (deep-equal).</summary>
    public static JsonObject Diff(JsonObject want, JsonObject have)
    {
        var r = new JsonObject();
        foreach (var (k, v) in want)
        {
            if (!have.TryGetPropertyValue(k, out var h) || h == null)
            {
                r[k] = v?.DeepClone();
                continue;
            }
            if (v is JsonObject wo && h is JsonObject ho)
            {
                var d = Diff(wo, ho);
                if (d.Count > 0) r[k] = d;
                continue;
            }
            if (!JsonNode.DeepEquals(v, h)) r[k] = v?.DeepClone();
        }
        foreach (var (k, h) in have)
            if (h != null && !want.ContainsKey(k)) r[k] = null;
        return r;
    }

    /// <summary>Every JSON name of <typeparamref name="T"/>'s properties.</summary>
    public static HashSet<string> PropertyNames<T>() =>
        typeof(T).GetProperties()
            .Select(p => p.GetCustomAttributes(typeof(JsonPropertyNameAttribute), false)
                          .OfType<JsonPropertyNameAttribute>().FirstOrDefault()?.Name)
            .Where(n => n != null).Select(n => n!).ToHashSet(StringComparer.Ordinal);

    public static readonly HashSet<string> StepProps = PropertyNames<ProgramStep>();
    public static readonly HashSet<string> VarProps  = PropertyNames<ProgramVariable>();
}
