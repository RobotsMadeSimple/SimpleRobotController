using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

/// <summary>
/// One record in an object-list variable: named numeric fields, read in expressions as
/// <c>$cells[$i].coverage</c>. Numbers only — booleans are 0/1 — because expressions
/// evaluate to doubles, so a string field would have nothing to evaluate to.
/// </summary>
public class ObjectRecord : Dictionary<string, double>
{
    public ObjectRecord() : base(StringComparer.OrdinalIgnoreCase) { }

    /// <summary>The key a Number-element list stores its scalar under.</summary>
    public const string ScalarKey = "value";

    public static ObjectRecord FromScalar(double v) => new() { [ScalarKey] = v };

    public static ObjectRecord FromPoint(Vector6Val p) => new()
    {
        ["x"] = p.X, ["y"] = p.Y, ["z"] = p.Z, ["rx"] = p.RX, ["ry"] = p.RY, ["rz"] = p.RZ,
    };

    public double Scalar => TryGetValue(ScalarKey, out double v) ? v : 0;

    public Vector6Val ToPoint() => new()
    {
        X  = TryGetValue("x",  out var x)  ? x  : 0,
        Y  = TryGetValue("y",  out var y)  ? y  : 0,
        Z  = TryGetValue("z",  out var z)  ? z  : 0,
        RX = TryGetValue("rx", out var rx) ? rx : 0,
        RY = TryGetValue("ry", out var ry) ? ry : 0,
        RZ = TryGetValue("rz", out var rz) ? rz : 0,
    };
}

/// <summary>What one element of a list variable is shaped like.</summary>
[JsonConverter(typeof(JsonStringEnumConverter))]
public enum ListElementType
{
    /// <summary>A bare number. Read as <c>$v[0]</c>.</summary>
    Number,
    /// <summary>
    /// A true/false flag. Stored exactly like a Number — 0 or 1 under the scalar key — so it
    /// reads as <c>$v[0]</c> and works in conditions unchanged. The separate type exists so
    /// the element renders as True/False and the editor offers toggles instead of a keypad.
    /// </summary>
    Boolean,
    /// <summary>Six named axes — x, y, z, rx, ry, rz. Read as <c>$v[0].x</c> or <c>$v[0][0]</c>.</summary>
    Point,
    /// <summary>Open-ended named numeric fields. Read as <c>$v[0].coverage</c>.</summary>
    Record,
}

/// <summary>
/// One list variable at runtime: the elements plus what shape they are.
///
/// Every element type is stored as <see cref="ObjectRecord"/> — a number or a boolean under
/// the reserved <c>value</c> key, a point under x/y/z/rx/ry/rz — because a record is already a
/// dictionary of named doubles and the rest are special cases of it. That is what lets the
/// evaluator carry one dictionary instead of several, and index one way instead of several.
/// <see cref="ElementType"/> is still needed because the *syntax* and the rendering differ: a
/// number list answers a bare <c>$v[0]</c>, only a point list answers positional
/// <c>$v[0][2]</c>, and a boolean list reads back as True/False rather than 1/0.
/// </summary>
public sealed class ListVar
{
    public ListElementType ElementType { get; init; }
    public List<ObjectRecord> Items { get; init; } = [];

    public int Count => Items.Count;

    /// <summary>
    /// Whether an element is itself a value, so <c>$v[0]</c> resolves without an accessor.
    /// True for Number and Boolean; a point or a record needs a field named after it.
    /// </summary>
    public bool HasScalarElements =>
        ElementType is ListElementType.Number or ListElementType.Boolean;

    public static ListVar OfNumbers(IEnumerable<double> vals) => new()
    {
        ElementType = ListElementType.Number,
        Items = vals.Select(ObjectRecord.FromScalar).ToList(),
    };

    public static ListVar OfBooleans(IEnumerable<bool> vals) => new()
    {
        ElementType = ListElementType.Boolean,
        Items = vals.Select(b => ObjectRecord.FromScalar(b ? 1 : 0)).ToList(),
    };

    public static ListVar OfPoints(IEnumerable<Vector6Val> pts) => new()
    {
        ElementType = ListElementType.Point,
        Items = pts.Select(ObjectRecord.FromPoint).ToList(),
    };

    public static ListVar OfRecords(IEnumerable<ObjectRecord> recs) => new()
    {
        ElementType = ListElementType.Record,
        Items = recs.ToList(),
    };

    /// <summary>Positional component access, which only a point list has an ordering for.</summary>
    private static readonly string[] PointAxes = ["x", "y", "z", "rx", "ry", "rz"];

    public static string? AxisName(int idx) =>
        idx >= 0 && idx < PointAxes.Length ? PointAxes[idx] : null;

    /// <summary>Elements as points — used by move targets, which need a full pose.</summary>
    public List<Vector6Val> AsPoints() => Items.ConvertAll(r => r.ToPoint());

    /// <summary>How a list reads in a status message: "3 points", "12 items".</summary>
    public string Describe() => ElementType switch
    {
        ListElementType.Point  => $"{Count} point{(Count == 1 ? "" : "s")}",
        ListElementType.Record => $"{Count} object{(Count == 1 ? "" : "s")}",
        _                      => $"{Count} item{(Count == 1 ? "" : "s")}",
    };
}

public class ProgramVariable
{
    [JsonPropertyName("id")]
    public string Id { get; set; } = "";
    [JsonPropertyName("name")]
    public string Name { get; set; } = "";
    [JsonPropertyName("value")]
    public double Value { get; set; }

    /// <summary>
    /// An expression evaluated at program start to produce the initial value, used instead
    /// of <see cref="Value"/> when it is set. Applies to Number and Boolean scalars — a
    /// boolean takes the usual non-zero-is-true reading, so "$count &gt; 5" works.
    ///
    /// <see cref="Value"/> is still written alongside it as the last known result, so a
    /// build that does not understand this field, or an expression that fails to evaluate,
    /// falls back to a sensible number instead of 0.
    /// </summary>
    [JsonPropertyName("valueExpression")]
    public string? ValueExpression { get; set; }

    /// <summary>
    /// When non-null, this is a list variable. Every element is a record of named doubles:
    /// a Number or Boolean element keeps its scalar under <c>value</c> (a boolean as 0/1), a
    /// Point element under x/y/z/rx/ry/rz. <see cref="ElementType"/> says which.
    /// </summary>
    [JsonPropertyName("items")]
    public List<ObjectRecord>? Items { get; set; }

    /// <summary>Shape of each element in <see cref="Items"/>. Absent reads as Record.</summary>
    [JsonPropertyName("elementType")]
    public ListElementType? ElementType { get; set; }

    // ── Legacy list fields ────────────────────────────────────────────────────
    // Read-only inputs kept so programs saved before the list types were unified still
    // load. ToListVar folds them into Items; nothing writes them any more, and once a
    // program has been saved by a current build they disappear from its JSON.

    /// <summary>Superseded by <see cref="Items"/> with ElementType Number.</summary>
    [JsonPropertyName("values")]
    public List<double>? Values { get; set; }
    /// <summary>Superseded by <see cref="Items"/> with ElementType Point.</summary>
    [JsonPropertyName("points")]
    public List<Vector6Val>? Points { get; set; }
    /// <summary>Superseded by <see cref="Items"/> with ElementType Record.</summary>
    [JsonPropertyName("objects")]
    public List<ObjectRecord>? Objects { get; set; }

    /// <summary>
    /// This variable as a list, or null if it is a scalar of some kind.
    ///
    /// Legacy fields are consulted only when <see cref="Items"/> is absent, so a program
    /// that carries both — one written by a current build, then edited by an older one —
    /// resolves to the current field rather than silently reverting.
    /// </summary>
    public ListVar? ToListVar()
    {
        if (Items != null)
            return new ListVar { ElementType = ElementType ?? ListElementType.Record, Items = Items };
        if (Points  != null) return ListVar.OfPoints(Points);
        if (Objects != null) return ListVar.OfRecords(Objects);
        // An empty legacy number list was indistinguishable from a scalar, and the app
        // treated it as one. Preserved deliberately: changing it would turn some saved
        // scalars into lists.
        if (Values != null && Values.Count > 0) return ListVar.OfNumbers(Values);
        return null;
    }

    [JsonPropertyName("description")]
    public string? Description { get; set; }
    [JsonPropertyName("isBoolean")]
    public bool? IsBoolean { get; set; }
    /// <summary>When true, this scalar variable is shared across all concurrently running programs via the global variable store.</summary>
    [JsonPropertyName("isGlobal")]
    public bool? IsGlobal { get; set; }
    /// <summary>When true, the current runtime value of this variable is shown on the monitor page while the program runs.</summary>
    [JsonPropertyName("displayOnMonitor")]
    public bool? DisplayOnMonitor { get; set; }
    /// <summary>When true, this variable is a stopwatch; its value holds elapsed milliseconds at runtime.</summary>
    [JsonPropertyName("isStopwatch")]
    public bool? IsStopwatch { get; set; }
    /// <summary>When true, the runtime value is saved to disk when the program finishes and restored on the next run.</summary>
    [JsonPropertyName("isPersistent")]
    public bool? IsPersistent { get; set; }
    /// <summary>When true, this variable holds a string value stored in StringValue.</summary>
    [JsonPropertyName("isString")]
    public bool? IsString { get; set; }
    /// <summary>String variable initial/default value — only meaningful when IsString is true.</summary>
    [JsonPropertyName("stringValue")]
    public string? StringValue { get; set; }
    /// <summary>When true, this variable holds a JPEG image as a base64 string, populated at runtime by CaptureImage steps.</summary>
    [JsonPropertyName("isImage")]
    public bool? IsImage { get; set; }
}


