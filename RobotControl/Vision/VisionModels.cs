using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Vision
{
    // The building-block vision model: a program is a list of image layers (shared filter
    // stacks) and a list of generic tools that read them, run in list order, and can follow
    // each other's found poses (fixtures). Full contract: docs/vision-tools.md.
    //
    // Conventions
    //  - Regions are normalized (0–1 of width / height), as drawn on the teach image.
    //  - Results (points, lines, poses, items) are in pixels of the processed frame.
    //  - Angles are degrees, atan2(dy, dx) in pixel coordinates with y down (the OpenCV
    //    convention): positive turns clockwise on screen. Region rotation uses the same sense.

    /// <summary>A normalized point (0–1) of a drawn region.</summary>
    public class VisionPoint
    {
        [JsonPropertyName("x")] public double X { get; set; }
        [JsonPropertyName("y")] public double Y { get; set; }

        public VisionPoint() { }
        public VisionPoint(double x, double y) { X = x; Y = y; }
    }

    /// <summary>
    /// A tool's region, normalized to the image. shape: "rect" (x, y = top-left, width, height,
    /// rotation in degrees about its centre), "circle" (cx, cy, radius = fraction of
    /// min(width, height)), "polygon" (points) or "quad" (exactly four points TL, TR, BR, BL —
    /// a perspective region whose grid cells follow the perspective).
    /// </summary>
    public class VisionRegion
    {
        [JsonPropertyName("shape")]    public string Shape    { get; set; } = "rect";
        [JsonPropertyName("x")]        public double X        { get; set; }
        [JsonPropertyName("y")]        public double Y        { get; set; }
        [JsonPropertyName("width")]    public double Width    { get; set; } = 1;
        [JsonPropertyName("height")]   public double Height   { get; set; } = 1;
        [JsonPropertyName("rotation")] public double Rotation { get; set; }
        [JsonPropertyName("cx")]       public double Cx       { get; set; } = 0.5;
        [JsonPropertyName("cy")]       public double Cy       { get; set; } = 0.5;
        [JsonPropertyName("radius")]   public double Radius   { get; set; } = 0.25;
        [JsonPropertyName("points")]   public List<VisionPoint>? Points { get; set; }
    }

    /// <summary>Splits a tool's region into rows × cols cells, each measured on its own.</summary>
    public class VisionGrid
    {
        [JsonPropertyName("rows")]  public int    Rows  { get; set; } = 1;
        [JsonPropertyName("cols")]  public int    Cols  { get; set; } = 1;
        /// <summary>Margin trimmed off every side of a cell before measuring, as a fraction of the cell (0–0.45).</summary>
        [JsonPropertyName("inset")] public double Inset { get; set; }
    }

    /// <summary>Judgement on one numeric value: passed when min ≤ value ≤ max (a null bound is open).</summary>
    public class VisionLimit
    {
        [JsonPropertyName("min")] public double? Min { get; set; }
        [JsonPropertyName("max")] public double? Max { get; set; }

        public bool Accepts(double v) => (!Min.HasValue || v >= Min.Value) && (!Max.HasValue || v <= Max.Value);
    }

    /// <summary>A pixel point in a result.</summary>
    public class PxPoint
    {
        [JsonPropertyName("x")] public double X { get; set; }
        [JsonPropertyName("y")] public double Y { get; set; }

        public PxPoint() { }
        public PxPoint(double x, double y) { X = Math.Round(x, 2); Y = Math.Round(y, 2); }
    }

    /// <summary>A pixel line segment in a result.</summary>
    public class PxLine
    {
        [JsonPropertyName("x1")] public double X1 { get; set; }
        [JsonPropertyName("y1")] public double Y1 { get; set; }
        [JsonPropertyName("x2")] public double X2 { get; set; }
        [JsonPropertyName("y2")] public double Y2 { get; set; }

        public PxLine() { }
        public PxLine(double x1, double y1, double x2, double y2)
        {
            X1 = Math.Round(x1, 2); Y1 = Math.Round(y1, 2); X2 = Math.Round(x2, 2); Y2 = Math.Round(y2, 2);
        }
    }

    /// <summary>
    /// Where a locating tool found its object, in pixels: origin, angle (degrees), scale, and
    /// for the four-corner fixtures the corners TL, TR, BR, BL (a perspective quad).
    /// </summary>
    public class VisionPose
    {
        [JsonPropertyName("x")]     public double X     { get; set; }
        [JsonPropertyName("y")]     public double Y     { get; set; }
        [JsonPropertyName("angle")] public double Angle { get; set; }
        [JsonPropertyName("scale")] public double Scale { get; set; } = 1;
        [JsonPropertyName("quad")]  public List<PxPoint>? Quad { get; set; }

        public VisionPose Clone() => new()
        {
            X = X, Y = Y, Angle = Angle, Scale = Scale,
            Quad = Quad?.Select(p => new PxPoint { X = p.X, Y = p.Y }).ToList(),
        };
    }

    /// <summary>
    /// One filter of a layer: <c>type</c> plus its parameters, which sit flat on the object
    /// (e.g. <c>{"id":"f1","type":"blur","enabled":true,"size":5}</c>) and are kept as raw JSON.
    /// </summary>
    public class VisionFilter
    {
        [JsonPropertyName("id")]      public string Id      { get; set; } = "";
        [JsonPropertyName("type")]    public string Type    { get; set; } = "";
        [JsonPropertyName("enabled")] public bool   Enabled { get; set; } = true;
        [JsonExtensionData]           public Dictionary<string, JsonElement>? Params { get; set; }

        [JsonIgnore] public ParamReader P => new(Params);
    }

    /// <summary>A shared filtered image: an input ("camera" or an earlier layer) and a filter stack.</summary>
    public class ImageLayer
    {
        [JsonPropertyName("id")]      public string             Id      { get; set; } = "";
        [JsonPropertyName("name")]    public string             Name    { get; set; } = "";
        [JsonPropertyName("input")]   public string             Input   { get; set; } = VisionTool.CameraInput;
        [JsonPropertyName("filters")] public List<VisionFilter> Filters { get; set; } = new();
    }

    /// <summary>One building-block tool. Type-specific settings live in <see cref="Params"/>.</summary>
    public class VisionTool
    {
        public const string CameraInput = "camera";

        [JsonPropertyName("id")]            public string  Id            { get; set; } = "";
        [JsonPropertyName("name")]          public string  Name          { get; set; } = "";
        [JsonPropertyName("type")]          public string  Type          { get; set; } = "";
        [JsonPropertyName("enabled")]       public bool    Enabled       { get; set; } = true;
        [JsonPropertyName("input")]         public string  Input         { get; set; } = CameraInput;
        [JsonPropertyName("fixtureId")]     public string? FixtureId     { get; set; }
        [JsonPropertyName("region")]        public VisionRegion? Region  { get; set; }
        [JsonPropertyName("grid")]          public VisionGrid?   Grid    { get; set; }
        [JsonPropertyName("limits")]        public Dictionary<string, VisionLimit>? Limits { get; set; }
        [JsonPropertyName("includeInPass")] public bool    IncludeInPass { get; set; } = true;
        [JsonPropertyName("reference")]     public VisionPose? Reference { get; set; }
        [JsonPropertyName("params")]        public Dictionary<string, JsonElement>? Params { get; set; }

        [JsonIgnore] public ParamReader P => new(Params);

        /// <summary>Sets one param to any JSON-serializable value.</summary>
        public void SetParam(string key, object? value)
        {
            Params ??= new Dictionary<string, JsonElement>();
            Params[key] = JsonSerializer.SerializeToElement(value);
        }
    }

    public class VisionProgram
    {
        [JsonPropertyName("id")]                public string           Id                { get; set; } = "";
        [JsonPropertyName("name")]              public string           Name              { get; set; } = "";
        [JsonPropertyName("description")]       public string           Description       { get; set; } = "";
        [JsonPropertyName("cameraId")]          public string           CameraId          { get; set; } = "";
        [JsonPropertyName("lastUpdatedUnixMs")] public long             LastUpdatedUnixMs { get; set; }
        [JsonPropertyName("layers")]            public List<ImageLayer> Layers            { get; set; } = new();
        [JsonPropertyName("tools")]             public List<VisionTool> Tools             { get; set; } = new();

        public VisionTool? Tool(string id) => (Tools ?? []).FirstOrDefault(t => t.Id == id);
    }

    // ── Results ──────────────────────────────────────────────────────────────────

    public static class ToolStatus
    {
        public const string Ok     = "ok";
        public const string Fail   = "fail";
        public const string NotRun = "notRun";
        public const string Error  = "error";
    }

    /// <summary>One grid cell of a presence / count tool: its main value and judgement.</summary>
    public class CellResult
    {
        [JsonPropertyName("row")]    public int    Row    { get; set; }
        [JsonPropertyName("col")]    public int    Col    { get; set; }
        /// <summary>Row-major position: row * cols + col.</summary>
        [JsonPropertyName("index")]  public int    Index  { get; set; }
        [JsonPropertyName("value")]  public double Value  { get; set; }
        [JsonPropertyName("passed")] public bool   Passed { get; set; }
        /// <summary>The measured cell window (after inset) in pixels, TL, TR, BR, BL — for the overlay.</summary>
        [JsonPropertyName("quad")]   public List<PxPoint>? Quad { get; set; }
    }

    public class ToolResult
    {
        [JsonPropertyName("id")]      public string  Id      { get; set; } = "";
        [JsonPropertyName("type")]    public string  Type    { get; set; } = "";
        [JsonPropertyName("status")]  public string  Status  { get; set; } = ToolStatus.Ok;
        [JsonPropertyName("passed")]  public bool    Passed  { get; set; }
        [JsonPropertyName("message")] public string? Message { get; set; }
        [JsonPropertyName("timeMs")]  public double  TimeMs  { get; set; }
        /// <summary>Numbers (booleans as 0/1) and strings, by key.</summary>
        [JsonPropertyName("values")]  public Dictionary<string, object> Values { get; set; } = new();
        [JsonPropertyName("pose")]    public VisionPose? Pose { get; set; }
        [JsonPropertyName("points")]  public List<PxPoint> Points { get; set; } = new();
        [JsonPropertyName("lines")]   public List<PxLine>  Lines  { get; set; } = new();
        /// <summary>List records (blobs, matches, codes, markers…): numbers and strings by key.</summary>
        [JsonPropertyName("items")]   public List<Dictionary<string, object>> Items { get; set; } = new();
        [JsonPropertyName("cells")]   public List<CellResult> Cells { get; set; } = new();
        /// <summary>The region outline as actually used (after the fixture), pixels.</summary>
        [JsonPropertyName("region")]  public List<PxPoint> Region { get; set; } = new();

        public bool Ok => Status == ToolStatus.Ok;

        public void Set(string key, double v) => Values[key] = Math.Round(v, 3);
        public void Set(string key, bool v)   => Values[key] = v ? 1 : 0;
        public void Set(string key, string v) => Values[key] = v;

        public bool TryGetNumber(string key, out double v)
        {
            v = 0;
            if (!Values.TryGetValue(key, out var o)) return false;
            switch (o)
            {
                case double d: v = d; return true;
                case int i:    v = i; return true;
                case long l:   v = l; return true;
                case float f:  v = f; return true;
                case bool b:   v = b ? 1 : 0; return true;
                case JsonElement { ValueKind: JsonValueKind.Number } e: v = e.GetDouble(); return true;
                default: return false;
            }
        }
    }

    public class VisionResult
    {
        [JsonPropertyName("programId")]   public string           ProgramId   { get; set; } = "";
        /// <summary>Unix ms when the frame was processed.</summary>
        [JsonPropertyName("timestamp")]   public long             Timestamp   { get; set; }
        /// <summary>Size of the analysed frame, px.</summary>
        [JsonPropertyName("imageWidth")]  public int              ImageWidth  { get; set; }
        [JsonPropertyName("imageHeight")] public int              ImageHeight { get; set; }
        /// <summary>AND of every enabled tool with includeInPass.</summary>
        [JsonPropertyName("passed")]      public bool             Passed      { get; set; }
        [JsonPropertyName("tools")]       public List<ToolResult> Tools       { get; set; } = new();
        /// <summary>Ids of the layers computed for this frame.</summary>
        [JsonPropertyName("layers")]      public List<string>     Layers      { get; set; } = new();
        /// <summary>Milliseconds per tool id (and "layer:{id}" per computed layer).</summary>
        [JsonPropertyName("timings")]     public Dictionary<string, double> Timings { get; set; } = new();
        /// <summary>Error messages, "{toolId}: {message}".</summary>
        [JsonPropertyName("errors")]      public List<string>     Errors      { get; set; } = new();

        public ToolResult? Tool(string id) => Tools.FirstOrDefault(t => t.Id == id);
    }

    /// <summary>
    /// Typed reads of a raw JSON params bag (a tool's <c>params</c> or a filter's flat fields),
    /// with defaults for missing or mistyped entries. Numbers may also arrive as numeric strings.
    /// </summary>
    public readonly struct ParamReader
    {
        private readonly Dictionary<string, JsonElement>? _p;

        public ParamReader(Dictionary<string, JsonElement>? p) => _p = p;

        public bool Has(string key) => _p != null && _p.TryGetValue(key, out var e) && e.ValueKind is not (JsonValueKind.Null or JsonValueKind.Undefined);

        public JsonElement? Raw(string key) =>
            _p != null && _p.TryGetValue(key, out var e) && e.ValueKind is not (JsonValueKind.Null or JsonValueKind.Undefined) ? e : null;

        public double Num(string key, double fallback)
        {
            if (Raw(key) is not { } e) return fallback;
            return e.ValueKind switch
            {
                JsonValueKind.Number => e.GetDouble(),
                JsonValueKind.True   => 1,
                JsonValueKind.False  => 0,
                JsonValueKind.String => double.TryParse(e.GetString(), System.Globalization.NumberStyles.Float,
                                                        System.Globalization.CultureInfo.InvariantCulture, out var d) ? d : fallback,
                _ => fallback,
            };
        }

        public double? NumOrNull(string key)
        {
            double d = Num(key, double.NaN);
            return double.IsNaN(d) ? null : d;
        }

        public int Int(string key, int fallback) => (int)Math.Round(Num(key, fallback));

        public bool Bool(string key, bool fallback)
        {
            if (Raw(key) is not { } e) return fallback;
            return e.ValueKind switch
            {
                JsonValueKind.True   => true,
                JsonValueKind.False  => false,
                JsonValueKind.Number => e.GetDouble() != 0,
                JsonValueKind.String => e.GetString() is "true" or "1",
                _ => fallback,
            };
        }

        public string Str(string key, string fallback) =>
            Raw(key) is { ValueKind: JsonValueKind.String } e ? e.GetString() ?? fallback : fallback;

        public List<string> StrList(string key)
        {
            var list = new List<string>();
            if (Raw(key) is { ValueKind: JsonValueKind.Array } e)
                foreach (var x in e.EnumerateArray())
                    if (x.ValueKind == JsonValueKind.String && x.GetString() is { } s) list.Add(s);
            return list;
        }

        public T? Obj<T>(string key) where T : class
        {
            if (Raw(key) is not { } e || e.ValueKind is not (JsonValueKind.Object or JsonValueKind.Array)) return null;
            try { return e.Deserialize<T>(VisionJson.Options); }
            catch (JsonException) { return null; }
        }
    }

    internal static class VisionJson
    {
        public static readonly JsonSerializerOptions Options = new() { PropertyNameCaseInsensitive = true };
    }
}
