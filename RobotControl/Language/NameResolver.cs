using Controller.RobotControl.Validation;

namespace Controller.RobotControl.Language;

/// <summary>The repositories the text language names things in.</summary>
public enum SrsRefKind { Point, Tool, Local, Grid, Stack, Routine, Program, Vision, Device, Camera }

/// <summary>One named thing: its stable id and its current name.</summary>
public readonly record struct SrsNamed(string Id, string Name);

/// <summary>
/// Where names come from. The controller's implementation (<see cref="ControllerSrsNames"/>)
/// reads the live repositories; tests use <see cref="SrsNameTable"/>.
/// </summary>
public interface ISrsNames
{
    /// <summary>Every item of a kind, in preference order (the first match of a name wins).</summary>
    IReadOnlyList<SrsNamed> List(SrsRefKind kind);

    /// <summary>The tools of a vision program (id + name), or empty when it is unknown.</summary>
    IReadOnlyList<SrsNamed> VisionTools(string visionProgramId);
}

/// <summary>An in-memory <see cref="ISrsNames"/>.</summary>
public sealed class SrsNameTable : ISrsNames
{
    public Dictionary<SrsRefKind, List<SrsNamed>> Items { get; } = new();
    public Dictionary<string, List<SrsNamed>> Tools { get; } = new();

    public SrsNameTable Add(SrsRefKind kind, string id, string name)
    {
        if (!Items.TryGetValue(kind, out var l)) Items[kind] = l = [];
        l.Add(new SrsNamed(id, name));
        return this;
    }

    public SrsNameTable AddTool(string visionProgramId, string toolId, string toolName)
    {
        if (!Tools.TryGetValue(visionProgramId, out var l)) Tools[visionProgramId] = l = [];
        l.Add(new SrsNamed(toolId, toolName));
        return this;
    }

    public IReadOnlyList<SrsNamed> List(SrsRefKind kind) => Items.TryGetValue(kind, out var l) ? l : [];
    public IReadOnlyList<SrsNamed> VisionTools(string id) => Tools.TryGetValue(id, out var l) ? l : [];

    public static readonly SrsNameTable Empty = new();
}

/// <summary>Names from the running controller's repositories.</summary>
internal sealed class ControllerSrsNames(RobotController robot) : ISrsNames
{
    public IReadOnlyList<SrsNamed> List(SrsRefKind kind)
    {
        try
        {
            return kind switch
            {
                SrsRefKind.Point => Vectors(robot.pointRepo.Items.Values.Select(p => (p.Id, p.Name))),
                SrsRefKind.Tool  => Vectors(robot.toolRepo.Items.Values.Select(p => (p.Id, p.Name))),
                SrsRefKind.Local => Vectors(robot.localRepo.Items.Values.Select(p => (p.Id, p.Name))),
                SrsRefKind.Grid  => robot.gridRepo.GetAll().Select(g => new SrsNamed(g.Id, g.Name)).ToList(),
                SrsRefKind.Stack => robot.stackRepo.GetAll().Select(s => new SrsNamed(s.Id, s.Name)).ToList(),
                SrsRefKind.Routine => robot.builtProgramRepo.GetAll().OrderByDescending(p => p.IsRoutine)
                                          .Select(p => new SrsNamed(p.Id, p.Name)).ToList(),
                SrsRefKind.Program => robot.builtProgramRepo.GetAll().OrderByDescending(p => p.IsBackground)
                                          .Select(p => new SrsNamed(p.Id, p.Name)).ToList(),
                SrsRefKind.Vision => robot.VisionRepo?.GetAll().Select(v => new SrsNamed(v.Id, v.Name)).ToList() ?? [],
                SrsRefKind.Device => Devices(),
                SrsRefKind.Camera => robot.CameraManager?.GetState().Select(c => new SrsNamed(c.Id, c.Name)).ToList() ?? [],
                _ => [],
            };
        }
        catch { return []; }
    }

    private static List<SrsNamed> Vectors(IEnumerable<(string? Id, string? Name)> items) =>
        items.Where(i => !string.IsNullOrEmpty(i.Id) && i.Name != null)
             .Select(i => new SrsNamed(i.Id!, i.Name!)).ToList();

    private List<SrsNamed> Devices()
    {
        var list = new List<SrsNamed>();
        var seen = new HashSet<string>();
        try
        {
            foreach (var d in robot.NanoManager?.GetAllStates() ?? [])
                if (seen.Add(d.Id)) list.Add(new SrsNamed(d.Id, d.Name));
        }
        catch { }
        try
        {
            foreach (var a in robot.AuxAxisManager?.GetState() ?? [])
                if (seen.Add(a.DeviceId)) list.Add(new SrsNamed(a.DeviceId, a.DeviceName));
        }
        catch { }
        return list;
    }

    public IReadOnlyList<SrsNamed> VisionTools(string visionProgramId)
    {
        try
        {
            return robot.VisionRepo?.Get(visionProgramId)?.Tools?.Select(t => new SrsNamed(t.Id, t.Name)).ToList() ?? [];
        }
        catch { return []; }
    }
}

/// <summary>
/// Name ↔ id lookups over an <see cref="ISrsNames"/>, cached per instance. A name matches
/// exactly first, then case-insensitively; the first item in the list wins a tie.
/// </summary>
internal sealed class NameResolver(ISrsNames names)
{
    private readonly Dictionary<SrsRefKind, IReadOnlyList<SrsNamed>> _cache = new();
    private readonly Dictionary<string, IReadOnlyList<SrsNamed>> _tools = new();

    public ISrsNames Source => names;

    public IReadOnlyList<SrsNamed> All(SrsRefKind kind)
    {
        if (!_cache.TryGetValue(kind, out var l)) _cache[kind] = l = names.List(kind);
        return l;
    }

    public string? NameOf(SrsRefKind kind, string? id)
    {
        if (string.IsNullOrEmpty(id)) return null;
        foreach (var x in All(kind)) if (x.Id == id) return x.Name;
        return null;
    }

    public bool IdExists(SrsRefKind kind, string? id) =>
        !string.IsNullOrEmpty(id) && All(kind).Any(x => x.Id == id);

    /// <summary>The id named <paramref name="name"/>, or null.</summary>
    public string? IdOf(SrsRefKind kind, string? name) => Find(All(kind), name)?.Id;

    /// <summary>The canonical name for <paramref name="name"/> (the repository's spelling), or null.</summary>
    public string? CanonicalName(SrsRefKind kind, string? name) => Find(All(kind), name)?.Name;

    /// <summary>What a name written in the text refers to: exact, then case-insensitive, then by
    /// its underscore form (<see cref="SrsNameMatch"/>). Only the text parser uses this — stored
    /// JSON names resolve through <see cref="IdOf"/> (no underscore fallback).</summary>
    public SrsNameMatch Resolve(SrsRefKind kind, string? text) => SrsNameMatch.Find(All(kind), text, firstWins: true);

    /// <summary>How the text writes the item <paramref name="id"/> named <paramref name="name"/>:
    /// bare when the name is an identifier, else its underscore form when that reads back as this
    /// same item, else quoted (null when it cannot be written).</summary>
    public string? RefName(SrsRefKind kind, string? id, string? name) =>
        SrsNameMatch.Text(name, u => Resolve(kind, u).Item is { } m && m.Name == name && (string.IsNullOrEmpty(id) || m.Id == id));

    public IReadOnlyList<SrsNamed> Tools(string? visionProgramId)
    {
        if (string.IsNullOrEmpty(visionProgramId)) return [];
        if (!_tools.TryGetValue(visionProgramId, out var l)) _tools[visionProgramId] = l = names.VisionTools(visionProgramId);
        return l;
    }

    /// <summary>A tool's id from its name (unique match, then its underscore form) or null.</summary>
    public string? ToolId(string? visionProgramId, string text) => ResolveTool(visionProgramId, text).Item?.Id;

    /// <summary>A vision tool named in the text: a name must be unique in the program (no
    /// first-wins), exact, then case-insensitive, then by its underscore form.</summary>
    public SrsNameMatch ResolveTool(string? visionProgramId, string text) =>
        SrsNameMatch.Find(Tools(visionProgramId), text, firstWins: false);

    /// <summary>How a tool id is written: its name when that name is unique in the program,
    /// otherwise the id itself.</summary>
    public string ToolText(string? visionProgramId, string toolId)
    {
        var tools = Tools(visionProgramId);
        foreach (var t in tools)
            if (t.Id == toolId && !string.IsNullOrEmpty(t.Name) && ToolId(visionProgramId, t.Name) == toolId) return t.Name;
        return toolId;
    }

    /// <summary><see cref="ToolText"/> as a token: bare, underscore form or quoted.</summary>
    public string? ToolRefText(string? visionProgramId, string toolId)
    {
        var t = ToolText(visionProgramId, toolId);
        return SrsNameMatch.Text(t, u => ToolId(visionProgramId, u) == toolId);
    }

    private static SrsNamed? Find(IReadOnlyList<SrsNamed> list, string? name)
    {
        if (name == null) return null;
        foreach (var x in list) if (x.Name == name) return x;
        foreach (var x in list) if (string.Equals(x.Name, name, StringComparison.OrdinalIgnoreCase)) return x;
        return null;
    }

    /// <summary>The parse-warning code for a name that does not resolve.</summary>
    public static string UnknownCode(SrsRefKind kind) => kind switch
    {
        SrsRefKind.Point   => ValidationCodes.UnknownPoint,
        SrsRefKind.Tool    => ValidationCodes.UnknownTool,
        SrsRefKind.Local   => ValidationCodes.UnknownLocal,
        SrsRefKind.Grid    => ValidationCodes.UnknownGrid,
        SrsRefKind.Stack   => ValidationCodes.UnknownStack,
        SrsRefKind.Routine => ValidationCodes.UnknownRoutine,
        SrsRefKind.Program => ValidationCodes.UnknownProgram,
        SrsRefKind.Vision  => ValidationCodes.UnknownVisionProgram,
        SrsRefKind.Device  => TextProblemCodes.UnknownDevice,
        SrsRefKind.Camera  => TextProblemCodes.UnknownCamera,
        _ => TextProblemCodes.Syntax,
    };

    public static string KindLabel(SrsRefKind kind) => kind switch
    {
        SrsRefKind.Routine => "routine",
        SrsRefKind.Program => "program",
        SrsRefKind.Vision  => "vision program",
        _ => kind.ToString().ToLowerInvariant(),
    };

    /// <summary>The <see cref="TextProblemCodes.AmbiguousName"/> message for a name whose
    /// underscore form matches several items.</summary>
    public static string AmbiguousMessage(string what, string text, IReadOnlyList<SrsNamed> candidates)
    {
        var names = string.Join(", ", candidates.Take(4).Select(c => SrsText.Quote(c.Name)));
        var first = candidates.Count > 0 ? SrsText.Quote(candidates[0].Name) : "\"…\"";
        return $"'{text}' could mean more than one {what} ({names}) — write the name in quotes, e.g. {first}";
    }
}

/// <summary>
/// The result of resolving a name written in the text. A name with spaces (or other characters
/// an identifier cannot hold) may be written as an identifier in its <b>underscore form</b>:
/// every run of non-identifier characters becomes one <c>_</c> — <c>"Pick 1"</c> is
/// <c>Pick_1</c>, <c>"Bin - left"</c> is <c>Bin_left</c>.
/// <para>Matching order: an exact name, then a case-insensitive name (the language's names are
/// case-insensitive), then the underscore form — case-sensitively first, then case-insensitively.
/// An underscore form that fits two different items (and no name matches exactly) is
/// <see cref="Ambiguous"/>: the text must quote the real name.</para>
/// </summary>
public readonly record struct SrsNameMatch(SrsNamed? Item, IReadOnlyList<SrsNamed> Candidates)
{
    /// <summary>True when the underscore form fits several items — quote the name instead.</summary>
    public bool Ambiguous => Item == null && Candidates.Count > 1;

    public static readonly SrsNameMatch None = new(null, []);

    /// <summary><paramref name="name"/> with each run of non-identifier characters as one <c>_</c>.</summary>
    public static string Underscore(string name)
    {
        var sb = new System.Text.StringBuilder(name.Length);
        bool run = false;
        foreach (var ch in name)
        {
            if (SrsCursor.IsIdentChar(ch)) { sb.Append(ch); run = false; }
            else if (!run) { sb.Append('_'); run = true; }
        }
        return sb.ToString();
    }

    /// <summary>Resolves <paramref name="text"/> in <paramref name="list"/>. With
    /// <paramref name="firstWins"/> the first exact / case-insensitive match wins a tie;
    /// without it such a tie matches nothing (vision tools must be unique).</summary>
    public static SrsNameMatch Find(IReadOnlyList<SrsNamed> list, string? text, bool firstWins)
    {
        if (string.IsNullOrEmpty(text) || list.Count == 0) return None;
        foreach (var cmp in new[] { StringComparison.Ordinal, StringComparison.OrdinalIgnoreCase })
        {
            var hits = list.Where(x => string.Equals(x.Name, text, cmp)).ToList();
            if (hits.Count == 1 || (hits.Count > 1 && firstWins)) return new(hits[0], [hits[0]]);
            if (hits.Count > 1) return None;
        }
        var u = Underscore(text);
        foreach (var cmp in new[] { StringComparison.Ordinal, StringComparison.OrdinalIgnoreCase })
        {
            var hits = new List<SrsNamed>();
            var seen = new HashSet<string>(StringComparer.Ordinal);
            foreach (var x in list)
                if (!string.IsNullOrEmpty(x.Name) && string.Equals(Underscore(x.Name), u, cmp) && seen.Add(x.Id + "\u0001" + x.Name))
                    hits.Add(x);
            if (hits.Count == 1) return new(hits[0], hits);
            if (hits.Count > 1) return new(null, hits);
        }
        return None;
    }

    /// <summary>
    /// How a name is written: bare when it is an identifier (and not a reserved word), else its
    /// underscore form when that is an identifier, is no keyword and <paramref name="readsBack"/>
    /// confirms it resolves to this same item, else quoted. Null when it cannot be written.
    /// </summary>
    public static string? Text(string? name, Func<string, bool> readsBack)
    {
        if (string.IsNullOrEmpty(name) || !SrsText.Quotable(name)) return null;
        var plain = SrsText.Name(name);
        if (plain == name) return plain;
        var u = Underscore(name);
        if (u != name && SrsCursor.IsIdent(u) && !SrsText.IsReservedOrKeyword(u) && readsBack(u)) return u;
        return SrsText.Quote(name);
    }
}
