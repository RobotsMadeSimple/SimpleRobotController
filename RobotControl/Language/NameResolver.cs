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

    public IReadOnlyList<SrsNamed> Tools(string? visionProgramId)
    {
        if (string.IsNullOrEmpty(visionProgramId)) return [];
        if (!_tools.TryGetValue(visionProgramId, out var l)) _tools[visionProgramId] = l = names.VisionTools(visionProgramId);
        return l;
    }

    /// <summary>A tool's id from its name (unique match) or its id.</summary>
    public string? ToolId(string? visionProgramId, string text)
    {
        var tools = Tools(visionProgramId);
        var byName = tools.Where(t => t.Name == text).ToList();
        if (byName.Count == 0) byName = tools.Where(t => string.Equals(t.Name, text, StringComparison.OrdinalIgnoreCase)).ToList();
        if (byName.Count == 1) return byName[0].Id;
        return null;
    }

    /// <summary>How a tool id is written: its name when that name is unique in the program,
    /// otherwise the id itself.</summary>
    public string ToolText(string? visionProgramId, string toolId)
    {
        var tools = Tools(visionProgramId);
        foreach (var t in tools)
            if (t.Id == toolId && !string.IsNullOrEmpty(t.Name) && ToolId(visionProgramId, t.Name) == toolId) return t.Name;
        return toolId;
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
}
