using System.Text.Json;
using System.Text.Json.Nodes;
using Controller.RobotControl;
using Controller.RobotControl.Language;
using static RobotControl.Tests.Language.SrsTestKit;

namespace RobotControl.Tests.Language;

/// <summary>
/// Round-trips every program of a real controller data directory, when one is given in
/// <c>SRS_REAL_DIR</c> (a copy of a data dir: builtPrograms/, points.json, tools.json, locals.json,
/// grids.json, vision_programs/). Skipped (passes trivially) otherwise.
/// </summary>
public class RealProgramTests
{
    [Fact]
    public void RealProgramsRoundTrip()
    {
        var dir = Environment.GetEnvironmentVariable("SRS_REAL_DIR");
        if (string.IsNullOrEmpty(dir) || !Directory.Exists(dir)) return;
        var names = new SrsNameTable();
        void Load(string file, SrsRefKind kind)
        {
            var path = Path.Combine(dir, file);
            if (!File.Exists(path)) return;
            foreach (var e in JsonNode.Parse(File.ReadAllText(path))!.AsArray())
            {
                var id = (e?["id"] ?? e?["Id"])?.GetValue<string>();
                var name = (e?["Name"] ?? e?["name"])?.GetValue<string>();
                if (id != null && name != null) names.Add(kind, id, name);
            }
        }
        Load("points.json", SrsRefKind.Point);
        Load("tools.json", SrsRefKind.Tool);
        Load("locals.json", SrsRefKind.Local);
        Load("grids.json", SrsRefKind.Grid);
        var programs = Directory.GetFiles(Path.Combine(dir, "builtPrograms"), "*.json")
            .Select(f => JsonSerializer.Deserialize<BuiltProgram>(File.ReadAllText(f), new JsonSerializerOptions
            {
                PropertyNameCaseInsensitive = true,
                Converters = { new System.Text.Json.Serialization.JsonStringEnumConverter() },
            })!).ToList();
        foreach (var p in programs) names.Add(SrsRefKind.Routine, p.Id, p.Name).Add(SrsRefKind.Program, p.Id, p.Name);
        var vdir = Path.Combine(dir, "vision_programs");
        if (Directory.Exists(vdir))
            foreach (var f in Directory.GetFiles(vdir, "*.json"))
            {
                var v = JsonNode.Parse(File.ReadAllText(f))!;
                var vid = v["id"]!.GetValue<string>();
                names.Add(SrsRefKind.Vision, vid, v["name"]!.GetValue<string>());
                foreach (var t in v["tools"]?.AsArray() ?? []) names.AddTool(vid, t!["id"]!.GetValue<string>(), t["name"]?.GetValue<string>() ?? "");
            }
        foreach (var p in programs)
        {
            var text = AssertRoundTrip(p, names);
            var outDir = Environment.GetEnvironmentVariable("SRS_REAL_OUT");
            if (!string.IsNullOrEmpty(outDir)) File.WriteAllText(Path.Combine(outDir, p.Name + ".srs"), text);
        }
    }
}
