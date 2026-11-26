using System.IO.Compression;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using Controller.RobotControl;
using Controller.RobotControl.Cells;
using Controller.RobotControl.Commands;

namespace RobotControl.Tests;

/// <summary>
/// 3D cell packages (docs/cell-package.md): upload validation, the store/list/active/delete round
/// trip, resolving the built-in ASTRO robot with its adjustable settings, and the commands.
/// </summary>
public class CellPackageTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "rms-cells-" + Guid.NewGuid().ToString("N"));
    private string? _active;

    public CellPackageTests() => Directory.CreateDirectory(_dir);
    public void Dispose() { try { Directory.Delete(_dir, recursive: true); } catch { } }

    private CellRepository Repo() => new(_dir, () => _active, id => _active = id);

    private static readonly byte[] FakeGlb = Encoding.ASCII.GetBytes("glTF-fixture");

    private static string CellJson(string id = "test-cell", string? mesh = "meshes/board.glb", string format = "rms-cell", int version = 1) =>
        JsonSerializer.Serialize(new Dictionary<string, object?>
        {
            ["format"]  = format,
            ["version"] = version,
            ["id"]      = id,
            ["name"]    = "Test cell",
            ["tools"]   = new object[] { new { id = "gripper", name = "Gripper", controllerTool = "abc", shapes = Array.Empty<object>() } },
            ["fixtures"] = mesh is null
                ? Array.Empty<object>()
                : new object[] { new { id = "board", name = "Board", meshes = new[] { new { mesh } }, collision = true } },
        });

    private static byte[] Zip(params (string Name, byte[] Data)[] entries)
    {
        using var ms = new MemoryStream();
        using (var zip = new ZipArchive(ms, ZipArchiveMode.Create, leaveOpen: true))
            foreach (var (name, data) in entries)
            {
                using var s = zip.CreateEntry(name).Open();
                s.Write(data);
            }
        return ms.ToArray();
    }

    private static byte[] GoodPackage(string id = "test-cell") =>
        Zip(("cell.json", Encoding.UTF8.GetBytes(CellJson(id))), ("meshes/board.glb", FakeGlb));

    // ── validation ───────────────────────────────────────────────────────────

    [Fact]
    public void GoodPackageInstalls()
    {
        var repo = Repo();
        var r = repo.Install(GoodPackage(), replace: false);
        Assert.True(r.Ok, r.Message);
        Assert.Equal("test-cell", r.Id);
        Assert.True(File.Exists(Path.Combine(_dir, "cells", "test-cell", "cell.json")));
        Assert.True(File.Exists(Path.Combine(_dir, "cells", "test-cell", "meshes", "board.glb")));
        Assert.Empty(Directory.GetDirectories(Path.Combine(_dir, "cells"), ".*"));  // no staging left behind
    }

    [Fact]
    public void MissingCellJsonIsRejected()
    {
        var r = Repo().Install(Zip(("meshes/board.glb", FakeGlb)), false);
        Assert.False(r.Ok);
        Assert.Equal("badPackage", r.Error);
        Assert.Contains("cell.json", r.Message);
    }

    [Fact]
    public void CellJsonInsideAFolderIsNotAtTheRoot()
    {
        var r = Repo().Install(Zip(("pkg/cell.json", Encoding.UTF8.GetBytes(CellJson(mesh: null)))), false);
        Assert.False(r.Ok);
        Assert.Equal("badPackage", r.Error);
    }

    [Theory]
    [InlineData("Bad-Id")]
    [InlineData("-leading")]
    [InlineData("has space")]
    [InlineData("active")]
    [InlineData("")]
    public void BadIdIsRejected(string id)
    {
        var r = Repo().Install(Zip(("cell.json", Encoding.UTF8.GetBytes(CellJson(id, mesh: null)))), false);
        Assert.False(r.Ok);
        Assert.Equal("badPackage", r.Error);
        Assert.Contains("id", r.Message);
    }

    [Fact]
    public void IdLongerThan64IsRejectedAnd64IsAccepted()
    {
        Assert.True(CellPackageValidator.IsValidId(new string('a', 64)));
        Assert.False(CellPackageValidator.IsValidId(new string('a', 65)));
        Assert.True(CellPackageValidator.IsValidId("a_b-9"));
    }

    [Fact]
    public void WrongFormatOrVersionIsRejected()
    {
        Assert.False(Repo().Install(Zip(("cell.json", Encoding.UTF8.GetBytes(CellJson(format: "other", mesh: null)))), false).Ok);
        Assert.False(Repo().Install(Zip(("cell.json", Encoding.UTF8.GetBytes(CellJson(version: 2, mesh: null)))), false).Ok);
    }

    [Fact]
    public void MissingMeshIsRejected()
    {
        var r = Repo().Install(Zip(("cell.json", Encoding.UTF8.GetBytes(CellJson()))), false);
        Assert.False(r.Ok);
        Assert.Contains("meshes/board.glb", r.Message);
    }

    [Fact]
    public void MeshPathWithDotDotIsRejected()
    {
        var r = Repo().Install(Zip(("cell.json", Encoding.UTF8.GetBytes(CellJson(mesh: "meshes/../board.glb"))), ("board.glb", FakeGlb)), false);
        Assert.False(r.Ok);
        Assert.Contains("..", r.Message);
    }

    [Theory]
    [InlineData("../evil.txt")]
    [InlineData("meshes/../../evil.txt")]
    [InlineData("/abs.txt")]
    [InlineData("C:/evil.txt")]
    [InlineData("..\\evil.txt")]
    public void ZipSlipEntryIsRejectedAndNothingIsWritten(string entry)
    {
        var r = Repo().Install(Zip(("cell.json", Encoding.UTF8.GetBytes(CellJson(mesh: null))), (entry, FakeGlb)), false);
        Assert.False(r.Ok);
        Assert.Equal("badPackage", r.Error);
        Assert.False(File.Exists(Path.Combine(_dir, "evil.txt")));
        Assert.False(Directory.Exists(Path.Combine(_dir, "cells", "test-cell")));
    }

    [Fact]
    public void NotAZipIsBadZip()
    {
        var r = Repo().Install(Encoding.ASCII.GetBytes("not a zip"), false);
        Assert.False(r.Ok);
        Assert.Equal("badZip", r.Error);
    }

    [Fact]
    public void MeshPathsFindsEveryMeshReference()
    {
        var cell = JsonNode.Parse("""
            { "robot": { "links": [ { "meshes": [ { "mesh": "a.glb" } ] } ] },
              "tools": [ { "meshes": [ { "mesh": "b.glb" } ], "moving": [ { "meshes": [ { "mesh": "c.glb" } ] } ] } ],
              "fixtures": [ { "meshes": [ { "mesh": "d.glb" } ] } ] }
            """);
        Assert.Equal(new[] { "a.glb", "b.glb", "c.glb", "d.glb" }, CellPackageValidator.MeshPaths(cell).ToArray());
    }

    // ── store / list / active / delete ───────────────────────────────────────

    [Fact]
    public void StoreListActiveDeleteRoundTrip()
    {
        var repo = Repo();
        long t0 = repo.LastUpdatedUnixMs;
        Assert.True(repo.Install(GoodPackage(), false).Ok);
        long t1 = repo.LastUpdatedUnixMs;
        Assert.True(t1 > t0);

        // A second upload without replace conflicts; with replace it swaps in.
        var dup = repo.Install(GoodPackage(), false);
        Assert.False(dup.Ok);
        Assert.Equal("idExists", dup.Error);
        Assert.True(repo.Install(GoodPackage(), replace: true).Ok);

        var list = repo.List();
        var s = Assert.Single(list);
        Assert.Equal("test-cell", s.Id);
        Assert.Equal("Test cell", s.Name);
        Assert.Equal(1, s.Tools);
        Assert.Equal(1, s.Fixtures);
        Assert.False(s.Active);
        Assert.True(s.UpdatedUnixMs > 0);

        Assert.False(repo.SetActive("nope"));
        long t2 = repo.LastUpdatedUnixMs;
        Assert.True(repo.SetActive("test-cell"));
        Assert.Equal("test-cell", _active);
        Assert.Equal("test-cell", repo.ActiveId);
        Assert.True(repo.LastUpdatedUnixMs > t2);
        Assert.True(Assert.Single(repo.List()).Active);

        Assert.NotNull(repo.FilePath("test-cell", "meshes/board.glb"));
        Assert.Null(repo.FilePath("test-cell", "../test-cell/cell.json"));
        Assert.Null(repo.FilePath("test-cell", "meshes/missing.glb"));

        long t3 = repo.LastUpdatedUnixMs;
        Assert.True(repo.Delete("test-cell"));
        Assert.Null(_active);                // deleting the active package clears it
        Assert.True(repo.LastUpdatedUnixMs > t3);
        Assert.Empty(repo.List());
        Assert.False(repo.Delete("test-cell"));
        Assert.Empty(Directory.GetDirectories(Path.Combine(_dir, "cells")));
    }

    [Fact]
    public void ActiveIdOfAMissingPackageReadsAsNone()
    {
        _active = "gone";
        Assert.Null(Repo().ActiveId);
    }

    [Fact]
    public void CommandsListSetActiveAndDelete()
    {
        var repo = Repo();
        Assert.True(repo.Install(GoodPackage(), false).Ok);
        var d = new CommandDispatcher();
        new CellCommands(repo).Register(d);

        JsonElement Send(string command, object? parameters = null)
        {
            Assert.True(d.TryGet(command, out var h));
            var msg = new CommandMessage { Type = "Command", Id = "t", Command = command,
                Params = parameters is null ? null : JsonSerializer.SerializeToElement(parameters) };
            return JsonSerializer.SerializeToElement(h(msg).GetAwaiter().GetResult());
        }

        var cells = Send("GetCells");
        Assert.Equal("test-cell", cells.GetProperty("cells")[0].GetProperty("id").GetString());
        Assert.Equal(JsonValueKind.Null, cells.GetProperty("activeCellId").ValueKind);

        Assert.Equal("test-cell", Send("SetActiveCell", new { id = "test-cell" }).GetProperty("activeCellId").GetString());
        Assert.True(Send("GetCells").GetProperty("cells")[0].GetProperty("active").GetBoolean());
        Assert.Equal("unknownCell", Send("SetActiveCell", new { id = "nope" }).GetProperty("error").GetString());
        Assert.Equal(JsonValueKind.Null, Send("SetActiveCell", new { id = (string?)null }).GetProperty("activeCellId").ValueKind);

        Assert.Equal("unknownCell", Send("DeleteCell", new { id = "nope" }).GetProperty("error").GetString());
        Assert.Equal(JsonValueKind.Null, Send("DeleteCell", new { id = "test-cell" }).ValueKind);
        Assert.Empty(Send("GetCells").GetProperty("cells").EnumerateArray());
    }

    // ── resolve + built-in robot ─────────────────────────────────────────────

    [Fact]
    public void ResolveFillsTheBuiltInAstroWithSettingValues()
    {
        var repo = Repo();
        Assert.True(repo.Install(GoodPackage(), false).Ok);
        var config = new RobotConfig { AstroVerticalTravelMm = 600, AstroRadialTravelMm = 350 };

        var resolved = CellResolver.Resolve(repo.Get("test-cell"), config);
        Assert.Equal("test-cell", resolved["id"]!.GetValue<string>());
        Assert.Equal("ASTRO", resolved["robot"]!["name"]!.GetValue<string>());
        Assert.Equal("Wrist", resolved["robot"]!["flange"]!.GetValue<string>());
        Assert.Equal(600, resolved["settings"]!["astroVerticalTravelMm"]!.GetValue<double>());
        Assert.Equal(350, resolved["settings"]!["astroRadialTravelMm"]!.GetValue<double>());
        Assert.Single(resolved["tools"]!.AsArray());

        // The stored cell.json itself is untouched.
        Assert.Null(repo.Get("test-cell")!["settings"]);
        Assert.Null(repo.Get("test-cell")!["robot"]);
    }

    [Fact]
    public void ResolveWithNoPackageIsTheBuiltInRobotAlone()
    {
        var resolved = CellResolver.Resolve(null, new RobotConfig());
        Assert.Equal("rms-cell", resolved["format"]!.GetValue<string>());
        Assert.Null(resolved["id"]);
        Assert.Equal("ASTRO", resolved["robot"]!["name"]!.GetValue<string>());
        Assert.Empty(resolved["tools"]!.AsArray());
        Assert.Empty(resolved["fixtures"]!.AsArray());
        Assert.Equal(400, resolved["settings"]!["astroVerticalTravelMm"]!.GetValue<double>());
        Assert.Equal(400, resolved["settings"]!["astroRadialTravelMm"]!.GetValue<double>());
    }

    [Fact]
    public void Cnc4AxisHasNoBuiltInRobot()
    {
        var resolved = CellResolver.Resolve(null, new RobotConfig { RobotType = "CNC4Axis" });
        Assert.Null(resolved["robot"]);
        Assert.Empty(resolved["settings"]!.AsObject());
    }

    [Fact]
    public void APackageRobotIsKeptAndItsUnknownSettingsReadNominal()
    {
        var cell = JsonNode.Parse("""
            { "format": "rms-cell", "version": 1, "id": "x",
              "robot": { "name": "Custom", "adjustable": [ { "id": "z", "setting": "customLenMm", "nominal": 250, "direction": [0,0,1] } ] } }
            """)!.AsObject();
        var resolved = CellResolver.Resolve(cell, new RobotConfig());
        Assert.Equal("Custom", resolved["robot"]!["name"]!.GetValue<string>());
        Assert.Equal(250, resolved["settings"]!["customLenMm"]!.GetValue<double>());
    }

    [Fact]
    public void BuiltInAstroParsesWithAdjustableTagsOnTheExpectedShapes()
    {
        var robot = BuiltInRobots.For("ASTRO")!;
        Assert.Null(robot["controller"]);  // FusionRobotSim's ws connection block is not part of the definition
        Assert.Equal(4, robot["joints"]!.AsArray().Count);

        var adjustable = robot["adjustable"]!.AsArray();
        Assert.Equal(2, adjustable.Count);
        Assert.Equal("vertical", adjustable[0]!["id"]!.GetValue<string>());
        Assert.Equal("astroVerticalTravelMm", adjustable[0]!["setting"]!.GetValue<string>());
        Assert.Equal(400, adjustable[0]!["nominal"]!.GetValue<double>());
        Assert.Equal(new[] { 0.0, 0, 1 }, adjustable[0]!["direction"]!.AsArray().Select(n => n!.GetValue<double>()));
        Assert.Equal("radial", adjustable[1]!["id"]!.GetValue<string>());
        Assert.Equal("astroRadialTravelMm", adjustable[1]!["setting"]!.GetValue<string>());
        Assert.Equal(new[] { 0.0, 1, 0 }, adjustable[1]!["direction"]!.AsArray().Select(n => n!.GetValue<double>()));

        JsonArray Shapes(string link) => robot["links"]!.AsArray().First(l => l!["name"]!.GetValue<string>() == link)!["shapes"]!.AsArray();
        static string? Tag(JsonNode? s) => s?["adjust"] is { } a ? $"{a["id"]!.GetValue<string>()}:{a["role"]!.GetValue<string>()}" : null;
        static double Min(JsonNode s, int axis) => s["min"]![axis]!.GetValue<double>();
        static double Max(JsonNode s, int axis) => s["max"]![axis]!.GetValue<double>();

        // Vertical: every Turret box spanning z 94.7–494.7 stretches; every Turret box at z ≥ 494.7 is "end".
        foreach (var s in Shapes("Turret"))
        {
            bool isBox = s!["type"]!.GetValue<string>() == "box";
            string? expected = !isBox ? null
                : Min(s, 2) == 94.7 && Max(s, 2) == 494.7 ? "vertical:stretch"
                : Min(s, 2) >= 494.7 ? "vertical:end" : null;
            Assert.Equal(expected, Tag(s));
        }
        Assert.Equal(3, Shapes("Turret").Count(s => Tag(s) == "vertical:stretch"));
        Assert.Equal(3, Shapes("Turret").Count(s => Tag(s) == "vertical:end"));

        // Radial: the reach extrusion + both rails stretch; the back end plate (y 187.7–199.7) is "end".
        foreach (var s in Shapes("Reach"))
        {
            bool isBox = s!["type"]!.GetValue<string>() == "box";
            string? expected = !isBox ? null
                : (Min(s, 1), Max(s, 1)) is (-292.3, 187.7) or (-252.3, 147.7) ? "radial:stretch"
                : (Min(s, 1), Max(s, 1)) is (187.7, 199.7) ? "radial:end" : null;
            Assert.Equal(expected, Tag(s));
        }
        Assert.Equal(3, Shapes("Reach").Count(s => Tag(s) == "radial:stretch"));
        Assert.Equal(1, Shapes("Reach").Count(s => Tag(s) == "radial:end"));

        // No other link is tagged.
        foreach (var link in new[] { "Base", "Carriage", "Wrist" })
            Assert.All(Shapes(link), s => Assert.Null(Tag(s)));

        // Each call is a fresh copy.
        BuiltInRobots.For("ASTRO")!["name"] = "changed";
        Assert.Equal("ASTRO", BuiltInRobots.For("astro")!["name"]!.GetValue<string>());
        Assert.Null(BuiltInRobots.For("CNC4Axis"));
    }

    // ── config ───────────────────────────────────────────────────────────────

    [Fact]
    public void ConfigDefaults()
    {
        var c = new RobotConfig();
        Assert.Equal(400, c.AstroVerticalTravelMm);
        Assert.Equal(400, c.AstroRadialTravelMm);
        Assert.Null(c.ActiveCellId);

        // Persisted under the camelCase names; an old robot-config.json without them gets the defaults.
        var json = JsonSerializer.Serialize(c);
        Assert.Contains("\"astroVerticalTravelMm\":400", json);
        Assert.Contains("\"astroRadialTravelMm\":400", json);
        Assert.Contains("\"activeCellId\":null", json);
        var old = JsonSerializer.Deserialize<RobotConfig>("{\"robotType\":\"ASTRO\"}")!;
        Assert.Equal(400, old.AstroVerticalTravelMm);
        Assert.Null(old.ActiveCellId);

        var p = JsonSerializer.Deserialize<SetRobotConfigParams>("{\"astroVerticalTravelMm\":550,\"astroRadialTravelMm\":300}")!;
        Assert.Equal(550, p.AstroVerticalTravelMm);
        Assert.Equal(300, p.AstroRadialTravelMm);
    }
}
