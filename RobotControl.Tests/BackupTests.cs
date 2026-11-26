using System.IO.Compression;
using System.Text;
using System.Text.Json;
using Controller.RobotControl.Backup;

namespace RobotControl.Tests;

/// <summary>
/// Whole-controller backup and restore (BackupService): the allowlist (user data in, caches, logs,
/// IO samples and plugin binaries out), the manifest, the round trip into a fresh data folder, the
/// preview counts, the pre-restore backup, and refusing zips with unsafe paths or no manifest.
/// </summary>
public class BackupTests : IDisposable
{
    private readonly string _root = Path.Combine(Path.GetTempPath(), "rms-backup-" + Guid.NewGuid().ToString("N"));
    private string A => Path.Combine(_root, "a");
    private string B => Path.Combine(_root, "b");

    private static readonly BackupIdentity Id = new("1.2.3", "ASTRO", "Bench", "SN-1",
        [new BackupPlugin("checkers", "Checkers", "0.4.0")]);

    public BackupTests()
    {
        Directory.CreateDirectory(A);
        Directory.CreateDirectory(B);
    }

    public void Dispose()
    {
        try { Directory.Delete(_root, recursive: true); } catch { }
    }

    private static void Write(string dir, string rel, string text)
    {
        var full = Path.Combine(dir, rel);
        Directory.CreateDirectory(Path.GetDirectoryName(full)!);
        File.WriteAllText(full, text);
    }

    private void SeedA()
    {
        Write(A, "robot-config.json", "{\"robotType\":\"ASTRO\",\"homingSpeed\":12}");
        Write(A, "points.json", "[{\"Id\":\"p1\",\"Name\":\"Home\"},{\"Id\":\"p2\",\"Name\":\"Pick\"}]");
        Write(A, "tools.json", "[{\"Id\":\"t1\",\"Name\":\"Gripper\"}]");
        Write(A, "io_devices.json", "{\"ioDevices\":[{\"id\":\"nano1\",\"name\":\"Cell IO\",\"pins\":[]}]}");
        Write(A, "persistent_vars.json", "{\"count\":3}");
        Write(A, "builtPrograms/Main.json", "{\"id\":\"main\",\"name\":\"Main\"}");
        Write(A, "builtPrograms/Sort.json", "{\"id\":\"sort\",\"name\":\"Sort\"}");
        Write(A, "builtPrograms/.revisions/Main/r1.json", "{}");
        Write(A, "vision_programs/v1.json", "{}");
        Write(A, "cells/bench/cell.json", "{}");
        Write(A, "cells/bench/meshes/a.glb", "glTF");
        Write(A, "pluginConfigs/checkers.json", "{\"depth\":4}");
        // Not user data — must never be in a backup:
        Write(A, "identity.json", "{\"serialNumber\":\"SN-1\"}");
        Write(A, "alarms.json", "[]");
        Write(A, "restart-controller.log", "log");
        Write(A, "io_history/index.json", "{}");
        Write(A, "io_history/samples/0001.bin", "samples");
        Write(A, "plugins/checkers/plugin.json", "{}");
        Write(A, "plugins/checkers/Checkers.dll", "binary");
        Write(A, "SimpleRobotController.dll", "binary");
        Write(A, "points.json.tmp", "half-written");
    }

    private static List<string> Entries(byte[] zip)
    {
        using var z = new ZipArchive(new MemoryStream(zip));
        return z.Entries.Select(e => e.FullName).ToList();
    }

    private static byte[] Zip(params (string Name, string Text)[] entries)
    {
        using var ms = new MemoryStream();
        using (var z = new ZipArchive(ms, ZipArchiveMode.Create, leaveOpen: true))
            foreach (var (name, text) in entries)
            {
                using var s = z.CreateEntry(name).Open();
                s.Write(Encoding.UTF8.GetBytes(text));
            }
        return ms.ToArray();
    }

    private static string Manifest(int version = 1, string format = BackupManifest.FormatName) =>
        JsonSerializer.Serialize(new { format, formatVersion = version, controllerVersion = "1.0.0", plugins = Array.Empty<object>() });

    [Fact]
    public void BackupHoldsOnlyUserDataAndAManifest()
    {
        SeedA();
        var zip = new BackupService(A).Create(Id);
        var names = Entries(zip);

        Assert.Contains("manifest.json", names);
        foreach (var expected in new[] { "robot-config.json", "points.json", "tools.json", "io_devices.json", "persistent_vars.json",
                                         "builtPrograms/Main.json", "builtPrograms/.revisions/Main/r1.json", "vision_programs/v1.json",
                                         "cells/bench/cell.json", "cells/bench/meshes/a.glb", "pluginConfigs/checkers.json" })
            Assert.Contains(expected, names);
        foreach (var excluded in new[] { "identity.json", "alarms.json", "restart-controller.log", "io_history/index.json",
                                         "io_history/samples/0001.bin", "plugins/checkers/plugin.json", "plugins/checkers/Checkers.dll",
                                         "SimpleRobotController.dll", "points.json.tmp" })
            Assert.DoesNotContain(excluded, names);

        using var z = new ZipArchive(new MemoryStream(zip));
        using var r = new StreamReader(z.GetEntry("manifest.json")!.Open());
        var m = JsonSerializer.Deserialize<BackupManifest>(r.ReadToEnd())!;
        Assert.Equal(BackupManifest.FormatName, m.Format);
        Assert.Equal(1, m.FormatVersion);
        Assert.Equal("1.2.3", m.ControllerVersion);
        Assert.Equal("ASTRO", m.RobotType);
        Assert.Equal("SN-1", m.SerialNumber);
        Assert.True(m.CreatedUnixMs > 0);
        Assert.Equal("checkers", Assert.Single(m.Plugins).Id);
        Assert.Equal("0.4.0", m.Plugins[0].Version);
    }

    [Fact]
    public void RoundTripRestoresIntoAnotherDataFolder()
    {
        SeedA();
        var zip = new BackupService(A).Create(Id);

        // B has its own, different data — some of it not in the backup at all.
        Write(B, "points.json", "[{\"Id\":\"x\",\"Name\":\"Other\"}]");
        Write(B, "grids.json", "[{\"Id\":\"g\"}]");
        Write(B, "builtPrograms/Old.json", "{}");
        Write(B, "identity.json", "{\"serialNumber\":\"SN-2\"}");
        Write(B, "plugins/other/plugin.json", "{}");

        var svc = new BackupService(B);
        var preview = svc.Preview(zip, Id with { SerialNumber = "SN-2", Plugins = [] });
        var programs = preview.Categories.Single(c => c.Key == "programs");
        Assert.Equal(1, programs.Current);
        Assert.Equal(2, programs.Backup);
        var points = preview.Categories.Single(c => c.Key == "points");
        Assert.Equal((1, 2), (points.Current, points.Backup));
        Assert.Equal((1, 0), preview.Categories.Single(c => c.Key == "grids") is var g ? (g.Current, g.Backup) : default);
        Assert.Equal(1, preview.Categories.Single(c => c.Key == "cells").Backup);
        Assert.Equal(1, preview.Categories.Single(c => c.Key == "ioDevices").Backup);
        Assert.Equal("checkers", Assert.Single(preview.MissingPlugins).Id);
        Assert.Contains(preview.Warnings, w => w.Contains("SN-1"));
        // Preview changes nothing.
        Assert.True(File.Exists(Path.Combine(B, "grids.json")));

        var result = svc.Apply(zip, Id with { SerialNumber = "SN-2" });
        Assert.StartsWith("backups/pre-restore-", result.PreRestoreBackup);

        foreach (var rel in new BackupService(A).EnumerateUserFiles())
            Assert.Equal(File.ReadAllText(Path.Combine(A, rel)), File.ReadAllText(Path.Combine(B, rel)));
        // Allowlisted data the backup did not have is gone; everything else is untouched.
        Assert.False(File.Exists(Path.Combine(B, "grids.json")));
        Assert.False(File.Exists(Path.Combine(B, "builtPrograms", "Old.json")));
        Assert.Contains("SN-2", File.ReadAllText(Path.Combine(B, "identity.json")));
        Assert.True(File.Exists(Path.Combine(B, "plugins", "other", "plugin.json")));
        // No staging leftovers.
        Assert.Empty(Directory.GetDirectories(B, ".restore-*"));

        // The pre-restore backup holds B's data as it was, and restores it back.
        var pre = File.ReadAllBytes(Path.Combine(B, result.PreRestoreBackup));
        Assert.Contains("grids.json", Entries(pre));
        svc.Apply(pre, Id);
        Assert.True(File.Exists(Path.Combine(B, "grids.json")));
        Assert.Contains("Other", File.ReadAllText(Path.Combine(B, "points.json")));
    }

    [Theory]
    [InlineData("../evil.json")]
    [InlineData("builtPrograms/../../evil.json")]
    [InlineData("/etc/passwd")]
    [InlineData("C:/Windows/evil.json")]
    [InlineData("builtPrograms\\..\\..\\evil.json")]
    [InlineData("builtPrograms//x.json")]
    public void PathEscapesAreRefusedAndNothingIsWritten(string entry)
    {
        Write(B, "points.json", "[]");
        var zip = Zip(("manifest.json", Manifest()), ("points.json", "[{\"Id\":\"p\"}]"), (entry, "{}"));
        var svc = new BackupService(B);
        var ex = Assert.Throws<BackupException>(() => svc.Preview(zip, Id));
        Assert.Equal("unsafePath", ex.Code);
        Assert.Throws<BackupException>(() => svc.Apply(zip, Id));
        Assert.Equal("[]", File.ReadAllText(Path.Combine(B, "points.json")));
        Assert.False(File.Exists(Path.Combine(_root, "evil.json")));
        Assert.False(Directory.Exists(Path.Combine(B, BackupService.BackupsDir)));
    }

    [Fact]
    public void ZipsThatAreNotBackupsAreRefused()
    {
        var svc = new BackupService(B);
        Assert.Equal("badZip", Assert.Throws<BackupException>(() => svc.Preview(Encoding.UTF8.GetBytes("not a zip"), Id)).Code);
        Assert.Equal("badManifest", Assert.Throws<BackupException>(() => svc.Preview(Zip(("points.json", "[]")), Id)).Code);
        Assert.Equal("badManifest", Assert.Throws<BackupException>(() => svc.Preview(Zip(("manifest.json", Manifest(format: "other"))), Id)).Code);
        Assert.Equal("badManifest", Assert.Throws<BackupException>(() => svc.Preview(Zip(("manifest.json", Manifest(version: 99))), Id)).Code);
    }

    [Fact]
    public void UnknownButSafeEntriesAreIgnoredWithAWarning()
    {
        var zip = Zip(("manifest.json", Manifest()), ("points.json", "[]"), ("plugins/x/x.dll", "bin"), ("notes.txt", "hi"));
        var svc = new BackupService(B);
        var preview = svc.Preview(zip, Id);
        Assert.Contains(preview.Warnings, w => w.Contains("2 file(s)"));
        svc.Apply(zip, Id);
        Assert.False(File.Exists(Path.Combine(B, "notes.txt")));
        Assert.False(Directory.Exists(Path.Combine(B, "plugins")));
        Assert.True(File.Exists(Path.Combine(B, "points.json")));
    }
}
