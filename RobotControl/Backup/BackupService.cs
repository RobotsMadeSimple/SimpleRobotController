using System.IO.Compression;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Backup;

/// <summary>A plugin installed when the backup was made (binaries are never in a backup).</summary>
public sealed record BackupPlugin(
    [property: JsonPropertyName("id")] string Id,
    [property: JsonPropertyName("name")] string? Name,
    [property: JsonPropertyName("version")] string? Version);

/// <summary><c>manifest.json</c> at the root of a controller backup zip.</summary>
public sealed class BackupManifest
{
    public const string FormatName = "rms-controller-backup";
    public const int CurrentFormatVersion = 1;

    [JsonPropertyName("format")]            public string Format { get; set; } = FormatName;
    [JsonPropertyName("formatVersion")]     public int FormatVersion { get; set; } = CurrentFormatVersion;
    [JsonPropertyName("controllerVersion")] public string ControllerVersion { get; set; } = "";
    [JsonPropertyName("robotType")]         public string RobotType { get; set; } = "";
    [JsonPropertyName("robotName")]         public string RobotName { get; set; } = "";
    [JsonPropertyName("serialNumber")]      public string SerialNumber { get; set; } = "";
    [JsonPropertyName("createdUnixMs")]     public long CreatedUnixMs { get; set; }
    [JsonPropertyName("createdUtc")]        public string CreatedUtc { get; set; } = "";
    [JsonPropertyName("plugins")]           public List<BackupPlugin> Plugins { get; set; } = new();
    [JsonPropertyName("files")]             public List<string> Files { get; set; } = new();
}

/// <summary>Who the backup is from — supplied by the controller, a fixed value in tests.</summary>
public sealed record BackupIdentity(string ControllerVersion, string RobotType, string RobotName, string SerialNumber, List<BackupPlugin> Plugins);

/// <summary>One category of the restore preview: how many items it has now and in the backup.</summary>
public sealed record BackupCategory(
    [property: JsonPropertyName("key")] string Key,
    [property: JsonPropertyName("label")] string Label,
    [property: JsonPropertyName("current")] int Current,
    [property: JsonPropertyName("backup")] int Backup);

public sealed class RestorePreview
{
    [JsonPropertyName("manifest")]   public BackupManifest Manifest { get; set; } = new();
    [JsonPropertyName("categories")] public List<BackupCategory> Categories { get; set; } = new();
    /// <summary>Plugins the backup lists that are not installed here (their configs restore, the plugin does not).</summary>
    [JsonPropertyName("missingPlugins")] public List<BackupPlugin> MissingPlugins { get; set; } = new();
    [JsonPropertyName("warnings")]   public List<string> Warnings { get; set; } = new();
    [JsonPropertyName("fileCount")]  public int FileCount { get; set; }
    [JsonPropertyName("totalBytes")] public long TotalBytes { get; set; }
}

/// <summary>Why a zip was refused: <c>badZip</c>, <c>badManifest</c>, <c>unsafePath</c>, <c>tooLarge</c>.</summary>
public sealed class BackupException(string code, string message) : Exception(message)
{
    public string Code { get; } = code;
}

/// <summary>
/// Whole-controller backup and restore of the data folder's user data (docs/websocket-api.md
/// "Backup &amp; restore"). Only an allowlist is ever read or written: programs, points, tools,
/// locals, grids, stacks, robot config, IO / aux / relay / camera / phone-device config, vision
/// programs, camera calibrations, cell packages, persistent variables, plugin configs, DXF files.
/// Caches, logs, IO history samples, the alarm log, the identity file and the plugin binaries
/// are never included — the manifest lists the installed plugins (id, version) instead.
/// </summary>
public sealed class BackupService
{
    /// <summary>Largest backup zip accepted, and the most its entries may expand to.</summary>
    public const long MaxZipBytes      = 500L * 1024 * 1024;
    public const long MaxExpandedBytes = 2L * 1024 * 1024 * 1024;
    public const int  MaxEntries       = 50_000;
    /// <summary>Pre-restore backups kept in &lt;data&gt;/backups.</summary>
    public const int  KeepPreRestore   = 5;
    public const string ManifestName   = "manifest.json";
    public const string BackupsDir     = "backups";

    /// <summary>Single files restored and backed up, relative to the data folder.</summary>
    public static readonly IReadOnlyList<string> AllowedFiles =
    [
        "robot-config.json",
        "points.json", "pointsHistory.json",
        "tools.json", "toolsHistory.json",
        "locals.json", "localsHistory.json",
        "grids.json", "stacks.json",
        "io_devices.json", "aux_config.json", "relay-config.json",
        "camera_config.json", "device_config.json",
        "persistent_vars.json",
    ];

    /// <summary>Folders restored and backed up whole.</summary>
    public static readonly IReadOnlyList<string> AllowedDirs =
    [
        "builtPrograms", "programImages", "vision_programs", "cameraCalibrations",
        "cells", "pluginConfigs", "dxf",
    ];

    private sealed record Category(string Key, string Label, string Path, CountKind Kind);
    private enum CountKind { JsonArray, Present, FilesTop, FilesAll, SubDirs, JsonArrayProp }

    private static readonly Category[] Categories =
    [
        new("programs",       "Programs",             "builtPrograms",      CountKind.FilesTop),
        new("programImages",  "Program images",       "programImages",      CountKind.FilesTop),
        new("points",         "Points",               "points.json",        CountKind.JsonArray),
        new("tools",          "Tools",                "tools.json",         CountKind.JsonArray),
        new("locals",         "Locals",               "locals.json",        CountKind.JsonArray),
        new("grids",          "Grids",                "grids.json",         CountKind.JsonArray),
        new("stacks",         "Stacks",               "stacks.json",        CountKind.JsonArray),
        new("visionPrograms", "Vision programs",      "vision_programs",    CountKind.FilesAll),
        new("calibrations",   "Camera calibrations",  "cameraCalibrations", CountKind.FilesAll),
        new("cells",          "Cell packages",        "cells",              CountKind.SubDirs),
        new("pluginConfigs",  "Plugin settings",      "pluginConfigs",      CountKind.FilesAll),
        new("dxf",            "DXF files",            "dxf",                CountKind.FilesAll),
        new("ioDevices",      "IO devices",           "io_devices.json",    CountKind.JsonArrayProp),
        new("robotConfig",    "Robot configuration",  "robot-config.json",  CountKind.Present),
        new("auxConfig",      "Aux axis settings",    "aux_config.json",    CountKind.Present),
        new("relayConfig",    "Relay settings",       "relay-config.json",  CountKind.Present),
        new("cameraConfig",   "Camera settings",      "camera_config.json", CountKind.Present),
        new("deviceConfig",   "Phone device settings","device_config.json", CountKind.Present),
        new("persistentVars", "Persistent variables", "persistent_vars.json", CountKind.Present),
    ];

    private readonly string _dataDir;

    public BackupService(string dataDir) => _dataDir = Path.GetFullPath(dataDir);

    // ── Create ───────────────────────────────────────────────────────────────

    /// <summary>The backup zip of the data folder's user data.</summary>
    public byte[] Create(BackupIdentity identity)
    {
        var files = EnumerateUserFiles().ToList();
        var manifest = new BackupManifest
        {
            ControllerVersion = identity.ControllerVersion,
            RobotType         = identity.RobotType,
            RobotName         = identity.RobotName,
            SerialNumber      = identity.SerialNumber,
            CreatedUnixMs     = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            CreatedUtc        = DateTimeOffset.UtcNow.ToString("O"),
            Plugins           = identity.Plugins,
            Files             = files,
        };

        using var ms = new MemoryStream();
        using (var zip = new ZipArchive(ms, ZipArchiveMode.Create, leaveOpen: true))
        {
            var m = zip.CreateEntry(ManifestName, CompressionLevel.Optimal);
            using (var s = m.Open())
                JsonSerializer.Serialize(s, manifest, new JsonSerializerOptions { WriteIndented = true });
            foreach (var rel in files)
            {
                var full = Path.Combine(_dataDir, rel);
                try
                {
                    // Read fully first so a file being rewritten never yields a torn entry.
                    var bytes = File.ReadAllBytes(full);
                    var e = zip.CreateEntry(rel, CompressionLevel.Optimal);
                    e.LastWriteTime = File.GetLastWriteTime(full);
                    using var s = e.Open();
                    s.Write(bytes);
                }
                catch (IOException ex) { Console.WriteLine($"[Backup] Skipped {rel}: {ex.Message}"); }
            }
        }
        return ms.ToArray();
    }

    /// <summary>Relative paths (forward slashes) of every allowlisted file that exists now.</summary>
    public IEnumerable<string> EnumerateUserFiles()
    {
        foreach (var f in AllowedFiles)
            if (File.Exists(Path.Combine(_dataDir, f))) yield return f;
        foreach (var d in AllowedDirs)
        {
            var dir = Path.Combine(_dataDir, d);
            if (!Directory.Exists(dir)) continue;
            foreach (var file in Directory.EnumerateFiles(dir, "*", SearchOption.AllDirectories).OrderBy(x => x, StringComparer.Ordinal))
            {
                var name = Path.GetFileName(file);
                if (name.EndsWith(".tmp", StringComparison.OrdinalIgnoreCase) || name.Contains(".corrupt-", StringComparison.Ordinal)) continue;
                yield return Path.GetRelativePath(_dataDir, file).Replace('\\', '/');
            }
        }
    }

    // ── Inspect ──────────────────────────────────────────────────────────────

    /// <summary>
    /// A validated entry path, or null when it is not one a restore writes. Throws
    /// <c>unsafePath</c> for anything that could leave the data folder.
    /// </summary>
    internal string? CheckEntryPath(string raw)
    {
        if (string.IsNullOrEmpty(raw)) throw new BackupException("unsafePath", "A zip entry has no name.");
        var name = raw.Replace('\\', '/');
        if (name.EndsWith('/')) return null; // a directory entry: nothing to write
        if (name.StartsWith('/') || name.Contains(':') || name.Contains('\0'))
            throw new BackupException("unsafePath", $"Zip entry '{raw}' has an absolute path.");
        var segments = name.Split('/');
        if (segments.Any(s => s.Length == 0 || s == "." || s == ".."))
            throw new BackupException("unsafePath", $"Zip entry '{raw}' leaves the data folder.");
        var full = Path.GetFullPath(Path.Combine(_dataDir, name));
        if (!full.StartsWith(_dataDir.TrimEnd(Path.DirectorySeparatorChar) + Path.DirectorySeparatorChar, StringComparison.OrdinalIgnoreCase))
            throw new BackupException("unsafePath", $"Zip entry '{raw}' leaves the data folder.");

        if (name == ManifestName) return null;
        if (segments.Length == 1)
            return AllowedFiles.Contains(name, StringComparer.Ordinal) ? name : null;
        return AllowedDirs.Contains(segments[0], StringComparer.Ordinal) ? name : null;
    }

    private sealed record Inspected(BackupManifest Manifest, List<(string Path, ZipArchiveEntry Entry)> Entries, List<string> Ignored, long TotalBytes);

    private Inspected Inspect(ZipArchive zip)
    {
        if (zip.Entries.Count > MaxEntries)
            throw new BackupException("tooLarge", $"The backup has more than {MaxEntries} files.");

        var manifestEntry = zip.GetEntry(ManifestName)
            ?? throw new BackupException("badManifest", "Not a controller backup: manifest.json is missing.");
        BackupManifest manifest;
        try
        {
            using var s = manifestEntry.Open();
            manifest = JsonSerializer.Deserialize<BackupManifest>(s, new JsonSerializerOptions { PropertyNameCaseInsensitive = true })
                ?? throw new BackupException("badManifest", "manifest.json is empty.");
        }
        catch (JsonException ex) { throw new BackupException("badManifest", $"manifest.json is not valid JSON: {ex.Message}"); }
        if (manifest.Format != BackupManifest.FormatName)
            throw new BackupException("badManifest", $"Not a controller backup (format '{manifest.Format}').");
        if (manifest.FormatVersion < 1 || manifest.FormatVersion > BackupManifest.CurrentFormatVersion)
            throw new BackupException("badManifest", $"Backup format version {manifest.FormatVersion} is not supported by this controller (supports {BackupManifest.CurrentFormatVersion}). Update the controller first.");

        var entries = new List<(string, ZipArchiveEntry)>();
        var ignored = new List<string>();
        var seen    = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        long total  = 0;
        foreach (var e in zip.Entries)
        {
            var path = CheckEntryPath(e.FullName); // throws on an unsafe path — the whole zip is refused
            if (path == null)
            {
                if (!e.FullName.EndsWith('/') && e.FullName != ManifestName) ignored.Add(e.FullName);
                continue;
            }
            if (!seen.Add(path)) throw new BackupException("badZip", $"Zip entry '{path}' appears twice.");
            total += e.Length;
            if (total > MaxExpandedBytes) throw new BackupException("tooLarge", "The backup expands to more than 2 GB.");
            entries.Add((path, e));
        }
        return new Inspected(manifest, entries, ignored, total);
    }

    private static ZipArchive OpenZip(byte[] zipBytes)
    {
        try { return new ZipArchive(new MemoryStream(zipBytes, writable: false), ZipArchiveMode.Read); }
        catch (InvalidDataException ex) { throw new BackupException("badZip", $"Not a zip file: {ex.Message}"); }
    }

    /// <summary>Validates <paramref name="zipBytes"/> and describes what a restore would replace.</summary>
    public RestorePreview Preview(byte[] zipBytes, BackupIdentity current)
    {
        using var zip = OpenZip(zipBytes);
        var ins = Inspect(zip);
        var byPath = ins.Entries.ToDictionary(x => x.Path, x => x.Entry, StringComparer.Ordinal);

        var preview = new RestorePreview
        {
            Manifest   = ins.Manifest,
            FileCount  = ins.Entries.Count,
            TotalBytes = ins.TotalBytes,
        };
        foreach (var c in Categories)
        {
            int now = CountOnDisk(c);
            int bak = CountInZip(c, byPath);
            if (now > 0 || bak > 0) preview.Categories.Add(new BackupCategory(c.Key, c.Label, now, bak));
        }

        var installed = current.Plugins.ToDictionary(p => p.Id, StringComparer.OrdinalIgnoreCase);
        foreach (var p in ins.Manifest.Plugins)
        {
            if (!installed.TryGetValue(p.Id, out var here)) preview.MissingPlugins.Add(p);
            else if (!string.IsNullOrEmpty(p.Version) && !string.Equals(p.Version, here.Version, StringComparison.OrdinalIgnoreCase))
                preview.Warnings.Add($"Plugin '{p.Name ?? p.Id}' is version {here.Version ?? "?"} here; the backup was made with {p.Version}.");
        }
        if (preview.MissingPlugins.Count > 0)
            preview.Warnings.Add($"{preview.MissingPlugins.Count} plugin(s) in the backup are not installed here. Their settings are restored; install the plugins to use them.");
        if (!string.IsNullOrEmpty(ins.Manifest.SerialNumber) && !string.IsNullOrEmpty(current.SerialNumber)
            && !string.Equals(ins.Manifest.SerialNumber, current.SerialNumber, StringComparison.OrdinalIgnoreCase))
            preview.Warnings.Add($"This backup was made on another controller (serial {ins.Manifest.SerialNumber}). Check homing offsets and motor setup after restoring.");
        if (!string.IsNullOrEmpty(ins.Manifest.RobotType) && !string.IsNullOrEmpty(current.RobotType)
            && !string.Equals(ins.Manifest.RobotType, current.RobotType, StringComparison.OrdinalIgnoreCase))
            preview.Warnings.Add($"The backup is from a {ins.Manifest.RobotType} robot; this one is {current.RobotType}.");
        if (ins.Ignored.Count > 0)
            preview.Warnings.Add($"{ins.Ignored.Count} file(s) in the zip are not controller data and will be ignored.");
        return preview;
    }

    // ── Apply ────────────────────────────────────────────────────────────────

    public sealed record ApplyResult(string PreRestoreBackup, int FilesWritten);

    /// <summary>
    /// Replaces the allowlisted data with the backup's. Writes a pre-restore backup first
    /// (<c>backups/pre-restore-*.zip</c>), extracts into a staging folder, then swaps each
    /// allowlisted file and folder in (an allowlisted item the backup does not have is removed, so
    /// the data matches the backup). Rolls back on failure. The caller restarts the controller so
    /// every repository reloads from disk.
    /// </summary>
    public ApplyResult Apply(byte[] zipBytes, BackupIdentity current)
    {
        using var zip = OpenZip(zipBytes);
        var ins = Inspect(zip);

        // 1. Safety net: the data as it is now.
        var backupsDir = Path.Combine(_dataDir, BackupsDir);
        Directory.CreateDirectory(backupsDir);
        var stamp   = DateTime.Now.ToString("yyyyMMdd-HHmmss-fff");
        var preName = $"pre-restore-{stamp}.zip";
        File.WriteAllBytes(Path.Combine(backupsDir, preName), Create(current));
        PrunePreRestore(backupsDir);

        // 2. Extract to staging (inside the data folder, so the swap is a same-volume move).
        var staging = Path.Combine(_dataDir, ".restore-staging-" + Guid.NewGuid().ToString("N"));
        var stash   = Path.Combine(_dataDir, ".restore-previous-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(staging);
        try
        {
            foreach (var (path, entry) in ins.Entries)
            {
                var dest = Path.Combine(staging, path);
                Directory.CreateDirectory(Path.GetDirectoryName(dest)!);
                using var src = entry.Open();
                using var dst = File.Create(dest);
                src.CopyTo(dst);
            }

            // 3. Swap: move the current item aside, move the staged one in.
            var moved = new List<string>();
            try
            {
                Directory.CreateDirectory(stash);
                foreach (var rel in AllowedFiles.Concat(AllowedDirs))
                {
                    var target = Path.Combine(_dataDir, rel);
                    var staged = Path.Combine(staging, rel);
                    var aside  = Path.Combine(stash, rel);
                    if (File.Exists(target)) File.Move(target, aside);
                    else if (Directory.Exists(target)) Directory.Move(target, aside);
                    moved.Add(rel);
                    if (File.Exists(staged)) File.Move(staged, target);
                    else if (Directory.Exists(staged)) Directory.Move(staged, target);
                }
            }
            catch
            {
                // Put back everything that was moved aside.
                foreach (var rel in moved)
                {
                    var target = Path.Combine(_dataDir, rel);
                    var aside  = Path.Combine(stash, rel);
                    try
                    {
                        if (File.Exists(target)) File.Delete(target);
                        else if (Directory.Exists(target)) Directory.Delete(target, recursive: true);
                        if (File.Exists(aside)) File.Move(aside, target);
                        else if (Directory.Exists(aside)) Directory.Move(aside, target);
                    }
                    catch (Exception ex) { Console.WriteLine($"[Backup] Rollback of {rel} failed: {ex.Message}"); }
                }
                throw;
            }
            return new ApplyResult($"{BackupsDir}/{preName}", ins.Entries.Count);
        }
        finally
        {
            TryDelete(staging);
            TryDelete(stash);
        }
    }

    private static void PrunePreRestore(string dir)
    {
        foreach (var old in new DirectoryInfo(dir).GetFiles("pre-restore-*.zip")
                     .OrderByDescending(f => f.Name, StringComparer.Ordinal).Skip(KeepPreRestore))
            try { old.Delete(); } catch { }
    }

    private static void TryDelete(string dir)
    {
        try { if (Directory.Exists(dir)) Directory.Delete(dir, recursive: true); }
        catch (Exception ex) { Console.WriteLine($"[Backup] Could not remove {dir}: {ex.Message}"); }
    }

    // ── Counting for the preview ─────────────────────────────────────────────

    private int CountOnDisk(Category c)
    {
        var full = Path.Combine(_dataDir, c.Path);
        try
        {
            return c.Kind switch
            {
                CountKind.FilesTop      => Directory.Exists(full) ? Directory.EnumerateFiles(full).Count(f => !f.EndsWith(".tmp", StringComparison.OrdinalIgnoreCase)) : 0,
                CountKind.FilesAll      => Directory.Exists(full) ? Directory.EnumerateFiles(full, "*", SearchOption.AllDirectories).Count() : 0,
                CountKind.SubDirs       => Directory.Exists(full) ? Directory.EnumerateDirectories(full).Count() : 0,
                CountKind.Present       => File.Exists(full) ? 1 : 0,
                CountKind.JsonArray     => File.Exists(full) ? CountJson(File.ReadAllText(full), null) : 0,
                CountKind.JsonArrayProp => File.Exists(full) ? CountJson(File.ReadAllText(full), "ioDevices") : 0,
                _ => 0,
            };
        }
        catch { return 0; }
    }

    private static int CountInZip(Category c, Dictionary<string, ZipArchiveEntry> byPath)
    {
        string prefix = c.Path + "/";
        switch (c.Kind)
        {
            case CountKind.FilesTop: return byPath.Keys.Count(k => k.StartsWith(prefix, StringComparison.Ordinal) && k.IndexOf('/', prefix.Length) < 0);
            case CountKind.FilesAll: return byPath.Keys.Count(k => k.StartsWith(prefix, StringComparison.Ordinal));
            case CountKind.SubDirs:
                return byPath.Keys.Where(k => k.StartsWith(prefix, StringComparison.Ordinal) && k.IndexOf('/', prefix.Length) > 0)
                    .Select(k => k[prefix.Length..k.IndexOf('/', prefix.Length)]).Distinct().Count();
            case CountKind.Present: return byPath.ContainsKey(c.Path) ? 1 : 0;
            default:
                if (!byPath.TryGetValue(c.Path, out var e)) return 0;
                try
                {
                    using var r = new StreamReader(e.Open());
                    return CountJson(r.ReadToEnd(), c.Kind == CountKind.JsonArrayProp ? "ioDevices" : null);
                }
                catch { return 0; }
        }
    }

    private static int CountJson(string json, string? arrayProp)
    {
        try
        {
            var node = JsonNode.Parse(json);
            if (arrayProp != null) node = node?[arrayProp];
            return node is JsonArray a ? a.Count : 0;
        }
        catch { return 0; }
    }
}
