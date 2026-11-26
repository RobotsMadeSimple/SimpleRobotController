using System.IO.Compression;
using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.RegularExpressions;

namespace Controller.RobotControl.Cells;

/// <summary>One stored package as <c>GET /cells</c> / <c>GetCells</c> list it.</summary>
public sealed record CellSummary(string Id, string Name, long UpdatedUnixMs, bool Active, int Tools, int Fixtures);

/// <summary>Outcome of <see cref="CellRepository.Install"/>: on failure <see cref="Error"/> is a code
/// (<c>tooLarge</c>, <c>badZip</c>, <c>badPackage</c>, <c>idExists</c>) and <see cref="Message"/> says why.</summary>
public sealed record CellInstallResult(bool Ok, string? Id, CellSummary? Cell, string? Error, string? Message)
{
    public static CellInstallResult Fail(string error, string message) => new(false, null, null, error, message);
}

/// <summary>A package that passed <see cref="CellPackageValidator.Validate"/>: its cell.json and the
/// zip entries to extract (normalized relative path → entry).</summary>
public sealed record ValidatedCellPackage(string Id, JsonObject Cell, IReadOnlyList<(string Path, ZipArchiveEntry Entry)> Files);

/// <summary>
/// Checks a cell package zip against docs/cell-package.md: <c>cell.json</c> at the root, format
/// <c>rms-cell</c> version 1, a valid id, every referenced mesh present, and no entry that escapes
/// the package folder (zip-slip).
/// </summary>
public static class CellPackageValidator
{
    public const string CellFileName = "cell.json";
    public const string Format = "rms-cell";
    public const int Version = 1;
    /// <summary>Guard against zip bombs: the total uncompressed size a package may unpack to.</summary>
    public const long MaxExtractedBytes = 1024L * 1024 * 1024;

    private static readonly Regex IdPattern = new("^[a-z0-9][a-z0-9-_]{0,63}$", RegexOptions.CultureInvariant);

    /// <summary>Ids the HTTP routes reserve (<c>GET /cells/active</c>).</summary>
    private static readonly HashSet<string> ReservedIds = new(StringComparer.Ordinal) { "active" };

    public static bool IsValidId(string? id) => id is not null && IdPattern.IsMatch(id) && !ReservedIds.Contains(id);

    /// <summary>
    /// A package-relative path is usable when it is forward-slash, relative, has no empty,
    /// <c>.</c> or <c>..</c> segments and no drive/colon. Used for both zip entries and the
    /// <c>mesh</c> paths cell.json references.
    /// </summary>
    public static bool IsSafeRelativePath(string? path)
    {
        if (string.IsNullOrEmpty(path)) return false;
        if (path.Contains('\\') || path.Contains(':') || path.StartsWith('/')) return false;
        foreach (var seg in path.Split('/'))
            if (seg.Length == 0 || seg == "." || seg == "..") return false;
        return true;
    }

    /// <summary>Validates an opened archive; <paramref name="error"/>/<paramref name="message"/> are set on failure.</summary>
    public static ValidatedCellPackage? Validate(ZipArchive archive, out string? error, out string? message)
    {
        error = "badPackage";
        var files = new List<(string, ZipArchiveEntry)>();
        var names = new HashSet<string>(StringComparer.Ordinal);
        long extracted = 0;
        foreach (var entry in archive.Entries)
        {
            bool isFolder = entry.FullName.EndsWith('/') || entry.FullName.EndsWith('\\');
            string rel = entry.FullName.Replace('\\', '/').TrimEnd('/');
            if (rel.Length == 0) continue;
            if (!IsSafeRelativePath(rel))
            {
                message = $"Zip entry '{entry.FullName}' escapes the package folder";
                return null;
            }
            if (isFolder) continue;
            if (!names.Add(rel)) { message = $"Zip entry '{rel}' appears twice"; return null; }
            files.Add((rel, entry));
            extracted += entry.Length;
            if (extracted > MaxExtractedBytes) { message = "Package unpacks to more than 1 GB"; return null; }
        }

        var cellEntry = files.FirstOrDefault(f => f.Item1 == CellFileName).Item2;
        if (cellEntry is null) { message = "cell.json not found at the zip root"; return null; }

        JsonObject cell;
        try
        {
            using var s = cellEntry.Open();
            cell = JsonNode.Parse(s, documentOptions: new JsonDocumentOptions { CommentHandling = JsonCommentHandling.Skip, AllowTrailingCommas = true })
                   as JsonObject ?? throw new JsonException("cell.json must be a JSON object");
        }
        catch (Exception ex) when (ex is JsonException or InvalidDataException)
        {
            message = "cell.json is not valid JSON: " + ex.Message;
            return null;
        }

        if (Str(cell, "format") != Format) { message = $"cell.json format must be \"{Format}\""; return null; }
        if (cell["version"] is not JsonValue v || !v.TryGetValue(out double ver) || ver != Version)
        {
            message = $"cell.json version must be {Version}";
            return null;
        }
        string? id = Str(cell, "id");
        if (!IsValidId(id))
        {
            message = $"cell.json id '{id}' is invalid (lowercase letters, digits, '-' and '_', starting with a letter or digit, at most 64 characters; 'active' is reserved)";
            return null;
        }
        if (cell["name"] is not null and not JsonValue) { message = "cell.json name must be a string"; return null; }
        if (cell["robot"] is not null and not JsonObject) { message = "cell.json robot must be an object or null"; return null; }
        foreach (var list in new[] { "tools", "fixtures" })
            if (cell[list] is not null and not JsonArray) { message = $"cell.json {list} must be an array"; return null; }

        foreach (var mesh in MeshPaths(cell))
        {
            if (!IsSafeRelativePath(mesh)) { message = $"Mesh path '{mesh}' must be relative, forward-slash and without '..'"; return null; }
            if (!names.Contains(mesh)) { message = $"Mesh '{mesh}' is referenced by cell.json but not in the zip"; return null; }
        }

        error = null; message = null;
        return new ValidatedCellPackage(id!, cell, files);
    }

    /// <summary>Every string <c>"mesh"</c> property anywhere in cell.json (robot links, tools, moving parts, fixtures).</summary>
    public static IEnumerable<string> MeshPaths(JsonNode? node)
    {
        switch (node)
        {
            case JsonObject o:
                foreach (var (k, child) in o)
                {
                    if (k == "mesh" && child is JsonValue mv && mv.TryGetValue(out string? m)) yield return m;
                    else foreach (var p in MeshPaths(child)) yield return p;
                }
                break;
            case JsonArray a:
                foreach (var child in a)
                    foreach (var p in MeshPaths(child)) yield return p;
                break;
        }
    }

    internal static string? Str(JsonObject o, string name) =>
        o[name] is JsonValue v && v.TryGetValue(out string? s) ? s : null;
}

/// <summary>
/// Cell packages stored under <c>&lt;data&gt;/cells/&lt;id&gt;/</c> (docs/cell-package.md §4). An upload is
/// validated, extracted to a staging folder and then swapped in, so a bad zip never damages a stored
/// package. The active id itself is the robot setting <c>activeCellId</c>, reached through the
/// getter/setter passed in. <see cref="LastUpdatedUnixMs"/> backs GetStatus <c>lastCellUpdate</c>.
/// </summary>
public sealed class CellRepository
{
    public const long MaxZipBytes = 200L * 1024 * 1024;

    private readonly object _lock = new();
    private readonly Func<string?> _getActive;
    private readonly Action<string?> _setActive;
    private long _lastUpdated = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

    public CellRepository(string dataDir, Func<string?> getActive, Action<string?> setActive)
    {
        CellsDir   = Path.Combine(Path.GetFullPath(dataDir), "cells");
        _getActive = getActive;
        _setActive = setActive;
    }

    public string CellsDir { get; }

    /// <summary>Bumped on any upload, delete or active change, and by <see cref="Touch"/> (adjustable setting / robot type change).</summary>
    public long LastUpdatedUnixMs => Interlocked.Read(ref _lastUpdated);

    /// <summary>Marks the resolved cell as changed (strictly increasing, so two changes in one ms still differ).</summary>
    public void Touch()
    {
        long now = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        long prev;
        do { prev = Interlocked.Read(ref _lastUpdated); }
        while (Interlocked.CompareExchange(ref _lastUpdated, Math.Max(now, prev + 1), prev) != prev);
    }

    /// <summary>The active package id, or null when none is set or it no longer exists.</summary>
    public string? ActiveId
    {
        get
        {
            var id = _getActive();
            return string.IsNullOrEmpty(id) || !Exists(id) ? null : id;
        }
    }

    public bool Exists(string id) => CellPackageValidator.IsValidId(id) && File.Exists(CellFile(id));

    private string Folder(string id) => Path.Combine(CellsDir, id);
    private string CellFile(string id) => Path.Combine(Folder(id), CellPackageValidator.CellFileName);

    // ── install ──────────────────────────────────────────────────────────────

    public CellInstallResult Install(byte[] zipBytes, bool replace)
    {
        if (zipBytes.LongLength > MaxZipBytes) return CellInstallResult.Fail("tooLarge", "Cell package must be at most 200 MB");

        ZipArchive archive;
        try { archive = new ZipArchive(new MemoryStream(zipBytes, writable: false), ZipArchiveMode.Read); }
        catch (Exception ex) when (ex is InvalidDataException or ArgumentException or IOException)
        {
            return CellInstallResult.Fail("badZip", "Not a zip file: " + ex.Message);
        }

        using (archive)
        {
            var pkg = CellPackageValidator.Validate(archive, out var error, out var message);
            if (pkg is null) return CellInstallResult.Fail(error ?? "badPackage", message ?? "Invalid cell package");

            lock (_lock)
            {
                string target = Folder(pkg.Id);
                if (Directory.Exists(target) && !replace)
                    return CellInstallResult.Fail("idExists", $"A cell package with id '{pkg.Id}' already exists (upload with ?replace=true to overwrite)");

                Directory.CreateDirectory(CellsDir);
                string staging = Path.Combine(CellsDir, $".staging-{Guid.NewGuid():N}");
                string? old = null;
                try
                {
                    Directory.CreateDirectory(staging);
                    string stagingFull = Path.GetFullPath(staging) + Path.DirectorySeparatorChar;
                    foreach (var (rel, entry) in pkg.Files)
                    {
                        string dest = Path.GetFullPath(Path.Combine(staging, rel.Replace('/', Path.DirectorySeparatorChar)));
                        if (!dest.StartsWith(stagingFull, StringComparison.Ordinal))
                            return CellInstallResult.Fail("badPackage", $"Zip entry '{entry.FullName}' escapes the package folder");
                        Directory.CreateDirectory(Path.GetDirectoryName(dest)!);
                        entry.ExtractToFile(dest, overwrite: true);
                    }
                    // updatedUnixMs reads cell.json's write time; extraction stamps the zip entry's time.
                    File.SetLastWriteTimeUtc(Path.Combine(staging, CellPackageValidator.CellFileName), DateTime.UtcNow);

                    // Swap: move the old folder aside, move the new one in, then drop the old one.
                    if (Directory.Exists(target))
                    {
                        old = Path.Combine(CellsDir, $".old-{Guid.NewGuid():N}");
                        Directory.Move(target, old);
                    }
                    try { Directory.Move(staging, target); }
                    catch
                    {
                        if (old != null && !Directory.Exists(target)) { Directory.Move(old, target); old = null; }
                        throw;
                    }
                }
                catch (Exception ex) when (ex is IOException or UnauthorizedAccessException or InvalidDataException)
                {
                    return CellInstallResult.Fail("badPackage", "Could not store the package: " + ex.Message);
                }
                finally
                {
                    TryDelete(staging);
                    if (old != null) TryDelete(old);
                }

                Touch();
                Console.WriteLine($"[Cells] Stored '{pkg.Id}' ({pkg.Files.Count} file(s))");
                return new CellInstallResult(true, pkg.Id, Summary(pkg.Id), null, null);
            }
        }
    }

    // ── read ─────────────────────────────────────────────────────────────────

    public List<CellSummary> List()
    {
        if (!Directory.Exists(CellsDir)) return new();
        var result = new List<CellSummary>();
        foreach (var dir in Directory.GetDirectories(CellsDir).OrderBy(d => d, StringComparer.Ordinal))
        {
            string id = Path.GetFileName(dir);
            if (!Exists(id)) continue;  // skips .staging-/.old- folders and anything without cell.json
            if (Summary(id) is { } s) result.Add(s);
        }
        return result;
    }

    /// <summary>The stored cell.json as uploaded, or null.</summary>
    public JsonObject? Get(string id)
    {
        if (!Exists(id)) return null;
        try { return JsonNode.Parse(File.ReadAllText(CellFile(id)), documentOptions: new JsonDocumentOptions { CommentHandling = JsonCommentHandling.Skip, AllowTrailingCommas = true }) as JsonObject; }
        catch (Exception ex) when (ex is JsonException or IOException) { return null; }
    }

    public CellSummary? Summary(string id)
    {
        var cell = Get(id);
        if (cell is null) return null;
        long updated = new DateTimeOffset(File.GetLastWriteTimeUtc(CellFile(id))).ToUnixTimeMilliseconds();
        return new CellSummary(id, CellPackageValidator.Str(cell, "name") ?? id, updated,
            string.Equals(ActiveId, id, StringComparison.Ordinal),
            (cell["tools"] as JsonArray)?.Count ?? 0, (cell["fixtures"] as JsonArray)?.Count ?? 0);
    }

    /// <summary>Full path of a file inside a stored package, or null when the id/path is invalid, escapes the folder, or is missing.</summary>
    public string? FilePath(string id, string? relPath)
    {
        if (!Exists(id) || relPath is null) return null;
        relPath = relPath.Replace('\\', '/');
        if (!CellPackageValidator.IsSafeRelativePath(relPath)) return null;
        string root = Path.GetFullPath(Folder(id)) + Path.DirectorySeparatorChar;
        string full = Path.GetFullPath(Path.Combine(root, relPath.Replace('/', Path.DirectorySeparatorChar)));
        return full.StartsWith(root, StringComparison.Ordinal) && File.Exists(full) ? full : null;
    }

    // ── write ────────────────────────────────────────────────────────────────

    /// <summary>Removes a package (and clears the active id if it was active). False when unknown.</summary>
    public bool Delete(string id)
    {
        lock (_lock)
        {
            if (!Exists(id)) return false;
            string old = Path.Combine(CellsDir, $".old-{Guid.NewGuid():N}");
            Directory.Move(Folder(id), old);  // vanish atomically, then clean up
            TryDelete(old);
            if (string.Equals(_getActive(), id, StringComparison.Ordinal)) _setActive(null);
            Touch();
            Console.WriteLine($"[Cells] Deleted '{id}'");
            return true;
        }
    }

    /// <summary>Sets the active package (null/"" clears). False when the id is not stored.</summary>
    public bool SetActive(string? id)
    {
        lock (_lock)
        {
            if (string.IsNullOrEmpty(id)) id = null;
            else if (!Exists(id)) return false;
            if (!string.Equals(_getActive() is { Length: > 0 } cur ? cur : null, id, StringComparison.Ordinal))
            {
                _setActive(id);
                Touch();
            }
            return true;
        }
    }

    private static void TryDelete(string dir)
    {
        try { if (Directory.Exists(dir)) Directory.Delete(dir, recursive: true); }
        catch (Exception ex) when (ex is IOException or UnauthorizedAccessException)
        {
            Console.WriteLine($"[Cells] Could not remove {dir}: {ex.Message}");
        }
    }
}
