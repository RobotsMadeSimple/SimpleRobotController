using System.Text.Json;

namespace Controller.RobotControl.Persistence;

/// <summary>
/// Persists built programs to disk as individual JSON files under builtPrograms/.
/// Program images are stored as separate JPEG files under programImages/.
/// Thread-safe via lock.
/// </summary>
public class BuiltProgramRepository
{
    private readonly string _dir;        // builtPrograms/
    private readonly string _imageDir;   // programImages/
    private readonly string _legacyFile; // builtPrograms.json — read once for migration, then renamed
    private readonly object _lock = new();
    private readonly JsonSerializerOptions _opts = new()
    {
        WriteIndented               = true,
        Converters                  = { new System.Text.Json.Serialization.JsonStringEnumConverter() },
        PropertyNameCaseInsensitive = true,
    };

    private Dictionary<string, BuiltProgram> _programs = new();

    public long LastUpdatedUnixMs { get; private set; }

    public BuiltProgramRepository(string baseDir = ".")
    {
        _dir        = Path.Combine(baseDir, "builtPrograms");
        _imageDir   = Path.Combine(baseDir, "programImages");
        _legacyFile = Path.Combine(baseDir, "builtPrograms.json");
        Directory.CreateDirectory(_dir);
        MigrateIfNeeded();
        Load();
    }

    // ── Migration from single-file format ────────────────────────────────────

    private void MigrateIfNeeded()
    {
        if (!File.Exists(_legacyFile)) return;
        try
        {
            var json = File.ReadAllText(_legacyFile);
            var raw  = JsonSerializer.Deserialize<List<JsonElement>>(json, _opts);
            if (raw != null)
            {
                foreach (var elem in raw)
                {
                    BuiltProgram? p = null;
                    try { p = elem.Deserialize<BuiltProgram>(_opts); }
                    catch { /* skip programs with unrecognized step types */ }
                    if (p == null || string.IsNullOrWhiteSpace(p.Name)) continue;

                    if (string.IsNullOrEmpty(p.Id)) p.Id = Guid.NewGuid().ToString();

                    var dest = ProgramPath(p.Name);
                    if (!File.Exists(dest))
                        AtomicFile.WriteAllText(dest, JsonSerializer.Serialize(p, _opts));
                }
            }
            // Rename so migration doesn't re-run on next boot
            File.Move(_legacyFile, _legacyFile + ".migrated", overwrite: true);
        }
        catch { /* best-effort — if migration fails, Load() still reads any files already written */ }
    }

    // ── Persistence ───────────────────────────────────────────────────────────

    // All valid StepType names — used by the patcher to detect unknown values.
    private static readonly HashSet<string> _knownStepTypes =
        Enum.GetNames<StepType>().ToHashSet(StringComparer.OrdinalIgnoreCase);

    private void Load()
    {
        foreach (var file in Directory.GetFiles(_dir, "*.json"))
        {
            var p = LoadFile(file);
            if (p != null && !string.IsNullOrWhiteSpace(p.Name))
                _programs[p.Name] = p;
        }
    }

    private BuiltProgram? LoadFile(string file)
    {
        string json;
        try { json = File.ReadAllText(file); }
        catch { return null; }

        // Fast path — normal deserialization
        try
        {
            var p = JsonSerializer.Deserialize<BuiltProgram>(json, _opts);
            if (p != null) return p;
        }
        catch { }

        // Slow path — patch unrecognized step types so the rest of the program loads
        try
        {
            var patched = PatchUnknownStepTypes(json);
            return JsonSerializer.Deserialize<BuiltProgram>(patched, _opts);
        }
        catch { return null; }
    }

    /// <summary>
    /// Walks the raw JSON and replaces any step object whose "type" value is not a
    /// known StepType with {"type":"Unknown","unknownStepType":"&lt;original&gt;",...}.
    /// This lets programs with renamed or removed step types still load and be edited.
    /// </summary>
    private static string PatchUnknownStepTypes(string json)
    {
        using var doc = JsonDocument.Parse(json);
        using var ms  = new System.IO.MemoryStream();
        using var w   = new Utf8JsonWriter(ms);
        WritePatched(doc.RootElement, w);
        w.Flush();
        return System.Text.Encoding.UTF8.GetString(ms.ToArray());
    }

    private static void WritePatched(JsonElement elem, Utf8JsonWriter w)
    {
        switch (elem.ValueKind)
        {
            case JsonValueKind.Object:
                w.WriteStartObject();
                // Detect a step object: has a "type" property with an unrecognized value
                string? unknownType = null;
                if (elem.TryGetProperty("type", out var typeProp) && typeProp.ValueKind == JsonValueKind.String)
                {
                    var typeStr = typeProp.GetString() ?? "";
                    if (!_knownStepTypes.Contains(typeStr))
                        unknownType = typeStr;
                }
                foreach (var prop in elem.EnumerateObject())
                {
                    if (unknownType != null && prop.Name == "type")
                    {
                        w.WriteString("type", "Unknown");
                        w.WriteString("unknownStepType", unknownType);
                    }
                    else
                    {
                        w.WritePropertyName(prop.Name);
                        WritePatched(prop.Value, w);
                    }
                }
                w.WriteEndObject();
                break;

            case JsonValueKind.Array:
                w.WriteStartArray();
                foreach (var item in elem.EnumerateArray())
                    WritePatched(item, w);
                w.WriteEndArray();
                break;

            case JsonValueKind.String:  w.WriteStringValue(elem.GetString()); break;
            case JsonValueKind.Number:  w.WriteRawValue(elem.GetRawText());   break;
            case JsonValueKind.True:    w.WriteBooleanValue(true);             break;
            case JsonValueKind.False:   w.WriteBooleanValue(false);            break;
            case JsonValueKind.Null:    w.WriteNullValue();                    break;
        }
    }

    private void WriteFile(BuiltProgram program)
    {
        AtomicFile.WriteAllText(ProgramPath(program.Name), JsonSerializer.Serialize(program, _opts));
        LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }

    private static string SafeName(string name) =>
        string.Concat(name.Split(Path.GetInvalidFileNameChars()));

    private string ProgramPath(string name) => Path.Combine(_dir, SafeName(name) + ".json");

    // ── Revisions ─────────────────────────────────────────────────────────────

    private const int MaxRevisionsPerProgram = 30;

    private string RevisionDir(string name) => Path.Combine(_dir, ".revisions", SafeName(name));

    /// <summary>
    /// If a program is already saved at <paramref name="program"/>'s path and its on-disk
    /// JSON differs from what would be written now, archives the CURRENT (pre-overwrite)
    /// content as a new revision, then trims the folder down to the newest 30.
    /// </summary>
    private void ArchiveRevisionIfChanged(BuiltProgram program)
    {
        var path = ProgramPath(program.Name);
        if (!File.Exists(path)) return;

        string oldJson;
        try { oldJson = File.ReadAllText(path); }
        catch { return; }

        if (!ContentDiffers(oldJson, program)) return;

        var revDir = RevisionDir(program.Name);
        Directory.CreateDirectory(revDir);

        var stamp = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        var dest = Path.Combine(revDir, stamp + ".json");
        while (File.Exists(dest))
        {
            stamp++;
            dest = Path.Combine(revDir, stamp + ".json");
        }
        AtomicFile.WriteAllText(dest, oldJson);

        TrimRevisions(revDir);
    }

    /// <summary>
    /// True unless the on-disk content and <paramref name="newProgram"/> are equal in every
    /// field but <see cref="BuiltProgram.LastUpdatedUnixMs"/> — <see cref="Save"/> always
    /// bumps that stamp, so comparing it too would archive a revision on every re-save even
    /// when nothing the user edited actually changed.
    /// </summary>
    private bool ContentDiffers(string oldJson, BuiltProgram newProgram)
    {
        BuiltProgram? oldProgram;
        try { oldProgram = JsonSerializer.Deserialize<BuiltProgram>(oldJson, _opts); }
        catch { return true; } // unreadable on-disk content — treat as changed, best effort

        if (oldProgram == null) return true;

        var actualStamp = newProgram.LastUpdatedUnixMs;
        newProgram.LastUpdatedUnixMs = oldProgram.LastUpdatedUnixMs;
        try
        {
            var oldCanonical = JsonSerializer.Serialize(oldProgram, _opts);
            var newCanonical = JsonSerializer.Serialize(newProgram, _opts);
            return oldCanonical != newCanonical;
        }
        finally
        {
            newProgram.LastUpdatedUnixMs = actualStamp;
        }
    }

    private static void TrimRevisions(string revDir)
    {
        var files = Directory.GetFiles(revDir, "*.json");
        if (files.Length <= MaxRevisionsPerProgram) return;

        var oldest = files
            .Select(f => (file: f, id: ParseRevisionId(f)))
            .OrderByDescending(x => x.id)
            .Skip(MaxRevisionsPerProgram)
            .Select(x => x.file);
        foreach (var file in oldest)
        {
            try { File.Delete(file); } catch { /* best effort */ }
        }
    }

    private static long ParseRevisionId(string file) =>
        long.TryParse(Path.GetFileNameWithoutExtension(file), out var v) ? v : 0;

    /// <summary>Moves a program's revision folder when it is renamed (same id, new name).</summary>
    private void MoveRevisionFolder(string oldName, string newName)
    {
        var oldDir = RevisionDir(oldName);
        var newDir = RevisionDir(newName);
        if (!Directory.Exists(oldDir) || string.Equals(oldDir, newDir, StringComparison.OrdinalIgnoreCase))
            return;

        Directory.CreateDirectory(Path.GetDirectoryName(newDir)!);
        if (!Directory.Exists(newDir))
        {
            Directory.Move(oldDir, newDir);
        }
        else
        {
            // Destination already has revisions (e.g. renamed onto a name that once existed) —
            // merge file-by-file rather than losing either set.
            foreach (var f in Directory.GetFiles(oldDir))
                File.Move(f, Path.Combine(newDir, Path.GetFileName(f)), overwrite: true);
            Directory.Delete(oldDir, recursive: true);
        }
    }

    /// <summary>Counts steps recursively (loop bodies, if/else-if/else branches) without following CallRoutine.</summary>
    private static int CountSteps(IEnumerable<ProgramStep>? steps)
    {
        if (steps == null) return 0;
        var count = 0;
        foreach (var s in steps)
        {
            count++;
            count += CountSteps(s.LoopSteps);
            count += CountSteps(s.IfSteps);
            count += CountSteps(s.ElseSteps);
            if (s.ElseIfBranches != null)
                foreach (var branch in s.ElseIfBranches)
                    count += CountSteps(branch.Steps);
        }
        return count;
    }

    // ── Public API ────────────────────────────────────────────────────────────

    public void Save(BuiltProgram program)
    {
        lock (_lock)
        {
            if (string.IsNullOrEmpty(program.Id))
                program.Id = Guid.NewGuid().ToString();
            program.LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

            // If a program with the same ID already exists under a different name, remove the old file
            var previous = _programs.Values.FirstOrDefault(p =>
                p.Id == program.Id &&
                !string.Equals(p.Name, program.Name, StringComparison.OrdinalIgnoreCase));
            if (previous != null)
            {
                var oldFile = ProgramPath(previous.Name);
                if (File.Exists(oldFile)) File.Delete(oldFile);
                _programs.Remove(previous.Name);
                MoveRevisionFolder(previous.Name, program.Name);
            }

            ArchiveRevisionIfChanged(program);

            _programs[program.Name] = program;
            WriteFile(program);
        }
    }

    public bool Delete(string name)
    {
        lock (_lock)
        {
            if (!_programs.Remove(name)) return false;
            var path = ProgramPath(name);
            if (File.Exists(path)) File.Delete(path);
            LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            var imgPath = ImagePath(name);
            if (File.Exists(imgPath)) File.Delete(imgPath);
            var revDir = RevisionDir(name);
            if (Directory.Exists(revDir)) { try { Directory.Delete(revDir, recursive: true); } catch { /* best effort */ } }
            return true;
        }
    }

    /// <summary>Newest-first summary of the saved revisions for a program. Empty if none/unknown.</summary>
    public List<ProgramRevisionInfo> ListRevisions(string name)
    {
        lock (_lock)
        {
            var revDir = RevisionDir(name);
            if (!Directory.Exists(revDir)) return new List<ProgramRevisionInfo>();

            var result = new List<ProgramRevisionInfo>();
            foreach (var file in Directory.GetFiles(revDir, "*.json"))
            {
                var idStr = Path.GetFileNameWithoutExtension(file);
                if (!long.TryParse(idStr, out var savedUnixMs)) continue;

                BuiltProgram? p = null;
                try { p = JsonSerializer.Deserialize<BuiltProgram>(File.ReadAllText(file), _opts); }
                catch { /* corrupt revision — still listed, with zero counts */ }

                result.Add(new ProgramRevisionInfo
                {
                    Id            = idStr,
                    SavedUnixMs   = savedUnixMs,
                    StepCount     = CountSteps(p?.Steps),
                    VariableCount = p?.Variables?.Count ?? 0,
                    Note          = null,
                });
            }
            result.Sort((a, b) => b.SavedUnixMs.CompareTo(a.SavedUnixMs));
            return result;
        }
    }

    private BuiltProgram? GetRevisionLocked(string name, string id)
    {
        if (string.IsNullOrEmpty(id) || !id.All(char.IsDigit)) return null;
        var file = Path.Combine(RevisionDir(name), id + ".json");
        if (!File.Exists(file)) return null;
        try { return JsonSerializer.Deserialize<BuiltProgram>(File.ReadAllText(file), _opts); }
        catch { return null; }
    }

    public BuiltProgram? GetRevision(string name, string id)
    {
        lock (_lock) { return GetRevisionLocked(name, id); }
    }

    /// <summary>
    /// Restores a revision as the current content, saving it via <see cref="Save"/> (which itself
    /// archives the pre-restore state as a new revision). Keeps the program's current id/name even
    /// if the revision predates a rename.
    /// </summary>
    public BuiltProgram? RestoreRevision(string name, string id)
    {
        lock (_lock)
        {
            if (!_programs.TryGetValue(name, out var current)) return null;
            var revision = GetRevisionLocked(name, id);
            if (revision == null) return null;

            revision.Id   = current.Id;
            revision.Name = current.Name;

            Save(revision);
            return revision;
        }
    }

    public BuiltProgram? Get(string name)
    {
        lock (_lock) { _programs.TryGetValue(name, out var p); return p; }
    }

    public BuiltProgram? GetById(string id)
    {
        lock (_lock) { return _programs.Values.FirstOrDefault(p => p.Id == id); }
    }

    public List<BuiltProgram> GetAll()
    {
        lock (_lock) { return _programs.Values.ToList(); }
    }

    // ── Image storage ─────────────────────────────────────────────────────────

    private string ImagePath(string name)
    {
        var safe = string.Concat(name.Split(Path.GetInvalidFileNameChars()));
        return Path.Combine(_imageDir, safe + ".jpg");
    }

    public void SaveImage(string name, byte[] bytes)
    {
        Directory.CreateDirectory(_imageDir);
        AtomicFile.WriteAllBytes(ImagePath(name), bytes);
    }

    public byte[]? GetImage(string name)
    {
        var path = ImagePath(name);
        return File.Exists(path) ? File.ReadAllBytes(path) : null;
    }

    public Dictionary<string, string?> GetAllImages()
    {
        var result = new Dictionary<string, string?>();
        lock (_lock)
        {
            foreach (var name in _programs.Keys)
            {
                var bytes = GetImage(name);
                result[name] = bytes != null ? Convert.ToBase64String(bytes) : null;
            }
        }
        return result;
    }
}
