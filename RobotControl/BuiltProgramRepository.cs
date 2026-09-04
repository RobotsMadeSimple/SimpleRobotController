using System.Text.Json;

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
                        File.WriteAllText(dest, JsonSerializer.Serialize(p, _opts));
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
        File.WriteAllText(ProgramPath(program.Name), JsonSerializer.Serialize(program, _opts));
        LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }

    private string ProgramPath(string name)
    {
        var safe = string.Concat(name.Split(Path.GetInvalidFileNameChars()));
        return Path.Combine(_dir, safe + ".json");
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
            }

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
            return true;
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
        File.WriteAllBytes(ImagePath(name), bytes);
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
