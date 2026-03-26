using System.Text.Json;

/// <summary>
/// Persists built programs (step sequences) to disk as builtPrograms.json.
/// Program images are stored as separate JPEG files under programImages/.
/// Thread-safe via lock.
/// </summary>
public class BuiltProgramRepository
{
    private readonly string _file;
    private readonly string _imageDir;
    private readonly object _lock = new();
    private readonly JsonSerializerOptions _opts = new()
    {
        WriteIndented          = true,
        Converters             = { new System.Text.Json.Serialization.JsonStringEnumConverter() },
        PropertyNameCaseInsensitive = true,
    };

    private Dictionary<string, BuiltProgram> _programs = new();

    public long LastUpdatedUnixMs { get; private set; }

    public BuiltProgramRepository(string file = "builtPrograms.json")
    {
        _file     = file;
        _imageDir = Path.Combine(Path.GetDirectoryName(Path.GetFullPath(file)) ?? ".", "programImages");
        Load();
    }

    // ── Persistence ──────────────────────────────────────────────────────────

    private void Load()
    {
        if (!File.Exists(_file)) return;
        try
        {
            var json = File.ReadAllText(_file);
            var list = JsonSerializer.Deserialize<List<BuiltProgram>>(json, _opts);
            if (list != null)
                _programs = list.ToDictionary(p => p.Name, p => p);
        }
        catch { /* corrupt file — start fresh */ }
    }

    private void Save()
    {
        var json = JsonSerializer.Serialize(_programs.Values.ToList(), _opts);
        File.WriteAllText(_file, json);
        LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }

    // ── Public API ───────────────────────────────────────────────────────────

    public void Save(BuiltProgram program)
    {
        lock (_lock)
        {
            program.LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            _programs[program.Name] = program;
            Save();
        }
    }

    public bool Delete(string name)
    {
        lock (_lock)
        {
            if (!_programs.Remove(name)) return false;
            Save();
            // Remove associated image file if present
            var imgPath = ImagePath(name);
            if (File.Exists(imgPath)) File.Delete(imgPath);
            return true;
        }
    }

    public BuiltProgram? Get(string name)
    {
        lock (_lock)
        {
            _programs.TryGetValue(name, out var p);
            return p;
        }
    }

    public List<BuiltProgram> GetAll()
    {
        lock (_lock)
        {
            return _programs.Values.ToList();
        }
    }

    // ── Image storage ─────────────────────────────────────────────────────────

    private string ImagePath(string name)
    {
        // Sanitise the program name so it's safe as a filename
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

    /// <summary>Returns a map of programName → base-64 image string for every program that has an image file.</summary>
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
