using System.Text.Json;

/// <summary>
/// Persists named grids to disk as grids.json.
/// Thread-safe via lock.
/// </summary>
public class GridRepository
{
    private readonly string _file;
    private readonly object _lock = new();
    private readonly JsonSerializerOptions _opts = new()
    {
        WriteIndented           = true,
        PropertyNameCaseInsensitive = true,
    };

    private Dictionary<string, Grid> _grids = new();

    public long LastUpdatedUnixMs { get; private set; }

    public GridRepository(string file = "grids.json")
    {
        _file = file;
        Load();
    }

    private void Load()
    {
        if (!File.Exists(_file)) return;
        try
        {
            var json = File.ReadAllText(_file);
            var list = JsonSerializer.Deserialize<List<Grid>>(json, _opts);
            if (list != null)
                _grids = list.ToDictionary(g => g.Id, g => g);
        }
        catch { }
    }

    private void Save()
    {
        var json = JsonSerializer.Serialize(_grids.Values.ToList(), _opts);
        File.WriteAllText(_file, json);
        LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }

    public Grid Upsert(Grid grid)
    {
        lock (_lock)
        {
            if (string.IsNullOrWhiteSpace(grid.Id))
                grid.Id = Guid.NewGuid().ToString();
            grid.LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            _grids[grid.Id] = grid;
            Save();
            return grid;
        }
    }

    public bool Delete(string id)
    {
        lock (_lock)
        {
            if (!_grids.Remove(id)) return false;
            Save();
            return true;
        }
    }

    public Grid? Get(string id)
    {
        lock (_lock)
        {
            _grids.TryGetValue(id, out var g);
            return g;
        }
    }

    public List<Grid> GetAll()
    {
        lock (_lock)
        {
            return _grids.Values.ToList();
        }
    }
}
