using System.Text.Json;

/// <summary>
/// Persists named stacks to disk as stacks.json.
/// Thread-safe via lock.
/// </summary>
public class StackRepository
{
    private readonly string _file;
    private readonly object _lock = new();
    private readonly JsonSerializerOptions _opts = new()
    {
        WriteIndented            = true,
        PropertyNameCaseInsensitive = true,
    };

    private Dictionary<string, RobotStack> _stacks = new();

    public long LastUpdatedUnixMs { get; private set; }

    public StackRepository(string file = "stacks.json")
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
            var list = JsonSerializer.Deserialize<List<RobotStack>>(json, _opts);
            if (list != null)
                _stacks = list.ToDictionary(s => s.Id, s => s);
        }
        catch { }
    }

    private void Save()
    {
        var json = JsonSerializer.Serialize(_stacks.Values.ToList(), _opts);
        File.WriteAllText(_file, json);
        LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }

    public RobotStack Upsert(RobotStack stack)
    {
        lock (_lock)
        {
            if (string.IsNullOrWhiteSpace(stack.Id))
                stack.Id = Guid.NewGuid().ToString();
            stack.LastUpdatedUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            _stacks[stack.Id] = stack;
            Save();
            return stack;
        }
    }

    public bool Delete(string id)
    {
        lock (_lock)
        {
            if (!_stacks.Remove(id)) return false;
            Save();
            return true;
        }
    }

    public RobotStack? Get(string id)
    {
        lock (_lock)
        {
            _stacks.TryGetValue(id, out var s);
            return s;
        }
    }

    public List<RobotStack> GetAll()
    {
        lock (_lock)
        {
            return _stacks.Values.ToList();
        }
    }
}
