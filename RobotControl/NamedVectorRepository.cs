using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.Json;

/// <summary>
/// Generic base repository for any named Vector6-derived entity (Point, Tool, …).
/// Handles JSON persistence, full CRUD, and a rolling 10-entry history per item.
/// Subclasses supply the concrete history-entry factory via CreateHistoryEntry().
/// </summary>
public abstract class NamedVectorRepository<TItem, TEntry>
    where TItem  : Vector6, INamedVector, new()
    where TEntry : class
{
    private readonly string _itemsFile;
    private readonly string _historyFile;

    protected readonly JsonSerializerOptions _jsonOptions = new()
    {
        WriteIndented = true,
        PropertyNameCaseInsensitive = true,
    };

    // ── Public state ─────────────────────────────────────────────────────────

    public Dictionary<string, TItem> Items { get; private set; } = new();
    public string? ItemsJson { get; private set; }
    public long LastUpdatedUnixMs { get; private set; }

    private Dictionary<string, List<TEntry>> _history = new();

    // ── Constructor ──────────────────────────────────────────────────────────

    protected NamedVectorRepository(string itemsFile, string historyFile)
    {
        _itemsFile   = itemsFile;
        _historyFile = historyFile;
        LoadItems();
        LoadHistory();
    }

    // ── Load / Save ──────────────────────────────────────────────────────────

    private void LoadItems()
    {
        if (!File.Exists(_itemsFile)) { SaveItems(); return; }

        try
        {
            ItemsJson = File.ReadAllText(_itemsFile);
            var list  = JsonSerializer.Deserialize<List<TItem>>(ItemsJson, _jsonOptions) ?? new();

            Items = list
                .Where(i => !string.IsNullOrWhiteSpace(i.Name))
                .ToDictionary(i => i.Name!, i => i);
        }
        catch
        {
            // File exists but is corrupt or invalid — start fresh
            Items = new();
            SaveItems();
        }
    }

    protected void SaveItems()
    {
        LastUpdatedUnixMs = NowUnixMs();
        ItemsJson         = JsonSerializer.Serialize(Items.Values.ToList(), _jsonOptions);
        File.WriteAllText(_itemsFile, ItemsJson);
    }

    private void LoadHistory()
    {
        if (!File.Exists(_historyFile)) { SaveHistory(); return; }

        try
        {
            var json = File.ReadAllText(_historyFile);
            _history = JsonSerializer.Deserialize<Dictionary<string, List<TEntry>>>(json, _jsonOptions) ?? new();
        }
        catch
        {
            // File exists but is corrupt or invalid — start fresh
            _history = new();
            SaveHistory();
        }
    }

    private void SaveHistory()
        => File.WriteAllText(_historyFile, JsonSerializer.Serialize(_history, _jsonOptions));

    // ── CRUD ─────────────────────────────────────────────────────────────────

    public TItem SaveItem(string name, Vector6 vector)
    {
        var item = new TItem();
        item.Copy(vector);
        item.Name             = name;
        item.LastUpdatedUnixMs = NowUnixMs();

        if (string.IsNullOrWhiteSpace(item.Name))
            throw new Exception($"{typeof(TItem).Name} must have a Name");

        Items[item.Name!] = item;
        AppendHistory(item);
        SaveItems();
        SaveHistory();
        return item;
    }

    public void EditItem(string name, Dictionary<string, object?> values)
    {
        if (!Items.TryGetValue(name, out var item))
            throw new Exception($"{typeof(TItem).Name} '{name}' not found");

        ApplyValues(item, values);

        // Rename: move to new key if Name changed
        if (item.Name != name && !string.IsNullOrWhiteSpace(item.Name))
        {
            Items.Remove(name);
            Items[item.Name!] = item;
        }

        item.LastUpdatedUnixMs = NowUnixMs();
        AppendHistory(item);
        SaveItems();
        SaveHistory();
    }

    public void DeleteItem(string name)
    {
        if (Items.Remove(name))
        {
            _history.Remove(name);
            SaveItems();
            SaveHistory();
        }
    }

    public TItem? Get(string name)
    {
        Items.TryGetValue(name, out var item);
        return item;
    }

    // ── History ──────────────────────────────────────────────────────────────

    /// <summary>Subclass supplies the concrete history-entry wrapper.</summary>
    protected abstract TEntry CreateHistoryEntry(TItem item);

    private void AppendHistory(TItem item)
    {
        if (string.IsNullOrWhiteSpace(item.Name)) return;

        if (!_history.ContainsKey(item.Name!))
            _history[item.Name!] = new();

        // Deep-clone via round-trip serialisation
        var clone = JsonSerializer.Deserialize<TItem>(
            JsonSerializer.Serialize(item, _jsonOptions), _jsonOptions)!;

        _history[item.Name!].Insert(0, CreateHistoryEntry(clone));

        // Keep rolling window of 10
        if (_history[item.Name!].Count > 10)
            _history[item.Name!].RemoveAt(10);
    }

    // ── Reflection apply ─────────────────────────────────────────────────────

    protected void ApplyValues(TItem item, Dictionary<string, object?> values)
    {
        // Include properties inherited from Vector6
        var props = typeof(TItem)
            .GetProperties(BindingFlags.Public | BindingFlags.Instance);

        foreach (var kvp in values)
        {
            var prop = props.FirstOrDefault(p =>
                string.Equals(p.Name, kvp.Key, StringComparison.OrdinalIgnoreCase));

            if (prop == null || !prop.CanWrite) continue;

            if (kvp.Value == null)
            {
                prop.SetValue(item, null);
                continue;
            }

            // Unwrap nullable types before converting
            var targetType = Nullable.GetUnderlyingType(prop.PropertyType) ?? prop.PropertyType;
            var converted  = Convert.ChangeType(kvp.Value, targetType);
            prop.SetValue(item, converted);
        }
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    protected static long NowUnixMs()
        => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
}
