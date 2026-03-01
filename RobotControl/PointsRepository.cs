using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.Json;

public class PointRepository
{
    private readonly string _pointsFile;
    private readonly string _historyFile;
    public string? pointsJson;

    private readonly JsonSerializerOptions _jsonOptions;

    // Unix ms when this point was last created or modified
    public long LastUpdatedUnixMs { get; set; }

    public Dictionary<string, Point> Points { get; private set; } = new();
    private Dictionary<string, List<PointHistoryEntry>> History { get; set; } = new();
    

    public PointRepository(
        string pointsFile = "points.json",
        string historyFile = "pointsHistory.json")
    {
        _pointsFile = pointsFile;
        _historyFile = historyFile;

        _jsonOptions = new JsonSerializerOptions
        {
            WriteIndented = true,
            PropertyNameCaseInsensitive = true
        };

        Load();
        LoadHistory();
    }

    // -------------------- LOAD / SAVE --------------------

    public void Load()
    {
        if (!File.Exists(_pointsFile))
        {
            Save();
            return;
        }

        pointsJson = File.ReadAllText(_pointsFile);
        var list = JsonSerializer.Deserialize<List<Point>>(pointsJson, _jsonOptions)
                   ?? new List<Point>();

        Points = list
            .Where(p => !string.IsNullOrWhiteSpace(p.Name))
            .ToDictionary(p => p.Name!, p => p);
    }

    public void Save()
    {
        LastUpdatedUnixMs = NowUnixMs();
        pointsJson = JsonSerializer.Serialize(Points.Values.ToList(), _jsonOptions);
        File.WriteAllText(_pointsFile, pointsJson);
    }

    private void LoadHistory()
    {
        if (!File.Exists(_historyFile))
        {
            SaveHistory();
            return;
        }

        var json = File.ReadAllText(_historyFile);
        History = JsonSerializer.Deserialize<
            Dictionary<string, List<PointHistoryEntry>>
        >(json, _jsonOptions) ?? new();
    }

    private void SaveHistory()
    {
        var json = JsonSerializer.Serialize(History, _jsonOptions);
        File.WriteAllText(_historyFile, json);
    }

    // -------------------- CREATE --------------------

    public Point SavePoint(string name, Vector6 vector)
    {
        var point = new Point();
        point.Copy(vector);
        point.Name = name;
        point.LastUpdatedUnixMs = NowUnixMs();

        if (string.IsNullOrWhiteSpace(point.Name))
            throw new Exception("Point must have a Name");

        Points[point.Name!] = point;

        AddHistory(point);

        Save();
        SaveHistory();

        return point;
    }

    // -------------------- EDIT --------------------

    public void EditPoint(string name, Dictionary<string, object?> values)
    {
        if (!Points.TryGetValue(name, out var point))
            throw new Exception($"Point '{name}' not found");

        ApplyValues(point, values);

        // Rename handling
        if (point.Name != name && !string.IsNullOrWhiteSpace(point.Name))
        {
            Points.Remove(name);
            Points[point.Name!] = point;
        }

        point.LastUpdatedUnixMs = NowUnixMs();

        AddHistory(point);

        Save();
        SaveHistory();
    }

    // -------------------- DELETE --------------------

    public void DeletePoint(string name)
    {
        if (Points.Remove(name))
        {
            History.Remove(name);
            Save();
            SaveHistory();
        }
    }

    public Point? Get(string name)
    {
        Points.TryGetValue(name, out var point);
        return point;
    }

    // -------------------- HISTORY --------------------

    private void AddHistory(Point point)
    {
        if (string.IsNullOrWhiteSpace(point.Name))
            return;

        if (!History.ContainsKey(point.Name!))
            History[point.Name!] = new List<PointHistoryEntry>();

        // Deep clone via serialization (future-proof)
        var clone = JsonSerializer.Deserialize<Point>(
            JsonSerializer.Serialize(point, _jsonOptions),
            _jsonOptions
        )!;

        History[point.Name!].Insert(0, new PointHistoryEntry
        {
            TimestampUnixMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
            Point = clone
        });

        // Keep only latest 10
        if (History[point.Name!].Count > 10)
            History[point.Name!].RemoveAt(10);
    }

    // -------------------- REFLECTION APPLY --------------------

    private void ApplyValues(Point point, Dictionary<string, object?> values)
    {
        var props = typeof(Point)
            .GetProperties(BindingFlags.Public | BindingFlags.Instance);

        foreach (var kvp in values)
        {
            var prop = props.FirstOrDefault(p =>
                string.Equals(p.Name, kvp.Key, StringComparison.OrdinalIgnoreCase));

            if (prop == null || !prop.CanWrite)
                continue;

            if (kvp.Value == null)
            {
                prop.SetValue(point, null);
                continue;
            }

            var converted = Convert.ChangeType(kvp.Value, prop.PropertyType);
            prop.SetValue(point, converted);
        }
    }

    private static long NowUnixMs()
    {
        return DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    }
}
