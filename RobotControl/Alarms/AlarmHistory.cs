using System.Text.Json.Serialization;
using Controller.RobotControl.Persistence;

namespace Controller.RobotControl.Alarms;

/// <summary>Where an alarm came from. Every field is optional; ids are stable ids, never names.</summary>
public sealed class AlarmSource
{
    [JsonPropertyName("programId")]   public string? ProgramId   { get; set; }
    /// <summary>The program's name when it was raised — display only.</summary>
    [JsonPropertyName("programName")] public string? ProgramName { get; set; }
    [JsonPropertyName("stepId")]      public string? StepId      { get; set; }
    [JsonPropertyName("deviceId")]    public string? DeviceId    { get; set; }
    [JsonPropertyName("pluginId")]    public string? PluginId    { get; set; }
    [JsonPropertyName("background")]  public bool?   Background  { get; set; }
}

/// <summary>
/// One alarm. <see cref="Cleared"/> means the condition has gone (returned to normal);
/// <see cref="Acknowledged"/> means an operator has seen it. An alarm is <b>active</b> — listed
/// first, counted in <c>activeAlarmCount</c> — until it is both. Events with no lasting condition
/// (a program error) are raised already cleared, so they stay active only until acknowledged.
/// </summary>
public sealed class AlarmEntry
{
    [JsonPropertyName("id")]                 public string Id { get; set; } = "";
    [JsonPropertyName("code")]               public string Code { get; set; } = "";
    [JsonPropertyName("severity")]           public string Severity { get; set; } = AlarmSeverity.Error;
    [JsonPropertyName("title")]              public string Title { get; set; } = "";
    [JsonPropertyName("message")]            public string Message { get; set; } = "";
    [JsonPropertyName("source")]             public AlarmSource Source { get; set; } = new();
    [JsonPropertyName("timestampUnixMs")]    public long TimestampUnixMs { get; set; }
    [JsonPropertyName("acknowledged")]       public bool Acknowledged { get; set; }
    [JsonPropertyName("acknowledgedUnixMs")] public long? AcknowledgedUnixMs { get; set; }
    [JsonPropertyName("cleared")]            public bool Cleared { get; set; }
    [JsonPropertyName("clearedUnixMs")]      public long? ClearedUnixMs { get; set; }
    /// <summary>Identifies the ongoing condition (e.g. "estop", "device:nano1") so it clears when the
    /// condition does. Null for one-off events.</summary>
    [JsonPropertyName("conditionKey")]       public string? ConditionKey { get; set; }

    [JsonIgnore] public bool Active => !(Cleared && Acknowledged);

    public AlarmEntry Copy() => (AlarmEntry)MemberwiseClone();
}

/// <summary>
/// The bounded alarm history (docs/websocket-api.md "Safety & alarms"). Holds the newest
/// <see cref="MaxEntries"/> entries; an entry whose condition is still present is never dropped by
/// the bound. Persisted to <c>alarms.json</c> in the data folder by a debounced write on the thread
/// pool — never on the caller's thread, so raising from the motion or safety thread costs a lock
/// and a list insert. Thread-safe.
/// </summary>
public sealed class AlarmHistory
{
    public const int MaxEntries = 500;
    public const int DefaultPageSize = 50;
    private const int WriteDelayMs = 250;

    private readonly string _file;
    private readonly int _maxEntries;
    private readonly object _lock = new();
    private readonly object _writeLock = new();
    private readonly List<AlarmEntry> _entries = new(); // oldest first
    private int _writePending;
    private long _lastUpdated;

    /// <summary>Unix ms of the last change (raise, acknowledge, clear) — clients refetch on change.</summary>
    public long LastUpdatedUnixMs => Interlocked.Read(ref _lastUpdated);

    /// <summary>Called (outside the lock) with a copy of every newly raised entry.</summary>
    public event Action<AlarmEntry>? Raised;

    public AlarmHistory(string file = "alarms.json", int maxEntries = MaxEntries)
    {
        _file       = file;
        _maxEntries = Math.Max(1, maxEntries);
        var loaded = JsonFiles.Load<List<AlarmEntry>>(_file, JsonDefaults.File, "Alarms");
        if (loaded != null)
        {
            _entries.AddRange(loaded.Where(e => !string.IsNullOrEmpty(e.Id)).OrderBy(e => e.TimestampUnixMs));
            TrimLocked();
            if (_entries.Count > 0) _lastUpdated = _entries.Max(e => Math.Max(e.TimestampUnixMs, Math.Max(e.AcknowledgedUnixMs ?? 0, e.ClearedUnixMs ?? 0)));
        }
    }

    /// <summary>
    /// Records an alarm. With a <paramref name="conditionKey"/>, an alarm for the same condition that
    /// has not cleared yet is reused (the condition is still the same occurrence) instead of adding a
    /// duplicate. <paramref name="cleared"/> raises a one-off event.
    /// </summary>
    public AlarmEntry Raise(string code, string message, AlarmSource? source = null,
                            string? conditionKey = null, bool cleared = false, string? severity = null, long? nowUnixMs = null)
    {
        long now = nowUnixMs ?? DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        var help = AlarmCatalog.Get(code);
        AlarmEntry copy;
        lock (_lock)
        {
            if (conditionKey != null && _entries.FindLast(e => e.ConditionKey == conditionKey && !e.Cleared) is { } open)
                return open.Copy();

            var entry = new AlarmEntry
            {
                Id              = Guid.NewGuid().ToString("N"),
                Code            = code,
                Severity        = severity ?? help.Severity,
                Title           = help.Title,
                Message         = message,
                Source          = source ?? new AlarmSource(),
                TimestampUnixMs = now,
                ConditionKey    = conditionKey,
                Cleared         = cleared,
                ClearedUnixMs   = cleared ? now : null,
            };
            _entries.Add(entry);
            TrimLocked();
            Touch(now);
            copy = entry.Copy();
        }
        ScheduleWrite();
        try { Raised?.Invoke(copy); } catch (Exception ex) { Console.WriteLine($"[Alarms] Raised handler failed: {ex.Message}"); }
        return copy;
    }

    /// <summary>Marks the open alarm(s) for <paramref name="conditionKey"/> cleared (the condition went away).</summary>
    public int ClearCondition(string conditionKey, long? nowUnixMs = null)
    {
        long now = nowUnixMs ?? DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        int n = 0;
        lock (_lock)
        {
            foreach (var e in _entries)
                if (e.ConditionKey == conditionKey && !e.Cleared) { e.Cleared = true; e.ClearedUnixMs = now; n++; }
            if (n > 0) Touch(now);
        }
        if (n > 0) ScheduleWrite();
        return n;
    }

    /// <summary>Acknowledges one alarm by id, or every alarm when <paramref name="id"/> is null.
    /// Returns how many changed; -1 when the id is unknown.</summary>
    public int Acknowledge(string? id, long? nowUnixMs = null)
    {
        long now = nowUnixMs ?? DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        int n = 0;
        lock (_lock)
        {
            if (id != null)
            {
                var e = _entries.Find(x => x.Id == id);
                if (e == null) return -1;
                if (!e.Acknowledged) { e.Acknowledged = true; e.AcknowledgedUnixMs = now; n = 1; }
            }
            else
            {
                foreach (var e in _entries)
                    if (!e.Acknowledged) { e.Acknowledged = true; e.AcknowledgedUnixMs = now; n++; }
            }
            if (n > 0) Touch(now);
        }
        if (n > 0) ScheduleWrite();
        return n;
    }

    /// <summary>
    /// Clears the history: removes every alarm whose condition has gone (acknowledged or not —
    /// clearing is the operator's acknowledgement). Alarms whose condition is still present stay.
    /// Returns how many were removed.
    /// </summary>
    public int ClearHistory(long? nowUnixMs = null)
    {
        long now = nowUnixMs ?? DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        int n;
        lock (_lock)
        {
            n = _entries.RemoveAll(e => e.Cleared);
            if (n > 0) Touch(now);
        }
        if (n > 0) ScheduleWrite();
        return n;
    }

    /// <summary>Number of active alarms (not both cleared and acknowledged).</summary>
    public int ActiveCount
    {
        get { lock (_lock) return _entries.Count(e => e.Active); }
    }

    public int Count
    {
        get { lock (_lock) return _entries.Count; }
    }

    /// <summary>
    /// A page of alarms: active ones first (most severe, then newest), then the rest newest first.
    /// </summary>
    public (int Total, int ActiveCount, List<AlarmEntry> Page) GetPage(int offset = 0, int limit = DefaultPageSize, bool activeOnly = false)
    {
        lock (_lock)
        {
            var ordered = _entries
                .Where(e => !activeOnly || e.Active)
                .OrderBy(e => e.Active ? 0 : 1)
                .ThenBy(e => e.Active ? AlarmSeverity.Rank(e.Severity) : 0)
                .ThenByDescending(e => e.TimestampUnixMs)
                .ToList();
            offset = Math.Max(0, offset);
            limit  = Math.Clamp(limit <= 0 ? DefaultPageSize : limit, 1, _maxEntries);
            var page = ordered.Skip(offset).Take(limit).Select(e => e.Copy()).ToList();
            return (ordered.Count, _entries.Count(e => e.Active), page);
        }
    }

    /// <summary>Writes alarms.json now (tests, shutdown). Normal writes are debounced on the thread pool.</summary>
    public void Flush()
    {
        lock (_writeLock)
        {
            // Snapshot under the write lock so a later write never lands an older snapshot.
            List<AlarmEntry> snapshot;
            lock (_lock) snapshot = _entries.Select(e => e.Copy()).ToList();
            try { JsonFiles.Save(_file, snapshot, JsonDefaults.File); }
            catch (Exception ex) { Console.WriteLine($"[Alarms] Failed to write {_file}: {ex.Message}"); }
        }
    }

    private void ScheduleWrite()
    {
        if (Interlocked.Exchange(ref _writePending, 1) == 1) return;
        _ = Task.Run(async () =>
        {
            await Task.Delay(WriteDelayMs);
            Interlocked.Exchange(ref _writePending, 0);
            Flush();
        });
    }

    // Drop the oldest entries past the bound, but never one whose condition is still present.
    private void TrimLocked()
    {
        int excess = _entries.Count - _maxEntries;
        for (int i = 0; i < _entries.Count && excess > 0;)
        {
            if (_entries[i].Cleared) { _entries.RemoveAt(i); excess--; }
            else i++;
        }
    }

    private void Touch(long now)
    {
        // Strictly increasing, so two changes in the same millisecond are still two updates.
        long prev = Interlocked.Read(ref _lastUpdated);
        Interlocked.Exchange(ref _lastUpdated, Math.Max(now, prev + 1));
    }
}
