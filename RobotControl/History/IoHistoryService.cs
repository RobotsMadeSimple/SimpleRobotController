using System.Collections.Concurrent;
using Controller.RobotControl.Persistence;

namespace Controller.RobotControl.History;

/// <summary>
/// In-memory time-series history for numeric IO channels, plus explicit recordings.
///
/// Generic by design: a channel is addressed by a stable string key
/// (<c>io/&lt;deviceId&gt;/&lt;pin&gt;</c> for load cells; <c>device/&lt;id&gt;/&lt;channel&gt;</c> reserved
/// for phone channels). Only load cells are wired up today via <see cref="AttachNano"/>.
///
/// Two concepts:
///  • Rolling history — an always-on per-channel ring buffer bounded by a configurable
///    max retention time. Backs the live sparkline and the traversable history.
///  • Recording — a start→stop capture. While active its samples are exempt from eviction;
///    on stop the slice is snapshotted so the full recording survives regardless of retention.
///
/// Default is in-memory only; a recording can be explicitly saved to disk (opt-in), which
/// makes it survive a restart and appear in the recordings list.
///
/// Thread-safety: <see cref="Record"/> runs on the Nano device thread; disk I/O only ever
/// happens off that thread (on the thread pool), so a slow write never stalls serial draining.
/// </summary>
public sealed class IoHistoryService
{
    private readonly string _dir;
    private readonly string _configPath;
    private readonly string _indexPath;

    private readonly ConcurrentDictionary<string, ChannelBuffer> _channels   = new();
    private readonly ConcurrentDictionary<string, Recording>     _recordings = new();

    private long _maxRetentionMs = 600_000; // 10 min
    private long _lastRecordingUpdate;
    private readonly object _indexLock = new();

    public long MaxRetentionMs      => Interlocked.Read(ref _maxRetentionMs);
    public long LastRecordingUpdate => Interlocked.Read(ref _lastRecordingUpdate);

    public IoHistoryService(string dir)
    {
        _dir        = dir;
        _configPath = Path.Combine(dir, "config.json");
        _indexPath  = Path.Combine(dir, "index.json");
        LoadConfig();
        LoadIndex();
    }

    /// <summary>Stable channel key for a load cell (rename-proof: nano id + DT pin).</summary>
    public static string LoadCellKey(string deviceId, int pin) => $"io/{deviceId}/{pin}";

    // ── Live capture ────────────────────────────────────────────────────────────

    /// <summary>Subscribe to a nano manager's per-sample load-cell event.</summary>
    public void AttachNano(Io.NanoManager nano) => nano.LoadCellSampled += OnLoadCellSample;

    private void OnLoadCellSample(string deviceId, Io.IoPinState state, long unixMs)
    {
        // Grams is 0 while uncalibrated; we keep raw too so the app can plot raw until then.
        Record(
            LoadCellKey(deviceId, state.Pin), unixMs, state.Grams, state.Raw,
            string.IsNullOrEmpty(state.Unit) ? "g" : state.Unit,
            string.IsNullOrEmpty(state.Name) ? $"Load Cell {state.Pin}" : state.Name);
    }

    /// <summary>Append one sample to a channel's rolling buffer. Safe to call at high rate.</summary>
    public void Record(string channelKey, long unixMs, double value, double raw, string unit, string label)
    {
        var buf = _channels.GetOrAdd(channelKey, k => new ChannelBuffer(k));
        buf.SetMeta(unit, label);
        buf.Append(new IoSample(unixMs, value, raw), MaxRetentionMs);
    }

    // ── Reads ─────────────────────────────────────────────────────────────────────

    /// <summary>Raw (undecimated) samples strictly newer than <paramref name="afterUnixMs"/>,
    /// capped at <paramref name="maxCount"/>. Used by the live stream + sparkline backfill.</summary>
    public List<IoSampleDto> GetSince(string channelKey, long afterUnixMs, int maxCount)
        => _channels.TryGetValue(channelKey, out var b)
            ? b.Since(afterUnixMs, maxCount).Select(ToDto).ToList()
            : new();

    public string ChannelUnit(string channelKey) => _channels.TryGetValue(channelKey, out var b) ? b.Unit : "g";

    /// <summary>A decimated window of a channel's rolling history for lazy traversal.</summary>
    public IoHistoryWindow GetWindow(string channelKey, long? fromMs, long? toMs, int maxPoints)
    {
        long now = Now();
        long to   = toMs   ?? now;
        long from = fromMs ?? (to - MaxRetentionMs);
        var w = new IoHistoryWindow { Key = channelKey, From = from, To = to, Unit = ChannelUnit(channelKey) };
        if (_channels.TryGetValue(channelKey, out var b))
            w.Samples = Decimate(b.Slice(from, to), maxPoints);
        return w;
    }

    // ── Recordings ──────────────────────────────────────────────────────────────

    public IoRecordingMeta StartRecording(string channelKey, string? name)
    {
        long now = Now();
        var buf = _channels.GetOrAdd(channelKey, k => new ChannelBuffer(k));
        var rec = new Recording
        {
            Meta = new IoRecordingMeta
            {
                Id          = Guid.NewGuid().ToString("N"),
                ChannelKey  = channelKey,
                Name        = string.IsNullOrWhiteSpace(name) ? DefaultName(now) : name!.Trim(),
                Unit        = buf.Unit,
                Label       = buf.Label,
                StartUnixMs = now,
                Active      = true,
            }
        };
        _recordings[rec.Meta.Id] = rec;
        RecomputeProtected(channelKey);
        Bump();
        return Clone(rec.Meta);
    }

    public IoRecordingMeta? StopRecording(string id)
    {
        if (!_recordings.TryGetValue(id, out var rec)) return null;
        lock (rec.Lock)
        {
            if (rec.Meta.Active)
            {
                long now = Now();
                rec.Meta.EndUnixMs = now;
                rec.Meta.Active    = false;
                // Everything captured since the recording began (its start is protected from
                // eviction). EndUnixMs is wall-clock metadata; the sample set is bounded by start.
                rec.Samples = _channels.TryGetValue(rec.Meta.ChannelKey, out var b)
                    ? b.Slice(rec.Meta.StartUnixMs, long.MaxValue)
                    : new List<IoSample>();
                rec.Meta.SampleCount = rec.Samples.Count;
            }
        }
        RecomputeProtected(rec.Meta.ChannelKey);
        Bump();
        return Clone(rec.Meta);
    }

    public List<IoRecordingMeta> ListRecordings()
    {
        var list = new List<IoRecordingMeta>();
        foreach (var r in _recordings.Values)
        {
            var m = Clone(r.Meta);
            if (m.Active && _channels.TryGetValue(m.ChannelKey, out var b))
                m.SampleCount = b.CountInRange(m.StartUnixMs, long.MaxValue);
            list.Add(m);
        }
        list.Sort((a, b) => b.StartUnixMs.CompareTo(a.StartUnixMs)); // newest first
        return list;
    }

    public IoRecordingMeta? GetRecordingMeta(string id)
        => _recordings.TryGetValue(id, out var r) ? Clone(r.Meta) : null;

    /// <summary>A decimated window of one recording's samples (live slice while active,
    /// snapshot once stopped, lazily loaded from disk if only saved).</summary>
    public IoHistoryWindow? GetRecordingWindow(string id, long? fromMs, long? toMs, int maxPoints)
    {
        if (!_recordings.TryGetValue(id, out var rec)) return null;
        List<IoSample> samples;
        lock (rec.Lock)
        {
            if (rec.Meta.Active && _channels.TryGetValue(rec.Meta.ChannelKey, out var b))
                samples = b.Slice(rec.Meta.StartUnixMs, long.MaxValue);
            else
            {
                rec.Samples ??= LoadSamplesFromDisk(id);
                samples = rec.Samples ?? new List<IoSample>();
            }
        }
        // No explicit bounds → the whole recording; explicit from/to → a zoom window.
        long from = fromMs ?? long.MinValue;
        long to   = toMs   ?? long.MaxValue;
        var filtered = (fromMs == null && toMs == null)
            ? samples
            : samples.Where(s => s.UnixMs >= from && s.UnixMs <= to).ToList();
        return new IoHistoryWindow
        {
            Key = rec.Meta.ChannelKey, Unit = rec.Meta.Unit, From = from, To = to,
            Samples = Decimate(filtered, maxPoints),
        };
    }

    /// <summary>Persist a recording to disk (opt-in). Stops it first if still active.</summary>
    public IoRecordingMeta? SaveRecording(string id)
    {
        if (!_recordings.TryGetValue(id, out var rec)) return null;
        if (rec.Meta.Active) StopRecording(id);

        StoredRecording stored;
        lock (rec.Lock)
        {
            rec.Meta.Saved = true;
            var dtos = (rec.Samples ?? new List<IoSample>()).Select(ToDto).ToList();
            rec.Meta.SampleCount = dtos.Count;
            stored = new StoredRecording { Meta = Clone(rec.Meta), Samples = dtos };
        }
        // Runs on the command thread (never the Nano device thread), so a synchronous write is
        // safe and keeps the save deterministic for callers that read it back immediately.
        PersistRecording(stored);
        Bump();
        return Clone(rec.Meta);
    }

    public bool DeleteRecording(string id)
    {
        if (!_recordings.TryRemove(id, out var rec)) return false;
        RecomputeProtected(rec.Meta.ChannelKey);
        if (rec.Meta.Saved)
        {
            try { File.Delete(RecordingPath(id)); } catch { /* best effort */ }
            RewriteIndex();
        }
        Bump();
        return true;
    }

    // ── Retention ───────────────────────────────────────────────────────────────

    public long SetRetention(long ms)
    {
        ms = Math.Clamp(ms, 1_000, 24L * 3600 * 1000);
        Interlocked.Exchange(ref _maxRetentionMs, ms);
        foreach (var b in _channels.Values) b.Reevict(ms);
        SaveConfig();
        return ms;
    }

    // ── Persistence ─────────────────────────────────────────────────────────────

    private void LoadConfig()
    {
        var cfg = JsonFiles.Load<IoHistoryConfig>(_configPath, logTag: "IoHistory");
        if (cfg != null) _maxRetentionMs = Math.Clamp(cfg.MaxRetentionMs, 1_000, 24L * 3600 * 1000);
    }

    private void SaveConfig()
    {
        try { JsonFiles.Save(_configPath, new IoHistoryConfig { MaxRetentionMs = MaxRetentionMs }); }
        catch (Exception ex) { Console.WriteLine($"[IoHistory] Failed to save config: {ex.Message}"); }
    }

    private void LoadIndex()
    {
        var index = JsonFiles.Load<List<IoRecordingMeta>>(_indexPath, logTag: "IoHistory");
        if (index == null) return;
        foreach (var m in index)
        {
            m.Active = false;
            m.Saved  = true;
            _recordings[m.Id] = new Recording { Meta = m, Samples = null }; // samples lazy-loaded
        }
    }

    private string RecordingPath(string id) => Path.Combine(_dir, $"{id}.json");

    private void PersistRecording(StoredRecording stored)
    {
        try { JsonFiles.Save(RecordingPath(stored.Meta.Id), stored); }
        catch (Exception ex) { Console.WriteLine($"[IoHistory] Failed to save recording {stored.Meta.Id}: {ex.Message}"); return; }
        RewriteIndex();
    }

    private void RewriteIndex()
    {
        lock (_indexLock)
        {
            var metas = _recordings.Values
                .Where(r => r.Meta.Saved)
                .Select(r => Clone(r.Meta))
                .OrderByDescending(m => m.StartUnixMs)
                .ToList();
            try { JsonFiles.Save(_indexPath, metas); }
            catch (Exception ex) { Console.WriteLine($"[IoHistory] Failed to save index: {ex.Message}"); }
        }
    }

    private List<IoSample>? LoadSamplesFromDisk(string id)
    {
        var stored = JsonFiles.Load<StoredRecording>(RecordingPath(id), logTag: "IoHistory");
        return stored?.Samples.Select(d => new IoSample(d.T, d.V, d.R)).ToList();
    }

    // ── Helpers ─────────────────────────────────────────────────────────────────

    private void RecomputeProtected(string channelKey)
    {
        long min = long.MaxValue;
        foreach (var r in _recordings.Values)
            if (r.Meta.Active && r.Meta.ChannelKey == channelKey && r.Meta.StartUnixMs < min)
                min = r.Meta.StartUnixMs;
        if (_channels.TryGetValue(channelKey, out var b)) b.MinProtectedUnixMs = min;
    }

    private static long Now() => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
    private void Bump() => Interlocked.Exchange(ref _lastRecordingUpdate, Now());
    private static string DefaultName(long unixMs)
        => "Recording " + DateTimeOffset.FromUnixTimeMilliseconds(unixMs).LocalDateTime.ToString("MMM d HH:mm:ss");

    private static IoSampleDto ToDto(IoSample s) => new() { T = s.UnixMs, V = s.Value, R = s.Raw };

    private static IoRecordingMeta Clone(IoRecordingMeta m) => new()
    {
        Id = m.Id, ChannelKey = m.ChannelKey, Name = m.Name, Unit = m.Unit, Label = m.Label,
        StartUnixMs = m.StartUnixMs, EndUnixMs = m.EndUnixMs, SampleCount = m.SampleCount,
        Active = m.Active, Saved = m.Saved,
    };

    /// <summary>Bucketed min/max decimation: preserves peaks/troughs when zoomed out.</summary>
    private static List<IoSampleDto> Decimate(List<IoSample> src, int maxPoints)
    {
        if (maxPoints <= 0) maxPoints = 1000;
        if (src.Count <= maxPoints) return src.Select(ToDto).ToList();

        var outl = new List<IoSampleDto>(maxPoints);
        int buckets = Math.Max(1, maxPoints / 2);
        double step = (double)src.Count / buckets;
        for (int bkt = 0; bkt < buckets; bkt++)
        {
            int start = (int)(bkt * step);
            int end   = (int)((bkt + 1) * step);
            if (end <= start) end = start + 1;
            if (end > src.Count) end = src.Count;

            int minI = start, maxI = start;
            for (int i = start; i < end; i++)
            {
                if (src[i].Value < src[minI].Value) minI = i;
                if (src[i].Value > src[maxI].Value) maxI = i;
            }
            int a = Math.Min(minI, maxI), c = Math.Max(minI, maxI);
            outl.Add(ToDto(src[a]));
            if (c != a) outl.Add(ToDto(src[c]));
        }
        return outl;
    }

    // ── Nested types ──────────────────────────────────────────────────────────────

    private sealed class Recording
    {
        public IoRecordingMeta  Meta    = new();
        public List<IoSample>?  Samples;        // null = active (read live) or saved-not-loaded
        public readonly object  Lock = new();
    }

    /// <summary>A single channel's rolling buffer: time-ascending samples with front eviction.</summary>
    private sealed class ChannelBuffer
    {
        public readonly string Key;
        public string Unit  = "g";
        public string Label = "";
        public long   MinProtectedUnixMs = long.MaxValue; // never evict samples at/after this

        private readonly List<IoSample> _samples = new();
        private readonly object _lock = new();

        public ChannelBuffer(string key) => Key = key;

        public void SetMeta(string unit, string label)
        {
            if (!string.IsNullOrEmpty(unit))  Unit  = unit;
            if (!string.IsNullOrEmpty(label)) Label = label;
        }

        public void Append(IoSample s, long retentionMs)
        {
            lock (_lock) { _samples.Add(s); Evict(retentionMs); }
        }

        public void Reevict(long retentionMs) { lock (_lock) Evict(retentionMs); }

        private void Evict(long retentionMs)
        {
            if (_samples.Count == 0) return;
            long cutoff = _samples[^1].UnixMs - retentionMs;
            int i = 0;
            while (i < _samples.Count && _samples[i].UnixMs < cutoff && _samples[i].UnixMs < MinProtectedUnixMs) i++;
            if (i > 0) _samples.RemoveRange(0, i);
        }

        public List<IoSample> Since(long afterUnixMs, int maxCount)
        {
            lock (_lock)
            {
                var outl = new List<IoSample>();
                for (int i = LowerBound(afterUnixMs); i < _samples.Count && outl.Count < maxCount; i++)
                    outl.Add(_samples[i]);
                return outl;
            }
        }

        public List<IoSample> Slice(long fromMs, long toMs)
        {
            lock (_lock)
            {
                var outl = new List<IoSample>();
                foreach (var s in _samples)
                    if (s.UnixMs >= fromMs && s.UnixMs <= toMs) outl.Add(s);
                return outl;
            }
        }

        public int CountInRange(long fromMs, long toMs)
        {
            lock (_lock)
            {
                int n = 0;
                foreach (var s in _samples) if (s.UnixMs >= fromMs && s.UnixMs <= toMs) n++;
                return n;
            }
        }

        // First index strictly greater than afterUnixMs (samples are append-ordered ascending).
        private int LowerBound(long afterUnixMs)
        {
            int lo = 0, hi = _samples.Count;
            while (lo < hi)
            {
                int mid = (lo + hi) / 2;
                if (_samples[mid].UnixMs <= afterUnixMs) lo = mid + 1; else hi = mid;
            }
            return lo;
        }
    }
}
