using System.Globalization;

namespace Controller.RobotControl.Plugins;

/// <summary>
/// One plugin's log: a 500-line ring buffer with absolute indices (paged like
/// <c>GetProgramLogs</c>), mirrored to a rolling <c>plugin.log</c> (2 MB × 3 files) and to
/// the controller console prefixed <c>[Plugin:&lt;id&gt;]</c>, rate-limited to 20 lines/s
/// (excess lines are counted and summarised). Thread-safe.
/// </summary>
public sealed class PluginLog
{
    public const int  RingCapacity      = 500;
    public const long MaxFileBytes      = 2 * 1024 * 1024;
    public const int  FileCount         = 3;
    public const int  ConsoleLinesPerSec = 20;

    private readonly string _pluginId;
    private readonly IPluginClock _clock;
    private readonly Action<string> _console;
    private readonly object _lock = new();
    private readonly Queue<string> _ring = new();
    private int _baseIndex;               // absolute index of _ring's first entry
    private string? _filePath;

    // console rate limit (per second window)
    private long _windowStartMs = long.MinValue / 2; // far past, without overflow in now - start
    private int  _windowCount;
    private int  _suppressed;

    public PluginLog(string pluginId, string? filePath, IPluginClock? clock = null, Action<string>? console = null)
    {
        _pluginId = pluginId;
        _filePath = filePath;
        _clock    = clock ?? SystemPluginClock.Instance;
        _console  = console ?? Console.WriteLine;
    }

    /// <summary>Stops writing to the file (before the plugin folder is deleted or replaced).</summary>
    public void DetachFile()
    {
        lock (_lock) _filePath = null;
    }

    /// <summary>Re-attaches the rolling file.</summary>
    public void AttachFile(string path)
    {
        lock (_lock) _filePath = path;
    }

    /// <summary>Total lines ever appended (absolute count).</summary>
    public int TotalCount { get { lock (_lock) return _baseIndex + _ring.Count; } }

    /// <summary>Appends one line with a timestamp and level (<c>debug|info|warn|error|stdout|stderr</c>).</summary>
    public void Append(string level, string message)
    {
        // A multi-line message becomes one entry per line so the viewer stays tidy.
        foreach (var raw in message.Replace("\r\n", "\n").Split('\n'))
        {
            var text = raw.TrimEnd('\r');
            if (text.Length == 0 && message.Length > 0 && message.Contains('\n')) continue;
            string stamp = DateTimeOffset.FromUnixTimeMilliseconds(_clock.UnixMs).ToLocalTime()
                .ToString("yyyy-MM-dd HH:mm:ss.fff", CultureInfo.InvariantCulture);
            string line = $"{stamp} [{level}] {text}";
            lock (_lock)
            {
                _ring.Enqueue(line);
                while (_ring.Count > RingCapacity) { _ring.Dequeue(); _baseIndex++; }
                WriteFileLocked(line);
                MirrorLocked($"[{level}] {text}");
            }
        }
    }

    /// <summary>Half-open [start, end) slice by absolute index, same semantics as <c>GetProgramLogs</c>.</summary>
    public (int Total, int Start, List<string> Logs) Get(int? start, int? end)
    {
        lock (_lock)
        {
            int total = _baseIndex + _ring.Count;
            int s = Math.Clamp(start ?? 0, _baseIndex, total);
            int e = Math.Clamp(end ?? total, s, total);
            var logs = _ring.Skip(s - _baseIndex).Take(e - s).ToList();
            return (total, s, logs);
        }
    }

    /// <summary>The last <paramref name="count"/> lines.</summary>
    public List<string> Tail(int count)
    {
        lock (_lock) return _ring.Skip(Math.Max(0, _ring.Count - count)).ToList();
    }

    /// <summary>Empties the ring buffer (absolute indices keep counting). The file is kept.</summary>
    public void Clear()
    {
        lock (_lock)
        {
            _baseIndex += _ring.Count;
            _ring.Clear();
        }
    }

    private void MirrorLocked(string text)
    {
        long now = _clock.NowMs;
        if (now - _windowStartMs >= 1000)
        {
            if (_suppressed > 0)
                _console($"[Plugin:{_pluginId}] ({_suppressed} more line(s) suppressed; see plugin.log)");
            _windowStartMs = now;
            _windowCount   = 0;
            _suppressed    = 0;
        }
        if (_windowCount < ConsoleLinesPerSec)
        {
            _windowCount++;
            _console($"[Plugin:{_pluginId}] {text}");
        }
        else _suppressed++;
    }

    private void WriteFileLocked(string line)
    {
        if (_filePath is null) return;
        try
        {
            var info = new FileInfo(_filePath);
            if (info.Exists && info.Length >= MaxFileBytes) RollLocked(_filePath);
            File.AppendAllText(_filePath, line + Environment.NewLine);
        }
        catch (IOException) { /* folder being replaced or disk full: the ring buffer still has it */ }
        catch (UnauthorizedAccessException) { }
    }

    /// <summary>plugin.log → plugin.log.1 → plugin.log.2 (the oldest is dropped).</summary>
    private static void RollLocked(string path)
    {
        for (int i = FileCount - 1; i >= 1; i--)
        {
            string src = i == 1 ? path : $"{path}.{i - 1}";
            string dst = $"{path}.{i}";
            if (File.Exists(src)) File.Move(src, dst, overwrite: true);
        }
    }
}
