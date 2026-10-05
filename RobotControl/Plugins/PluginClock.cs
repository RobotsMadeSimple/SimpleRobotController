namespace Controller.RobotControl.Plugins;

/// <summary>
/// Time source and one-shot timers for the plugin host (ready timeouts, restart backoff,
/// event poll intervals). Injected so tests can drive time deterministically.
/// </summary>
public interface IPluginClock
{
    /// <summary>Monotonic milliseconds.</summary>
    long NowMs { get; }

    /// <summary>Wall-clock Unix milliseconds (for <c>startedUnixMs</c> and log stamps).</summary>
    long UnixMs { get; }

    /// <summary>Runs <paramref name="callback"/> once after <paramref name="delayMs"/>. Dispose to cancel.</summary>
    IDisposable Schedule(int delayMs, Action callback);
}

/// <summary>The real clock: <see cref="Environment.TickCount64"/> and <see cref="System.Threading.Timer"/>.</summary>
public sealed class SystemPluginClock : IPluginClock
{
    public static readonly SystemPluginClock Instance = new();

    public long NowMs  => Environment.TickCount64;
    public long UnixMs => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();

    public IDisposable Schedule(int delayMs, Action callback)
    {
        var timer = new Timer(_ =>
        {
            try { callback(); }
            catch (Exception ex) { Console.WriteLine($"[Plugins] Timer callback failed: {ex}"); }
        });
        timer.Change(Math.Max(0, delayMs), Timeout.Infinite);
        return timer;
    }
}
