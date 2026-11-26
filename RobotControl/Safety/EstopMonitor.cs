namespace Controller.RobotControl.Safety;

/// <summary>
/// The e-stop state machine (docs/estop.md), independent of the controller so it can be tested on
/// its own. Fed one <see cref="EstopReading"/> per tick by the safety monitor thread.
/// </summary>
/// <remarks>
/// <list type="bullet">
/// <item><b>Active</b>: monitoring is enabled and the input is tripped right now (circuit open for
/// normally-closed, closed for normally-open), or the source is offline (fail-safe).</item>
/// <item><b>Latched</b>: a trip happened and has not been reset. Survives the input going healthy
/// again; only <see cref="TryReset"/> clears it, and only once the input reads healthy.</item>
/// <item><b>Armed</b>: a healthy reading has been seen since boot, enabling or the last trip. A trip
/// latches only while armed, so a controller that boots with the button already pressed (or before
/// its source device has connected) blocks motion without latching — nothing was running.</item>
/// </list>
/// Thread-safe: every member takes the same lock.
/// </remarks>
public sealed class EstopMonitor
{
    private readonly object _lock = new();
    private bool    _active;
    private bool    _latched;
    private bool    _armed;
    private bool    _enabled;
    private bool    _sourceOnline;
    private bool?   _raw;
    private string  _label = "";
    private string? _reason;
    private long    _trippedUnixMs;

    /// <summary>A consistent copy of the state, for status and commands.</summary>
    public readonly record struct Snapshot(
        bool Enabled, bool Active, bool Latched, bool SourceOnline, bool? Raw, bool? Healthy,
        string Label, string? Reason, long TrippedUnixMs);

    public Snapshot State
    {
        get
        {
            lock (_lock)
            {
                bool? healthy = !_enabled || !_sourceOnline || _raw is null ? null : !_active;
                return new Snapshot(_enabled, _active, _latched, _sourceOnline, _raw, healthy, _label, _reason, _trippedUnixMs);
            }
        }
    }

    /// <summary>True while motion must stay blocked: active or latched.</summary>
    public bool Inhibits { get { lock (_lock) return _active || _latched; } }

    /// <summary>Whether <paramref name="reading"/> is an e-stop under <paramref name="config"/>.</summary>
    public static bool IsTripped(EstopConfig config, EstopReading reading) =>
        !reading.Available || reading.Raw is null || (config.NormallyClosed ? reading.Raw != true : reading.Raw == true);

    /// <summary>
    /// Applies one reading. Returns true exactly when this reading is a new trip — the caller then runs
    /// the e-stop reaction (stop, abort, unhome, event, alarm) once.
    /// </summary>
    public bool Evaluate(EstopConfig config, EstopReading reading, long nowUnixMs)
    {
        lock (_lock)
        {
            _sourceOnline = reading.Available;
            _raw          = reading.Available ? reading.Raw : null;
            _label        = reading.Label;

            if (!config.Enabled)
            {
                // Not monitored: nothing is active, and the next enable starts unarmed. A latch from
                // before stays until it is reset (which then succeeds — there is no input to check).
                _enabled = false;
                _active  = false;
                _armed   = false;
                return false;
            }

            _enabled = true;
            bool tripped = IsTripped(config, reading);
            _active = tripped;
            if (!tripped)
            {
                _armed = true;
                if (!_latched) _reason = null;
                return false;
            }

            if (!_armed)
            {
                // Active without a latch: still say why (a latched trip keeps its original reason).
                if (!_latched) _reason = Describe(config, reading);
                return false;
            }
            _armed         = false;
            _latched       = true;
            _trippedUnixMs = nowUnixMs;
            _reason        = Describe(config, reading);
            return true;
        }
    }

    /// <summary>
    /// The operator's reset. Clears the latch when the input reads healthy (or monitoring is off);
    /// otherwise returns the reason it cannot. Null on success.
    /// </summary>
    public string? TryReset(EstopConfig config, EstopReading reading)
    {
        lock (_lock)
        {
            if (config.Enabled && IsTripped(config, reading))
            {
                _active = true;
                return reading.Available
                    ? $"The e-stop input ({reading.Label}) still reads {(config.NormallyClosed ? "open" : "closed")}. Release the e-stop button, then reset."
                    : $"The e-stop source ({reading.Label}) is offline. Reconnect it, then reset.";
            }
            _latched = false;
            _active  = false;
            _reason  = null;
            // A healthy reading arms the next trip straight away.
            if (config.Enabled) _armed = true;
            return null;
        }
    }

    private static string Describe(EstopConfig config, EstopReading reading) =>
        !reading.Available || reading.Raw is null
            ? $"E-stop source offline ({reading.Label})"
            : $"E-stop pressed ({reading.Label} {(config.NormallyClosed ? "open" : "closed")})";
}
