using System.Collections.Concurrent;
using System.Text.Json;

namespace Controller.RobotControl.Plugins;

/// <summary>Plugin lifecycle states (docs/plugins.md §3). Serialised camelCase (<c>running</c>).</summary>
public enum PluginState { Disabled, Stopped, Installing, Starting, Running, Degraded, Crashed, Error }

/// <summary><c>PluginSummary</c> (docs/plugins.md §7).</summary>
public sealed record PluginSummary(
    string Id, string Name, string? Version, string? Description, string? Author, string? Runtime,
    PluginState State, string? Message, bool Enabled, bool Connected, int? Pid, long? StartedUnixMs,
    int RestartCount, int StepCount, int FunctionCount, int PropertyCount, bool HasConfig,
    string StatusState, string? StatusMessage);

/// <summary><c>PluginDetail</c> = summary + manifest, config, token (external only), live properties, log tail.</summary>
public sealed record PluginDetail(
    string Id, string Name, string? Version, string? Description, string? Author, string? Runtime,
    PluginState State, string? Message, bool Enabled, bool Connected, int? Pid, long? StartedUnixMs,
    int RestartCount, int StepCount, int FunctionCount, int PropertyCount, bool HasConfig,
    string StatusState, string? StatusMessage,
    PluginManifest? Manifest, Dictionary<string, JsonElement> Config, string? Token,
    Dictionary<string, double> Properties, List<string> LogTail, IReadOnlyList<ManifestProblem> Problems);

/// <summary>
/// One installed plugin: manifest, persisted config, process, live session and the
/// state machine of docs/plugins.md §3 — launch, ready timeout, restart policy with
/// exponential backoff and a crash budget (<c>maxRestarts</c> per 10 minutes), stop
/// (shutdown request → grace → kill tree). Also the inbound handler for its session.
/// Every state change happens under one lock; process/timer/session callbacks carry a
/// launch generation so stale ones are ignored.
/// </summary>
public sealed class PluginHost : IPluginSessionHandler
{
    /// <summary>Window in which <c>restart.maxRestarts</c> is counted.</summary>
    public const int CrashWindowMs = 10 * 60 * 1000;
    /// <summary>Backoff cap.</summary>
    public const int MaxBackoffMs = 30_000;
    /// <summary>A plugin that stayed ready this long resets its consecutive-crash backoff.</summary>
    public const int StableRunMs = 60_000;

    private readonly PluginManager _manager;
    private readonly object _lock = new();
    private readonly Queue<long> _crashTimes = new();
    private int _consecutiveCrashes;
    private long? _readyAtMs;
    private int _gen;
    private IPluginProcess? _process;
    private PluginSession? _session;
    private IDisposable? _readyTimer;
    private IDisposable? _backoffTimer;
    private PluginConfigFile _config;

    internal PluginHost(PluginManager manager, string id, string folder, PluginManifestLoadResult load, PluginLog? log = null)
    {
        _manager = manager;
        Id       = id;
        Folder   = folder;
        Log      = log ?? new PluginLog(id, Path.Combine(folder, "plugin.log"), manager.Clock, manager.ConsoleSink);
        _config  = manager.ConfigStore.LoadOrCreate(id);
        ApplyLoad(load);
        State = Problems.Count > 0 ? PluginState.Error : _config.Enabled ? PluginState.Stopped : PluginState.Disabled;
        if (Problems.Count > 0)
            Log.Append("error", "Manifest invalid: " + Message);
    }

    // ── identity / manifest ──────────────────────────────────────────────────

    public string Id { get; }
    public string Folder { get; }
    public PluginLog Log { get; }

    /// <summary>The manifest file as loaded (null when unreadable).</summary>
    public PluginManifest? FileManifest { get; private set; }

    /// <summary>The manifest in effect: the file's, with the <c>plugin.ready</c> override applied while connected.</summary>
    public PluginManifest? Manifest { get; private set; }

    public IReadOnlyList<ManifestProblem> Problems { get; private set; } = Array.Empty<ManifestProblem>();

    public string Name => Manifest?.DisplayName ?? Id;

    // ── live state ───────────────────────────────────────────────────────────

    public PluginState State { get; private set; }
    public string? Message { get; private set; }
    public int? Pid { get; private set; }
    public long? StartedUnixMs { get; private set; }
    /// <summary>Automatic restarts since the last manual start.</summary>
    public int RestartCount { get; private set; }
    public int? LastExitCode { get; private set; }
    /// <summary>Socket up and <c>plugin.ready</c> accepted.</summary>
    public bool Connected { get; private set; }
    /// <summary><c>ok</c> / <c>degraded</c> / <c>error</c> as last reported by the plugin.</summary>
    public string StatusState { get; private set; } = "ok";
    public string? StatusMessage { get; private set; }

    /// <summary>Live property values (lock-free reads). Cleared on disconnect.</summary>
    public ConcurrentDictionary<string, double> Properties { get; } = new(StringComparer.OrdinalIgnoreCase);

    public bool Enabled { get { lock (_lock) return _config.Enabled; } }
    public string Token { get { lock (_lock) return _config.Token; } }

    /// <summary>Connected and ready (running or degraded).</summary>
    public bool IsRunning { get { lock (_lock) return Connected && State is PluginState.Running or PluginState.Degraded; } }

    /// <summary>Installing, starting, running, degraded, or waiting out a restart backoff.</summary>
    public bool IsActive
    {
        get
        {
            lock (_lock)
                return State is PluginState.Installing or PluginState.Starting or PluginState.Running or PluginState.Degraded
                       || _backoffTimer != null;
        }
    }

    internal PluginSession? Session { get { lock (_lock) return _session; } }

    /// <summary>The stored config with schema defaults merged in.</summary>
    public Dictionary<string, JsonElement> GetConfig()
    {
        lock (_lock) return PluginConfigValidator.MergeDefaults(FileManifest?.ConfigSchema ?? new(), _config.Config);
    }

    // ── manifest (re)load ───────────────────────────────────────────────────

    private void ApplyLoad(PluginManifestLoadResult load)
    {
        FileManifest = load.Manifest;
        Manifest     = load.Manifest;
        Problems     = load.Problems;
        Message      = load.Problems.Count > 0 ? string.Join("; ", load.Problems.Select(p => p.ToString())) : null;
    }

    /// <summary>Replaces the manifest of a plugin that is not active (ReloadPlugins).</summary>
    internal void Reload(PluginManifestLoadResult load)
    {
        lock (_lock)
        {
            if (IsActiveLocked()) return;
            var wasError = State == PluginState.Error;
            _config = _manager.ConfigStore.LoadOrCreate(Id);
            ApplyLoad(load);
            if (Problems.Count > 0) { State = PluginState.Error; return; }
            if (wasError || State is PluginState.Stopped or PluginState.Disabled)
                State = _config.Enabled ? PluginState.Stopped : PluginState.Disabled;
        }
    }

    private bool IsActiveLocked() =>
        State is PluginState.Installing or PluginState.Starting or PluginState.Running or PluginState.Degraded || _backoffTimer != null;

    // ── start / stop ─────────────────────────────────────────────────────────

    /// <summary>
    /// Starts the plugin. A manual start resets the crash budget (and gets a plugin out of
    /// <c>crashed</c>). Returns an error code when it cannot start (<c>pluginDisabled</c>,
    /// <c>pluginError</c>), else null. Returns immediately; state follows.
    /// </summary>
    public string? Start(bool manual = true)
    {
        lock (_lock)
        {
            if (Problems.Count > 0) { State = PluginState.Error; return "pluginError"; }
            if (!_config.Enabled) { State = PluginState.Disabled; return "pluginDisabled"; }
            if (State is PluginState.Installing or PluginState.Starting or PluginState.Running or PluginState.Degraded)
                return null;
            CancelTimersLocked();
            if (manual)
            {
                _crashTimes.Clear();
                _consecutiveCrashes = 0;
                RestartCount = 0;
            }
            BeginLaunchLocked();
            return null;
        }
    }

    /// <summary>
    /// Stops the plugin: <c>shutdown</c> request, wait up to the grace period for the process
    /// to exit, then kill the tree. Ends in <paramref name="finalState"/> (Stopped or Disabled).
    /// </summary>
    public async Task StopAsync(string reason = "stopped", PluginState finalState = PluginState.Stopped)
    {
        PluginSession? session;
        IPluginProcess? process;
        int gen;
        bool wasConnected;
        lock (_lock)
        {
            gen = ++_gen;
            CancelTimersLocked();
            session      = _session;
            process      = _process;
            wasConnected = Connected;
            _session     = null;
            _process     = null;
            Connected    = false;
            Properties.Clear();
            if (State != PluginState.Error) Message = process != null || session != null ? "stopping" : null;
        }
        if (wasConnected) _manager.OnPluginDisconnected(this);

        if (session != null)
        {
            try { await session.RequestAsync("shutdown", new { reason }, Math.Min(1000, _manager.StopGraceMs)); }
            catch { /* not answering: the kill below handles it */ }
        }
        if (process != null)
        {
            var exited = await Task.WhenAny(process.Exited, Task.Delay(_manager.StopGraceMs)) == process.Exited;
            if (!exited)
            {
                Log.Append("warn", "Did not exit after shutdown — killing");
                await Task.Run(process.Kill);
                await Task.WhenAny(process.Exited, Task.Delay(2000));
            }
            if (process.Exited.IsCompleted) LastExitCode = process.Exited.Result;
        }
        if (session != null && !session.IsClosed)
            await session.CloseAsync(1000, reason);

        lock (_lock)
        {
            if (gen != _gen) return; // started again meanwhile
            Pid = null;
            StatusState = "ok"; StatusMessage = null;
            if (State == PluginState.Error) return;
            State   = finalState;
            Message = null;
        }
        Log.Append("info", $"Stopped ({reason})");
    }

    /// <summary>Stop then start (manual).</summary>
    public async Task<string?> RestartAsync()
    {
        await StopAsync("restart");
        return Start(manual: true);
    }

    /// <summary>Persists the enabled flag. Disabling stops the plugin; enabling starts it when <c>autoStart</c>.</summary>
    public async Task SetEnabledAsync(bool enabled)
    {
        lock (_lock)
        {
            _config.Enabled = enabled;
            _manager.ConfigStore.Save(Id, _config);
        }
        if (!enabled)
        {
            await StopAsync("disabled", PluginState.Disabled);
            return;
        }
        lock (_lock)
        {
            if (State == PluginState.Disabled) State = PluginState.Stopped;
        }
        if (Manifest?.AutoStart != false && _manager.IsStarted) Start(manual: true);
    }

    /// <summary>Validates, persists and (when connected) pushes <c>config.changed</c>. Returns (field, message) on failure.</summary>
    public (string Field, string Message)? SetConfig(Dictionary<string, JsonElement> config)
    {
        Dictionary<string, JsonElement> merged;
        PluginSession? session;
        lock (_lock)
        {
            var schema = FileManifest?.ConfigSchema ?? new();
            if (PluginConfigValidator.Validate(schema, config) is { } problem) return problem;
            _config.Config = config.Where(kv => kv.Value.ValueKind != JsonValueKind.Null)
                                   .ToDictionary(kv => kv.Key, kv => kv.Value.Clone(), StringComparer.Ordinal);
            _manager.ConfigStore.Save(Id, _config);
            merged  = PluginConfigValidator.MergeDefaults(schema, _config.Config);
            session = Connected ? _session : null;
        }
        Log.Append("info", "Config changed");
        session?.Notify("config.changed", new { config = merged });
        return null;
    }

    /// <summary>New random token, persisted. An external plugin's live connection is closed (it must reconnect).</summary>
    public void RotateToken()
    {
        PluginSession? toClose = null;
        lock (_lock)
        {
            _config.Token = PluginConfigStore.NewToken();
            _manager.ConfigStore.Save(Id, _config);
            if (Manifest?.IsExternal == true) toClose = _session;
        }
        Log.Append("info", "Connect token rotated");
        if (toClose != null) _ = toClose.CloseAsync(4401, "tokenRotated");
    }

    private void BeginLaunchLocked()
    {
        int gen = ++_gen;
        StatusState = "ok"; StatusMessage = null;
        LastExitCode = null;
        if (Manifest!.IsExternal)
        {
            State   = PluginState.Starting;
            Message = "waiting for connection";
            return;
        }
        State   = PluginState.Starting;
        Message = null;
        _ = LaunchAsync(gen);
    }

    private async Task LaunchAsync(int gen)
    {
        PluginLaunchContext ctx;
        lock (_lock)
        {
            ctx = new PluginLaunchContext
            {
                Manifest     = Manifest!,
                PluginDir    = Path.GetFullPath(Folder),
                Environment  = BuildEnvironmentLocked(),
                Log          = Log,
                OnInstalling = () => { lock (_lock) if (gen == _gen) { State = PluginState.Installing; Message = "installing requirements"; } },
            };
        }

        IPluginProcess process;
        try
        {
            process = await _manager.Launcher.LaunchAsync(ctx, CancellationToken.None);
        }
        catch (PluginLaunchException ex)
        {
            Log.Append("error", $"{ex.Code}: {ex.Message}");
            lock (_lock)
            {
                if (gen != _gen) return;
                State   = PluginState.Error;
                Message = $"{ex.Code}: {ex.Message}";
            }
            return;
        }
        catch (Exception ex)
        {
            Log.Append("error", $"Launch failed: {ex.Message}");
            lock (_lock)
            {
                if (gen != _gen) return;
                HandleFailureLocked($"launch failed: {ex.Message}", failure: true);
            }
            return;
        }

        lock (_lock)
        {
            if (gen != _gen) { _ = Task.Run(process.Kill); return; } // stopped while launching
            _process      = process;
            Pid           = process.Pid;
            StartedUnixMs = _manager.Clock.UnixMs;
            State         = PluginState.Starting;
            Message       = null;
            int timeout   = Manifest!.ReadyTimeoutMs > 0 ? Manifest.ReadyTimeoutMs : PluginManifest.DefaultReadyTimeoutMs;
            _readyTimer   = _manager.Clock.Schedule(timeout, () => OnReadyTimeout(gen));
        }
        _ = process.Exited.ContinueWith(t => OnProcessExited(gen, t.Result), CancellationToken.None,
                                         TaskContinuationOptions.ExecuteSynchronously, TaskScheduler.Default);
    }

    internal Dictionary<string, string> BuildEnvironmentLocked() => new()
    {
        ["SRC_PLUGIN_ID"]          = Id,
        ["SRC_PLUGIN_URL"]         = $"ws://127.0.0.1:{_manager.Port}/plugin",
        ["SRC_PLUGIN_TOKEN"]       = _config.Token,
        ["SRC_PLUGIN_DIR"]         = Path.GetFullPath(Folder),
        ["SRC_DATA_DIR"]           = _manager.DataDir,
        ["SRC_CONTROLLER_VERSION"] = _manager.ControllerVersion,
    };

    private void OnReadyTimeout(int gen)
    {
        lock (_lock)
        {
            if (gen != _gen || Connected || State is not (PluginState.Starting or PluginState.Installing)) return;
            Log.Append("error", "plugin.ready did not arrive in time — killing");
            HandleFailureLocked("ready timeout", failure: true);
        }
    }

    private void OnProcessExited(int gen, int exitCode)
    {
        lock (_lock)
        {
            if (gen != _gen) return;
            LastExitCode = exitCode;
            Log.Append(exitCode == 0 ? "info" : "error", $"Process exited with code {exitCode}");
            HandleFailureLocked($"exited with code {exitCode}", failure: exitCode != 0);
        }
    }

    private void OnSessionClosed(PluginSession session)
    {
        bool notify;
        lock (_lock)
        {
            if (!ReferenceEquals(session, _session)) return; // replaced or stopped
            _session  = null;
            notify    = Connected;
            Connected = false;
            Properties.Clear();
            Manifest  = FileManifest;
            Log.Append("warn", $"Connection closed ({session.CloseReason ?? "disconnected"})");
            if (Manifest?.IsExternal == true)
            {
                if (State is PluginState.Running or PluginState.Degraded)
                {
                    State   = PluginState.Starting;
                    Message = "waiting for connection";
                }
            }
            else if (State is PluginState.Running or PluginState.Degraded or PluginState.Starting)
            {
                HandleFailureLocked("lost connection", failure: true);
            }
        }
        if (notify) _manager.OnPluginDisconnected(this);
    }

    /// <summary>
    /// A launch ended badly (exit, lost socket, ready timeout). Kills what is left, then
    /// either restarts after a backoff, gives up (<c>crashed</c>), or stops — per <c>restart.mode</c>.
    /// </summary>
    private void HandleFailureLocked(string reason, bool failure)
    {
        int gen = ++_gen; // ignore any further callbacks of the failed launch
        CancelTimersLocked();
        var process = _process;
        var session = _session;
        bool wasConnected = Connected;
        _process  = null;
        _session  = null;
        Connected = false;
        Pid       = null;
        Properties.Clear();
        Manifest  = FileManifest;
        if (process != null && !process.Exited.IsCompleted) _ = Task.Run(process.Kill);
        if (session != null) _ = session.CloseAsync(1011, reason);
        if (wasConnected) _ = Task.Run(() => _manager.OnPluginDisconnected(this));

        var policy  = Manifest?.Restart ?? new PluginRestartPolicy();
        bool restart = policy.Mode == "always" || (policy.Mode == "onFailure" && failure);
        if (!restart)
        {
            Message = reason;
            State   = PluginState.Stopped;
            return;
        }

        long now = _manager.Clock.NowMs;
        while (_crashTimes.Count > 0 && now - _crashTimes.Peek() > CrashWindowMs) _crashTimes.Dequeue();
        if (_crashTimes.Count >= policy.MaxRestarts)
        {
            Message = $"{reason}; gave up after {_crashTimes.Count} restart(s) in 10 minutes";
            State   = PluginState.Crashed;
            Log.Append("error", Message);
            return;
        }

        if (_readyAtMs is { } readyAt && now - readyAt >= StableRunMs) _consecutiveCrashes = 0;
        _readyAtMs = null;
        _consecutiveCrashes++;
        long delay = Math.Min((long)Math.Max(0, policy.BackoffMs) << Math.Min(_consecutiveCrashes - 1, 20), MaxBackoffMs);
        _crashTimes.Enqueue(now);
        Message = $"{reason}; restarting in {delay} ms";
        State   = PluginState.Crashed;
        Log.Append("warn", Message);
        _backoffTimer = _manager.Clock.Schedule((int)delay, () => OnBackoffElapsed(gen));
    }

    private void OnBackoffElapsed(int gen)
    {
        lock (_lock)
        {
            if (gen != _gen || State != PluginState.Crashed) return;
            _backoffTimer?.Dispose();
            _backoffTimer = null;
            RestartCount++;
            BeginLaunchLocked();
        }
    }

    private void CancelTimersLocked()
    {
        _readyTimer?.Dispose();   _readyTimer = null;
        _backoffTimer?.Dispose(); _backoffTimer = null;
    }

    // ── connection (plugin.ready) ────────────────────────────────────────────

    /// <summary>
    /// Accepts <paramref name="session"/> as this plugin's connection after a valid
    /// <c>plugin.ready</c>. Returns the ready result, or null when the plugin is not in a state
    /// that accepts a connection (the caller closes with 4401). An older connection is closed with 4409.
    /// </summary>
    internal object? AttachSession(PluginSession session, JsonElement readyParams)
    {
        PluginSession? old;
        lock (_lock)
        {
            if (State is not (PluginState.Starting or PluginState.Running or PluginState.Degraded)) return null;
            old       = _session;
            _session  = null; // so the old session's close is not treated as a crash
            _readyTimer?.Dispose(); _readyTimer = null;

            Manifest = FileManifest;
            if (readyParams.ValueKind == JsonValueKind.Object && readyParams.TryGetProperty("manifest", out var over)
                && over.ValueKind == JsonValueKind.Object && FileManifest != null)
            {
                try
                {
                    var candidate = FileManifest.WithOverride(over);
                    var problems  = candidate.Validate(_manager.IsFunctionName);
                    if (problems.Count == 0) Manifest = candidate;
                    else Log.Append("warn", "Ignored the plugin.ready manifest override: " + string.Join("; ", problems));
                }
                catch (JsonException ex) { Log.Append("warn", "Ignored the plugin.ready manifest override: " + ex.Message); }
            }

            session.Handler = this;
            session.Closed += OnSessionClosed;
            _session    = session;
            Connected   = true;
            State       = StatusState == "ok" ? PluginState.Running : PluginState.Degraded;
            Message     = null;
            _readyAtMs  = _manager.Clock.NowMs;
            StartedUnixMs ??= _manager.Clock.UnixMs;
            Properties.Clear();
        }
        if (old != null) _ = old.CloseAsync(4409, "replaced");

        string sdk = readyParams.ValueKind == JsonValueKind.Object && readyParams.TryGetProperty("sdk", out var s) && s.ValueKind == JsonValueKind.Object
            ? $" (sdk {Str(s, "name")} {Str(s, "version")})" : "";
        Log.Append("info", $"Connected{sdk}");
        _manager.OnPluginConnected(this);
        return ReadyResult();
    }

    private object ReadyResult() => new
    {
        controllerVersion = _manager.ControllerVersion,
        pluginId          = Id,
        config            = GetConfig(),
        dataDir           = _manager.DataDir,
    };

    // ── inbound requests / events ────────────────────────────────────────────

    async Task<object?> IPluginSessionHandler.HandleRequestAsync(PluginSession session, string method, JsonElement p)
    {
        switch (method)
        {
            case "plugin.ready":
                return ReadyResult();

            case "controller.command":
            {
                string command = Str(p, "command") ?? throw new PluginProtocolException("badParams", "command is required");
                JsonElement? cmdParams = p.TryGetProperty("params", out var cp) && cp.ValueKind != JsonValueKind.Null ? cp.Clone() : null;
                return await _manager.ExecuteCommandAsync(Id, command, cmdParams);
            }

            case "events.subscribe":
            {
                var patterns = new List<string>();
                if (p.TryGetProperty("events", out var ev) && ev.ValueKind == JsonValueKind.Array)
                    foreach (var e in ev.EnumerateArray())
                        if (e.ValueKind == JsonValueKind.String && !string.IsNullOrWhiteSpace(e.GetString()))
                            patterns.Add(e.GetString()!.Trim());
                var sub = new PluginSubscription(patterns,
                    Interval(p, "positionIntervalMs", PluginSubscription.DefaultPositionMs, PluginSubscription.MinPositionMs),
                    Interval(p, "statusIntervalMs",   PluginSubscription.DefaultStatusMs,   PluginSubscription.MinStatusMs),
                    Interval(p, "ioIntervalMs",       PluginSubscription.DefaultIoMs,       PluginSubscription.MinIoMs));
                session.LastIo = null;
                session.NextIoDueMs = session.NextPositionDueMs = session.NextStatusDueMs = 0;
                session.Subscription = sub;
                _manager.OnSubscriptionChanged();
                return new { subscribed = patterns };
            }

            case "events.unsubscribe":
                session.Subscription = PluginSubscription.None;
                session.LastIo = null;
                _manager.OnSubscriptionChanged();
                return null;

            case "variables.get":
            {
                var getter = _manager.VariablesGetter ?? throw new PluginProtocolException("unsupported", "variables.get is not available");
                string? program = Str(p, "programName");
                var snapshot = getter(string.IsNullOrEmpty(program) ? null : program)
                    ?? throw new PluginProtocolException("unknownProgram", $"Program '{program}' is not running");
                return snapshot;
            }

            case "variables.set":
            {
                var setter = _manager.VariablesSetter ?? throw new PluginProtocolException("unsupported", "variables.set is not available");
                if (!p.TryGetProperty("values", out var values) || values.ValueKind != JsonValueKind.Object)
                    throw new PluginProtocolException("badParams", "values must be an object");
                var dict = values.EnumerateObject().ToDictionary(v => v.Name, v => v.Value.Clone(), StringComparer.Ordinal);
                string? program = Str(p, "programName");
                if (setter(string.IsNullOrEmpty(program) ? null : program, dict) is { } err)
                    throw new PluginProtocolException(err, err);
                return null;
            }

            case "config.get":
                return new { config = GetConfig() };

            case "plugin.state":
                lock (_lock) return new { state = State, enabled = _config.Enabled, restartCount = RestartCount };

            default:
                throw new PluginProtocolException("unknownMethod", $"Unknown method '{method}'");
        }
    }

    void IPluginSessionHandler.HandleEvent(PluginSession session, string name, JsonElement data)
    {
        if (!ReferenceEquals(session, Session)) return;
        switch (name)
        {
            case "properties.set":
                if (data.TryGetProperty("values", out var values) && values.ValueKind == JsonValueKind.Object)
                    foreach (var v in values.EnumerateObject())
                    {
                        switch (v.Value.ValueKind)
                        {
                            case JsonValueKind.Number when v.Value.TryGetDouble(out var d) && double.IsFinite(d):
                                Properties[v.Name] = d; break;
                            case JsonValueKind.True:  Properties[v.Name] = 1; break;
                            case JsonValueKind.False: Properties[v.Name] = 0; break;
                            // anything else is ignored (contract: non-numeric values are ignored)
                        }
                    }
                break;

            case "properties.clear":
                Properties.Clear();
                break;

            case "log":
            {
                string level = Str(data, "level") is { } l && l is "debug" or "info" or "warn" or "error" ? l : "info";
                Log.Append(level, Str(data, "message") ?? "");
                break;
            }

            case "status":
            {
                string state = Str(data, "state") ?? "ok";
                if (state is not ("ok" or "degraded" or "error")) state = "ok";
                lock (_lock)
                {
                    StatusState   = state;
                    StatusMessage = state == "ok" ? null : Str(data, "message");
                    if (Connected)
                        State = state == "ok" ? PluginState.Running : PluginState.Degraded;
                }
                if (state != "ok") Log.Append("warn", $"Status {state}: {StatusMessage}");
                break;
            }

            case "step.progress":
            {
                string? inv = Str(data, "invocationId");
                if (string.IsNullOrEmpty(inv)) break;
                double? percent = data.TryGetProperty("percent", out var pc) && pc.ValueKind == JsonValueKind.Number ? pc.GetDouble() : null;
                _manager.OnStepProgress(inv, Str(data, "message"), percent);
                break;
            }

            default:
                Log.Append("debug", $"Ignored unknown event '{name}'");
                break;
        }
    }

    // ── summaries ────────────────────────────────────────────────────────────

    public PluginSummary ToSummary()
    {
        lock (_lock)
        {
            var m = Manifest;
            return new PluginSummary(Id, Name, m?.Version, m?.Description, m?.Author, m?.Runtime,
                State, Message, _config.Enabled, Connected, Pid, StartedUnixMs, RestartCount,
                m?.Steps?.Count ?? 0, m?.Functions?.Count ?? 0, m?.Properties?.Count ?? 0,
                (FileManifest?.ConfigSchema?.Count ?? 0) > 0, StatusState, StatusMessage);
        }
    }

    public PluginDetail ToDetail()
    {
        var s = ToSummary();
        lock (_lock)
        {
            return new PluginDetail(s.Id, s.Name, s.Version, s.Description, s.Author, s.Runtime, s.State, s.Message,
                s.Enabled, s.Connected, s.Pid, s.StartedUnixMs, s.RestartCount, s.StepCount, s.FunctionCount,
                s.PropertyCount, s.HasConfig, s.StatusState, s.StatusMessage,
                Manifest, PluginConfigValidator.MergeDefaults(FileManifest?.ConfigSchema ?? new(), _config.Config),
                Manifest?.IsExternal == true ? _config.Token : null,
                new Dictionary<string, double>(Properties, StringComparer.OrdinalIgnoreCase),
                Log.Tail(50), Problems);
        }
    }

    // ── helpers ──────────────────────────────────────────────────────────────

    private static string? Str(JsonElement obj, string name) =>
        obj.ValueKind == JsonValueKind.Object && obj.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() : null;

    private static int Interval(JsonElement p, string name, int def, int min)
    {
        if (p.ValueKind != JsonValueKind.Object || !p.TryGetProperty(name, out var v) || v.ValueKind != JsonValueKind.Number) return def;
        return Math.Max(min, (int)Math.Min(int.MaxValue, v.GetDouble()));
    }
}
