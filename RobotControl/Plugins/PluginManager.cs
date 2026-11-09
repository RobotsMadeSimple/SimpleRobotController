using System.Collections.Concurrent;
using System.IO.Compression;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Plugins;

/// <summary>Dependencies of a <see cref="PluginManager"/> (all optional; tests swap the launcher and clock).</summary>
public sealed class PluginManagerOptions
{
    public IPluginProcessLauncher Launcher { get; init; } = new PluginProcessLauncher();
    public IPluginClock Clock { get; init; } = SystemPluginClock.Instance;
    /// <summary>Runs a WebSocket command exactly like a client (<c>RobotController.AddCommand</c>).</summary>
    public Func<CommandMessage, Task<object?>>? CommandHandler { get; init; }
    /// <summary>The <c>GetStatus</c> payload for the <c>status</c> event.</summary>
    public Func<object?>? StatusProvider { get; init; }
    /// <summary>Robot pose/flags for <c>robot.position</c> and the transition events.</summary>
    public Func<PluginRobotSnapshot?>? RobotProvider { get; init; }
    /// <summary>Fills a dictionary with live IO values (<c>ProgramExecutor.AddIoVariables</c>).</summary>
    public Action<Dictionary<string, double>>? IoProvider { get; init; }
    public string ControllerVersion { get; init; } = RobotController.Version;
    /// <summary>Console sink for the <c>[Plugin:id]</c> mirror and manager messages.</summary>
    public Action<string>? Console { get; init; }
    /// <summary>Start the event poll thread on demand. Tests disable it and call <see cref="PluginManager.PollOnce"/>.</summary>
    public bool RunPollThread { get; init; } = true;
    /// <summary>How long StopPlugin waits for the process to exit after <c>shutdown</c> before killing it.</summary>
    public int StopGraceMs { get; init; } = 3000;
    /// <summary>Existing expression function names (ids may not shadow them).</summary>
    public Func<string, bool>? IsFunctionName { get; init; }
}

/// <summary>
/// Owns every installed plugin (docs/plugins.md §5): discovery, install/uninstall/download,
/// lookup, the plugin WebSocket handshake, the event bus (<see cref="PublishEvent"/> with
/// glob subscriptions and the IO/position/status poll thread), expression functions and
/// properties, and plugin step dispatch.
/// </summary>
public sealed class PluginManager
{
    public const long MaxZipBytes = 200L * 1024 * 1024;
    /// <summary>A connection must send <c>plugin.ready</c> within this time.</summary>
    public const int HandshakeTimeoutMs = 10_000;

    private static readonly JsonSerializerOptions CommandResultJson = new()
    {
        PropertyNameCaseInsensitive = true,
        NumberHandling = JsonNumberHandling.AllowNamedFloatingPointLiterals,
    };

    private readonly PluginManagerOptions _options;
    private readonly ConcurrentDictionary<string, PluginHost> _hosts = new(StringComparer.OrdinalIgnoreCase);
    private readonly ConcurrentDictionary<string, StepInvocation> _invocations = new(StringComparer.Ordinal);
    private readonly SemaphoreSlim _structureLock = new(1, 1); // install / uninstall / reload
    private readonly AutoResetEvent _pollWake = new(false);
    private readonly object _pollLock = new();
    private Thread? _pollThread;
    private PluginRobotSnapshot? _lastRobot;
    private volatile bool _stopping;

    private sealed record StepInvocation(string Id, string PluginId, PluginSession Session, Action<PluginStepReply> OnReply);

    public PluginManager(string dataDir, PluginManagerOptions? options = null)
    {
        _options       = options ?? new PluginManagerOptions();
        DataDir        = Path.GetFullPath(dataDir);
        PluginsDir     = Path.Combine(DataDir, "plugins");
        ConfigStore    = new PluginConfigStore(Path.Combine(DataDir, "pluginConfigs"));
        Properties     = new PluginPropertySource(this);
    }

    // ── configuration ────────────────────────────────────────────────────────

    public string DataDir { get; }
    public string PluginsDir { get; }
    public PluginConfigStore ConfigStore { get; }
    public int Port { get; private set; }
    /// <summary>True after <see cref="StartAll"/> (the web host is listening; plugins may be launched).</summary>
    public bool IsStarted { get; private set; }

    internal IPluginClock Clock => _options.Clock;
    internal IPluginProcessLauncher Launcher => _options.Launcher;
    internal Action<string> ConsoleSink => _options.Console ?? System.Console.WriteLine;
    internal string ControllerVersion => _options.ControllerVersion;
    internal int StopGraceMs => _options.StopGraceMs;
    internal Func<string, bool> IsFunctionName => _options.IsFunctionName ?? ExpressionEvaluator.IsFunctionName;

    /// <summary>Answers <c>variables.get</c> (programName or null for globals). Unset → <c>unsupported</c>; null result → <c>unknownProgram</c>.</summary>
    public Func<string?, VariablesSnapshot?>? VariablesGetter { get; set; }

    /// <summary>Applies <c>variables.set</c>; returns an error code (<c>computedVariable</c>, <c>unknownProgram</c>, …) or null. Unset → <c>unsupported</c>.</summary>
    public Func<string?, Dictionary<string, JsonElement>, string?>? VariablesSetter { get; set; }

    /// <summary>A plugin reported <c>step.progress</c>: (invocationId, message, percent).</summary>
    public event Action<string, string?, double?>? StepProgress;

    // ── lookup ───────────────────────────────────────────────────────────────

    public IReadOnlyList<PluginHost> Plugins => _hosts.Values.OrderBy(h => h.Id, StringComparer.Ordinal).ToList();

    public PluginHost? Get(string id) => id != null && _hosts.TryGetValue(id, out var h) ? h : null;

    /// <summary>IPropertySource over every running plugin's property table.</summary>
    public IPropertySource Properties { get; }

    /// <summary><c>$scale.weight</c> lookup (name without <c>$</c>).</summary>
    public bool TryGetProperty(string dottedName, out double value) => Properties.TryGet(dottedName, out value);

    /// <summary><c>GetStatus.plugins</c>: <c>[{ id, name, state, statusState }]</c>.</summary>
    public List<object> GetStatusSummary() =>
        Plugins.Select(h => (object)new { id = h.Id, name = h.Name, state = StateName(h.State), statusState = h.StatusState }).ToList();

    internal static string StateName(PluginState s) => JsonNamingPolicy.CamelCase.ConvertName(s.ToString());

    // ── boot / shutdown ──────────────────────────────────────────────────────

    /// <summary>Scans <c>plugins/</c> and creates a host per folder. Starts nothing.</summary>
    public void Discover()
    {
        Directory.CreateDirectory(PluginsDir);
        foreach (var dir in PluginFolders())
        {
            string name = Path.GetFileName(dir);
            if (_hosts.ContainsKey(name))
            {
                if (!_hosts[name].Folder.Equals(dir, StringComparison.Ordinal))
                    ConsoleSink($"[Plugins] Skipping '{name}': another plugin already uses that id (ids are case-insensitive)");
                continue;
            }
            _hosts[name] = CreateHost(dir);
        }
        ConsoleSink($"[Plugins] {_hosts.Count} plugin(s) installed in {PluginsDir}");
    }

    /// <summary>
    /// Records the port plugins connect back to and starts every enabled <c>autoStart</c> plugin.
    /// Call once the web host is listening.
    /// </summary>
    public void StartAll(int port)
    {
        Port = port;
        IsStarted = true;
        foreach (var h in Plugins)
            if (h.State == PluginState.Stopped && h.Manifest?.AutoStart != false)
                h.Start(manual: true);
    }

    /// <summary>Stops every plugin (shutdown request, grace, kill). Blocks up to a few seconds.</summary>
    public void StopAll(int timeoutMs = 6000)
    {
        _stopping = true;
        foreach (var inv in _invocations.Values.ToList()) CancelStep(inv.Id, "shutdown");
        var tasks = Plugins.Select(h => h.StopAsync("shutdown")).ToArray();
        try { Task.WaitAll(tasks, timeoutMs); } catch { /* individual failures are logged by the hosts */ }
        _pollWake.Set();
    }

    // ── handshake ────────────────────────────────────────────────────────────

    /// <summary>
    /// Serves one plugin connection: the first frame must be a <c>plugin.ready</c> request with
    /// a valid token (else the connection is closed with 4401); then the session runs until it ends.
    /// </summary>
    public async Task AcceptConnectionAsync(IPluginTransport transport, CancellationToken ct = default)
    {
        string? first;
        using (var handshakeCts = CancellationTokenSource.CreateLinkedTokenSource(ct))
        {
            handshakeCts.CancelAfter(HandshakeTimeoutMs);
            first = await transport.ReceiveAsync(handshakeCts.Token);
        }
        if (first is null) { await transport.CloseAsync(4401, "no plugin.ready"); return; }

        string? id = null, token = null;
        JsonElement readyParams = default;
        try
        {
            using var doc = JsonDocument.Parse(first);
            var root = doc.RootElement;
            if (root.ValueKind == JsonValueKind.Object
                && root.TryGetProperty("t", out var t) && t.GetString() == "req"
                && root.TryGetProperty("method", out var m) && m.GetString() == "plugin.ready"
                && root.TryGetProperty("id", out var idEl)
                && root.TryGetProperty("params", out var p) && p.ValueKind == JsonValueKind.Object
                && p.TryGetProperty("token", out var tok) && tok.ValueKind == JsonValueKind.String)
            {
                id = idEl.ValueKind == JsonValueKind.Number ? idEl.GetRawText() : idEl.GetString();
                token = tok.GetString();
                readyParams = p.Clone();
            }
        }
        catch (JsonException) { }

        var host = token is null ? null : FindByToken(token);
        if (host is null || id is null)
        {
            await transport.CloseAsync(4401, "invalid plugin.ready or token");
            return;
        }

        var session = new PluginSession(transport, host.Id, host.Log);
        var result  = host.AttachSession(session, readyParams);
        if (result is null)
        {
            host.Log.Append("warn", $"Rejected a connection: plugin is {StateName(host.State)}");
            await transport.CloseAsync(4401, "plugin not started");
            return;
        }
        session.Reply(id, result);
        await session.RunAsync(ct);
    }

    private PluginHost? FindByToken(string token)
    {
        var given = Encoding.UTF8.GetBytes(token);
        foreach (var h in _hosts.Values)
        {
            var expected = Encoding.UTF8.GetBytes(h.Token);
            if (expected.Length == given.Length && CryptographicOperations.FixedTimeEquals(expected, given)) return h;
        }
        return null;
    }

    internal void OnPluginConnected(PluginHost host)
    {
        PublishEvent(PluginEvents.PluginStarted, new { pluginId = host.Id }, excludePluginId: host.Id);
        OnSubscriptionChanged();
    }

    internal void OnPluginDisconnected(PluginHost host)
    {
        RefreshSubscribedPatterns();
        if (!_stopping)
            PublishEvent(PluginEvents.PluginStopped, new { pluginId = host.Id }, excludePluginId: host.Id);
    }

    // ── controller.command ───────────────────────────────────────────────────

    internal async Task<object?> ExecuteCommandAsync(string pluginId, string command, JsonElement? parameters)
    {
        var handler = _options.CommandHandler ?? throw new PluginProtocolException("unsupported", "Commands are not available");
        var msg = new CommandMessage { Type = "Command", Id = "plugin:" + pluginId, Command = command, Params = parameters };
        var result = await handler(msg);
        var element = JsonSerializer.SerializeToElement(result ?? new { }, CommandResultJson);
        if (element.ValueKind == JsonValueKind.Object && element.TryGetProperty("ok", out var ok) && ok.ValueKind == JsonValueKind.False)
        {
            string err = element.TryGetProperty("error", out var e) && e.ValueKind == JsonValueKind.String ? e.GetString()! : "commandFailed";
            string? message = element.TryGetProperty("message", out var me) && me.ValueKind == JsonValueKind.String ? me.GetString() : null;
            throw new PluginProtocolException(err, message ?? err);
        }
        return element;
    }

    // ── events ───────────────────────────────────────────────────────────────

    /// <summary>
    /// Sends an event to every connected plugin whose subscription matches <paramref name="name"/>.
    /// Non-blocking; the payload is serialized (camelCase) once.
    /// </summary>
    public void PublishEvent(string name, object payload) => PublishEvent(name, payload, null);

    internal void PublishEvent(string name, object payload, string? excludePluginId)
    {
        string? json = null;
        foreach (var h in _hosts.Values)
        {
            if (excludePluginId != null && string.Equals(h.Id, excludePluginId, StringComparison.OrdinalIgnoreCase)) continue;
            var s = h.Session;
            if (s is null || s.IsClosed || !h.Connected || !s.Subscription.Wants(name)) continue;
            json ??= payload is RawJson raw ? raw.Json : JsonSerializer.Serialize(payload, PluginJson.Options);
            s.SendEvent(name, json);
        }
    }

    // Every pattern some live session subscribes to; rebuilt when a subscription or a
    // connection changes, so HasSubscribers is a lock-free scan of (usually) nothing.
    private volatile string[] _subscribedPatterns = Array.Empty<string>();

    /// <summary>
    /// True when a connected plugin may want <paramref name="eventName"/> — the cheap gate the
    /// executor checks before building a program/step event payload (false with no plugins).
    /// </summary>
    public bool HasSubscribers(string eventName)
    {
        var patterns = _subscribedPatterns;
        foreach (var p in patterns)
            if (PluginEvents.Matches(p, eventName)) return true;
        return false;
    }

    private void RefreshSubscribedPatterns() =>
        _subscribedPatterns = _hosts.Values
            .Select(h => h.Session)
            .Where(s => s is { IsClosed: false })
            .SelectMany(s => s!.Subscription.Patterns)
            .Distinct(StringComparer.Ordinal)
            .ToArray();

    internal void OnSubscriptionChanged()
    {
        RefreshSubscribedPatterns();
        if (_options.RunPollThread && _pollThread == null && AnyPeriodicSubscriber())
        {
            lock (_pollLock)
            {
                if (_pollThread == null)
                {
                    _pollThread = new Thread(PollLoop) { IsBackground = true, Name = "PluginEventPoll" };
                    _pollThread.Start();
                }
            }
        }
        _pollWake.Set();
    }

    private static readonly string[] PeriodicEvents =
        [PluginEvents.RobotPosition, PluginEvents.Status, PluginEvents.IoChanged,
         PluginEvents.RobotHomed, PluginEvents.RobotFault, PluginEvents.RobotFaultCleared];

    private bool AnyPeriodicSubscriber() =>
        _hosts.Values.Any(h => h.Session is { IsClosed: false } s && PeriodicEvents.Any(s.Subscription.Wants));

    private void PollLoop()
    {
        while (!_stopping)
        {
            int wait = 1000;
            try
            {
                if (AnyPeriodicSubscriber())
                {
                    PollOnce();
                    wait = 5;
                }
                else _lastRobot = null;
            }
            catch (Exception ex) { ConsoleSink($"[Plugins] Event poll failed: {ex.Message}"); wait = 100; }
            _pollWake.WaitOne(wait);
        }
    }

    /// <summary>
    /// One pass of the event poll: sends due <c>robot.position</c>, <c>status</c> and
    /// <c>io.changed</c> events and the robot transition events (homed / fault / faultCleared).
    /// Called by the poll thread; tests call it directly with a fake clock.
    /// </summary>
    public void PollOnce()
    {
        long now = Clock.NowMs;
        PluginRobotSnapshot? robot = null;
        bool robotRead = false;
        string? statusJson = null;
        Dictionary<string, double>? io = null;
        bool wantsTransitions = false;

        PluginRobotSnapshot? Robot()
        {
            if (!robotRead) { robot = _options.RobotProvider?.Invoke(); robotRead = true; }
            return robot;
        }

        foreach (var h in _hosts.Values)
        {
            var s = h.Session;
            if (s is null || s.IsClosed || !h.Connected) continue;
            var sub = s.Subscription;

            if (sub.Wants(PluginEvents.RobotHomed) || sub.Wants(PluginEvents.RobotFault) || sub.Wants(PluginEvents.RobotFaultCleared))
                wantsTransitions = true;

            if (sub.Wants(PluginEvents.RobotPosition) && now >= s.NextPositionDueMs && Robot() is { } r)
            {
                s.NextPositionDueMs = now + sub.PositionIntervalMs;
                s.SendEvent(PluginEvents.RobotPosition, JsonSerializer.Serialize(new { x = r.X, y = r.Y, z = r.Z, rx = r.RX, ry = r.RY, rz = r.RZ, moving = r.Moving }));
            }

            if (sub.Wants(PluginEvents.Status) && now >= s.NextStatusDueMs && _options.StatusProvider != null)
            {
                s.NextStatusDueMs = now + sub.StatusIntervalMs;
                statusJson ??= JsonSerializer.Serialize(_options.StatusProvider() ?? new { }, CommandResultJson);
                s.SendEvent(PluginEvents.Status, statusJson);
            }

            if (sub.Wants(PluginEvents.IoChanged) && now >= s.NextIoDueMs && _options.IoProvider != null)
            {
                s.NextIoDueMs = now + sub.IoIntervalMs;
                if (io == null) { io = new Dictionary<string, double>(StringComparer.Ordinal); _options.IoProvider(io); }
                var last = s.LastIo;
                var changes = new Dictionary<string, double>(StringComparer.Ordinal);
                foreach (var (k, v) in io)
                    if (last == null || !last.TryGetValue(k, out var old) || old != v) changes[k] = v;
                s.LastIo = new Dictionary<string, double>(io, StringComparer.Ordinal);
                if (changes.Count > 0)
                    s.SendEvent(PluginEvents.IoChanged, JsonSerializer.Serialize(new { changes }));
            }
        }

        if (!wantsTransitions) { _lastRobot = null; return; }
        var cur = Robot();
        if (cur == null) return;
        var prev = _lastRobot;
        _lastRobot = cur;
        if (prev == null) return;
        if (cur.Homed && !prev.Homed)
            PublishEvent(PluginEvents.RobotHomed, new { homed = true });
        if (cur.Faulted && !prev.Faulted)
            PublishEvent(PluginEvents.RobotFault, new { joint = cur.FaultJoint, message = cur.FaultMessage });
        if (!cur.Faulted && prev.Faulted)
            PublishEvent(PluginEvents.RobotFaultCleared, new { });
    }

    // ── functions ────────────────────────────────────────────────────────────

    /// <summary>Every function of every installed (valid) plugin.</summary>
    public IReadOnlyList<PluginFunctionInfo> Functions
    {
        get
        {
            var list = new List<PluginFunctionInfo>();
            foreach (var h in Plugins)
            {
                if (h.Problems.Count > 0 || h.Manifest is not { } m) continue;
                bool running = h.IsRunning;
                foreach (var f in m.Functions)
                    list.Add(new PluginFunctionInfo(h.Id, f.Name, $"{h.Id}.{f.Name}", f.MinArgs, f.EffectiveMaxArgs,
                        string.IsNullOrWhiteSpace(f.Signature) ? $"{h.Id}.{f.Name}(…)" : f.Signature!,
                        f.Description ?? "", f.EffectiveTimeoutMs, running));
            }
            return list;
        }
    }

    /// <summary>
    /// Calls <c>id.name(args)</c> synchronously (blocks up to the function's timeout).
    /// Throws <see cref="PluginFunctionException"/>.
    /// </summary>
    public double CallFunction(string fullName, ReadOnlySpan<double> args)
    {
        int dot = fullName.IndexOf('.');
        var host = dot > 0 ? Get(fullName[..dot]) : null;
        string name = dot > 0 ? fullName[(dot + 1)..] : fullName;
        var def = host?.Manifest?.Functions.FirstOrDefault(f => string.Equals(f.Name, name, StringComparison.OrdinalIgnoreCase));
        if (host is null || def is null)
            throw new PluginFunctionException("unknownPluginFunction", $"Unknown plugin function '{fullName}'");

        var session = host.Session;
        if (session is null || !host.IsRunning)
            throw new PluginFunctionException("pluginNotRunning", $"Plugin '{host.Id}' is not running");

        var argArray = args.ToArray();
        int timeout  = def.EffectiveTimeoutMs;
        JsonElement result;
        try
        {
            result = session.RequestAsync("function.call", new { name = def.Name, args = argArray }, timeout).GetAwaiter().GetResult();
        }
        catch (TimeoutException)
        {
            throw new PluginFunctionException("pluginFunctionTimeout", $"{host.Id}.{def.Name}() did not answer within {timeout} ms");
        }
        catch (PluginProtocolException ex) when (ex.Code == "disconnected")
        {
            throw new PluginFunctionException("pluginNotRunning", $"Plugin '{host.Id}' is not running");
        }
        catch (PluginProtocolException ex)
        {
            throw new PluginFunctionException("pluginFunctionFailed", $"{host.Id}.{def.Name}() failed: {ex.Message}");
        }

        if (result.ValueKind == JsonValueKind.Object && result.TryGetProperty("value", out var v))
        {
            switch (v.ValueKind)
            {
                case JsonValueKind.Number: return v.GetDouble();
                case JsonValueKind.True:   return 1;
                case JsonValueKind.False:  return 0;
            }
        }
        throw new PluginFunctionException("pluginFunctionFailed", $"{host.Id}.{def.Name}() returned no numeric value");
    }

    // ── steps ────────────────────────────────────────────────────────────────

    /// <summary>
    /// Sends <c>step.execute</c> and returns the invocation id immediately. <paramref name="onReply"/>
    /// runs once on a thread-pool thread with the outcome (never after <see cref="CancelStep"/>).
    /// Throws <see cref="PluginNotRunningException"/>.
    /// </summary>
    public string ExecuteStep(string pluginId, string stepId, PluginStepRequest request, Action<PluginStepReply> onReply)
    {
        var host = Get(pluginId) ?? throw new PluginNotRunningException(pluginId);
        var session = host.Session;
        if (session is null || !host.IsRunning) throw new PluginNotRunningException(host.Id);

        string invocationId = Guid.NewGuid().ToString("N");
        _invocations[invocationId] = new StepInvocation(invocationId, host.Id, session, onReply);
        var task = session.RequestAsync("step.execute", new
        {
            invocationId,
            stepId,
            programName  = request.ProgramName,
            stepName     = request.StepName,
            @params      = request.Params,
            isBackground = request.IsBackground,
        });
        _ = task.ContinueWith(t => CompleteInvocation(invocationId, t), TaskScheduler.Default);
        return invocationId;
    }

    /// <summary>Abandons an outstanding step (a late reply is discarded) and sends <c>step.cancel</c>.</summary>
    public void CancelStep(string invocationId, string reason)
    {
        if (!_invocations.TryRemove(invocationId, out var inv)) return;
        if (!inv.Session.IsClosed) inv.Session.Notify("step.cancel", new { invocationId, reason });
    }

    /// <summary>Invocations awaiting a reply.</summary>
    public int OutstandingSteps => _invocations.Count;

    private void CompleteInvocation(string invocationId, Task<JsonElement> t)
    {
        if (!_invocations.TryRemove(invocationId, out var inv)) return; // cancelled: discard
        PluginStepReply reply;
        if (t.IsCompletedSuccessfully)
        {
            var outputs = new Dictionary<string, JsonElement>(StringComparer.Ordinal);
            if (t.Result.ValueKind == JsonValueKind.Object && t.Result.TryGetProperty("outputs", out var o) && o.ValueKind == JsonValueKind.Object)
                foreach (var p in o.EnumerateObject()) outputs[p.Name] = p.Value.Clone();
            reply = new PluginStepReply { Ok = true, Outputs = outputs };
        }
        else
        {
            var ex = t.Exception?.GetBaseException();
            reply = ex is PluginProtocolException { Code: "disconnected" }
                ? new PluginStepReply { Ok = false, Error = "pluginDisconnected", Message = $"Plugin '{inv.PluginId}' disconnected" }
                : ex is PluginProtocolException pe
                    ? new PluginStepReply { Ok = false, Error = pe.Code, Message = pe.Message }
                    : new PluginStepReply { Ok = false, Error = "pluginStepFailed", Message = ex?.Message ?? "failed" };
        }
        try { inv.OnReply(reply); }
        catch (Exception ex) { ConsoleSink($"[Plugins] Step reply handler failed: {ex}"); }
    }

    internal void OnStepProgress(string invocationId, string? message, double? percent)
    {
        try { StepProgress?.Invoke(invocationId, message, percent); }
        catch (Exception ex) { ConsoleSink($"[Plugins] StepProgress handler failed: {ex.Message}"); }
    }

    // ── contributions ────────────────────────────────────────────────────────

    /// <summary>Steps, functions and properties of every installed (valid) plugin.</summary>
    public PluginContributions GetContributions()
    {
        var steps = new List<PluginStepContribution>();
        var fns   = new List<PluginFunctionContribution>();
        var props = new List<PluginPropertyContribution>();
        foreach (var h in Plugins)
        {
            if (h.Problems.Count > 0 || h.Manifest is not { } m) continue;
            bool running = h.IsRunning;
            foreach (var s in m.Steps) steps.Add(new PluginStepContribution(h.Id, h.Name, running, s));
            foreach (var f in m.Functions) fns.Add(new PluginFunctionContribution(h.Id, h.Name, running, f));
            var declared = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var p in m.Properties)
            {
                declared.Add(p.Name);
                props.Add(new PluginPropertyContribution(h.Id, h.Name, running, p,
                    h.Properties.TryGetValue(p.Name, out var v) ? v : null));
            }
            foreach (var (name, value) in h.Properties)
                if (!declared.Contains(name))
                    props.Add(new PluginPropertyContribution(h.Id, h.Name, running,
                        new PluginPropertyDef { Name = name, Description = PluginPropertySource.Undocumented, Type = "number" }, value));
        }
        return new PluginContributions(steps, fns, props);
    }

    // ── discovery / reload ───────────────────────────────────────────────────

    private IEnumerable<string> PluginFolders() =>
        Directory.Exists(PluginsDir)
            ? Directory.GetDirectories(PluginsDir).Where(d => !Path.GetFileName(d).StartsWith('.')).OrderBy(d => d, StringComparer.Ordinal)
            : Enumerable.Empty<string>();

    private PluginHost CreateHost(string dir, PluginLog? log = null)
    {
        var load = PluginManifestLoader.Load(dir, IsFunctionName);
        return new PluginHost(this, Path.GetFileName(dir), dir, load, log);
    }

    /// <summary>
    /// Rescans <c>plugins/</c>: new folders appear (and start when enabled + autoStart), removed
    /// ones are stopped and vanish, manifests of plugins that are not active reload.
    /// </summary>
    public async Task<IReadOnlyList<PluginHost>> ReloadAsync()
    {
        await _structureLock.WaitAsync();
        try
        {
            Directory.CreateDirectory(PluginsDir);
            var folders = PluginFolders().GroupBy(d => Path.GetFileName(d)!, StringComparer.OrdinalIgnoreCase)
                                          .ToDictionary(g => g.Key, g => g.First(), StringComparer.OrdinalIgnoreCase);

            foreach (var h in _hosts.Values.ToList())
            {
                if (folders.ContainsKey(h.Id)) continue;
                await h.StopAsync("removed");
                _hosts.TryRemove(h.Id, out _);
                ConsoleSink($"[Plugins] '{h.Id}' removed (folder gone)");
            }

            foreach (var (name, dir) in folders)
            {
                if (_hosts.TryGetValue(name, out var existing))
                {
                    if (!existing.IsActive)
                    {
                        bool wasError = existing.State == PluginState.Error;
                        existing.Reload(PluginManifestLoader.Load(dir, IsFunctionName));
                        // A plugin fixed from 'error' never ran: start it like a newly found one.
                        if (wasError && IsStarted && existing.State == PluginState.Stopped && existing.Manifest?.AutoStart != false)
                            existing.Start(manual: true);
                    }
                    continue;
                }
                var host = CreateHost(dir);
                _hosts[name] = host;
                if (IsStarted && host.State == PluginState.Stopped && host.Manifest?.AutoStart != false)
                    host.Start(manual: true);
            }
            return Plugins;
        }
        finally { _structureLock.Release(); }
    }

    // ── install / uninstall / download ───────────────────────────────────────

    /// <summary>
    /// Installs a plugin from zip bytes: <c>plugin.json</c> at the zip root or inside a single
    /// top-level folder. <paramref name="replace"/> allows overwriting an installed id (its
    /// config survives). The plugin starts when enabled and <c>autoStart</c>.
    /// </summary>
    public async Task<PluginInstallResult> InstallAsync(byte[] zipBytes, bool replace)
    {
        if (zipBytes.LongLength > MaxZipBytes) return PluginInstallResult.Fail("badZip", "Zip is larger than 200 MB");

        ZipArchive archive;
        try { archive = new ZipArchive(new MemoryStream(zipBytes, writable: false), ZipArchiveMode.Read); }
        catch (Exception ex) when (ex is InvalidDataException or ArgumentException or IOException)
        {
            return PluginInstallResult.Fail("badZip", "Not a zip file: " + ex.Message);
        }

        using (archive)
        {
            var files = archive.Entries.Where(e => !e.FullName.EndsWith('/') && !e.FullName.EndsWith('\\')).ToList();
            string? prefix = FindManifestPrefix(files);
            if (prefix is null)
                return PluginInstallResult.Fail("badZip", "plugin.json not found at the zip root or inside a single top-level folder");

            string json;
            using (var reader = new StreamReader(files.First(e => Normalize(e.FullName) == prefix + PluginManifestLoader.ManifestFileName).Open()))
                json = await reader.ReadToEndAsync();
            var load = PluginManifestLoader.Parse(json, IsFunctionName);
            if (load.Manifest is null || load.Problems.Count > 0)
                return PluginInstallResult.Fail("badManifest", string.Join("; ", load.Problems.Select(p => p.ToString())));
            var manifest = load.Manifest;
            string id = manifest.Id;

            await _structureLock.WaitAsync();
            try
            {
                Directory.CreateDirectory(PluginsDir);
                var existing = Get(id);
                string target = Path.Combine(PluginsDir, id);
                bool folderExists = Directory.Exists(target)
                    || PluginFolders().Any(d => string.Equals(Path.GetFileName(d), id, StringComparison.OrdinalIgnoreCase));
                if ((existing != null || folderExists) && !replace)
                    return PluginInstallResult.Fail("idExists", $"A plugin with id '{id}' is already installed");

                // Extract to a staging folder first so a bad zip never damages an installed plugin.
                string staging = Path.Combine(PluginsDir, $".staging-{Guid.NewGuid():N}");
                try
                {
                    Directory.CreateDirectory(staging);
                    string stagingFull = Path.GetFullPath(staging) + Path.DirectorySeparatorChar;
                    foreach (var entry in files)
                    {
                        string rel = Normalize(entry.FullName);
                        if (!rel.StartsWith(prefix, StringComparison.Ordinal)) continue;
                        rel = rel[prefix.Length..];
                        if (rel.Length == 0) continue;
                        string dest = Path.GetFullPath(Path.Combine(staging, rel.Replace('/', Path.DirectorySeparatorChar)));
                        if (!dest.StartsWith(stagingFull, StringComparison.Ordinal))
                            return PluginInstallResult.Fail("badZip", $"Zip entry '{entry.FullName}' escapes the plugin folder");
                        Directory.CreateDirectory(Path.GetDirectoryName(dest)!);
                        entry.ExtractToFile(dest, overwrite: true);
                    }
                    if (manifest.Runtime == "exe" && !OperatingSystem.IsWindows())
                    {
                        string exe = Path.Combine(staging, manifest.Entry!);
                        if (File.Exists(exe))
                            File.SetUnixFileMode(exe, File.GetUnixFileMode(exe) | UnixFileMode.UserExecute | UnixFileMode.GroupExecute | UnixFileMode.OtherExecute);
                    }

                    PluginLog? log = null;
                    if (existing != null)
                    {
                        await existing.StopAsync("replaced");
                        existing.Log.DetachFile();
                        log = existing.Log;
                        _hosts.TryRemove(existing.Id, out _);
                    }
                    if (Directory.Exists(target)) DeleteDirectory(target);
                    Directory.Move(staging, target);
                    log?.AttachFile(Path.Combine(target, "plugin.log"));

                    var host = CreateHost(target, log);
                    _hosts[id] = host;
                    host.Log.Append("info", $"Installed version {manifest.Version ?? "?"}");
                    ConsoleSink($"[Plugins] Installed '{id}' {manifest.Version}");
                    if (IsStarted && host.State == PluginState.Stopped && host.Manifest?.AutoStart != false)
                        host.Start(manual: true);
                    return new PluginInstallResult(true, id, host, null, null);
                }
                finally
                {
                    if (Directory.Exists(staging)) DeleteDirectory(staging);
                }
            }
            finally { _structureLock.Release(); }
        }
    }

    /// <summary>"" when plugin.json is at the root, "folder/" for a single top-level folder, else null.</summary>
    private static string? FindManifestPrefix(List<ZipArchiveEntry> files)
    {
        var names = files.Select(e => Normalize(e.FullName)).ToList();
        if (names.Contains(PluginManifestLoader.ManifestFileName)) return "";
        var tops = names.Select(n => n.Split('/')[0]).Distinct(StringComparer.Ordinal).ToList();
        // ignore macOS metadata folders
        tops.Remove("__MACOSX");
        if (tops.Count != 1) return null;
        string prefix = tops[0] + "/";
        return names.Contains(prefix + PluginManifestLoader.ManifestFileName) ? prefix : null;
    }

    private static string Normalize(string zipPath) => zipPath.Replace('\\', '/').TrimStart('/');

    /// <summary>Stops the plugin and deletes its folder and config. False when the id is unknown.</summary>
    public async Task<bool> UninstallAsync(string id)
    {
        await _structureLock.WaitAsync();
        try
        {
            var host = Get(id);
            if (host is null) return false;
            await host.StopAsync("uninstall");
            host.Log.DetachFile();
            _hosts.TryRemove(host.Id, out _);
            DeleteDirectory(host.Folder);
            ConfigStore.Delete(host.Id);
            ConsoleSink($"[Plugins] Uninstalled '{host.Id}'");
            return true;
        }
        finally { _structureLock.Release(); }
    }

    /// <summary>The plugin folder as a zip (without <c>.venv</c> and <c>plugin.log*</c>), or null for an unknown id.</summary>
    public byte[]? DownloadZip(string id)
    {
        var host = Get(id);
        if (host is null || !Directory.Exists(host.Folder)) return null;
        using var ms = new MemoryStream();
        using (var zip = new ZipArchive(ms, ZipArchiveMode.Create, leaveOpen: true))
        {
            foreach (var file in Directory.EnumerateFiles(host.Folder, "*", SearchOption.AllDirectories))
            {
                string rel = Path.GetRelativePath(host.Folder, file).Replace('\\', '/');
                if (rel.StartsWith(".venv/", StringComparison.Ordinal) || rel.Contains("/__pycache__/") || rel.StartsWith("__pycache__/", StringComparison.Ordinal)) continue;
                if (!rel.Contains('/') && rel.StartsWith("plugin.log", StringComparison.Ordinal)) continue;
                var entry = zip.CreateEntry(rel, CompressionLevel.Optimal);
                using var src = new FileStream(file, FileMode.Open, FileAccess.Read, FileShare.ReadWrite | FileShare.Delete);
                using var dst = entry.Open();
                src.CopyTo(dst);
            }
        }
        return ms.ToArray();
    }

    /// <summary>Recursive delete with a few retries (Windows can hold handles of a just-killed process briefly).</summary>
    private static void DeleteDirectory(string dir)
    {
        for (int attempt = 0; ; attempt++)
        {
            try
            {
                if (!Directory.Exists(dir)) return;
                foreach (var f in Directory.EnumerateFiles(dir, "*", SearchOption.AllDirectories))
                    File.SetAttributes(f, FileAttributes.Normal);
                Directory.Delete(dir, recursive: true);
                return;
            }
            catch (Exception ex) when ((ex is IOException or UnauthorizedAccessException) && attempt < 10)
            {
                Thread.Sleep(100);
            }
        }
    }
}
