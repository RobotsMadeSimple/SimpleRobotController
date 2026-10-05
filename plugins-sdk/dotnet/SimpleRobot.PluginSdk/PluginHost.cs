using System.Reflection;
using System.Runtime.ExceptionServices;
using System.Text.Json;

namespace SimpleRobot.PluginSdk;

/// <summary>
/// Entry point of a plugin: register handlers, then <see cref="RunAsync"/>. See docs/plugins.md §4 and §8.2.
/// </summary>
public sealed class PluginHost
{
    internal readonly PluginHostOptions Options;
    private readonly List<Func<IPluginContext, Task>> _ready = new();
    private readonly Dictionary<string, Func<IPluginContext, StepParams, Task<StepResult>>> _steps = new();
    private readonly Dictionary<string, Func<IPluginContext, double[], Task<double>>> _functions = new();
    private readonly List<(string Pattern, Func<IPluginContext, JsonElement, Task> Handler)> _events = new();
    private readonly List<Func<IPluginContext, CancellationToken, Task>> _background = new();
    private readonly List<Func<IPluginContext, JsonElement, Task>> _configChanged = new();

    private readonly object _subLock = new();
    private readonly HashSet<string> _extraEvents = new();
    private int? _positionMs, _statusMs, _ioMs;
    private bool _running;

    /// <summary>Reads <c>SRC_PLUGIN_ID/URL/TOKEN/DIR</c>, <c>SRC_DATA_DIR</c> from the environment (set by the controller).</summary>
    public PluginHost() : this(FromEnvironment()) { }

    /// <summary>Development / external mode: connect with explicit settings.</summary>
    public PluginHost(PluginHostOptions options)
    {
        ArgumentNullException.ThrowIfNull(options);
        if (string.IsNullOrWhiteSpace(options.Url)) throw new ArgumentException("Url is required", nameof(options));
        if (string.IsNullOrEmpty(options.Token)) throw new ArgumentException("Token is required", nameof(options));
        Options = options;
    }

    private static PluginHostOptions FromEnvironment()
    {
        static string? E(string n) => Environment.GetEnvironmentVariable(n);
        var url = E("SRC_PLUGIN_URL");
        var token = E("SRC_PLUGIN_TOKEN");
        if (string.IsNullOrEmpty(url) || string.IsNullOrEmpty(token))
            throw new InvalidOperationException(
                "SRC_PLUGIN_URL / SRC_PLUGIN_TOKEN are not set. Run the plugin from the controller, or use new PluginHost(new PluginHostOptions { ... }).");
        return new PluginHostOptions
        {
            Url = url, Token = token,
            PluginId = E("SRC_PLUGIN_ID") ?? "",
            PluginDir = E("SRC_PLUGIN_DIR"),
            DataDir = E("SRC_DATA_DIR"),
        };
    }

    // ---- registration ----------------------------------------------------------------

    public PluginHost OnReady(Func<IPluginContext, Task> handler) { Guard(); _ready.Add(handler); return this; }

    public PluginHost Step(string id, Func<IPluginContext, StepParams, Task<StepResult>> handler)
    {
        Guard();
        if (!_steps.TryAdd(id, handler)) throw new ArgumentException($"Step '{id}' is already registered", nameof(id));
        return this;
    }

    public PluginHost Function(string name, Func<IPluginContext, double[], Task<double>> handler)
    {
        Guard();
        if (!_functions.TryAdd(name, handler)) throw new ArgumentException($"Function '{name}' is already registered", nameof(name));
        return this;
    }

    /// <summary>Handles a controller event (<c>program.started</c>, a glob such as <c>program.*</c>, …). Subscribed automatically at ready.</summary>
    public PluginHost On(string eventName, Func<IPluginContext, JsonElement, Task> handler)
    {
        Guard();
        _events.Add((eventName, handler));
        return this;
    }

    /// <summary>Runs while connected (restarted after a reconnect); the token is cancelled when the connection ends or on shutdown.</summary>
    public PluginHost Background(Func<IPluginContext, CancellationToken, Task> handler) { Guard(); _background.Add(handler); return this; }

    public PluginHost OnConfigChanged(Func<IPluginContext, JsonElement, Task> handler) { Guard(); _configChanged.Add(handler); return this; }

    /// <summary>Default intervals used by the automatic subscription (null = controller default).</summary>
    public PluginHost SubscribeIntervals(int? positionIntervalMs = null, int? statusIntervalMs = null, int? ioIntervalMs = null)
    {
        Guard();
        lock (_subLock) { _positionMs = positionIntervalMs; _statusMs = statusIntervalMs; _ioMs = ioIntervalMs; }
        return this;
    }

    /// <summary>Registers every method of <paramref name="instance"/> marked with <see cref="PluginStepAttribute"/>, <see cref="PluginFunctionAttribute"/> or <see cref="PluginEventAttribute"/>.</summary>
    public PluginHost Register(object instance)
    {
        ArgumentNullException.ThrowIfNull(instance);
        Guard();
        const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
        foreach (var m in instance.GetType().GetMethods(flags))
        {
            foreach (var a in m.GetCustomAttributes<PluginStepAttribute>())
            {
                Expect(m, typeof(StepResult), typeof(IPluginContext), typeof(StepParams));
                Step(a.Id, async (c, p) => (StepResult)(await InvokeAsync(m, instance, c, p).ConfigureAwait(false))!);
            }
            foreach (var a in m.GetCustomAttributes<PluginFunctionAttribute>())
            {
                Expect(m, typeof(double), typeof(IPluginContext), typeof(double[]));
                Function(a.Name, async (c, args) => (double)(await InvokeAsync(m, instance, c, args).ConfigureAwait(false))!);
            }
            foreach (var a in m.GetCustomAttributes<PluginEventAttribute>())
            {
                Expect(m, typeof(void), typeof(IPluginContext), typeof(JsonElement));
                On(a.EventName, async (c, e) => await InvokeAsync(m, instance, c, e).ConfigureAwait(false));
            }
        }
        return this;
    }

    private static void Expect(MethodInfo m, Type returns, params Type[] parameters)
    {
        var ps = m.GetParameters();
        var rt = m.ReturnType;
        bool retOk = returns == typeof(void)
            ? rt == typeof(void) || rt == typeof(Task)
            : rt == returns || rt == typeof(Task<>).MakeGenericType(returns);
        if (!retOk || ps.Length != parameters.Length || ps.Where((p, i) => p.ParameterType != parameters[i]).Any())
            throw new ArgumentException(
                $"{m.DeclaringType?.Name}.{m.Name} must have the signature ({string.Join(", ", parameters.Select(p => p.Name))}) returning {returns.Name} or Task<{returns.Name}>");
    }

    private static async Task<object?> InvokeAsync(MethodInfo m, object instance, params object?[] args)
    {
        object? r;
        try { r = m.Invoke(instance, args); }
        catch (TargetInvocationException e) when (e.InnerException != null)
        {
            ExceptionDispatchInfo.Capture(e.InnerException).Throw();
            throw;
        }
        if (r is Task t)
        {
            await t.ConfigureAwait(false);
            return t.GetType().IsGenericType ? t.GetType().GetProperty("Result")!.GetValue(t) : null;
        }
        return r;
    }

    private void Guard()
    {
        if (_running) throw new InvalidOperationException("Handlers must be registered before RunAsync.");
    }

    // ---- subscription state (survives reconnects) -------------------------------------

    internal bool TryBuildSubscription(out Dictionary<string, object?> payload)
    {
        lock (_subLock)
        {
            var events = new SortedSet<string>(StringComparer.Ordinal);
            foreach (var (p, _) in _events) events.Add(p);
            foreach (var e in _extraEvents) events.Add(e);
            payload = new Dictionary<string, object?> { ["events"] = events.ToArray() };
            if (_positionMs is int a) payload["positionIntervalMs"] = a;
            if (_statusMs is int b) payload["statusIntervalMs"] = b;
            if (_ioMs is int c) payload["ioIntervalMs"] = c;
            return events.Count > 0;
        }
    }

    internal void AddSubscription(IEnumerable<string> events, int? pos, int? status, int? io)
    {
        lock (_subLock)
        {
            foreach (var e in events) _extraEvents.Add(e);
            if (pos != null) _positionMs = pos;
            if (status != null) _statusMs = status;
            if (io != null) _ioMs = io;
        }
    }

    // ---- handler access for the session -------------------------------------------------

    internal IReadOnlyList<Func<IPluginContext, Task>> ReadyHandlers => _ready;
    internal IReadOnlyList<Func<IPluginContext, CancellationToken, Task>> BackgroundHandlers => _background;
    internal IReadOnlyList<Func<IPluginContext, JsonElement, Task>> ConfigHandlers => _configChanged;
    internal bool TryGetStep(string id, out Func<IPluginContext, StepParams, Task<StepResult>>? h) => _steps.TryGetValue(id, out h);
    internal bool TryGetFunction(string name, out Func<IPluginContext, double[], Task<double>>? h) => _functions.TryGetValue(name, out h);

    internal IEnumerable<Func<IPluginContext, JsonElement, Task>> EventHandlersFor(string name)
    {
        foreach (var (pattern, handler) in _events)
        {
            if (pattern == name || pattern == "*"
                || (pattern.EndsWith('*') && name.StartsWith(pattern.AsSpan(0, pattern.Length - 1), StringComparison.Ordinal)))
                yield return handler;
        }
    }

    // ---- run loop --------------------------------------------------------------------------

    /// <summary>
    /// Connects, sends <c>plugin.ready</c>, serves requests concurrently and reconnects with exponential backoff
    /// (1 s → 30 s) when the connection is lost. Returns after replying to <c>shutdown</c> (or when <paramref name="cancellationToken"/> is cancelled).
    /// Throws <see cref="PluginAuthException"/> when the controller rejects the token and <see cref="PluginReplacedException"/> when another connection took over.
    /// </summary>
    public async Task RunAsync(CancellationToken cancellationToken = default)
    {
        _running = true;
        var delay = Options.MinReconnectDelay;
        while (!cancellationToken.IsCancellationRequested)
        {
            var session = new Session(this);
            try
            {
                var end = await session.RunAsync(cancellationToken).ConfigureAwait(false);
                if (end != SessionEnd.Lost) return;
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested) { return; }
            catch (Exception e) when (e is PluginAuthException or PluginReplacedException) { throw; }
            catch (Exception e)
            {
                Console.Error.WriteLine($"[SimpleRobot.PluginSdk] connection failed: {e.Message}");
            }

            if (session.WasReady) delay = Options.MinReconnectDelay;
            try { await Task.Delay(delay, cancellationToken).ConfigureAwait(false); }
            catch (OperationCanceledException) { return; }
            delay = TimeSpan.FromTicks(Math.Min(delay.Ticks * 2, Options.MaxReconnectDelay.Ticks));
        }
    }
}

internal enum SessionEnd { Shutdown, Lost, Cancelled }
