using Controller.RobotControl.Plugins;

namespace RobotControl.Tests.Plugins;

/// <summary>The per-plugin state machine with a fake launcher/process and a manual clock.</summary>
public class PluginHostTests : IDisposable
{
    private readonly PluginTestEnv _env = new();

    public void Dispose() => _env.Dispose();

    private PluginHost Boot(object? extra = null, string id = "py")
    {
        _env.WritePlugin(id, PluginTestEnv.Manifest(id, "python", extra));
        _env.Boot(9123);
        return _env.Manager.Get(id)!;
    }

    [Fact]
    public void LaunchPassesTheContractEnvironment()
    {
        var h = Boot();
        Assert.Equal(PluginState.Starting, h.State);
        var ctx = _env.Launcher.Launches.Single();
        Assert.Equal("py", ctx.Environment["SRC_PLUGIN_ID"]);
        Assert.Equal("ws://127.0.0.1:9123/plugin", ctx.Environment["SRC_PLUGIN_URL"]);
        Assert.Equal(h.Token, ctx.Environment["SRC_PLUGIN_TOKEN"]);
        Assert.Equal(Path.GetFullPath(Path.Combine(_env.PluginsDir, "py")), ctx.Environment["SRC_PLUGIN_DIR"]);
        Assert.Equal(_env.Manager.DataDir, ctx.Environment["SRC_DATA_DIR"]);
        Assert.Equal("9.9.9-test", ctx.Environment["SRC_CONTROLLER_VERSION"]);
        Assert.Equal(ctx.PluginDir, ctx.Environment["SRC_PLUGIN_DIR"]);
        Assert.Equal(_env.Launcher.Last.Pid, h.Pid);
    }

    [Fact]
    public async Task ReadyTimeoutBackoffDoublingAndMaxRestarts()
    {
        var h = Boot(new { restart = new { mode = "always", maxRestarts = 2, backoffMs = 1000 }, readyTimeoutMs = 5000 });
        var first = _env.Launcher.Last;

        _env.Clock.Advance(4999);
        Assert.Equal(PluginState.Starting, h.State);
        _env.Clock.Advance(1);                                   // ready timeout → killed, counted as a crash
        Assert.Equal(PluginState.Crashed, h.State);
        Assert.Contains("ready timeout", h.Message);
        Assert.Contains("restarting in 1000 ms", h.Message);
        await Wait.UntilAsync(() => first.Killed, "kill");

        _env.Clock.Advance(999);
        Assert.Single(_env.Launcher.Launches);
        _env.Clock.Advance(1);
        Assert.Equal(2, _env.Launcher.Launches.Count);
        Assert.Equal(PluginState.Starting, h.State);
        Assert.Equal(1, h.RestartCount);

        _env.Launcher.Last.Exit(1);                              // second crash: backoff doubles
        Assert.Equal(PluginState.Crashed, h.State);
        Assert.Contains("restarting in 2000 ms", h.Message);
        Assert.Equal(1, h.LastExitCode);
        _env.Clock.Advance(2000);
        Assert.Equal(3, _env.Launcher.Launches.Count);
        Assert.Equal(2, h.RestartCount);

        _env.Launcher.Last.Exit(1);                              // budget of 2 restarts in 10 min used up
        Assert.Equal(PluginState.Crashed, h.State);
        Assert.Contains("gave up", h.Message);
        _env.Clock.Advance(10 * 60 * 1000);
        Assert.Equal(3, _env.Launcher.Launches.Count);

        Assert.Null(h.Start());                                  // manual start resets the budget
        Assert.Equal(4, _env.Launcher.Launches.Count);
        Assert.Equal(0, h.RestartCount);
        Assert.Equal(PluginState.Starting, h.State);
    }

    [Fact]
    public void BackoffIsCappedAt30Seconds()
    {
        var h = Boot(new { restart = new { mode = "always", maxRestarts = 20, backoffMs = 20000 } });
        _env.Launcher.Last.Exit(1);
        Assert.Contains("restarting in 20000 ms", h.Message);
        _env.Clock.Advance(20000);
        _env.Launcher.Last.Exit(1);
        Assert.Contains("restarting in 30000 ms", h.Message);
    }

    [Fact]
    public async Task RestartModesNeverAndOnFailure()
    {
        var never = Boot(new { restart = new { mode = "never" } }, "never");
        _env.Launcher.Last.Exit(3);
        Assert.Equal(PluginState.Stopped, never.State);
        Assert.Contains("exited with code 3", never.Message);

        _env.WritePlugin("onfail", PluginTestEnv.Manifest("onfail", "python", new { restart = new { mode = "onFailure" } }));
        await _env.Manager.ReloadAsync();
        var onFail = _env.Manager.Get("onfail")!;
        Assert.Equal(PluginState.Starting, onFail.State);
        _env.Launcher.Last.Exit(0);
        Assert.Equal(PluginState.Stopped, onFail.State);
        onFail.Start();
        _env.Launcher.Last.Exit(2);
        Assert.Equal(PluginState.Crashed, onFail.State);
    }

    [Fact]
    public async Task ReadyCancelsTheTimeoutAndLostConnectionIsACrash()
    {
        var h = Boot(new { readyTimeoutMs = 1000 });
        var c = await _env.ConnectReadyAsync("py");
        Assert.Equal(PluginState.Running, h.State);
        _env.Clock.Advance(5000);
        Assert.Equal(PluginState.Running, h.State);              // ready timer cancelled

        var proc = _env.Launcher.Last;
        await c.CloseAsync();
        await Wait.UntilAsync(() => h.State == PluginState.Crashed, "crash on lost socket");
        await Wait.UntilAsync(() => proc.Killed, "process killed");
        Assert.Contains("lost connection", h.Message);
    }

    [Fact]
    public async Task StopSendsShutdownThenKillsAfterGrace()
    {
        var h = Boot();
        var c = await _env.ConnectReadyAsync("py");
        var proc = _env.Launcher.Last;
        await h.StopAsync();
        Assert.Contains(c.Received, x => x.GetProperty("t").GetString() == "req" && x.GetProperty("method").GetString() == "shutdown");
        Assert.True(proc.Killed);                                // fake ignores shutdown → killed after the grace period
        Assert.Equal(PluginState.Stopped, h.State);
        Assert.Null(h.Pid);
        Assert.False(h.Connected);
        _env.Clock.Advance(60_000);
        Assert.Single(_env.Launcher.Launches);                   // a stop never restarts
    }

    [Fact]
    public async Task LaunchErrorsInstallingAndDisable()
    {
        _env.Launcher.Throw = new PluginLaunchException("pythonTooOld", "Python 3.8 is older than 3.9");
        var h = Boot();
        Assert.Equal(PluginState.Error, h.State);
        Assert.StartsWith("pythonTooOld", h.Message);

        _env.Launcher.Throw = null;
        _env.Launcher.SimulateInstall = true;
        _env.Launcher.Gate = new TaskCompletionSource();
        await h.StopAsync();                                     // error stays error (manifest is fine, though)
        Assert.Null(h.Start());
        Assert.Equal(PluginState.Installing, h.State);
        _env.Launcher.Gate.SetResult();
        await Wait.UntilAsync(() => h.State == PluginState.Starting && h.Pid != null, "starting after install");

        await h.SetEnabledAsync(false);
        Assert.Equal(PluginState.Disabled, h.State);
        Assert.Equal("pluginDisabled", h.Start());
        Assert.False(_env.Manager.ConfigStore.LoadOrCreate("py").Enabled);

        _env.Launcher.Gate = null;
        int before = _env.Launcher.Launches.Count;
        await h.SetEnabledAsync(true);
        Assert.Equal(before + 1, _env.Launcher.Launches.Count);  // enabling an autoStart plugin starts it
    }

    [Fact]
    public void InvalidManifestIsErrorAndNeverLaunched()
    {
        _env.WritePlugin("robot", PluginTestEnv.Manifest("robot", "python"));
        _env.Boot();
        var h = _env.Manager.Get("robot")!;
        Assert.Equal(PluginState.Error, h.State);
        Assert.Contains("reservedId", h.Message);
        Assert.Empty(_env.Launcher.Launches);
        Assert.Equal("pluginError", h.Start());
    }
}
