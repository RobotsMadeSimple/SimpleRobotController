using System.IO.Compression;
using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Plugins;

namespace RobotControl.Tests.Plugins;

/// <summary>Install / uninstall / download / reload and the WebSocket commands over a standalone manager.</summary>
public class PluginInstallTests : IDisposable
{
    private readonly PluginTestEnv _env = new();

    public void Dispose() => _env.Dispose();

    private static string ManifestJson(string id, string version = "1.0.0", string runtime = "python") =>
        JsonSerializer.Serialize(PluginTestEnv.Manifest(id, runtime, new { version }), PluginJson.Options);

    [Fact]
    public async Task InstallFromRootAndSingleTopFolderZips()
    {
        _env.Boot();
        var r = await _env.Manager.InstallAsync(PluginTestEnv.Zip(("plugin.json", ManifestJson("alpha")), ("main.py", "print(1)"), ("lib/util.py", "x=1")), replace: false);
        Assert.True(r.Ok, r.Message);
        Assert.Equal("alpha", r.Id);
        Assert.True(File.Exists(Path.Combine(_env.PluginsDir, "alpha", "lib", "util.py")));
        Assert.Equal(PluginState.Starting, r.Host!.State);         // enabled + autoStart → launched
        Assert.Single(_env.Launcher.Launches);

        r = await _env.Manager.InstallAsync(PluginTestEnv.Zip(("beta-1.0/plugin.json", ManifestJson("beta")), ("beta-1.0/main.py", "")), false);
        Assert.True(r.Ok, r.Message);
        Assert.True(File.Exists(Path.Combine(_env.PluginsDir, "beta", "main.py")));
        Assert.Empty(Directory.GetDirectories(_env.PluginsDir, ".staging-*"));
    }

    [Fact]
    public async Task InstallRejectsBadZipsManifestsAndExistingIds()
    {
        _env.Manager.Discover();
        var r = await _env.Manager.InstallAsync(new byte[] { 1, 2, 3, 4 }, false);
        Assert.Equal("badZip", r.Error);

        r = await _env.Manager.InstallAsync(PluginTestEnv.Zip(("readme.txt", "hi")), false);
        Assert.Equal("badZip", r.Error);

        r = await _env.Manager.InstallAsync(PluginTestEnv.Zip(("a/plugin.json", ManifestJson("a1")), ("b/x.txt", "")), false);
        Assert.Equal("badZip", r.Error);                        // two top-level folders

        r = await _env.Manager.InstallAsync(PluginTestEnv.Zip(("plugin.json", ManifestJson("time"))), false);
        Assert.Equal("badManifest", r.Error);
        Assert.Contains("reservedId", r.Message);

        r = await _env.Manager.InstallAsync(PluginTestEnv.Zip(("plugin.json", ManifestJson("evil")), ("../../escape.txt", "x")), false);
        Assert.Equal("badZip", r.Error);
        Assert.False(Directory.Exists(Path.Combine(_env.PluginsDir, "evil")));

        Assert.True((await _env.Manager.InstallAsync(PluginTestEnv.Zip(("plugin.json", ManifestJson("gamma"))), false)).Ok);
        r = await _env.Manager.InstallAsync(PluginTestEnv.Zip(("plugin.json", ManifestJson("gamma", "2.0.0"))), false);
        Assert.Equal("idExists", r.Error);
    }

    [Fact]
    public async Task ReplaceKeepsConfigAndToken()
    {
        _env.Manager.Discover();
        Assert.True((await _env.Manager.InstallAsync(PluginTestEnv.Zip(("plugin.json", ManifestJson("gamma")), ("old.txt", "")), false)).Ok);
        var token = _env.Manager.Get("gamma")!.Token;
        Assert.Null(_env.Manager.Get("gamma")!.SetConfig(new() { ["samples"] = JsonSerializer.SerializeToElement(42) }));

        var r = await _env.Manager.InstallAsync(PluginTestEnv.Zip(("plugin.json", ManifestJson("gamma", "2.0.0"))), replace: true);
        Assert.True(r.Ok, r.Message);
        var h = _env.Manager.Get("gamma")!;
        Assert.Equal("2.0.0", h.Manifest!.Version);
        Assert.Equal(token, h.Token);
        Assert.Equal(42, h.GetConfig()["samples"].GetInt32());
        Assert.False(File.Exists(Path.Combine(_env.PluginsDir, "gamma", "old.txt")));
    }

    [Fact]
    public async Task UninstallDeletesFolderAndConfig()
    {
        _env.Boot();
        Assert.True((await _env.Manager.InstallAsync(PluginTestEnv.Zip(("plugin.json", ManifestJson("delta"))), false)).Ok);
        var proc = _env.Launcher.Last;
        Assert.True(await _env.Manager.UninstallAsync("delta"));
        Assert.True(proc.Killed);
        Assert.Null(_env.Manager.Get("delta"));
        Assert.False(Directory.Exists(Path.Combine(_env.PluginsDir, "delta")));
        Assert.False(File.Exists(_env.Manager.ConfigStore.PathFor("delta")));
        Assert.False(await _env.Manager.UninstallAsync("delta"));
    }

    [Fact]
    public async Task DownloadExcludesVenvAndLogs()
    {
        _env.Manager.Discover();
        Assert.True((await _env.Manager.InstallAsync(PluginTestEnv.Zip(("plugin.json", ManifestJson("eps")), ("main.py", "")), false)).Ok);
        var dir = Path.Combine(_env.PluginsDir, "eps");
        Directory.CreateDirectory(Path.Combine(dir, ".venv", "bin"));
        File.WriteAllText(Path.Combine(dir, ".venv", "bin", "python"), "");
        File.WriteAllText(Path.Combine(dir, "plugin.log.1"), "old");

        var bytes = _env.Manager.DownloadZip("eps")!;
        using var zip = new ZipArchive(new MemoryStream(bytes));
        var names = zip.Entries.Select(e => e.FullName).ToList();
        Assert.Contains("plugin.json", names);
        Assert.Contains("main.py", names);
        Assert.DoesNotContain(names, n => n.StartsWith(".venv") || n.StartsWith("plugin.log"));
        Assert.Null(_env.Manager.DownloadZip("nope"));
    }

    [Fact]
    public async Task ReloadAddsRemovesAndRefreshes()
    {
        _env.WritePlugin("one", PluginTestEnv.Manifest("one", "python", new { autoStart = false }));
        _env.Boot();
        Assert.Equal(PluginState.Stopped, _env.Manager.Get("one")!.State);

        _env.WritePlugin("two", PluginTestEnv.Manifest("two", "python"));
        _env.WritePlugin("one", PluginTestEnv.Manifest("one", "python", new { autoStart = false, name = "Renamed" }));
        var list = await _env.Manager.ReloadAsync();
        Assert.Equal(new[] { "one", "two" }, list.Select(h => h.Id));
        Assert.Equal("Renamed", _env.Manager.Get("one")!.Name);
        Assert.Equal(PluginState.Starting, _env.Manager.Get("two")!.State);

        var proc = _env.Launcher.Last;
        Directory.Delete(Path.Combine(_env.PluginsDir, "two"), true);
        list = await _env.Manager.ReloadAsync();
        Assert.Single(list);
        Assert.True(proc.Killed);
    }

    // ── commands ──────────────────────────────────────────────────────────────

    private JsonElement Send(string command, object? parameters = null)
    {
        var d = new CommandDispatcher();
        new PluginCommands(_env.Manager).Register(d);
        Assert.True(d.TryGet(command, out var handler));
        var msg = new CommandMessage
        {
            Type = "Command", Id = "t", Command = command,
            Params = parameters == null ? null : JsonSerializer.SerializeToElement(parameters),
        };
        return JsonSerializer.SerializeToElement(handler(msg).GetAwaiter().GetResult() ?? new { });
    }

    [Fact]
    public async Task CommandsExposeSummariesConfigAndLogs()
    {
        _env.WritePlugin("demo", PluginTestEnv.Manifest("demo", "external"));
        _env.Boot();

        var plugins = Send("GetPlugins").GetProperty("plugins");
        var s = plugins[0];
        Assert.Equal("demo", s.GetProperty("id").GetString());
        Assert.Equal("starting", s.GetProperty("state").GetString());
        Assert.Equal("external", s.GetProperty("runtime").GetString());
        Assert.Equal(1, s.GetProperty("stepCount").GetInt32());
        Assert.Equal(3, s.GetProperty("functionCount").GetInt32());
        Assert.True(s.GetProperty("hasConfig").GetBoolean());
        Assert.Equal("ok", s.GetProperty("statusState").GetString());

        var detail = Send("GetPlugin", new { id = "demo" }).GetProperty("plugin");
        Assert.Equal(_env.Manager.Get("demo")!.Token, detail.GetProperty("token").GetString()); // external → token shown
        Assert.Equal("COM3", detail.GetProperty("config").GetProperty("port").GetString());
        Assert.Equal("weigh", detail.GetProperty("manifest").GetProperty("steps")[0].GetProperty("id").GetString());

        var unknown = Send("GetPlugin", new { id = "zzz" });
        Assert.False(unknown.GetProperty("ok").GetBoolean());
        Assert.Equal("unknownPlugin", unknown.GetProperty("error").GetString());

        var bad = Send("SetPluginConfig", new { id = "demo", config = new { samples = 1000 } });
        Assert.False(bad.GetProperty("ok").GetBoolean());
        Assert.Equal("badConfig", bad.GetProperty("error").GetString());
        Assert.Equal("samples", bad.GetProperty("field").GetString());
        Assert.False(string.IsNullOrEmpty(bad.GetProperty("message").GetString()));

        var c = await _env.ConnectReadyAsync("demo");
        c.Handlers["config.changed"] = _ => null;
        var good = Send("SetPluginConfig", new { id = "demo", config = new { samples = 7 } });
        Assert.Equal("running", good.GetProperty("plugin").GetProperty("state").GetString());
        var changed = await c.WaitForRequestAsync("config.changed");
        Assert.Equal(7, changed.GetProperty("params").GetProperty("config").GetProperty("samples").GetInt32());
        Assert.Equal("COM3", changed.GetProperty("params").GetProperty("config").GetProperty("port").GetString());

        var contrib = Send("GetPluginContributions");
        Assert.Equal("weigh", contrib.GetProperty("steps")[0].GetProperty("step").GetProperty("id").GetString());
        Assert.True(contrib.GetProperty("steps")[0].GetProperty("running").GetBoolean());
        Assert.Equal("twice", contrib.GetProperty("functions")[0].GetProperty("function").GetProperty("name").GetString());
        Assert.Equal("weight", contrib.GetProperty("properties")[0].GetProperty("property").GetProperty("name").GetString());

        var logs = Send("GetPluginLogs", new { id = "demo" });
        Assert.True(logs.GetProperty("totalCount").GetInt32() > 0);
        Assert.Equal(0, logs.GetProperty("start").GetInt32());
        Send("ClearPluginLogs", new { id = "demo" });
        Assert.Equal(0, Send("GetPluginLogs", new { id = "demo" }).GetProperty("logs").GetArrayLength());

        var oldToken = _env.Manager.Get("demo")!.Token;
        Send("RotatePluginToken", new { id = "demo" });
        Assert.NotEqual(oldToken, _env.Manager.Get("demo")!.Token);
        await c.Closed.WaitAsync(TimeSpan.FromSeconds(5));        // external plugin must reconnect
        Assert.Equal(4401, c.Transport.PeerCloseCode);

        var disabled = Send("SetPluginEnabled", new { id = "demo", enabled = false });
        Assert.Equal("disabled", disabled.GetProperty("plugin").GetProperty("state").GetString());
        Assert.Equal("pluginDisabled", Send("StartPlugin", new { id = "demo" }).GetProperty("error").GetString());

        Assert.Equal(1, Send("ReloadPlugins").GetProperty("plugins").GetArrayLength());
        Send("UninstallPlugin", new { id = "demo" });
        Assert.Equal(0, Send("GetPlugins").GetProperty("plugins").GetArrayLength());

        var summary = _env.Manager.GetStatusSummary();
        Assert.Empty(summary);
    }
}
