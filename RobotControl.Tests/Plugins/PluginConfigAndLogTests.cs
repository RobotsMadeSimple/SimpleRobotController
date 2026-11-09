using System.Text.Json;
using Controller.RobotControl.Plugins;

namespace RobotControl.Tests.Plugins;

public class PluginConfigAndLogTests
{
    private static JsonElement J(object o) => JsonSerializer.SerializeToElement(o);

    [Fact]
    public void ConfigStoreRoundTripsAndDeletes()
    {
        var dir = Path.Combine(Path.GetTempPath(), "srccfg-" + Guid.NewGuid().ToString("N"));
        try
        {
            var store = new PluginConfigStore(dir);
            var file = store.LoadOrCreate("scale");
            Assert.True(file.Enabled);
            Assert.Equal(43, file.Token.Length); // 32 bytes base64url, no padding
            Assert.DoesNotContain('=', file.Token);
            Assert.True(File.Exists(store.PathFor("scale")));

            file.Enabled = false;
            file.Config["port"] = J("COM7");
            file.Config["samples"] = J(12);
            store.Save("scale", file);

            var again = new PluginConfigStore(dir).LoadOrCreate("scale");
            Assert.False(again.Enabled);
            Assert.Equal(file.Token, again.Token);
            Assert.Equal("COM7", again.Config["port"].GetString());
            Assert.Equal(12, again.Config["samples"].GetInt32());
            Assert.False(File.Exists(store.PathFor("scale") + ".tmp"));

            store.Delete("scale");
            Assert.False(File.Exists(store.PathFor("scale")));
            Assert.NotEqual(file.Token, store.LoadOrCreate("scale").Token);
        }
        finally { Directory.Delete(dir, true); }
    }

    [Fact]
    public void ConfigValidationAndDefaults()
    {
        var schema = new List<PluginConfigField>
        {
            new() { Key = "port", Type = "string", Default = J("COM3"), Required = true },
            new() { Key = "baud", Type = "enum", Default = J("9600"), Options = new() { "9600", "115200" } },
            new() { Key = "samples", Type = "number", Default = J(5), Min = 1, Max = 100 },
            new() { Key = "tare", Type = "boolean", Default = J(true) },
        };
        Assert.Null(PluginConfigValidator.Validate(schema, new Dictionary<string, JsonElement> { ["samples"] = J(7) }));
        Assert.Equal("samples", PluginConfigValidator.Validate(schema, new Dictionary<string, JsonElement> { ["samples"] = J(500) })!.Value.Field);
        Assert.Equal("samples", PluginConfigValidator.Validate(schema, new Dictionary<string, JsonElement> { ["samples"] = J("5") })!.Value.Field);
        Assert.Equal("baud", PluginConfigValidator.Validate(schema, new Dictionary<string, JsonElement> { ["baud"] = J("300") })!.Value.Field);
        Assert.Equal("tare", PluginConfigValidator.Validate(schema, new Dictionary<string, JsonElement> { ["tare"] = J(1) })!.Value.Field);
        Assert.Equal("nope", PluginConfigValidator.Validate(schema, new Dictionary<string, JsonElement> { ["nope"] = J(1) })!.Value.Field);
        Assert.Equal("port", PluginConfigValidator.Validate(schema, new Dictionary<string, JsonElement> { ["port"] = J("") })!.Value.Field);

        var merged = PluginConfigValidator.MergeDefaults(schema, new Dictionary<string, JsonElement> { ["samples"] = J(9) });
        Assert.Equal("COM3", merged["port"].GetString());
        Assert.Equal(9, merged["samples"].GetInt32());
        Assert.True(merged["tare"].GetBoolean());
    }

    [Fact]
    public void LogRingPagesLikeProgramLogs()
    {
        var log = new PluginLog("t", null, new FakeClock(), _ => { });
        for (int i = 0; i < 520; i++) log.Append("info", "line " + i);
        var (total, start, logs) = log.Get(null, null);
        Assert.Equal(520, total);
        Assert.Equal(20, start);           // the oldest 20 fell out of the 500-line ring
        Assert.Equal(500, logs.Count);
        Assert.EndsWith("[info] line 20", logs[0]);

        (total, start, logs) = log.Get(515, null);
        Assert.Equal(5, logs.Count);
        Assert.Equal(515, start);

        log.Clear();
        (total, start, logs) = log.Get(null, null);
        Assert.Equal(520, total);
        Assert.Empty(logs);
        Assert.Empty(log.Tail(3));
        log.Append("warn", "new");
        Assert.Equal(521, log.Get(null, null).Total);
        Assert.Single(log.Tail(3));
    }

    [Fact]
    public void LogMirrorsToConsoleRateLimitedAndRollsFiles()
    {
        var clock = new FakeClock();
        var console = new List<string>();
        var dir = Path.Combine(Path.GetTempPath(), "srclog-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        try
        {
            var path = Path.Combine(dir, "plugin.log");
            var log = new PluginLog("scale", path, clock, console.Add);
            for (int i = 0; i < 25; i++) log.Append("stdout", "x" + i);
            Assert.Equal(20, console.Count);
            Assert.All(console, l => Assert.StartsWith("[Plugin:scale] ", l));

            clock.Advance(1000);
            log.Append("info", "after");
            Assert.Contains(console, l => l.Contains("5 more line(s) suppressed"));
            Assert.EndsWith("after", console[^1]);

            // Rolling: a file at the size cap is moved to .1 before the next write.
            File.WriteAllText(path, new string('a', (int)PluginLog.MaxFileBytes));
            log.Append("info", "fresh");
            Assert.True(File.Exists(path + ".1"));
            Assert.Contains("fresh", File.ReadAllText(path));
            Assert.True(new FileInfo(path).Length < 1000);
        }
        finally { Directory.Delete(dir, true); }
    }

    [Fact]
    public void LauncherBuildsStartInfoAndParsesPythonVersion()
    {
        var env = new Dictionary<string, string> { ["SRC_PLUGIN_ID"] = "scale", ["SRC_PLUGIN_TOKEN"] = "tok" };
        var psi = PluginProcessLauncher.BuildStartInfo("python3", new[] { "main.py", "--x" }, "/plugins/scale", env);
        Assert.Equal(new[] { "main.py", "--x" }, psi.ArgumentList);
        Assert.Equal("/plugins/scale", psi.WorkingDirectory);
        Assert.Equal("scale", psi.Environment["SRC_PLUGIN_ID"]);
        Assert.True(psi.RedirectStandardOutput && psi.RedirectStandardError && !psi.UseShellExecute);

        Assert.Equal(new Version(3, 11, 4), PluginProcessLauncher.ParsePythonVersion("Python 3.11.4"));
        Assert.Equal(new Version(3, 9), PluginProcessLauncher.ParsePythonVersion("Python 3.9"));
        Assert.Null(PluginProcessLauncher.ParsePythonVersion("nope"));
        Assert.Null(PluginProcessLauncher.VenvPython(Path.GetTempPath()));
    }
}
