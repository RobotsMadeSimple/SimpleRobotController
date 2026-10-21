using System.Text.Json;
using Controller.RobotControl.Plugins;

namespace RobotControl.Tests.Plugins;

/// <summary>The custom-GUI surface: manifest <c>ui</c>, the <c>view.set</c>/<c>view.clear</c> store, and <c>ui.action</c> routing.</summary>
public class PluginUiTests : IDisposable
{
    private readonly PluginTestEnv _env = new();

    public PluginUiTests()
    {
        _env.WritePlugin("demo", PluginTestEnv.Manifest("demo", "external", new { ui = new { entry = "ui/index.html", title = "Demo" } }));
        _env.Boot();
    }

    public void Dispose() => _env.Dispose();

    private PluginHost Demo => _env.Manager.Get("demo")!;

    [Fact]
    public void ManifestUiLoadsAndSurfaces()
    {
        Assert.Empty(Demo.Problems);
        Assert.True(Demo.Manifest!.HasUi);
        Assert.Equal("ui/index.html", Demo.Manifest!.Ui!.Entry);
    }

    [Fact]
    public void UiEntryOutsideFolderIsRejected()
    {
        var m = PluginManifest.Parse(JsonSerializer.Serialize(
            PluginTestEnv.Manifest("bad", "external", new { ui = new { entry = "../escape.html" } }), PluginJson.Options));
        Assert.Contains(m.Validate(), p => p.Field == "ui.entry");
    }

    [Fact]
    public async Task ViewSetMergesAndClears()
    {
        var c = await _env.ConnectReadyAsync("demo");
        Assert.Equal(0, Demo.ViewSnapshot().Version);

        await c.EventAsync("view.set", new { state = new { turn = 1, selected = -1 } });
        await Wait.UntilAsync(() => Demo.ViewSnapshot().Version > 0, "view.set");
        using (var doc = JsonDocument.Parse(Demo.ViewSnapshot().Json))
        {
            Assert.Equal(1, doc.RootElement.GetProperty("turn").GetInt32());
            Assert.Equal(-1, doc.RootElement.GetProperty("selected").GetInt32());
        }

        // A second view.set shallow-merges: selected changes, turn is kept.
        long v1 = Demo.ViewSnapshot().Version;
        await c.EventAsync("view.set", new { state = new { selected = 5 } });
        await Wait.UntilAsync(() => Demo.ViewSnapshot().Version > v1, "view.set merge");
        using (var doc = JsonDocument.Parse(Demo.ViewSnapshot().Json))
        {
            Assert.Equal(1, doc.RootElement.GetProperty("turn").GetInt32());
            Assert.Equal(5, doc.RootElement.GetProperty("selected").GetInt32());
        }

        await c.EventAsync("view.clear", new { });
        await Wait.UntilAsync(() => Demo.ViewSnapshot().Json == "{}", "view.clear");
    }

    [Fact]
    public async Task InvokeUiActionRoundTrips()
    {
        var c = await _env.ConnectReadyAsync("demo");
        c.Handlers["ui.action"] = p => new
        {
            echoedAction = p.GetProperty("actionId").GetString(),
            echoedIndex = p.GetProperty("args").GetProperty("index").GetInt32(),
        };

        var args = JsonSerializer.SerializeToElement(new { index = 7 }, PluginJson.Options);
        var result = await Demo.InvokeUiActionAsync("tapSquare", args);

        Assert.Equal("tapSquare", result.GetProperty("echoedAction").GetString());
        Assert.Equal(7, result.GetProperty("echoedIndex").GetInt32());
    }

    [Fact]
    public async Task InvokeUiActionFailsWhenNotRunning()
    {
        // demo is external and not connected yet → not running.
        var args = JsonSerializer.SerializeToElement(new { }, PluginJson.Options);
        await Assert.ThrowsAsync<PluginProtocolException>(() => Demo.InvokeUiActionAsync("x", args));
    }
}
