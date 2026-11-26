using Controller.RobotControl.Alarms;
using Controller.RobotControl.Programs;

namespace RobotControl.Tests;

/// <summary>
/// The alarm history: bounded to the newest entries (never dropping a present condition), persisted
/// to alarms.json, condition alarms deduplicated and cleared, acknowledge / clear, and the page
/// order (active first). Plus the catalog's coverage of every program error code.
/// </summary>
public class AlarmHistoryTests : IDisposable
{
    private readonly string _file = Path.Combine(Path.GetTempPath(), "rms-alarm-test-" + Guid.NewGuid().ToString("N") + ".json");

    public void Dispose()
    {
        try { File.Delete(_file); } catch { }
        try { File.Delete(_file + ".tmp"); } catch { }
    }

    [Fact]
    public void HistoryIsBoundedToTheNewestEntries()
    {
        var h = new AlarmHistory(_file, maxEntries: 10);
        for (int i = 0; i < 25; i++)
            h.Raise(ErrorCodes.UserError, $"alarm {i}", cleared: true, nowUnixMs: 1000 + i);
        Assert.Equal(10, h.Count);
        var page = h.GetPage(0, 100).Page;
        Assert.Equal("alarm 24", page[0].Message);             // newest first
        Assert.DoesNotContain(page, a => a.Message == "alarm 14");
    }

    [Fact]
    public void TheBoundNeverDropsAConditionThatIsStillPresent()
    {
        var h = new AlarmHistory(_file, maxEntries: 5);
        var estop = h.Raise(ErrorCodes.EmergencyStop, "pressed", conditionKey: "estop", nowUnixMs: 1);
        for (int i = 0; i < 20; i++)
            h.Raise(ErrorCodes.UserError, $"e{i}", cleared: true, nowUnixMs: 10 + i);
        var all = h.GetPage(0, 100).Page;
        Assert.Contains(all, a => a.Id == estop.Id);
        Assert.Equal(5, h.Count);
    }

    [Fact]
    public void HistoryPersistsAcrossInstances()
    {
        var h = new AlarmHistory(_file);
        var a = h.Raise(ErrorCodes.JointLimit, "J2 reached its upper limit.", new AlarmSource { DeviceId = "stb" }, conditionKey: "jointLimit");
        h.Raise(ErrorCodes.WaitTimeout, "Main: Wait timed out", new AlarmSource { ProgramId = "p1", StepId = "s7" }, cleared: true);
        h.Acknowledge(a.Id);
        h.Flush();

        var back = new AlarmHistory(_file);
        Assert.Equal(2, back.Count);
        var page = back.GetPage(0, 10).Page;
        var jl = page.Single(x => x.Code == ErrorCodes.JointLimit);
        Assert.True(jl.Acknowledged);
        Assert.False(jl.Cleared);
        Assert.Equal("stb", jl.Source.DeviceId);
        var wt = page.Single(x => x.Code == ErrorCodes.WaitTimeout);
        Assert.Equal("p1", wt.Source.ProgramId);
        Assert.Equal("s7", wt.Source.StepId);
        Assert.Equal("Wait timed out", wt.Title);              // title from the catalog
        Assert.True(back.LastUpdatedUnixMs > 0);

        // The reloaded condition alarm still clears by key.
        Assert.Equal(1, back.ClearCondition("jointLimit"));
    }

    [Fact]
    public void DebouncedWriteLandsOnDiskOffTheCallersThread()
    {
        var h = new AlarmHistory(_file);
        h.Raise(ErrorCodes.UserError, "background write", cleared: true);
        var deadline = DateTime.UtcNow.AddSeconds(5);
        while (DateTime.UtcNow < deadline && !(File.Exists(_file) && File.ReadAllText(_file).Contains("background write")))
            Thread.Sleep(20);
        Assert.Contains("background write", File.ReadAllText(_file));
    }

    [Fact]
    public void ConditionAlarmsDeduplicateAndClear()
    {
        var h = new AlarmHistory(_file);
        var first  = h.Raise(AlarmCatalog.DeviceDisconnected, "nano1 disconnected", conditionKey: "device:nano1");
        var second = h.Raise(AlarmCatalog.DeviceDisconnected, "nano1 disconnected", conditionKey: "device:nano1");
        Assert.Equal(first.Id, second.Id);
        Assert.Equal(1, h.Count);

        h.ClearCondition("device:nano1");
        var again = h.Raise(AlarmCatalog.DeviceDisconnected, "nano1 disconnected", conditionKey: "device:nano1");
        Assert.NotEqual(first.Id, again.Id);                     // a new occurrence
        Assert.Equal(2, h.Count);
    }

    [Fact]
    public void ActiveUntilClearedAndAcknowledged_ActiveListedFirst()
    {
        var h = new AlarmHistory(_file);
        var old    = h.Raise(ErrorCodes.UserError, "old", cleared: true, nowUnixMs: 1);
        var warn   = h.Raise(AlarmCatalog.PluginCrashed, "plugin crashed", conditionKey: "plugin:x", nowUnixMs: 2);
        var estop  = h.Raise(ErrorCodes.EmergencyStop, "pressed", conditionKey: "estop", nowUnixMs: 3);
        h.Acknowledge(old.Id);
        Assert.Equal(2, h.ActiveCount);

        var page = h.GetPage(0, 10).Page;
        Assert.Equal(estop.Id, page[0].Id);                      // critical before warning
        Assert.Equal(warn.Id,  page[1].Id);
        Assert.Equal(old.Id,   page[2].Id);                      // inactive history after

        Assert.Single(h.GetPage(0, 10, activeOnly: true).Page, a => a.Id == estop.Id);
        Assert.Equal(-1, h.Acknowledge("nope"));
        Assert.Equal(2, h.Acknowledge(null));                    // acknowledge all
        Assert.Equal(2, h.ActiveCount);                          // conditions still present

        // Clear removes only what has returned to normal.
        Assert.Equal(1, h.ClearHistory());
        Assert.Equal(2, h.Count);
        h.ClearCondition("estop");
        Assert.Equal(1, h.ActiveCount);
    }

    [Fact]
    public void PagingSkipsAndTakes()
    {
        var h = new AlarmHistory(_file);
        for (int i = 0; i < 12; i++) h.Raise(ErrorCodes.UserError, $"a{i}", cleared: true, nowUnixMs: i + 1);
        h.Acknowledge(null);
        var (total, active, page) = h.GetPage(5, 5);
        Assert.Equal(12, total);
        Assert.Equal(0, active);
        Assert.Equal(new[] { "a6", "a5", "a4", "a3", "a2" }, page.Select(a => a.Message));
    }

    [Fact]
    public void CatalogCoversEveryProgramErrorCode()
    {
        foreach (var spec in ErrorCodes.All)
        {
            var e = AlarmCatalog.Get(spec.Code);
            Assert.False(string.IsNullOrWhiteSpace(e.Title));
            Assert.False(string.IsNullOrWhiteSpace(e.Cause));
            Assert.False(string.IsNullOrWhiteSpace(e.Remedy));
            Assert.Equal(spec.Catchable, e.Catchable);
        }
        // The 23 codes this catalog was written against all have hand-written help.
        foreach (var code in new[] { "waitTimeout", "visionTimeout", "visionFailed", "httpFailed", "httpTimeout", "receiveTimeout",
                                     "pluginFailed", "pluginTimeout", "pluginNotRunning", "promptTimeout", "pointNotFound",
                                     "arcDegenerate", "indexOutOfRange", "userError", "motionFault", "jointLimit",
                                     "driverDisconnected", "emergencyStop", "userStop", "unknownVariable", "expressionSyntax",
                                     "computedVariable", "badListOperation" })
            Assert.True(AlarmCatalog.Has(code), code);
        Assert.True(AlarmCatalog.Has(AlarmCatalog.DeviceDisconnected));
        Assert.True(AlarmCatalog.Has(AlarmCatalog.PluginCrashed));
        Assert.Equal(AlarmSeverity.Critical, AlarmCatalog.Get(ErrorCodes.EmergencyStop).Severity);
        // A program's own code gets a generic, catchable entry.
        Assert.True(AlarmCatalog.Get("partMissing").Catchable);
    }
}
