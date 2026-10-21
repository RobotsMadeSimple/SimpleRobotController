using Controller.RobotControl.History;

namespace RobotControl.Tests;

/// <summary>IoHistoryService — rolling retention, recordings, decimation, and disk persistence.</summary>
public class IoHistoryServiceTests : IDisposable
{
    private readonly string _dir;
    private const string Ch = "nano/n1/4";

    public IoHistoryServiceTests()
    {
        _dir = Path.Combine(Path.GetTempPath(), "iohistory-tests-" + Guid.NewGuid().ToString("N"));
    }

    public void Dispose()
    {
        try { if (Directory.Exists(_dir)) Directory.Delete(_dir, recursive: true); } catch { }
    }

    private static void Feed(IoHistoryService svc, string ch, long startMs, int count, int stepMs = 10)
    {
        for (int i = 0; i < count; i++)
            svc.Record(ch, startMs + (long)i * stepMs, value: i, raw: i, unit: "g", label: "Cell");
    }

    // ── Rolling retention ────────────────────────────────────────────────────

    [Fact]
    public void Retention_evicts_samples_older_than_the_window()
    {
        var svc = new IoHistoryService(_dir);
        svc.SetRetention(1_000); // keep 1 s

        long t0 = 10_000_000;
        // 200 samples spanning 2 s (10 ms apart). The newest is at t0+1990; retention 1 s means
        // everything older than t0+990 should be gone.
        Feed(svc, Ch, t0, 200, stepMs: 10);

        var all = svc.GetSince(Ch, 0, 10_000);
        Assert.NotEmpty(all);
        Assert.True(all[0].T >= t0 + 990, $"oldest retained sample {all[0].T} should be within 1 s of the newest");
        Assert.True(all.Count <= 102, $"expected ~100 retained, got {all.Count}");
    }

    // ── Recording capture + retention exemption ───────────────────────────────

    [Fact]
    public void Recording_keeps_full_slice_even_when_retention_is_shorter()
    {
        var svc = new IoHistoryService(_dir);
        svc.SetRetention(100); // tiny: 100 ms rolling window

        long t0 = 20_000_000;
        var rec = svc.StartRecording(Ch, "force test");
        Assert.True(rec.Active);

        // The recording started at ~now (real clock). Record 300 samples stamped at the recording's
        // start time onward so they fall inside [start, now]; they must survive despite 100 ms retention.
        Feed(svc, Ch, rec.StartUnixMs, 300, stepMs: 1);

        var stopped = svc.StopRecording(rec.Id);
        Assert.NotNull(stopped);
        Assert.False(stopped!.Active);
        Assert.Equal(300, stopped.SampleCount);

        // Full recording is still retrievable even though the rolling buffer would have evicted most.
        var w = svc.GetRecordingWindow(rec.Id, null, null, maxPoints: 10_000);
        Assert.NotNull(w);
        Assert.Equal(300, w!.Samples.Count);
    }

    [Fact]
    public void Active_recording_reports_live_sample_count_in_list()
    {
        var svc = new IoHistoryService(_dir);
        var rec = svc.StartRecording(Ch, null);
        Feed(svc, Ch, rec.StartUnixMs, 50, stepMs: 1);

        var listed = svc.ListRecordings().Single(r => r.Id == rec.Id);
        Assert.True(listed.Active);
        Assert.Equal(50, listed.SampleCount);
    }

    // ── Window decimation ──────────────────────────────────────────────────────

    [Fact]
    public void GetWindow_decimates_to_the_point_budget()
    {
        var svc = new IoHistoryService(_dir);
        svc.SetRetention(60_000);
        long t0 = 30_000_000;
        Feed(svc, Ch, t0, 5_000, stepMs: 1);

        var w = svc.GetWindow(Ch, t0, t0 + 5_000, maxPoints: 200);
        Assert.True(w.Samples.Count <= 200, $"expected <= 200 points, got {w.Samples.Count}");
        Assert.True(w.Samples.Count > 0);
        // Min/max decimation must preserve the global extremes (values 0..4999).
        Assert.Contains(w.Samples, s => s.V == 0);
        Assert.Contains(w.Samples, s => s.V == 4999);
    }

    [Fact]
    public void GetSince_returns_only_newer_samples()
    {
        var svc = new IoHistoryService(_dir);
        svc.SetRetention(60_000);
        long t0 = 40_000_000;
        Feed(svc, Ch, t0, 100, stepMs: 10);

        var since = svc.GetSince(Ch, t0 + 500, 10_000);
        Assert.All(since, s => Assert.True(s.T > t0 + 500));
        Assert.Equal(49, since.Count); // t0+510 .. t0+990 inclusive
    }

    // ── Disk persistence ────────────────────────────────────────────────────────

    [Fact]
    public void Saved_recording_survives_a_new_service_instance()
    {
        string id;
        {
            var svc = new IoHistoryService(_dir);
            var rec = svc.StartRecording(Ch, "persisted");
            Feed(svc, Ch, rec.StartUnixMs, 120, stepMs: 1);
            svc.StopRecording(rec.Id);
            var saved = svc.SaveRecording(rec.Id);
            Assert.NotNull(saved);
            Assert.True(saved!.Saved);
            id = rec.Id;
        }

        // A fresh instance loads the index; the saved recording is listed and its samples lazy-load.
        var svc2 = new IoHistoryService(_dir);
        var listed = svc2.ListRecordings().SingleOrDefault(r => r.Id == id);
        Assert.NotNull(listed);
        Assert.True(listed!.Saved);

        var w = svc2.GetRecordingWindow(id, null, null, maxPoints: 10_000);
        Assert.NotNull(w);
        Assert.Equal(120, w!.Samples.Count);
    }

    [Fact]
    public void Deleting_a_recording_removes_it()
    {
        var svc = new IoHistoryService(_dir);
        var rec = svc.StartRecording(Ch, null);
        svc.StopRecording(rec.Id);

        Assert.True(svc.DeleteRecording(rec.Id));
        Assert.DoesNotContain(svc.ListRecordings(), r => r.Id == rec.Id);
        Assert.False(svc.DeleteRecording(rec.Id)); // already gone
    }

    [Fact]
    public void Retention_is_persisted_and_reloaded()
    {
        { var svc = new IoHistoryService(_dir); svc.SetRetention(45_000); }
        var svc2 = new IoHistoryService(_dir);
        Assert.Equal(45_000, svc2.MaxRetentionMs);
    }
}
