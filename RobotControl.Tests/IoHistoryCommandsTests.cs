using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;

namespace RobotControl.Tests;

/// <summary>
/// The IO history / recording commands through the dispatcher, driven against a real
/// RobotController + IoHistoryService with synthetic samples (no Nano hardware). Exercises the
/// exact JSON contract the app consumes: retention, live window, and the recording lifecycle.
/// </summary>
[Collection("RobotController")]
public sealed class IoHistoryCommandsTests : IDisposable
{
    private const string Ch = "io/vtest/4";
    private readonly RobotController _robot = new();   // ctor only — no threads/devices started
    private readonly CommandDispatcher _d = new();
    private readonly List<string> _created = new();

    public IoHistoryCommandsTests() => new IoCommands(_robot).Register(_d);

    public void Dispose()
    {
        foreach (var id in _created)
            try { Send("DeleteIoRecording", new { id }); } catch { /* already gone */ }
    }

    private JsonElement Send(string command, object? parameters = null)
    {
        Assert.True(_d.TryGet(command, out var handler), command);
        var msg = new CommandMessage
        {
            Type = "Command", Id = "t", Command = command,
            Params = parameters == null ? null : JsonSerializer.SerializeToElement(parameters),
        };
        var result = handler(msg).GetAwaiter().GetResult();
        return result == null ? JsonSerializer.SerializeToElement(new { }) : JsonSerializer.SerializeToElement(result);
    }

    private void FeedFrom(long startMs, int count, int stepMs = 1)
    {
        for (int i = 0; i < count; i++)
            _robot.IoHistory.Record(Ch, startMs + (long)i * stepMs, value: i, raw: i, unit: "g", label: "Cell");
    }

    [Fact]
    public void Retention_roundtrips_through_commands()
    {
        Assert.Equal(120_000, Send("SetIoHistoryRetention", new { maxRetentionMs = 120_000 })
            .GetProperty("maxRetentionMs").GetInt64());
        Assert.Equal(120_000, Send("GetIoHistoryConfig").GetProperty("maxRetentionMs").GetInt64());
    }

    [Fact]
    public void History_window_is_returned_and_decimated()
    {
        Send("SetIoHistoryRetention", new { maxRetentionMs = 600_000 });
        long t0 = 50_000_000;
        FeedFrom(t0, 2_000);

        var samples = Send("GetIoHistoryWindow",
            new { channelKey = Ch, fromUnixMs = t0, toUnixMs = t0 + 2_000, maxPoints = 100 })
            .GetProperty("window").GetProperty("samples");
        Assert.True(samples.GetArrayLength() is > 0 and <= 100);
    }

    [Fact]
    public void Recording_lifecycle_start_feed_stop_window_save_list_delete()
    {
        var started = Send("StartIoRecording", new { channelKey = Ch, name = "virtual test" }).GetProperty("recording");
        string id = started.GetProperty("id").GetString()!;
        _created.Add(id);
        Assert.True(started.GetProperty("active").GetBoolean());

        long start = started.GetProperty("startUnixMs").GetInt64();
        FeedFrom(start, 150);

        var stopped = Send("StopIoRecording", new { id }).GetProperty("recording");
        Assert.False(stopped.GetProperty("active").GetBoolean());
        Assert.Equal(150, stopped.GetProperty("sampleCount").GetInt32());

        Assert.Equal(150, Send("GetIoRecordingWindow", new { id, maxPoints = 1_000_000 })
            .GetProperty("window").GetProperty("samples").GetArrayLength());

        Assert.True(Send("SaveIoRecording", new { id }).GetProperty("recording").GetProperty("saved").GetBoolean());
        Assert.Contains(Send("ListIoRecordings").GetProperty("recordings").EnumerateArray(),
            e => e.GetProperty("id").GetString() == id);

        Send("DeleteIoRecording", new { id });
        _created.Remove(id);
        Assert.DoesNotContain(Send("ListIoRecordings").GetProperty("recordings").EnumerateArray(),
            e => e.GetProperty("id").GetString() == id);
    }

    [Fact]
    public void Unknown_recording_ids_surface_an_error()
        => Assert.ThrowsAny<Exception>(() => Send("StopIoRecording", new { id = "does-not-exist" }));
}
