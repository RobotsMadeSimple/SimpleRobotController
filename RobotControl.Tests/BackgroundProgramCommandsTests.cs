using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Persistence;

namespace RobotControl.Tests;

/// <summary>
/// StartBackgroundProgram / StopBackgroundProgram through the dispatcher, the way the app's
/// Background page calls them: by stable id first, then by name. A RobotController is
/// constructed but never started, and its program repository lives in a temp folder.
/// </summary>
[Collection("RobotController")]
public class BackgroundProgramCommandsTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "rms-bg-cmd-" + Guid.NewGuid().ToString("N"));
    private readonly BackgroundProgramManager _background;
    private readonly CommandDispatcher _dispatcher = new();

    public BackgroundProgramCommandsTests()
    {
        var robot    = new RobotController();
        robot.builtProgramRepo = new BuiltProgramRepository(_dir);
        var programs = new ProgramCycleManager();
        _background  = new BackgroundProgramManager(robot, programs, robot.pointRepo, robot.toolRepo,
            robot.localRepo, robot.builtProgramRepo, robot.gridRepo, robot.stackRepo);
        new BuiltProgramCommands(robot, programs, null, _background).Register(_dispatcher);
        new SystemCommands(robot, programs, _background).Register(_dispatcher);

        robot.builtProgramRepo.Save(new BuiltProgram
        {
            Id = "bg-1", Name = "Watcher", IsBackground = true,
            Steps = [new ProgramStep { Id = "s1", Type = StepType.PauseProgram }],
        });
    }

    public void Dispose()
    {
        _background.StopAll();
        try { Directory.Delete(_dir, true); } catch { /* best effort */ }
    }

    private JsonElement Send(string command, object parameters)
    {
        Assert.True(_dispatcher.TryGet(command, out var handler));
        var msg = new CommandMessage
        {
            Type = "Command", Id = "t", Command = command,
            Params = JsonSerializer.SerializeToElement(parameters),
        };
        return JsonSerializer.SerializeToElement(handler(msg).GetAwaiter().GetResult());
    }

    [Fact]
    public void StopByIdIgnoresAStaleName()
    {
        Send("StartBackgroundProgram", new { programId = "bg-1", programName = "Old name" });
        Assert.True(_background.IsRunning("bg-1"));

        Send("StopBackgroundProgram", new { programId = "bg-1", programName = "Old name" });
        Assert.False(_background.IsRunning("bg-1"));
    }

    [Fact]
    public void StopFallsBackToTheName()
    {
        Send("StartBackgroundProgram", new { programName = "Watcher" });
        Assert.True(_background.IsRunning("bg-1"));

        Send("StopBackgroundProgram", new { programName = "Watcher" });
        Assert.False(_background.IsRunning("bg-1"));
    }

    [Fact]
    public void StopWithAnUnknownIdLeavesOthersRunning()
    {
        Send("StartBackgroundProgram", new { programId = "bg-1" });
        Send("StopBackgroundProgram", new { programId = "nope" });
        Assert.True(_background.IsRunning("bg-1"));
    }

    [Fact]
    public void StatusCarriesTheRunningProgramsId()
    {
        Send("StartBackgroundProgram", new { programId = "bg-1" });
        var bg = Send("GetStatus", new { }).GetProperty("backgroundPrograms");
        Assert.Equal(1, bg.GetArrayLength());
        Assert.Equal("bg-1", bg[0].GetProperty("id").GetString());
        Assert.Equal("Watcher", bg[0].GetProperty("name").GetString());
    }
}
