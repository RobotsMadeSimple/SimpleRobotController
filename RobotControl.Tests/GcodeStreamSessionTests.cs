using System.Threading;
using Controller.RobotControl;
using Controller.RobotControl.Gcode;

namespace RobotControl.Tests;

/// <summary>The streaming session translates lines to queued moves and guards the motion queue.
/// The controller is constructed but not Start()ed, so no motion thread drains the queue.</summary>
public class GcodeStreamSessionTests
{
    [Fact]
    public void FeedTranslatesAbsoluteMoveToAQueuedMoveL()
    {
        var robot = new RobotController();
        Assert.True(GcodeStreamSession.TryCreate(robot, out var session, out var err), err);
        using (session!)
        {
            Assert.Equal("ok", session.Feed("G21 G90", CancellationToken.None)); // no motion
            Assert.True(robot.QueuedCommands.IsEmpty);

            Assert.Equal("ok", session.Feed("G1 X10 Y5 F600", CancellationToken.None));
            Assert.True(robot.QueuedCommands.TryDequeue(out var cmd));
            Assert.Equal("MoveL", cmd!.CommandType);
            Assert.Equal(10, cmd.X);
            Assert.Equal(5, cmd.Y);
            Assert.Equal(10.0, cmd.Speed!.Value, 6); // 600 mm/min ÷ 60
        }
    }

    [Fact]
    public void BadLineReturnsErrorNotOk()
    {
        var robot = new RobotController();
        Assert.True(GcodeStreamSession.TryCreate(robot, out var session, out _));
        using (session!)
        {
            var reply = session.Feed("G99 X1", CancellationToken.None);
            Assert.StartsWith("error:", reply);
        }
    }

    [Fact]
    public void SecondSessionIsRefusedWhileOneIsActive()
    {
        var robot = new RobotController();
        Assert.True(GcodeStreamSession.TryCreate(robot, out var first, out _));
        using (first!)
        {
            Assert.False(GcodeStreamSession.TryCreate(robot, out var second, out var err));
            Assert.Null(second);
            Assert.Contains("active", err);
        }
        // Freed on dispose — a new one can open.
        Assert.True(GcodeStreamSession.TryCreate(robot, out var third, out _));
        third!.Dispose();
    }

    [Fact]
    public void DisposeClearsTheActiveFlag()
    {
        var robot = new RobotController();
        GcodeStreamSession.TryCreate(robot, out var session, out _);
        Assert.True(robot.GcodeStreamActive);
        session!.Dispose();
        Assert.False(robot.GcodeStreamActive);
    }
}
