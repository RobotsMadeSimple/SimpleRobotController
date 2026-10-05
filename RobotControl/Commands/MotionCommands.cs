namespace Controller.RobotControl.Commands;

/// <summary>
/// Homing, stops, fault recovery and the queued motion commands (moves, jogs,
/// speed/accel settings) that are executed on the motion thread by RunCommands.
/// </summary>
internal sealed class MotionCommands
{
    private readonly RobotController _robot;

    public MotionCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("Home",       _ => _robot.RequestHome());
        d.Add("SetHomed",   _ => _robot.RequestSetHomed());
        d.Add("Reset",      _ => _robot.stb.Reset());
        d.Add("HardStop",   _ => _robot.HardStop());
        d.Add("ClearFault", _ => _robot.ClearFault());
        d.Add("StopJog",    _ => _robot.StopJog());

        // Queued on RobotController.QueuedCommands and executed on the motion thread.
        foreach (var name in RobotController.QueuedMotionCommandNames)
            d.Add(name, _robot.EnqueueMotionCommand);
    }
}
