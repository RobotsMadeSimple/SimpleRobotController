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
        d.Add("Home",            _ => _robot.RequestHome());
        d.Add("SetHomed",        _ => _robot.RequestSetHomed());
        d.Add("SetJointPosition", SetJointPosition);
        d.Add("Reset",           _ => _robot.stb.Reset());
        d.Add("HardStop",        _ => _robot.HardStop());
        d.Add("ClearFault",      _ => _robot.ClearFault());
        d.Add("StopJog",         _ => _robot.StopJog());

        // Queued on RobotController.QueuedCommands and executed on the motion thread.
        foreach (var name in RobotController.QueuedMotionCommandNames)
            d.Add(name, _robot.EnqueueMotionCommand);
    }

    /// <summary>
    /// Declares one joint to be at a known value now — manual homing of a single joint, no
    /// motion. Refused while the robot is moving or homing, which would change the pose out
    /// from under the value being declared.
    /// </summary>
    private object? SetJointPosition(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetJointPositionParams>(msg);
        if (p.Joint is < 0 or > 3)
            return new { ok = false, error = "joint must be 0..3 (0=J1/X, 1=Horizontal/Y, 2=Vertical/Z, 3=J4/RZ)" };
        if (_robot.IsMoving || _robot.MotionBusy || _robot.HomingRequestedOrActive)
            return new { ok = false, error = "Cannot set a joint position while the robot is moving or homing." };

        _robot.RequestSetJointPosition(p.Joint, p.Value);
        return new { ok = true, joint = p.Joint, value = p.Value };
    }
}
