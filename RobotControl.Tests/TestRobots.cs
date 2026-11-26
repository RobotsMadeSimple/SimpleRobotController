using Controller.RobotControl;

namespace RobotControl.Tests;

internal static class TestRobots
{
    /// <summary>
    /// Switches a never-connected test controller to identity (CNC4Axis) kinematics. Program-logic
    /// tests use small coordinates around the origin, which the ASTRO arm cannot physically reach
    /// (they sit inside its J4 offset circle), so the pre-move reach check would fail them with
    /// <c>outOfReach</c>. With joints = axes every target is reachable and the check stays out of
    /// the way of what those tests are about.
    /// </summary>
    public static RobotController WithIdentityKinematics(this RobotController robot)
    {
        robot.SetConfig(new RobotConfig { RobotType = RobotTypes.Cnc4Axis });
        return robot;
    }
}
