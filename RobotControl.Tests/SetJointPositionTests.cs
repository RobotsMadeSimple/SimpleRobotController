using Controller.RobotControl;
using Xunit;

namespace RobotControl.Tests;

/// <summary>
/// SetJointPosition declares one joint's value with no motion (manual homing of a single
/// joint). Referenced state is per-joint, and the robot counts as homed only once all four
/// joints are referenced. A RobotController is constructed but never started — no devices,
/// threads or motion.
/// </summary>
public class SetJointPositionTests
{
    [Fact]
    public void FreshControllerIsUnreferencedAndNotHomed()
    {
        var robot = new RobotController();
        Assert.False(robot.Homed);
        Assert.All(robot.JointReferenced, r => Assert.False(r));
    }

    [Fact]
    public void AstroSetJointWritesValueAndReferencesOnlyThatJoint()
    {
        var robot = new RobotController(); // defaults to ASTRO
        robot.SetJointPosition(0, 30);

        // The value round-trips through FK→IK→motor targets (same path homing uses), so it
        // snaps to the nearest motor step — a fraction of a degree under the test's default
        // step config. Confirm it was applied, not held at the default.
        var a = robot.Kinematics.GetJointAngles();
        Assert.True(System.Math.Abs(a.joint1 - 30) < 1.0, $"J1 readout {a.joint1}");
        Assert.True(robot.JointReferenced[0]);
        Assert.False(robot.JointReferenced[1]);
        Assert.False(robot.JointReferenced[2]);
        Assert.False(robot.JointReferenced[3]);
        Assert.False(robot.Homed);
        Assert.False(robot.IsMoving);   // no motion was started
    }

    [Fact]
    public void AstroHomedOnlyAfterAllFourReferenced()
    {
        var robot = new RobotController();
        robot.SetJointPosition(0, 10);
        robot.SetJointPosition(1, 120);
        robot.SetJointPosition(2, 40);
        Assert.False(robot.Homed);

        robot.SetJointPosition(3, 45);
        Assert.True(robot.Homed);

        // Readouts snap to the nearest motor step (see note above); confirm each joint landed
        // near its declared value.
        var a = robot.Kinematics.GetJointAngles();
        Assert.True(System.Math.Abs(a.joint1  - 10)  < 1.0, $"J1 {a.joint1}");
        Assert.True(System.Math.Abs(a.joint2x - 120) < 1.0, $"Horizontal {a.joint2x}");
        Assert.True(System.Math.Abs(a.joint2z - 40)  < 1.0, $"Vertical {a.joint2z}");
        Assert.True(System.Math.Abs(a.joint4  - 45)  < 1.0, $"J4 {a.joint4}");
    }

    [Fact]
    public void CncSetJointReferencesAndReportsValue()
    {
        var robot = new RobotController();
        robot.SetConfig(new RobotConfig { RobotType = "CNC4Axis" });
        Assert.False(robot.Homed);

        robot.SetJointPosition(0, 12.5);
        Assert.True(robot.JointReferenced[0]);
        Assert.Equal(12.5, robot.Kinematics.GetJointAngles().joint1, 3);
        Assert.False(robot.IsMoving);

        robot.SetJointPosition(1, 5);
        robot.SetJointPosition(2, -3);
        Assert.False(robot.Homed);

        robot.SetJointPosition(3, 0);
        Assert.True(robot.Homed);
    }

    [Fact]
    public void SwitchingRobotTypeClearsReferencedState()
    {
        var robot = new RobotController();
        robot.SetJointPosition(0, 10);
        robot.SetJointPosition(1, 120);
        robot.SetJointPosition(2, 40);
        robot.SetJointPosition(3, 45);
        Assert.True(robot.Homed);

        robot.SetConfig(new RobotConfig { RobotType = "CNC4Axis" });
        Assert.False(robot.Homed);
        Assert.All(robot.JointReferenced, r => Assert.False(r));
    }

    [Fact]
    public void JointOutOfRangeThrows()
    {
        var robot = new RobotController();
        Assert.Throws<System.ArgumentOutOfRangeException>(() => robot.SetJointPosition(4, 0));
    }
}
