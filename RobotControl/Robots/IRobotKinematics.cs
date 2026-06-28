namespace Controller.RobotControl.Robots
{
    internal interface IRobotKinematics
    {
        string RobotTypeName { get; }

        /// <summary>Compute TCP position from current internal joint state and tool offset.</summary>
        Vector6 ForwardKinematics(Vector6 toolOffset);

        /// <summary>Compute joint-space targets from a TCP target and tool offset.</summary>
        Vector6 InverseKinematics(Vector6 tcp, Vector6 toolOffset);

        /// <summary>Drive motor targets from joint-space targets; updates internal joint state.</summary>
        void UpdateMotorTargets(Vector6 joints, out double m1Deg, out double m2Deg, out double m3Deg, out double m4Deg);

        /// <summary>Returns a pose vector for 3D robot visualisation, or null if not applicable.</summary>
        Vector6? GetVisualRobotPose(Vector6 currentTcp, Vector6 toolOffset);

        /// <summary>Returns current joint angles (or Cartesian equivalents) for status reporting.</summary>
        (double joint1, double joint2x, double joint2z, double joint4) GetJointAngles();
    }
}
