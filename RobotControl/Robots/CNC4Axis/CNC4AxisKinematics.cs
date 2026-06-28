using Controller.RobotControl.Robots;

namespace Controller.RobotControl.Robots.CNC4Axis
{
    /// <summary>
    /// Kinematics for a 4-axis CNC machine: X, Y, Z linear axes + RZ rotary (threading spindle).
    /// Joints ARE the Cartesian position — FK and IK are identity transforms.
    /// Motor degrees are scaled from mm/deg via configurable ratios.
    /// </summary>
    internal class CNC4AxisKinematics : IRobotKinematics
    {
        public string RobotTypeName => "CNC4Axis";

        // Conversion ratios: motor shaft degrees per unit of joint travel
        public double MotorDegsPerMmX  { get; set; } = 1.0;
        public double MotorDegsPerMmY  { get; set; } = 1.0;
        public double MotorDegsPerMmZ  { get; set; } = 1.0;
        public double MotorDegsPerDegRZ { get; set; } = 1.0;

        // Current joint / Cartesian state (updated by UpdateMotorTargets)
        public double CurrentX  { get; private set; }
        public double CurrentY  { get; private set; }
        public double CurrentZ  { get; private set; }
        public double CurrentRZ { get; private set; }

        // FK: joints == Cartesian for a CNC machine
        public Vector6 ForwardKinematics(Vector6 toolOffset)
        {
            return new Vector6(
                CurrentX  + toolOffset.X,
                CurrentY  + toolOffset.Y,
                CurrentZ  + toolOffset.Z,
                0,
                0,
                CurrentRZ + toolOffset.RZ
            );
        }

        // IK: trivial — Cartesian IS joint space
        public Vector6 InverseKinematics(Vector6 tcp, Vector6 toolOffset)
        {
            return new Vector6(
                tcp.X  - toolOffset.X,
                tcp.Y  - toolOffset.Y,
                tcp.Z  - toolOffset.Z,
                0,
                0,
                tcp.RZ - toolOffset.RZ
            );
        }

        public void UpdateMotorTargets(
            Vector6 joints,
            out double m1Deg,
            out double m2Deg,
            out double m3Deg,
            out double m4Deg)
        {
            CurrentX  = joints.X;
            CurrentY  = joints.Y;
            CurrentZ  = joints.Z;
            CurrentRZ = joints.RZ;

            m1Deg = joints.X  * MotorDegsPerMmX;
            m2Deg = joints.Y  * MotorDegsPerMmY;
            m3Deg = joints.Z  * MotorDegsPerMmZ;
            m4Deg = joints.RZ * MotorDegsPerDegRZ;
        }

        // No 3D arm model for CNC
        public Vector6? GetVisualRobotPose(Vector6 currentTcp, Vector6 toolOffset) => null;

        // Map CNC axes to the generic joint-status tuple used in GetStatus
        public (double joint1, double joint2x, double joint2z, double joint4) GetJointAngles()
            => (CurrentX, CurrentY, CurrentZ, CurrentRZ);
    }
}
