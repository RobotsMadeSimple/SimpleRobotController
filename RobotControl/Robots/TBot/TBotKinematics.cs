using System;
using System.Numerics;

namespace Controller.RobotControl.Robots.TBot
{
    internal class TBotKinematics
    {
        private static readonly double pulley30tPcd = 19.099;

        // ======= Mechanical Offset =======
        // Offset from J1 column center to J4 axis (mm)
        // Expressed in flange-local coordinates
        private static readonly Vector3 J4OffsetLocal = new Vector3(
            0f,  // X offset (Radial Offset)
            48.356f,   // Y offset (Offcenter Distance)
            0f    // Z offset
        );

        // ======= Interpolated Joints =======
        public RotaryJoint InterpolatedJoint1 { get; } = new RotaryJoint(120.0 / 30.0);
        public CoreXYStage InterpolatedJoint2 { get; } = new CoreXYStage(pulley30tPcd, pulley30tPcd);
        public RotaryJoint InterpolatedJoint4 { get; } = new RotaryJoint(10.0);

        // ======= Current Joints =======
        public RotaryJoint CurrentJoint1 { get; } = new RotaryJoint(120.0 / 30.0);
        public CoreXYStage CurrentJoint2 { get; } = new CoreXYStage(pulley30tPcd, pulley30tPcd);
        public RotaryJoint CurrentJoint4 { get; } = new RotaryJoint(10.0);

        public TBotKinematics() { }

        // ============================================================
        // FORWARD KINEMATICS
        // ============================================================
        public Vector6 TcpPosition(Vector6 toolOffset)
        {
            double j1Rad = CurrentJoint1.JointAngleRad;
            double j4Rad = CurrentJoint4.JointAngleRad;

            var (radial, flangeZ) = CurrentJoint2.Cartesian;

            // --- Flange centerline position ---
            Vector3 flange = new Vector3(
                (float)(radial * Math.Cos(j1Rad)),
                (float)(radial * Math.Sin(j1Rad)),
                (float)flangeZ
            );

            // --- J4 axis world position ---
            Matrix4x4 Rj1 = Matrix4x4.CreateRotationZ((float)j1Rad);
            Vector3 j4OffsetWorld = Vector3.Transform(J4OffsetLocal, Rj1);
            Vector3 j4World = flange + j4OffsetWorld;

            // --- Tool world position ---
            // Tool rotates by (J1 + J4)
            Matrix4x4 Rtool = Matrix4x4.CreateRotationZ((float)(j1Rad + j4Rad));

            Vector3 toolWorld = Vector3.Transform(
                new Vector3((float)toolOffset.X,
                            (float)toolOffset.Y,
                            (float)toolOffset.Z),
                Rtool
            );

            Vector3 tcp = j4World + toolWorld;

            double worldRzMath =
                CurrentJoint1.JointAngleDeg +
                CurrentJoint4.JointAngleDeg;

            return new Vector6(
                tcp.X,
                -tcp.Y,
                tcp.Z,
                0,
                0,
                worldRzMath * -1
            );
        }

        // ============================================================
        // INVERSE KINEMATICS
        // ============================================================
        public static Vector6 InverseKinematics(
            Vector6 tcp,
            Vector6 toolOffset
        )
        {
            double tcpX = tcp.X;
            double tcpY = -tcp.Y;
            double tcpZ = tcp.Z;

            // Convert operator RZ -> math RZ
            double desiredWorldRzDeg = tcp.RZ * -1;
            double desiredWorldRzRad = desiredWorldRzDeg * Math.PI / 180.0;

            // --- Solve J1 ignoring J4 first (initial guess) ---
            double j1Rad = Math.Atan2(tcpY, tcpX);

            for (int i = 0; i < 8; i++)
            {
                // J4 from world orientation
                double j4Rad = desiredWorldRzRad - j1Rad;

                double c1 = Math.Cos(j1Rad);
                double s1 = Math.Sin(j1Rad);

                // Flange center
                double flangeX = tcpX;
                double flangeY = tcpY;

                // Remove tool contribution
                double toolRot = j1Rad + j4Rad;

                double toolXw =
                    toolOffset.X * Math.Cos(toolRot)
                  - toolOffset.Y * Math.Sin(toolRot);

                double toolYw =
                    toolOffset.X * Math.Sin(toolRot)
                  + toolOffset.Y * Math.Cos(toolRot);

                flangeX -= toolXw;
                flangeY -= toolYw;

                // Remove J4 axis offset
                double j4OffX =
                    J4OffsetLocal.X * c1
                  - J4OffsetLocal.Y * s1;

                double j4OffY =
                    J4OffsetLocal.X * s1
                  + J4OffsetLocal.Y * c1;

                flangeX -= j4OffX;
                flangeY -= j4OffY;

                double newJ1 = Math.Atan2(flangeY, flangeX);

                if (Math.Abs(newJ1 - j1Rad) < 1e-10)
                {
                    j1Rad = newJ1;
                    break;
                }

                j1Rad = newJ1;
            }

            double j4FinalRad = desiredWorldRzRad - j1Rad;

            // --- Compute final flange center ---
            double c = Math.Cos(j1Rad);
            double s = Math.Sin(j1Rad);

            double toolRotFinal = j1Rad + j4FinalRad;

            double toolXwFinal =
                toolOffset.X * Math.Cos(toolRotFinal)
              - toolOffset.Y * Math.Sin(toolRotFinal);

            double toolYwFinal =
                toolOffset.X * Math.Sin(toolRotFinal)
              + toolOffset.Y * Math.Cos(toolRotFinal);

            double j4OffXFinal =
                J4OffsetLocal.X * c
              - J4OffsetLocal.Y * s;

            double j4OffYFinal =
                J4OffsetLocal.X * s
              + J4OffsetLocal.Y * c;

            double flangeXFinal = tcpX - toolXwFinal - j4OffXFinal;
            double flangeYFinal = tcpY - toolYwFinal - j4OffYFinal;
            double flangeZFinal = tcpZ - toolOffset.Z - J4OffsetLocal.Z;

            double radial = Math.Sqrt(
                flangeXFinal * flangeXFinal +
                flangeYFinal * flangeYFinal
            );

            return new Vector6
            {
                X = j1Rad * 180.0 / Math.PI,
                Y = radial,
                Z = flangeZFinal,
                RX = 0,
                RY = 0,
                RZ = j4FinalRad * 180.0 / Math.PI
            };
        }

        // ============================================================
        // MOTOR TARGET UPDATE
        // ============================================================
        public void UpdateJointTargets(
            Vector6 JointTargets,
            out double m1Deg,
            out double m2Deg,
            out double m3Deg,
            out double m4Deg
        )
        {
            // J1
            InterpolatedJoint1.JointAngleDeg = JointTargets.X;
            m1Deg = InterpolatedJoint1.MotorAngleDeg;

            // CoreXY
            InterpolatedJoint2.Cartesian = (JointTargets.Y, JointTargets.Z);
            (m2Deg, m3Deg) = InterpolatedJoint2.GetLinears();

            // J4
            InterpolatedJoint4.JointAngleDeg = JointTargets.RZ;
            m4Deg = InterpolatedJoint4.MotorAngleDeg;

            // Update current states
            CurrentJoint1.MotorAngleDeg = m1Deg;
            CurrentJoint2.Motor1AngleDeg = m2Deg;
            CurrentJoint2.Motor2AngleDeg = m3Deg;
            CurrentJoint4.MotorAngleDeg = m4Deg;
        }

        public Vector6 GetVisualRobotPose(Vector6 currentTcp, Vector6 toolOffset)
        {
            double j1Deg = CurrentJoint1.JointAngleDeg;
            double j4Deg = CurrentJoint4.JointAngleDeg;

            double j1Rad = j1Deg * Math.PI / 180.0;
            double j4Rad = j4Deg * Math.PI / 180.0;

            // ----- World TCP -----
            Vector3 tcp = new Vector3(
                (float)currentTcp.X,
                (float)(-currentTcp.Y),
                (float)currentTcp.Z
            );

            // ----- Build transforms -----

            // 1) Remove tool contribution
            double toolWorldAngle = j1Rad + j4Rad;

            Vector3 toolWorld = new Vector3(
                (float)(
                    toolOffset.X * Math.Cos(toolWorldAngle)
                  - toolOffset.Y * Math.Sin(toolWorldAngle)
                ),
                (float)(
                    toolOffset.X * Math.Sin(toolWorldAngle)
                  + toolOffset.Y * Math.Cos(toolWorldAngle)
                ),
                (float)toolOffset.Z
            );

            Vector3 afterToolRemoval = tcp - toolWorld;

            // 2) Remove J4 axis offset
            Vector3 j4OffsetWorld = new Vector3(
                (float)(
                    J4OffsetLocal.X * Math.Cos(j1Rad)
                  - J4OffsetLocal.Y * Math.Sin(j1Rad)
                ),
                (float)(
                    J4OffsetLocal.X * Math.Sin(j1Rad)
                  + J4OffsetLocal.Y * Math.Cos(j1Rad)
                ),
                J4OffsetLocal.Z
            );

            Vector3 flange = afterToolRemoval - j4OffsetWorld;

            // ----- Convert to joint-friendly coordinates -----

            double radial = Math.Sqrt(
                flange.X * flange.X +
                flange.Y * flange.Y
            );

            double vertical = flange.Z;

            return new Vector6(
                j1Deg,
                radial,
                vertical,
                0,
                0,
                j4Deg
            );
        }
    }
}