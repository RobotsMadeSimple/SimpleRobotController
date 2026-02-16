using System;
using System.Collections.Generic;
using System.Text;

namespace Controller.RobotControl.Robots.TBot
{
    internal class TBotKinematics
    {
        private static readonly double pulley30tPcd = 19.099;
        public RotaryJoint InterpolatedJoint1 { get; } = new RotaryJoint(120 / 30);
        public CoreXYStage InterpolatedJoint2 { get; } = new CoreXYStage(pulley30tPcd, pulley30tPcd);

        public RotaryJoint CurrentJoint1 { get; } = new RotaryJoint(120 / 30);
        public CoreXYStage CurrentJoint2 { get; } = new CoreXYStage(pulley30tPcd, pulley30tPcd);

        public TBotKinematics() { }

        public static Vector6 InverseKinematics(
            Vector6 tcp,
            Vector6 toolOffset
        )
        {
            // TCP in world (mm)
            double tcpX = tcp.X;
            double tcpY = tcp.Y;
            double tcpZ = tcp.Z;

            // Tool offset in flange-local (mm)
            double tX = toolOffset.X;
            double tY = toolOffset.Y;
            double tZ = toolOffset.Z;

            // -----------------------------
            // 1) Solve J1 rotation (tooling-aware)
            //    Initial guess: angle to TCP (good start)
            // -----------------------------
            double j1Rad = Math.Atan2(tcpY, tcpX);

            // Iterate to account for lateral tool offsets
            // 3-5 iterations is plenty; converges fast for realistic tool lengths.
            for (int i = 0; i < 6; i++)
            {
                double cos = Math.Cos(j1Rad);
                double sin = Math.Sin(j1Rad);

                // Rotate tool offset into world XY
                double toolXw = tX * cos - tY * sin;
                double toolYw = tX * sin + tY * cos;

                // Flange position in XY
                double flangeX = tcpX - toolXw;
                double flangeY = tcpY - toolYw;

                double newJ1Rad = Math.Atan2(flangeY, flangeX);

                // Early exit if converged
                if (Math.Abs(newJ1Rad - j1Rad) < 1e-12)
                {
                    j1Rad = newJ1Rad;
                    break;
                }

                j1Rad = newJ1Rad;
            }

            double j1Deg = j1Rad * 180.0 / Math.PI;

            // -----------------------------
            // 2) Compute flange position (final, using converged j1)
            // -----------------------------
            double c = Math.Cos(j1Rad);
            double s = Math.Sin(j1Rad);

            double toolXwFinal = tX * c - tY * s;
            double toolYwFinal = tX * s + tY * c;
            double toolZwFinal = tZ;

            double flangeXFinal = tcpX - toolXwFinal;
            double flangeYFinal = tcpY - toolYwFinal;
            double flangeZFinal = tcpZ - toolZwFinal;

            // -----------------------------
            // 3) Solve radial distance (flange)
            // -----------------------------
            double radial = Math.Sqrt(flangeXFinal * flangeXFinal + flangeYFinal * flangeYFinal);

            // -----------------------------
            // 4) Return joint-space (radial + vertical)
            // -----------------------------
            return new Vector6
            {
                X = j1Deg,          // J1 rotation (deg)
                Y = radial,         // CoreXY radial component (mm)
                Z = flangeZFinal,   // CoreXY vertical component (mm)
                RX = 0,
                RY = 0,
                RZ = 0
            };
        }
        public Vector6 TcpPosition(Vector6 CurrentTool)
        {
            double j1Rad = CurrentJoint1.JointAngleRad;

            // radial (mm) and flangeZ (mm)
            var (radial, flangeZ) = CurrentJoint2.Cartesian;

            double flangeX = radial * Math.Cos(j1Rad);
            double flangeY = radial * Math.Sin(j1Rad);

            double toolXw = CurrentTool.X * Math.Cos(j1Rad) - CurrentTool.Y * Math.Sin(j1Rad);
            double toolYw = CurrentTool.X * Math.Sin(j1Rad) + CurrentTool.Y * Math.Cos(j1Rad);

            return new Vector6(
                flangeX + toolXw,
                flangeY + toolYw,
                flangeZ + CurrentTool.Z
            );
        }

        public Vector6 GetVisualRobotPose(Vector6 CurrentPosition, Vector6 CurrentTool)
        {
            // J1 from actual joint state (recommended)
            double j1Deg = CurrentJoint1.JointAngleDeg;
            float j1Rad = (float)(j1Deg * Math.PI / 180.0);

            // TCP position (world, mm)
            var tcp = new System.Numerics.Vector3(
                (float)CurrentPosition.X,
                (float)CurrentPosition.Y,
                (float)CurrentPosition.Z
            );

            // Tool offset in flange-local (mm)
            var toolLocal = new System.Numerics.Vector3(
                (float)CurrentTool.X,
                (float)CurrentTool.Y,
                (float)CurrentTool.Z
            );

            // Rotate tool offset into world (about Z)
            var Rz = System.Numerics.Matrix4x4.CreateRotationZ(j1Rad);
            var toolWorld = System.Numerics.Vector3.TransformNormal(toolLocal, Rz);

            // Flange position = TCP - tool(world)
            var flange = tcp - toolWorld;

            // Joint-friendly values
            double radial = Math.Sqrt(flange.X * flange.X + flange.Y * flange.Y);
            double vertical = flange.Z;

            return new(j1Deg, radial, vertical);
        }

        public void UpdateJointTargets(Vector6 JointTargets, out double m1Deg, out double m2Deg, out double m3Deg)
        {
            // Joint Targets are Joint Angles and Belt Length 1 and Belt Length 2
            InterpolatedJoint1.JointAngleDeg = JointTargets.X;
            m1Deg = InterpolatedJoint1.MotorAngleDeg;
            InterpolatedJoint2.Cartesian = (JointTargets.Y, JointTargets.Z);
            (m2Deg, m3Deg) = InterpolatedJoint2.GetLinears();

            // Update the current Joint poses with the new target motor angles
            CurrentJoint1.MotorAngleDeg = m1Deg;
            CurrentJoint2.Motor1AngleDeg = m2Deg;
            CurrentJoint2.Motor2AngleDeg = m3Deg;
        }
    }
}
