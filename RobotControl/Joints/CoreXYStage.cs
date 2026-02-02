using System;
using System.Numerics;

namespace Controller.RobotControl.Joints
{
    public class CoreXYStage
    {
        // -----------------------------
        // Pulley geometry (mm)
        // -----------------------------
        private readonly double _pulleyDiameter1;
        private readonly double _pulleyDiameter2;

        // -----------------------------
        // Internal linear belt motion (mm)
        // -----------------------------
        private double _linear1;
        private double _linear2;

        /// <summary>
        /// Fixed radial offset from J1 axis (optional, mm)
        /// </summary>
        public double BaseOffset { get; set; }

        // -----------------------------
        // Constructor
        // -----------------------------
        public CoreXYStage(double pulleyDiameter1, double pulleyDiameter2)
        {
            _pulleyDiameter1 = pulleyDiameter1;
            _pulleyDiameter2 = pulleyDiameter2;
        }

        // -----------------------------
        // Motor angle interface (degrees)
        // -----------------------------
        public double Motor1AngleDeg
        {
            get => LinearToDegrees(_linear1, _pulleyDiameter1);
            set => _linear1 = DegreesToLinear(value, _pulleyDiameter1);
        }

        public double Motor2AngleDeg
        {
            get => LinearToDegrees(_linear2, _pulleyDiameter2);
            set => _linear2 = DegreesToLinear(value, _pulleyDiameter2);
        }

        // -----------------------------
        // NEW: Set linears directly (IK path)
        // -----------------------------
        public (double motor1Deg, double motor2Deg) SetLinears(double linear1, double linear2)
        {
            _linear1 = linear1;
            _linear2 = linear2;

            double m1Deg = LinearToDegrees(_linear1, _pulleyDiameter1);
            double m2Deg = LinearToDegrees(_linear2, _pulleyDiameter2);

            return (m1Deg, m2Deg);
        }

        // -----------------------------
        // CoreXY kinematics
        // -----------------------------
        public (double x, double z) Cartesian()
        {
            double x = (_linear1 + _linear2) * 0.5 + BaseOffset;
            double z = (_linear1 - _linear2) * 0.5;
            return (x, z);
        }

        public Matrix4x4 Transform()
        {
            var (x, z) = Cartesian();

            return Matrix4x4.CreateTranslation(
                (float)x,
                0f,
                (float)z
            );
        }

        // -----------------------------
        // Helpers
        // -----------------------------
        private static double DegreesToLinear(double angleDeg, double pulleyDiameter)
        {
            double radius = pulleyDiameter * 0.5;
            double angleRad = angleDeg * Math.PI / 180.0;
            return angleRad * radius;
        }

        private static double LinearToDegrees(double linear, double pulleyDiameter)
        {
            double radius = pulleyDiameter * 0.5;
            double angleRad = linear / radius;
            return angleRad * 180.0 / Math.PI;
        }
    }
}
