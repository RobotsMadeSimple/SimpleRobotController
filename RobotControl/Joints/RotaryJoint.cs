using System.Numerics;

public class RotaryJoint
{
    private double _jointAngleRad;
    private readonly double _gearRatio; // motor_angle / joint_angle

    public RotaryJoint(double gearRatio = 1.0)
    {
        _gearRatio = gearRatio;
    }

    // -----------------------------
    // Joint-space API (degrees)
    // -----------------------------
    public double JointAngleDeg
    {
        get => _jointAngleRad * 180.0 / Math.PI;
        set => _jointAngleRad = value * Math.PI / 180.0;
    }

    public double JointAngleRad => _jointAngleRad;

    // -----------------------------
    // Motor-space API (degrees)
    // -----------------------------
    public double MotorAngleDeg
    {
        get => JointAngleDeg * _gearRatio;
        set => JointAngleDeg = value / _gearRatio;
    }

    // -----------------------------
    // FK transform (always joint space)
    // -----------------------------
    public Matrix4x4 Transform()
    {
        float c = (float)Math.Cos(_jointAngleRad);
        float s = (float)Math.Sin(_jointAngleRad);

        return new Matrix4x4(
            c, -s, 0, 0,
            s, c, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        );
    }
}
