using System.Numerics;

public class CoreXYStage
{
    private readonly double _pulleyDiameter1;
    private readonly double _pulleyDiameter2;

    public double Linear1 { get; private set; }
    public double Linear2 { get; private set; }

    public CoreXYStage(double pulleyDiameter1, double pulleyDiameter2)
    {
        _pulleyDiameter1 = pulleyDiameter1;
        _pulleyDiameter2 = pulleyDiameter2;
    }

    // -----------------------------
    // Motor angle interface (pure belt space)
    // -----------------------------
    public double Motor1AngleDeg
    {
        get => LinearToDegrees(Linear1, _pulleyDiameter1);
        set => Linear1 = DegreesToLinear(value, _pulleyDiameter1);
    }

    public double Motor2AngleDeg
    {
        get => LinearToDegrees(Linear2, _pulleyDiameter2);
        set => Linear2 = DegreesToLinear(value, _pulleyDiameter2);
    }

    // -----------------------------
    // IK Path (belt space directly)
    // -----------------------------
    public (double motor1Deg, double motor2Deg) SetLinears(double linear1, double linear2)
    {
        Linear1 = linear1;
        Linear2 = linear2;

        return (
            LinearToDegrees(Linear1, _pulleyDiameter1),
            LinearToDegrees(Linear2, _pulleyDiameter2)
        );
    }

    // -----------------------------
    // CoreXY kinematics
    // -----------------------------
    public (double x, double z) Cartesian
    {
        get
        {
            double x = (Linear1 + Linear2) * 0.5;
            double z = (Linear1 - Linear2) * 0.5;
            return (x, z);
        }
        set
        {
            Linear1 = value.x + value.z;
            Linear2 = value.x - value.z;
        }
    }

    public (double motor1Deg, double motor2Deg) GetLinears()
    {
        return (
            LinearToDegrees(Linear1, _pulleyDiameter1),
            LinearToDegrees(Linear2, _pulleyDiameter2)
        );
    }

    public Matrix4x4 Transform()
    {
        var (x, z) = Cartesian;

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
