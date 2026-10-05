namespace Controller.RobotControl;

/// <summary>Values of <see cref="RobotConfig.RobotType"/> that select the kinematics model.</summary>
internal static class RobotTypes
{
    /// <summary>ASTRO SCARA-style arm (the default).</summary>
    public const string Astro = "ASTRO";

    /// <summary>Four-axis CNC gantry (X, Y, Z, RZ).</summary>
    public const string Cnc4Axis = "CNC4Axis";
}
