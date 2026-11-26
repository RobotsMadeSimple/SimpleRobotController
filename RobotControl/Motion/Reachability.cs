using Controller.RobotControl.Robots;

namespace Controller.RobotControl;

/// <summary>The answer to "can the robot get there?": the joint solution, and why not when it cannot.</summary>
/// <param name="Ok">True when inverse kinematics solves and every joint is inside its soft limits.</param>
/// <param name="Joints">The joint solution (X = J1, Y = J2, Z = J3, RZ = J4); set even when a limit fails.</param>
/// <param name="Reason">"" when ok, else "outside reach" or e.g. "J2 limit (512.4 > 500)".</param>
/// <param name="Joint">The joint (0..3) out of range, or -1.</param>
internal readonly record struct ReachResult(bool Ok, Vector6 Joints, string Reason, int Joint = -1);

/// <summary>
/// Pre-move reach checks (SPEL+ <c>TargetOK</c>, RAPID <c>CalcJointT</c>): runs the robot's real
/// inverse kinematics and the same joint windows the motion thread enforces
/// (<see cref="JointLimiter"/>), without moving anything. Pure — the caller passes the
/// kinematics, the target in world coordinates (local frame already applied) and the tool.
/// </summary>
internal static class Reachability
{
    /// <summary>How far forward kinematics of the IK solution may land from the target and still
    /// count as a solution (mm). The kinematics run in single precision, so this is well above
    /// rounding yet far below anything a part would notice.</summary>
    public const double SolveTolerance = 0.05;

    /// <summary>Reach check of a world-frame TCP target with <paramref name="tool"/>.</summary>
    /// <param name="limits">The enabled joint windows, or null when joint limits are off.</param>
    public static ReachResult Check(IRobotKinematics kin, Vector6 worldTarget, Vector6 tool,
                                    (double lo, double hi)[]? limits, IReadOnlyList<string> jointNames)
    {
        var joints = kin.InverseKinematics(worldTarget, tool);
        if (!Finite(joints))
            return new ReachResult(false, joints, "outside reach");

        // IK always returns *something*; a target it cannot really solve (inside the J4 offset
        // circle, say) shows up as forward kinematics landing somewhere else.
        var back = kin.ForwardFromJoints(joints, tool);
        double dx = back.X - worldTarget.X, dy = back.Y - worldTarget.Y, dz = back.Z - worldTarget.Z;
        if (!(Math.Sqrt(dx * dx + dy * dy + dz * dz) <= SolveTolerance))
            return new ReachResult(false, joints, "outside reach");

        return CheckJoints(joints, limits, jointNames);
    }

    /// <summary>Limit check of a joint target (MoveJoints, <c>poseOf</c>-style inputs).</summary>
    public static ReachResult CheckJoints(Vector6 joints, (double lo, double hi)[]? limits, IReadOnlyList<string> jointNames)
    {
        if (!Finite(joints))
            return new ReachResult(false, joints, "joint values are not numbers");
        if (limits == null) return new ReachResult(true, joints, "");

        var r = JointLimiter.Check(joints, limits);
        if (!r.Violated) return new ReachResult(true, joints, "");

        int j = r.Joint;
        double v = j switch { 0 => joints.X, 1 => joints.Y, 2 => joints.Z, _ => joints.RZ };
        double lo = Math.Min(limits[j].lo, limits[j].hi), hi = Math.Max(limits[j].lo, limits[j].hi);
        string name = j < jointNames.Count ? jointNames[j] : $"joint {j + 1}";
        string reason = r.Direction > 0
            ? $"{name} limit ({Fmt(v)} > {Fmt(hi)})"
            : $"{name} limit ({Fmt(v)} < {Fmt(lo)})";
        return new ReachResult(false, joints, reason, j);
    }

    private static string Fmt(double v) => v.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture);

    private static bool Finite(Vector6 v) =>
        double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z) && double.IsFinite(v.RZ);

    /// <summary>Joint vector ↔ the joints record a program stores (an Objects-list element with
    /// fields j1…j4): X = j1, Y = j2, Z = j3, RZ = j4.</summary>
    public static ObjectRecord ToRecord(Vector6 joints) => new()
    {
        ["j1"] = joints.X, ["j2"] = joints.Y, ["j3"] = joints.Z, ["j4"] = joints.RZ,
    };

    /// <summary>The joint vector of a joints record; a missing field reads as <paramref name="fallback"/>'s value (or 0).</summary>
    public static Vector6 FromRecord(IReadOnlyDictionary<string, double> rec, Vector6? fallback = null)
    {
        double Get(string k, double d) => rec.TryGetValue(k, out var v) ? v : d;
        var f = fallback ?? Vector6.Zero;
        return new Vector6(Get("j1", f.X), Get("j2", f.Y), Get("j3", f.Z), 0, 0, Get("j4", f.RZ));
    }

    /// <summary>The field names of a joints record, in joint order.</summary>
    public static readonly IReadOnlyList<string> JointFields = ["j1", "j2", "j3", "j4"];

    /// <summary>True when the record has at least one of the joint fields — i.e. it is a joints value.</summary>
    public static bool IsJointsRecord(IReadOnlyDictionary<string, double> rec) =>
        JointFields.Any(rec.ContainsKey);
}
