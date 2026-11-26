using Controller.RobotControl;
using Controller.RobotControl.Robots.ASTRO;
using Controller.RobotControl.Robots.CNC4Axis;

namespace RobotControl.Tests;

/// <summary>
/// The pure reach check (real ASTRO IK + FK + <see cref="JointLimiter.Check"/>): reachable and
/// unreachable targets, the limit edges, the tool offset, and the IK ↔ FK round trip.
/// </summary>
public class ReachabilityTests
{
    private static readonly string[] Names = ["J1", "J2", "J3", "J4"];
    private static readonly ASTROKinematics Astro = new();

    private static (double, double)[] Limits(double j2Max = 450) =>
        [(-170, 170), (60, j2Max), (0, 400), (-360, 360)];

    /// <summary>A pose the arm really reaches: FK of J1 30°, radial 300, Z 200, J4 10°.</summary>
    private static Vector6 Reachable(Vector6? tool = null) =>
        Astro.ForwardFromJoints(new Vector6(30, 300, 200, 0, 0, 10), tool ?? Vector6.Zero);

    [Fact]
    public void A_pose_from_forward_kinematics_is_reachable_and_solves_back_to_its_joints()
    {
        var r = Reachability.Check(Astro, Reachable(), Vector6.Zero, Limits(), Names);
        Assert.True(r.Ok, r.Reason);
        Assert.Equal(30, r.Joints.X, 3);
        Assert.Equal(300, r.Joints.Y, 3);
        Assert.Equal(200, r.Joints.Z, 3);
        Assert.Equal(10, r.Joints.RZ, 3);
    }

    [Fact]
    public void A_point_inside_the_J4_offset_circle_is_outside_reach()
    {
        // The J4 axis sits 48.4 mm off the flange centreline, so with no tool the TCP can never
        // come closer than that to the base axis — IK has no solution there.
        var r = Reachability.Check(Astro, new Vector6(5, 5, 150), Vector6.Zero, null, Names);
        Assert.False(r.Ok);
        Assert.Equal("outside reach", r.Reason);
    }

    [Fact]
    public void A_joint_past_its_limit_is_unreachable_and_named()
    {
        var r = Reachability.Check(Astro, Reachable(), Vector6.Zero, Limits(j2Max: 250), Names);
        Assert.False(r.Ok);
        Assert.Equal(1, r.Joint);
        Assert.StartsWith("J2 limit (300", r.Reason);
        Assert.EndsWith("> 250)", r.Reason);
    }

    [Fact]
    public void The_limit_edge_counts_as_inside()
    {
        var target = Reachable();
        double radial = Reachability.Check(Astro, target, Vector6.Zero, null, Names).Joints.Y;
        Assert.True(Reachability.Check(Astro, target, Vector6.Zero, Limits(j2Max: radial + 1e-6), Names).Ok);
        Assert.True(Reachability.Check(Astro, target, Vector6.Zero, Limits(j2Max: Math.Round(radial, 3) + 0.001), Names).Ok);
        Assert.False(Reachability.Check(Astro, target, Vector6.Zero, Limits(j2Max: radial - 0.01), Names).Ok);
    }

    [Fact]
    public void Limits_off_checks_only_that_ik_solves()
    {
        Assert.True(Reachability.Check(Astro, Reachable(), Vector6.Zero, null, Names).Ok);
    }

    [Fact]
    public void A_tool_offset_changes_the_answer()
    {
        // The same TCP point with a 120 mm tool (along the tool X) needs a different radial reach.
        var target = Reachable();
        var tool   = new Vector6(120, 0, 0);
        var bare   = Reachability.Check(Astro, target, Vector6.Zero, null, Names);
        var tooled = Reachability.Check(Astro, target, tool, null, Names);
        Assert.True(bare.Ok && tooled.Ok);
        Assert.NotEqual(bare.Joints.Y, tooled.Joints.Y, 1);

        // A limit window that admits the bare solution rejects the tooled one (or vice versa).
        double lo = Math.Min(bare.Joints.Y, tooled.Joints.Y), hi = Math.Max(bare.Joints.Y, tooled.Joints.Y);
        (double, double)[] window = [(-170, 170), (lo - 1, (lo + hi) / 2), (0, 400), (-360, 360)];
        Assert.NotEqual(Reachability.Check(Astro, target, Vector6.Zero, window, Names).Ok,
                        Reachability.Check(Astro, target, tool, window, Names).Ok);
    }

    [Fact]
    public void Ik_and_fk_round_trip_with_a_tool()
    {
        var tool = new Vector6(40, -15, -60, 0, 0, 0);
        var target = Reachable(tool);
        var r = Reachability.Check(Astro, target, tool, null, Names);
        Assert.True(r.Ok, r.Reason);
        var back = Astro.ForwardFromJoints(r.Joints, tool);
        Assert.Equal(target.X, back.X, 3);
        Assert.Equal(target.Y, back.Y, 3);
        Assert.Equal(target.Z, back.Z, 3);
        Assert.Equal(target.RZ, back.RZ, 3);
    }

    [Fact]
    public void ForwardFromJoints_matches_the_stateful_forward_kinematics()
    {
        var k = new ASTROKinematics();
        var joints = new Vector6(-42, 333, 120, 0, 0, 75);
        k.UpdateMotorTargets(joints, out _, out _, out _, out _);
        var tool = new Vector6(25, 10, -30);
        var a = k.ForwardKinematics(tool);
        var b = k.ForwardFromJoints(joints, tool);
        Assert.Equal(a.X, b.X, 3); Assert.Equal(a.Y, b.Y, 3); Assert.Equal(a.Z, b.Z, 3); Assert.Equal(a.RZ, b.RZ, 3);
    }

    [Fact]
    public void Cnc_is_identity_so_only_the_limits_decide()
    {
        var cnc = new CNC4AxisKinematics();
        var r = Reachability.Check(cnc, new Vector6(10, 20, -5, 0, 0, 90), Vector6.Zero, null, ["X", "Y", "Z", "RZ"]);
        Assert.True(r.Ok);
        Assert.Equal(10, r.Joints.X); Assert.Equal(-5, r.Joints.Z); Assert.Equal(90, r.Joints.RZ);
        (double, double)[] lim = [(0, 300), (0, 300), (0, 100), (-360, 360)];
        var below = Reachability.Check(cnc, new Vector6(10, 20, -5), Vector6.Zero, lim, ["X", "Y", "Z", "RZ"]);
        Assert.False(below.Ok);
        Assert.Equal("Z limit (-5 < 0)", below.Reason);
    }

    [Fact]
    public void JointLimiter_Check_reports_direction_and_ignores_unset_bounds()
    {
        (double, double)[] lim = [(double.NegativeInfinity, double.PositiveInfinity), (0, 100), (0, 100), (-10, 10)];
        Assert.False(JointLimiter.Check(new Vector6(999, 50, 50, 0, 0, 0), lim).Violated);
        var hi = JointLimiter.Check(new Vector6(0, 100.01, 50, 0, 0, 0), lim);
        Assert.True(hi.Violated); Assert.Equal(1, hi.Joint); Assert.Equal(1, hi.Direction);
        var lo = JointLimiter.Check(new Vector6(0, 50, 50, 0, 0, -10.5), lim);
        Assert.True(lo.Violated); Assert.Equal(3, lo.Joint); Assert.Equal(-1, lo.Direction);
        Assert.False(JointLimiter.Check(new Vector6(0, 100, 0, 0, 0, 10), lim).Violated);
    }
}
