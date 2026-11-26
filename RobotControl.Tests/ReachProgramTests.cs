using Controller.RobotControl;
using Controller.RobotControl.Robots.ASTRO;

namespace RobotControl.Tests;

/// <summary>
/// reachable() / jointsOf() / poseOf() and the outOfReach failure, run through the real program
/// executor on the real ASTRO kinematics with joint limits on: conditions, Set Variable, a saved
/// point, a local frame, the active and a named tool, grid cells, Try / catch, moves that refuse an
/// unreachable target, Move to Joints, Save Position → joints and a List step taking a whole value.
/// </summary>
[Collection("RobotController")]
public sealed class ReachProgramTests : IDisposable
{
    private readonly StepsRig _rig = new(identityKinematics: false);
    public void Dispose() => _rig.Dispose();

    private static readonly ASTROKinematics Astro = new();

    /// <summary>FK of a joint vector — poses the arm really reaches (J1 °, radial, Z, J4 °).</summary>
    private static Vector6Val Fk(double j1, double radial, double z, double j4)
    {
        var p = Astro.ForwardFromJoints(new Vector6(j1, radial, z, 0, 0, j4), Vector6.Zero);
        return new Vector6Val { X = p.X, Y = p.Y, Z = p.Z, RX = p.RX, RY = p.RY, RZ = p.RZ };
    }

    private static readonly Vector6Val Near = Fk(30, 300, 200, 10);   // inside every window
    private static readonly Vector6Val Far  = Fk(30, 520, 200, 10);   // radial 520 > J2 max 450

    private void Limits(double j2Min = 60, double j2Max = 450) => _rig.Robot.SetConfig(new RobotConfig
    {
        JointLimitsEnabled = true,
        Joint1Min = -170, Joint1Max = 170,
        Joint2Min = j2Min, Joint2Max = j2Max,
        Joint3Min = 0,    Joint3Max = 400,
        Joint4Min = -360, Joint4Max = 360,
    });

    private static ProgramStep If(string expr, params ProgramStep[] then) => StepsRig.Step(StepType.IfCondition, s =>
    {
        s.Condition = new ConditionGroup { Items = [new ConditionItem { Id = "c", Expr = expr }] };
        s.IfSteps   = then.ToList();
    });

    private static ProgramStep Try(ProgramStep[] body, params ProgramStep[] catchSteps) => StepsRig.Step(StepType.Try, s =>
    {
        s.TrySteps = body.ToList();
        s.CatchSteps = catchSteps.ToList();
        s.ErrorVariableName = "msg";
        s.ErrorCodeVariableName = "code";
    });

    private static ProgramVariable[] Vars(params ProgramVariable[] extra) =>
    [
        StepsRig.PointList("pts", Near, Far),
        StepsRig.Num("ok"), StepsRig.Num("bad"), StepsRig.Num("v"), StepsRig.Num("caught"),
        StepsRig.Str("msg"), StepsRig.Str("code"),
        .. extra,
    ];

    // ── reachable() ───────────────────────────────────────────────────────

    [Fact]
    public void Reachable_in_conditions_and_set_variable()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(),
            If("reachable($pts[0])", StepsRig.Set("ok", "1")),
            If("not reachable($pts[1])", StepsRig.Set("bad", "1")),
            StepsRig.Set("v", "reachable($pts[1]) + 10 * reachable($pts[0])")));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$ok"));
        Assert.Equal(1, _rig.Live("$bad"));
        Assert.Equal(10, _rig.Live("$v"));
    }

    [Fact]
    public void Reachable_turns_false_exactly_past_the_limit_edge()
    {
        // The radial joint for Near is 300: a window ending at 300 admits it, one ending at 299.99 does not.
        Limits(j2Max: 300.001);
        _rig.Run(StepsRig.Prog(Vars(), StepsRig.Set("ok", "reachable($pts[0])")));
        Assert.Equal(1, _rig.Live("$ok"));

        Limits(j2Max: 299.99);
        _rig.Run(StepsRig.Prog(Vars(), StepsRig.Set("ok", "reachable($pts[0])")));
        Assert.Equal(0, _rig.Live("$ok"));
    }

    [Fact]
    public void A_saved_point_and_a_local_frame_resolve_like_a_move_target()
    {
        Limits();
        _rig.Points.SavePoint("Pick", new Vector6(Near.X, Near.Y, Near.Z, 0, 0, Near.RZ));
        _rig.Locals.SaveLocal("Shift", new Vector6(250, 0, 0));
        _rig.Run(StepsRig.Prog(Vars(),
            StepsRig.Set("ok",  "reachable(\"Pick\")"),
            StepsRig.Set("bad", "reachable(\"Pick\", \"\", \"Shift\")")));     // +250 mm in X: past J2's 450
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$ok"));
        Assert.Equal(0, _rig.Live("$bad"));

        // The program's active local applies when none is named (SetLocal), just as for a move.
        _rig.Run(StepsRig.Prog(Vars(),
            StepsRig.Step(StepType.SetLocal, s => s.LocalName = "Shift"),
            StepsRig.Set("ok", "reachable(\"Pick\")"),
            StepsRig.Set("v",  "reachable(\"Pick\", \"\", \"none\")")));
        Assert.Equal(0, _rig.Live("$ok"));
        Assert.Equal(1, _rig.Live("$v"));
    }

    [Fact]
    public void The_active_tool_and_a_named_tool_change_the_answer()
    {
        Limits(j2Min: 250);     // a long outward tool pulls the flange in, below 250 mm
        var saved = _rig.Robot.CurrentTool;
        string toolName = "ReachTestTool-" + Guid.NewGuid().ToString("N")[..8];
        _rig.Robot.toolRepo.SaveTool(toolName, new Vector6(200, 0, 0));
        try
        {
            _rig.Run(StepsRig.Prog(Vars(),
                StepsRig.Set("ok",  "reachable($pts[0])"),
                StepsRig.Set("bad", $"reachable($pts[0], \"{toolName}\")")));
            Assert.Equal(1, _rig.Live("$ok"));
            Assert.Equal(0, _rig.Live("$bad"));

            _rig.Robot.CurrentTool = new Vector6(200, 0, 0);     // as SetTool leaves it
            _rig.Run(StepsRig.Prog(Vars(),
                StepsRig.Set("ok",  "reachable($pts[0])"),
                StepsRig.Set("v",   "reachable($pts[0], \"none\")")));
            Assert.Equal(0, _rig.Live("$ok"));
            Assert.Equal(1, _rig.Live("$v"));
        }
        finally
        {
            _rig.Robot.CurrentTool = saved;
            _rig.Robot.toolRepo.DeleteTool(toolName);
        }
    }

    [Fact]
    public void Grid_and_stack_cells_are_checked_like_moves_to_them()
    {
        Limits();
        _rig.Points.SavePoint("TrayBase", new Vector6(Near.X, Near.Y, Near.Z, 0, 0, Near.RZ));
        _rig.Grids.Upsert(new Grid { Id = "g-tray", Name = "Tray", BasePointName = "TrayBase", ColOffsetX = 100, RowOffsetY = 30, ColCount = 6 });
        _rig.Stacks.Upsert(new RobotStack { Id = "s-pal", Name = "Pallet", BasePointName = "TrayBase", OffsetZ = 50 });
        _rig.Run(StepsRig.Prog(Vars(),
            StepsRig.Set("ok",  "reachable(grid(\"Tray\", 1, 0)) + reachable(grid(\"Tray\", 0))"),
            StepsRig.Set("bad", "reachable(grid(\"Tray\", 0, 5))"),          // +500 mm in X
            StepsRig.Set("v",   "reachable(stack(\"Pallet\", 1)) + 10 * reachable(stack(\"Pallet\", 9))")));   // Z 250 ok, 650 > 400
        _rig.AssertComplete();
        Assert.Equal(2, _rig.Live("$ok"));
        Assert.Equal(0, _rig.Live("$bad"));
        Assert.Equal(1, _rig.Live("$v"));
    }

    // ── jointsOf() / poseOf() ─────────────────────────────────────────────

    [Fact]
    public void JointsOf_and_poseOf_round_trip()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(StepsRig.Num("j1"), StepsRig.Num("j2"), StepsRig.Num("j4"),
                                     StepsRig.Num("x"), StepsRig.Num("y"), StepsRig.Num("rz")),
            StepsRig.Set("j1", "jointsOf($pts[0]).j1"),
            StepsRig.Set("j2", "jointsOf($pts[0])[1]"),
            StepsRig.Set("j4", "jointsOf($pts[0]).j4"),
            StepsRig.Set("x",  "poseOf(jointsOf($pts[0])).x"),
            StepsRig.Set("y",  "poseOf(jointsOf($pts[0])).y"),
            StepsRig.Set("rz", "poseOf(jointsOf($pts[0])).rz")));
        _rig.AssertComplete();
        Assert.Equal(30,  _rig.Live("$j1"), 3);
        Assert.Equal(300, _rig.Live("$j2"), 3);
        Assert.Equal(10,  _rig.Live("$j4"), 3);
        Assert.Equal(Near.X,  _rig.Live("$x"), 3);
        Assert.Equal(Near.Y,  _rig.Live("$y"), 3);
        Assert.Equal(Near.RZ, _rig.Live("$rz"), 3);
    }

    [Fact]
    public void PoseOf_a_joints_list_element_in_a_local_frame()
    {
        Limits();
        _rig.Locals.SaveLocal("Up", new Vector6(0, 0, 50));
        _rig.Run(StepsRig.Prog(Vars(StepsRig.Records("home", StepsRig.Rec(("j1", 30), ("j2", 300), ("j3", 200), ("j4", 10)))),
            StepsRig.Set("v",  "poseOf($home[0]).z"),
            StepsRig.Set("ok", "poseOf($home[0], \"\", \"Up\").z")));
        _rig.AssertComplete();
        Assert.Equal(200, _rig.Live("$v"), 3);
        Assert.Equal(150, _rig.Live("$ok"), 3);       // world Z 200 is local Z 150 in a frame 50 up
    }

    [Fact]
    public void JointsOf_an_unreachable_point_fails_with_a_catchable_outOfReach()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(),
            Try([StepsRig.Set("v", "jointsOf($pts[1]).j1")], StepsRig.Set("caught", "1")),
            StepsRig.Set("ok", "$code.contains(\"outOfReach\") + $msg.contains(\"J2 limit\")")));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$caught"));
        Assert.Equal(2, _rig.Live("$ok"));
        Assert.Equal(0, _rig.Live("$v"));
    }

    [Fact]
    public void Uncaught_outOfReach_ends_the_program_with_the_reason()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(), StepsRig.Set("v", "jointsOf($pts[1]).j2")));
        _rig.AssertError("out of reach — J2 limit");
    }

    [Fact]
    public void A_bare_jointsOf_in_a_number_field_says_how_to_read_it()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(), StepsRig.Set("v", "jointsOf($pts[0])")));
        _rig.AssertError(".j1");
    }

    // ── Moves fail the same way ───────────────────────────────────────────

    [Fact]
    public void A_move_to_an_unreachable_target_fails_before_moving_and_try_catches_it()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(),
            Try([StepsRig.Step(StepType.MoveJ, s => s.PointNameExpr = "$pts[1]")], StepsRig.Set("caught", "1")),
            StepsRig.Set("ok", "$code.contains(\"outOfReach\")"),
            StepsRig.Step(StepType.MoveL, s => { s.PointNameExpr = "$pts[0]"; s.MotionProfile = "scurve"; })));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$caught"));
        Assert.Equal(1, _rig.Live("$ok"));
        var moves = _rig.Dispatched.Where(c => c.CommandType is "MoveJ" or "MoveL").ToList();
        Assert.Single(moves);                                   // only the reachable MoveL went out
        Assert.Equal("MoveL", moves[0].CommandType);
        Assert.Equal("scurve", moves[0].Profile);
    }

    [Fact]
    public void A_jump_checks_its_lift_and_traverse_points_too()
    {
        Limits();
        // The target itself is fine, but the lift height (Z 450) is past J3's 400.
        _rig.Run(StepsRig.Prog(Vars(),
            Try([StepsRig.Step(StepType.JumpJ, s => { s.PointNameExpr = "$pts[0]"; s.JumpZ = 450; })], StepsRig.Set("caught", "1"))));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$caught"));
        Assert.DoesNotContain(_rig.Dispatched, c => c.CommandType is "MoveL" or "MoveJ");
    }

    // ── Move to Joints ────────────────────────────────────────────────────

    [Fact]
    public void MoveJoints_takes_a_joints_value_with_overrides_and_dispatches_a_joint_move()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(StepsRig.Records("home", StepsRig.Rec(("j1", 10), ("j2", 200), ("j3", 100), ("j4", 0)))),
            StepsRig.Step(StepType.MoveJoints, s =>
            {
                s.JointsExpr = "$home[0]";
                s.Expressions = new() { ["j4"] = "40 + 5" };
                s.Speed = 50;
                s.MotionProfile = "scurve";
            })));
        _rig.AssertComplete();
        var cmd = Assert.Single(_rig.Dispatched, c => c.CommandType == "MoveJoints");
        Assert.Equal(10,  cmd.X);
        Assert.Equal(200, cmd.Y);
        Assert.Equal(100, cmd.Z);
        Assert.Equal(45,  cmd.RZ);
        Assert.Equal(50,  cmd.Speed);
        Assert.Equal("scurve", cmd.Profile);
        Assert.True(cmd.ApplySpeedOverride);
    }

    [Fact]
    public void MoveJoints_to_jointsOf_a_point()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(), StepsRig.Step(StepType.MoveJoints, s => s.JointsExpr = "jointsOf($pts[0])")));
        _rig.AssertComplete();
        var cmd = Assert.Single(_rig.Dispatched, c => c.CommandType == "MoveJoints");
        Assert.Equal(30, cmd.X!.Value, 3);
        Assert.Equal(300, cmd.Y!.Value, 3);
    }

    [Fact]
    public void MoveJoints_out_of_range_fails_outOfReach_and_does_not_move()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(),
            Try([StepsRig.Step(StepType.MoveJoints, s => { s.J1 = 0; s.J2 = 500; s.J3 = 100; s.J4 = 0; })], StepsRig.Set("caught", "1")),
            StepsRig.Set("ok", "$msg.contains(\"J2 limit (500 > 450)\")")));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$caught"));
        Assert.Equal(1, _rig.Live("$ok"));
        Assert.DoesNotContain(_rig.Dispatched, c => c.CommandType == "MoveJoints");
    }

    [Fact]
    public void MoveJoints_rejects_a_points_element()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(), StepsRig.Step(StepType.MoveJoints, s => s.JointsExpr = "$pts[0]")));
        _rig.AssertError("jointsOf");
    }

    // ── Capturing joints ──────────────────────────────────────────────────

    [Fact]
    public void SavePosition_joints_stores_the_live_joints_in_an_objects_list()
    {
        _rig.Robot.SetJointPosition(1, 222);     // radial joint now reads 222
        _rig.Robot.SetJointPosition(3, -15);
        _rig.Run(StepsRig.Prog([StepsRig.Records("saved")],
            StepsRig.Step(StepType.SavePosition, s => { s.VariableName = "saved"; s.SavePositionSource = "joints"; s.SavePositionMode = "append"; })));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Count("saved"));
        Assert.Equal(222, _rig.Live("$saved[0].j2"), 3);
        Assert.Equal(-15, _rig.Live("$saved[0].j4"), 3);
    }

    [Fact]
    public void A_list_step_appends_a_whole_jointsOf_value()
    {
        Limits();
        _rig.Run(StepsRig.Prog(Vars(StepsRig.Records("js")),
            StepsRig.Step(StepType.ListOp, s => { s.VariableName = "js"; s.ListAction = "append"; s.ListValueExpr = "jointsOf($pts[0])"; }),
            StepsRig.Step(StepType.ListOp, s => { s.VariableName = "pts"; s.ListAction = "append"; s.ListValueExpr = "poseOf($js[0])"; })));
        _rig.AssertComplete();
        Assert.Equal(300, _rig.Live("$js[0].j2"), 3);
        Assert.Equal(3, _rig.Count("pts"));
        Assert.Equal(Near.X, _rig.Live("$pts[2].x"), 3);
    }
}

/// <summary>CNC4Axis has identity kinematics: Move to Joints moves the axes directly.</summary>
[Collection("RobotController")]
public sealed class MoveJointsCncTests : IDisposable
{
    private readonly StepsRig _rig = new();   // identity (CNC4Axis) kinematics
    public void Dispose() => _rig.Dispose();

    [Fact]
    public void Joints_are_the_axes_and_limits_apply()
    {
        _rig.Robot.SetConfig(new RobotConfig { RobotType = RobotTypes.Cnc4Axis, JointLimitsEnabled = true, Joint3Min = -50, Joint3Max = 0 });
        _rig.Run(StepsRig.Prog([StepsRig.Num("caught"), StepsRig.Str("msg"), StepsRig.Str("code"), StepsRig.Num("x")],
            StepsRig.Step(StepType.MoveJoints, s => { s.J1 = 120; s.J2 = 80; s.J3 = -10; s.J4 = 90; }),
            StepsRig.Set("x", "jointsOf(\"none-such\").j1 * 0"),   // a missing point is pointNotFound — caught below
            StepsRig.Step(StepType.Wait, s => s.WaitMs = 0)));
        _rig.AssertError("no saved point named 'none-such'");
        var cmd = Assert.Single(_rig.Dispatched, c => c.CommandType == "MoveJoints");
        Assert.Equal(120, cmd.X); Assert.Equal(80, cmd.Y); Assert.Equal(-10, cmd.Z); Assert.Equal(90, cmd.RZ);

        _rig.Dispatched.Clear();
        _rig.Run(StepsRig.Prog([StepsRig.Num("caught"), StepsRig.Str("msg"), StepsRig.Str("code"), StepsRig.Num("x")],
            StepsRig.Step(StepType.Try, s =>
            {
                s.TrySteps = [StepsRig.Step(StepType.MoveJoints, m => m.J3 = 5)];
                s.CatchSteps = [StepsRig.Set("caught", "1")];
                s.ErrorVariableName = "msg";
            }),
            StepsRig.Set("x", "$msg.contains(\"Z limit (5 > 0)\")")));
        _rig.AssertComplete();
        Assert.Equal(1, _rig.Live("$caught"));
        Assert.Equal(1, _rig.Live("$x"));
    }
}
