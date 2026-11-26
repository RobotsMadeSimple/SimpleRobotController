using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Execution
{
    // ── MoveJoints (MoveAbsJ) ─────────────────────────────────────────────────

    /// <summary>
    /// Move to Joints: a joint-interpolated move straight to joint values — no Cartesian target, no
    /// IK (RAPID MoveAbsJ). The target starts from the current joints, takes a joints value from
    /// <c>jointsExpr</c> when set (<c>$home[0]</c> — an Objects-list element with j1…j4 — or
    /// <c>jointsOf(…)</c>), then any of <c>j1</c>…<c>j4</c> (literal or expression) on top. Joints out of
    /// their soft limits fail with the catchable <c>outOfReach</c> before anything moves.
    /// ASTRO: J1 °, J2 radial mm, J3 vertical mm, J4 °. CNC4Axis (identity kinematics): X, Y, Z, RZ.
    /// </summary>
    internal sealed class MoveJointsStep : IStepHandler
    {
        public static readonly MoveJointsStep Instance = new();

        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            if (ctx.Motion.AwaitingMove) return StepOutcome.Yield;
            string desc = ProgressReporter.StepDescription(step);
            var ev = ctx.Eval;

            var joints = ctx.Controller.LiveJoints;
            if (!string.IsNullOrWhiteSpace(step.JointsExpr))
            {
                if (!ev.TryEvaluateValue(step.JointsExpr, out var value, out var kind))
                    return ctx.Finish(ProgramStatus.Error,
                        $"{desc}: '{step.JointsExpr}' is not a joints value — use $list[i] (an Objects list with j1…j4) or jointsOf(…)");
                if (!Reachability.IsJointsRecord(value))
                    return ctx.Finish(ProgramStatus.Error,
                        $"{desc}: '{step.JointsExpr}' is a {(kind == "pose" ? "pose" : kind + " element")}, not joint values — " +
                        "convert a point with jointsOf(…)");
                joints = Reachability.FromRecord(value, joints);
            }

            joints = new Vector6(
                ev.OptionalField(step, "j1", step.J1) ?? joints.X,
                ev.OptionalField(step, "j2", step.J2) ?? joints.Y,
                ev.OptionalField(step, "j3", step.J3) ?? joints.Z,
                0, 0,
                ev.OptionalField(step, "j4", step.J4) ?? joints.RZ);

            var check = ctx.Controller.CheckJointTarget(joints);
            if (!check.Ok)
                return ctx.Fail(ErrorCodes.OutOfReach, $"{desc}: the joint target is out of range — {check.Reason}");

            ctx.Motion.Enqueue(new RobotCommand
            {
                CommandType = "MoveJoints",
                X  = joints.X,
                Y  = joints.Y,
                Z  = joints.Z,
                RZ = joints.RZ,
                // Raw speed — the global override is applied centrally, as for MoveJ.
                Speed = ev.OptionalField(step, "speed", step.Speed),
                Accel = ev.OptionalField(step, "accel", step.Accel),
                Decel = ev.OptionalField(step, "decel", step.Decel),
                ApplySpeedOverride = true,
                Profile = step.MotionProfile,
            });
            ctx.AwaitMove(step, frame);
            ctx.ArmMoveWatch(step);
            return StepOutcome.Yield;
        }
    }
}
