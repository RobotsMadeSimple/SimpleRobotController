using Controller.RobotControl.MotionProfilers;

namespace Controller.RobotControl
{
    // Motion profiles (trapezoid / S-curve), the joint-target move (MoveJoints) and the pre-move
    // reach checks behind reachable() / jointsOf() / poseOf(). Homing and jogging never come
    // through here: they keep their own (trapezoid) profilers, untouched.
    internal partial class RobotController
    {
        /// <summary>
        /// The speed profile for one move: the move's own <paramref name="moveProfile"/>
        /// ("trapezoid" | "scurve") when set, else the robot config's <c>motionProfile</c>.
        /// Anything unrecognised reads as the trapezoid, the classic behaviour.
        /// </summary>
        internal MotionProfileSettings ResolveProfile(string? moveProfile)
        {
            var c = _config;
            var kind = MotionProfileSettings.ParseKind(moveProfile)
                       ?? MotionProfileSettings.ParseKind(c.MotionProfile)
                       ?? MotionProfileKind.Trapezoid;
            return kind == MotionProfileKind.SCurve
                ? new MotionProfileSettings(MotionProfileKind.SCurve, c.ScurveSmoothness, c.ScurveJerk is > 0 ? c.ScurveJerk : null)
                : MotionProfileSettings.Trapezoid;
        }

        /// <summary>The joint windows the motion thread enforces, or null while joint limits are off.</summary>
        internal (double lo, double hi)[]? ActiveJointLimits => _config.JointLimitsEnabled ? JointLimitWindows() : null;

        /// <summary>The joints' display names for the robot type: J1…J4 (ASTRO) or X, Y, Z, RZ (CNC).</summary>
        internal IReadOnlyList<string> JointNames => _config.RobotType == RobotTypes.Cnc4Axis ? CncJointNames : AstroJointNames;

        /// <summary>Can the robot put <paramref name="tool"/> at the world-frame <paramref name="worldTarget"/>?
        /// The real IK plus the enforced joint windows — nothing moves.</summary>
        internal ReachResult CheckReach(Vector6 worldTarget, Vector6 tool) =>
            Reachability.Check(_kinematics, worldTarget, tool, ActiveJointLimits, JointNames);

        /// <summary>Is the joint target inside the enforced joint windows?</summary>
        internal ReachResult CheckJointTarget(Vector6 joints) =>
            Reachability.CheckJoints(joints, ActiveJointLimits, JointNames);

        /// <summary>The world TCP pose of a joint vector with <paramref name="tool"/> (forward kinematics, no motion).</summary>
        internal Vector6 PoseOfJoints(Vector6 joints, Vector6 tool) => _kinematics.ForwardFromJoints(joints, tool);

        /// <summary>The live joint position (X = J1, Y = J2, Z = J3, RZ = J4) — the joint readouts.</summary>
        internal Vector6 LiveJoints
        {
            get
            {
                var (j1, j2, j3, j4) = _kinematics.GetJointAngles();
                return new Vector6(j1, j2, j3, 0, 0, j4);
            }
        }

        /// <summary>
        /// A joint-interpolated move straight to a joint target (MoveJoints / MoveAbsJ): X = J1,
        /// Y = J2, Z = J3, RZ = J4 — degrees or mm per joint (CNC: the axes). Same speed, override
        /// and fault rules as <see cref="MoveJ"/>; the joint-limit clamp still guards every tick.
        /// </summary>
        public void MoveJoints(Vector6 targetJoints, double? Speed, double? Accel, double? Decel,
                               bool applyOverride = false, double programFactor = 1.0, string? profile = null)
        {
            if (IsMoving)
                return;
            if (_faulted)  // no automatic moves while a joint-limit fault is latched
                return;

            double jointSpeed = (Speed ?? this.SpeedJ) * (applyOverride ? SpeedOverrideFactor * ProgramSpeedFactor(programFactor) : 1.0);
            double jointAccel = Accel ?? this.AccelJ;
            double jointDecel = Decel ?? this.DecelJ;

            this.TargetJoints   = new Vector6(targetJoints.X, targetJoints.Y, targetJoints.Z, 0, 0, targetJoints.RZ);
            this.TargetPosition = _kinematics.ForwardFromJoints(this.TargetJoints, this.CurrentTool);

            jointMotionProfiler = new(CurrentJointTargets, this.TargetJoints, jointSpeed, jointAccel, jointDecel, ResolveProfile(profile));
        }
    }
}
