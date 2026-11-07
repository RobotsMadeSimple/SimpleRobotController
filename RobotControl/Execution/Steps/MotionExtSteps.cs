using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Program editor v2 motion blocks (docs/program-language.md §4.3): MoveC (a circular arc
    /// through a via point, dispatched as one blended path), SavePosition ("Here": the TCP pose
    /// into a Points list or a saved point) and SetSpeedFactor (this program's speed percentage).
    /// </summary>
    internal static class MotionExtSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.MoveC]          = MoveCStep.Instance;
            r[StepType.SavePosition]   = SavePositionStep.Instance;
            r[StepType.SetSpeedFactor] = SetSpeedFactorStep.Instance;
        }

        /// <summary>
        /// The robot's pose as a program sees it: the live TCP (<paramref name="source"/>
        /// "actual", the default) or the last commanded target ("target"), expressed in the
        /// program's active local frame — so a later move to it under the same local returns
        /// to the same place (named points and points-list targets are local coordinates).
        /// </summary>
        internal static Vector6 ProgramPose(ExecutionContext ctx, string? source = null)
        {
            var world = source == "target" ? ctx.Controller.LiveTargetPosition : ctx.LivePosition();
            return ctx.ActiveLocal is { } local ? LocalFrame.Inverse(local, world) : world;
        }

        internal static Vector6Val ToVal(Vector6 p) => new() { X = p.X, Y = p.Y, Z = p.Z, RX = p.RX, RY = p.RY, RZ = p.RZ };
    }

    // ── MoveC ─────────────────────────────────────────────────────────────────

    /// <summary>
    /// Move Arc: the circle through the current pose, the via point and the end target (in 3D),
    /// cut into straight segments whose chord error is at most <c>arcTolerance</c> and sent as one
    /// continuous path through the existing blended-move dispatch. The start is where the robot
    /// is when the step runs (the executor has already waited for the previous move to finish).
    /// </summary>
    internal sealed class MoveCStep : IStepHandler
    {
        public static readonly MoveCStep Instance = new();

        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            if (ctx.Motion.AwaitingMove) return StepOutcome.Yield;
            string desc = ProgressReporter.StepDescription(step);

            if (!HasVia(step))
                return ctx.Finish(ProgramStatus.Error, $"{desc}: no via point is set");

            // A via linked only by id whose point was deleted would otherwise resolve to "the
            // current position" (a degenerate arc) — say what is actually wrong.
            if (string.IsNullOrEmpty(step.ViaPointNameExpr) && string.IsNullOrEmpty(step.ViaPointName)
                && ctx.TargetSources.Points.GetById(step.ViaPointId) == null)
                return ctx.Fail(ErrorCodes.PointNotFound, $"{desc}: the via point no longer exists");

            if (!MoveTargetResolver.TryResolve(ViaAsTarget(step), ctx.TargetSources, ctx.Vars, ctx.ActiveLocal,
                                               ctx.LivePosition, out var via, out var viaError))
                return ctx.Fail(ErrorCodes.PointNotFound, $"{desc}: via point — {viaError}");

            if (!ctx.ResolveMoveTarget(step, out Vector6 end)) return StepOutcome.Finished;

            var start = ctx.LivePosition();
            double tolerance = step.ArcTolerance is double t && t > 0 && double.IsFinite(t) ? t : ArcPath.DefaultTolerance;

            if (!ArcPath.TryTessellate(start, via, end, tolerance, out var waypoints, out var radii, out var arcError))
                return ctx.Fail(ErrorCodes.ArcDegenerate, $"{desc}: {arcError}");

            var ev = ctx.Eval;
            ctx.Motion.StartContinuous(waypoints, radii,
                ev.OptionalField(step, "speed", step.Speed),
                ev.OptionalField(step, "accel", step.Accel),
                ev.OptionalField(step, "decel", step.Decel));
            ctx.AwaitMove(step, frame);
            return StepOutcome.Yield;
        }

        internal static bool HasVia(ProgramStep s) =>
            !string.IsNullOrEmpty(s.ViaPointId) || !string.IsNullOrEmpty(s.ViaPointName) || !string.IsNullOrEmpty(s.ViaPointNameExpr);

        /// <summary>
        /// The via point as a move target of its own: the via reference in the target fields, with
        /// the step's offsets, overrides and local frame — they apply to both points of the arc.
        /// </summary>
        internal static ProgramStep ViaAsTarget(ProgramStep s) => new()
        {
            Id            = s.Id,
            Type          = s.Type,
            PointId       = s.ViaPointId,
            PointName     = s.ViaPointName,
            PointNameExpr = s.ViaPointNameExpr,
            OffsetX  = s.OffsetX,  OffsetY  = s.OffsetY,  OffsetZ  = s.OffsetZ,
            OffsetRX = s.OffsetRX, OffsetRY = s.OffsetRY, OffsetRZ = s.OffsetRZ,
            OverrideX  = s.OverrideX,  OverrideY  = s.OverrideY,  OverrideZ  = s.OverrideZ,
            OverrideRX = s.OverrideRX, OverrideRY = s.OverrideRY, OverrideRZ = s.OverrideRZ,
            LocalId     = s.LocalId,
            LocalName   = s.LocalName,
            Expressions = s.Expressions,
        };
    }

    /// <summary>
    /// Circle-through-three-points geometry for MoveC. Pure math: no robot, no executor.
    /// </summary>
    internal static class ArcPath
    {
        /// <summary>Default largest gap between the true arc and its straight segments (mm).</summary>
        public const double DefaultTolerance = 0.1;
        /// <summary>Smallest tolerance honoured (mm); a smaller value is raised to this.</summary>
        public const double MinTolerance = 0.001;
        /// <summary>Largest angle one segment may span, whatever the tolerance allows.</summary>
        public const double MaxSegmentDeg = 2.0;
        /// <summary>Hard cap on the number of segments (a huge, very tight arc).</summary>
        public const int MaxSegments = 3600;

        private readonly record struct V3(double X, double Y, double Z)
        {
            public static V3 Of(Vector6 p) => new(p.X, p.Y, p.Z);
            public static V3 operator +(V3 a, V3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
            public static V3 operator -(V3 a, V3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
            public static V3 operator *(V3 a, double k) => new(a.X * k, a.Y * k, a.Z * k);
            public double Dot(V3 b) => X * b.X + Y * b.Y + Z * b.Z;
            public V3 Cross(V3 b) => new(Y * b.Z - Z * b.Y, Z * b.X - X * b.Z, X * b.Y - Y * b.X);
            public double Length => Math.Sqrt(Dot(this));
        }

        /// <summary>The circle through three points: centre, radius, the in-plane unit axes
        /// (<c>E1</c> towards the start, <c>E2</c> a quarter turn on in the direction of travel)
        /// and the sweep from start to end through via (radians, 0–2π).</summary>
        internal readonly record struct Circle(double CX, double CY, double CZ, double Radius,
            double E1X, double E1Y, double E1Z, double E2X, double E2Y, double E2Z, double Sweep, double ViaAngle);

        /// <summary>
        /// The circle through <paramref name="start"/>, <paramref name="via"/> and <paramref name="end"/>.
        /// False (with a reason) when two of them coincide or all three lie on one line.
        /// </summary>
        public static bool TryCircle(Vector6 start, Vector6 via, Vector6 end, out Circle circle, out string error)
        {
            circle = default;
            var a = V3.Of(start); var b = V3.Of(via); var c = V3.Of(end);
            var ab = b - a; var ac = c - a; var bc = c - b;
            double lab = ab.Length, lac = ac.Length, lbc = bc.Length;
            const double MinGap = 1e-6; // mm
            if (lab < MinGap) { error = "the via point is the start position — an arc needs three different points"; return false; }
            if (lbc < MinGap) { error = "the via point is the end point — an arc needs three different points"; return false; }
            if (lac < MinGap) { error = "the end point is the start position — use two moves for a full circle"; return false; }

            var n = ab.Cross(ac);
            double nl = n.Length;
            // sin of the angle at the start between the two chords: ~0 means a straight line.
            if (nl / (lab * lac) < 1e-9)
            {
                error = "the start, via and end points are in a line — no arc passes through them";
                return false;
            }

            // Circumcentre: a + (|ac|² (n × ab) + |ab|² (ac × n)) / (2 |n|²)
            var centre = a + (n.Cross(ab) * (lac * lac) + ac.Cross(n) * (lab * lab)) * (1.0 / (2 * nl * nl));
            var ra = a - centre;
            double radius = ra.Length;
            var e1 = ra * (1.0 / radius);
            var nhat = n * (1.0 / nl);
            var e2 = nhat.Cross(e1); // a→b→c turn counter-clockwise about n, so e2 points along the travel

            double Angle(V3 p)
            {
                var d = p - centre;
                double ang = Math.Atan2(d.Dot(e2), d.Dot(e1));
                return ang < 0 ? ang + 2 * Math.PI : ang;
            }

            double sweep = Angle(c);
            double viaAngle = Angle(b);
            if (sweep <= 0) sweep += 2 * Math.PI;
            circle = new Circle(centre.X, centre.Y, centre.Z, radius, e1.X, e1.Y, e1.Z, e2.X, e2.Y, e2.Z, sweep, viaAngle);
            error = "";
            return true;
        }

        /// <summary>
        /// How many equal segments the arc needs so each chord's gap to the arc (the sagitta,
        /// R·(1 − cos(θ/2))) stays within <paramref name="tolerance"/>, with at most
        /// <see cref="MaxSegmentDeg"/> per segment; at least 2 (the blended path needs two waypoints).
        /// </summary>
        public static int SegmentCount(double radius, double sweep, double tolerance)
        {
            tolerance = Math.Max(tolerance, MinTolerance);
            double maxStep = MaxSegmentDeg * Math.PI / 180.0;
            if (tolerance < 2 * radius)
                maxStep = Math.Min(maxStep, 2 * Math.Acos(1 - tolerance / radius));
            int n = (int)Math.Ceiling(sweep / maxStep - 1e-9);
            return Math.Clamp(n, 2, MaxSegments);
        }

        /// <summary>
        /// The arc's waypoints (excluding the start, ending exactly on <paramref name="end"/>) and the
        /// blend radius for each — the arc radius at every interior waypoint, so the path stays
        /// round between them, and 0 at the end (an exact stop). RX/RY/RZ go linearly from the start
        /// to the end by arc angle; the via point's orientation is not used.
        /// </summary>
        public static bool TryTessellate(Vector6 start, Vector6 via, Vector6 end, double tolerance,
            out List<Vector6> waypoints, out List<double> radii, out string error)
        {
            waypoints = new();
            radii     = new();
            if (!TryCircle(start, via, end, out var c, out error)) return false;

            int n = SegmentCount(c.Radius, c.Sweep, tolerance);
            for (int k = 1; k <= n; k++)
            {
                double f = (double)k / n;
                if (k == n)
                {
                    waypoints.Add(end);
                    radii.Add(0);
                    break;
                }
                double phi = c.Sweep * f, cos = Math.Cos(phi) * c.Radius, sin = Math.Sin(phi) * c.Radius;
                waypoints.Add(new Vector6(
                    c.CX + cos * c.E1X + sin * c.E2X,
                    c.CY + cos * c.E1Y + sin * c.E2Y,
                    c.CZ + cos * c.E1Z + sin * c.E2Z,
                    start.RX + (end.RX - start.RX) * f,
                    start.RY + (end.RY - start.RY) * f,
                    start.RZ + (end.RZ - start.RZ) * f));
                radii.Add(c.Radius);
            }
            return true;
        }
    }

    // ── SavePosition ──────────────────────────────────────────────────────────

    /// <summary>
    /// Save Position ("Here"): the TCP pose, in the active local frame, into a Points list
    /// (replace / append / insert / set at an index) or a saved point (overwritten or created
    /// through the point repository, so the points history records it).
    /// </summary>
    internal sealed class SavePositionStep : IStepHandler
    {
        public static readonly SavePositionStep Instance = new();

        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            var pose = MotionExtSteps.ProgramPose(ctx, step.SavePositionSource);
            return step.SavePositionTo == "point" ? ToPoint(step, ctx, pose) : ToVariable(step, ctx, pose);
        }

        private static StepOutcome ToPoint(ProgramStep step, ExecutionContext ctx, Vector6 pose)
        {
            string desc = ProgressReporter.StepDescription(step);
            var points = ctx.TargetSources.Points;
            var name = step.PointName?.Trim();
            // By stable id first (rename-proof), then by name; a name that does not exist yet is created.
            var existing = points.GetById(step.PointId) ?? (string.IsNullOrEmpty(name) ? null : points.Get(name));
            if (existing?.Name is { } existingName)
            {
                points.EditPoint(existingName, new Dictionary<string, object?>
                {
                    ["X"] = pose.X, ["Y"] = pose.Y, ["Z"] = pose.Z, ["RX"] = pose.RX, ["RY"] = pose.RY, ["RZ"] = pose.RZ,
                });
            }
            else if (!string.IsNullOrEmpty(name))
            {
                points.SavePoint(name, pose);
            }
            else if (!string.IsNullOrEmpty(step.PointId))
            {
                return ctx.Fail(ErrorCodes.PointNotFound, $"{desc}: the saved point no longer exists");
            }
            else
            {
                return ctx.Finish(ProgramStatus.Error, $"{desc}: no saved point is selected");
            }
            return StepOutcome.Advance;
        }

        private static StepOutcome ToVariable(ProgramStep step, ExecutionContext ctx, Vector6 pose)
        {
            string desc = ProgressReporter.StepDescription(step);
            var name = step.VariableName?.Trim().TrimStart('$') ?? "";
            if (name.Length == 0) return ctx.Finish(ProgramStatus.Error, $"{desc}: no Points variable is selected");

            var vars = ctx.Vars;
            vars.Lists.TryGetValue(name, out var list);
            if (list != null && list.ElementType != ListElementType.Point)
                return ctx.Fail(ErrorCodes.BadListOperation, $"{desc}: '${name}' is a {list.ElementType} list, not a Points list");

            var element = ObjectRecord.FromPoint(MotionExtSteps.ToVal(pose));
            string mode = string.IsNullOrEmpty(step.SavePositionMode) ? "replace" : step.SavePositionMode;
            if (mode == "replace")
            {
                // Exactly one element, as a stop condition's position variable is: $var[0] is the pose.
                vars.SetList(name, new ListVar { ElementType = ListElementType.Point, Items = [element] });
                return StepOutcome.Advance;
            }

            var items = list != null ? new List<ObjectRecord>(list.Items) : new List<ObjectRecord>();
            switch (mode)
            {
                case "append":
                    items.Add(element);
                    break;
                case "insert":
                case "set":
                {
                    int index = ListOpStep.ToIndex(ctx.Eval.EvalField(step, "savePositionIndex", step.SavePositionIndex ?? 0));
                    int max = mode == "insert" ? items.Count : items.Count - 1;
                    if (index < 0 || index > max)
                        return ctx.Fail(ErrorCodes.IndexOutOfRange,
                            $"{desc}: index {index} is outside '${name}' ({items.Count} point{(items.Count == 1 ? "" : "s")}; {(mode == "insert" ? "insert at" : "set")} 0–{max})");
                    if (mode == "insert") items.Insert(index, element);
                    else items[index] = element;
                    break;
                }
                default:
                    return ctx.Fail(ErrorCodes.BadListOperation, $"{desc}: unknown mode '{mode}' (replace, append, insert or set)");
            }
            vars.SetList(name, new ListVar { ElementType = ListElementType.Point, Items = items });
            return StepOutcome.Advance;
        }
    }

    // ── SetSpeedFactor ────────────────────────────────────────────────────────

    /// <summary>
    /// Set Speed Factor: every later move of this program runs at operator override × factor / 100
    /// (1–100 %, so it can only slow the program down). Back to 100 when the run ends or is reset.
    /// </summary>
    internal sealed class SetSpeedFactorStep : IStepHandler
    {
        public static readonly SetSpeedFactorStep Instance = new();

        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            // Clamped to 1–100 by the dispatcher; a non-number reads as 100.
            ctx.Motion.SpeedFactorPercent = ctx.Eval.EvalField(step, "speedFactor", step.SpeedFactor ?? 100);
            return StepOutcome.Advance;
        }
    }
}
