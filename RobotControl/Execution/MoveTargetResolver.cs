using System.Text.RegularExpressions;
using Controller.RobotControl.Persistence;

namespace Controller.RobotControl.Execution
{
    /// <summary>The saved geometry a move target can reference.</summary>
    internal readonly record struct MoveTargetSources(
        PointRepository Points, GridRepository Grids, StackRepository Stacks, LocalRepository Locals);

    /// <summary>
    /// Pure target math for move steps: base pose (grid cell, stack slot, points-list
    /// element, named point or the current position) + offsets + per-axis overrides + the
    /// active local frame, plus the per-move tool offset and the resume-point search used
    /// when a paused blended path continues.
    /// </summary>
    /// <remarks>
    /// Nothing here touches the robot or the executor: inputs are the step, the saved
    /// geometry, a variable scope (read through its evaluation snapshot) and the positions
    /// the caller supplies. Errors come back as a message for the caller to report, except
    /// an unknown variable in an expression, which propagates as
    /// <see cref="UnknownVariableException"/> like everywhere else.
    /// </remarks>
    internal static class MoveTargetResolver
    {
        /// <summary>
        /// Resolves a move step's final Cartesian target.
        /// </summary>
        /// <param name="activeLocal">The program's active local frame, or null.</param>
        /// <param name="livePosition">Reads the robot's live position; only called for a
        /// move with no target (a relative move) when <paramref name="currentPos"/> is null.</param>
        /// <param name="currentPos">Overrides the base used for "current position" moves —
        /// during a blended run this is the previous waypoint (where the robot ends up), not
        /// its live position.</param>
        /// <returns>False with <paramref name="error"/> set when the target cannot be resolved.</returns>
        public static bool TryResolve(
            ProgramStep step, in MoveTargetSources src, VariableScope vars, Vector6? activeLocal,
            Func<Vector6> livePosition, out Vector6 target, out string error, Vector6? currentPos = null)
        {
            target = Vector6.Zero;
            error  = "";
            var ev = vars.Eval;

            // True when the base pose comes from the live current position — those
            // axes are already world-frame, so the local shift must skip them.
            bool baseIsCurrent = false;

            Point point;
            if (step.GridPoint != null)
            {
                var gp   = step.GridPoint;
                var grid = src.Grids.Get(gp.GridId);
                if (grid == null) { error = $"Grid not found: {gp.GridId}"; return false; }

                var basePoint = src.Points.Get(grid.BasePointName);
                if (basePoint == null) { error = $"Grid base point not found: {grid.BasePointName}"; return false; }

                int row, col;
                if (gp.UseGridIndex)
                {
                    if (!grid.ColCount.HasValue || grid.ColCount.Value <= 0)
                    {
                        error = $"Grid '{grid.Name}' requires colCount to use grid index";
                        return false;
                    }
                    int idx = (int)Math.Round(ev.EvalField(step, "gridGridIndex", gp.GridIndex ?? 0));
                    row = idx / grid.ColCount.Value;
                    col = idx % grid.ColCount.Value;
                }
                else
                {
                    row = (int)Math.Round(ev.EvalField(step, "gridRowIndex", gp.RowIndex ?? 0));
                    col = (int)Math.Round(ev.EvalField(step, "gridColIndex", gp.ColIndex ?? 0));
                }

                point = GridCell(grid, basePoint, row, col);
            }
            else if (step.StackPoint != null)
            {
                var sp    = step.StackPoint;
                var stack = src.Stacks.Get(sp.StackId);
                if (stack == null) { error = $"Stack not found: {sp.StackId}"; return false; }

                var basePoint = src.Points.Get(stack.BasePointName);
                if (basePoint == null) { error = $"Stack base point not found: {stack.BasePointName}"; return false; }

                int idx = (int)Math.Round(ev.EvalField(step, "stackIndex", sp.Index ?? 0));
                point = StackSlot(stack, basePoint, idx);
            }
            else if (!string.IsNullOrEmpty(step.VarPointName))
            {
                if (!vars.TryGetPointList(step.VarPointName, out var ptList) || ptList.Count == 0)
                {
                    error = $"Variable point '{step.VarPointName}' is empty or not set";
                    return false;
                }
                point = ListElement(ptList, step.VarPointIndex, ev);
            }
            else if (!string.IsNullOrEmpty(step.PointNameExpr))
            {
                // One field, two kinds of target. An expression that is nothing but an
                // indexed points variable ("$pts[$i]") already *is* a coordinate, so it is
                // used directly; anything else is text naming a saved point. Both are
                // resolved fresh on every execution, so assigning the variables they
                // reference retargets the move.
                if (TryParsePointsRef(step.PointNameExpr, out var refName, out var idxExpr) &&
                    vars.TryGetPointList(refName, out var ptList))
                {
                    if (ptList.Count == 0)
                    {
                        error = $"Points variable '{refName}' is empty or not set";
                        return false;
                    }
                    point = ListElement(ptList, idxExpr, ev);
                }
                else
                {
                    var targetName = vars.Interpolate(step.PointNameExpr).Trim();
                    if (string.IsNullOrEmpty(targetName))
                    {
                        error = $"Point name '{step.PointNameExpr}' resolved to nothing";
                        return false;
                    }

                    var namedPoint = src.Points.Get(targetName);
                    if (namedPoint is null)
                    {
                        error = $"Point not found: {targetName} (from '{step.PointNameExpr}')";
                        return false;
                    }
                    point = namedPoint;
                }
            }
            else if (string.IsNullOrEmpty(step.PointName))
            {
                // No point specified — use the base TCP position so offsets act as relative
                // displacements. In a blended run this is the previous waypoint (where the
                // robot will be), otherwise the robot's live current position.
                baseIsCurrent = true;
                var pos = currentPos ?? livePosition();
                point = new Point { X = pos.X, Y = pos.Y, Z = pos.Z, RX = pos.RX, RY = pos.RY, RZ = pos.RZ };
            }
            else
            {
                var found = src.Points.Get(step.PointName);
                if (found is null)
                {
                    error = $"Point not found: {step.PointName}";
                    return false;
                }
                point = found;
            }

            // Base position + offsets
            double finalX  = point.X  + ev.EvalField(step, "offsetX",  step.OffsetX  ?? 0);
            double finalY  = point.Y  + ev.EvalField(step, "offsetY",  step.OffsetY  ?? 0);
            double finalZ  = point.Z  + ev.EvalField(step, "offsetZ",  step.OffsetZ  ?? 0);
            double finalRX = point.RX + ev.EvalField(step, "offsetRX", step.OffsetRX ?? 0);
            double finalRY = point.RY + ev.EvalField(step, "offsetRY", step.OffsetRY ?? 0);
            double finalRZ = point.RZ + ev.EvalField(step, "offsetRZ", step.OffsetRZ ?? 0);

            // Per-axis absolute overrides — replace calculated value when set
            double? oX  = ev.OptionalField(step, "overrideX",  step.OverrideX);
            double? oY  = ev.OptionalField(step, "overrideY",  step.OverrideY);
            double? oZ  = ev.OptionalField(step, "overrideZ",  step.OverrideZ);
            double? oRX = ev.OptionalField(step, "overrideRX", step.OverrideRX);
            double? oRY = ev.OptionalField(step, "overrideRY", step.OverrideRY);
            double? oRZ = ev.OptionalField(step, "overrideRZ", step.OverrideRZ);
            bool ovX = oX.HasValue, ovY = oY.HasValue, ovZ = oZ.HasValue, ovRZ = oRZ.HasValue;
            finalX  = oX  ?? finalX;
            finalY  = oY  ?? finalY;
            finalZ  = oZ  ?? finalZ;
            finalRX = oRX ?? finalRX;
            finalRY = oRY ?? finalRY;
            finalRZ = oRZ ?? finalRZ;

            // Apply active local offset — per-step localName overrides the program-level active local
            Vector6? effectiveLocal;
            if (!string.IsNullOrEmpty(step.LocalName))
            {
                var stepLocal = src.Locals.Get(step.LocalName);
                effectiveLocal = stepLocal != null ? new Vector6(stepLocal.X, stepLocal.Y, stepLocal.Z, stepLocal.RX, stepLocal.RY, stepLocal.RZ) : null;
            }
            else
            {
                effectiveLocal = activeLocal;
            }
            if (effectiveLocal != null)
            {
                // The local transforms ABSOLUTE coordinates: named/grid/stack/var
                // points and overridden axes. Axes riding on the live current
                // position are already world-frame — transforming them again would
                // double-apply the local on every relative move.
                //
                // When both X and Y are absolute the full rigid frame applies —
                // rotation (including RX/RY tilt, which maps XY motion onto the
                // frame's Z slope) around the local origin plus translation. With
                // mixed/relative XY a rotation is ill-defined, so those fall back
                // to per-axis translation (tilt unsupported there).
                if (!baseIsCurrent || (ovX && ovY))
                {
                    // When Z rides the live position, hold the local-frame height
                    // of the current base so the target still follows a tilted
                    // frame's slope as XY moves.
                    double lz = (!baseIsCurrent || ovZ)
                        ? finalZ
                        : LocalFrame.Inverse(effectiveLocal, new Vector6(point.X, point.Y, point.Z)).Z
                          + (finalZ - point.Z);
                    var w = LocalFrame.Apply(effectiveLocal,
                        new Vector6(finalX, finalY, lz, finalRX, finalRY, finalRZ));
                    finalX = w.X; finalY = w.Y; finalZ = w.Z;
                    // RX/RY pass through (tool can't tilt); yaw only when absolute
                    if (!baseIsCurrent || ovRZ) finalRZ = w.RZ;
                }
                else
                {
                    if (ovX)  finalX  += effectiveLocal.X;
                    if (ovY)  finalY  += effectiveLocal.Y;
                    if (ovZ)  finalZ  += effectiveLocal.Z;
                    if (ovRZ) finalRZ += effectiveLocal.RZ;
                }
            }

            target = new Vector6(finalX, finalY, finalZ, finalRX, finalRY, finalRZ);
            return true;
        }

        /// <summary>A grid cell: row/column offsets rotated about the base point's Z axis.</summary>
        internal static Point GridCell(Grid grid, Point basePoint, int row, int col)
        {
            double rawX = row * grid.RowOffsetX + col * grid.ColOffsetX;
            double rawY = row * grid.RowOffsetY + col * grid.ColOffsetY;
            double rawZ = row * grid.RowOffsetZ + col * grid.ColOffsetZ;

            double theta = grid.Rotation * Math.PI / 180.0;
            double rotX  = rawX * Math.Cos(theta) - rawY * Math.Sin(theta);
            double rotY  = rawX * Math.Sin(theta) + rawY * Math.Cos(theta);

            return new Point
            {
                X  = basePoint.X  + rotX,
                Y  = basePoint.Y  + rotY,
                Z  = basePoint.Z  + rawZ,
                RX = basePoint.RX,
                RY = basePoint.RY,
                RZ = basePoint.RZ,
            };
        }

        /// <summary>A stack slot; the index wraps round-robin when the stack has a MaxCount.</summary>
        internal static Point StackSlot(RobotStack stack, Point basePoint, int idx)
        {
            if (stack.MaxCount.HasValue && stack.MaxCount.Value > 0)
                idx = ((idx % stack.MaxCount.Value) + stack.MaxCount.Value) % stack.MaxCount.Value;

            return new Point
            {
                X  = basePoint.X  + idx * stack.OffsetX,
                Y  = basePoint.Y  + idx * stack.OffsetY,
                Z  = basePoint.Z  + idx * stack.OffsetZ,
                RX = basePoint.RX,
                RY = basePoint.RY,
                RZ = basePoint.RZ,
            };
        }

        /// <summary>One element of a (non-empty) points list; the index is clamped, and a
        /// malformed index expression reads as 0.</summary>
        private static Point ListElement(ListVar ptList, string? indexExpr, EvalContext ev)
        {
            int ptIdx = 0;
            if (!string.IsNullOrEmpty(indexExpr))
            {
                try { ptIdx = (int)Math.Round(ev.Evaluate(indexExpr)); }
                catch (UnknownVariableException) { throw; }
                catch (ExpressionParseException) { throw; }
                catch { /* malformed expression — default 0 */ }
            }
            ptIdx = Math.Clamp(ptIdx, 0, ptList.Count - 1);
            var vp = ptList.Items[ptIdx].ToPoint();
            return new Point { X = vp.X, Y = vp.Y, Z = vp.Z, RX = vp.RX, RY = vp.RY, RZ = vp.RZ };
        }

        // ── Tool offset ───────────────────────────────────────────────────────

        /// <summary>Whether the step carries a per-move tool offset on top of the active tool.</summary>
        public static bool HasToolOffset(ProgramStep step) =>
            step.ToolOffsetX.HasValue || step.ToolOffsetY.HasValue || step.ToolOffsetZ.HasValue
            || step.ToolOffsetRX.HasValue || step.ToolOffsetRY.HasValue || step.ToolOffsetRZ.HasValue;

        /// <summary>The step's evaluated tool offset (unset axes are 0), or null when it has none.</summary>
        public static Vector6? ResolveToolOffset(ProgramStep step, EvalContext ev)
        {
            if (!HasToolOffset(step)) return null;
            return new Vector6(
                ev.EvalField(step, "toolOffsetX",  step.ToolOffsetX  ?? 0),
                ev.EvalField(step, "toolOffsetY",  step.ToolOffsetY  ?? 0),
                ev.EvalField(step, "toolOffsetZ",  step.ToolOffsetZ  ?? 0),
                ev.EvalField(step, "toolOffsetRX", step.ToolOffsetRX ?? 0),
                ev.EvalField(step, "toolOffsetRY", step.ToolOffsetRY ?? 0),
                ev.EvalField(step, "toolOffsetRZ", step.ToolOffsetRZ ?? 0));
        }

        // ── Points-variable reference ─────────────────────────────────────────

        private static readonly Regex PointsRef = new(@"^\{?\s*\$(?<name>\w+)\s*\[(?<idx>[^\]]*)\]\s*\}?$");

        /// <summary>
        /// Recognises a pointNameExpr that is <em>only</em> an indexed variable reference,
        /// e.g. "$pts[$i]" or "{$pts[0]}" — the form that resolves to coordinates rather
        /// than to the name of a saved point.
        ///
        /// Deliberately anchored: "bin$pts[0]" is text being assembled, not a coordinate,
        /// and must fall through to the name lookup. Whether <paramref name="name"/> is
        /// actually a points variable is the caller's to decide, since only it knows the
        /// variable state.
        /// </summary>
        public static bool TryParsePointsRef(string expr, out string name, out string indexExpr)
        {
            var m = PointsRef.Match(expr?.Trim() ?? "");
            name      = m.Success ? m.Groups["name"].Value        : "";
            indexExpr = m.Success ? m.Groups["idx"].Value.Trim()  : "";
            return m.Success;
        }

        // ── Resume point ──────────────────────────────────────────────────────

        /// <summary>
        /// Index of the waypoint a paused continuous run should resume from: the
        /// stop position is projected onto every path segment (including the
        /// original start → first-waypoint leg) and the end of the nearest
        /// segment wins, so the resumed motion continues forward along the path.
        /// </summary>
        public static int FindResumeIndex(Vector6 pos, Vector6 start, List<Vector6> waypoints)
        {
            int best = 0;
            double bestD = double.MaxValue;
            var prev = start;
            for (int i = 0; i < waypoints.Count; i++)
            {
                var wp = waypoints[i];
                double d = DistToSegmentSq(pos, prev, wp);
                if (d < bestD) { bestD = d; best = i; }
                prev = wp;
            }
            return best;
        }

        internal static double DistToSegmentSq(Vector6 p, Vector6 a, Vector6 b)
        {
            double abx = b.X - a.X, aby = b.Y - a.Y, abz = b.Z - a.Z;
            double lenSq = abx * abx + aby * aby + abz * abz;
            double t = lenSq < 1e-12
                ? 0
                : Math.Clamp(((p.X - a.X) * abx + (p.Y - a.Y) * aby + (p.Z - a.Z) * abz) / lenSq, 0, 1);
            double dx = p.X - (a.X + t * abx);
            double dy = p.Y - (a.Y + t * aby);
            double dz = p.Z - (a.Z + t * abz);
            return dx * dx + dy * dy + dz * dz;
        }
    }
}
