using Controller.RobotControl.Execution;
using Controller.RobotControl.Programs;

namespace Controller.RobotControl
{
    // Manual MoveL / MoveJ to a grid cell or stack slot (the app's "tap a cell, move to it"): the
    // target is resolved exactly as a program's grid / stack move resolves it — MoveTargetResolver's
    // GridCell / StackSlot, then the active local frame, with the active tool — and reach-checked
    // before anything is queued. The e-stop gate (AddCommand) runs before this, like every Move.
    internal partial class RobotController
    {
        /// <summary>
        /// Resolves <paramref name="cmd"/>'s grid / stack target into its x…rz (and clears
        /// <c>name</c>, so the motion thread queues it as a raw world-frame vector). Null when the
        /// move may be queued, with <paramref name="ack"/> the extra ACK fields (the resolved
        /// target); otherwise the refusal to return (<c>pointNotFound</c>, <c>indexOutOfRange</c>,
        /// <c>outOfReach</c>, <c>badParams</c>).
        /// </summary>
        internal object? ResolveArrayMove(RobotCommand cmd, out object? ack)
        {
            ack = null;
            Point basePose;
            string label;
            object cell;
            if (cmd.Grid != null)
            {
                var grid = gridRepo.Get(cmd.Grid);
                if (grid == null) return Refuse(ErrorCodes.PointNotFound, $"Grid not found: {cmd.Grid}");
                var bp = pointRepo.Get(grid.BasePointName);
                if (bp == null) return Refuse(ErrorCodes.PointNotFound, $"Grid base point not found: {grid.BasePointName}");
                if (cmd.Row is not { } row || cmd.Col is not { } col) return Refuse("badParams", "A grid move needs row and col");
                if (row < 0 || (grid.RowCount is > 0 && row >= grid.RowCount))
                    return Refuse(ErrorCodes.IndexOutOfRange, $"Row {row} is outside grid '{grid.Name}'");
                if (col < 0 || (grid.ColCount is > 0 && col >= grid.ColCount))
                    return Refuse(ErrorCodes.IndexOutOfRange, $"Column {col} is outside grid '{grid.Name}'");
                basePose = MoveTargetResolver.GridCell(grid, bp, row, col);
                label = $"Grid '{grid.Name}' row {row}, col {col}";
                cell = new { grid = grid.Id, row, col };
            }
            else
            {
                var stack = stackRepo.Get(cmd.Stack!);
                if (stack == null) return Refuse(ErrorCodes.PointNotFound, $"Stack not found: {cmd.Stack}");
                var bp = pointRepo.Get(stack.BasePointName);
                if (bp == null) return Refuse(ErrorCodes.PointNotFound, $"Stack base point not found: {stack.BasePointName}");
                if (cmd.Index is not { } index) return Refuse("badParams", "A stack move needs index");
                if (index < 0 || (stack.MaxCount is > 0 && index >= stack.MaxCount))
                    return Refuse(ErrorCodes.IndexOutOfRange, $"Slot {index} is outside stack '{stack.Name}'");
                basePose = MoveTargetResolver.StackSlot(stack, bp, index);
                label = $"Stack '{stack.Name}' slot {index}";
                cell = new { stack = stack.Id, index };
            }

            // Saved geometry is base-frame: the active local transforms it, as for a program move.
            var p = new Vector6(basePose.X, basePose.Y, basePose.Z, basePose.RX, basePose.RY, basePose.RZ);
            var world = ActiveLocalOffset is { } loc ? LocalFrame.Apply(loc, p) : p;

            if (ManualMoveReach(world, cmd.ToolOffsetVector6, line: cmd.CommandType == "MoveL") is { Ok: false } r)
                return Refuse(ErrorCodes.OutOfReach, $"{label} is out of reach — {r.Reason}");

            cmd.Name = null;
            cmd.X = world.X; cmd.Y = world.Y; cmd.Z = world.Z;
            cmd.RX = world.RX; cmd.RY = world.RY; cmd.RZ = world.RZ;
            ack = new
            {
                resolvedBy = "controller",
                cell,
                target = new { x = world.X, y = world.Y, z = world.Z, rx = world.RX, ry = world.RY, rz = world.RZ },
            };
            return null;
        }

        /// <summary>
        /// The program moves' pre-move reach check for a manual move: the end (with the per-move tool
        /// offset, solved with the active tool), and for a line every ~5 mm from the live position —
        /// skipped between the ends when the start itself is out of range (a move back into range).
        /// </summary>
        internal ReachResult ManualMoveReach(Vector6 world, Vector6 toolOffset, bool line)
        {
            var tool = CurrentTool;
            bool hasOffset = toolOffset.X != 0 || toolOffset.Y != 0 || toolOffset.Z != 0 || toolOffset.RX != 0 || toolOffset.RY != 0 || toolOffset.RZ != 0;
            Vector6 W(Vector6 t) => hasOffset ? ApplyToolOffset(t, toolOffset) : t;
            var end = CheckReach(W(world), tool);
            if (!end.Ok || !line) return end;
            var start = LivePosition;
            if (!CheckReach(W(start), tool).Ok) return end;
            int n = (int)Math.Clamp(Math.Ceiling(start.Distance3(world) / 5.0), 1, 200);
            for (int i = 1; i < n; i++)
            {
                var r = CheckReach(W(start + (world - start) * ((double)i / n)), tool);
                if (!r.Ok) return r;
            }
            return end;
        }

        private static object Refuse(string error, string message) => new { ok = false, error, message };
    }
}
