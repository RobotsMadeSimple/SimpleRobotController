namespace Controller.RobotControl.Execution
{
    // The reach functions' view of the robot (reachable / jointsOf / poseOf — ExpressionEvaluator.Reach.cs):
    // saved geometry by name, the program's active tool and local, and the controller's own IK,
    // joint windows and FK. Points, grids and stacks resolve exactly as a move target would.
    internal sealed partial class RobotPropertySource : IMotionQueries
    {
        bool IMotionQueries.MotionAvailable => _robot != null;

        // A program resolves saved geometry through its own executor's repositories (the
        // controller's, in the running app); outside a program, the controller's.
        // Program-aware read (the running program's own points over the global library); the global
        // repo implements IPointLookup, so outside a program it resolves the library directly.
        private Persistence.IPointLookup? Points => _run is ExecutionContext ctx ? ctx.TargetSources.PointReader : _robot?.pointRepo;
        private Persistence.GridRepository?  Grids  => _run is ExecutionContext ctx ? ctx.TargetSources.Grids  : _robot?.gridRepo;
        private Persistence.StackRepository? Stacks => _run is ExecutionContext ctx ? ctx.TargetSources.Stacks : _robot?.stackRepo;

        bool IMotionQueries.TryPoint(string name, out Vector6 point)
        {
            point = Vector6.Zero;
            var p = Points?.Get(name);
            if (p == null) return false;
            point = new Vector6(p.X, p.Y, p.Z, p.RX, p.RY, p.RZ);
            return true;
        }

        private Grid? FindGrid(string nameOrId) =>
            Grids?.Get(nameOrId)
            ?? Grids?.GetAll().FirstOrDefault(g => string.Equals(g.Name, nameOrId, StringComparison.OrdinalIgnoreCase));

        private RobotStack? FindStack(string nameOrId) =>
            Stacks?.Get(nameOrId)
            ?? Stacks?.GetAll().FirstOrDefault(s => string.Equals(s.Name, nameOrId, StringComparison.OrdinalIgnoreCase));

        bool IMotionQueries.TryGridCell(string grid, int row, int col, out Vector6 point, out string error)
        {
            point = Vector6.Zero;
            var g = FindGrid(grid);
            if (g == null) { error = $"no grid named '{grid}'"; return false; }
            var basePoint = Points!.Get(g.BasePointName);
            if (basePoint == null) { error = $"grid '{g.Name}': base point '{g.BasePointName}' not found"; return false; }
            var c = MoveTargetResolver.GridCell(g, basePoint, row, col);
            point = new Vector6(c.X, c.Y, c.Z, c.RX, c.RY, c.RZ);
            error = "";
            return true;
        }

        bool IMotionQueries.TryGridIndex(string grid, int index, out Vector6 point, out string error)
        {
            point = Vector6.Zero;
            var g = FindGrid(grid);
            if (g == null) { error = $"no grid named '{grid}'"; return false; }
            if (!g.ColCount.HasValue || g.ColCount.Value <= 0)
            {
                error = $"grid '{g.Name}' needs a column count to use a cell index — write grid(\"{g.Name}\", row, col)";
                return false;
            }
            return ((IMotionQueries)this).TryGridCell(g.Id, index / g.ColCount.Value, index % g.ColCount.Value, out point, out error);
        }

        bool IMotionQueries.TryStackSlot(string stack, int index, out Vector6 point, out string error)
        {
            point = Vector6.Zero;
            var s = FindStack(stack);
            if (s == null) { error = $"no stack named '{stack}'"; return false; }
            var basePoint = Points!.Get(s.BasePointName);
            if (basePoint == null) { error = $"stack '{s.Name}': base point '{s.BasePointName}' not found"; return false; }
            var c = MoveTargetResolver.StackSlot(s, basePoint, index);
            point = new Vector6(c.X, c.Y, c.Z, c.RX, c.RY, c.RZ);
            error = "";
            return true;
        }

        bool IMotionQueries.TryTool(string? name, out Vector6 tool, out string error)
        {
            error = "";
            name = name?.Trim();
            if (string.IsNullOrEmpty(name)) { var t = _robot!.CurrentTool; tool = new Vector6(t.X, t.Y, t.Z, t.RX, t.RY, t.RZ); return true; }
            if (name.Equals("none", StringComparison.OrdinalIgnoreCase)) { tool = Vector6.Zero; return true; }
            var saved = _robot!.toolRepo.Get(name);
            if (saved == null) { tool = Vector6.Zero; error = $"no tool named '{name}'"; return false; }
            tool = new Vector6(saved.X, saved.Y, saved.Z, saved.RX, saved.RY, saved.RZ);
            return true;
        }

        bool IMotionQueries.TryLocal(string? name, out Vector6? local, out string error)
        {
            error = "";
            name = name?.Trim();
            if (string.IsNullOrEmpty(name))
            {
                // The program's active local (SetLocal / ClearLocal), else the robot's.
                local = _run is ExecutionContext ctx ? ctx.ActiveLocal : _robot!.ActiveLocalOffset;
                return true;
            }
            if (name.Equals("none", StringComparison.OrdinalIgnoreCase)) { local = null; return true; }
            var saved = (_run is ExecutionContext lc ? lc.Locals : _robot!.localRepo).Get(name);
            if (saved == null) { local = null; error = $"no local frame named '{name}'"; return false; }
            local = new Vector6(saved.X, saved.Y, saved.Z, saved.RX, saved.RY, saved.RZ);
            return true;
        }

        ReachResult IMotionQueries.Reach(Vector6 worldTarget, Vector6 tool) => _robot!.CheckReach(worldTarget, tool);

        Vector6 IMotionQueries.Pose(Vector6 joints, Vector6 tool) => _robot!.PoseOfJoints(joints, tool);
    }
}
