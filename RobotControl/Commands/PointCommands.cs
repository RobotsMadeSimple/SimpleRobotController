namespace Controller.RobotControl.Commands;

/// <summary>Taught point repository.</summary>
internal sealed class PointCommands
{
    private readonly RobotController _robot;

    public PointCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetPoints",   GetPoints);
        d.Add("TeachPoint",  TeachPoint);
        d.Add("DeletePoint", DeletePoint);
        d.Add("EditPoint",   EditPoint);
    }

    private object? GetPoints(CommandMessage msg) => new
    {
        points = _robot.pointRepo.pointsJson
    };

    private void TeachPoint(CommandMessage msg)
    {
        var tp = CommandJson.LoadParams<TeachPointParams>(msg);
        // Points are stored base-frame: teaching under an active
        // local inverts the frame (rotation + translation), so
        // "move to point" returns exactly here while that local
        // stays active — and re-targets correctly when a
        // different local is applied later.
        var pos = _robot.LivePosition;
        var loc = _robot.ActiveLocalOffset;
        var basePos = loc == null ? pos : LocalFrame.Inverse(loc, pos);
        _robot.pointRepo.SavePoint(tp.Name, basePos);
    }

    private void DeletePoint(CommandMessage msg)
    {
        var dp = CommandJson.LoadParams<TeachPointParams>(msg);
        _robot.pointRepo.DeletePoint(dp.Name);
    }

    private void EditPoint(CommandMessage msg)
    {
        var ep = CommandJson.LoadParams<EditPointParams>(msg);
        var values = new Dictionary<string, object?>();
        if (ep.NewName != null)   values["Name"] = ep.NewName;
        if (ep.X.HasValue)        values["X"]    = ep.X.Value;
        if (ep.Y.HasValue)        values["Y"]    = ep.Y.Value;
        if (ep.Z.HasValue)        values["Z"]    = ep.Z.Value;
        if (ep.RX.HasValue)       values["RX"]   = ep.RX.Value;
        if (ep.RY.HasValue)       values["RY"]   = ep.RY.Value;
        if (ep.RZ.HasValue)       values["RZ"]   = ep.RZ.Value;
        _robot.pointRepo.EditPoint(ep.Name, values);
    }
}
