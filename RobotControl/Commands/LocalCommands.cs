namespace Controller.RobotControl.Commands;

/// <summary>Local (user frame) repository and active-local selection.</summary>
internal sealed class LocalCommands
{
    private readonly RobotController _robot;

    public LocalCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetLocals",      GetLocals);
        d.Add("CreateLocal",    CreateLocal);
        d.Add("EditLocal",      EditLocal);
        d.Add("DeleteLocal",    DeleteLocal);
        d.Add("SetActiveLocal", SetActiveLocal);
    }

    private object? GetLocals(CommandMessage msg) => new { locals = _robot.localRepo.localsJson };

    private void CreateLocal(CommandMessage msg)
    {
        var ep = CommandJson.LoadParams<EditLocalParams>(msg);
        var v  = new Vector6(ep.X ?? 0, ep.Y ?? 0, ep.Z ?? 0,
                             ep.RX ?? 0, ep.RY ?? 0, ep.RZ ?? 0);
        _robot.localRepo.SaveLocal(ep.Name, v);
        if (!string.IsNullOrEmpty(ep.Description))
        {
            _robot.localRepo.EditLocal(ep.Name, new()
            {
                ["Description"] = ep.Description
            });
        }
    }

    private void EditLocal(CommandMessage msg)
    {
        var ep     = CommandJson.LoadParams<EditLocalParams>(msg);
        var values = new Dictionary<string, object?>();
        if (ep.NewName      != null) values["Name"]        = ep.NewName;
        if (ep.Description  != null) values["Description"] = ep.Description;
        if (ep.X.HasValue)           values["X"]           = ep.X.Value;
        if (ep.Y.HasValue)           values["Y"]           = ep.Y.Value;
        if (ep.Z.HasValue)           values["Z"]           = ep.Z.Value;
        if (ep.RX.HasValue)          values["RX"]          = ep.RX.Value;
        if (ep.RY.HasValue)          values["RY"]          = ep.RY.Value;
        if (ep.RZ.HasValue)          values["RZ"]          = ep.RZ.Value;
        _robot.localRepo.EditLocal(ep.Name, values);

        // Keep the active local name in sync after a rename.
        if (ep.NewName != null)
            _robot.RenameActiveLocal(ep.Name, ep.NewName);
    }

    private void DeleteLocal(CommandMessage msg)
    {
        var lp = CommandJson.LoadParams<LocalNameParams>(msg);
        _robot.localRepo.DeleteLocal(lp.Name);
        // Clear the active local if the deleted one was active.
        _robot.ForgetDeletedLocal(lp.Name);
    }

    private void SetActiveLocal(CommandMessage msg)
    {
        var lp = CommandJson.LoadParams<LocalNameParams>(msg);
        _robot.SelectLocal(lp.Name);
    }
}
