namespace Controller.RobotControl.Commands;

/// <summary>Tool repository and active-tool selection.</summary>
internal sealed class ToolCommands
{
    private readonly RobotController _robot;

    public ToolCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetTools",      GetTools);
        d.Add("CreateTool",    CreateTool);
        d.Add("EditTool",      EditTool);
        d.Add("DeleteTool",    DeleteTool);
        d.Add("SetActiveTool", SetActiveTool);
    }

    private object? GetTools(CommandMessage msg) => new { tools = _robot.toolRepo.toolsJson };

    private void CreateTool(CommandMessage msg)
    {
        var ep = CommandJson.LoadParams<EditToolParams>(msg);
        var v  = new Vector6(ep.X ?? 0, ep.Y ?? 0, ep.Z ?? 0,
                             ep.RX ?? 0, ep.RY ?? 0, ep.RZ ?? 0);
        _robot.toolRepo.SaveTool(ep.Name, v);
        if (!string.IsNullOrEmpty(ep.Description))
        {
            _robot.toolRepo.EditTool(ep.Name, new()
            {
                ["Description"] = ep.Description
            });
        }
    }

    private void EditTool(CommandMessage msg)
    {
        var ep     = CommandJson.LoadParams<EditToolParams>(msg);
        var values = new Dictionary<string, object?>();
        if (ep.NewName      != null) values["Name"]        = ep.NewName;
        if (ep.Description  != null) values["Description"] = ep.Description;
        if (ep.X.HasValue)           values["X"]           = ep.X.Value;
        if (ep.Y.HasValue)           values["Y"]           = ep.Y.Value;
        if (ep.Z.HasValue)           values["Z"]           = ep.Z.Value;
        if (ep.RX.HasValue)          values["RX"]          = ep.RX.Value;
        if (ep.RY.HasValue)          values["RY"]          = ep.RY.Value;
        if (ep.RZ.HasValue)          values["RZ"]          = ep.RZ.Value;
        _robot.toolRepo.EditTool(ep.Name, values);

        // Keep the active tool name in sync after a rename.
        if (ep.NewName != null)
            _robot.RenameActiveTool(ep.Name, ep.NewName);
    }

    private void DeleteTool(CommandMessage msg)
    {
        var tp = CommandJson.LoadParams<ToolNameParams>(msg);
        _robot.toolRepo.DeleteTool(tp.Name);
        // Clear the active tool if the deleted one was active.
        _robot.ForgetDeletedTool(tp.Name);
    }

    private void SetActiveTool(CommandMessage msg)
    {
        var tp = CommandJson.LoadParams<ToolNameParams>(msg);
        _robot.SelectTool(tp.Name);
    }
}
