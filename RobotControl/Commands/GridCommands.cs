using System.Text.Json;

namespace Controller.RobotControl.Commands;

/// <summary>Grid (pallet pattern) repository.</summary>
internal sealed class GridCommands
{
    private readonly RobotController _robot;

    public GridCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetGrids",   GetGrids);
        d.Add("SaveGrid",   SaveGrid);
        d.Add("DeleteGrid", DeleteGrid);
    }

    private object? GetGrids(CommandMessage msg)
    {
        var list = _robot.gridRepo.GetAll();
        var json = JsonSerializer.Serialize(list, CommandJson.PascalCase);
        return new { grids = json };
    }

    private void SaveGrid(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SaveGridParams>(msg);
        _robot.gridRepo.Upsert(new Grid
        {
            Id            = p.Id,
            Name          = p.Name,
            BasePointName = p.BasePointName,
            RowOffsetX    = p.RowOffsetX,
            RowOffsetY    = p.RowOffsetY,
            RowOffsetZ    = p.RowOffsetZ,
            ColOffsetX    = p.ColOffsetX,
            ColOffsetY    = p.ColOffsetY,
            ColOffsetZ    = p.ColOffsetZ,
            RowCount      = p.RowCount,
            ColCount      = p.ColCount,
            Rotation      = p.Rotation,
        });
    }

    private void DeleteGrid(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<GridIdParams>(msg);
        _robot.gridRepo.Delete(p.Id);
    }
}
