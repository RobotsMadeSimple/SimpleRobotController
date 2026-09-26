using System.Text.Json;

namespace Controller.RobotControl.Commands;

/// <summary>Stack (pick/place column) repository.</summary>
internal sealed class StackCommands
{
    private readonly RobotController _robot;

    public StackCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetStacks",   GetStacks);
        d.Add("SaveStack",   SaveStack);
        d.Add("DeleteStack", DeleteStack);
    }

    private object? GetStacks(CommandMessage msg)
    {
        var list = _robot.stackRepo.GetAll();
        var json = JsonSerializer.Serialize(list, CommandJson.PascalCase);
        return new { stacks = json };
    }

    private void SaveStack(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SaveStackParams>(msg);
        _robot.stackRepo.Upsert(new RobotStack
        {
            Id            = p.Id,
            Name          = p.Name,
            BasePointName = p.BasePointName,
            OffsetX       = p.OffsetX,
            OffsetY       = p.OffsetY,
            OffsetZ       = p.OffsetZ,
            MaxCount      = p.MaxCount,
        });
    }

    private void DeleteStack(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<StackIdParams>(msg);
        _robot.stackRepo.Delete(p.Id);
    }
}
