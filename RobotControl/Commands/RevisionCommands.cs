using System.Text.Json;

namespace Controller.RobotControl.Commands;

/// <summary>
/// Saved-program revision history: list, fetch, and restore prior snapshots captured
/// automatically by <see cref="Persistence.BuiltProgramRepository.Save"/>. See
/// docs/expressions-and-variables.md section 6.
/// </summary>
internal sealed class RevisionCommands
{
    private readonly RobotController _robot;

    public RevisionCommands(RobotController robot)
    {
        _robot = robot;
    }

    public void Register(CommandDispatcher d)
    {
        d.Add("GetBuiltProgramRevisions",    GetBuiltProgramRevisions);
        d.Add("GetBuiltProgramRevision",     GetBuiltProgramRevision);
        d.Add("RestoreBuiltProgramRevision", RestoreBuiltProgramRevision);
    }

    private object? GetBuiltProgramRevisions(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<BuiltProgramNameParams>(msg);
        if (_robot.builtProgramRepo.Get(p.Name) == null)
            return new { ok = false, error = "unknownProgram" };

        var revisions = _robot.builtProgramRepo.ListRevisions(p.Name);
        return new { revisions };
    }

    private object? GetBuiltProgramRevision(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<BuiltProgramRevisionParams>(msg);
        if (_robot.builtProgramRepo.Get(p.Name) == null)
            return new { ok = false, error = "unknownProgram" };

        var revision = _robot.builtProgramRepo.GetRevision(p.Name, p.Id);
        if (revision == null)
            return new { ok = false, error = "unknownRevision" };

        var json = JsonSerializer.Serialize(revision, CommandJson.CamelCaseWithEnums);
        return new { program = json };
    }

    private object? RestoreBuiltProgramRevision(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<BuiltProgramRevisionParams>(msg);
        if (_robot.builtProgramRepo.Get(p.Name) == null)
            return new { ok = false, error = "unknownProgram" };

        var restored = _robot.builtProgramRepo.RestoreRevision(p.Name, p.Id);
        if (restored == null)
            return new { ok = false, error = "unknownRevision" };

        var json = JsonSerializer.Serialize(restored, CommandJson.CamelCaseWithEnums);
        return new { program = json };
    }
}
