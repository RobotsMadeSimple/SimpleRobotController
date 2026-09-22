using System.Text.Json;
using Controller.RobotControl.Vision;

namespace Controller.RobotControl.Commands;

/// <summary>Vision program repository, start/stop and results.</summary>
internal sealed class VisionCommands
{
    private readonly RobotController _robot;

    public VisionCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetVisionPrograms",   GetVisionPrograms);
        d.Add("SaveVisionProgram",   SaveVisionProgram);
        d.Add("DeleteVisionProgram", DeleteVisionProgram);
        d.Add("StartVision",         StartVision);
        d.Add("StopVision",          StopVision);
        d.Add("GetVisionResult",     GetVisionResult);
    }

    private object? GetVisionPrograms(CommandMessage msg)
    {
        var programs = _robot.VisionRepo.GetAll();
        var json = JsonSerializer.Serialize(programs, CommandJson.CamelCase);
        return new { programs = json, runningIds = _robot.VisionManager.GetRunningIds() };
    }

    private object? SaveVisionProgram(CommandMessage msg)
    {
        var prog = CommandJson.LoadParams<VisionProgram>(msg);
        if (string.IsNullOrEmpty(prog.Id))
            prog.Id = Guid.NewGuid().ToString("N")[..8];
        _robot.VisionRepo.Save(prog);
        _robot.VisionManager.OnProgramSaved(prog);
        return new { programId = prog.Id, lastUpdatedUnixMs = prog.LastUpdatedUnixMs };
    }

    private void DeleteVisionProgram(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<DeleteVisionProgramParams>(msg);
        _robot.VisionManager.StopProgram(p.Id);
        _robot.VisionRepo.Delete(p.Id);
    }

    private void StartVision(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<StartStopVisionParams>(msg);
        _robot.VisionManager.StartProgram(p.Id);
    }

    private void StopVision(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<StartStopVisionParams>(msg);
        _robot.VisionManager.StopProgram(p.Id);
    }

    private object? GetVisionResult(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<StartStopVisionParams>(msg);
        var proc = _robot.VisionManager.GetProcessor(p.Id);
        // Live result while running, else the last one captured at the end
        // of the most recent RunVision step for this program.
        var result = proc?.GetLatestResult() ?? _robot.GetProgramVisionResult(p.Id);
        var json   = result != null
            ? JsonSerializer.Serialize(result, CommandJson.CamelCase)
            : null;
        return new { result = json };
    }
}
