using System;
using System.Text.Json;
using Controller.RobotControl.Vision;
using OpenCvSharp;

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
        d.Add("RunVisionOnImage",    RunVisionOnImage);
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

    /// <summary>
    /// Runs a vision program against a supplied image rather than a live camera, and returns
    /// the same VisionResult a camera frame would have produced. Lets callers test an inspection
    /// (e.g. the checkers board grid) with a known image, with no camera attached. Takes either a
    /// saved <c>programId</c> or an inline <c>program</c>, plus a base64 <c>image</c>.
    /// </summary>
    private object? RunVisionOnImage(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<RunVisionOnImageParams>(msg);

        var program = p.Program
            ?? (string.IsNullOrEmpty(p.ProgramId) ? null : _robot.VisionRepo.Get(p.ProgramId));
        if (program == null)
            throw new InvalidOperationException(
                string.IsNullOrEmpty(p.ProgramId)
                    ? "RunVisionOnImage needs a 'program' or a 'programId'"
                    : $"No vision program with id '{p.ProgramId}'");

        if (string.IsNullOrWhiteSpace(p.Image))
            throw new InvalidOperationException("RunVisionOnImage needs a base64 'image'");

        // Optional runtime zone override: point every inspection at one zone for this run.
        if (!string.IsNullOrEmpty(p.ZoneId))
            foreach (var insp in program.AllInspections())
                insp.ZoneId = p.ZoneId;

        var bytes = Convert.FromBase64String(StripDataUrl(p.Image));
        using var src = Cv2.ImDecode(bytes, ImreadModes.Color);
        if (src.Empty())
            throw new InvalidOperationException("Could not decode 'image' as a PNG/JPEG");

        var result       = VisionProcessor.RunProgramOnce(program, src, out var annotatedJpeg);
        var json         = JsonSerializer.Serialize(result, CommandJson.CamelCase);
        string? annotated = p.IncludeAnnotated && annotatedJpeg != null
            ? Convert.ToBase64String(annotatedJpeg)
            : null;
        return new { result = json, annotated };
    }

    /// <summary>Drops a <c>data:image/...;base64,</c> prefix so a browser data URL works unchanged.</summary>
    private static string StripDataUrl(string image)
    {
        int comma = image.StartsWith("data:", StringComparison.OrdinalIgnoreCase)
            ? image.IndexOf(',')
            : -1;
        return comma >= 0 ? image[(comma + 1)..] : image;
    }
}
