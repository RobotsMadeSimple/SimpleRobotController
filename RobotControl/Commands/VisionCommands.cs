using System;
using System.Text.Json;
using Controller.RobotControl.Vision;
using Controller.RobotControl.Vision.Tools;
using OpenCvSharp;

namespace Controller.RobotControl.Commands;

/// <summary>Vision program repository, start/stop, results, references and teaching.</summary>
internal sealed class VisionCommands
{
    private readonly RobotController _robot;

    public VisionCommands(RobotController robot) => _robot = robot;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetVisionPrograms",      GetVisionPrograms);
        d.Add("SaveVisionProgram",      SaveVisionProgram);
        d.Add("DeleteVisionProgram",    DeleteVisionProgram);
        d.Add("StartVision",            StartVision);
        d.Add("StopVision",             StopVision);
        d.Add("GetVisionResult",        GetVisionResult);
        d.Add("RunVisionOnImage",       RunVisionOnImage);
        d.Add("SetVisionToolReference", SetVisionToolReference);
        d.Add("TeachVisionTool",        TeachVisionTool);
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

    /// <summary>Live result while running, else the last one captured by a RunVision step / RunVisionOnImage.</summary>
    private VisionResult? LatestResult(string programId) =>
        _robot.VisionManager.GetProcessor(programId)?.GetLatestResult() ?? _robot.GetProgramVisionResult(programId);

    private object? GetVisionResult(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<StartStopVisionParams>(msg);
        var result = LatestResult(p.Id);
        return new { result = result != null ? JsonSerializer.Serialize(result, CommandJson.CamelCase) : null };
    }

    /// <summary>
    /// Runs a vision program against a supplied image rather than a live camera, and returns
    /// the same VisionResult a camera frame would have produced. Takes either a saved
    /// <c>programId</c> or an inline <c>program</c>, plus a base64 <c>image</c>. A saved
    /// program whose reference is captured on this run is saved with it.
    /// </summary>
    private object? RunVisionOnImage(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<RunVisionOnImageParams>(msg);
        bool saved = p.Program == null;
        var program = p.Program
            ?? (string.IsNullOrEmpty(p.ProgramId) ? null : _robot.VisionRepo.Get(p.ProgramId));
        if (program == null)
            throw new InvalidOperationException(
                string.IsNullOrEmpty(p.ProgramId)
                    ? "RunVisionOnImage needs a 'program' or a 'programId'"
                    : $"No vision program with id '{p.ProgramId}'");

        using var src = DecodeImage(p.Image, "RunVisionOnImage");
        var result = VisionProcessor.RunProgramOnce(program, src, out var annotatedJpeg, out bool captured);
        if (saved)
        {
            _robot.SetProgramVisionResult(program.Id, result);
            if (captured)
            {
                _robot.VisionRepo.Save(program);
                _robot.VisionManager.OnProgramSaved(program);
            }
        }
        var json = JsonSerializer.Serialize(result, CommandJson.CamelCase);
        string? annotated = p.IncludeAnnotated ? Convert.ToBase64String(annotatedJpeg) : null;
        return new { result = json, annotated };
    }

    /// <summary>
    /// A pose tool's reference := the pose it found in the program's latest result (or cleared
    /// with <c>clear</c>). Dependent tools then follow the fixture relative to that pose.
    /// </summary>
    private object? SetVisionToolReference(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetVisionToolReferenceParams>(msg);
        var program = _robot.VisionRepo.Get(p.ProgramId) ?? throw new InvalidOperationException($"No vision program with id '{p.ProgramId}'");
        var tool = program.Tool(p.ToolId) ?? throw new InvalidOperationException($"No tool '{p.ToolId}' in vision program '{program.Name}'");
        if (ToolRegistry.Get(tool.Type) is not { MakesPose: true })
            throw new InvalidOperationException($"Tool '{tool.Name}' ({tool.Type}) does not output a pose, so it has no reference");
        if (p.Clear)
            tool.Reference = null;
        else
        {
            var tr = LatestResult(program.Id)?.Tool(tool.Id)
                ?? throw new InvalidOperationException($"No result for tool '{tool.Name}' yet — start the program or run it on an image first");
            if (tr.Pose == null)
                throw new InvalidOperationException($"Tool '{tool.Name}' found nothing in the latest result ({tr.Message ?? tr.Status})");
            tool.Reference = tr.Pose.Clone();
        }
        _robot.VisionRepo.Save(program);
        _robot.VisionManager.OnProgramSaved(program);
        return new
        {
            programId = program.Id, toolId = tool.Id, lastUpdatedUnixMs = program.LastUpdatedUnixMs,
            reference = tool.Reference == null ? null : JsonSerializer.Serialize(tool.Reference, CommandJson.CamelCase),
        };
    }

    /// <summary>
    /// Teaches a patternMatch / contourMatch tool: its template is cut from a frame (the given
    /// <c>image</c>, else the program's latest camera frame) inside the tool's region, after
    /// the earlier tools have run so a fixtured region is where its fixture puts it.
    /// </summary>
    private object? TeachVisionTool(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<TeachVisionToolParams>(msg);
        var program = _robot.VisionRepo.Get(p.ProgramId) ?? throw new InvalidOperationException($"No vision program with id '{p.ProgramId}'");
        Mat src;
        if (!string.IsNullOrWhiteSpace(p.Image)) src = DecodeImage(p.Image, "TeachVisionTool");
        else
        {
            var jpeg = _robot.VisionManager.LatestFrame(program.Id)
                ?? throw new InvalidOperationException($"No camera frame for vision program '{program.Name}' — pass an 'image' or start its camera");
            src = Cv2.ImDecode(jpeg, ImreadModes.Color);
        }
        string summary;
        using (src)
        using (var runtime = new VisionRuntime())
            summary = VisionEngine.Teach(program, p.ToolId, src, runtime);
        _robot.VisionRepo.Save(program);
        _robot.VisionManager.OnProgramSaved(program);
        return new { programId = program.Id, toolId = p.ToolId, taught = summary, lastUpdatedUnixMs = program.LastUpdatedUnixMs };
    }

    private static Mat DecodeImage(string? image, string command)
    {
        if (string.IsNullOrWhiteSpace(image))
            throw new InvalidOperationException($"{command} needs a base64 'image'");
        var bytes = Convert.FromBase64String(StripDataUrl(image));
        var src = Cv2.ImDecode(bytes, ImreadModes.Color);
        if (src.Empty())
        {
            src.Dispose();
            throw new InvalidOperationException("Could not decode 'image' as a PNG/JPEG");
        }
        return src;
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
