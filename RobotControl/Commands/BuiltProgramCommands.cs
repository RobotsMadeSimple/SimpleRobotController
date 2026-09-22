using System.Text.Json;

namespace Controller.RobotControl.Commands;

/// <summary>
/// Built (in-app authored) programs: the repository, foreground execution,
/// background programs, live variables/images and the active CNC toolpath.
/// </summary>
internal sealed class BuiltProgramCommands
{
    private readonly RobotController _robot;
    private readonly ProgramCycleManager _programs;
    private readonly ProgramExecutor? _executor;
    private readonly BackgroundProgramManager _background;

    public BuiltProgramCommands(RobotController robot, ProgramCycleManager programs,
        ProgramExecutor? executor, BackgroundProgramManager background)
    {
        _robot      = robot;
        _programs   = programs;
        _executor   = executor;
        _background = background;
    }

    public void Register(CommandDispatcher d)
    {
        d.Add("GetBuiltPrograms",        GetBuiltPrograms);
        d.Add("SaveBuiltProgram",        SaveBuiltProgram);
        d.Add("DeleteBuiltProgram",      DeleteBuiltProgram);
        d.Add("SaveBuiltProgramImage",   SaveBuiltProgramImage);
        d.Add("ExecuteBuiltProgram",     ExecuteBuiltProgram);
        d.Add("StopBuiltProgram",        _ => _executor?.Stop());
        d.Add("StartBackgroundProgram",  StartBackgroundProgram);
        d.Add("StopBackgroundProgram",   StopBackgroundProgram);
        d.Add("GetProgramVariables",     GetProgramVariables);
        // Not "GetProgramImage" — GetProgramImages is the program *thumbnail*
        // list, an unrelated thing, and the two would be a singular/plural apart.
        d.Add("GetProgramVariableImage", GetProgramVariableImage);
        d.Add("GetCncToolpath",          GetCncToolpath);
    }

    private object? GetBuiltPrograms(CommandMessage msg)
    {
        var list = _robot.builtProgramRepo.GetAll();
        var json = JsonSerializer.Serialize(list, CommandJson.CamelCaseWithEnums);
        return new { programs = json };
    }

    private void SaveBuiltProgram(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SaveBuiltProgramParams>(msg);
        _robot.builtProgramRepo.Save(new BuiltProgram
        {
            Id                  = p.Id,
            Name                = p.Name,
            Description         = p.Description,
            Steps               = p.Steps,
            Variables           = p.Variables,
            IsRoutine           = p.IsRoutine,
            IsBackground        = p.IsBackground,
            KillBackgroundOnStop = p.KillBackgroundOnStop,
        });
    }

    private void DeleteBuiltProgram(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<BuiltProgramNameParams>(msg);
        _robot.builtProgramRepo.Delete(p.Name);
        _programs.RemoveProgram(p.Name);
    }

    private void SaveBuiltProgramImage(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SaveBuiltProgramImageParams>(msg);
        var bytes = Convert.FromBase64String(p.Image);
        _robot.builtProgramRepo.SaveImage(p.Name, bytes);
    }

    private void ExecuteBuiltProgram(CommandMessage msg)
    {
        var p    = CommandJson.LoadParams<BuiltProgramNameParams>(msg);
        var prog = _robot.builtProgramRepo.Get(p.Name);
        if (prog != null)
        {
            _robot.DisplaceRunningBuiltProgram(p.Name);
            var imgBytes = _robot.builtProgramRepo.GetImage(p.Name);
            _executor?.Start(prog, imgBytes != null ? Convert.ToBase64String(imgBytes) : null);
        }
    }

    private void StartBackgroundProgram(CommandMessage msg)
    {
        var p     = CommandJson.LoadParams<ProgramActionParams>(msg);
        var built = _robot.builtProgramRepo.Get(p.ProgramName);
        if (built != null && built.IsBackground)
            _background.TryStart(built);
    }

    private void StopBackgroundProgram(CommandMessage msg)
    {
        var p     = CommandJson.LoadParams<ProgramActionParams>(msg);
        var built = _robot.builtProgramRepo.Get(p.ProgramName);
        if (built != null) _background.Stop(built.Id);
    }

    private bool IsForeground(string name) =>
        _executor?.CurrentProgramName?.Equals(name, StringComparison.OrdinalIgnoreCase) == true;

    private object? GetProgramVariables(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<BuiltProgramNameParams>(msg);
        bool foreground = IsForeground(p.Name);
        var vars = foreground
            ? _executor!.GetDisplayVariables()
            : _background.GetDisplayVariables(p.Name);
        // Images are listed by name and revision only — see GetDisplayImages.
        // The monitor fetches the bytes with GetProgramImage when a revision
        // moves, which keeps this poll the same size whether or not the
        // program holds a camera frame.
        var images = foreground
            ? _executor!.GetDisplayImages()
            : _background.GetDisplayImages(p.Name);
        return new
        {
            variables = vars.Select(v => new { name = v.Name, value = v.Value, isBoolean = v.IsBoolean }).ToList(),
            images    = images.Select(i => new { name = i.Name, revision = i.Revision }).ToList()
        };
    }

    private object? GetProgramVariableImage(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ProgramImageParams>(msg);
        var data = IsForeground(p.Name)
            ? _executor!.GetDisplayImage(p.Variable)
            : _background.GetDisplayImage(p.Name, p.Variable);
        // Empty rather than an error when there is nothing to send: a monitor
        // asking about a program that has just stopped is ordinary, not a fault.
        return new { name = p.Name, variable = p.Variable, image = data };
    }

    private object? GetCncToolpath(CommandMessage msg)
    {
        // Resolved toolpath of the CNC block currently executing —
        // anchor and variables applied. Null when no block is active.
        var tp = _robot.ActiveCncToolpath;
        return new
        {
            toolpath = tp == null ? null : new
            {
                programName = tp.ProgramName,
                paths       = tp.Paths,
                holes       = tp.Holes.Select(h => new { x = h.X, y = h.Y }).ToList(),
            },
        };
    }
}
