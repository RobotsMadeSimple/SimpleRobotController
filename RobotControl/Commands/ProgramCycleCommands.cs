namespace Controller.RobotControl.Commands;

/// <summary>
/// Program cycle: the program list shown in the app, their status/logs/images,
/// and the Start/Stop/Reset/Abort actions for both built and external programs.
/// </summary>
internal sealed class ProgramCycleCommands
{
    private readonly RobotController _robot;
    private readonly ProgramCycleManager _programs;
    private readonly ProgramExecutor? _executor;

    public ProgramCycleCommands(RobotController robot, ProgramCycleManager programs, ProgramExecutor? executor)
    {
        _robot    = robot;
        _programs = programs;
        _executor = executor;
    }

    public void Register(CommandDispatcher d)
    {
        d.Add("SetAvailablePrograms", SetAvailablePrograms);
        d.Add("SetProgramStatus",     SetProgramStatus);
        d.Add("GetProgramImages",     GetProgramImages);
        d.Add("GetProgramLogs",       GetProgramLogs);
        d.Add("StartProgram",         StartProgram);
        d.Add("StopProgram",          StopProgram);
        d.Add("ResetProgram",         msg => ResetOrAbort(msg, "Reset"));
        d.Add("AbortProgram",         msg => ResetOrAbort(msg, "Abort"));
        d.Add("ClearProgramActions",  ClearProgramActions);
    }

    private void SetAvailablePrograms(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SetAvailableProgramsParams>(msg);
        _programs.SetAvailablePrograms(p.Programs);
    }

    private void SetProgramStatus(CommandMessage msg)
    {
        var update = CommandJson.LoadParams<ProgramCycleUpdate>(msg);
        _programs.ApplyStatusUpdate(update);
    }

    private object? GetProgramImages(CommandMessage msg)
    {
        // Merge live in-memory images (Python/external) with persisted images
        // for built programs so idle built programs still show their image.
        var merged = _programs.GetAllImages();
        foreach (var kv in _robot.builtProgramRepo.GetAllImages())
            if (kv.Value != null) merged[kv.Key] = kv.Value;
        return new { images = merged };
    }

    private object? GetProgramLogs(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<GetProgramLogsParams>(msg);
        var (total, start, logs) = _programs.GetProgramLogs(p.ProgramName, p.Start, p.End);
        return new
        {
            programName = p.ProgramName,
            totalCount  = total,
            start,
            logs
        };
    }

    private void StartProgram(CommandMessage msg)
    {
        var p     = CommandJson.LoadParams<ProgramActionParams>(msg);
        var built = _robot.builtProgramRepo.Get(p.ProgramName);
        if (built != null)
        {
            if (_executor?.IsPaused == true && _executor.CurrentProgramName == p.ProgramName)
                _executor.Resume();
            else
            {
                _robot.DisplaceRunningBuiltProgram(p.ProgramName);
                _executor?.Start(built);
            }
        }
        else
            _programs.SetFlag(p.ProgramName, "Start");
    }

    private void StopProgram(CommandMessage msg)
    {
        var p     = CommandJson.LoadParams<ProgramActionParams>(msg);
        var built = _robot.builtProgramRepo.Get(p.ProgramName);
        if (built != null)
        {
            if (_executor?.CurrentProgramName == p.ProgramName)
                _executor.Stop();
        }
        else
            _programs.SetFlag(p.ProgramName, "Stop");
    }

    // ResetProgram and AbortProgram are identical for built programs (reset the
    // executor and the cycle entry); for external programs they raise their own flag.
    private void ResetOrAbort(CommandMessage msg, string externalFlag)
    {
        var p     = CommandJson.LoadParams<ProgramActionParams>(msg);
        var built = _robot.builtProgramRepo.Get(p.ProgramName);
        if (built != null)
        {
            if (_executor?.CurrentProgramName == p.ProgramName)
                _executor.Reset();
            _programs.ResetToReady(p.ProgramName,
                ProgramExecutor.CountSteps(built.Steps));
        }
        else
        {
            _programs.SetFlag(p.ProgramName, externalFlag);
        }
    }

    private void ClearProgramActions(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ProgramActionParams>(msg);
        _programs.ClearActions(p.ProgramName);
    }
}
