using System;
using System.IO;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Gcode;

namespace Controller.RobotControl.Commands;

/// <summary>
/// Running stored G-code files. <c>RunGcodeFile</c> wraps a file in a one-step synthetic program
/// and hands it to the executor, so stop/pause/resume/monitor/speed-override all work as for any
/// built program. <c>ValidateGcodeFile</c> parses without running and reports the first error.
/// Upload/list/delete are HTTP (<c>/gcode</c>, see HttpEndpoints).
/// </summary>
internal sealed class GcodeCommands
{
    private readonly RobotController _robot;
    private readonly ProgramExecutor? _executor;

    public GcodeCommands(RobotController robot, ProgramExecutor? executor)
    {
        _robot    = robot;
        _executor = executor;
    }

    public void Register(CommandDispatcher d)
    {
        d.Add("RunGcodeFile",      RunGcodeFile);
        d.Add("ValidateGcodeFile", ValidateGcodeFile);
    }

    private object? RunGcodeFile(CommandMessage msg)
    {
        var p    = CommandJson.LoadParams<BuiltProgramNameParams>(msg);
        var path = GcodeSteps.PathFor(p.Name);
        if (!File.Exists(path)) return new { ok = false, error = $"G-code file not found: {p.Name}" };
        if (_robot.GcodeStreamActive) return new { ok = false, error = "A G-code stream is active." };

        // A left-running jog would hold the motion queue until its watchdog times out.
        _robot.StopJog();
        var prog = new BuiltProgram
        {
            Id    = "gcode:" + p.Name,
            Name  = p.Name,
            Steps = new() { new ProgramStep { Id = Guid.NewGuid().ToString("N"), Type = StepType.GcodeProgram, GcodeFile = p.Name } },
        };
        _executor?.Start(prog);
        return new { ok = true };
    }

    private object? ValidateGcodeFile(CommandMessage msg)
    {
        var p    = CommandJson.LoadParams<BuiltProgramNameParams>(msg);
        var path = GcodeSteps.PathFor(p.Name);
        if (!File.Exists(path)) return new { ok = false, error = $"G-code file not found: {p.Name}" };

        string text;
        try { text = File.ReadAllText(path); }
        catch (Exception ex) { return new { ok = false, error = ex.Message }; }

        var interp = new GcodeInterpreter(_robot.Config.GcodeOptions());
        int lineNo = 0, moves = 0;
        foreach (var line in text.Replace("\r\n", "\n").Replace('\r', '\n').Split('\n'))
        {
            lineNo++;
            try
            {
                foreach (var op in interp.Feed(line))
                    if (op.Kind == GcodeOpKind.Move) moves++;
            }
            catch (GcodeException ex)
            {
                return new { ok = false, error = $"Line {lineNo}: {ex.Message}", line = lineNo, moves };
            }
        }
        return new { ok = true, lines = lineNo, moves };
    }
}
