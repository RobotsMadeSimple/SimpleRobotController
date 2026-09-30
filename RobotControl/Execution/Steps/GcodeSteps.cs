using System.IO;
using Controller.RobotControl.Gcode;

namespace Controller.RobotControl.Execution
{
    /// <summary>The GcodeProgram step: expands stored or inline G-code into moves at run time.</summary>
    internal static class GcodeSteps
    {
        /// <summary>Data-directory subfolder holding uploaded G-code files (relative to cwd).</summary>
        public const string FileDir = "gcode";

        /// <summary>Full path to a stored G-code file, path components stripped for safety.</summary>
        public static string PathFor(string name) => Path.Combine(FileDir, Path.GetFileName(name));

        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.GcodeProgram] = new GcodeProgramStep();
        }

        private sealed class GcodeProgramStep : IStepHandler
        {
            public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
            {
                string? text = step.GcodeText;
                if (!string.IsNullOrWhiteSpace(step.GcodeFile))
                {
                    var path = PathFor(step.GcodeFile!);
                    if (!File.Exists(path))
                        return ctx.Finish(ProgramStatus.Error, $"G-code file not found: {step.GcodeFile}");
                    try { text = File.ReadAllText(path); }
                    catch (System.Exception ex) { return ctx.Finish(ProgramStatus.Error, $"Could not read {step.GcodeFile}: {ex.Message}"); }
                }
                if (string.IsNullOrWhiteSpace(text)) return StepOutcome.Advance;

                var cfg   = ctx.Controller.Config;
                var inner = GcodeToSteps.Generate(text!, cfg.GcodeOptions(), cfg.GcodeSpindle());
                if (inner.Count == 0) return StepOutcome.Advance;

                // Inner steps run in their own (non-loop) frame, like a CNC block.
                ctx.Frames.Push(StepListFrame.Cnc(inner));
                return StepOutcome.Advance;
            }
        }
    }
}
