using System;
using System.Collections.Generic;

namespace Controller.RobotControl.Gcode
{
    /// <summary>Where spindle/laser on-off (M3/M4/M5) is routed.</summary>
    public readonly record struct SpindleOutput(string Type, int Pin)
    {
        /// <summary>"none" (default) means M3/M4/M5 are parsed but do nothing physical.</summary>
        public bool Enabled => !string.Equals(Type, "none", StringComparison.OrdinalIgnoreCase)
                               && !string.IsNullOrWhiteSpace(Type);
    }

    /// <summary>
    /// Expands G-code text into executable <see cref="ProgramStep"/>s, mirroring the pattern in
    /// <c>Execution/CncSteps.cs</c> (CncStepGenerator). Moves become MoveL (absolute overrides in
    /// G90, offsets in G91), arcs a blended MoveL chain, dwell a Wait, spindle a SetOutput, pause a
    /// PauseProgram, home a RunHoming. A line the interpreter rejects is skipped with a log so one
    /// bad line doesn't abort the whole file (streaming reports the error instead — see
    /// GcodeStreamSession). Rapids use the configured rapid speed; feeds come from the F word.
    /// </summary>
    public static class GcodeToSteps
    {
        public static List<ProgramStep> Generate(string gcodeText, GcodeOptions options, SpindleOutput spindle)
        {
            var steps  = new List<ProgramStep>();
            var interp = new GcodeInterpreter(options);
            string NewId() => Guid.NewGuid().ToString("N");

            int lineNo = 0;
            foreach (var raw in SplitLines(gcodeText))
            {
                lineNo++;
                IReadOnlyList<GcodeOp> ops;
                try { ops = interp.Feed(raw); }
                catch (GcodeException ex)
                {
                    Console.WriteLine($"[Gcode] Skipped line {lineNo}: {ex.Message}  ({raw.Trim()})");
                    continue;
                }

                foreach (var op in ops)
                {
                    switch (op.Kind)
                    {
                        case GcodeOpKind.Move:
                            steps.Add(MoveStep(op, options, NewId()));
                            break;
                        case GcodeOpKind.Dwell:
                            steps.Add(new ProgramStep
                            {
                                Id = NewId(), Type = StepType.Wait,
                                WaitMode = "time", WaitMs = (int)Math.Round(op.DwellMs),
                            });
                            break;
                        case GcodeOpKind.Spindle:
                            if (spindle.Enabled)
                                steps.Add(new ProgramStep
                                {
                                    Id = NewId(), Type = StepType.SetOutput,
                                    OutputCard = spindle.Type, OutputNumber = spindle.Pin,
                                    OutputValue = op.SpindleOn,
                                });
                            break;
                        case GcodeOpKind.Pause:
                            steps.Add(new ProgramStep { Id = NewId(), Type = StepType.PauseProgram });
                            break;
                        case GcodeOpKind.Home:
                            steps.Add(new ProgramStep { Id = NewId(), Type = StepType.RunHoming });
                            break;
                        case GcodeOpKind.End:
                            return steps; // M2/M30 — stop expanding
                        case GcodeOpKind.SetPosition:
                            break;        // interpreter state only
                    }
                }
            }
            return steps;
        }

        // A single move → MoveL. Absolute moves set per-axis Overrides (unmentioned axes hold the
        // current position — safe for axes the file never touches); relative moves set Offsets.
        // Arc segments come through Relative=false with a blend radius so the executor rounds them
        // into one continuous path (see MotionSteps.TryDispatchBlendedRun).
        private static ProgramStep MoveStep(GcodeOp op, GcodeOptions options, string id)
        {
            double speed = op.Rapid ? options.RapidSpeedMmPerSec : (op.FeedMmPerSec ?? options.DefaultFeedMmPerSec);
            var step = new ProgramStep
            {
                Id = id,
                Type = StepType.MoveL,
                Speed = speed,
            };
            if (op.Relative)
            {
                step.OffsetX  = op.X;
                step.OffsetY  = op.Y;
                step.OffsetZ  = op.Z;
                step.OffsetRZ = op.A;
            }
            else
            {
                step.OverrideX  = op.X;
                step.OverrideY  = op.Y;
                step.OverrideZ  = op.Z;
                step.OverrideRZ = op.A;
                // Flattened arc segments round into one continuous path; a plain G1 is an exact stop.
                if (op.ArcSegment)
                {
                    step.Blend = true;
                    step.BlendRadius = Math.Max(0.05, options.ArcToleranceMm * 4);
                }
            }
            return step;
        }

        private static IEnumerable<string> SplitLines(string text)
        {
            if (string.IsNullOrEmpty(text)) yield break;
            foreach (var line in text.Replace("\r\n", "\n").Replace('\r', '\n').Split('\n'))
                yield return line;
        }
    }
}
