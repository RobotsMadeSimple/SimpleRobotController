using Controller.RobotControl.Persistence;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Everything the executor tells <see cref="ProgramCycleManager"/> about a run: step
    /// started/completed progress, the lifecycle transitions (starting, resuming, stopped,
    /// paused, finished) and the human-readable step descriptions they carry.
    /// </summary>
    /// <remarks>
    /// Owns the overall step counter. A completed step only counts while no loop frame is on
    /// the stack (<see cref="FrameStack.LoopDepth"/> is 0), so a loop body adds nothing
    /// however often it runs. Not thread-safe; used under the owning executor's lock only.
    /// </remarks>
    internal sealed class ProgressReporter
    {
        private readonly ProgramCycleManager _programManager;
        private readonly FrameStack          _frames;
        private readonly VariableScope       _vars;

        public ProgressReporter(ProgramCycleManager programManager, FrameStack frames, VariableScope vars)
        {
            _programManager = programManager;
            _frames         = frames;
            _vars           = vars;
        }

        /// <summary>Name of the program being run; set by the executor on Start.</summary>
        public string ProgramName { get; set; } = "";

        /// <summary>Completed top-level steps so far.</summary>
        public int GlobalStepIndex { get; private set; }

        /// <summary>Description of the step most recently started (background status list).</summary>
        public string CurrentStepDescription { get; private set; } = "";

        public void Reset()
        {
            GlobalStepIndex        = 0;
            CurrentStepDescription = "";
        }

        // ── Step progress ─────────────────────────────────────────────────────

        /// <summary>Emits a "step in progress" update — description shown but count not yet incremented.</summary>
        public void StepStarted(ProgramStep step)
        {
            var isMove = step.Type == StepType.MoveL || step.Type == StepType.MoveJ
                      || step.Type == StepType.JumpL || step.Type == StepType.JumpJ;

            var ev = _vars.Eval;
            double? Off(string key, double? raw) => isMove ? ev.OptionalField(step, key, raw) : null;

            var desc = !string.IsNullOrEmpty(step.StatusMessage) ? step.StatusMessage : StepDescription(step);
            CurrentStepDescription = desc;

            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName          = ProgramName,
                ProgramStatus        = ProgramStatus.Running,
                CurrentStepNumber    = GlobalStepIndex,
                StepDescription      = desc,
                WarningDescription   = string.IsNullOrEmpty(step.StatusWarning) ? null : step.StatusWarning,
                ErrorDescription     = string.IsNullOrEmpty(step.StatusError)   ? null : step.StatusError,
                CurrentPointName     = isMove ? (step.PointName ?? "") : null,
                CurrentOffsetX       = Off("offsetX",       step.OffsetX),
                CurrentOffsetY       = Off("offsetY",       step.OffsetY),
                CurrentOffsetZ       = Off("offsetZ",       step.OffsetZ),
                CurrentOffsetRX      = Off("offsetRX",      step.OffsetRX),
                CurrentOffsetRY      = Off("offsetRY",      step.OffsetRY),
                CurrentOffsetRZ      = Off("offsetRZ",      step.OffsetRZ),
                CurrentToolOffsetX   = Off("toolOffsetX",   step.ToolOffsetX),
                CurrentToolOffsetY   = Off("toolOffsetY",   step.ToolOffsetY),
                CurrentToolOffsetZ   = Off("toolOffsetZ",   step.ToolOffsetZ),
                CurrentToolOffsetRX  = Off("toolOffsetRX",  step.ToolOffsetRX),
                CurrentToolOffsetRY  = Off("toolOffsetRY",  step.ToolOffsetRY),
                CurrentToolOffsetRZ  = Off("toolOffsetRZ",  step.ToolOffsetRZ),
            });
        }

        /// <summary>
        /// Increments the completed step count and emits the updated progress. A StatusUpdate
        /// step's completion is its message: interpolated, with its warning/error text.
        /// </summary>
        public void StepCompleted(ProgramStep step)
        {
            if (step.Type == StepType.StatusUpdate)
            {
                StatusUpdateCompleted(step);
                return;
            }

            if (Diag.Enabled) Diag.StepDone(step.Type);
            if (_frames.LoopDepth == 0) GlobalStepIndex++;
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName        = ProgramName,
                ProgramStatus      = ProgramStatus.Running,
                CurrentStepNumber  = GlobalStepIndex,
                StepDescription    = !string.IsNullOrEmpty(step.StatusMessage) ? step.StatusMessage : StepDescription(step),
                WarningDescription = string.IsNullOrEmpty(step.StatusWarning) ? null : step.StatusWarning,
                ErrorDescription   = string.IsNullOrEmpty(step.StatusError)   ? null : step.StatusError,
                ShouldLog          = true,
            });
        }

        private void StatusUpdateCompleted(ProgramStep step)
        {
            if (_frames.LoopDepth == 0) GlobalStepIndex++;
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName        = ProgramName,
                ProgramStatus      = ProgramStatus.Running,
                CurrentStepNumber  = GlobalStepIndex,
                StepDescription    = !string.IsNullOrEmpty(step.StatusMessage)
                    ? _vars.Interpolate(step.StatusMessage)
                    : StepDescription(step),
                WarningDescription = !string.IsNullOrEmpty(step.StatusWarning)
                    ? _vars.Interpolate(step.StatusWarning)
                    : null,
                ErrorDescription   = !string.IsNullOrEmpty(step.StatusError)
                    ? _vars.Interpolate(step.StatusError)
                    : null,
                ShouldLog          = true,
            });
        }

        /// <summary>A step a background program skips (motion/tool/homing) — logged, then completed.</summary>
        public void SkippedInBackground(ProgramStep step) =>
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName     = ProgramName,
                StepDescription = $"[Skipped — background] {step.Type}",
                ShouldLog       = true,
            });

        /// <summary>A Running update with a free-form description and the current step number.</summary>
        public void Announce(string description) =>
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName       = ProgramName,
                ProgramStatus     = ProgramStatus.Running,
                CurrentStepNumber = GlobalStepIndex,
                StepDescription   = description,
            });

        // ── Lifecycle ─────────────────────────────────────────────────────────

        /// <summary>Registers the program with the cycle manager and reports it Running at step 0.</summary>
        public void Starting(BuiltProgram program, string? imageBase64, int totalSteps)
        {
            // Register program in the cycle manager so the monitor tab can see it
            _programManager.SetAvailablePrograms(new()
            {
                new() { Name = program.Name, Description = program.Description, Image = imageBase64 }
            });

            // Clear any terminal state so the incoming Running update is not blocked by the guard
            _programManager.ResetToReady(program.Name, totalSteps);
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName       = program.Name,
                ProgramStatus     = ProgramStatus.Running,
                CurrentStepNumber = 0,
                MaxStepCount      = totalSteps,
                StepDescription   = "Starting…",
            });
        }

        public void Resuming()
        {
            // The status guard blocks Running over a terminal Stopped, so the
            // resume transition goes through its dedicated path.
            _programManager.ResumeToRunning(ProgramName);
            Announce("Resuming…");
        }

        /// <summary>A user Stop that pauses the main program.</summary>
        public void StoppedByUser() =>
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName       = ProgramName,
                ProgramStatus     = ProgramStatus.Stopped,
                CurrentStepNumber = GlobalStepIndex,
                StepDescription   = "Stopped — Continue resumes from the current step",
            });

        /// <summary>A PauseProgram step: Stopped, with the move display cleared.</summary>
        public void Paused() =>
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName         = ProgramName,
                ProgramStatus       = ProgramStatus.Stopped,
                CurrentStepNumber   = GlobalStepIndex,
                StepDescription     = "Paused — press Continue to resume",
                CurrentPointName    = "",
                CurrentOffsetX      = null, CurrentOffsetY   = null, CurrentOffsetZ   = null,
                CurrentOffsetRX     = null, CurrentOffsetRY  = null, CurrentOffsetRZ  = null,
                CurrentToolOffsetX  = null, CurrentToolOffsetY  = null, CurrentToolOffsetZ  = null,
                CurrentToolOffsetRX = null, CurrentToolOffsetRY = null, CurrentToolOffsetRZ = null,
            });

        /// <summary>The run's terminal status, with the move display cleared.</summary>
        public void Finished(ProgramStatus status, string description, int finalStepIndex) =>
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName          = ProgramName,
                ProgramStatus        = status,
                CurrentStepNumber    = finalStepIndex,
                StepDescription      = description,
                ErrorDescription     = status == ProgramStatus.Error ? description : null,
                CurrentPointName  = "",
                CurrentOffsetX   = null, CurrentOffsetY  = null, CurrentOffsetZ  = null,
                CurrentOffsetRX  = null, CurrentOffsetRY = null, CurrentOffsetRZ = null,
                CurrentToolOffsetX  = null, CurrentToolOffsetY  = null, CurrentToolOffsetZ  = null,
                CurrentToolOffsetRX = null, CurrentToolOffsetRY = null, CurrentToolOffsetRZ = null,
            });

        // ── Descriptions ──────────────────────────────────────────────────────

        /// <summary>
        /// Label for a move's destination, in the same precedence order the executor resolves it.
        /// Static, so variable-backed targets show the variable reference rather than a runtime value.
        /// </summary>
        private static string MoveTargetLabel(ProgramStep step) =>
              step.GridPoint  != null                  ? "grid point"
            : step.StackPoint != null                  ? "stack point"
            : !string.IsNullOrEmpty(step.VarPointName) ? $"${step.VarPointName}[{step.VarPointIndex ?? "0"}]"
            : !string.IsNullOrEmpty(step.PointNameExpr) ? step.PointNameExpr
            : !string.IsNullOrEmpty(step.PointName)    ? step.PointName
            : "current position";

        public static string StepDescription(ProgramStep step)
        {
            var type = step.Type switch
            {
                StepType.MoveL        => $"MoveL → {MoveTargetLabel(step)}",
                StepType.MoveJ        => $"MoveJ → {MoveTargetLabel(step)}",
                StepType.JumpL        => $"JumpL → {MoveTargetLabel(step)}",
                StepType.JumpJ        => $"JumpJ → {MoveTargetLabel(step)}",
                StepType.SetOutput    => BuildSetOutputDescription(step),
                StepType.Wait         => $"Wait {step.WaitMs} ms",
                StepType.Loop         => $"Loop ×{(step.LoopCount == 0 ? "∞" : step.LoopCount)}",
                StepType.StatusUpdate => step.StatusMessage ?? "Status update",
                StepType.CallRoutine  => $"Routine → {step.RoutineName}",
                StepType.SetSpeedL    => $"Set Linear Speed → {step.Speed} mm/s",
                StepType.SetSpeedJ    => $"Set Joint Speed → {step.Speed} mm/s",
                StepType.SetVariable  => $"${step.VariableName} = {step.VariableExpr}",
                StepType.PauseProgram => "Pause Program",
                StepType.Label        => $"Label: {step.LabelName ?? step.LabelId}",
                StepType.GoToLabel    => $"Go To: {step.LabelName ?? step.LabelId}",
                StepType.IfCondition  => step.Condition != null
                    ? $"If [{step.Condition.Combinator} · {step.Condition.Items.Count} condition(s)]"
                    : "If Condition",
                StepType.SetTool      => $"Set Tool → {(string.IsNullOrEmpty(step.ToolName) ? "None" : step.ToolName)}",
                StepType.SetLocal     => $"Set Local → {(string.IsNullOrEmpty(step.LocalName) ? "None" : step.LocalName)}",
                StepType.ClearLocal   => "Clear Local",
                StepType.RunHoming     => "Run Homing",
                StepType.AuxMove       => !string.IsNullOrEmpty(step.AuxUnit) && step.AuxDistance.HasValue
                    ? $"Aux Move · axis {step.AuxAxisIndex} · {step.AuxDistance} {step.AuxUnit}"
                    : $"Aux Move · axis {step.AuxAxisIndex} · {step.AuxSteps} steps",
                StepType.AuxContinuous => $"Aux Continuous · axis {step.AuxAxisIndex}",
                StepType.AuxStop       => $"Aux Stop · device {step.AuxDeviceId ?? "default"}",
                StepType.AuxEnable     => $"Aux Motors {(step.AuxEnable == true ? "ON" : "OFF")}",
                StepType.RunVision          => $"Vision → {step.VisionProgramName ?? step.VisionProgramId ?? "?"}",
                StepType.StartBackground    => $"Start Background → {step.BackgroundProgramName ?? "?"}",
                StepType.StopBackground     => $"Stop Background → {step.BackgroundProgramName ?? "?"}",
                StepType.WaitForBackground  => $"Wait for Background → {step.BackgroundProgramName ?? "?"}",
                StepType.StopwatchControl   => $"Stopwatch {step.StopwatchAction ?? "?"} → ${step.StopwatchVariableName ?? "?"}",
                StepType.SaveImage          => $"Save Image → {step.SaveImagePath ?? "?"}",
                _                           => step.Type.ToString(),
            };
            return string.IsNullOrEmpty(step.Name) ? type : $"{step.Name}  ({type})";
        }

        private static string BuildSetOutputDescription(ProgramStep step)
        {
            var state = step.OutputValue == true ? "ON" : "OFF";
            var base_ = step.OutputCard switch {
                "relay" => $"Relay {step.OutputNumber} → {state}",
                "nano"  => $"Nano Output {step.OutputNumber} → {state}",
                _       => $"STB Output {step.OutputNumber} → {state}",
            };
            var pulseSuffix = (step.PulseMs ?? 0) > 0
                ? $"  (pulse {step.PulseMs} ms{(step.PulseBlocking == true ? ", blocking" : "")})"
                : "";
            return $"{base_}{pulseSuffix}";
        }

        /// <summary>
        /// Total steps a program reports as its MaxStepCount: every step, plus routine bodies
        /// (when a repository is given), if/else branches and CNC blocks (estimated from the
        /// spec with the same arithmetic as <see cref="CncStepGenerator.Generate"/>).
        /// </summary>
        public static int CountSteps(List<ProgramStep> steps, BuiltProgramRepository? repo = null)
        {
            int count = 0;
            foreach (var s in steps)
            {
                count++;
                if (s.Type == StepType.CallRoutine && repo != null)
                {
                    var routine = !string.IsNullOrEmpty(s.RoutineId) ? repo.GetById(s.RoutineId!) : null;
                    routine ??= repo.Get(s.RoutineName ?? "");
                    if (routine != null) count += CountSteps(routine.Steps, repo);
                }
                if (s.Type == StepType.IfCondition)
                {
                    if (s.IfSteps != null) count += CountSteps(s.IfSteps, repo);
                    foreach (var elif in s.ElseIfBranches ?? []) count += CountSteps(elif.Steps, repo);
                    if (s.ElseSteps != null) count += CountSteps(s.ElseSteps, repo);
                }
                if (s.Type == StepType.CncProgram)
                {
                    if (s.CncSpec != null)
                    {
                        // Same arithmetic as GenerateCncSteps (numeric estimate —
                        // structural expressions may shift drill pass counts)
                        int perHole = 2;
                        if (s.CncSpec.HoleOp == "drill")
                        {
                            double depth = Math.Abs(s.CncSpec.HoleDepth ?? 0);
                            double peck  = s.CncSpec.HolePeck == true ? Math.Abs(s.CncSpec.HolePeckDepth ?? 0) : 0;
                            int passes = (peck > 0.0001 && depth > peck) ? (int)Math.Ceiling(depth / peck) : 1;
                            perHole = 1 + passes * 2;
                        }
                        count += perHole * (s.CncSpec.Holes?.Count ?? 0);
                        foreach (var flat in s.CncSpec.Paths ?? [])
                            if (flat is { Count: >= 4 }) count += flat.Count / 2 + 2;
                    }
                    else if (s.CncProgramSteps != null)
                        count += CountSteps(s.CncProgramSteps, repo);
                }
            }
            return count;
        }
    }
}
