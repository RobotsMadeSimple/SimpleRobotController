using System.Collections.Generic;
using System.Diagnostics;
using System.Threading.Tasks;

namespace Controller.RobotControl
{
    /// <summary>
    /// Executes a BuiltProgram step-by-step inside the main control loop.
    /// Call Update() on every Loop() tick. Enqueues RobotCommands for moves
    /// and reports progress via ProgramCycleManager.
    /// </summary>
    internal class ProgramExecutor
    {
        // ── Injected dependencies ─────────────────────────────────────────────
        private readonly RobotController          _controller;
        private readonly ProgramCycleManager      _programManager;
        private readonly PointRepository          _pointRepo;
        private readonly ToolRepository           _toolRepo;
        private readonly BuiltProgramRepository   _builtProgramRepo;
        private readonly GridRepository           _gridRepo;

        // ── Execution state ──────────────────────────────────────────────────
        private BuiltProgram?   _program;
        private bool            _running;
        // volatile so the control-loop thread always sees writes from the WebSocket thread
        private volatile bool   _stopRequested;

        // Stack for nested step lists (supports Loop)
        private readonly Stack<StepListFrame> _frameStack = new();

        // For Wait steps
        private long _waitStartMs;

        // Whether we have dispatched a move command and are awaiting completion
        private bool _awaitingMove;

        // Step that was dispatched asynchronously; reported complete when the move finishes
        private ProgramStep? _pendingStep;

        // Program variables — initialised from BuiltProgram.Variables on Start(), mutated by SetVariable steps
        private readonly Dictionary<string, double> _variables = new();

        public bool IsRunning => _running;
        public string? CurrentProgramName => _program?.Name;

        public ProgramExecutor(RobotController controller, ProgramCycleManager programManager, PointRepository pointRepo, ToolRepository toolRepo, BuiltProgramRepository builtProgramRepo, GridRepository gridRepo)
        {
            _controller       = controller;
            _programManager   = programManager;
            _pointRepo        = pointRepo;
            _toolRepo         = toolRepo;
            _builtProgramRepo = builtProgramRepo;
            _gridRepo         = gridRepo;
        }

        // ── Public control ───────────────────────────────────────────────────

        public void Start(BuiltProgram program, string? imageBase64 = null)
        {
            // Register program in the cycle manager so the monitor tab can see it
            _programManager.SetAvailablePrograms(new()
            {
                new() { Name = program.Name, Description = program.Description, Image = imageBase64 }
            });

            var totalSteps = CountSteps(program.Steps);
            // Clear any terminal state so the incoming Running update is not blocked by the guard
            _programManager.ResetToReady(program.Name, totalSteps);
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName       = program.Name,
                ProgramStatus     = global::ProgramStatus.Running,
                CurrentStepNumber = 0,
                MaxStepCount      = totalSteps,
                StepDescription   = "Starting…",
            });

            _frameStack.Clear();
            _frameStack.Push(new StepListFrame(program.Steps, 0));

            // Initialise variables from the program definition
            _variables.Clear();
            foreach (var v in program.Variables ?? [])
                _variables[v.Name] = v.Value;

            _program = program;
            _stopRequested = false;
            _awaitingMove = false;
            _running = true;
        }

        public void Stop()
        {
            if (!_running) return;

            // Request a queue drain on the control loop thread — calling Clear() directly
            // here would race with RunCommands() which reads QueuedCommands on the loop thread.
            _controller.RequestQueueDrain();
            _awaitingMove = false;
            _pendingStep  = null;

            // Hard-stop any active motion profiler so IsMoving clears immediately.
            // Without this, a stuck profiler (e.g. zero-speed) would leave the
            // controller in a permanent "moving" state even after the program stops.
            _controller.HardStop();

            // Emit Stopped status immediately rather than waiting for the next Update() tick.
            Finish(global::ProgramStatus.Stopped, "Stopped by user");
        }

        /// <summary>
        /// Immediately halts execution and clears all state — no status update is emitted.
        /// The caller is responsible for pushing a final status (e.g. Ready) to programManager.
        /// </summary>
        public void Reset()
        {
            _running         = false;
            _stopRequested   = false;
            _awaitingMove    = false;
            _pendingStep     = null;
            _globalStepIndex = 0;
            _loopDepth       = 0;
            _frameStack.Clear();
            _variables.Clear();
        }

        // ── Main update — called every control loop tick ──────────────────────

        public void Update()
        {
            if (!_running || _program is null) return;

            // If we dispatched a move, wait until the queue is clear and the robot is idle
            if (_awaitingMove)
            {
                if (_controller.QueuedCommands.Count == 0 && !_controller.IsMoving)
                {
                    _awaitingMove = false;
                    // Report the step as completed now that the move has finished
                    if (_pendingStep is not null)
                    {
                        ReportStepCompleted(_pendingStep);
                        _pendingStep = null;
                    }
                }
                else
                    return;
            }

            // Nothing left to execute?
            if (_frameStack.Count == 0)
            {
                Finish(global::ProgramStatus.Complete, "Complete");
                return;
            }

            var frame = _frameStack.Peek();

            // Frame exhausted — pop and continue
            if (frame.Index >= frame.Steps.Count)
            {
                _frameStack.Pop();

                // If this was a loop frame, decrement and possibly re-push
                if (frame.IsLoop)
                {
                    frame.LoopRemaining--;
                    if (frame.LoopRemaining == 0)
                    {
                        // Loop finished; outer frame already advanced past the loop step
                        _loopDepth--;
                    }
                    else
                    {
                        // Re-run the loop body
                        _frameStack.Push(new StepListFrame(frame.Steps, 0, isLoop: true,
                            loopRemaining: frame.LoopRemaining));
                    }
                }
                return; // Re-enter next tick with updated stack
            }

            var step = frame.Steps[frame.Index];
            ExecuteStep(step, frame);
        }

        // ── Step execution ────────────────────────────────────────────────────

        private void ExecuteStep(ProgramStep step, StepListFrame frame)
        {
            switch (step.Type)
            {
                case StepType.MoveL:
                case StepType.MoveJ:
                    ExecuteMove(step, frame);
                    break;

                case StepType.SetOutput:
                    ExecuteSetOutput(step, frame);
                    break;

                case StepType.Wait:
                    ExecuteWait(step, frame);
                    break;

                case StepType.Loop:
                    ExecuteLoop(step, frame);
                    break;

                case StepType.StatusUpdate:
                    ExecuteStatusUpdate(step, frame);
                    break;

                case StepType.CallRoutine:
                    ExecuteCallRoutine(step, frame);
                    break;

                case StepType.SetSpeedL:
                    ExecuteSetSpeedL(step, frame);
                    break;

                case StepType.SetSpeedJ:
                    ExecuteSetSpeedJ(step, frame);
                    break;

                case StepType.SetVariable:
                    ExecuteSetVariable(step, frame);
                    break;
            }
        }

        private void ExecuteMove(ProgramStep step, StepListFrame frame)
        {
            if (_awaitingMove) return;

            Point point;
            if (step.GridPoint != null)
            {
                var gp   = step.GridPoint;
                var grid = _gridRepo.Get(gp.GridId);
                if (grid == null) { Finish(global::ProgramStatus.Error, $"Grid not found: {gp.GridId}"); return; }

                var basePoint = _pointRepo.Get(grid.BasePointName);
                if (basePoint == null) { Finish(global::ProgramStatus.Error, $"Grid base point not found: {grid.BasePointName}"); return; }

                int row, col;
                if (gp.UseGridIndex)
                {
                    if (!grid.ColCount.HasValue || grid.ColCount.Value <= 0)
                    {
                        Finish(global::ProgramStatus.Error, $"Grid '{grid.Name}' requires colCount to use grid index");
                        return;
                    }
                    int idx = (int)Math.Round(EvalField(step, "gridGridIndex", gp.GridIndex ?? 0));
                    row = idx / grid.ColCount.Value;
                    col = idx % grid.ColCount.Value;
                }
                else
                {
                    row = (int)Math.Round(EvalField(step, "gridRowIndex", gp.RowIndex ?? 0));
                    col = (int)Math.Round(EvalField(step, "gridColIndex", gp.ColIndex ?? 0));
                }

                double rawX = row * grid.RowOffsetX + col * grid.ColOffsetX;
                double rawY = row * grid.RowOffsetY + col * grid.ColOffsetY;
                double rawZ = row * grid.RowOffsetZ + col * grid.ColOffsetZ;

                double theta = grid.Rotation * Math.PI / 180.0;
                double rotX  = rawX * Math.Cos(theta) - rawY * Math.Sin(theta);
                double rotY  = rawX * Math.Sin(theta) + rawY * Math.Cos(theta);

                point = new Point
                {
                    X  = basePoint.X  + rotX,
                    Y  = basePoint.Y  + rotY,
                    Z  = basePoint.Z  + rawZ,
                    RX = basePoint.RX,
                    RY = basePoint.RY,
                    RZ = basePoint.RZ,
                };
            }
            else
            {
                var found = _pointRepo.Get(step.PointName ?? "");
                if (found is null)
                {
                    Finish(global::ProgramStatus.Error, $"Point not found: {step.PointName}");
                    return;
                }
                point = found;
            }

            // Determine if a local tool offset is set on this step
            bool hasToolOffset = step.ToolOffsetX.HasValue || step.ToolOffsetY.HasValue || step.ToolOffsetZ.HasValue
                               || step.ToolOffsetRX.HasValue || step.ToolOffsetRY.HasValue || step.ToolOffsetRZ.HasValue;

            var cmd = new RobotCommand
            {
                CommandType = step.Type == StepType.MoveL ? "MoveL" : "MoveJ",
                // Target point + optional position offset
                X  = point.X  + EvalField(step, "offsetX",  step.OffsetX  ?? 0),
                Y  = point.Y  + EvalField(step, "offsetY",  step.OffsetY  ?? 0),
                Z  = point.Z  + EvalField(step, "offsetZ",  step.OffsetZ  ?? 0),
                RX = point.RX + EvalField(step, "offsetRX", step.OffsetRX ?? 0),
                RY = point.RY + EvalField(step, "offsetRY", step.OffsetRY ?? 0),
                RZ = point.RZ + EvalField(step, "offsetRZ", step.OffsetRZ ?? 0),
                // Optional local tool offset applied on top of the active tool
                TX  = hasToolOffset ? EvalField(step, "toolOffsetX",  step.ToolOffsetX  ?? 0) : null,
                TY  = hasToolOffset ? EvalField(step, "toolOffsetY",  step.ToolOffsetY  ?? 0) : null,
                TZ  = hasToolOffset ? EvalField(step, "toolOffsetZ",  step.ToolOffsetZ  ?? 0) : null,
                TRX = hasToolOffset ? EvalField(step, "toolOffsetRX", step.ToolOffsetRX ?? 0) : null,
                TRY = hasToolOffset ? EvalField(step, "toolOffsetRY", step.ToolOffsetRY ?? 0) : null,
                TRZ = hasToolOffset ? EvalField(step, "toolOffsetRZ", step.ToolOffsetRZ ?? 0) : null,
                Speed = (step.Speed.HasValue || step.Expressions?.ContainsKey("speed") == true) ? EvalField(step, "speed", step.Speed ?? 0) : (double?)null,
                Accel = (step.Accel.HasValue || step.Expressions?.ContainsKey("accel") == true) ? EvalField(step, "accel", step.Accel ?? 0) : (double?)null,
                Decel = (step.Decel.HasValue || step.Expressions?.ContainsKey("decel") == true) ? EvalField(step, "decel", step.Decel ?? 0) : (double?)null,
            };

            _controller.QueuedCommands.Add(cmd);
            _awaitingMove = true;
            _pendingStep  = step; // completion is reported in Update() once the move finishes

            // Announce the step is in progress without counting it yet
            ReportStepStarted(step);
            frame.Index++;
        }

        private void ExecuteStatusUpdate(ProgramStep step, StepListFrame frame)
        {
            // StatusUpdate steps complete instantly; count them immediately
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteSetOutput(ProgramStep step, StepListFrame frame)
        {
            var card   = step.OutputCard ?? "stb";
            var number = step.OutputNumber ?? 1;
            var value  = step.OutputValue ?? false;
            var pulse  = step.PulseMs ?? 0;

            // Set to the requested state immediately
            ApplyOutput(_controller, card, number, value, step.OutputNanoId);

            // Non-blocking pulse: after the pulse duration, flip to the opposite state
            if (pulse > 0)
            {
                var ctrl   = _controller;
                var nanoId = step.OutputNanoId;
                _ = Task.Run(async () =>
                {
                    await Task.Delay(pulse);
                    ApplyOutput(ctrl, card, number, !value, nanoId);
                });
            }

            ReportStepCompleted(step);
            frame.Index++;
        }

        /// <summary>Applies a single output state to the correct IO card.</summary>
        private static void ApplyOutput(RobotController ctrl, string card, int number, bool value, string? nanoId)
        {
            switch (card)
            {
                case "relay":
                    ctrl.RelayManager.SetRelay(number, value);
                    break;
                case "nano":
                    if (!string.IsNullOrEmpty(nanoId))
                        ctrl.NanoManager.SetOutput(nanoId, number, value);
                    break;
                default: // "stb"
                    ctrl.stb.SetOutput(number, value);
                    break;
            }
        }

        private void ExecuteWait(ProgramStep step, StepListFrame frame)
        {
            if (!frame.WaitStarted)
            {
                _waitStartMs      = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                frame.WaitStarted = true;
                // Announce the wait is in progress without counting it yet
                ReportStepStarted(step);
                return;
            }

            var elapsed = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() - _waitStartMs;
            if (elapsed >= EvalField(step, "waitMs", step.WaitMs ?? 0))
            {
                frame.WaitStarted = false;
                frame.Index++;
                ReportStepCompleted(step); // count it now that the wait has elapsed
            }
        }

        private void ExecuteCallRoutine(ProgramStep step, StepListFrame frame)
        {
            var routine = _builtProgramRepo.Get(step.RoutineName ?? "");
            if (routine is null)
            {
                Finish(global::ProgramStatus.Error, $"Routine not found: {step.RoutineName}");
                return;
            }
            if (routine.Steps.Count == 0) { frame.Index++; ReportStepCompleted(step); return; }

            ReportStepCompleted(step); // the call step itself counts as done; routine steps count separately
            frame.Index++;

            // Push the routine's steps as a plain (non-loop) frame
            _frameStack.Push(new StepListFrame(routine.Steps, 0));
        }

        private void ExecuteLoop(ProgramStep step, StepListFrame frame)
        {
            var innerSteps = step.LoopSteps ?? new();
            if (innerSteps.Count == 0) { frame.Index++; ReportStepCompleted(step); return; }

            int count     = (int)EvalField(step, "loopCount", step.LoopCount ?? 1);
            int remaining = count == 0 ? int.MaxValue : count; // 0 = infinite

            ReportStepCompleted(step); // the loop header itself is done; body steps count separately
            frame.Index++;

            // Push a new frame for the loop body
            _frameStack.Push(new StepListFrame(innerSteps, 0, isLoop: true, loopRemaining: remaining));
            _loopDepth++;
        }

        private void ExecuteSetSpeedL(ProgramStep step, StepListFrame frame)
        {
            bool hasSpeed = step.Speed.HasValue || step.Expressions?.ContainsKey("speed") == true;
            bool hasAccel = step.Accel.HasValue || step.Expressions?.ContainsKey("accel") == true;
            bool hasDecel = step.Decel.HasValue || step.Expressions?.ContainsKey("decel") == true;
            if (hasSpeed)
                _controller.QueuedCommands.Add(new RobotCommand { CommandType = "SpeedS", Speed = EvalField(step, "speed", step.Speed ?? 0) });
            if (hasAccel || hasDecel)
                _controller.QueuedCommands.Add(new RobotCommand { CommandType = "AccelS",
                    Accel = hasAccel ? EvalField(step, "accel", step.Accel ?? 0) : (double?)null,
                    Decel = hasDecel ? EvalField(step, "decel", step.Decel ?? 0) : (double?)null });
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteSetSpeedJ(ProgramStep step, StepListFrame frame)
        {
            bool hasSpeed = step.Speed.HasValue || step.Expressions?.ContainsKey("speed") == true;
            bool hasAccel = step.Accel.HasValue || step.Expressions?.ContainsKey("accel") == true;
            bool hasDecel = step.Decel.HasValue || step.Expressions?.ContainsKey("decel") == true;
            if (hasSpeed)
                _controller.QueuedCommands.Add(new RobotCommand { CommandType = "SpeedJ", Speed = EvalField(step, "speed", step.Speed ?? 0) });
            if (hasAccel || hasDecel)
                _controller.QueuedCommands.Add(new RobotCommand { CommandType = "AccelJ",
                    Accel = hasAccel ? EvalField(step, "accel", step.Accel ?? 0) : (double?)null,
                    Decel = hasDecel ? EvalField(step, "decel", step.Decel ?? 0) : (double?)null });
            ReportStepCompleted(step);
            frame.Index++;
        }

        private void ExecuteSetVariable(ProgramStep step, StepListFrame frame)
        {
            if (!string.IsNullOrEmpty(step.VariableName) && !string.IsNullOrEmpty(step.VariableExpr))
            {
                try
                {
                    _variables[step.VariableName] = ExpressionEvaluator.Evaluate(step.VariableExpr, _variables);
                }
                catch
                {
                    // If expression fails, leave variable unchanged
                }
            }
            ReportStepCompleted(step);
            frame.Index++;
        }

        // ── Helpers ──────────────────────────────────────────────────────────

        /// <summary>
        /// Returns the evaluated value for a numeric field.
        /// If the step has an expression keyed by <paramref name="fieldName"/>, that expression is
        /// evaluated against the current variable dictionary; otherwise <paramref name="fallback"/> is returned.
        /// </summary>
        private double EvalField(ProgramStep step, string fieldName, double fallback)
        {
            if (step.Expressions != null && step.Expressions.TryGetValue(fieldName, out var expr))
            {
                try { return ExpressionEvaluator.Evaluate(expr, _variables); }
                catch { /* fall through */ }
            }
            return fallback;
        }

        private int _globalStepIndex = 0;
        private int _loopDepth       = 0;

        /// <summary>Emits a "step in progress" update — description shown but count not yet incremented.</summary>
        private void ReportStepStarted(ProgramStep step)
        {
            var isMove = step.Type == StepType.MoveL || step.Type == StepType.MoveJ;

            double? Off(string key, double? raw) =>
                isMove && (raw.HasValue || (step.Expressions?.ContainsKey(key) == true))
                    ? EvalField(step, key, raw ?? 0)
                    : (double?)null;

            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName          = _program!.Name,
                ProgramStatus        = global::ProgramStatus.Running,
                CurrentStepNumber    = _globalStepIndex,
                StepDescription      = !string.IsNullOrEmpty(step.StatusMessage) ? step.StatusMessage : StepDescription(step),
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

        /// <summary>Increments the completed step count and emits the updated progress.</summary>
        private void ReportStepCompleted(ProgramStep step)
        {
            if (_loopDepth == 0) _globalStepIndex++;
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName        = _program!.Name,
                ProgramStatus      = global::ProgramStatus.Running,
                CurrentStepNumber  = _globalStepIndex,
                StepDescription    = !string.IsNullOrEmpty(step.StatusMessage) ? step.StatusMessage : StepDescription(step),
                WarningDescription = string.IsNullOrEmpty(step.StatusWarning) ? null : step.StatusWarning,
                ErrorDescription   = string.IsNullOrEmpty(step.StatusError)   ? null : step.StatusError,
                ShouldLog          = true,
            });
        }

        private void Finish(global::ProgramStatus status, string description)
        {
            _running       = false;
            _stopRequested = false;
            _awaitingMove  = false;
            _pendingStep   = null;
            _frameStack.Clear();

            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName          = _program!.Name,
                ProgramStatus        = status,
                CurrentStepNumber    = _globalStepIndex,
                StepDescription      = description,
                ErrorDescription     = status == global::ProgramStatus.Error ? description : null,
                CurrentPointName  = "",
                CurrentOffsetX   = null, CurrentOffsetY  = null, CurrentOffsetZ  = null,
                CurrentOffsetRX  = null, CurrentOffsetRY = null, CurrentOffsetRZ = null,
                CurrentToolOffsetX  = null, CurrentToolOffsetY  = null, CurrentToolOffsetZ  = null,
                CurrentToolOffsetRX = null, CurrentToolOffsetRY = null, CurrentToolOffsetRZ = null,
            });

            _globalStepIndex = 0;
            _loopDepth       = 0;
        }

        private static string StepDescription(ProgramStep step)
        {
            var type = step.Type switch
            {
                StepType.MoveL        => $"MoveL → {step.PointName}",
                StepType.MoveJ        => $"MoveJ → {step.PointName}",
                StepType.SetOutput    => BuildSetOutputDescription(step),
                StepType.Wait         => $"Wait {step.WaitMs} ms",
                StepType.Loop         => $"Loop ×{(step.LoopCount == 0 ? "∞" : step.LoopCount)}",
                StepType.StatusUpdate => step.StatusMessage ?? "Status update",
                StepType.CallRoutine  => $"Routine → {step.RoutineName}",
                StepType.SetSpeedL    => $"Set Linear Speed → {step.Speed} mm/s",
                StepType.SetSpeedJ    => $"Set Joint Speed → {step.Speed} mm/s",
                StepType.SetVariable  => $"${step.VariableName} = {step.VariableExpr}",
                _                     => step.Type.ToString(),
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
            return (step.PulseMs ?? 0) > 0 ? $"{base_}  (pulse {step.PulseMs} ms)" : base_;
        }

        internal static int CountSteps(List<ProgramStep> steps, BuiltProgramRepository? repo = null)
        {
            int count = 0;
            foreach (var s in steps)
            {
                count++;
                if (s.Type == StepType.CallRoutine && repo != null)
                {
                    var routine = repo.Get(s.RoutineName ?? "");
                    if (routine != null) count += CountSteps(routine.Steps, repo);
                }
            }
            return count;
        }

        // ── Inner frame class ─────────────────────────────────────────────────
        private class StepListFrame
        {
            public List<ProgramStep> Steps       { get; }
            public int               Index       { get; set; }
            public bool              IsLoop      { get; }
            public int               LoopRemaining { get; set; }
            public bool              WaitStarted { get; set; }

            public StepListFrame(List<ProgramStep> steps, int index, bool isLoop = false, int loopRemaining = 0)
            {
                Steps         = steps;
                Index         = index;
                IsLoop        = isLoop;
                LoopRemaining = loopRemaining;
            }
        }
    }
}
