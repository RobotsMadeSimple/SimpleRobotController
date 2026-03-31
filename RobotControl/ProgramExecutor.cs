using System.Diagnostics;

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

        public bool IsRunning => _running;

        public ProgramExecutor(RobotController controller, ProgramCycleManager programManager, PointRepository pointRepo, ToolRepository toolRepo)
        {
            _controller     = controller;
            _programManager = programManager;
            _pointRepo      = pointRepo;
            _toolRepo       = toolRepo;
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

            _program = program;
            _stopRequested = false;
            _awaitingMove = false;
            _running = true;
        }

        public void Stop()
        {
            if (!_running) return;

            // Clear any move commands we may have queued so the robot doesn't
            // start a new segment after the stop is acknowledged.
            _controller.QueuedCommands.Clear();
            _awaitingMove = false;
            _pendingStep  = null;

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
            _frameStack.Clear();
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
            }
        }

        private void ExecuteMove(ProgramStep step, StepListFrame frame)
        {
            if (_awaitingMove) return;

            var point = _pointRepo.Get(step.PointName ?? "");
            if (point is null)
            {
                Finish(global::ProgramStatus.Error, $"Point not found: {step.PointName}");
                return;
            }

            // Determine if a local tool offset is set on this step
            bool hasToolOffset = step.ToolOffsetX.HasValue || step.ToolOffsetY.HasValue || step.ToolOffsetZ.HasValue
                               || step.ToolOffsetRX.HasValue || step.ToolOffsetRY.HasValue || step.ToolOffsetRZ.HasValue;

            var cmd = new RobotCommand
            {
                CommandType = step.Type == StepType.MoveL ? "MoveL" : "MoveJ",
                // Target point + optional position offset
                X  = point.X  + (step.OffsetX  ?? 0),
                Y  = point.Y  + (step.OffsetY  ?? 0),
                Z  = point.Z  + (step.OffsetZ  ?? 0),
                RX = point.RX + (step.OffsetRX ?? 0),
                RY = point.RY + (step.OffsetRY ?? 0),
                RZ = point.RZ + (step.OffsetRZ ?? 0),
                // Optional local tool offset applied on top of the active tool
                TX  = hasToolOffset ? step.ToolOffsetX  ?? 0 : null,
                TY  = hasToolOffset ? step.ToolOffsetY  ?? 0 : null,
                TZ  = hasToolOffset ? step.ToolOffsetZ  ?? 0 : null,
                TRX = hasToolOffset ? step.ToolOffsetRX ?? 0 : null,
                TRY = hasToolOffset ? step.ToolOffsetRY ?? 0 : null,
                TRZ = hasToolOffset ? step.ToolOffsetRZ ?? 0 : null,
                Speed = step.Speed,
                Accel = step.Accel,
                Decel = step.Decel,
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
            // Placeholder — hardware output API not yet implemented in STB4100
            // Step advances immediately so execution is not blocked
            ReportStepCompleted(step);
            frame.Index++;
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
            if (elapsed >= (step.WaitMs ?? 0))
            {
                frame.WaitStarted = false;
                frame.Index++;
                ReportStepCompleted(step); // count it now that the wait has elapsed
            }
        }

        private void ExecuteLoop(ProgramStep step, StepListFrame frame)
        {
            var innerSteps = step.LoopSteps ?? new();
            if (innerSteps.Count == 0) { frame.Index++; ReportStepCompleted(step); return; }

            int count     = step.LoopCount ?? 1;
            int remaining = count == 0 ? int.MaxValue : count; // 0 = infinite

            ReportStepCompleted(step); // the loop header itself is done; body steps count separately
            frame.Index++;

            // Push a new frame for the loop body
            _frameStack.Push(new StepListFrame(innerSteps, 0, isLoop: true, loopRemaining: remaining));
        }

        // ── Helpers ──────────────────────────────────────────────────────────

        private int _globalStepIndex = 0;

        /// <summary>Emits a "step in progress" update — description shown but count not yet incremented.</summary>
        private void ReportStepStarted(ProgramStep step)
        {
            _programManager.ApplyStatusUpdate(new ProgramCycleUpdate
            {
                ProgramName        = _program!.Name,
                ProgramStatus      = global::ProgramStatus.Running,
                CurrentStepNumber  = _globalStepIndex,
                StepDescription    = !string.IsNullOrEmpty(step.StatusMessage) ? step.StatusMessage : StepDescription(step),
                WarningDescription = string.IsNullOrEmpty(step.StatusWarning) ? null : step.StatusWarning,
                ErrorDescription   = string.IsNullOrEmpty(step.StatusError)   ? null : step.StatusError,
            });
        }

        /// <summary>Increments the completed step count and emits the updated progress.</summary>
        private void ReportStepCompleted(ProgramStep step)
        {
            _globalStepIndex++;
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
                ProgramName       = _program!.Name,
                ProgramStatus     = status,
                CurrentStepNumber = _globalStepIndex,
                StepDescription   = description,
                ErrorDescription  = status == global::ProgramStatus.Error ? description : null,
            });

            _globalStepIndex = 0;
        }

        private static string StepDescription(ProgramStep step)
        {
            var type = step.Type switch
            {
                StepType.MoveL        => $"MoveL → {step.PointName}",
                StepType.MoveJ        => $"MoveJ → {step.PointName}",
                StepType.SetOutput    => $"Output {step.OutputNumber} → {(step.OutputValue == true ? "ON" : "OFF")}",
                StepType.Wait         => $"Wait {step.WaitMs} ms",
                StepType.Loop         => $"Loop ×{(step.LoopCount == 0 ? "∞" : step.LoopCount)}",
                StepType.StatusUpdate => step.StatusMessage ?? "Status update",
                _                     => step.Type.ToString(),
            };
            return string.IsNullOrEmpty(step.Name) ? type : $"{step.Name}  ({type})";
        }

        internal static int CountSteps(List<ProgramStep> steps)
        {
            int count = 0;
            foreach (var s in steps)
            {
                count++;
                if (s.Type == StepType.Loop && s.LoopSteps != null)
                    count += CountSteps(s.LoopSteps) * Math.Max(s.LoopCount ?? 1, 1);
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
