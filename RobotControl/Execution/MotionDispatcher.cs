namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Sends program motion to the robot's command queue and tracks what the executor is
    /// waiting on: the in-flight robot move (or blended run), an aux-axis move, and the
    /// snapshot a user Stop takes so Continue can re-dispatch the interrupted motion.
    /// </summary>
    /// <remarks>Not thread-safe; used under the owning executor's control lock only.</remarks>
    internal sealed class MotionDispatcher
    {
        private readonly RobotController _controller;

        public MotionDispatcher(RobotController controller) => _controller = controller;

        // ── What the executor is waiting on ───────────────────────────────────

        /// <summary>A robot move was dispatched and has not finished yet.</summary>
        public bool AwaitingMove { get; set; }

        /// <summary>Step dispatched asynchronously; reported complete when the move finishes.</summary>
        public ProgramStep? PendingStep { get; set; }

        /// <summary>Steps consumed by a single blended (continuous) move; all reported on completion.</summary>
        public List<ProgramStep>? PendingSteps { get; set; }

        /// <summary>An aux indexed move with WaitForDone=true is in progress.</summary>
        public bool AwaitingAux { get; set; }
        public ProgramStep? PendingAuxStep { get; set; }

        /// <summary>Program default blend radius (set by SetBlendRadius); per-move BlendRadius overrides it.</summary>
        public double DefaultBlendRadius { get; set; }

        // ── Pause/resume snapshot ─────────────────────────────────────────────
        // The last dispatched motion, remembered so a user Stop can pause the
        // program and Continue can re-dispatch the interrupted move. The saved
        // command/waypoints hold fully-resolved ABSOLUTE targets, so moves that
        // were resolved relative to "current position" resume toward their
        // original target instead of re-resolving from wherever the robot
        // stopped.
        private RobotCommand?  _lastMotionCommand;   // single queued MoveL/MoveJ (incl. jump legs, thread strokes)
        private List<Vector6>? _lastRunWaypoints;    // continuous (blended) path
        private List<double>?  _lastRunRadii;
        private Vector6?       _lastRunStart;        // robot position when the run was dispatched
        private double? _lastRunSpeed, _lastRunAccel, _lastRunDecel;
        // Captured at Stop() for Resume() to re-dispatch:
        private RobotCommand?      _resumeCommand;
        private List<Vector6>?     _resumeRunWaypoints;
        private List<double>?      _resumeRunRadii;
        private ProgramStep?       _resumePendingStep;
        private List<ProgramStep>? _resumePendingSteps;

        /// <summary>
        /// Set by Resume() (WebSocket thread, under the executor lock); consumed by Update()
        /// on the control loop thread so the actual motion dispatch happens on the loop.
        /// </summary>
        public bool ResumeDispatchPending { get; private set; }

        // ── Dispatch ──────────────────────────────────────────────────────────

        /// <summary>
        /// Enqueues a robot motion command, remembering it (with its fully-resolved
        /// absolute target) so a user Stop can be resumed mid-move.
        /// </summary>
        public void Enqueue(RobotCommand cmd)
        {
            _lastMotionCommand = cmd;
            _lastRunWaypoints  = null;
            _lastRunRadii      = null;
            _lastRunStart      = null;
            _controller.QueuedCommands.Enqueue(cmd);
        }

        /// <summary>Enqueues a command that is not itself motion (speed/accel settings).</summary>
        public void EnqueueSetting(RobotCommand cmd) => _controller.QueuedCommands.Enqueue(cmd);

        /// <summary>
        /// Dispatches a continuous (blended) path, remembering its waypoints so a user Stop
        /// can resume the remainder of the path.
        /// </summary>
        public void StartContinuous(List<Vector6> waypoints, List<double> radii,
            double? speed, double? accel, double? decel)
        {
            _lastRunWaypoints  = waypoints;
            _lastRunRadii      = radii;
            _lastRunStart      = _controller.GetCurrentPosition();
            _lastRunSpeed      = speed;
            _lastRunAccel      = accel;
            _lastRunDecel      = decel;
            _lastMotionCommand = null;
            // Enqueue rather than calling StartContinuousMove directly: the blended
            // path must be started on the motion thread (which owns continuousProfiler),
            // not here on the program-execution thread.
            _controller.QueuedCommands.Enqueue(new RobotCommand
            {
                CommandType        = "StartContinuous",
                Waypoints          = waypoints,
                BlendRadii         = radii,
                Speed              = speed,
                Accel              = accel,
                Decel              = decel,
                ApplySpeedOverride = true,
            });
        }

        /// <summary>True once the queue is drained and the motion thread reports idle.</summary>
        public bool MoveFinished =>
            // MotionBusy (not IsMoving) — the motion thread owns the profiler
            // fields; MotionBusy is the published, race-free completion signal.
            _controller.QueuedCommands.IsEmpty && !_controller.MotionBusy;

        // ── Stop / pause / resume ─────────────────────────────────────────────

        /// <summary>
        /// On a user Stop: remembers the interrupted motion for <see cref="DispatchResume"/>
        /// and clears the main-motion wait. Aux waits stay armed.
        /// </summary>
        public void CaptureResumeSnapshot()
        {
            _resumeCommand      = null;
            _resumeRunWaypoints = null;
            _resumeRunRadii     = null;
            _resumePendingStep  = null;
            _resumePendingSteps = null;
            if (AwaitingMove)
            {
                if (_lastRunWaypoints is { Count: > 0 })
                {
                    // Continuous (blended) path: find the segment the robot
                    // stopped on and resume from that segment's END — always
                    // straight ahead, never backtracking to a passed waypoint.
                    var pos  = _controller.GetCurrentPosition();
                    int best = MoveTargetResolver.FindResumeIndex(pos, _lastRunStart ?? _lastRunWaypoints[0], _lastRunWaypoints);
                    _resumeRunWaypoints = _lastRunWaypoints.GetRange(best, _lastRunWaypoints.Count - best);
                    _resumeRunRadii     = _lastRunRadii!.GetRange(best, _lastRunRadii.Count - best);
                }
                else if (_lastMotionCommand != null)
                {
                    _resumeCommand = _lastMotionCommand;
                }
                _resumePendingStep  = PendingStep;
                _resumePendingSteps = PendingSteps;
            }

            // Clear only the main-motion wait. Aux, vision, and background waits
            // stay armed so the program picks them up again after Resume().
            AwaitingMove = false;
            PendingStep  = null;
            PendingSteps = null;
        }

        /// <summary>On Resume: schedules the interrupted motion (if any) for the next tick.</summary>
        public void ArmResume() =>
            ResumeDispatchPending = _resumeRunWaypoints != null || _resumeCommand != null;

        /// <summary>
        /// Re-dispatches motion interrupted by a pause. Runs from Update() on the control
        /// loop thread, before any step execution can advance.
        /// </summary>
        public void DispatchResume()
        {
            ResumeDispatchPending = false;
            if (_resumeRunWaypoints is { Count: > 0 })
            {
                StartContinuous(_resumeRunWaypoints, _resumeRunRadii!,
                    _lastRunSpeed, _lastRunAccel, _lastRunDecel);
                PendingStep  = _resumePendingStep;
                PendingSteps = _resumePendingSteps;
                AwaitingMove = true;
            }
            else if (_resumeCommand != null)
            {
                Enqueue(_resumeCommand);
                PendingStep  = _resumePendingStep;
                PendingSteps = _resumePendingSteps;
                AwaitingMove = true;
            }
            _resumeCommand      = null;
            _resumeRunWaypoints = null;
            _resumeRunRadii     = null;
            _resumePendingStep  = null;
            _resumePendingSteps = null;
        }

        /// <summary>A PauseProgram step: drop the move and aux waits (blended-run steps are kept).</summary>
        public void ClearWaitsForPause()
        {
            AwaitingMove   = false;
            PendingStep    = null;
            AwaitingAux    = false;
            PendingAuxStep = null;
        }

        /// <summary>Back to idle for a new run.</summary>
        public void Reset()
        {
            DefaultBlendRadius    = 0;

            AwaitingMove          = false;
            PendingStep           = null;
            PendingSteps          = null;
            AwaitingAux           = false;
            PendingAuxStep        = null;

            _lastMotionCommand    = null;
            _lastRunWaypoints     = null;
            _lastRunRadii         = null;
            _lastRunStart         = null;
            _lastRunSpeed = _lastRunAccel = _lastRunDecel = null;
            _resumeCommand        = null;
            _resumeRunWaypoints   = null;
            _resumeRunRadii       = null;
            _resumePendingStep    = null;
            _resumePendingSteps   = null;
            ResumeDispatchPending = false;
        }
    }
}
