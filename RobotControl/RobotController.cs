using Controller.RobotControl.Commands;
using Controller.RobotControl.Controllers.STB4100;
using Controller.RobotControl.Hosting;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.AuxAxis;
using Controller.RobotControl.MotionProfilers;
using Controller.RobotControl.Nano;
using Controller.RobotControl.Robots;
using Controller.RobotControl.Robots.ASTRO;
using Controller.RobotControl.Robots.CNC4Axis;
using Controller.RobotControl.UsbRelay;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Linq;
using System.Numerics;

namespace Controller.RobotControl
{
    internal class RobotController : IHomingHost
    {
        // ── Tuning constants ──────────────────────────────────────────────────
        /// <summary>Speed override bounds: 5%–200% of the programmed speed.</summary>
        internal const double MinSpeedOverrideFactor = 0.05;
        internal const double MaxSpeedOverrideFactor = 2.0;
        /// <summary>CNC jogs run at a third of the commanded jog speed.</summary>
        private const double CncJogSpeedDivisor = 3.0;
        /// <summary>How often the program thread refreshes the Nano status light.</summary>
        private const long StatusLightPeriodMs = 500;

        public PointRepository       pointRepo       = new();
        public ToolRepository        toolRepo        = new();
        public LocalRepository       localRepo       = new();
        public BuiltProgramRepository builtProgramRepo = new();
        public GridRepository  gridRepo  = new();
        public StackRepository stackRepo = new();
        public STB4100 stb = new();
        private RobotIdentity _identity = new();
        public Action<RobotIdentity>? OnIdentityChanged;
        // ── Motion error latch ────────────────────────────────────────────
        // Set on the motion thread when a queued move cannot be executed (e.g. the
        // named point does not exist). The program executor consumes it after each
        // awaited move so a dropped move is never mistaken for a completed one.
        private volatile string? _lastMotionError;

        /// <summary>Records a motion-level failure for the next ConsumeMotionError() call.</summary>
        internal void LatchMotionError(string message) => _lastMotionError = message;

        /// <summary>Returns and clears the last motion error, or null if none.</summary>
        public string? ConsumeMotionError() => Interlocked.Exchange(ref _lastMotionError, null);
        private IRobotKinematics _kinematics = new ASTROKinematics();
        private readonly ProgramCycleManager programManager = new();
        private ProgramExecutor? programExecutor;
        private BackgroundProgramManager backgroundProgramManager = null!;

        // Command names AddCommand may enqueue — exactly the cases RunCommands handles.
        // Anything else is rejected with "unknownCommand" instead of being queued as a no-op.
        internal static readonly HashSet<string> QueuedMotionCommandNames = new(StringComparer.Ordinal)
        {
            "MoveL", "OffsetL", "MoveJ", "StartContinuous", "SetTool",
            "SpeedS", "AccelS", "SpeedJ", "AccelJ", "JogL", "JogJ", "JogTool",
        };

        // Work posted by WebSocket/other threads that mutates motion-owned state.
        // Drained on the motion thread at the top of every MotionLoop tick so the
        // motion thread remains the sole writer of that state.
        private readonly ConcurrentQueue<Action> _controlThreadActions = new();
        private void PostToMotionThread(Action action) => _controlThreadActions.Enqueue(action);

        // Motion thread only. Runs every posted action; one failing action is logged
        // and does not prevent the rest from running. Cheap no-op when empty.
        private void DrainControlThreadActions()
        {
            while (_controlThreadActions.TryDequeue(out var action))
            {
                try
                {
                    action();
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[MotionLoop] Posted control action failed: {ex}");
                }
            }
        }


        private static readonly string _version = GetAssemblyVersion();
        public static string Version => _version;

        private static string GetAssemblyVersion()
        {
            var asm = System.Reflection.Assembly.GetExecutingAssembly();
            var info = System.Reflection.CustomAttributeExtensions
                .GetCustomAttribute<System.Reflection.AssemblyInformationalVersionAttribute>(asm)
                ?.InformationalVersion ?? "0.0.0";
            var plus = info.IndexOf('+');
            return plus >= 0 ? info[..plus] : info;
        }

        // Hard-stop flag — set from any thread, consumed exclusively on the control loop thread
        private volatile bool _hardStopRequested;

        // Drain-queue flag — set from any thread, consumed exclusively on the control loop thread in RunCommands()
        private volatile bool _drainQueueRequested;

        // Jog epoch. Bumped by StopJog (any thread); each jog command is stamped with
        // the current value when enqueued. A queued jog whose stamp is behind the
        // current generation was superseded by a stop and is dropped rather than
        // re-enabling motion after the operator released.
        private volatile int _jogGeneration;

        // ── Joint soft-limit fault ────────────────────────────────────────────
        // A commanded move that crosses a joint limit latches _faulted: all motion
        // halts and stays halted until the operator engages _limitBypass (recovery
        // jogging, corrective direction only) and/or clears the fault. Read on the
        // WS threads for status, mutated only on the control-loop thread.
        private volatile bool _faulted;
        private volatile bool _limitBypass;
        private volatile int    _faultJoint = -1;     // 0..3 joint index, -1 = none
        private volatile int    _faultDirection;      // +1 past max, -1 past min (the unsafe direction)
        private volatile string _faultMessage = "";
        // Joint targets as they stood at the start of this control tick — the clamp
        // reference for "don't move a joint further out of range than it already is".
        private Vector6 _jointsBeforeTick = new();

        // Joint motion profiler
        private Vector6MotionProfiler? jointMotionProfiler;
        private Vector6 TargetJoints = new();
        private double SpeedJ = 100;
        private double AccelJ = 100;
        private double DecelJ = 100;

        // Linear Positioning
        private Vector6MotionProfiler? linearMotionProfiler;
        private Vector6 TargetPosition = new();
        private double SpeedS = 100; // Linear Max Velocity
        private double AccelS = 100; // Linear Acceleration
        private double DecelS = 100; // Linear Deceleration

        // Continuous (blended) linear pathing — runs a multi-waypoint blended path.
        private Controller.RobotControl.MotionProfilers.ContinuousPathingProfiler? continuousProfiler;

        // Current Status of Robot
        private Vector6 CurrentPosition = new();  // Actual position of the robot
        public Vector6 GetCurrentPosition() => new Vector6(CurrentPosition.X, CurrentPosition.Y, CurrentPosition.Z, CurrentPosition.RX, CurrentPosition.RY, CurrentPosition.RZ);
        public bool IsMoving => linearMotionProfiler is not null || jointMotionProfiler is not null || continuousProfiler is not null || IsJogging || IsJointJogging || IsToolJogging;

        // Cross-thread motion-busy signal. The motion thread owns all motion state;
        // the program-execution thread must NOT read the profiler fields directly
        // (torn/stale reads across the thread boundary). It reads MotionBusy instead.
        // The motion thread sets this true in RunCommands the instant it begins a
        // motion command — BEFORE dequeuing it — and republishes it as IsMoving at
        // the end of every motion tick. Setting it before the dequeue closes the
        // completion race: the executor can never observe an empty queue together
        // with a stale "not moving" for a move that has just been picked up.
        private volatile bool _motionActive;
        public bool MotionBusy => _motionActive;
        // X is away from flange, Y is towards the inside of the robot, Z is Vertical
        public Vector6 CurrentTool = new(0, 0, 0);
        // Current Pose Of the Joints
        private Vector6 CurrentJointTargets = new();

        // Active tool name — "" means no tool (origin Vector6)
        private string activeTool = "";
        // Active local name — "" means no local (zero offset)
        private string activeLocal = "";
        public Vector6 CurrentLocal = Vector6.Zero;
        /// <summary>Legacy string homing state ("WaitingForStart" when idle), backed by the sequencer's phase.</summary>
        public string HomingState => _homing.StateName;
        public void TriggerHoming() => startHoming = true;
        public void ApplyLocal(string? name)
        {
            if (string.IsNullOrEmpty(name) || string.Equals(name, "none", StringComparison.OrdinalIgnoreCase))
            {
                activeLocal  = "";
                CurrentLocal = Vector6.Zero;
            }
            else
            {
                var local = localRepo.Get(name);
                if (local != null) { activeLocal = name; CurrentLocal = new Vector6(local.X, local.Y, local.Z, local.RX, local.RY, local.RZ); }
            }
        }

        /// <summary>
        /// The active local's offset, or null when none is set. This is the frame
        /// shift applied to ABSOLUTE targets: saved points resolved by direct
        /// MoveL/MoveJ commands, and program moves (the executor seeds its local
        /// from this at program start; SetLocal steps override it).
        /// </summary>
        public Vector6? ActiveLocalOffset => string.IsNullOrEmpty(activeLocal) ? null : CurrentLocal;
        public void ApplyTool(string? name)
        {
            if (string.IsNullOrEmpty(name) || name == "none")
            {
                activeTool  = "";
                CurrentTool = Vector6.Zero;
            }
            else
            {
                var tool = toolRepo.Get(name);
                if (tool != null) { activeTool = name; CurrentTool = new Vector6(tool.X, tool.Y, tool.Z, tool.RX, tool.RY, tool.RZ); }
            }
            CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
            CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
        }

        // Robot configuration (homing offsets, speeds, etc.). Volatile reference:
        // replaced by SetConfig, read on the motion, program and WS threads.
        private volatile RobotConfig _config = new();

        // If the Robot was homed from startup
        // Volatile: written on the motion thread, read on WS/program threads (and
        // startHoming is requested from those threads).
        private volatile bool homed = false;
        private volatile bool startHoming = false;

        // Homing state machine — ticked by RunHoming on the motion thread.
        private readonly HomingSequencer _homing;

        // WebSocket command name → handler (see Commands/).
        private readonly CommandDispatcher _commands;

        private JoggingMotionProfiler joggingMotionProfiler = new();
        private JoggingMotionProfiler jointJoggingProfiler = new();
        private ToolJoggingMotionProfiler toolJoggingMotionProfiler = new();

        private bool IsJogging => !joggingMotionProfiler.IsFinished;
        private bool IsJointJogging => !jointJoggingProfiler.IsFinished;
        private bool IsToolJogging => !toolJoggingMotionProfiler.IsFinished;

        // ConcurrentQueue allows WebSocket handler threads to Enqueue safely while
        // the control loop thread reads via TryPeek / TryDequeue. Clear() is used by
        // ExecuteHardStop() and the drain-flag path — both run on the loop thread.
        public ConcurrentQueue<RobotCommand> QueuedCommands = new();

        /// <summary>
        /// Resolved XY toolpath of the CNC block currently executing (anchor and
        /// runtime variables applied). Set by the program executor when a CNC
        /// block starts, cleared when it finishes. Read by GetCncToolpath so the
        /// monitor can preview the path being made. Volatile reference swap —
        /// written on the control loop thread, read from WebSocket threads.
        /// </summary>
        public sealed record CncToolpathInfo(string ProgramName, List<List<double>> Paths, List<CncHole> Holes);
        public volatile CncToolpathInfo? ActiveCncToolpath;

        // Speed override: MinSpeedOverrideFactor–MaxSpeedOverrideFactor (5%–200%), default 1.0 (100%)
        public double SpeedOverrideFactor { get; internal set; } = 1.0;

        // ── Nano IO ───────────────────────────────────────────────────────────
        public NanoManager NanoManager { get; private set; } = null!;
        private long _lastStatusLightMs = 0;

        // ── USB Relay ─────────────────────────────────────────────────────────
        public UsbRelayManager RelayManager { get; private set; } = null!;

        // ── Aux Stepper Axes ──────────────────────────────────────────────────
        public AuxAxisManager AuxAxisManager { get; private set; } = null!;

        // ── USB Cameras ───────────────────────────────────────────────────────
        public Camera.CameraManager CameraManager { get; private set; } = null!;

        // ── Vision ────────────────────────────────────────────────────────────
        public Vision.VisionProgramRepository VisionRepo    { get; private set; } = null!;
        public Vision.VisionManager           VisionManager { get; private set; } = null!;

        // ── Camera-to-robot calibration ───────────────────────────────────────
        public Vision.Calibration.CameraCalibrationRepository CalibrationRepo     { get; private set; } = null!;
        public Vision.Calibration.CalibrationSessionManager   CalibrationSessions { get; } = new();

        // ── Webhooks ──────────────────────────────────────────────────────────
        public WebhookManager WebhookManager { get; private set; } = new();

        // ── Program vision snapshots ──────────────────────────────────────────
        private readonly Dictionary<string, byte[]> _programVisionSnapshots = new();
        private readonly object _visionSnapshotLock = new();

        public void SetProgramVisionSnapshot(string visionProgramId, byte[] jpeg)
        {
            lock (_visionSnapshotLock) _programVisionSnapshots[visionProgramId] = jpeg;
        }

        public byte[]? GetProgramVisionSnapshot(string visionProgramId)
        {
            lock (_visionSnapshotLock)
                return _programVisionSnapshots.TryGetValue(visionProgramId, out var b) ? b : null;
        }

        // Last inspection result per program — persists after the RunVision step's
        // processor stops, so the monitor page can show values with the snapshot.
        private readonly Dictionary<string, Vision.VisionResult> _programVisionResults = new();

        public void SetProgramVisionResult(string visionProgramId, Vision.VisionResult result)
        {
            lock (_visionSnapshotLock) _programVisionResults[visionProgramId] = result;
        }

        public Vision.VisionResult? GetProgramVisionResult(string visionProgramId)
        {
            lock (_visionSnapshotLock)
                return _programVisionResults.TryGetValue(visionProgramId, out var r) ? r : null;
        }

        private string? _auxActiveDeviceId;
        private int     _auxActiveAxis;

        public bool IsAuxMoving =>
            _auxActiveDeviceId is not null && AuxAxisManager.IsDeviceMoving(_auxActiveDeviceId);

        public RobotController()
        {
            // Construction only — no device connections or threads are started
            // here. Call Start() once identity/config have been applied so the
            // first motion tick and the first STB write already see the real config.
            _homing = new HomingSequencer(this);

            NanoManager = new NanoManager("nano_config.json");

            RelayManager = new UsbRelayManager();

            AuxAxisManager = new AuxAxisManager("aux_config.json");

            CameraManager = new Camera.CameraManager("camera_config.json");

            VisionRepo    = new Vision.VisionProgramRepository("vision_programs");
            VisionManager = new Vision.VisionManager(CameraManager, VisionRepo);
            CalibrationRepo = new Vision.Calibration.CameraCalibrationRepository("cameraCalibrations");


            backgroundProgramManager = new BackgroundProgramManager(
                this, programManager, pointRepo, toolRepo, localRepo, builtProgramRepo, gridRepo, stackRepo);

            programExecutor = new ProgramExecutor(
                this, programManager, pointRepo, toolRepo, localRepo, builtProgramRepo, gridRepo, stackRepo,
                isBackground: false, globalVars: backgroundProgramManager.GlobalVars,
                globalImages: backgroundProgramManager.GlobalImages, backgroundManager: backgroundProgramManager);

            _commands = CommandDispatcher.Create(this, programManager, programExecutor, backgroundProgramManager);
        }

        private int _started;

        /// <summary>
        /// Starts the device managers, the STB driver and the motion/program threads.
        /// Call once, after SetIdentity/SetConfig, so nothing runs against the default
        /// config. Subsequent calls are no-ops.
        /// </summary>
        public void Start()
        {
            if (Interlocked.Exchange(ref _started, 1) != 0)
                return;

            NanoManager.Start();
            RelayManager.Start();
            AuxAxisManager.Start();
            CameraManager.Start();

            stb.Start();

            // Motion and program execution run on SEPARATE threads. The motion
            // thread owns all motion state and runs unthrottled; the program thread
            // drives the built/background programs and sleeps between ticks. A stall
            // in program execution (GC, preemption, a slow step) can no longer freeze
            // the arm — the motion thread keeps running. They communicate only
            // through QueuedCommands, the volatile request flags, and MotionBusy.
            new Thread(MotionLoop)  { IsBackground = true, Name = "MotionLoop"  }.Start();
            new Thread(ProgramLoop) { IsBackground = true, Name = "ProgramLoop" }.Start();
        }

        // LOAD-BEARING TIMING — see docs/stb-loop-timing.md before changing.
        // Motion thread. Unthrottled, exactly like the PR #100 era: no gate, no
        // sleep, no spin. Owns every motion-state field (profilers, positions,
        // targets, homing state) — nothing else may mutate them. Adding any
        // gate/Sleep/SpinWait here delays sensor reaction and reintroduces homing
        // overshoot (#112). Burns a core continuously.
        private void MotionLoop()
        {
            while (true)
            {
                try
                {
                    // [diag] phase timing — off unless Diag.Enabled (guarded so the
                    // interpolated strings below never allocate in normal operation).
                    bool diag = Diag.Enabled;
                    if (diag) Diag.LoopSw.Restart();
                    int gc0 = diag ? GC.CollectionCount(0) : 0;
                    int gc2 = diag ? GC.CollectionCount(2) : 0;

                    // Consume hard-stop flag before anything else touches the profilers
                    if (_hardStopRequested)
                        ExecuteHardStop();

                    // Apply motion-state changes posted from other threads (tool/local
                    // selection, StopJog, fault/bypass, SetHomed, kinematics reconfig)
                    // so this thread stays the sole writer of motion state.
                    DrainControlThreadActions();

                    // Execute pending robot commands (creates/updates profilers)
                    RunCommands();
                    double tCmds = diag ? Diag.LoopSw.Elapsed.TotalMilliseconds : 0;

                    // Advance the active motion profile toward the target
                    RunMotion();
                    double tMotion = diag ? Diag.LoopSw.Elapsed.TotalMilliseconds : 0;

                    // Execute Homing
                    RunHoming();
                    double tHoming = diag ? Diag.LoopSw.Elapsed.TotalMilliseconds : 0;

                    // Let the stepper motor drive toward the new targets, and publish
                    // the motion-busy signal the program thread polls for completion.
                    stb.moving   = IsMoving;
                    _motionActive = IsMoving;

                    if (diag)
                    {
                        if (tHoming > Diag.SlowTickMs)
                            Diag.Log($"motion-cycle {tHoming:F1}ms | cmds={tCmds:F1} " +
                                     $"motion={tMotion - tCmds:F1} homing={tHoming - tMotion:F1} " +
                                     $"| gc0={GC.CollectionCount(0) - gc0} gc2={GC.CollectionCount(2) - gc2}");
                        Diag.Tick(tHoming, $"q={QueuedCommands.Count} mv={IsMoving} busy={_motionActive} " +
                                  $"lin={linearMotionProfiler is not null} cont={continuousProfiler is not null} " +
                                  $"jnt={jointMotionProfiler is not null} jog={IsJogging}/{IsJointJogging}/{IsToolJogging} home={_homing.StateName}");
                    }
                }
                catch (Exception ex)
                {
                    // Catch-all: log, hard-stop the robot, then keep looping.
                    // The process must survive any tick-level exception.
                    Console.WriteLine($"[MotionLoop] Unhandled exception on tick: {ex}");
                    try
                    {
                        ExecuteHardStop();
                    }
                    catch (Exception stopEx)
                    {
                        Console.WriteLine($"[MotionLoop] Hard stop after exception failed: {stopEx}");
                    }
                }
            }
        }

        // Program-execution thread. Drives the built program and background programs
        // and updates the status light, then sleeps. Never touches motion state
        // directly — it enqueues motion via QueuedCommands and reads completion via
        // MotionBusy — so a stall here cannot delay the motion thread.
        private void ProgramLoop()
        {
            while (true)
            {
                try
                {
                    bool diag = Diag.Enabled;
                    long ts = diag ? Diag.Now() : 0;
                    programExecutor?.Update();
                    backgroundProgramManager.Update();
                    if (diag)
                    {
                        double progMs = Diag.MsBetween(ts);
                        if (progMs > Diag.SlowStepMs)
                            Diag.Log($"prog-cycle {progMs:F1}ms (executor thread — motion NOT affected)");
                    }

                    long nowMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                    if (nowMs - _lastStatusLightMs >= StatusLightPeriodMs)
                    {
                        _lastStatusLightMs = nowMs;
                        UpdateStatusLight();
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[ProgramLoop] Unhandled exception on tick: {ex}");
                }

                Thread.Sleep(1);
            }
        }

        /// <summary>
        /// Sets all pixels on the first configured Neopixel strip to a colour that
        /// reflects the current robot state.  Colours:
        ///   Purple  — Nano not connected
        ///   Red     — Motor driver not connected
        ///   Yellow  — Homing in progress
        ///   Orange  — Not yet homed
        ///   Blue    — Moving
        ///   Green   — Idle and ready
        /// </summary>
        private void UpdateStatusLight()
        {
            var neoResult = NanoManager.FindFirstNeopixel();
            if (neoResult == null) return;

            var (device, neoPin) = neoResult.Value;

            NeoPixelColor color;

            if (!device.Connected)
                color = NeoPixelColor.Purple;
            else if (!stb.connected)
                color = NeoPixelColor.Red;
            else if (startHoming || _homing.IsActive)
                color = NeoPixelColor.Yellow;
            else if (!homed)
                color = NeoPixelColor.Orange;
            else if (IsMoving)
                color = NeoPixelColor.Blue;
            else
                color = NeoPixelColor.Green;

            var colors = new NeoPixelColor[neoPin.PixelCount];
            for (int i = 0; i < colors.Length; i++)
                colors[i] = color;

            NanoManager.SetNeoPixel(device.Id, neoPin.Pin, colors);
        }

        public void RunMotion()
        {
            // Snapshot the committed joint targets before any profiler advances them —
            // the soft-limit clamp uses this as the "where we already are" reference.
            _jointsBeforeTick.Copy(CurrentJointTargets);

            if (continuousProfiler is not null)
            {
                CurrentPosition = continuousProfiler.Loop();
                if (continuousProfiler.IsFinished)
                {
                    CurrentPosition.Copy(TargetPosition);
                    continuousProfiler = null;
                }

                // Per-tick IK, same as the plain linear path.
                CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                UpdateJointTargets();
                if (_lastClampViolated) CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
            }
            else if (linearMotionProfiler is not null)
            {
                CurrentPosition = linearMotionProfiler.Update();
                if (linearMotionProfiler.IsFinished)
                {
                    // Set the position to the final postiion
                    CurrentPosition.Copy(TargetPosition);
                    // Destroy the profiler
                    linearMotionProfiler = null;
                }

                // Calculate IK to get the joint targets for the next interpolated linear movement
                CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                UpdateJointTargets();
                if (_lastClampViolated) CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
            }
            else if (jointMotionProfiler is not null)
            {
                // Always update first (matches linear pattern)
                CurrentJointTargets = jointMotionProfiler.Update();

                if (jointMotionProfiler.IsFinished)
                {
                    // Snap to exact target joints on the same iteration the profiler finishes,
                    // so the motors are commanded to the precise endpoint rather than whatever
                    // floating-point value the profiler's last step returned
                    CurrentJointTargets.Copy(TargetJoints);

                    // Destroy the profiler
                    jointMotionProfiler = null;
                }

                // Update the joint angles with the new calculated ones
                UpdateJointTargets();

                // Recalculate the Cartesian Coordinate position to keep it current
                CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
            }
            else if (IsJogging)
            {
                CurrentPosition = joggingMotionProfiler.Update(CurrentPosition);
                // Calculate IK to get the joint targets for the next Jog movement
                CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                UpdateJointTargets();
                if (_lastClampViolated) CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
            }
            else if (IsJointJogging)
            {
                // Continue Jogging the joints that are moving
                CurrentJointTargets = jointJoggingProfiler.Update(CurrentJointTargets);

                // Update the joint angles with the new calculated ones
                UpdateJointTargets();

                // Recalculate the Cartesian Coordinate position to keep it current
                CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
            }
            else if (IsToolJogging)
            {
                CurrentPosition = toolJoggingMotionProfiler.Update(CurrentPosition);
                // Calculate IK to get the joint targets for the next Jog movement
                CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                UpdateJointTargets();
                if (_lastClampViolated) CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
            }
        }
        public void UpdateJointTargets()
        {
            ApplyJointLimits();
            _kinematics.UpdateMotorTargets(CurrentJointTargets, out double m1Deg, out double m2Deg, out double m3Deg, out double m4Deg);
            stb.SetMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
        }

        // Joint-space windows for joints 0..3 (X / radial / vertical / RZ). An unset
        // bound becomes ±infinity so it never clamps — only the bounds the operator
        // actually set are enforced.
        private (double lo, double hi)[] JointLimitWindows() => new[]
        {
            (_config.Joint1Min ?? double.NegativeInfinity, _config.Joint1Max ?? double.PositiveInfinity),
            (_config.Joint2Min ?? double.NegativeInfinity, _config.Joint2Max ?? double.PositiveInfinity),
            (_config.Joint3Min ?? double.NegativeInfinity, _config.Joint3Max ?? double.PositiveInfinity),
            (_config.Joint4Min ?? double.NegativeInfinity, _config.Joint4Max ?? double.PositiveInfinity),
        };

        /// <summary>
        /// Enforces joint soft limits on <see cref="CurrentJointTargets"/> in place.
        /// Must run on the control-loop thread (called from UpdateJointTargets).
        ///  • Disabled → no-op (and clears any latched fault).
        ///  • Homing → skipped entirely; homing deliberately drives to the limit
        ///    switches, which may sit outside the soft window.
        ///  • Bypass → skipped entirely; the operator has taken responsibility, so a
        ///    joint may be driven past its window in either direction. The fault
        ///    stays latched until cleared.
        ///  • Otherwise → clamp so no joint moves further outside its window. The
        ///    corrective direction never violates, so jogging back into range always
        ///    flows through; only the first crossing latches a fault (and stops the
        ///    motion that caused it), and further pushes into the limit are silently
        ///    clamped without re-latching.
        /// </summary>
        private void ApplyJointLimits()
        {
            _lastClampViolated = false;

            if (!_config.JointLimitsEnabled)
            {
                if (_faulted) ClearFaultInternal();
                return;
            }

            // Homing deliberately drives toward the mechanical limit switches, which
            // may sit outside the soft window — never fault or clamp while homing.
            if (_homing.IsActive)
                return;

            // Bypass overrides the limits entirely: the operator has taken
            // responsibility, so allow driving a joint past its window in either
            // direction. The fault stays latched (the banner remains) until cleared.
            if (_limitBypass)
                return;

            var before = _jointsBeforeTick;
            var result = JointLimiter.Clamp(CurrentJointTargets, before, JointLimitWindows());
            if (result.Violated)
            {
                // Hold the joint at its boundary. Jogging the corrective direction
                // does not violate, so it always flows through — the operator can
                // recover without engaging bypass. Only the first crossing latches
                // (and stops the motion that caused it); further pushes into the
                // limit are silently clamped.
                CurrentJointTargets.Copy(result.Clamped);
                _lastClampViolated = true;
                if (!_faulted)
                    LatchFault(result.Joint, result.Direction);
            }
        }

        // Set by ApplyJointLimits when it clamped/froze this tick — cartesian jog
        // branches re-derive CurrentPosition from the clamped joints so the tool
        // frame can't drift past a blocked joint.
        private bool _lastClampViolated;

        private static readonly string[] AstroJointNames = { "J1", "J2", "J3", "J4" };
        private static readonly string[] CncJointNames   = { "X", "Y", "Z", "RZ" };

        private void LatchFault(int joint, int direction)
        {
            bool wasFaulted = _faulted;
            _faulted = true;
            if (!wasFaulted)
            {
                _faultJoint = joint;
                _faultDirection = direction;
                var names = _config.RobotType == RobotTypes.Cnc4Axis ? CncJointNames : AstroJointNames;
                string name = joint >= 0 && joint < names.Length ? names[joint] : $"joint {joint}";
                string edge = direction > 0 ? "upper" : "lower";
                _faultMessage = $"{name} reached its {edge} limit. Bypass and jog it back into range to recover.";
                Console.WriteLine($"[JointLimit] FAULT — {_faultMessage}");
            }

            // Stop everything: clear profilers, force-stop jogs, halt any program so
            // it cannot keep re-issuing the offending move.
            linearMotionProfiler = null;
            jointMotionProfiler  = null;
            continuousProfiler   = null;
            joggingMotionProfiler.ForceStop();
            jointJoggingProfiler.ForceStop();
            toolJoggingMotionProfiler.ForceStop();
            QueuedCommands.Clear();
            // Executor.Stop() takes the executor lock and may wait on a step doing
            // file I/O; never block the motion thread on it. Motion is already halted.
            var exec = programExecutor;
            if (exec != null) _ = Task.Run(() => exec.Stop());
        }

        // Clears fault state without touching motion — used when limits get disabled
        // or the operator acknowledges the fault.
        private void ClearFaultInternal()
        {
            _faulted = false;
            _limitBypass = false;
            _faultJoint = -1;
            _faultDirection = 0;
            _faultMessage = "";
        }

        /// <summary>Operator acknowledgement: clears the fault and exits bypass. If a
        /// joint is still out of range the next commanded move simply re-faults.</summary>
        public void ClearFault() => PostToMotionThread(ClearFaultInternal);

        /// <summary>Enter/exit limit bypass. While enabled the soft limits are
        /// ignored entirely, so a joint can be jogged past its window in either
        /// direction. Jogging itself is always available during a fault; bypass only
        /// unlocks the worsening direction.</summary>
        public void SetLimitBypass(bool enable) => PostToMotionThread(() => _limitBypass = enable);

        // ── Aux axis motion ───────────────────────────────────────────────────

        /// <summary>
        /// Start an indexed aux move: Arduino runs the trapezoidal profile and reports DONE when complete.
        /// Sign of steps determines direction.
        /// </summary>
        public void StartAuxMove(string deviceId, int axis, long steps, double velocity, double accel, double decel)
        {
            var axisCfg = AuxAxisManager.GetAxisConfig(deviceId, axis);
            bool ccw    = (axisCfg?.InvertDirection ?? false) ? steps > 0 : steps < 0;

            AuxAxisManager.SetDirection(deviceId, axis, ccw);

            _auxActiveDeviceId = deviceId;
            _auxActiveAxis     = axis;
            AuxAxisManager.StartMove(deviceId, axis, Math.Abs(steps),
                (int)Math.Max(1, velocity), (int)Math.Max(1, accel), (int)Math.Max(1, decel));
        }

        /// <summary>
        /// Start continuous aux motion (conveyor): Arduino ramps up to velocity and holds until StopAux().
        /// Positive velocity = CW, negative = CCW.
        /// </summary>
        public void StartAuxContinuous(string deviceId, int axis, double velocity, double accel)
        {
            var axisCfg = AuxAxisManager.GetAxisConfig(deviceId, axis);
            bool ccw    = (axisCfg?.InvertDirection ?? false) ? velocity > 0 : velocity < 0;

            AuxAxisManager.SetDirection(deviceId, axis, ccw);

            _auxActiveDeviceId = deviceId;
            _auxActiveAxis     = axis;
            AuxAxisManager.SetContinuous(deviceId, axis,
                (int)Math.Max(1, Math.Abs(velocity)), (int)Math.Max(1, Math.Abs(accel)));
        }

        /// <summary>Stop the active aux axis. Immediate sends X; otherwise decelerates gracefully.</summary>
        public void StopAux(double decel = 10000, bool immediate = false)
        {
            if (_auxActiveDeviceId is null) return;
            if (immediate)
                AuxAxisManager.StopAll(_auxActiveDeviceId);
            else
                AuxAxisManager.StopSmooth(_auxActiveDeviceId, _auxActiveAxis, (int)Math.Max(1, decel));
        }

        public void SetIdentity(RobotIdentity identity)
        {
            _identity = identity;
        }

        public void SetConfig(RobotConfig config)
        {
            _config = config;
            ApplyMotorDirections();
            InitializeKinematics();
        }

        private void InitializeKinematics()
        {
            if (_config.RobotType == RobotTypes.Cnc4Axis)
            {
                stb.Motor1.Reconfigure(_config.CncStepsPerRevX);
                stb.Motor2.Reconfigure(_config.CncStepsPerRevY);
                stb.Motor3.Reconfigure(_config.CncStepsPerRevZ);
                stb.Motor4.Reconfigure(_config.CncStepsPerRevRZ);

                _kinematics = new CNC4AxisKinematics
                {
                    MotorDegsPerMmX   = _config.CncMmPerRevX  > 0 ? 360.0 / _config.CncMmPerRevX  : 1.0,
                    MotorDegsPerMmY   = _config.CncMmPerRevY  > 0 ? 360.0 / _config.CncMmPerRevY  : 1.0,
                    MotorDegsPerMmZ   = _config.CncMmPerRevZ  > 0 ? 360.0 / _config.CncMmPerRevZ  : 1.0,
                    MotorDegsPerDegRZ = _config.CncDegPerRevRZ > 0 ? 360.0 / _config.CncDegPerRevRZ : 1.0,
                };
            }
            else
            {
                stb.Motor1.Reconfigure(_config.AstroStepsPerRevM1, _config.AstroGearRatioM1);
                stb.Motor2.Reconfigure(_config.AstroStepsPerRevM2, _config.AstroGearRatioM2);
                stb.Motor3.Reconfigure(_config.AstroStepsPerRevM3, _config.AstroGearRatioM3);
                stb.Motor4.Reconfigure(_config.AstroStepsPerRevM4, _config.AstroGearRatioM4);

                _kinematics = new ASTROKinematics(
                    _config.AstroJoint1GearRatio,
                    _config.AstroJoint4GearRatio,
                    _config.AstroCoreXyPulleyPcdMm);
            }
        }

        private void ApplyMotorDirections()
        {
            stb.Motor1.InvertDirection = _config.M1Direction == -1;
            stb.Motor2.InvertDirection = _config.M2Direction == -1;
            stb.Motor3.InvertDirection = _config.M3Direction == -1;
            stb.Motor4.InvertDirection = _config.M4Direction == -1;
        }

        /// <summary>
        /// Handles one WebSocket command: looks the name up in the dispatcher and
        /// runs its handler (see Commands/). Any failure while handling a command
        /// (missing/null params, bad base64, JsonException, …) is reported to the
        /// caller as { ok:false, error } rather than propagating out and tearing
        /// down the client's WebSocket.
        /// </summary>
        public async Task<object> AddCommand(CommandMessage command)
        {
            object? payload;
            try
            {
                payload = _commands.TryGet(command.Command, out var handler)
                    ? await handler(command)
                    : new { ok = false, error = "unknownCommand" };
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[AddCommand] '{command.Command}' failed: {ex}");
                return new { ok = false, error = ex.Message };
            }
            return payload ?? new { };
        }

        // ── Command-handler surface ───────────────────────────────────────────
        // What the handlers in Commands/ need beyond the public API. Reads are
        // snapshots of motion-thread state; every write to motion-owned state is
        // posted to the motion thread.

        internal RobotIdentity     Identity           => _identity;
        internal RobotConfig       Config             => _config;
        internal IRobotKinematics  Kinematics         => _kinematics;
        internal Vector6           LivePosition       => CurrentPosition;
        internal Vector6           LiveTargetPosition => TargetPosition;
        internal bool              Homed              => homed;
        internal string            ActiveToolName     => activeTool;
        internal string            ActiveLocalName    => activeLocal;

        /// <summary>A homing run has been requested or is in progress.</summary>
        internal bool HomingRequestedOrActive => startHoming || _homing.IsActive;

        internal (double SpeedS, double AccelS, double DecelS, double SpeedJ, double AccelJ, double DecelJ) MotionParameters
            => (SpeedS, AccelS, DecelS, SpeedJ, AccelJ, DecelJ);

        internal (bool Faulted, int Joint, int Direction, string Message, bool Bypass) FaultStatus
            => (_faulted, _faultJoint, _faultDirection, _faultMessage, _limitBypass);

        /// <summary>Queues one of <see cref="QueuedMotionCommandNames"/> for RunCommands.</summary>
        internal object? EnqueueMotionCommand(CommandMessage command)
        {
            // Only names RunCommands actually handles may be queued — anything
            // else would sit at the head of the queue as a silent no-op.
            if (command.Command is null || !QueuedMotionCommandNames.Contains(command.Command))
                return new { ok = false, error = "unknownCommand" };

            RobotCommand NewCommand = CommandJson.LoadParams<RobotCommand>(command);
            NewCommand.CommandType = command.Command;
            // Stamp with the current jog epoch so a stop arriving after this
            // enqueue can invalidate a trailing jog (see _jogGeneration).
            NewCommand.JogGeneration = _jogGeneration;
            QueuedCommands.Enqueue(NewCommand);
            return null;
        }

        internal void RequestHome() => PostToMotionThread(() =>
        {
            ClearFaultInternal();  // re-homing re-establishes position; drop any latched fault
            startHoming = true;
        });

        internal void RequestSetHomed() => PostToMotionThread(SetAllHomed);

        internal void StopJog()
        {
            // Bump the epoch first so any jog already queued (but not yet
            // processed on the loop thread) is invalidated and cannot
            // re-enable motion after this stop.
            // The profiler stop itself runs on the motion thread (drained
            // before RunCommands on the next tick).
            Interlocked.Increment(ref _jogGeneration);
            PostToMotionThread(() =>
            {
                joggingMotionProfiler.StopJog();
                jointJoggingProfiler.StopJog();
                toolJoggingMotionProfiler.StopJog();
            });
        }

        /// <summary>Re-applies motor directions and/or rebuilds the kinematics after a
        /// config change — on the motion thread, which drives the STB motors.</summary>
        internal void ReapplyConfigOnMotionThread(bool motorDirections, bool kinematics) => PostToMotionThread(() =>
        {
            if (motorDirections) ApplyMotorDirections();
            if (kinematics)      InitializeKinematics();
        });

        /// <summary>Swaps in a whole new config and re-applies it (motor directions +
        /// kinematics/motor setup) on the motion thread. Used by a full reset-to-defaults.</summary>
        internal void ResetConfigOnMotionThread(RobotConfig newConfig) => PostToMotionThread(() =>
        {
            _config = newConfig;
            ApplyMotorDirections();
            InitializeKinematics();
        });

        internal void SelectTool(string? toolName) => PostToMotionThread(() =>
        {
            // Tool/position state is owned by the motion thread — apply there.
            if (string.IsNullOrEmpty(toolName) || toolName == "None")
            {
                activeTool  = "";
                CurrentTool = Vector6.Zero;
            }
            else
            {
                var tool = toolRepo.Get(toolName);
                if (tool != null)
                {
                    activeTool  = toolName;
                    CurrentTool = new Vector6(tool.X, tool.Y, tool.Z,
                                              tool.RX, tool.RY, tool.RZ);
                }
            }
            // Recalculate position with new tool offset
            CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
            CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
        });

        internal void RenameActiveTool(string oldName, string newName) => PostToMotionThread(() =>
        {
            if (activeTool == oldName) activeTool = newName;
        });

        internal void ForgetDeletedTool(string deletedTool) => PostToMotionThread(() =>
        {
            if (activeTool == deletedTool)
            {
                activeTool          = "";
                CurrentTool         = Vector6.Zero;
                CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
                CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
            }
        });

        internal void SelectLocal(string? localName) => PostToMotionThread(() => ApplyLocal(localName));

        internal void RenameActiveLocal(string oldName, string newName) => PostToMotionThread(() =>
        {
            if (activeLocal == oldName) activeLocal = newName;
        });

        // Motion thread reads CurrentLocal to resolve moves/jogs, so clear it there.
        internal void ForgetDeletedLocal(string deletedLocal) => PostToMotionThread(() =>
        {
            if (activeLocal == deletedLocal)
            {
                activeLocal  = "";
                CurrentLocal = Vector6.Zero;
            }
        });
        public void RunCommands()
        {
            if (_drainQueueRequested)
            {
                QueuedCommands.Clear();
                _drainQueueRequested = false;
                return;
            }

            // Peek at the head without removing it — if IsMoving we return early and
            // the command stays at the head for the next tick.
            if (!QueuedCommands.TryPeek(out RobotCommand? Command) || Command is null)
                return;

            Vector6? target = null;

            string CommandType = Command.CommandType ?? "";

            if (IsMoving && CommandType is not ("JogL" or "JogJ" or "JogTool"))
                return;

            // Apply any status update that was attached to this command at send-time
            if (Command.StatusUpdate != null)
                programManager.ApplyStatusUpdate(Command.StatusUpdate);

            // Mark motion busy the instant we pick up a move — BEFORE it is dequeued
            // below — so the program thread never observes an empty queue together
            // with a stale "not moving" for a move we've just started (completion
            // race across the thread boundary). Reconciled to IsMoving each motion tick.
            if (CommandType is "MoveL" or "OffsetL" or "MoveJ" or "StartContinuous")
                _motionActive = true;

            switch (CommandType)
            {
                case "MoveL":
                    {
                        target = ResolveVector(Command);
                        if (target == null)
                        {
                            // Named point not found — log already emitted; drop the command and
                            // latch the error so the program executor fails the step.
                            LatchMotionError($"Point '{Command.Name}' not found");
                            break;
                        }
                        MoveL(target, Command.Speed, Command.Accel, Command.Decel, Command.ToolOffsetVector6, Command.ApplySpeedOverride);
                    }
                    break;

                case "OffsetL":
                    Vector6 NewPosition = CurrentPosition + Command.Vector6;
                    MoveL(NewPosition, Command.Speed, Command.Accel, Command.Decel, Command.ToolOffsetVector6, Command.ApplySpeedOverride);
                    break;

                case "MoveJ":
                    {
                        target = ResolveVector(Command);
                        if (target == null)
                        {
                            // Named point not found — log already emitted; drop the command and
                            // latch the error so the program executor fails the step.
                            LatchMotionError($"Point '{Command.Name}' not found");
                            break;
                        }
                        MoveJ(target, Command.Speed, Command.Accel, Command.Decel, Command.ToolOffsetVector6, Command.ApplySpeedOverride);
                    }
                    break;

                case "StartContinuous":
                    // Blended path — started here on the motion thread rather than the
                    // program thread mutating continuousProfiler directly.
                    if (Command.Waypoints is { Count: >= 2 })
                        StartContinuousMove(Command.Waypoints, Command.BlendRadii ?? new List<double>(),
                            Command.Speed, Command.Accel, Command.Decel, Command.ApplySpeedOverride);
                    break;

                case "SetTool":
                    this.CurrentTool    = Command.Vector6;
                    CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
                    CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                    break;

                case "SpeedS":
                    this.SpeedS = Math.Max(1.0, Command.Speed ?? this.SpeedS);
                    break;

                case "AccelS":
                    this.AccelS = Command.Accel ??= this.AccelS;
                    this.DecelS = Command.Decel ??= this.DecelS;
                    break;

                case "SpeedJ":
                    this.SpeedJ = Math.Max(1.0, Command.Speed ?? this.SpeedJ);
                    break;

                case "AccelJ":
                    this.AccelJ = Command.Accel ?? this.AccelJ;
                    this.DecelJ = Command.Decel ?? this.DecelJ;
                    break;

                case "JogL":
                    // Drop a jog superseded by a later StopJog — see _jogGeneration.
                    if (Command.JogGeneration == _jogGeneration)
                        JogL(Command.Vector6, Command.Speed, Command.Accel, Command.Decel);
                    break;

                case "JogJ":
                    if (Command.JogGeneration == _jogGeneration)
                        JogJ(Command.Vector6, Command.Speed, Command.Accel, Command.Decel);
                    break;

                case "JogTool":
                    if (Command.JogGeneration == _jogGeneration)
                        JogTool(Command.Vector6, Command.Speed, Command.Accel, Command.Decel);
                    break;

                default:
                    break;
            }

            // The command was processed — consume it from the head of the queue.
            // TryDequeue is safe here: only the loop thread ever dequeues, so the
            // item peeked above is guaranteed to still be at the head.
            QueuedCommands.TryDequeue(out _);
        }

        public void SetAllHomed()
        {
            double m1Deg, m2Deg, m3Deg, m4Deg;

            if (_kinematics is ASTROKinematics astro)
            {
                astro.InterpolatedJoint1.JointAngleDeg = _config.J1HomeOffsetDeg;
                astro.CurrentJoint1.JointAngleDeg      = _config.J1HomeOffsetDeg;
                astro.InterpolatedJoint2.Cartesian     = (_config.HorizontalHomePosition, astro.InterpolatedJoint2.Cartesian.z);
                astro.CurrentJoint2.Cartesian          = (_config.HorizontalHomePosition, astro.CurrentJoint2.Cartesian.z);
                astro.InterpolatedJoint2.Cartesian     = (astro.InterpolatedJoint2.Cartesian.x, _config.VerticalHomePosition);
                astro.CurrentJoint2.Cartesian          = (astro.CurrentJoint2.Cartesian.x,      _config.VerticalHomePosition);
                astro.InterpolatedJoint4.JointAngleDeg = _config.J4HomeOffsetDeg;
                astro.CurrentJoint4.JointAngleDeg      = _config.J4HomeOffsetDeg;

                CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
                CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
            }
            else
            {
                // CNC4Axis: apply configured home positions directly as joint targets
                CurrentJointTargets = new Vector6(
                    _config.CncXHomePosition,
                    _config.CncYHomePosition,
                    _config.CncZHomePosition,
                    0, 0,
                    _config.CncRzHomePosition
                );
                _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
                CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
            }

            stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
        }
        // ── Homing ────────────────────────────────────────────────────────────
        // Motion thread only. One sequencer phase per tick — see HomingSequencer.
        public void RunHoming()
        {
            switch (_homing.Phase)
            {
                case HomingPhase.WaitingForStart:
                    if (startHoming)
                        _homing.Begin(_kinematics is CNC4AxisKinematics ? CncHomingAxes() : AstroHomingAxes());
                    break;

                case HomingPhase.Complete:
                    startHoming = false;
                    _homing.Reset();
                    homed = true;
                    break;

                default:
                    _homing.Tick();
                    break;
            }
        }

        // ASTRO: vertical (Input2) → horizontal (Input3) → J1 (Input1) → drive J4 to 0°.
        // Speeds, directions and back-off are captured when homing starts; the
        // home positions are read from the config when each axis is set homed.
        private List<HomingAxis> AstroHomingAxes()
        {
            var c = _config;
            return new List<HomingAxis>
            {
                new(Joint: 2, c.VerticalHomingDirection, () => stb.Input2,
                    c.HomingSpeed, c.HomingSlowSpeed, c.HomingBackoffMm,
                    SetAstroVerticalHomed, HomingAxis.AstroNames("Vertical")),
                new(Joint: 1, c.HorizontalHomingDirection, () => stb.Input3,
                    c.HomingSpeed, c.HomingSlowSpeed, c.HomingBackoffMm,
                    SetAstroHorizontalHomed, HomingAxis.AstroNames("Horizontal")),
                new(Joint: 0, c.J1HomingDirection, () => stb.Input1,
                    c.HomingSpeed, c.HomingSlowSpeed, c.HomingBackoffMm,
                    SetAstroJ1Homed, HomingAxis.AstroNames("J1", setHomedName: "SetJ1MotorHomed")),
                // J4 has no switch: drive it to 0° (mechanical zero) with a joint
                // move — J1/J2/J3 stay put — then declare the J4 home offset there.
                new(Joint: 5, Direction: 0, Sensor: null,
                    c.HomingSpeed, c.HomingSlowSpeed, c.HomingBackoffMm,
                    SetAstroJ4Homed,
                    HomingAxis.Names(driveToZero: "HomeJ4", waitMove: "WaitJ4MoveComplete", setHomed: "SetJ4Homed"),
                    DriveToZero: true),
            };
        }

        // CNC4Axis: Z (Input3) → X (Input1) → Y (Input2) → zero RZ.
        private List<HomingAxis> CncHomingAxes()
        {
            var c = _config;
            return new List<HomingAxis>
            {
                new(Joint: 2, c.CncZHomingDirection, () => stb.Input3,
                    c.HomingSpeed, c.HomingSlowSpeed, c.HomingBackoffMm,
                    () => SetCncJointHomed(2, _config.CncZHomePosition), HomingAxis.CncNames("Z")),
                new(Joint: 0, c.CncXHomingDirection, () => stb.Input1,
                    c.HomingSpeed, c.HomingSlowSpeed, c.HomingBackoffMm,
                    () => SetCncJointHomed(0, _config.CncXHomePosition), HomingAxis.CncNames("X")),
                new(Joint: 1, c.CncYHomingDirection, () => stb.Input2,
                    c.HomingSpeed, c.HomingSlowSpeed, c.HomingBackoffMm,
                    () => SetCncJointHomed(1, _config.CncYHomePosition), HomingAxis.CncNames("Y")),
                // RZ (threading spindle) has no limit switch — zero it at its current position.
                new(Joint: 5, Direction: 0, Sensor: null,
                    c.HomingSpeed, c.HomingSlowSpeed, c.HomingBackoffMm,
                    () => SetCncJointHomed(5, _config.CncRzHomePosition),
                    HomingAxis.Names(setHomed: "CNC_ZeroRZ")),
            };
        }

        private ASTROKinematics Astro => (ASTROKinematics)_kinematics;

        private void SetAstroVerticalHomed()
        {
            var astro = Astro;
            astro.InterpolatedJoint2.Cartesian = (astro.InterpolatedJoint2.Cartesian.x, _config.VerticalHomePosition);
            astro.CurrentJoint2.Cartesian      = (astro.CurrentJoint2.Cartesian.x,      _config.VerticalHomePosition);
            CommitAstroHomedJoints();
        }

        private void SetAstroHorizontalHomed()
        {
            var astro = Astro;
            astro.InterpolatedJoint2.Cartesian = (_config.HorizontalHomePosition, astro.InterpolatedJoint2.Cartesian.z);
            astro.CurrentJoint2.Cartesian      = (_config.HorizontalHomePosition, astro.CurrentJoint2.Cartesian.z);
            CommitAstroHomedJoints();
        }

        private void SetAstroJ1Homed()
        {
            var astro = Astro;
            astro.InterpolatedJoint1.JointAngleDeg = _config.J1HomeOffsetDeg;
            astro.CurrentJoint1.JointAngleDeg      = _config.J1HomeOffsetDeg;
            CommitAstroHomedJoints();
        }

        private void SetAstroJ4Homed()
        {
            var astro = Astro;
            astro.InterpolatedJoint4.JointAngleDeg = _config.J4HomeOffsetDeg;
            astro.CurrentJoint4.JointAngleDeg      = _config.J4HomeOffsetDeg;
            CommitAstroHomedJoints();
        }

        // ASTRO: the joint state was written into the kinematics — derive the pose and
        // joint targets from it and overwrite the STB's motor positions (no motion).
        private void CommitAstroHomedJoints()
        {
            CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
            CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
            _kinematics.UpdateMotorTargets(CurrentJointTargets, out double m1Deg, out double m2Deg, out double m3Deg, out double m4Deg);
            stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
        }

        // CNC: declare one joint target homed, overwrite the STB's motor positions
        // (no motion), then refresh the pose.
        private void SetCncJointHomed(int joint, double homePosition)
        {
            switch (joint)
            {
                case 0: CurrentJointTargets.X  = homePosition; break;
                case 1: CurrentJointTargets.Y  = homePosition; break;
                case 2: CurrentJointTargets.Z  = homePosition; break;
                case 5: CurrentJointTargets.RZ = homePosition; break;
                default: throw new ArgumentOutOfRangeException(nameof(joint));
            }
            _kinematics.UpdateMotorTargets(CurrentJointTargets, out double m1Deg, out double m2Deg, out double m3Deg, out double m4Deg);
            stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
            CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
        }

        // IHomingHost — called by the sequencer on the motion thread.
        Vector6 IHomingHost.JointTargets => CurrentJointTargets;

        void IHomingHost.JogJoints(Vector6 direction, double speed, double accel, double decel, double watchdogSeconds)
            => jointJoggingProfiler.Jog(direction, speed, accel, decel, watchdogSeconds);

        void IHomingHost.MoveJoints(Vector6 target, double speed, double accel, double decel)
        {
            // TargetJoints must be set before creating the profiler — RunMotion snaps
            // CurrentJointTargets to it on the tick the profiler finishes.
            this.TargetJoints   = target;
            jointMotionProfiler = new(CurrentJointTargets, target, speed, accel, decel);
        }

        void IHomingHost.StopAtSwitch() => ExecuteHardStop();

        /// <summary>
        /// Thread-safe: sets a flag that is consumed at the top of the next control loop iteration.
        /// Never touches the profilers directly from outside the loop thread.
        /// </summary>
        public void HardStop()
        {
            _hardStopRequested = true;
        }

        /// <summary>
        /// Thread-safe: queues a QueuedCommands.Clear() to run on the control loop thread.
        /// Direct List mutation from outside the loop thread would race with RunCommands().
        /// </summary>
        public void RequestQueueDrain()
        {
            _drainQueueRequested = true;
        }

        /// <summary>
        /// If a different built program is currently running in the executor, stops it and
        /// resets its status to Ready so the UI returns to the Start button state.
        /// Call this before starting a new built program.
        /// </summary>
        internal void DisplaceRunningBuiltProgram(string incomingProgramName)
        {
            // Stop active execution if a different built program is running
            var currentName = programExecutor?.CurrentProgramName;
            if (currentName != null && currentName != incomingProgramName && programExecutor?.IsRunning == true)
            {
                programExecutor.Stop();
                var currentBuilt = builtProgramRepo.Get(currentName);
                if (currentBuilt != null)
                    programManager.ResetToReady(currentName, ProgramExecutor.CountSteps(currentBuilt.Steps));
            }

            // Reset any other built programs that are Stopped or Complete back to Ready
            var others = builtProgramRepo.GetAll()
                .Where(bp => bp.Name != incomingProgramName)
                .Select(bp => (bp.Name, ProgramExecutor.CountSteps(bp.Steps)));
            programManager.ResetTerminatedToReady(others);
        }

        /// <summary>
        /// Must only be called from the control loop thread.
        /// Clears all motion state immediately and safely.
        /// </summary>
        private void ExecuteHardStop()
        {
            // In-memory state first — this cannot fail, so the arm is always stopped
            // even if the device I/O below throws.
            ClearMotionStateForHardStop();

            // Device I/O (serial) can throw; never let it escape the motion thread.
            try
            {
                AuxAxisManager.StopAllDevices();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[HardStop] Aux device stop failed: {ex}");
            }
        }

        /// <summary>
        /// Must only be called from the control loop thread. Clears profilers,
        /// jogs, the command queue and homing state — no device I/O.
        /// </summary>
        private void ClearMotionStateForHardStop()
        {
            _hardStopRequested = false;
            linearMotionProfiler = null;
            jointMotionProfiler = null;
            continuousProfiler = null;
            joggingMotionProfiler.ForceStop();
            jointJoggingProfiler.ForceStop();
            toolJoggingMotionProfiler.ForceStop();
            QueuedCommands.Clear();
            startHoming = false;
            _homing.Reset();
        }

        public void MoveJ(Vector6 TargetPosition, double? Speed, double? Accel, double? Decel, Vector6? ToolOffset, bool applyOverride = false)
        {
            if (IsMoving)
                return;
            if (_faulted)  // no automatic/point moves while a joint-limit fault is latched
                return;

            // Gather the commands motion params if there specified otherwise default to the last set ones.
            // The global speed override scales program moves only (applyOverride) — including
            // program steps that don't set an explicit speed. Manual point moves and jogging
            // run at their commanded speed.
            double jointSpeed = (Speed ?? this.SpeedJ) * (applyOverride ? SpeedOverrideFactor : 1.0);
            double jointAccel = Accel ??= this.AccelJ;
            double jointDecel = Decel ??= this.DecelJ;

            if (ToolOffset is not null)
            {
                this.TargetPosition = ApplyToolOffset(TargetPosition, ToolOffset);
            }
            else
            {
                // Copy the Command Position to the Target Position
                this.TargetPosition = TargetPosition;
            }

            // Calculate the joint positions for the target position and the current tooling
            this.TargetJoints = _kinematics.InverseKinematics(this.TargetPosition, this.CurrentTool);

            // Generate a joint motion profile using the current and target joint positions
            jointMotionProfiler = new(CurrentJointTargets, this.TargetJoints, jointSpeed, jointAccel, jointDecel);
        }

        

        public void MoveL(Vector6 TargetPosition, double? Speed, double? Accel, double? Decel, Vector6? ToolOffset, bool applyOverride = false)
        {
            if (IsMoving)
                return;
            if (_faulted)  // no automatic/point moves while a joint-limit fault is latched
                return;

            // Gather the commands motion params if there specified otherwise default to the last set ones.
            // Override scales program moves only (see MoveJ); manual/jog moves are unaffected.
            double lineSpeed = (Speed ?? this.SpeedS) * (applyOverride ? SpeedOverrideFactor : 1.0);
            double lineAccel = Accel ??= this.AccelS;
            double lineDecel = Decel ??= this.DecelS;

            if (ToolOffset is not null)
            {
                this.TargetPosition = ApplyToolOffset(TargetPosition, ToolOffset);
            }
            else
            {
                // Copy the Command Position to the Target Position
                this.TargetPosition = TargetPosition;
            }

            // Generate a new linear motion profiler for this move
            linearMotionProfiler = new(CurrentPosition, this.TargetPosition, lineSpeed, lineAccel, lineDecel);
        }

        /// <summary>
        /// Start a continuous (blended) linear path through a list of waypoints. Each
        /// interior waypoint is rounded by its blend radius (blendRadii[i] applies at
        /// waypoints[i]); the final waypoint is an exact stop. The path is driven by a
        /// single trapezoidal speed profile over its total blended length.
        /// </summary>
        public void StartContinuousMove(List<Vector6> waypoints, List<double> blendRadii,
            double? Speed, double? Accel, double? Decel, bool applyOverride = false)
        {
            if (IsMoving) return;
            if (_faulted) return;  // no automatic moves while a joint-limit fault is latched
            if (waypoints == null || waypoints.Count < 2) return;

            double lineSpeed = (Speed ?? this.SpeedS) * (applyOverride ? SpeedOverrideFactor : 1.0);
            double lineAccel = Accel ?? this.AccelS;
            double lineDecel = Decel ?? this.DecelS;

            // Prepend the current position so the path starts from where the robot is.
            var pts = new List<Vector6>(waypoints.Count + 1) { GetCurrentPosition() };
            pts.AddRange(waypoints);

            // Align radii to the point list: index 0 (current pos) has no corner.
            var radii = new List<double>(pts.Count) { 0 };
            radii.AddRange(blendRadii);

            this.TargetPosition = waypoints[^1];
            continuousProfiler = new(pts, radii, lineSpeed, lineAccel, lineDecel);
        }

        public void JogJ(Vector6 jogJointDirection, double? Speed, double? Accel, double? Decel)
        {
            double jointSpeed = Speed ??= this.SpeedJ;
            double jointAccel = Accel ??= this.AccelJ;
            double jointDecel = Decel ??= this.DecelJ;
            if (_config.RobotType == RobotTypes.Cnc4Axis) jointSpeed /= CncJogSpeedDivisor;
            jointJoggingProfiler.Jog(jogJointDirection, jointSpeed, jointAccel, jointDecel);
        }

        public void JogL(Vector6 jogDirection, double? Speed, double? Accel, double? Decel)
        {
            double lineSpeed = Speed ??= this.SpeedS;
            double lineAccel = Accel ??= this.AccelS;
            double lineDecel = Decel ??= this.DecelS;
            if (_config.RobotType == RobotTypes.Cnc4Axis) lineSpeed /= CncJogSpeedDivisor;
            // Jog along the active local's axes: rotate the linear direction into
            // world space so "+X" tracks the local frame, not the world frame.
            if (ActiveLocalOffset is { } loc)
                jogDirection = LocalFrame.Rotate(loc, jogDirection);
            joggingMotionProfiler.Jog(jogDirection, lineSpeed, lineAccel, lineDecel);
        }

        public void JogTool(Vector6 jogDirection, double? Speed, double? Accel, double? Decel)
        {
            double lineSpeed = Speed ??= this.SpeedS;
            double lineAccel = Accel ??= this.AccelS;
            double lineDecel = Decel ??= this.DecelS;
            if (_config.RobotType == RobotTypes.Cnc4Axis) lineSpeed /= CncJogSpeedDivisor;
            toolJoggingMotionProfiler.Jog(jogDirection, lineSpeed, lineAccel, lineDecel);
        }

        /// <summary>
        /// Offsets a pose in its own tool frame by the given offset Vector6.
        /// The linear offset (X/Y/Z) is rotated into world space using the pose's current orientation,
        /// then added to the pose's position. The rotation offset (RX/RY/RZ) is applied after translation.
        /// </summary>
        public static Vector6 ApplyToolOffset(Vector6 pose, Vector6 offset)
        {
            float rx = (float)(pose.RX * Math.PI / 180.0);
            float ry = (float)(pose.RY * Math.PI / 180.0);
            float rz = (float)(pose.RZ * Math.PI / 180.0);

            Matrix4x4 rot =
                Matrix4x4.CreateRotationZ(rz) *
                Matrix4x4.CreateRotationY(ry) *
                Matrix4x4.CreateRotationX(rx);

            Vector3 worldOffset = Vector3.Transform(
                new Vector3((float)offset.X, (float)offset.Y, (float)offset.Z),
                rot
            );

            return new Vector6(
                pose.X  + worldOffset.X,
                pose.Y  + worldOffset.Y,
                pose.Z  + worldOffset.Z,
                pose.RX + offset.RX,
                pose.RY + offset.RY,
                pose.RZ + offset.RZ
            );
        }

        private Vector6? ResolveVector(RobotCommand command)
        {
            if (!string.IsNullOrWhiteSpace(command.Name))
            {
                // pointRepo.Points returns a snapshot copy (thread-safe), so TryGetValue is safe here
                if (!pointRepo.Points.TryGetValue(command.Name, out var point))
                {
                    Console.WriteLine($"[RunCommands] Point '{command.Name}' not found — dropping command");
                    return null;
                }

                // Saved points are base-frame — the active local frame transforms
                // them (rotation + translation), the same way program moves do.
                // Raw-vector commands are already fully resolved by their sender
                // and pass through untouched.
                var p   = new Vector6(point.X, point.Y, point.Z, point.RX, point.RY, point.RZ);
                var loc = ActiveLocalOffset;
                return loc == null ? p : LocalFrame.Apply(loc, p);
            }

            return command.Vector6;
        }
        
    }
}
