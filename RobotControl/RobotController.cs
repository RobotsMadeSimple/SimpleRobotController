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
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl
{
    internal class RobotController
    {
        public PointRepository       pointRepo       = new();
        public ToolRepository        toolRepo        = new();
        public LocalRepository       localRepo       = new();
        public BuiltProgramRepository builtProgramRepo = new();
        public GridRepository  gridRepo  = new();
        public StackRepository stackRepo = new();
        public STB4100 stb = new();
        private RobotIdentity _identity = new();
        public Action<RobotIdentity>? OnIdentityChanged;
        public ScalarMotionProfiler mp = new();
        private IRobotKinematics _kinematics = new ASTROKinematics();
        private readonly ProgramCycleManager programManager = new();
        private ProgramExecutor? programExecutor;
        private BackgroundProgramManager backgroundProgramManager = null!;

        // Shared deserialisation options — handles string enums and camelCase from the client
        private static readonly JsonSerializerOptions _jsonOptions = new()
        {
            Converters = { new JsonStringEnumConverter() },
            PropertyNameCaseInsensitive = true
        };

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
        private int    _faultJoint = -1;     // 0..3 joint index, -1 = none
        private int    _faultDirection;      // +1 past max, -1 past min (the unsafe direction)
        private string _faultMessage = "";
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
        public string HomingState => homingState;
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

        // Robot configuration (homing offsets, speeds, etc.)
        private RobotConfig _config = new();

        // If the Robot was homed from startup
        private bool homed = false;
        private bool startHoming = false;
        private String homingState = "WaitingForStart";

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

        // Speed override: 0.05–2.0 (5%–200%), default 1.0 (100%)
        public double SpeedOverrideFactor { get; private set; } = 1.0;

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
            NanoManager = new NanoManager("nano_config.json");
            NanoManager.Start();

            RelayManager = new UsbRelayManager();
            RelayManager.Start();

            AuxAxisManager = new AuxAxisManager("aux_config.json");
            AuxAxisManager.Start();

            CameraManager = new Camera.CameraManager("camera_config.json");
            CameraManager.Start();

            VisionRepo    = new Vision.VisionProgramRepository("vision_programs");
            VisionManager = new Vision.VisionManager(CameraManager, VisionRepo);

            stb.Start();


            backgroundProgramManager = new BackgroundProgramManager(
                this, programManager, pointRepo, toolRepo, localRepo, builtProgramRepo, gridRepo, stackRepo);

            programExecutor = new ProgramExecutor(
                this, programManager, pointRepo, toolRepo, localRepo, builtProgramRepo, gridRepo, stackRepo,
                isBackground: false, globalVars: backgroundProgramManager.GlobalVars, backgroundManager: backgroundProgramManager);

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
                    // [diag] phase timing — logs only when a motion tick stalls.
                    Diag.LoopSw.Restart();
                    int gc0 = GC.CollectionCount(0);
                    int gc2 = GC.CollectionCount(2);

                    // Consume hard-stop flag before anything else touches the profilers
                    if (_hardStopRequested)
                        ExecuteHardStop();

                    // Execute pending robot commands (creates/updates profilers)
                    RunCommands();
                    double tCmds = Diag.LoopSw.Elapsed.TotalMilliseconds;

                    // Advance the active motion profile toward the target
                    RunMotion();
                    double tMotion = Diag.LoopSw.Elapsed.TotalMilliseconds;

                    // Execute Homing
                    RunHoming();
                    double tHoming = Diag.LoopSw.Elapsed.TotalMilliseconds;

                    // Let the stepper motor drive toward the new targets, and publish
                    // the motion-busy signal the program thread polls for completion.
                    stb.moving   = IsMoving;
                    _motionActive = IsMoving;

                    if (tHoming > Diag.SlowTickMs)
                        Diag.Log($"motion-cycle {tHoming:F1}ms | cmds={tCmds:F1} " +
                                 $"motion={tMotion - tCmds:F1} homing={tHoming - tMotion:F1} " +
                                 $"| gc0={GC.CollectionCount(0) - gc0} gc2={GC.CollectionCount(2) - gc2}");
                    Diag.Tick(tHoming);
                }
                catch (Exception ex)
                {
                    // Catch-all: log, hard-stop the robot, then keep looping.
                    // The process must survive any tick-level exception.
                    Console.WriteLine($"[MotionLoop] Unhandled exception on tick: {ex}");
                    ExecuteHardStop();
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
                    long ts = Diag.Now();
                    programExecutor?.Update();
                    backgroundProgramManager.Update();
                    double progMs = Diag.MsBetween(ts);
                    if (progMs > Diag.SlowStepMs)
                        Diag.Log($"prog-cycle {progMs:F1}ms (executor thread — motion NOT affected)");

                    long nowMs = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                    if (nowMs - _lastStatusLightMs >= 500)
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
            else if (startHoming || homingState != "WaitingForStart")
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
            if (homingState != "WaitingForStart")
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
                var names = _config.RobotType == "CNC4Axis" ? CncJointNames : AstroJointNames;
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
            programExecutor?.Stop();
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
        public void ClearFault() => ClearFaultInternal();

        /// <summary>Enter/exit limit bypass. While enabled the soft limits are
        /// ignored entirely, so a joint can be jogged past its window in either
        /// direction. Jogging itself is always available during a fault; bypass only
        /// unlocks the worsening direction.</summary>
        public void SetLimitBypass(bool enable) => _limitBypass = enable;

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
            if (_config.RobotType == "CNC4Axis")
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
                _kinematics = new ASTROKinematics();
            }
        }

        private void ApplyMotorDirections()
        {
            stb.Motor1.InvertDirection = _config.M1Direction == -1;
            stb.Motor2.InvertDirection = _config.M2Direction == -1;
            stb.Motor3.InvertDirection = _config.M3Direction == -1;
            stb.Motor4.InvertDirection = _config.M4Direction == -1;
        }

        public async Task<object> AddCommand(CommandMessage command)
        {
            object? payload = null;

            switch (command.Command)
            {
                case "GetRobotInfo":
                    payload = new
                    {
                        robotName    = _identity.RobotName,
                        robotType    = _identity.RobotType,
                        serialNumber = _identity.SerialNumber,
                    };
                    break;

                case "Update":
                    {
                        if (!System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.Linux))
                        {
                            payload = new { ok = false, error = "Update is only supported on Linux." };
                            break;
                        }

                        _ = Task.Run(async () =>
                        {
                            try
                            {
                                Console.WriteLine("[Update] Fetching latest release from GitHub…");
                                using var http = new System.Net.Http.HttpClient();
                                http.DefaultRequestHeaders.Add("User-Agent", "SimpleRobotController");

                                var json = await http.GetStringAsync(
                                    "https://api.github.com/repos/RobotsMadeSimple/SimpleRobotController/releases/latest");
                                using var doc = JsonDocument.Parse(json);

                                string? downloadUrl = null;
                                foreach (var asset in doc.RootElement.GetProperty("assets").EnumerateArray())
                                {
                                    if (asset.GetProperty("name").GetString() == "SimpleRobotController")
                                    {
                                        downloadUrl = asset.GetProperty("browser_download_url").GetString();
                                        break;
                                    }
                                }

                                if (downloadUrl == null)
                                {
                                    Console.WriteLine("[Update] Linux binary not found in latest release.");
                                    return;
                                }

                                var exePath = Environment.ProcessPath
                                    ?? System.Diagnostics.Process.GetCurrentProcess().MainModule!.FileName;
                                var tempPath = exePath + ".update";

                                Console.WriteLine($"[Update] Downloading {downloadUrl}…");
                                var bytes = await http.GetByteArrayAsync(downloadUrl);
                                await File.WriteAllBytesAsync(tempPath, bytes);

                                File.SetUnixFileMode(tempPath,
                                    System.IO.UnixFileMode.UserRead   | System.IO.UnixFileMode.UserWrite  | System.IO.UnixFileMode.UserExecute |
                                    System.IO.UnixFileMode.GroupRead  | System.IO.UnixFileMode.GroupExecute |
                                    System.IO.UnixFileMode.OtherRead  | System.IO.UnixFileMode.OtherExecute);

                                File.Move(tempPath, exePath, overwrite: true);
                                Console.WriteLine("[Update] Binary replaced. Exiting for systemd restart…");
                                await Task.Delay(500);
                                Environment.Exit(0);
                            }
                            catch (Exception ex)
                            {
                                Console.WriteLine($"[Update] Failed: {ex.Message}");
                            }
                        });

                        payload = new { ok = true };
                        break;
                    }

                case "RestartController":
                    _ = Task.Run(async () =>
                    {
                        await Task.Delay(500);
                        try
                        {
                            // On a dev machine the project source sits three levels above the
                            // build output (…/RobotControl/bin/<Config>/net10.0/). When it's
                            // present, rebuild the latest code and relaunch the fresh binary
                            // instead of re-running the stale one. In production (published,
                            // no .csproj) we just relaunch the current binary as before.
                            var baseDir    = AppContext.BaseDirectory;
                            var projectDir = Path.GetFullPath(Path.Combine(baseDir, "..", "..", ".."));
                            var csproj     = Directory.Exists(projectDir)
                                ? Directory.GetFiles(projectDir, "*.csproj").FirstOrDefault()
                                : null;
                            var exePath = Environment.ProcessPath;

                            if (csproj != null && exePath != null)
                            {
                                var sep    = Path.DirectorySeparatorChar;
                                var config = baseDir.Contains($"{sep}Release{sep}") ? "Release" : "Debug";
                                var psi = new System.Diagnostics.ProcessStartInfo
                                {
                                    UseShellExecute  = true,
                                    WorkingDirectory = projectDir,
                                };
                                if (OperatingSystem.IsWindows())
                                {
                                    // Wait for this process to release its own binary, rebuild, then relaunch.
                                    psi.FileName  = "cmd.exe";
                                    psi.Arguments = $"/c timeout /t 2 /nobreak >nul & dotnet build \"{csproj}\" -c {config} --nologo && start \"\" \"{exePath}\"";
                                }
                                else
                                {
                                    psi.FileName  = "/bin/bash";
                                    psi.Arguments = $"-c \"sleep 2 && dotnet build '{csproj}' -c {config} --nologo && nohup '{exePath}' >/dev/null 2>&1 &\"";
                                }
                                System.Diagnostics.Process.Start(psi);
                            }
                            else if (exePath != null)
                            {
                                System.Diagnostics.Process.Start(exePath);
                            }
                        }
                        catch (Exception ex)
                        {
                            Console.WriteLine($"[Restart] Failed: {ex.Message}");
                        }
                        Environment.Exit(0);
                    });
                    break;

                case "SetRobotIdentity":
                {
                    var p = JsonSerializer.Deserialize<SetRobotIdentityParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    if (p.RobotName != null) _identity.RobotName = p.RobotName;
                    if (p.RobotType != null) _identity.RobotType = p.RobotType;
                    RobotIdentityService.Save(_identity);
                    OnIdentityChanged?.Invoke(_identity);
                    break;
                }

                case "GetRobotConfig":
                    payload = new
                    {
                        robotType                 = _config.RobotType,
                        homingSpeed               = _config.HomingSpeed,
                        homingSlowSpeed           = _config.HomingSlowSpeed,
                        homingBackoffMm           = _config.HomingBackoffMm,
                        j1HomeOffsetDeg           = _config.J1HomeOffsetDeg,
                        verticalHomePosition      = _config.VerticalHomePosition,
                        horizontalHomePosition    = _config.HorizontalHomePosition,
                        verticalHomingDirection   = _config.VerticalHomingDirection,
                        horizontalHomingDirection = _config.HorizontalHomingDirection,
                        j1HomingDirection         = _config.J1HomingDirection,
                        j4HomeOffsetDeg           = _config.J4HomeOffsetDeg,
                        m1Direction               = _config.M1Direction,
                        m2Direction               = _config.M2Direction,
                        m3Direction               = _config.M3Direction,
                        m4Direction               = _config.M4Direction,
                        enableNanoCards           = _config.EnableNanoCards,
                        enableRelayCard           = _config.EnableRelayCard,
                        enableAuxAxis             = _config.EnableAuxAxis,
                        enableCameras             = _config.EnableCameras,
                        jogSlowSpeed              = _config.JogSlowSpeed,
                        jogNormalSpeed            = _config.JogNormalSpeed,
                        jogFastSpeed              = _config.JogFastSpeed,
                        cncStepsPerRevX           = _config.CncStepsPerRevX,
                        cncStepsPerRevY           = _config.CncStepsPerRevY,
                        cncStepsPerRevZ           = _config.CncStepsPerRevZ,
                        cncStepsPerRevRZ          = _config.CncStepsPerRevRZ,
                        cncMmPerRevX              = _config.CncMmPerRevX,
                        cncMmPerRevY              = _config.CncMmPerRevY,
                        cncMmPerRevZ              = _config.CncMmPerRevZ,
                        cncDegPerRevRZ            = _config.CncDegPerRevRZ,
                        cncXHomePosition          = _config.CncXHomePosition,
                        cncYHomePosition          = _config.CncYHomePosition,
                        cncZHomePosition          = _config.CncZHomePosition,
                        cncRzHomePosition         = _config.CncRzHomePosition,
                        cncXHomingDirection       = _config.CncXHomingDirection,
                        cncYHomingDirection       = _config.CncYHomingDirection,
                        cncZHomingDirection       = _config.CncZHomingDirection,
                        jointLimitsEnabled        = _config.JointLimitsEnabled,
                        joint1Min                 = _config.Joint1Min,
                        joint1Max                 = _config.Joint1Max,
                        joint2Min                 = _config.Joint2Min,
                        joint2Max                 = _config.Joint2Max,
                        joint3Min                 = _config.Joint3Min,
                        joint3Max                 = _config.Joint3Max,
                        joint4Min                 = _config.Joint4Min,
                        joint4Max                 = _config.Joint4Max,
                    };
                    break;

                case "SetRobotConfig":
                {
                    var p = JsonSerializer.Deserialize<SetRobotConfigParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    if (p.HomingSpeed.HasValue)               _config.HomingSpeed               = p.HomingSpeed.Value;
                    if (p.HomingSlowSpeed.HasValue)           _config.HomingSlowSpeed           = p.HomingSlowSpeed.Value;
                    if (p.HomingBackoffMm.HasValue)           _config.HomingBackoffMm           = p.HomingBackoffMm.Value;
                    if (p.J1HomeOffsetDeg.HasValue)           _config.J1HomeOffsetDeg           = p.J1HomeOffsetDeg.Value;
                    if (p.VerticalHomePosition.HasValue)      _config.VerticalHomePosition      = p.VerticalHomePosition.Value;
                    if (p.HorizontalHomePosition.HasValue)    _config.HorizontalHomePosition    = p.HorizontalHomePosition.Value;
                    if (p.VerticalHomingDirection.HasValue)   _config.VerticalHomingDirection   = p.VerticalHomingDirection.Value;
                    if (p.HorizontalHomingDirection.HasValue) _config.HorizontalHomingDirection = p.HorizontalHomingDirection.Value;
                    if (p.J1HomingDirection.HasValue)         _config.J1HomingDirection         = p.J1HomingDirection.Value;
                    if (p.J4HomeOffsetDeg.HasValue)           _config.J4HomeOffsetDeg           = p.J4HomeOffsetDeg.Value;
                    if (p.M1Direction.HasValue)               { _config.M1Direction             = p.M1Direction.Value;   ApplyMotorDirections(); }
                    if (p.M2Direction.HasValue)               { _config.M2Direction             = p.M2Direction.Value;   ApplyMotorDirections(); }
                    if (p.M3Direction.HasValue)               { _config.M3Direction             = p.M3Direction.Value;   ApplyMotorDirections(); }
                    if (p.M4Direction.HasValue)               { _config.M4Direction             = p.M4Direction.Value;   ApplyMotorDirections(); }
                    if (p.EnableNanoCards.HasValue)           _config.EnableNanoCards           = p.EnableNanoCards.Value;
                    if (p.EnableRelayCard.HasValue)           _config.EnableRelayCard           = p.EnableRelayCard.Value;
                    if (p.EnableAuxAxis.HasValue)             _config.EnableAuxAxis             = p.EnableAuxAxis.Value;
                    if (p.EnableCameras.HasValue)             _config.EnableCameras             = p.EnableCameras.Value;
                    if (p.JogSlowSpeed.HasValue)              _config.JogSlowSpeed              = p.JogSlowSpeed.Value;
                    if (p.JogNormalSpeed.HasValue)            _config.JogNormalSpeed            = p.JogNormalSpeed.Value;
                    if (p.JogFastSpeed.HasValue)              _config.JogFastSpeed              = p.JogFastSpeed.Value;
                    if (p.RobotType != null)                  { _config.RobotType               = p.RobotType;             InitializeKinematics(); }
                    bool cncMotorConfigChanged = false;
                    if (p.CncStepsPerRevX.HasValue)  { _config.CncStepsPerRevX  = p.CncStepsPerRevX.Value;  cncMotorConfigChanged = true; }
                    if (p.CncStepsPerRevY.HasValue)  { _config.CncStepsPerRevY  = p.CncStepsPerRevY.Value;  cncMotorConfigChanged = true; }
                    if (p.CncStepsPerRevZ.HasValue)  { _config.CncStepsPerRevZ  = p.CncStepsPerRevZ.Value;  cncMotorConfigChanged = true; }
                    if (p.CncStepsPerRevRZ.HasValue) { _config.CncStepsPerRevRZ = p.CncStepsPerRevRZ.Value; cncMotorConfigChanged = true; }
                    if (p.CncMmPerRevX.HasValue)     { _config.CncMmPerRevX     = p.CncMmPerRevX.Value;     cncMotorConfigChanged = true; }
                    if (p.CncMmPerRevY.HasValue)     { _config.CncMmPerRevY     = p.CncMmPerRevY.Value;     cncMotorConfigChanged = true; }
                    if (p.CncMmPerRevZ.HasValue)     { _config.CncMmPerRevZ     = p.CncMmPerRevZ.Value;     cncMotorConfigChanged = true; }
                    if (p.CncDegPerRevRZ.HasValue)   { _config.CncDegPerRevRZ   = p.CncDegPerRevRZ.Value;   cncMotorConfigChanged = true; }
                    if (cncMotorConfigChanged)        InitializeKinematics();
                    if (p.CncXHomePosition.HasValue)          _config.CncXHomePosition          = p.CncXHomePosition.Value;
                    if (p.CncYHomePosition.HasValue)          _config.CncYHomePosition          = p.CncYHomePosition.Value;
                    if (p.CncZHomePosition.HasValue)          _config.CncZHomePosition          = p.CncZHomePosition.Value;
                    if (p.CncRzHomePosition.HasValue)         _config.CncRzHomePosition         = p.CncRzHomePosition.Value;
                    if (p.CncXHomingDirection.HasValue)       _config.CncXHomingDirection       = p.CncXHomingDirection.Value;
                    if (p.CncYHomingDirection.HasValue)       _config.CncYHomingDirection       = p.CncYHomingDirection.Value;
                    if (p.CncZHomingDirection.HasValue)       _config.CncZHomingDirection       = p.CncZHomingDirection.Value;
                    if (p.JointLimitsEnabled.HasValue)        _config.JointLimitsEnabled        = p.JointLimitsEnabled.Value;
                    // Joint-limit bounds: a property present in the patch is
                    // authoritative, INCLUDING an explicit null which clears the
                    // bound (so it is no longer enforced). Absent means unchanged —
                    // so we read the raw params rather than the HasValue pattern,
                    // which cannot tell "sent null" from "not sent".
                    if (command.Params is { } rawCfg)
                    {
                        void ApplyLimit(string name, Action<double?> set)
                        {
                            if (rawCfg.TryGetProperty(name, out var el))
                                set(el.ValueKind == JsonValueKind.Null ? (double?)null : el.GetDouble());
                        }
                        ApplyLimit("joint1Min", v => _config.Joint1Min = v);
                        ApplyLimit("joint1Max", v => _config.Joint1Max = v);
                        ApplyLimit("joint2Min", v => _config.Joint2Min = v);
                        ApplyLimit("joint2Max", v => _config.Joint2Max = v);
                        ApplyLimit("joint3Min", v => _config.Joint3Min = v);
                        ApplyLimit("joint3Max", v => _config.Joint3Max = v);
                        ApplyLimit("joint4Min", v => _config.Joint4Min = v);
                        ApplyLimit("joint4Max", v => _config.Joint4Max = v);
                    }
                    RobotConfigService.Save(_config);
                    break;
                }

                case "SetSpeedOverride":
                {
                    var p = LoadParams<SetSpeedOverrideParams>(command);
                    SpeedOverrideFactor = Math.Clamp(p.Percent / 100.0, 0.05, 2.0);
                    break;
                }

                case "Home":
                    ClearFaultInternal();  // re-homing re-establishes position; drop any latched fault
                    startHoming = true;
                    break;

                case "SetHomed":
                    SetAllHomed();
                    break;

                case "Reset":
                    stb.Reset();
                    break;

                case "HardStop":
                    HardStop();
                    break;

                // ── Joint-limit fault recovery ─────────────────────────────────────
                case "ClearFault":
                    ClearFaultInternal();
                    break;

                case "SetLimitBypass":
                    {
                        var p = LoadParams<SetLimitBypassParams>(command);
                        SetLimitBypass(p.Enable);
                    }
                    break;

                // ── Aux axis commands ──────────────────────────────────────────────

                case "GetAuxState":
                {
                    var auxStates = AuxAxisManager.GetState();
                    var auxJson   = JsonSerializer.Serialize(auxStates, new JsonSerializerOptions
                    {
                        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                    });
                    payload = new { state = auxJson };
                }
                break;

                case "GetAuxConfig":
                {
                    var auxCfg    = AuxAxisManager.GetConfig();
                    var auxCfgJson = JsonSerializer.Serialize(auxCfg, new JsonSerializerOptions
                    {
                        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                    });
                    payload = new { config = auxCfgJson };
                }
                break;

                // ── Camera commands ────────────────────────────────────────────────

                case "GetCameras":
                {
                    var states    = CameraManager.GetState();
                    var statesJson = JsonSerializer.Serialize(states, new JsonSerializerOptions
                    {
                        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                    });
                    payload = new { cameras = statesJson };
                }
                break;

                case "AddCamera":
                {
                    var p = JsonSerializer.Deserialize<AddCameraParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    CameraManager.AddCamera(new Camera.CameraConfig
                    {
                        Name        = p.Name,
                        DeviceIndex = p.DeviceIndex,
                        Enabled     = p.Enabled,
                        Width       = p.Width,
                        Height      = p.Height,
                        TargetFps   = p.TargetFps,
                    });
                    break;
                }

                case "RemoveCamera":
                {
                    var p = JsonSerializer.Deserialize<RemoveCameraParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    CameraManager.RemoveCamera(p.Id);
                    break;
                }

                case "SetCameraConfig":
                {
                    var p = JsonSerializer.Deserialize<SetCameraConfigParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    CameraManager.UpdateCamera(p.Id, new Camera.CameraConfig
                    {
                        Id          = p.Id,
                        Name        = p.Name,
                        DeviceIndex = p.DeviceIndex,
                        Enabled     = p.Enabled,
                        Width       = p.Width,
                        Height      = p.Height,
                        TargetFps   = p.TargetFps,
                    });
                    break;
                }

                case "GetCameraResolutions":
                {
                    var p = JsonSerializer.Deserialize<GetCameraResolutionsParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    var deviceIndex = p.DeviceIndex;
                    var resolutions = await Task.Run(() => CameraManager.ProbeResolutionsForIndex(deviceIndex));
                    var json = JsonSerializer.Serialize(resolutions, new JsonSerializerOptions
                    {
                        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                    });
                    payload = new { resolutions = json };
                }
                break;

                // ── End camera commands ────────────────────────────────────────────

                case "MoveAux":
                {
                    var p = JsonSerializer.Deserialize<MoveAuxParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    StartAuxMove(p.DeviceId, p.Axis, p.Steps, p.Velocity, p.Accel, p.Decel);
                    break;
                }

                case "JogAux":
                {
                    var p = JsonSerializer.Deserialize<JogAuxParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    if (p.Velocity == 0)
                        StopAux(p.Decel);
                    else
                        StartAuxContinuous(p.DeviceId, p.Axis, p.Velocity, p.Accel);
                    break;
                }

                case "StopAux":
                {
                    var p = JsonSerializer.Deserialize<StopAuxParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    StopAux(p.Decel, p.Immediate);
                    break;
                }

                case "SetAuxAxisConfig":
                {
                    var p = JsonSerializer.Deserialize<SetAuxAxisConfigParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    AuxAxisManager.UpdateAxisConfig(p.DeviceId, p.AxisIndex, new AuxAxis.AuxAxisChannelConfig
                    {
                        AxisIndex       = p.AxisIndex,
                        Name            = p.Name,
                        StepsPerRev     = p.StepsPerRev,
                        InvertDirection = p.InvertDirection,
                        AxisType        = p.AxisType,
                        GearRatio       = p.GearRatio,
                        MmPerRev        = p.MmPerRev,
                    });
                    break;
                }

                case "EnableAux":
                {
                    var p = JsonSerializer.Deserialize<EnableAuxParams>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    AuxAxisManager.Enable(p.DeviceId, p.Enable);
                    break;
                }

                // ── End aux axis commands ──────────────────────────────────────────

                case "StopJog":
                    // Bump the epoch first so any jog already queued (but not yet
                    // processed on the loop thread) is invalidated and cannot
                    // re-enable motion after this stop.
                    System.Threading.Interlocked.Increment(ref _jogGeneration);
                    joggingMotionProfiler.StopJog();
                    jointJoggingProfiler.StopJog();
                    toolJoggingMotionProfiler.StopJog();
                    break;

                case "GetPoints":
                    payload = new
                    {
                        points = pointRepo.pointsJson
                    };
                    break;

                case "TeachPoint":
                    {
                        var tp = LoadParams<TeachPointParams>(command);
                        // Points are stored base-frame: teaching under an active
                        // local inverts the frame (rotation + translation), so
                        // "move to point" returns exactly here while that local
                        // stays active — and re-targets correctly when a
                        // different local is applied later.
                        var pos = CurrentPosition;
                        var loc = ActiveLocalOffset;
                        var basePos = loc == null ? pos : LocalFrame.Inverse(loc, pos);
                        pointRepo.SavePoint(tp.Name, basePos);
                    }
                    break;

                case "DeletePoint":
                    {
                        var dp = LoadParams<TeachPointParams>(command);
                        pointRepo.DeletePoint(dp.Name);
                    }
                    break;

                case "EditPoint":
                    {
                        var ep = LoadParams<EditPointParams>(command);
                        var values = new Dictionary<string, object?>();
                        if (ep.NewName != null)   values["Name"] = ep.NewName;
                        if (ep.X.HasValue)        values["X"]    = ep.X.Value;
                        if (ep.Y.HasValue)        values["Y"]    = ep.Y.Value;
                        if (ep.Z.HasValue)        values["Z"]    = ep.Z.Value;
                        if (ep.RX.HasValue)       values["RX"]   = ep.RX.Value;
                        if (ep.RY.HasValue)       values["RY"]   = ep.RY.Value;
                        if (ep.RZ.HasValue)       values["RZ"]   = ep.RZ.Value;
                        pointRepo.EditPoint(ep.Name, values);
                    }
                    break;

                case "GetStatus":
                    {
                        Vector6? pose = _kinematics.GetVisualRobotPose(CurrentPosition, CurrentTool);
                        var (j1, j2x, j2z, j4) = _kinematics.GetJointAngles();

                        // Position expressed in the active local's frame — the jog page
                        // shows this so the readout tracks the selected local. Equals
                        // the world position when no local is active.
                        var localPos = ActiveLocalOffset is { } locStat
                            ? LocalFrame.Inverse(locStat, CurrentPosition)
                            : CurrentPosition;

                        payload = new
                        {
                            moving = IsMoving,
                            wasHomed = homed,
                            homingState = this.homingState,
                            isHoming = this.homingState != "WaitingForStart",
                            lastPointUpdate = pointRepo.LastUpdatedUnixMs,
                            driverConnected = stb.connected,
                            driverOk = stb.connected && stb.status != 0,

                            x = CurrentPosition.X,
                            y = CurrentPosition.Y,
                            z = CurrentPosition.Z,
                            rx = CurrentPosition.RX,
                            ry = CurrentPosition.RY,
                            rz = CurrentPosition.RZ,

                            localX = localPos.X,
                            localY = localPos.Y,
                            localZ = localPos.Z,
                            localRZ = localPos.RZ,

                            targetX = this.TargetPosition.X,
                            targetY = this.TargetPosition.Y,
                            targetZ = this.TargetPosition.Z,
                            targetRX = this.TargetPosition.RX,
                            targetRY = this.TargetPosition.RY,
                            targetRZ = this.TargetPosition.RZ,

                            joint1Angle = j1,
                            joint2X = j2x,
                            joint2Z = j2z,
                            joint4Angle = j4,

                            poseX = pose?.X ?? 0,
                            poseY = pose?.Y ?? 0,
                            poseZ = pose?.Z ?? 0,
                            poseRX = pose?.RX ?? 0,
                            poseRY = pose?.RY ?? 0,
                            poseRZ = pose?.RZ ?? 0,

                            speedS = SpeedS,
                            accelS = AccelS,
                            decelS = DecelS,

                            speedJ = SpeedJ,
                            accelJ = AccelJ,
                            decelJ = DecelJ,

                            // STB digital inputs
                            input1 = stb.Input1,
                            input2 = stb.Input2,
                            input3 = stb.Input3,
                            input4 = stb.Input4,

                            // STB digital outputs
                            output1 = stb.Output1,
                            output2 = stb.Output2,
                            output3 = stb.Output3,
                            output4 = stb.Output4,

                            // Program cycle — summary only (no logs / images)
                            programs = programManager.GetProgramsSummary(),

                            // Tool repository
                            lastToolUpdate = toolRepo.LastUpdatedUnixMs,
                            activeTool     = this.activeTool,

                            // Local repository
                            lastLocalUpdate = localRepo.LastUpdatedUnixMs,
                            activeLocal     = this.activeLocal,

                            // Background programs currently running
                            backgroundPrograms = backgroundProgramManager.GetStatuses()
                                .Select(s => new { name = s.Name, currentStep = s.CurrentStep })
                                .ToList(),

                            // Built program repository
                            lastBuiltProgramUpdate = builtProgramRepo.LastUpdatedUnixMs,

                            // Grid repository
                            lastGridUpdate = gridRepo.LastUpdatedUnixMs,

                            // Stack repository
                            lastStackUpdate = stackRepo.LastUpdatedUnixMs,

                            speedOverridePercent = SpeedOverrideFactor * 100.0,

                            // Joint soft-limit fault state
                            faulted            = _faulted,
                            faultJoint         = _faultJoint,
                            faultDirection     = _faultDirection,
                            faultMessage       = _faultMessage,
                            limitBypass        = _limitBypass,
                            jointLimitsEnabled = _config.JointLimitsEnabled,
                            robotType          = _config.RobotType,

                            version = _version,
                            isLinux = System.Runtime.InteropServices.RuntimeInformation.IsOSPlatform(System.Runtime.InteropServices.OSPlatform.Linux),
                        };
                        break;
                    }

                // ── Program cycle ─────────────────────────────────────────

                case "SetAvailablePrograms":
                    {
                        var p = LoadParams<SetAvailableProgramsParams>(command);
                        programManager.SetAvailablePrograms(p.Programs);
                    }
                    break;

                case "SetProgramStatus":
                    {
                        var update = LoadParams<ProgramCycleUpdate>(command);
                        programManager.ApplyStatusUpdate(update);
                    }
                    break;

                case "GetProgramImages":
                    {
                        // Merge live in-memory images (Python/external) with persisted images
                        // for built programs so idle built programs still show their image.
                        var merged = programManager.GetAllImages();
                        foreach (var kv in builtProgramRepo.GetAllImages())
                            if (kv.Value != null) merged[kv.Key] = kv.Value;
                        payload = new { images = merged };
                    }
                    break;

                case "GetProgramLogs":
                    {
                        var p = LoadParams<GetProgramLogsParams>(command);
                        var (total, start, logs) = programManager.GetProgramLogs(p.ProgramName, p.Start, p.End);
                        payload = new
                        {
                            programName = p.ProgramName,
                            totalCount  = total,
                            start,
                            logs
                        };
                    }
                    break;

                case "StartProgram":
                    {
                        var p     = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null)
                        {
                            if (programExecutor?.IsPaused == true && programExecutor.CurrentProgramName == p.ProgramName)
                                programExecutor.Resume();
                            else
                            {
                                DisplaceRunningBuiltProgram(p.ProgramName);
                                programExecutor?.Start(built);
                            }
                        }
                        else
                            programManager.SetFlag(p.ProgramName, "Start");
                    }
                    break;

                case "StopProgram":
                    {
                        var p     = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null)
                        {
                            if (programExecutor?.CurrentProgramName == p.ProgramName)
                                programExecutor.Stop();
                        }
                        else
                            programManager.SetFlag(p.ProgramName, "Stop");
                    }
                    break;

                case "ResetProgram":
                    {
                        var p     = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null)
                        {
                            if (programExecutor?.CurrentProgramName == p.ProgramName)
                                programExecutor.Reset();
                            programManager.ResetToReady(p.ProgramName,
                                ProgramExecutor.CountSteps(built.Steps));
                        }
                        else
                        {
                            programManager.SetFlag(p.ProgramName, "Reset");
                        }
                    }
                    break;

                case "AbortProgram":
                    {
                        var p     = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null)
                        {
                            if (programExecutor?.CurrentProgramName == p.ProgramName)
                                programExecutor.Reset();
                            programManager.ResetToReady(p.ProgramName,
                                ProgramExecutor.CountSteps(built.Steps));
                        }
                        else
                        {
                            programManager.SetFlag(p.ProgramName, "Abort");
                        }
                    }
                    break;

                case "ClearProgramActions":
                    {
                        var p = LoadParams<ProgramActionParams>(command);
                        programManager.ClearActions(p.ProgramName);
                    }
                    break;

                // ── Tool repository ───────────────────────────────────────

                case "GetTools":
                    payload = new { tools = toolRepo.toolsJson };
                    break;

                case "CreateTool":
                    {
                        var ep = LoadParams<EditToolParams>(command);
                        var v  = new Vector6(ep.X ?? 0, ep.Y ?? 0, ep.Z ?? 0,
                                             ep.RX ?? 0, ep.RY ?? 0, ep.RZ ?? 0);
                        var tool = toolRepo.SaveTool(ep.Name, v);
                        if (!string.IsNullOrEmpty(ep.Description))
                        {
                            toolRepo.EditTool(ep.Name, new()
                            {
                                ["Description"] = ep.Description
                            });
                        }
                    }
                    break;

                case "EditTool":
                    {
                        var ep     = LoadParams<EditToolParams>(command);
                        var values = new Dictionary<string, object?>();
                        if (ep.NewName      != null) values["Name"]        = ep.NewName;
                        if (ep.Description  != null) values["Description"] = ep.Description;
                        if (ep.X.HasValue)           values["X"]           = ep.X.Value;
                        if (ep.Y.HasValue)           values["Y"]           = ep.Y.Value;
                        if (ep.Z.HasValue)           values["Z"]           = ep.Z.Value;
                        if (ep.RX.HasValue)          values["RX"]          = ep.RX.Value;
                        if (ep.RY.HasValue)          values["RY"]          = ep.RY.Value;
                        if (ep.RZ.HasValue)          values["RZ"]          = ep.RZ.Value;
                        toolRepo.EditTool(ep.Name, values);

                        // Keep activeTool name in sync after a rename
                        if (ep.NewName != null && activeTool == ep.Name)
                            activeTool = ep.NewName;
                    }
                    break;

                case "DeleteTool":
                    {
                        var tp = LoadParams<ToolNameParams>(command);
                        toolRepo.DeleteTool(tp.Name);
                        // Clear active tool if the deleted one was active
                        if (activeTool == tp.Name)
                        {
                            activeTool          = "";
                            CurrentTool         = Vector6.Zero;
                            CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
                            CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                        }
                    }
                    break;

                // ── Local repository ──────────────────────────────────────

                case "GetLocals":
                    payload = new { locals = localRepo.localsJson };
                    break;

                case "CreateLocal":
                    {
                        var ep = LoadParams<EditLocalParams>(command);
                        var v  = new Vector6(ep.X ?? 0, ep.Y ?? 0, ep.Z ?? 0,
                                             ep.RX ?? 0, ep.RY ?? 0, ep.RZ ?? 0);
                        localRepo.SaveLocal(ep.Name, v);
                        if (!string.IsNullOrEmpty(ep.Description))
                        {
                            localRepo.EditLocal(ep.Name, new()
                            {
                                ["Description"] = ep.Description
                            });
                        }
                    }
                    break;

                case "EditLocal":
                    {
                        var ep     = LoadParams<EditLocalParams>(command);
                        var values = new Dictionary<string, object?>();
                        if (ep.NewName      != null) values["Name"]        = ep.NewName;
                        if (ep.Description  != null) values["Description"] = ep.Description;
                        if (ep.X.HasValue)           values["X"]           = ep.X.Value;
                        if (ep.Y.HasValue)           values["Y"]           = ep.Y.Value;
                        if (ep.Z.HasValue)           values["Z"]           = ep.Z.Value;
                        if (ep.RX.HasValue)          values["RX"]          = ep.RX.Value;
                        if (ep.RY.HasValue)          values["RY"]          = ep.RY.Value;
                        if (ep.RZ.HasValue)          values["RZ"]          = ep.RZ.Value;
                        localRepo.EditLocal(ep.Name, values);

                        // Keep activeLocal name in sync after a rename
                        if (ep.NewName != null && activeLocal == ep.Name)
                            activeLocal = ep.NewName;
                    }
                    break;

                case "DeleteLocal":
                    {
                        var lp = LoadParams<LocalNameParams>(command);
                        localRepo.DeleteLocal(lp.Name);
                        // Clear active local if the deleted one was active
                        if (activeLocal == lp.Name)
                        {
                            activeLocal  = "";
                            CurrentLocal = Vector6.Zero;
                        }
                    }
                    break;

                case "SetActiveLocal":
                    {
                        var lp = LoadParams<LocalNameParams>(command);
                        ApplyLocal(lp.Name);
                    }
                    break;

                // ── Built program repository ──────────────────────────────

                case "SaveBuiltProgram":
                    {
                        var p = LoadParams<SaveBuiltProgramParams>(command);
                        builtProgramRepo.Save(new BuiltProgram
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
                    break;

                case "StartBackgroundProgram":
                    {
                        var p    = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null && built.IsBackground)
                            backgroundProgramManager.TryStart(built);
                    }
                    break;

                case "StopBackgroundProgram":
                    {
                        var p     = LoadParams<ProgramActionParams>(command);
                        var built = builtProgramRepo.Get(p.ProgramName);
                        if (built != null) backgroundProgramManager.Stop(built.Id);
                    }
                    break;

                case "GetProgramVariables":
                    {
                        var p    = LoadParams<BuiltProgramNameParams>(command);
                        var vars = (programExecutor?.CurrentProgramName?.Equals(p.Name, StringComparison.OrdinalIgnoreCase) == true)
                            ? programExecutor.GetDisplayVariables()
                            : backgroundProgramManager.GetDisplayVariables(p.Name);
                        payload = new
                        {
                            variables = vars.Select(v => new { name = v.Name, value = v.Value, isBoolean = v.IsBoolean }).ToList()
                        };
                    }
                    break;

                case "DeleteBuiltProgram":
                    {
                        var p = LoadParams<BuiltProgramNameParams>(command);
                        builtProgramRepo.Delete(p.Name);
                        programManager.RemoveProgram(p.Name);
                    }
                    break;

                case "SaveBuiltProgramImage":
                    {
                        var p = LoadParams<SaveBuiltProgramImageParams>(command);
                        var bytes = Convert.FromBase64String(p.Image);
                        builtProgramRepo.SaveImage(p.Name, bytes);
                    }
                    break;

                case "GetBuiltPrograms":
                    {
                        var list = builtProgramRepo.GetAll();
                        var json = System.Text.Json.JsonSerializer.Serialize(list, new System.Text.Json.JsonSerializerOptions
                        {
                            Converters = { new System.Text.Json.Serialization.JsonStringEnumConverter() },
                            PropertyNamingPolicy = System.Text.Json.JsonNamingPolicy.CamelCase,
                        });
                        payload = new { programs = json };
                    }
                    break;

                // ── Grid repository ───────────────────────────────────────────────
                case "GetGrids":
                    {
                        var list = gridRepo.GetAll();
                        var json = System.Text.Json.JsonSerializer.Serialize(list, new System.Text.Json.JsonSerializerOptions
                        {
                            PropertyNameCaseInsensitive = true,
                        });
                        payload = new { grids = json };
                    }
                    break;

                case "SaveGrid":
                    {
                        var p = LoadParams<SaveGridParams>(command);
                        gridRepo.Upsert(new Grid
                        {
                            Id            = p.Id,
                            Name          = p.Name,
                            BasePointName = p.BasePointName,
                            RowOffsetX    = p.RowOffsetX,
                            RowOffsetY    = p.RowOffsetY,
                            RowOffsetZ    = p.RowOffsetZ,
                            ColOffsetX    = p.ColOffsetX,
                            ColOffsetY    = p.ColOffsetY,
                            ColOffsetZ    = p.ColOffsetZ,
                            RowCount      = p.RowCount,
                            ColCount      = p.ColCount,
                            Rotation      = p.Rotation,
                        });
                    }
                    break;

                case "DeleteGrid":
                    {
                        var p = LoadParams<GridIdParams>(command);
                        gridRepo.Delete(p.Id);
                    }
                    break;

                // ── Stack repository ──────────────────────────────────────────────
                case "GetStacks":
                    {
                        var list = stackRepo.GetAll();
                        var json = System.Text.Json.JsonSerializer.Serialize(list, new System.Text.Json.JsonSerializerOptions
                        {
                            PropertyNameCaseInsensitive = true,
                        });
                        payload = new { stacks = json };
                    }
                    break;

                case "SaveStack":
                    {
                        var p = LoadParams<SaveStackParams>(command);
                        stackRepo.Upsert(new RobotStack
                        {
                            Id            = p.Id,
                            Name          = p.Name,
                            BasePointName = p.BasePointName,
                            OffsetX       = p.OffsetX,
                            OffsetY       = p.OffsetY,
                            OffsetZ       = p.OffsetZ,
                            MaxCount      = p.MaxCount,
                        });
                    }
                    break;

                case "DeleteStack":
                    {
                        var p = LoadParams<StackIdParams>(command);
                        stackRepo.Delete(p.Id);
                    }
                    break;

                case "ExecuteBuiltProgram":
                    {
                        var p    = LoadParams<BuiltProgramNameParams>(command);
                        var prog = builtProgramRepo.Get(p.Name);
                        if (prog != null)
                        {
                            DisplaceRunningBuiltProgram(p.Name);
                            var imgBytes = builtProgramRepo.GetImage(p.Name);
                            programExecutor?.Start(prog, imgBytes != null ? Convert.ToBase64String(imgBytes) : null);
                        }
                    }
                    break;

                case "StopBuiltProgram":
                    programExecutor?.Stop();
                    break;

                case "SetActiveTool":
                    {
                        var tp = LoadParams<ToolNameParams>(command);
                        if (string.IsNullOrEmpty(tp.Name) || tp.Name == "None")
                        {
                            activeTool  = "";
                            CurrentTool = Vector6.Zero;
                        }
                        else
                        {
                            var tool = toolRepo.Get(tp.Name);
                            if (tool != null)
                            {
                                activeTool  = tp.Name;
                                CurrentTool = new Vector6(tool.X, tool.Y, tool.Z,
                                                          tool.RX, tool.RY, tool.RZ);
                            }
                        }
                        // Recalculate position with new tool offset
                        CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
                        CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                    }
                    break;

                // ── STB4100 IO ────────────────────────────────────────────────

                case "SetSTBOutput":
                    {
                        var p = LoadParams<SetNanoOutputParams>(command); // reuse same params shape
                        stb.SetOutput(p.Pin, p.Value);
                    }
                    break;

                // ── Nano IO ───────────────────────────────────────────────────

                case "GetIO":
                    {
                        var states = NanoManager.GetAllStates();
                        var nanoJson = JsonSerializer.Serialize(states, new JsonSerializerOptions
                        {
                            Converters           = { new JsonStringEnumConverter() },
                            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                        });

                        var relayStates = RelayManager.GetRelayStates();
                        var relayState  = new UsbRelayState
                        {
                            Connected = RelayManager.IsConnected,
                            Serial    = RelayManager.GetSerial(),
                            Relays    = relayStates,
                            Names     = RelayManager.GetRelayNames(),
                        };
                        var relayJson = JsonSerializer.Serialize(relayState, new JsonSerializerOptions
                        {
                            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                        });

                        payload = new { nanos = nanoJson, relay = relayJson };
                    }
                    break;

                case "SetNanoOutput":
                    {
                        var p = LoadParams<SetNanoOutputParams>(command);
                        NanoManager.SetOutput(p.NanoId, p.Pin, p.Value);
                    }
                    break;

                case "SetNeoPixel":
                    {
                        var p = LoadParams<SetNeoPixelParams>(command);
                        var colors = p.Colors
                            .Select(c => new NeoPixelColor(c.R, c.G, c.B))
                            .ToArray();
                        NanoManager.SetNeoPixel(p.NanoId, p.Pin, colors);
                    }
                    break;

                case "RenameNanoPin":
                    {
                        var p = LoadParams<RenameNanoPinParams>(command);
                        NanoManager.RenamePin(p.NanoId, p.Pin, p.Name);
                    }
                    break;

                case "ConfigureNanoPin":
                    {
                        var p    = LoadParams<ConfigureNanoPinParams>(command);
                        var type = p.Type switch
                        {
                            "Output"       => Nano.PinType.Output,
                            "Neopixel"     => Nano.PinType.Neopixel,
                            "Unconfigured" => Nano.PinType.Unconfigured,
                            _              => Nano.PinType.Input,
                        };
                        NanoManager.SetPinType(p.NanoId, p.Pin, type, p.PixelCount);
                    }
                    break;

                case "SetRelay":
                    {
                        var p = LoadParams<SetRelayParams>(command);
                        RelayManager.SetRelay(p.Relay, p.Value);
                    }
                    break;

                case "RenameRelay":
                    {
                        var p = LoadParams<RenameRelayParams>(command);
                        RelayManager.RenameRelay(p.Relay, p.Name);
                    }
                    break;

                case "GetRelayState":
                    {
                        var relayStates = RelayManager.GetRelayStates();
                        payload = new UsbRelayState
                        {
                            Connected = RelayManager.IsConnected,
                            Serial    = RelayManager.GetSerial(),
                            Relays    = relayStates,
                            Names     = RelayManager.GetRelayNames(),
                        };
                    }
                    break;

                // ── Vision commands ───────────────────────────────────────────────

                case "GetVisionPrograms":
                {
                    var programs = VisionRepo.GetAll();
                    var json = JsonSerializer.Serialize(programs, new JsonSerializerOptions
                    {
                        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
                    });
                    payload = new { programs = json, runningIds = VisionManager.GetRunningIds() };
                }
                break;

                case "SaveVisionProgram":
                {
                    var prog = JsonSerializer.Deserialize<Vision.VisionProgram>(
                        command.Params!.Value.GetRawText(), _jsonOptions)!;
                    if (string.IsNullOrEmpty(prog.Id))
                        prog.Id = Guid.NewGuid().ToString("N")[..8];
                    VisionRepo.Save(prog);
                    VisionManager.OnProgramSaved(prog);
                    payload = new { programId = prog.Id, lastUpdatedUnixMs = prog.LastUpdatedUnixMs };
                }
                break;

                case "DeleteVisionProgram":
                {
                    var p = LoadParams<DeleteVisionProgramParams>(command);
                    VisionManager.StopProgram(p.Id);
                    VisionRepo.Delete(p.Id);
                }
                break;

                case "StartVision":
                {
                    var p = LoadParams<StartStopVisionParams>(command);
                    VisionManager.StartProgram(p.Id);
                }
                break;

                case "StopVision":
                {
                    var p = LoadParams<StartStopVisionParams>(command);
                    VisionManager.StopProgram(p.Id);
                }
                break;

                case "GetCncToolpath":
                {
                    // Resolved toolpath of the CNC block currently executing —
                    // anchor and variables applied. Null when no block is active.
                    var tp = ActiveCncToolpath;
                    payload = new
                    {
                        toolpath = tp == null ? null : new
                        {
                            programName = tp.ProgramName,
                            paths       = tp.Paths,
                            holes       = tp.Holes.Select(h => new { x = h.X, y = h.Y }).ToList(),
                        },
                    };
                }
                break;

                case "GetVisionResult":
                {
                    var p = LoadParams<StartStopVisionParams>(command);
                    var proc = VisionManager.GetProcessor(p.Id);
                    // Live result while running, else the last one captured at the end
                    // of the most recent RunVision step for this program.
                    var result = proc?.GetLatestResult() ?? GetProgramVisionResult(p.Id);
                    var json   = result != null
                        ? JsonSerializer.Serialize(result, new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase })
                        : null;
                    payload = new { result = json };
                }
                break;

                // ── End vision commands ───────────────────────────────────────────

                default:
                    RobotCommand NewCommand = LoadParams<RobotCommand>(command);
                    NewCommand.CommandType = command.Command;
                    // Stamp with the current jog epoch so a stop arriving after this
                    // enqueue can invalidate a trailing jog (see _jogGeneration).
                    NewCommand.JogGeneration = _jogGeneration;
                    QueuedCommands.Enqueue(NewCommand);
                    break;

            }
            payload ??= new { };

            return (object)payload;
        }
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

            if (IsMoving && !CommandType.Contains("Jog"))
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
                        if (target == null) break;  // named point not found — log already emitted; drop command
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
                        if (target == null) break;  // named point not found — log already emitted; drop command
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
        public void RunHoming()
        {
            double m1Deg, m2Deg, m3Deg, m4Deg;

            switch (homingState)
            {
                case "WaitingForStart":
                    if (startHoming)
                        homingState = _kinematics is CNC4AxisKinematics ? "CNC_HomeZ" : "HomeVertical";
                    break;

                case "HomeVertical":
                    if (stb.Input2)
                    {
                        // Sensor already triggered — skip fast approach and go straight to back-off
                        homingState = "BackOffVertical";
                        break;
                    }
                    // The last Jog() arg is the profiler's watchdog reset time. RunHoming re-issues
                    // this jog every tick, but RunMotion (which advances the profiler) runs a full
                    // ~4ms tick BEFORE the next RunHoming, so the watchdog MUST exceed one tick or it
                    // expires between the Jog and the next Update and the axis never moves (flicker,
                    // no motion). It used to be 0.001s, which only worked when Loop() was a tight
                    // busy-spin; the 4ms gated tick requires a larger value. ExecuteHardStop handles
                    // the precise stop at the switch, so this is only a safety net.
                    jointJoggingProfiler.Jog(new(0, 0, _config.VerticalHomingDirection), _config.HomingSpeed, 100, 10000000, 0.1);
                    if (stb.Input2)
                    {
                        ExecuteHardStop();
                        homingState = "WaitVerticalStop1";
                    }
                    break;

                case "WaitVerticalStop1":
                    if (!IsMoving)
                        homingState = "BackOffVertical";
                    break;

                case "BackOffVertical":
                {
                    var t = new Vector6(
                        CurrentJointTargets.X,
                        CurrentJointTargets.Y,
                        CurrentJointTargets.Z - (_config.HomingBackoffMm * _config.VerticalHomingDirection),
                        CurrentJointTargets.RX,
                        CurrentJointTargets.RY,
                        CurrentJointTargets.RZ
                    );
                    this.TargetJoints = t;
                    jointMotionProfiler = new(CurrentJointTargets, t, _config.HomingSpeed, 100, 200);
                    homingState = "WaitVerticalBackoff";
                    break;
                }

                case "WaitVerticalBackoff":
                    if (!IsMoving)
                        homingState = "HomeVerticalSlow";
                    break;

                case "HomeVerticalSlow":
                    jointJoggingProfiler.Jog(new(0, 0, _config.VerticalHomingDirection), _config.HomingSlowSpeed, 50, 10000000, 0.1);
                    if (stb.Input2)
                    {
                        ExecuteHardStop();
                        homingState = "WaitVerticalMoveComplete";
                    }
                    break;

                case "WaitVerticalMoveComplete":
                    if (!IsMoving)
                        homingState = "SetVerticalHomed";
                    break;

                case "SetVerticalHomed":
                {
                    var astro = (ASTROKinematics)_kinematics;
                    astro.InterpolatedJoint2.Cartesian = (astro.InterpolatedJoint2.Cartesian.x, _config.VerticalHomePosition);
                    astro.CurrentJoint2.Cartesian      = (astro.CurrentJoint2.Cartesian.x,      _config.VerticalHomePosition);

                    CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
                    CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                    _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);

                    homingState = "HomeHorizontal";
                    break;
                }

                case "HomeHorizontal":
                    if (stb.Input3)
                    {
                        homingState = "BackOffHorizontal";
                        break;
                    }
                    jointJoggingProfiler.Jog(new(0, _config.HorizontalHomingDirection), _config.HomingSpeed, 100, 10000000, 0.1);
                    if (stb.Input3)
                    {
                        ExecuteHardStop();
                        homingState = "WaitHorizontalStop1";
                    }
                    break;

                case "WaitHorizontalStop1":
                    if (!IsMoving)
                        homingState = "BackOffHorizontal";
                    break;

                case "BackOffHorizontal":
                {
                    var t = new Vector6(
                        CurrentJointTargets.X,
                        CurrentJointTargets.Y - (_config.HomingBackoffMm * _config.HorizontalHomingDirection),
                        CurrentJointTargets.Z,
                        CurrentJointTargets.RX,
                        CurrentJointTargets.RY,
                        CurrentJointTargets.RZ
                    );
                    this.TargetJoints = t;
                    jointMotionProfiler = new(CurrentJointTargets, t, _config.HomingSpeed, 100, 200);
                    homingState = "WaitHorizontalBackoff";
                    break;
                }

                case "WaitHorizontalBackoff":
                    if (!IsMoving)
                        homingState = "HomeHorizontalSlow";
                    break;

                case "HomeHorizontalSlow":
                    jointJoggingProfiler.Jog(new(0, _config.HorizontalHomingDirection), _config.HomingSlowSpeed, 50, 10000000, 0.1);
                    if (stb.Input3)
                    {
                        ExecuteHardStop();
                        homingState = "WaitHorizontalMoveComplete";
                    }
                    break;

                case "WaitHorizontalMoveComplete":
                    if (!IsMoving)
                        homingState = "SetHorizontalHomed";
                    break;

                case "SetHorizontalHomed":
                {
                    var astro = (ASTROKinematics)_kinematics;
                    astro.InterpolatedJoint2.Cartesian = (_config.HorizontalHomePosition, astro.InterpolatedJoint2.Cartesian.z);
                    astro.CurrentJoint2.Cartesian      = (_config.HorizontalHomePosition, astro.CurrentJoint2.Cartesian.z);

                    CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
                    CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                    _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);

                    homingState = "HomeJ1";
                    break;
                }

                case "HomeJ1":
                    if (stb.Input1)
                    {
                        homingState = "BackOffJ1";
                        break;
                    }
                    jointJoggingProfiler.Jog(new(_config.J1HomingDirection), _config.HomingSpeed, 100, 10000000, 0.1);
                    if (stb.Input1)
                    {
                        ExecuteHardStop();
                        homingState = "WaitJ1Stop1";
                    }
                    break;

                case "WaitJ1Stop1":
                    if (!IsMoving)
                        homingState = "BackOffJ1";
                    break;

                case "BackOffJ1":
                {
                    var t = new Vector6(
                        CurrentJointTargets.X - (_config.HomingBackoffMm * _config.J1HomingDirection),
                        CurrentJointTargets.Y,
                        CurrentJointTargets.Z,
                        CurrentJointTargets.RX,
                        CurrentJointTargets.RY,
                        CurrentJointTargets.RZ
                    );
                    this.TargetJoints = t;
                    jointMotionProfiler = new(CurrentJointTargets, t, _config.HomingSpeed, 100, 200);
                    homingState = "WaitJ1Backoff";
                    break;
                }

                case "WaitJ1Backoff":
                    if (!IsMoving)
                        homingState = "HomeJ1Slow";
                    break;

                case "HomeJ1Slow":
                    jointJoggingProfiler.Jog(new(_config.J1HomingDirection), _config.HomingSlowSpeed, 50, 10000000, 0.1);
                    if (stb.Input1)
                    {
                        ExecuteHardStop();
                        homingState = "WaitJ1MoveComplete";
                    }
                    break;

                case "WaitJ1MoveComplete":
                    if (!IsMoving)
                        homingState = "SetJ1MotorHomed";
                    break;

                case "SetJ1MotorHomed":
                {
                    var astro = (ASTROKinematics)_kinematics;
                    astro.InterpolatedJoint1.JointAngleDeg = _config.J1HomeOffsetDeg;
                    astro.CurrentJoint1.JointAngleDeg      = _config.J1HomeOffsetDeg;

                    CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
                    CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                    _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);

                    homingState = "HomeJ4";
                    break;
                }

                case "HomeJ4":
                {
                    // Drive J4 to 0° (mechanical zero) using a joint motion profile.
                    // J1/J2/J3 stay at their current positions; only RZ (J4) changes.
                    var j4Target = new Vector6(
                        CurrentJointTargets.X,
                        CurrentJointTargets.Y,
                        CurrentJointTargets.Z,
                        CurrentJointTargets.RX,
                        CurrentJointTargets.RY,
                        0  // J4 → 0 degrees
                    );
                    // TargetJoints must be set before creating the profiler — RunMotion snaps
                    // CurrentJointTargets to TargetJoints on the tick the profiler finishes.
                    // Without this, the snap would restore stale pre-homing joint values and
                    // corrupt the home positions already set for J1/J2.
                    this.TargetJoints = j4Target;
                    jointMotionProfiler = new(CurrentJointTargets, j4Target, _config.HomingSpeed, 100, 200);
                    homingState = "WaitJ4MoveComplete";
                    break;
                }

                case "WaitJ4MoveComplete":
                    if (!IsMoving)
                        homingState = "SetJ4Homed";
                    break;

                case "SetJ4Homed":
                {
                    var astro = (ASTROKinematics)_kinematics;
                    astro.InterpolatedJoint4.JointAngleDeg = _config.J4HomeOffsetDeg;
                    astro.CurrentJoint4.JointAngleDeg      = _config.J4HomeOffsetDeg;

                    CurrentPosition     = _kinematics.ForwardKinematics(CurrentTool);
                    CurrentJointTargets = _kinematics.InverseKinematics(CurrentPosition, CurrentTool);
                    _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);

                    homingState = "HomingComplete";
                    break;
                }

                case "HomingComplete":
                    startHoming = false;
                    homingState = "WaitingForStart";
                    homed = true;
                    break;

                // ── CNC4Axis homing ─────────────────────────────────────────────────
                // Sequence: Z (Input3) → X (Input1) → Y (Input2) → zero RZ

                case "CNC_HomeZ":
                    if (stb.Input3)
                    {
                        homingState = "CNC_BackOffZ";
                        break;
                    }
                    jointJoggingProfiler.Jog(new(0, 0, _config.CncZHomingDirection), _config.HomingSpeed, 100, 10000000, 0.1);
                    if (stb.Input3)
                    {
                        ExecuteHardStop();
                        homingState = "CNC_WaitZStop";
                    }
                    break;

                case "CNC_WaitZStop":
                    if (!IsMoving) homingState = "CNC_BackOffZ";
                    break;

                case "CNC_BackOffZ":
                {
                    var t = new Vector6(
                        CurrentJointTargets.X,
                        CurrentJointTargets.Y,
                        CurrentJointTargets.Z - (_config.HomingBackoffMm * _config.CncZHomingDirection),
                        CurrentJointTargets.RX,
                        CurrentJointTargets.RY,
                        CurrentJointTargets.RZ
                    );
                    TargetJoints = t;
                    jointMotionProfiler = new(CurrentJointTargets, t, _config.HomingSpeed, 100, 200);
                    homingState = "CNC_WaitZBackoff";
                    break;
                }

                case "CNC_WaitZBackoff":
                    if (!IsMoving) homingState = "CNC_HomeZSlow";
                    break;

                case "CNC_HomeZSlow":
                    jointJoggingProfiler.Jog(new(0, 0, _config.CncZHomingDirection), _config.HomingSlowSpeed, 50, 10000000, 0.1);
                    if (stb.Input3)
                    {
                        ExecuteHardStop();
                        homingState = "CNC_WaitZMoveDone";
                    }
                    break;

                case "CNC_WaitZMoveDone":
                    if (!IsMoving) homingState = "CNC_SetZHomed";
                    break;

                case "CNC_SetZHomed":
                    CurrentJointTargets.Z = _config.CncZHomePosition;
                    _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
                    CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
                    homingState = "CNC_HomeX";
                    break;

                case "CNC_HomeX":
                    if (stb.Input1)
                    {
                        homingState = "CNC_BackOffX";
                        break;
                    }
                    jointJoggingProfiler.Jog(new(_config.CncXHomingDirection), _config.HomingSpeed, 100, 10000000, 0.1);
                    if (stb.Input1)
                    {
                        ExecuteHardStop();
                        homingState = "CNC_WaitXStop";
                    }
                    break;

                case "CNC_WaitXStop":
                    if (!IsMoving) homingState = "CNC_BackOffX";
                    break;

                case "CNC_BackOffX":
                {
                    var t = new Vector6(
                        CurrentJointTargets.X - (_config.HomingBackoffMm * _config.CncXHomingDirection),
                        CurrentJointTargets.Y,
                        CurrentJointTargets.Z,
                        CurrentJointTargets.RX,
                        CurrentJointTargets.RY,
                        CurrentJointTargets.RZ
                    );
                    TargetJoints = t;
                    jointMotionProfiler = new(CurrentJointTargets, t, _config.HomingSpeed, 100, 200);
                    homingState = "CNC_WaitXBackoff";
                    break;
                }

                case "CNC_WaitXBackoff":
                    if (!IsMoving) homingState = "CNC_HomeXSlow";
                    break;

                case "CNC_HomeXSlow":
                    jointJoggingProfiler.Jog(new(_config.CncXHomingDirection), _config.HomingSlowSpeed, 50, 10000000, 0.1);
                    if (stb.Input1)
                    {
                        ExecuteHardStop();
                        homingState = "CNC_WaitXMoveDone";
                    }
                    break;

                case "CNC_WaitXMoveDone":
                    if (!IsMoving) homingState = "CNC_SetXHomed";
                    break;

                case "CNC_SetXHomed":
                    CurrentJointTargets.X = _config.CncXHomePosition;
                    _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
                    CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
                    homingState = "CNC_HomeY";
                    break;

                case "CNC_HomeY":
                    if (stb.Input2)
                    {
                        homingState = "CNC_BackOffY";
                        break;
                    }
                    jointJoggingProfiler.Jog(new(0, _config.CncYHomingDirection), _config.HomingSpeed, 100, 10000000, 0.1);
                    if (stb.Input2)
                    {
                        ExecuteHardStop();
                        homingState = "CNC_WaitYStop";
                    }
                    break;

                case "CNC_WaitYStop":
                    if (!IsMoving) homingState = "CNC_BackOffY";
                    break;

                case "CNC_BackOffY":
                {
                    var t = new Vector6(
                        CurrentJointTargets.X,
                        CurrentJointTargets.Y - (_config.HomingBackoffMm * _config.CncYHomingDirection),
                        CurrentJointTargets.Z,
                        CurrentJointTargets.RX,
                        CurrentJointTargets.RY,
                        CurrentJointTargets.RZ
                    );
                    TargetJoints = t;
                    jointMotionProfiler = new(CurrentJointTargets, t, _config.HomingSpeed, 100, 200);
                    homingState = "CNC_WaitYBackoff";
                    break;
                }

                case "CNC_WaitYBackoff":
                    if (!IsMoving) homingState = "CNC_HomeYSlow";
                    break;

                case "CNC_HomeYSlow":
                    jointJoggingProfiler.Jog(new(0, _config.CncYHomingDirection), _config.HomingSlowSpeed, 50, 10000000, 0.1);
                    if (stb.Input2)
                    {
                        ExecuteHardStop();
                        homingState = "CNC_WaitYMoveDone";
                    }
                    break;

                case "CNC_WaitYMoveDone":
                    if (!IsMoving) homingState = "CNC_SetYHomed";
                    break;

                case "CNC_SetYHomed":
                    CurrentJointTargets.Y = _config.CncYHomePosition;
                    _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
                    CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
                    homingState = "CNC_ZeroRZ";
                    break;

                case "CNC_ZeroRZ":
                    // RZ (threading spindle) has no limit switch — zero it at current position
                    CurrentJointTargets.RZ = _config.CncRzHomePosition;
                    _kinematics.UpdateMotorTargets(CurrentJointTargets, out m1Deg, out m2Deg, out m3Deg, out m4Deg);
                    stb.OverwriteMotorTargets(m1Deg, m2Deg, m3Deg, m4Deg);
                    CurrentPosition = _kinematics.ForwardKinematics(CurrentTool);
                    homingState = "HomingComplete";
                    break;

                default:
                    break;
            }
        }

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
        private void DisplaceRunningBuiltProgram(string incomingProgramName)
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
            _hardStopRequested = false;
            linearMotionProfiler = null;
            jointMotionProfiler = null;
            continuousProfiler = null;
            joggingMotionProfiler.ForceStop();
            jointJoggingProfiler.ForceStop();
            toolJoggingMotionProfiler.ForceStop();
            QueuedCommands.Clear();
            startHoming = false;
            homingState = "WaitingForStart";
            AuxAxisManager.StopAllDevices();
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
            if (_config.RobotType == "CNC4Axis") jointSpeed /= 3.0;
            jointJoggingProfiler.Jog(jogJointDirection, jointSpeed, jointAccel, jointDecel);
        }

        public void JogL(Vector6 jogDirection, double? Speed, double? Accel, double? Decel)
        {
            double lineSpeed = Speed ??= this.SpeedS;
            double lineAccel = Accel ??= this.AccelS;
            double lineDecel = Decel ??= this.DecelS;
            if (_config.RobotType == "CNC4Axis") lineSpeed /= 3.0;
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
            if (_config.RobotType == "CNC4Axis") lineSpeed /= 3.0;
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

        public static T LoadParams<T>(CommandMessage msg)
        {
            if (msg.Params == null)
                throw new InvalidOperationException("Command has no params");

            return msg.Params.Value.Deserialize<T>(_jsonOptions)!;
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
