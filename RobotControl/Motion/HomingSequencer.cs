namespace Controller.RobotControl;

/// <summary>
/// One step of the homing sequence. Axes with a limit switch run
/// Approach → WaitStop → BackOff → WaitBackoff → ApproachSlow → WaitMoveComplete → SetHomed;
/// an axis without a switch either drives to joint zero (DriveToZero → WaitMoveComplete →
/// SetHomed) or is simply declared homed where it stands (SetHomed).
/// </summary>
internal enum HomingPhase
{
    WaitingForStart,
    Approach,
    WaitStop,
    BackOff,
    WaitBackoff,
    ApproachSlow,
    WaitMoveComplete,
    SetHomed,
    DriveToZero,
    Complete,
}

/// <summary>
/// One axis of the homing sequence.
/// </summary>
/// <param name="Joint">Joint-space component to drive: 0 = X, 1 = Y, 2 = Z, 5 = RZ (Vector6 order).</param>
/// <param name="Direction">Jog direction toward the switch (±1).</param>
/// <param name="Sensor">Limit switch, or null for an axis without one.</param>
/// <param name="FastSpeed">Approach speed; also the back-off and drive-to-zero move speed.</param>
/// <param name="SlowSpeed">Second, precise approach speed.</param>
/// <param name="BackoffDistance">Distance backed off the switch between the two approaches.</param>
/// <param name="SetHomed">Declares the axis homed at its configured home position.</param>
/// <param name="StateNames">Legacy per-phase state names reported as <c>homingState</c>, indexed by <see cref="HomingPhase"/>.</param>
/// <param name="DriveToZero">Switchless axis that is first driven to joint value 0.</param>
internal sealed record HomingAxis(
    int Joint,
    double Direction,
    Func<bool>? Sensor,
    double FastSpeed,
    double SlowSpeed,
    double BackoffDistance,
    Action SetHomed,
    string?[] StateNames,
    bool DriveToZero = false)
{
    public HomingPhase FirstPhase =>
        Sensor != null ? HomingPhase.Approach
        : DriveToZero  ? HomingPhase.DriveToZero
        :                HomingPhase.SetHomed;

    /// <summary>ASTRO naming: HomeVertical, WaitVerticalStop1, BackOffVertical, …</summary>
    public static string?[] AstroNames(string axis, string? setHomedName = null) => Names(
        approach:     $"Home{axis}",
        waitStop:     $"Wait{axis}Stop1",
        backOff:      $"BackOff{axis}",
        waitBackoff:  $"Wait{axis}Backoff",
        approachSlow: $"Home{axis}Slow",
        waitMove:     $"Wait{axis}MoveComplete",
        setHomed:     setHomedName ?? $"Set{axis}Homed");

    /// <summary>CNC naming: CNC_HomeZ, CNC_WaitZStop, CNC_BackOffZ, …</summary>
    public static string?[] CncNames(string axis) => Names(
        approach:     $"CNC_Home{axis}",
        waitStop:     $"CNC_Wait{axis}Stop",
        backOff:      $"CNC_BackOff{axis}",
        waitBackoff:  $"CNC_Wait{axis}Backoff",
        approachSlow: $"CNC_Home{axis}Slow",
        waitMove:     $"CNC_Wait{axis}MoveDone",
        setHomed:     $"CNC_Set{axis}Homed");

    public static string?[] Names(string? approach = null, string? waitStop = null, string? backOff = null,
        string? waitBackoff = null, string? approachSlow = null, string? waitMove = null,
        string? setHomed = null, string? driveToZero = null)
    {
        var names = new string?[Enum.GetValues<HomingPhase>().Length];
        names[(int)HomingPhase.Approach]         = approach;
        names[(int)HomingPhase.WaitStop]         = waitStop;
        names[(int)HomingPhase.BackOff]          = backOff;
        names[(int)HomingPhase.WaitBackoff]      = waitBackoff;
        names[(int)HomingPhase.ApproachSlow]     = approachSlow;
        names[(int)HomingPhase.WaitMoveComplete] = waitMove;
        names[(int)HomingPhase.SetHomed]         = setHomed;
        names[(int)HomingPhase.DriveToZero]      = driveToZero;
        return names;
    }
}

/// <summary>Motion-thread operations the homing sequencer drives.</summary>
internal interface IHomingHost
{
    bool IsMoving { get; }

    /// <summary>Current commanded joint targets (read only by the sequencer).</summary>
    Vector6 JointTargets { get; }

    /// <summary>Re-arm the joint jog profiler (re-issued every tick while approaching).</summary>
    void JogJoints(Vector6 direction, double speed, double accel, double decel, double watchdogSeconds);

    /// <summary>Start a profiled joint move to <paramref name="target"/>.</summary>
    void MoveJoints(Vector6 target, double speed, double accel, double decel);

    /// <summary>Immediate stop at the switch (the controller's hard stop). Resets the sequencer.</summary>
    void StopAtSwitch();
}

/// <summary>
/// Table-driven homing state machine. Ticked on the motion thread only; exactly
/// one phase transition per tick, so the tick-by-tick behaviour matches the
/// original hand-written state machine. <see cref="StateName"/> is safe to read
/// from any thread.
/// </summary>
internal sealed class HomingSequencer
{
    public const string IdleStateName     = "WaitingForStart";
    public const string CompleteStateName = "HomingComplete";

    // ── Tuning (unchanged from the original state machine) ─────────────────
    // Fast approach: HomingSpeed, accel 100.
    public const double ApproachAccel     = 100;
    // Slow approach: HomingSlowSpeed, accel 50.
    public const double SlowApproachAccel = 50;
    // Jog decel — effectively instantaneous; the hard stop at the switch does the real stop.
    public const double JogDecel          = 10000000;
    // The jog profiler's watchdog reset time. The approach re-issues its jog every
    // tick, but RunMotion (which advances the profiler) runs BEFORE the next
    // RunHoming, so the watchdog MUST exceed one motion tick or it expires between
    // the Jog and the next Update and the axis never moves (flicker, no motion).
    // It used to be 0.001 s, which only worked when the loop was a tight busy-spin.
    // The hard stop handles the precise stop at the switch; this is a safety net.
    public const double JogWatchdogSeconds = 0.1;
    // Back-off and drive-to-zero joint moves: HomingSpeed, accel 100, decel 200.
    public const double MoveAccel = 100;
    public const double MoveDecel = 200;

    private readonly IHomingHost _host;
    private IReadOnlyList<HomingAxis> _axes = Array.Empty<HomingAxis>();
    private int _axis;

    // Written only on the motion thread; read cross-thread for status.
    private volatile HomingPhase _phase = HomingPhase.WaitingForStart;
    private volatile string _stateName = IdleStateName;

    public HomingSequencer(IHomingHost host) => _host = host;

    public HomingPhase Phase => _phase;

    /// <summary>True from the first homing tick until the sequence finishes or is hard-stopped.</summary>
    public bool IsActive => _phase != HomingPhase.WaitingForStart;

    /// <summary>The legacy string state (e.g. "HomeVertical", "CNC_WaitZStop", "WaitingForStart").</summary>
    public string StateName => _stateName;

    /// <summary>Starts the sequence at the first axis's first phase.</summary>
    public void Begin(IReadOnlyList<HomingAxis> axes)
    {
        _axes = axes;
        _axis = 0;
        EnterAxisOrComplete();
    }

    /// <summary>Back to idle (hard stop, or after completion). Keeps the axis table.</summary>
    public void Reset() => SetPhase(HomingPhase.WaitingForStart);

    /// <summary>Advance one phase. Call only while <see cref="IsActive"/> and not Complete.</summary>
    public void Tick()
    {
        int index = _axis;
        var axis  = _axes[index];

        switch (_phase)
        {
            case HomingPhase.Approach:
                if (axis.Sensor!())
                {
                    // Switch already made — skip the fast approach and go straight to back-off.
                    SetPhase(HomingPhase.BackOff);
                    break;
                }
                _host.JogJoints(JogVector(axis), axis.FastSpeed, ApproachAccel, JogDecel, JogWatchdogSeconds);
                if (axis.Sensor())
                    StopAtSwitchThen(index, HomingPhase.WaitStop);
                break;

            case HomingPhase.WaitStop:
                if (!_host.IsMoving)
                    SetPhase(HomingPhase.BackOff);
                break;

            case HomingPhase.BackOff:
            {
                var current = _host.JointTargets;
                var target  = WithJoint(current, axis.Joint,
                    Get(current, axis.Joint) - (axis.BackoffDistance * axis.Direction));
                _host.MoveJoints(target, axis.FastSpeed, MoveAccel, MoveDecel);
                SetPhase(HomingPhase.WaitBackoff);
                break;
            }

            case HomingPhase.WaitBackoff:
                if (!_host.IsMoving)
                    SetPhase(HomingPhase.ApproachSlow);
                break;

            case HomingPhase.ApproachSlow:
                _host.JogJoints(JogVector(axis), axis.SlowSpeed, SlowApproachAccel, JogDecel, JogWatchdogSeconds);
                if (axis.Sensor!())
                    StopAtSwitchThen(index, HomingPhase.WaitMoveComplete);
                break;

            case HomingPhase.DriveToZero:
            {
                // The host sets TargetJoints before creating the profiler — RunMotion snaps
                // the joint targets to it on the tick the profiler finishes. Without that,
                // the snap would restore stale pre-homing joint values and corrupt the home
                // positions already set for the other axes.
                var target = WithJoint(_host.JointTargets, axis.Joint, 0);
                _host.MoveJoints(target, axis.FastSpeed, MoveAccel, MoveDecel);
                SetPhase(HomingPhase.WaitMoveComplete);
                break;
            }

            case HomingPhase.WaitMoveComplete:
                if (!_host.IsMoving)
                    SetPhase(HomingPhase.SetHomed);
                break;

            case HomingPhase.SetHomed:
                axis.SetHomed();
                _axis = index + 1;
                EnterAxisOrComplete();
                break;
        }
    }

    // The hard stop resets the sequencer to idle; restore our position afterwards.
    private void StopAtSwitchThen(int index, HomingPhase next)
    {
        _host.StopAtSwitch();
        _axis = index;
        SetPhase(next);
    }

    private void EnterAxisOrComplete()
    {
        if (_axis < _axes.Count)
            SetPhase(_axes[_axis].FirstPhase);
        else
            SetPhase(HomingPhase.Complete);
    }

    private void SetPhase(HomingPhase phase)
    {
        _stateName = phase switch
        {
            HomingPhase.WaitingForStart => IdleStateName,
            HomingPhase.Complete        => CompleteStateName,
            _                           => _axes[_axis].StateNames[(int)phase] ?? phase.ToString(),
        };
        _phase = phase;
    }

    private static Vector6 JogVector(HomingAxis axis)
    {
        var v = new Vector6();
        Set(v, axis.Joint, axis.Direction);
        return v;
    }

    private static Vector6 WithJoint(Vector6 v, int joint, double value)
    {
        var r = new Vector6(v.X, v.Y, v.Z, v.RX, v.RY, v.RZ);
        Set(r, joint, value);
        return r;
    }

    private static double Get(Vector6 v, int joint) => joint switch
    {
        0 => v.X, 1 => v.Y, 2 => v.Z, 3 => v.RX, 4 => v.RY, 5 => v.RZ,
        _ => throw new ArgumentOutOfRangeException(nameof(joint)),
    };

    private static void Set(Vector6 v, int joint, double value)
    {
        switch (joint)
        {
            case 0: v.X  = value; break;
            case 1: v.Y  = value; break;
            case 2: v.Z  = value; break;
            case 3: v.RX = value; break;
            case 4: v.RY = value; break;
            case 5: v.RZ = value; break;
            default: throw new ArgumentOutOfRangeException(nameof(joint));
        }
    }
}
