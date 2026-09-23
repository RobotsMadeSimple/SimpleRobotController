namespace Controller.RobotControl.Execution
{
    /// <summary>The live state of one program run, as the <c>$program.*</c> properties expose it.</summary>
    internal interface IProgramRunInfo
    {
        /// <summary>How many times this program has been started since the controller booted (this run included).</summary>
        int  RunCount  { get; }
        /// <summary>Completed top-level steps so far (the monitor's step number).</summary>
        int  StepIndex { get; }
        /// <summary>Total steps the run reports (MaxStepCount).</summary>
        int  StepCount { get; }
        /// <summary>Milliseconds since the run started; 0 when not running.</summary>
        long ElapsedMs { get; }
        /// <summary>Number of loop bodies currently executing (0 at top level).</summary>
        int  LoopDepth { get; }
    }

    /// <summary>
    /// The read-only system properties of the expression language — <c>$robot.*</c>,
    /// <c>$program.*</c>, <c>$time.*</c> and <c>$aux.*</c> (docs/expressions-and-variables.md §3).
    /// </summary>
    /// <remarks>
    /// Every value is read at lookup time, so nothing is added to the per-tick variable
    /// snapshot; a program that never mentions a property pays nothing for it. Names are
    /// case-insensitive. Robot reads are the same lock-free snapshots GetStatus uses.
    /// </remarks>
    internal sealed class RobotPropertySource : IPropertySource
    {
        private readonly RobotController? _robot;
        private readonly IProgramRunInfo? _run;

        /// <param name="robot">Null in tests / offline: <c>$robot.*</c> and <c>$aux.*</c> are then unknown.</param>
        /// <param name="run">Null when evaluating outside a program: <c>$program.*</c> reads 0.</param>
        public RobotPropertySource(RobotController? robot, IProgramRunInfo? run)
        {
            _robot = robot;
            _run   = run;
        }

        private enum Group { Robot, Program, Time }

        private sealed record Prop(string Name, string Description, string Type, Group Group,
                                   Func<RobotController, double>? Robot = null,
                                   Func<IProgramRunInfo, double>? Program = null,
                                   Func<double>? Time = null);

        private static double B(bool b) => b ? 1 : 0;

        private static readonly Prop[] Props =
        [
            new("robot.x",  "Current TCP X (mm)",   "number", Group.Robot, r => r.LivePosition.X),
            new("robot.y",  "Current TCP Y (mm)",   "number", Group.Robot, r => r.LivePosition.Y),
            new("robot.z",  "Current TCP Z (mm)",   "number", Group.Robot, r => r.LivePosition.Z),
            new("robot.rx", "Current TCP RX (deg)", "number", Group.Robot, r => r.LivePosition.RX),
            new("robot.ry", "Current TCP RY (deg)", "number", Group.Robot, r => r.LivePosition.RY),
            new("robot.rz", "Current TCP RZ (deg)", "number", Group.Robot, r => r.LivePosition.RZ),
            new("robot.targetX",  "Commanded target X (mm)",   "number", Group.Robot, r => r.LiveTargetPosition.X),
            new("robot.targetY",  "Commanded target Y (mm)",   "number", Group.Robot, r => r.LiveTargetPosition.Y),
            new("robot.targetZ",  "Commanded target Z (mm)",   "number", Group.Robot, r => r.LiveTargetPosition.Z),
            new("robot.targetRx", "Commanded target RX (deg)", "number", Group.Robot, r => r.LiveTargetPosition.RX),
            new("robot.targetRy", "Commanded target RY (deg)", "number", Group.Robot, r => r.LiveTargetPosition.RY),
            new("robot.targetRz", "Commanded target RZ (deg)", "number", Group.Robot, r => r.LiveTargetPosition.RZ),
            new("robot.moving",          "1 while the robot is moving",          "boolean", Group.Robot, r => B(r.IsMoving)),
            new("robot.homed",           "1 once the robot has been homed",      "boolean", Group.Robot, r => B(r.Homed)),
            new("robot.faulted",         "1 while a joint-limit fault is active", "boolean", Group.Robot, r => B(r.FaultStatus.Faulted)),
            new("robot.driverConnected", "1 while the motor driver is connected", "boolean", Group.Robot, r => B(r.stb.connected)),
            new("robot.speedS", "Current linear speed default (mm/s)",      "number", Group.Robot, r => r.MotionParameters.SpeedS),
            new("robot.accelS", "Current linear accel default (mm/s²)",     "number", Group.Robot, r => r.MotionParameters.AccelS),
            new("robot.decelS", "Current linear decel default (mm/s²)",     "number", Group.Robot, r => r.MotionParameters.DecelS),
            new("robot.speedJ", "Current joint speed default",              "number", Group.Robot, r => r.MotionParameters.SpeedJ),
            new("robot.accelJ", "Current joint accel default",              "number", Group.Robot, r => r.MotionParameters.AccelJ),
            new("robot.decelJ", "Current joint decel default",              "number", Group.Robot, r => r.MotionParameters.DecelJ),
            new("robot.speedOverride", "Speed override (percent)",          "number", Group.Robot, r => r.SpeedOverrideFactor * 100.0),
            new("robot.joint1",  "Joint 1 angle (deg)",  "number", Group.Robot, r => r.Kinematics.GetJointAngles().joint1),
            new("robot.joint2x", "Joint 2 X readout",    "number", Group.Robot, r => r.Kinematics.GetJointAngles().joint2x),
            new("robot.joint2z", "Joint 2 Z readout",    "number", Group.Robot, r => r.Kinematics.GetJointAngles().joint2z),
            new("robot.joint4",  "Joint 4 angle (deg)",  "number", Group.Robot, r => r.Kinematics.GetJointAngles().joint4),

            new("program.runCount",  "Times this program has been started (this run included)", "number", Group.Program, Program: p => p.RunCount),
            new("program.stepIndex", "Completed top-level steps in this run",   "number", Group.Program, Program: p => p.StepIndex),
            new("program.stepCount", "Total steps in this run",                 "number", Group.Program, Program: p => p.StepCount),
            new("program.elapsedMs", "Milliseconds since this run started",     "number", Group.Program, Program: p => p.ElapsedMs),
            new("program.loopDepth", "Number of loops currently executing",     "number", Group.Program, Program: p => p.LoopDepth),

            new("time.now",       "Unix time in milliseconds",           "number", Group.Time, Time: () => DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()),
            new("time.hour",      "Local hour (0–23)",                   "number", Group.Time, Time: () => DateTime.Now.Hour),
            new("time.minute",    "Local minute (0–59)",                 "number", Group.Time, Time: () => DateTime.Now.Minute),
            new("time.second",    "Local second (0–59)",                 "number", Group.Time, Time: () => DateTime.Now.Second),
            new("time.dayOfWeek", "Day of the week, 0 = Sunday",         "number", Group.Time, Time: () => (int)DateTime.Now.DayOfWeek),
            new("time.dayOfYear", "Day of the year (1–366)",             "number", Group.Time, Time: () => DateTime.Now.DayOfYear),
        ];

        private static readonly Dictionary<string, Prop> ByName =
            Props.ToDictionary(p => p.Name, StringComparer.OrdinalIgnoreCase);

        /// <summary>The fixed property names (everything except <c>$aux.*</c>, which depends on configured devices).</summary>
        public static IEnumerable<string> StaticNames => Props.Select(p => p.Name);

        /// <summary>The first segment of every property name — a <c>$name.…</c> starting with one of these is a property reference.</summary>
        public static readonly IReadOnlySet<string> Roots =
            new HashSet<string>(["robot", "program", "time", "aux"], StringComparer.OrdinalIgnoreCase);

        public bool TryGet(string name, out double value)
        {
            if (ByName.TryGetValue(name, out var p))
            {
                switch (p.Group)
                {
                    case Group.Robot:
                        if (_robot == null) break;
                        value = p.Robot!(_robot);
                        return true;
                    case Group.Program:
                        value = _run == null ? 0 : p.Program!(_run);
                        return true;
                    case Group.Time:
                        value = p.Time!();
                        return true;
                }
                value = 0;
                return false;
            }

            if (name.StartsWith("aux.", StringComparison.OrdinalIgnoreCase))
                return TryGetAux(name, out value);

            value = 0;
            return false;
        }

        // $aux.<deviceId>.<axisIndex>.position  and  $aux.<deviceId>.moving
        private bool TryGetAux(string name, out double value)
        {
            value = 0;
            if (_robot == null) return false;
            var parts = name.Split('.');
            if (parts.Length < 3) return false;

            var deviceId = ResolveDeviceId(parts[1]);
            if (deviceId == null) return false;
            var mgr = _robot.AuxAxisManager;

            if (parts.Length == 3 && parts[2].Equals("moving", StringComparison.OrdinalIgnoreCase))
            {
                value = B(mgr.IsDeviceMoving(deviceId));
                return true;
            }
            if (parts.Length == 4 && parts[3].Equals("position", StringComparison.OrdinalIgnoreCase)
                && int.TryParse(parts[2], out int axis)
                && mgr.GetAxisConfig(deviceId, axis) != null)
            {
                value = mgr.GetPosition(deviceId, axis);
                return true;
            }
            return false;
        }

        /// <summary>The configured device id matching <paramref name="id"/> case-insensitively, or null.</summary>
        private string? ResolveDeviceId(string id)
        {
            foreach (var d in _robot!.AuxAxisManager.GetConfig().Devices)
                if (string.Equals(d.Id, id, StringComparison.OrdinalIgnoreCase)) return d.Id;
            return null;
        }

        public IEnumerable<(string Name, string Description, string Type)> List()
        {
            foreach (var p in Props)
                if (p.Group != Group.Robot || _robot != null)
                    yield return (p.Name, p.Description, p.Type);

            if (_robot == null) yield break;
            foreach (var d in _robot.AuxAxisManager.GetConfig().Devices)
            {
                yield return ($"aux.{d.Id}.moving", $"1 while aux device '{d.Name}' is moving", "boolean");
                foreach (var a in d.Axes)
                    yield return ($"aux.{d.Id}.{a.AxisIndex}.position",
                                  $"Aux '{d.Name}' axis {a.AxisIndex}{(string.IsNullOrEmpty(a.Name) ? "" : $" ({a.Name})")} position (steps)",
                                  "number");
            }
        }
    }
}
