using Controller.RobotControl.AuxAxis;
using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Execution
{
    /// <summary>A silo action that cannot run (not configured, not homed, empty). <see cref="Code"/>
    /// is the program error code (<see cref="ErrorCodes.SiloNotHomed"/> / <see cref="ErrorCodes.SiloEmpty"/>
    /// are catchable, <see cref="ErrorCodes.SiloConfig"/> is not).</summary>
    public sealed class SiloException(string code, string message) : Exception(message)
    {
        public string Code { get; } = code;
    }

    /// <summary>The motion half of a silo action, finished by <see cref="SiloActions.Complete"/> once
    /// the aux move it dispatched has stopped. <paramref name="EndSteps"/> is where the axis ends
    /// when the move runs to its end — a move cut short (stopped) is not completed.</summary>
    public sealed record SiloPending(string DeviceId, int Axis, string Action, long EndSteps);

    /// <summary>
    /// The Silo role's actions (docs/aux-silo.md), shared by the SiloAction program step and the
    /// SiloAction WebSocket command. Each runs in two halves: <see cref="Begin"/> checks, updates the
    /// count where that needs no motion and dispatches the aux move; the caller waits for the axis to
    /// stop and calls <see cref="Complete"/> (Home/Refill zero the axis and mark it homed; Present
    /// takes one part off the count; Accept lowers the stack to receive a part at the rim and adds
    /// one to the count).
    /// </summary>
    public static class SiloActions
    {
        public const string Home     = "Home";
        public const string Present  = "Present";
        public const string Accept   = "Accept";
        public const string Refill   = "Refill";
        public const string SetCount = "SetCount";

        private static readonly string[] All = [Home, Present, Accept, Refill, SetCount];

        /// <summary>The canonical action name for <paramref name="action"/> (any case), or null.</summary>
        public static string? Normalize(string? action) =>
            All.FirstOrDefault(a => string.Equals(a, action?.Trim(), StringComparison.OrdinalIgnoreCase));

        public static bool IsKnown(string? action) => Normalize(action) != null;

        /// <summary>The silo axis's config, checked for the fields a lift needs. Throws
        /// <see cref="SiloException"/> (<see cref="ErrorCodes.SiloConfig"/>) when it is not usable.</summary>
        public static AuxAxisChannelConfig RequireSilo(AuxAxisManager aux, string deviceId, int axis, bool needsMotion)
        {
            var cfg = aux.GetAxisConfig(deviceId, axis)
                      ?? throw new SiloException(ErrorCodes.SiloConfig, $"Aux device '{deviceId}' has no axis {axis}");
            if (cfg.Role != AuxAxisRole.Silo)
                throw new SiloException(ErrorCodes.SiloConfig, $"Aux axis {axis} of '{deviceId}' is not set up as a silo");
            if (!needsMotion) return cfg;
            if (cfg.StepsPerUnit() <= 0 || cfg.AxisType != AuxAxisType.Linear)
                throw new SiloException(ErrorCodes.SiloConfig, $"Silo axis {axis} of '{deviceId}' needs a linear axis type with mm per rev set");
            if (cfg.SiloCapacity <= 0 || cfg.SiloPitchMm <= 0)
                throw new SiloException(ErrorCodes.SiloConfig, $"Silo axis {axis} of '{deviceId}' needs a capacity and a part height");
            return cfg;
        }

        /// <summary>Steps per second for a silo move: home at siloHomeSpeed (else the action
        /// speed); lift at actionVelocity.</summary>
        private static double Speed(AuxAxisChannelConfig cfg, bool homing)
        {
            double spu = cfg.StepsPerUnit();
            if (homing && cfg.SiloHomeSpeed > 0) return Math.Max(1, cfg.SiloHomeSpeed * spu);
            return Math.Max(1, AuxSteps.ActionVel(cfg));
        }

        /// <summary>Home move (steps, signed): down by travel + over-travel. Negative = platform down
        /// (InvertDirection is applied by <see cref="RobotController.StartAuxMove"/>).</summary>
        public static long HomeSteps(AuxAxisChannelConfig cfg) =>
            -(long)Math.Round((cfg.SiloTravelMm() + Math.Max(0, cfg.SiloHomeOverTravelMm)) * cfg.StepsPerUnit());

        // The tracked aux position counts motor steps (an inverted axis counts down while it drives
        // "positive"), so a silo reads and targets it in platform-up steps through this sign.
        private static int UpSign(AuxAxisChannelConfig cfg) => cfg.InvertDirection ? -1 : 1;

        /// <summary>The platform's height above home, in steps (positive = up whatever the motor direction).</summary>
        public static long UpSteps(AuxAxisManager aux, AuxAxisChannelConfig cfg, string deviceId, int axis) =>
            UpSign(cfg) * aux.GetPosition(deviceId, axis);

        /// <summary>Absolute target (steps from home) that presents the top of a <paramref name="partsLeft"/> stack.</summary>
        public static long PresentSteps(AuxAxisChannelConfig cfg, int partsLeft) =>
            (long)Math.Round(cfg.SiloPresentMm(partsLeft) * cfg.StepsPerUnit());

        /// <summary>
        /// Starts <paramref name="action"/> on the silo. Returns the pending half to complete once the
        /// axis stops, or null when nothing moves (SetCount; a Present already in place is completed
        /// here). Throws <see cref="SiloException"/>.
        /// </summary>
        internal static SiloPending? Begin(RobotController ctrl, string deviceId, int axis, string action, int? count)
        {
            var aux = ctrl.AuxAxisManager;
            string act = Normalize(action) ?? throw new SiloException(ErrorCodes.SiloConfig,
                $"Silo action '{action}' must be Home, Present, Accept, Refill or SetCount");

            if (act == SetCount)
            {
                RequireSilo(aux, deviceId, axis, needsMotion: false);
                aux.SetSiloCount(deviceId, axis, count ?? 0);
                return null;
            }

            var cfg = RequireSilo(aux, deviceId, axis, needsMotion: true);
            if (act is Home or Refill)
            {
                if (act == Refill) aux.SetSiloCount(deviceId, axis, cfg.SiloCapacity);
                aux.ClearSiloHomed(deviceId, axis);
                double v = Speed(cfg, homing: true);
                long home = HomeSteps(cfg);
                long end  = aux.GetPosition(deviceId, axis) + UpSign(cfg) * home; // raw tracked steps
                ctrl.StartAuxMove(deviceId, axis, home, v, v * 4, v * 4);
                return new SiloPending(deviceId, axis, act, end);
            }

            if (act == Accept)
            {
                // Receive a part on top of the stack (docs/aux-silo.md): lower the platform to the
                // present height for one more part, so the new part lands at the same rim pose, then
                // count it in once the lift stops.
                if (!aux.IsSiloHomed(deviceId, axis))
                    throw new SiloException(ErrorCodes.SiloNotHomed, $"Silo axis {axis} of '{deviceId}' is not homed — home it first");
                if (cfg.SiloCount >= cfg.SiloCapacity)
                    throw new SiloException(ErrorCodes.SiloFull, $"Silo axis {axis} of '{deviceId}' is full");

                long acceptTarget = PresentSteps(cfg, cfg.SiloCount + 1);
                long acceptDelta  = acceptTarget - UpSteps(aux, cfg, deviceId, axis);
                var acceptPending = new SiloPending(deviceId, axis, act, UpSign(cfg) * acceptTarget);
                if (acceptDelta == 0)
                {
                    Complete(ctrl, acceptPending);
                    return null;
                }
                double acceptVel = Speed(cfg, homing: false);
                ctrl.StartAuxMove(deviceId, axis, acceptDelta, acceptVel, acceptVel * 4, acceptVel * 4);
                aux.MarkSiloHomed(deviceId, axis); // the silo's own move keeps its reference
                return acceptPending;
            }

            // Present
            if (!aux.IsSiloHomed(deviceId, axis))
                throw new SiloException(ErrorCodes.SiloNotHomed, $"Silo axis {axis} of '{deviceId}' is not homed — home it first");
            if (cfg.SiloCount < 1)
                throw new SiloException(ErrorCodes.SiloEmpty, $"Silo axis {axis} of '{deviceId}' is empty");

            long target = PresentSteps(cfg, cfg.SiloCount);
            long delta  = target - UpSteps(aux, cfg, deviceId, axis);
            var pending = new SiloPending(deviceId, axis, act, UpSign(cfg) * target);
            if (delta == 0)
            {
                Complete(ctrl, pending);
                return null;
            }
            double vel = Speed(cfg, homing: false);
            ctrl.StartAuxMove(deviceId, axis, delta, vel, vel * 4, vel * 4);
            aux.MarkSiloHomed(deviceId, axis); // the silo's own move keeps its reference
            return pending;
        }

        /// <summary>The half after the move: Home/Refill zero the axis and mark it homed; Present
        /// takes the presented part off the count. False (nothing changed) when the move did not
        /// reach its end — it was stopped part-way.</summary>
        internal static bool Complete(RobotController ctrl, SiloPending p)
        {
            var aux = ctrl.AuxAxisManager;
            if (Math.Abs(aux.GetPosition(p.DeviceId, p.Axis) - p.EndSteps) > 1) return false;
            if (p.Action is Home or Refill)
            {
                aux.ZeroPosition(p.DeviceId, p.Axis);
                aux.MarkSiloHomed(p.DeviceId, p.Axis);
            }
            else if (p.Action == Present && aux.GetAxisConfig(p.DeviceId, p.Axis) is { } cfg)
            {
                aux.SetSiloCount(p.DeviceId, p.Axis, cfg.SiloCount - 1);
            }
            else if (p.Action == Accept && aux.GetAxisConfig(p.DeviceId, p.Axis) is { } acceptCfg)
            {
                aux.SetSiloCount(p.DeviceId, p.Axis, acceptCfg.SiloCount + 1);
            }
            return true;
        }
    }
}
