using System.Collections.Concurrent;

namespace Controller.RobotControl.Serial
{
    /// <summary>
    /// Coordinates access to serial ports across the independent device discovery
    /// scanners (Nano, Aux axis, …).
    ///
    /// Each scanner runs on its own thread and, when searching, opens every serial
    /// port to send an "ID?" probe. Opening a port toggles DTR, which resets the
    /// Arduino on the other end. Without coordination a scanner still hunting for a
    /// missing device would repeatedly open — and therefore reset — a port that a
    /// different, already-connected device is actively using. That reset reboots the
    /// Arduino into its default (motors disabled) state, which is exactly the
    /// "enabled for a second then disables" symptom on the aux stepper.
    ///
    /// This registry lets a device claim its port once connected so other scanners
    /// skip it, and provides a per-port lock so two scanners never open the same
    /// port simultaneously.
    /// </summary>
    public static class SerialPortRegistry
    {
        // port name -> owner id of the connected device using it
        private static readonly ConcurrentDictionary<string, string> _claimed = new();

        // port name -> lock object, so probes on the same port are serialized
        private static readonly ConcurrentDictionary<string, object> _locks = new();

        /// <summary>A per-port gate so two scanners never probe one port at once.</summary>
        public static object LockFor(string port) => _locks.GetOrAdd(port, _ => new object());

        /// <summary>True when the port is currently owned by a different device.</summary>
        public static bool IsClaimedByOther(string port, string ownerId) =>
            _claimed.TryGetValue(port, out var owner) && owner != ownerId;

        /// <summary>Mark a port as owned by a connected device.</summary>
        public static void Claim(string port, string ownerId) => _claimed[port] = ownerId;

        /// <summary>Release a port on disconnect (only if we still own it).</summary>
        public static void Release(string port, string ownerId)
        {
            if (_claimed.TryGetValue(port, out var owner) && owner == ownerId)
                _claimed.TryRemove(port, out _);
        }
    }
}
