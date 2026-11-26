using System;
using System.Collections.Generic;
using System.Linq;
using Controller.RobotControl.Io;

namespace Controller.RobotControl.AuxAxis
{
    /// <summary>
    /// Manages all AuxAxisDevice instances. Loads aux_config.json, starts serial
    /// discovery, and provides a unified motion API to RobotController.
    /// </summary>
    public class AuxAxisManager
    {
        private readonly string              _configPath;
        private AuxAxisManagerConfig         _config;
        private readonly List<AuxAxisDevice> _devices = new();
        // Guards _devices: populated on Start() while GetState() and device lookups can be
        // called concurrently from status-broadcast / motion-command threads.
        private readonly object              _devicesLock = new();

        public AuxAxisManager(string configPath)
        {
            _configPath = configPath;
            _config     = Load();
        }

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            foreach (var cfg in _config.Devices)
            {
                var device = new AuxAxisDevice(cfg);
                device.InputLatched += e => InputLatched?.Invoke(e);
                lock (_devicesLock) _devices.Add(device);
                device.Start();
                foreach (var a in cfg.Axes) ApplyAxisBounds(device, a);
            }
        }

        // A gripper's tracked position is a jaw gap confined to [closed, open]; pin the device's
        // soft bounds there so over-driving a hard stop can't advance the dead-reckoned position
        // past the physical limit. Any other role stays unbounded.
        private static void ApplyAxisBounds(AuxAxisDevice device, AuxAxisChannelConfig a)
        {
            double spu = a.StepsPerUnit();
            if (a.Role == AuxAxisRole.Gripper && spu > 0 && a.GripperOpenMm != a.GripperClosedMm)
            {
                double loMm = Math.Min(a.GripperOpenMm, a.GripperClosedMm);
                double hiMm = Math.Max(a.GripperOpenMm, a.GripperClosedMm);
                device.SetPositionBounds(a.AxisIndex, (long)Math.Round(loMm * spu), (long)Math.Round(hiMm * spu));
            }
            else
            {
                device.ClearPositionBounds(a.AxisIndex);
            }
        }

        public void Stop()
        {
            List<AuxAxisDevice> snapshot;
            lock (_devicesLock) snapshot = new List<AuxAxisDevice>(_devices);
            foreach (var d in snapshot) d.Stop();
        }

        // ── Device lookup ─────────────────────────────────────────────────────

        public AuxAxisDevice? GetDevice(string id)
        {
            lock (_devicesLock) return _devices.FirstOrDefault(d => d.Id == id);
        }

        public AuxAxisDevice? GetFirstDevice()
        {
            lock (_devicesLock) return _devices.Count > 0 ? _devices[0] : null;
        }

        public AuxAxisManagerConfig GetConfig()       => _config;

        public AuxAxisChannelConfig? GetAxisConfig(string deviceId, int axis)
        {
            var dev = _config.Devices.FirstOrDefault(d => d.Id == deviceId);
            return dev?.Axes.FirstOrDefault(a => a.AxisIndex == axis);
        }

        // ── Motion commands ───────────────────────────────────────────────────

        public void StartMove(string deviceId, int axis, long steps, int velocityHz, int accelHz, int decelHz) =>
            GetDevice(deviceId)?.StartMove(axis, steps, velocityHz, accelHz, decelHz);

        public void SetContinuous(string deviceId, int axis, int velocityHz, int accelHz) =>
            GetDevice(deviceId)?.SetContinuous(axis, velocityHz, accelHz);

        public void StopSmooth(string deviceId, int axis, int decelHz) =>
            GetDevice(deviceId)?.StopSmooth(axis, decelHz);

        public bool IsDeviceMoving(string deviceId) =>
            GetDevice(deviceId)?.IsMoving ?? false;

        public void SetDirection(string deviceId, int axis, bool ccw) =>
            GetDevice(deviceId)?.SetDirection(axis, ccw);

        public void StopAll(string deviceId) =>
            GetDevice(deviceId)?.StopAll();

        public void StopAllDevices()
        {
            List<AuxAxisDevice> snapshot;
            lock (_devicesLock) snapshot = new List<AuxAxisDevice>(_devices);
            foreach (var d in snapshot) d.StopAll();
        }

        public void Enable(string deviceId, bool enable) =>
            GetDevice(deviceId)?.Enable(enable);

        public long GetPosition(string deviceId, int axis) =>
            GetDevice(deviceId)?.GetPosition(axis) ?? 0;

        public void ZeroPosition(string deviceId, int axis) =>
            GetDevice(deviceId)?.ZeroPosition(axis);

        /// <summary>Resolve (set, or clear with 0) the tracked step position of an axis without moving it.</summary>
        public void SetPosition(string deviceId, int axis, long steps) =>
            GetDevice(deviceId)?.SetPosition(axis, steps);

        // ── Live position (conveyor tracking) ─────────────────────────────────

        /// <summary>Input edges from every aux device (latch pins with exact board positions, plain
        /// inputs interpolated) — see <see cref="AuxAxisDevice.InputLatched"/>. Raised on the device thread.</summary>
        public event Action<AuxLatchEvent>? InputLatched;

        /// <summary>Live position (steps) + velocity (steps/s) of an axis, extrapolated to now. False when
        /// the device is unknown. Never waits on serial — see <see cref="AuxAxisDevice.TryGetLivePosition"/>.</summary>
        public bool TryGetLivePosition(string deviceId, int axis, out double steps, out double stepsPerSec)
        {
            var dev = GetDevice(deviceId);
            if (dev != null) return dev.TryGetLivePosition(axis, out steps, out stepsPerSec);
            steps = 0; stepsPerSec = 0;
            return false;
        }

        /// <summary>Set a device's POS stream period (ms; 0 = baseline) — see <see cref="AuxAxisDevice.SetPositionStreaming"/>.</summary>
        public void SetPositionStreaming(string deviceId, int periodMs) =>
            GetDevice(deviceId)?.SetPositionStreaming(periodMs);

        // ── State query ───────────────────────────────────────────────────────

        public List<AuxAxisState> GetState()
        {
            List<AuxAxisDevice> snapshot;
            lock (_devicesLock) snapshot = new List<AuxAxisDevice>(_devices);

            var result = new List<AuxAxisState>();
            foreach (var device in snapshot)
            {
                var cfg   = _config.Devices.FirstOrDefault(c => c.Id == device.Id);
                result.Add(new AuxAxisState
                {
                    Connected       = device.Connected,
                    MotorEnabled    = device.MotorEnabled,
                    DeviceId        = device.Id,
                    DeviceName      = device.Name,
                    PortName        = device.PortName,
                    FirmwareVersion = device.Firmware.Version,
                    Capabilities    = device.Firmware.CapabilityArray(),
                    Axes       = cfg?.Axes.Select(a => new AuxAxisChannelState
                    {
                        AxisIndex       = a.AxisIndex,
                        Name            = a.Name,
                        Active          = device.IsMoving,
                        Position        = device.GetPosition(a.AxisIndex),
                        StepsPerRev     = a.StepsPerRev,
                        InvertDirection = a.InvertDirection,
                        AxisType        = a.AxisType,
                        GearRatio       = a.GearRatio,
                        MmPerRev        = a.MmPerRev,
                        Role            = a.Role,
                        ActionDistance  = a.ActionDistance,
                        ActionVelocity  = a.ActionVelocity,
                        GripperOpenMm   = a.GripperOpenMm,
                        GripperClosedMm = a.GripperClosedMm,
                        GripperOpenTimeSec  = a.GripperOpenTimeSec,
                        GripperCloseTimeSec = a.GripperCloseTimeSec,
                        GripperAccel        = a.GripperAccel,
                        GripperOverTravelMm = a.GripperOverTravelMm,
                        SiloPitchMm          = a.SiloPitchMm,
                        SiloCapacity         = a.SiloCapacity,
                        SiloClearanceMm      = a.SiloClearanceMm,
                        SiloHomeOverTravelMm = a.SiloHomeOverTravelMm,
                        SiloHomeSpeed        = a.SiloHomeSpeed,
                        SiloCount            = a.SiloCount,
                        SiloHomed            = IsSiloHomed(device.Id, a.AxisIndex),
                    }).ToList() ?? new(),
                });
            }
            return result;
        }

        public void UpdateAxisConfig(string deviceId, int axisIndex, AuxAxisChannelConfig patch)
        {
            var dev = _config.Devices.FirstOrDefault(d => d.Id == deviceId);
            if (dev == null) return;
            var axis = dev.Axes.FirstOrDefault(a => a.AxisIndex == axisIndex);
            if (axis == null) return;

            axis.Name            = patch.Name;
            axis.StepsPerRev     = patch.StepsPerRev;
            axis.InvertDirection = patch.InvertDirection;
            axis.AxisType        = patch.AxisType;
            axis.GearRatio       = patch.GearRatio;
            axis.MmPerRev        = patch.MmPerRev;
            axis.Role            = patch.Role;
            axis.ActionDistance  = patch.ActionDistance;
            axis.ActionVelocity  = patch.ActionVelocity;
            axis.GripperOpenMm   = patch.GripperOpenMm;
            axis.GripperClosedMm = patch.GripperClosedMm;
            axis.GripperOpenTimeSec  = patch.GripperOpenTimeSec;
            axis.GripperCloseTimeSec = patch.GripperCloseTimeSec;
            axis.GripperAccel        = patch.GripperAccel;
            axis.GripperOverTravelMm = patch.GripperOverTravelMm;
            axis.SiloPitchMm          = patch.SiloPitchMm;
            axis.SiloCapacity         = patch.SiloCapacity;
            axis.SiloClearanceMm      = patch.SiloClearanceMm;
            axis.SiloHomeOverTravelMm = patch.SiloHomeOverTravelMm;
            axis.SiloHomeSpeed        = patch.SiloHomeSpeed;
            // siloCount is runtime inventory, not geometry: a config save keeps it (only clamped to
            // a smaller capacity). SiloAction SetCount / Refill change it.
            if (axis.SiloCapacity > 0) axis.SiloCount = Math.Clamp(axis.SiloCount, 0, axis.SiloCapacity);
            Save(_config);

            // Re-pin the live device's soft bounds — role or gap limits may have just changed.
            var device = GetDevice(deviceId);
            if (device != null) ApplyAxisBounds(device, axis);
        }

        // ── Silo role (docs/aux-silo.md) ────────────────────────────────────────

        // (deviceId, axis) -> the device's reference epoch when the silo was homed. Runtime only:
        // any other move / zero / reconnect of the axis bumps the epoch and so clears "homed".
        private readonly Dictionary<(string, int), long> _siloHomedEpoch = new();
        private readonly object _siloLock = new();

        /// <summary>True when the silo axis was homed and nothing else has moved, zeroed or
        /// reconnected it since.</summary>
        public bool IsSiloHomed(string deviceId, int axis)
        {
            var dev = GetDevice(deviceId);
            if (dev == null) return false;
            lock (_siloLock)
                return _siloHomedEpoch.TryGetValue((deviceId, axis), out long e) && e == dev.ReferenceEpoch(axis);
        }

        /// <summary>Marks the silo homed at the axis's current reference epoch — called after the
        /// home zero, and again right after a silo's own Present move is dispatched (so that move,
        /// unlike any other, keeps it homed).</summary>
        public void MarkSiloHomed(string deviceId, int axis)
        {
            var dev = GetDevice(deviceId);
            if (dev == null) return;
            lock (_siloLock) _siloHomedEpoch[(deviceId, axis)] = dev.ReferenceEpoch(axis);
        }

        /// <summary>Forgets the homed flag of a silo axis.</summary>
        public void ClearSiloHomed(string deviceId, int axis)
        {
            lock (_siloLock) _siloHomedEpoch.Remove((deviceId, axis));
        }

        /// <summary>Sets the persisted part count of a silo axis, clamped to 0…capacity (no upper
        /// clamp while the capacity is unset). Returns the stored count, or null when the axis is unknown.</summary>
        public int? SetSiloCount(string deviceId, int axis, int count)
        {
            var cfg = GetAxisConfig(deviceId, axis);
            if (cfg == null) return null;
            int n = Math.Max(0, count);
            if (cfg.SiloCapacity > 0) n = Math.Min(n, cfg.SiloCapacity);
            if (cfg.SiloCount != n)
            {
                cfg.SiloCount = n;
                Save(_config);
            }
            return n;
        }

        // ── Digital IO (Uno spare pins) ─────────────────────────────────────────

        public bool HasDevice(string deviceId) => GetDevice(deviceId) != null;

        /// <summary>IO device state for every aux device — including one with no IO pins configured
        /// yet (an empty <c>pins</c> list), so the app's "Configure IO" screen can find the device and
        /// add its first pin. Carries the board's firmware version/capabilities like a Nano row.</summary>
        public List<IoDeviceState> GetAllIoStates()
        {
            List<AuxAxisDevice> snapshot;
            lock (_devicesLock) snapshot = new List<AuxAxisDevice>(_devices);

            var result = new List<IoDeviceState>();
            foreach (var dev in snapshot)
            {
                result.Add(new IoDeviceState
                {
                    Id              = dev.Id,
                    Name            = dev.Name,
                    Connected       = dev.Connected,
                    FirmwareVersion = dev.Firmware.Version,
                    Capabilities    = dev.Firmware.CapabilityArray(),
                    Pins            = dev.GetIoPinStates(),
                });
            }
            return result;
        }

        public void SetIoOutput(string deviceId, int pin, bool value) =>
            GetDevice(deviceId)?.SetIoOutput(pin, value);

        /// <summary>Add, update, or remove (type = Unconfigured) an IO pin on the device + config.
        /// <paramref name="latch"/> makes an Input a latch input (edges reported with exact step
        /// counts; plain input on firmware without "latch"). Returns false when the device isn't live.</summary>
        public bool SetIoPinType(string deviceId, int pin, PinType type, bool latch = false)
        {
            latch &= type == PinType.Input;
            var dev = GetDevice(deviceId);
            if (dev == null) return false;
            var cfg = _config.Devices.FirstOrDefault(d => d.Id == deviceId);

            if (type == PinType.Unconfigured)
            {
                dev.RemoveIoPin(pin);
                cfg?.IoPins.RemoveAll(p => p.Pin == pin);
                Save(_config);
                return true;
            }

            dev.UpsertIoPin(pin, type, dev.GetIoPinStates().FirstOrDefault(s => s.Pin == pin)?.Name ?? "", latch);
            dev.ConfigureIoPin(pin, type, latch);
            if (cfg != null)
            {
                var pinCfg = cfg.IoPins.FirstOrDefault(p => p.Pin == pin);
                if (pinCfg != null) { pinCfg.Type = type; pinCfg.Latch = latch; }
                else cfg.IoPins.Add(new IoPinConfig { Pin = pin, Type = type, Name = "", Latch = latch });
                Save(_config);
            }
            return true;
        }

        public bool RenameIoPin(string deviceId, int pin, string name)
        {
            var dev = GetDevice(deviceId);
            if (dev == null || !dev.RenameIoPin(pin, name)) return false;
            var pinCfg = _config.Devices.FirstOrDefault(d => d.Id == deviceId)?.IoPins.FirstOrDefault(p => p.Pin == pin);
            if (pinCfg != null) { pinCfg.Name = name; Save(_config); }
            return true;
        }

        // ── Config persistence ────────────────────────────────────────────────

        private AuxAxisManagerConfig Load()
        {
            var loaded = Persistence.JsonFiles.Load<AuxAxisManagerConfig>(_configPath, logTag: "AuxAxisManager");
            if (loaded != null) return loaded;

            var def = DefaultConfig();
            Save(def);
            return def;
        }

        private void Save(AuxAxisManagerConfig config)
        {
            try { Persistence.JsonFiles.Save(_configPath, config); }
            catch (Exception ex) { Console.WriteLine($"[AuxAxisManager] Failed to save config: {ex.Message}"); }
        }

        private static AuxAxisManagerConfig DefaultConfig() => new()
        {
            Devices = new List<AuxAxisConfig>
            {
                new()
                {
                    Id   = "AUX_STEPPER_001",
                    Name = "Aux Stepper",
                    Axes = new List<AuxAxisChannelConfig>
                    {
                        new() { AxisIndex = 0, Name = "Axis 0 (X)", StepsPerRev = 1600, InvertDirection = false },
                        new() { AxisIndex = 1, Name = "Axis 1 (Y)", StepsPerRev = 1600, InvertDirection = false },
                        new() { AxisIndex = 2, Name = "Axis 2 (Z)", StepsPerRev = 1600, InvertDirection = false },
                        new() { AxisIndex = 3, Name = "Axis 3 (A)", StepsPerRev = 1600, InvertDirection = false },
                    }
                }
            }
        };
    }
}
