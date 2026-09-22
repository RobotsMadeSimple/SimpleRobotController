using System;
using System.Collections.Generic;
using System.Linq;

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
                lock (_devicesLock) _devices.Add(device);
                device.Start();
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
                    Connected    = device.Connected,
                    MotorEnabled = device.MotorEnabled,
                    DeviceId     = device.Id,
                    DeviceName   = device.Name,
                    PortName     = device.PortName,
                    Axes       = cfg?.Axes.Select(a => new AuxAxisChannelState
                    {
                        AxisIndex       = a.AxisIndex,
                        Name            = a.Name,
                        Active          = false,
                        StepsPerRev     = a.StepsPerRev,
                        InvertDirection = a.InvertDirection,
                        AxisType        = a.AxisType,
                        GearRatio       = a.GearRatio,
                        MmPerRev        = a.MmPerRev,
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
            Save(_config);
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
