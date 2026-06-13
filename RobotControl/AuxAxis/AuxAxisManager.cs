using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

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

        private static readonly JsonSerializerOptions _json = new()
        {
            Converters               = { new JsonStringEnumConverter() },
            PropertyNameCaseInsensitive = true,
            WriteIndented            = true,
        };

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
                _devices.Add(device);
                device.Start();
            }
        }

        public void Stop()
        {
            foreach (var d in _devices) d.Stop();
        }

        // ── Device lookup ─────────────────────────────────────────────────────

        public AuxAxisDevice? GetDevice(string id)    => _devices.FirstOrDefault(d => d.Id == id);
        public AuxAxisDevice? GetFirstDevice()        => _devices.Count > 0 ? _devices[0] : null;
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
            foreach (var d in _devices) d.StopAll();
        }

        // ── State query ───────────────────────────────────────────────────────

        public List<AuxAxisState> GetState()
        {
            var result = new List<AuxAxisState>();
            foreach (var device in _devices)
            {
                var cfg   = _config.Devices.FirstOrDefault(c => c.Id == device.Id);
                result.Add(new AuxAxisState
                {
                    Connected  = device.Connected,
                    DeviceId   = device.Id,
                    DeviceName = device.Name,
                    PortName   = device.PortName,
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
            try
            {
                if (File.Exists(_configPath))
                {
                    var text = File.ReadAllText(_configPath);
                    return JsonSerializer.Deserialize<AuxAxisManagerConfig>(text, _json)
                           ?? DefaultConfig();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[AuxAxisManager] Failed to load config: {ex.Message}");
            }

            var def = DefaultConfig();
            Save(def);
            return def;
        }

        private void Save(AuxAxisManagerConfig config)
        {
            try { File.WriteAllText(_configPath, JsonSerializer.Serialize(config, _json)); }
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
