using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;

namespace Controller.RobotControl.Camera
{
    public class CameraManager
    {
        private readonly string              _configPath;
        private CameraManagerConfig          _config;
        private readonly List<CameraDevice>  _devices = new();

        private static readonly JsonSerializerOptions _json = new()
        {
            PropertyNameCaseInsensitive = true,
            WriteIndented               = true,
        };

        public CameraManager(string configPath)
        {
            _configPath = configPath;
            _config     = Load();
        }

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            foreach (var cfg in _config.Cameras)
            {
                var device = new CameraDevice(cfg);
                _devices.Add(device);
                device.Start();
            }
        }

        public void Stop()
        {
            foreach (var d in _devices) d.Stop();
        }

        // ── Device lookup ─────────────────────────────────────────────────────

        public CameraDevice? GetCamera(string id) =>
            _devices.FirstOrDefault(d => d.Id == id);

        // ── State query ───────────────────────────────────────────────────────

        public List<CameraState> GetState() =>
            _devices.Select(d => d.GetState()).ToList();

        // ── Config mutations ──────────────────────────────────────────────────

        public void AddCamera(CameraConfig cfg)
        {
            if (string.IsNullOrEmpty(cfg.Id))
                cfg.Id = $"CAM_{_config.Cameras.Count}";

            _config.Cameras.Add(cfg);
            Save();

            var device = new CameraDevice(cfg);
            _devices.Add(device);
            device.Start();
        }

        public void RemoveCamera(string id)
        {
            var device = _devices.FirstOrDefault(d => d.Id == id);
            if (device != null)
            {
                device.Stop();
                _devices.Remove(device);
            }

            _config.Cameras.RemoveAll(c => c.Id == id);
            Save();
        }

        public void UpdateCamera(string id, CameraConfig patch)
        {
            patch.Id = id;

            var existing = _config.Cameras.FirstOrDefault(c => c.Id == id);
            if (existing != null)
            {
                var idx = _config.Cameras.IndexOf(existing);
                _config.Cameras[idx] = patch;
            }

            var device = _devices.FirstOrDefault(d => d.Id == id);
            if (device != null)
            {
                bool wasEnabled = device.Enabled;
                device.ApplyConfig(patch);

                // Restart capture thread to pick up resolution / device index changes
                device.Stop();
                device.Start();
            }

            Save();
        }

        // ── Persistence ───────────────────────────────────────────────────────

        private CameraManagerConfig Load()
        {
            try
            {
                if (File.Exists(_configPath))
                {
                    var text = File.ReadAllText(_configPath);
                    return JsonSerializer.Deserialize<CameraManagerConfig>(text, _json)
                           ?? new CameraManagerConfig();
                }
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[CameraManager] Failed to load config: {ex.Message}");
            }
            return new CameraManagerConfig();
        }

        private void Save()
        {
            try { File.WriteAllText(_configPath, JsonSerializer.Serialize(_config, _json)); }
            catch (Exception ex) { Console.WriteLine($"[CameraManager] Failed to save config: {ex.Message}"); }
        }
    }
}
