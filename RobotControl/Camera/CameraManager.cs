using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text.Json;
using OpenCvSharp;

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
                WireDevice(device);
                _devices.Add(device);
                device.Start();
            }
        }

        private void WireDevice(CameraDevice device)
        {
            device.OnResolutionsDetected = resolutions =>
            {
                var cfg = _config.Cameras.FirstOrDefault(c => c.Id == device.Id);
                if (cfg != null)
                {
                    cfg.SupportedResolutions = resolutions;
                    Save();
                    Console.WriteLine($"[Camera] {device.Id} auto-detected {resolutions.Count} resolution(s)");
                }
            };
        }

        public void Stop()
        {
            foreach (var d in _devices) d.Stop();
        }

        // ── Device lookup ─────────────────────────────────────────────────────

        public CameraDevice? GetCamera(string id) =>
            _devices.FirstOrDefault(d => d.Id == id);

        public CameraDevice? GetFirstCamera() =>
            _devices.FirstOrDefault();

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
            WireDevice(device);
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

        // ── Resolution probing ────────────────────────────────────────────────

        /// <summary>
        /// Probes the camera at the given device index.
        /// - If a CameraDevice is already open at that index, pauses it and uses its
        ///   capture (avoids the MSMF double-open conflict).
        /// - If a device is configured but not yet connected, returns [] so the caller
        ///   can tell the user to wait for the camera to connect.
        /// - If no device is configured, falls back to a one-shot static probe.
        /// </summary>
        public List<CameraResolution> ProbeResolutionsForIndex(int deviceIndex)
        {
            var device = _devices.FirstOrDefault(d => d.DeviceIndex == deviceIndex);
            List<CameraResolution> resolutions;

            if (device != null)
            {
                // Connected → use the open capture (fast, no double-open conflict)
                // Not connected → returns [] immediately (MSMF still initialising)
                resolutions = device.ProbeResolutions();
            }
            else
            {
                resolutions = GetSupportedResolutions(deviceIndex);
            }

            if (resolutions.Count > 0 && device != null)
            {
                device.SupportedResolutions = resolutions;
                var cfg = _config.Cameras.FirstOrDefault(c => c.Id == device.Id);
                if (cfg != null) { cfg.SupportedResolutions = resolutions; Save(); }
            }

            return resolutions;
        }

        private static readonly (int W, int H)[] _commonResolutions =
        {
            (160, 120), (320, 240), (640, 480), (800, 600),
            (1024, 768), (1280, 720), (1280, 960), (1920, 1080),
            (2560, 1440), (3840, 2160),
        };

        public static List<CameraResolution> GetSupportedResolutions(int deviceIndex)
        {
            if (RuntimeInformation.IsOSPlatform(OSPlatform.Linux))
            {
                var linuxResults = ProbeResolutionsLinux(deviceIndex);
                if (linuxResults.Count > 0) return linuxResults;
            }
            return ProbeResolutionsOpenCv(deviceIndex);
        }

        private static List<CameraResolution> ProbeResolutionsLinux(int deviceIndex)
        {
            var results = new List<CameraResolution>();
            try
            {
                var psi = new ProcessStartInfo
                {
                    FileName               = "v4l2-ctl",
                    Arguments              = $"--device=/dev/video{deviceIndex} --list-formats-ext",
                    RedirectStandardOutput = true,
                    RedirectStandardError  = true,
                    UseShellExecute        = false,
                    CreateNoWindow         = true,
                };
                using var proc = Process.Start(psi);
                if (proc == null) return results;

                var output = proc.StandardOutput.ReadToEnd();
                proc.WaitForExit(5000);

                foreach (var line in output.Split('\n'))
                {
                    var t = line.Trim();
                    if (!t.StartsWith("Size: Discrete ")) continue;
                    var dim   = t["Size: Discrete ".Length..].Trim();
                    var parts = dim.Split('x');
                    if (parts.Length == 2
                        && int.TryParse(parts[0], out var w)
                        && int.TryParse(parts[1], out var h)
                        && !results.Any(r => r.Width == w && r.Height == h))
                    {
                        results.Add(new CameraResolution { Width = w, Height = h });
                    }
                }
            }
            catch { }
            return results;
        }

        private static List<CameraResolution> ProbeResolutionsOpenCv(int deviceIndex)
        {
            var results = new List<CameraResolution>();
            VideoCapture? cap = null;
            bool opened = false;
            try
            {
                var api = RuntimeInformation.IsOSPlatform(OSPlatform.Windows)
                    ? VideoCaptureAPIs.DSHOW
                    : VideoCaptureAPIs.V4L2;

                cap = new VideoCapture(deviceIndex, api);
                if (!cap.IsOpened())
                {
                    // Same DSHOW partial-init hazard as CameraDevice.OpenCapture — suppress finalizer.
                    GC.SuppressFinalize(cap);
                    cap = null;
                    return results;
                }

                opened = true;

                foreach (var (w, h) in _commonResolutions)
                {
                    cap.Set(VideoCaptureProperties.FrameWidth,  w);
                    cap.Set(VideoCaptureProperties.FrameHeight, h);
                    var actualW = (int)cap.Get(VideoCaptureProperties.FrameWidth);
                    var actualH = (int)cap.Get(VideoCaptureProperties.FrameHeight);
                    if (actualW > 0 && actualH > 0
                        && !results.Any(r => r.Width == actualW && r.Height == actualH))
                    {
                        results.Add(new CameraResolution { Width = actualW, Height = actualH });
                    }
                }
            }
            catch
            {
                if (cap != null && !opened) { GC.SuppressFinalize(cap); cap = null; }
            }
            finally
            {
                if (cap != null && opened) { try { cap.Dispose(); } catch { } }
            }
            return results.OrderBy(r => r.Width).ThenBy(r => r.Height).ToList();
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
