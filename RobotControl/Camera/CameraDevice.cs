using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Controller.RobotControl.Camera
{
    public class CameraDevice
    {
        public string Id          { get; }
        public string Name        { get; private set; }
        public bool   Connected   { get; private set; }
        public int    DeviceIndex { get; private set; }
        public int    Width       { get; private set; }
        public int    Height      { get; private set; }
        public int    TargetFps   { get; private set; }
        public bool   Enabled     { get; private set; }

        public List<CameraResolution>           SupportedResolutions  { get; set; } = new();
        public Action<List<CameraResolution>>?  OnResolutionsDetected;

        private volatile bool   _autoProbeScheduled;
        private VideoCapture?   _capture;
        private byte[]?         _latestFrame;
        private readonly object _frameLock = new();
        private Thread?         _thread;
        private volatile bool   _running;

        private static readonly int[] JpegParams = { (int)ImwriteFlags.JpegQuality, 75 };

        private static readonly (int W, int H)[] _probeResolutions =
        {
            (160, 120), (320, 240), (640, 480), (800, 600),
            (1024, 768), (1280, 720), (1280, 960), (1920, 1080),
            (2560, 1440), (3840, 2160),
        };

        public CameraDevice(CameraConfig cfg)
        {
            Id                   = cfg.Id;
            Name                 = cfg.Name;
            DeviceIndex          = cfg.DeviceIndex;
            Width                = cfg.Width;
            Height               = cfg.Height;
            TargetFps            = cfg.TargetFps;
            Enabled              = cfg.Enabled;
            SupportedResolutions = new List<CameraResolution>(cfg.SupportedResolutions);
        }

        public void Start()
        {
            if (!Enabled) return;
            _running = true;
            _thread  = new Thread(CaptureLoopGuarded) { IsBackground = true, Name = $"Camera-{Id}" };
            _thread.Start();
        }

        private void CaptureLoopGuarded()
        {
            try   { CaptureLoop(); }
            catch (Exception ex) { Console.WriteLine($"[Camera] {Id} thread died: {ex.Message}"); }
            catch                { Console.WriteLine($"[Camera] {Id} thread died (native exception)"); }
            Connected = false;
        }

        public void Stop()
        {
            _running = false;
            _thread?.Join(2000);
            AbandonCapture(ref _capture);
            Connected = false;
        }

        public void ApplyConfig(CameraConfig cfg)
        {
            Name        = cfg.Name;
            DeviceIndex = cfg.DeviceIndex;
            Width       = cfg.Width;
            Height      = cfg.Height;
            TargetFps   = cfg.TargetFps;
            Enabled     = cfg.Enabled;
            // SupportedResolutions preserved — they are detected, not user-configured
        }

        public byte[]? GetLatestFrame()
        {
            lock (_frameLock) return _latestFrame;
        }

        public CameraState GetState() => new()
        {
            Id                   = Id,
            Name                 = Name,
            Connected            = Connected,
            DeviceIndex          = DeviceIndex,
            Width                = Width,
            Height               = Height,
            TargetFps            = TargetFps,
            Enabled              = Enabled,
            SupportedResolutions = SupportedResolutions,
        };

        /// <summary>
        /// Pauses the capture thread, probes supported resolutions using the already-open
        /// capture, then restarts the thread. Returns [] if not currently connected.
        /// </summary>
        public List<CameraResolution> ProbeResolutions()
        {
            if (_capture == null || !_capture.IsOpened())
                return new List<CameraResolution>();

            bool wasRunning = _running;
            _running = false;
            _thread?.Join(3000);

            var results = new List<CameraResolution>();
            try
            {
                var cap = _capture;
                if (cap != null && cap.IsOpened())
                {
                    foreach (var (w, h) in _probeResolutions)
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
                    if (Width > 0 && Height > 0)
                    {
                        cap.Set(VideoCaptureProperties.FrameWidth,  Width);
                        cap.Set(VideoCaptureProperties.FrameHeight, Height);
                    }
                }
            }
            catch (Exception ex) { Console.WriteLine($"[Camera] {Id} probe error: {ex.Message}"); }
            catch                { Console.WriteLine($"[Camera] {Id} probe native error"); }

            if (wasRunning)
            {
                _running = true;
                _thread  = new Thread(CaptureLoopGuarded) { IsBackground = true, Name = $"Camera-{Id}" };
                _thread.Start();
            }

            return results.OrderBy(r => r.Width).ThenBy(r => r.Height).ToList();
        }

        // Suppress the finalizer instead of calling Dispose on a partially-initialized DSHOW
        // VideoCapture — calling release() on corrupt native state terminates the process.
        private static void AbandonCapture(ref VideoCapture? cap)
        {
            if (cap == null) return;
            GC.SuppressFinalize(cap);
            cap = null;
        }

        private static VideoCapture? OpenCapture(int index)
        {
            VideoCapture? cap = null;
            try
            {
                cap = new VideoCapture(index, VideoCaptureAPIs.MSMF);
                if (cap.IsOpened()) return cap;
                GC.SuppressFinalize(cap);
                cap = null;
            }
            catch (Exception) { if (cap != null) { GC.SuppressFinalize(cap); cap = null; } }
            catch              { if (cap != null) { GC.SuppressFinalize(cap); cap = null; } }
            return null;
        }

        private void CaptureLoop()
        {
            var intervalMs = Math.Max(1, 1000 / Math.Max(1, TargetFps));

            while (_running)
            {
                try
                {
                    if (_capture == null || !_capture.IsOpened())
                    {
                        Connected = false;
                        AbandonCapture(ref _capture);
                        _capture = OpenCapture(DeviceIndex);

                        if (_capture == null)
                        {
                            Thread.Sleep(3000);
                            continue;
                        }

                        if (Width > 0 && Height > 0)
                        {
                            _capture.Set(VideoCaptureProperties.FrameWidth,  Width);
                            _capture.Set(VideoCaptureProperties.FrameHeight, Height);
                        }

                        Console.WriteLine($"[Camera] {Id} opened on device {DeviceIndex}");

                        // Auto-probe resolutions on first connect
                        if (!_autoProbeScheduled && SupportedResolutions.Count == 0)
                        {
                            _autoProbeScheduled = true;
                            _ = Task.Run(() =>
                            {
                                Thread.Sleep(500); // let first frames stabilise
                                var res = ProbeResolutions();
                                if (res.Count > 0)
                                {
                                    SupportedResolutions = res;
                                    OnResolutionsDetected?.Invoke(res);
                                }
                                else
                                {
                                    _autoProbeScheduled = false; // allow retry on next connect
                                }
                            });
                        }
                    }

                    // Ensure Connected reflects that capture is open (covers thread restarts after probe)
                    if (!Connected) Connected = true;

                    using var frame = new Mat();
                    if (!_capture.Read(frame) || frame.Empty())
                    {
                        Connected = false;
                        AbandonCapture(ref _capture);
                        Thread.Sleep(1000);
                        continue;
                    }

                    Cv2.ImEncode(".jpg", frame, out var buf, JpegParams);
                    lock (_frameLock) _latestFrame = buf;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Camera] {Id} error: {ex.Message}");
                    Connected = false;
                    AbandonCapture(ref _capture);
                    Thread.Sleep(3000);
                    continue;
                }
                catch
                {
                    Console.WriteLine($"[Camera] {Id} native exception caught — retrying in 5s");
                    Connected = false;
                    AbandonCapture(ref _capture);
                    Thread.Sleep(5000);
                    continue;
                }

                Thread.Sleep(intervalMs);
            }

            Connected = false;
        }
    }
}
