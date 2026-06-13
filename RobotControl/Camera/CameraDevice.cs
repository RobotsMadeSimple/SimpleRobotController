using OpenCvSharp;
using System;
using System.Threading;

namespace Controller.RobotControl.Camera
{
    /// <summary>
    /// Manages a single USB camera: opens it, captures frames on a background thread,
    /// and exposes the latest JPEG-encoded frame for streaming.
    /// </summary>
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

        private VideoCapture? _capture;
        private byte[]?       _latestFrame;
        private readonly object _frameLock = new();
        private Thread?         _thread;
        private volatile bool   _running;

        private static readonly int[] JpegParams = { (int)ImwriteFlags.JpegQuality, 75 };

        public CameraDevice(CameraConfig cfg)
        {
            Id          = cfg.Id;
            Name        = cfg.Name;
            DeviceIndex = cfg.DeviceIndex;
            Width       = cfg.Width;
            Height      = cfg.Height;
            TargetFps   = cfg.TargetFps;
            Enabled     = cfg.Enabled;
        }

        public void Start()
        {
            if (!Enabled) return;
            _running = true;
            _thread  = new Thread(CaptureLoop) { IsBackground = true, Name = $"Camera-{Id}" };
            _thread.Start();
        }

        public void Stop()
        {
            _running = false;
            _thread?.Join(2000);
            _capture?.Dispose();
            _capture   = null;
            Connected  = false;
        }

        public void ApplyConfig(CameraConfig cfg)
        {
            Name        = cfg.Name;
            DeviceIndex = cfg.DeviceIndex;
            Width       = cfg.Width;
            Height      = cfg.Height;
            TargetFps   = cfg.TargetFps;
            Enabled     = cfg.Enabled;
        }

        /// <summary>Returns the latest JPEG frame bytes, or null if no frame is available.</summary>
        public byte[]? GetLatestFrame()
        {
            lock (_frameLock) return _latestFrame;
        }

        public CameraState GetState() => new()
        {
            Id          = Id,
            Name        = Name,
            Connected   = Connected,
            DeviceIndex = DeviceIndex,
            Width       = Width,
            Height      = Height,
            TargetFps   = TargetFps,
            Enabled     = Enabled,
        };

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
                        _capture?.Dispose();
                        _capture = new VideoCapture(DeviceIndex, VideoCaptureAPIs.DSHOW);

                        if (Width > 0 && Height > 0)
                        {
                            _capture.Set(VideoCaptureProperties.FrameWidth,  Width);
                            _capture.Set(VideoCaptureProperties.FrameHeight, Height);
                        }

                        if (!_capture.IsOpened())
                        {
                            _capture.Dispose();
                            _capture = null;
                            Thread.Sleep(3000);
                            continue;
                        }

                        Connected = true;
                        Console.WriteLine($"[Camera] {Id} opened on device {DeviceIndex}");
                    }

                    using var frame = new Mat();
                    if (!_capture.Read(frame) || frame.Empty())
                    {
                        Connected = false;
                        _capture.Dispose();
                        _capture = null;
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
                    try { _capture?.Dispose(); } catch { }
                    _capture = null;
                    Thread.Sleep(3000);
                    continue;
                }

                Thread.Sleep(intervalMs);
            }

            Connected = false;
        }
    }
}
