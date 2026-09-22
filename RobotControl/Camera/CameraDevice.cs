using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Controller.RobotControl.Camera
{
    /// <summary>
    /// One USB camera. A single capture thread owns the OpenCV <see cref="VideoCapture"/>
    /// for its whole life: it opens the device, reads frames, services resolution probes
    /// between frames, and releases the device when it exits. No other thread ever
    /// touches the capture handle, which is what makes the lifecycle race-free.
    ///
    /// <para>Resolution probing used to pause the capture thread, poke the shared handle
    /// from the caller's thread and start a replacement thread. Two overlapping probes
    /// (the auto-probe on first connect and the app's GetCameraResolutions) could leave
    /// two capture threads sharing one handle; the first failed read nulled it and the
    /// other thread crashed on a null reference. Probes are now requests that the
    /// capture thread fulfils in place.</para>
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

        public List<CameraResolution>           SupportedResolutions  { get; set; } = new();
        public Action<List<CameraResolution>>?  OnResolutionsDetected;

        private byte[]?         _latestFrame;
        private readonly object _frameLock = new();

        // Capture thread lifecycle. _generation is bumped on every Start(); a thread whose
        // generation is stale exits at its next check, so a Stop() whose Join timed out
        // (the thread was blocked inside a device read) can never end up sharing the
        // device with the thread a later Start() created.
        private Thread?       _thread;
        private volatile bool _running;
        private int           _generation;

        // Pending resolution probe, fulfilled by the capture thread between two frames.
        // Concurrent callers share one request.
        private readonly object _probeLock = new();
        private TaskCompletionSource<List<CameraResolution>>? _probeRequest;
        private volatile bool _autoProbeDone;

        private const int StopJoinMs        = 2000;
        private const int ProbeTimeoutMs    = 15000;  // MSMF can take seconds per resolution
        private const int ReopenDelayMs     = 3000;
        private const int ReadFailDelayMs   = 1000;

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

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            if (!Enabled) return;
            if (_thread is { IsAlive: true } && _running) return;   // idempotent

            int gen  = Interlocked.Increment(ref _generation);
            _running = true;
            _thread  = new Thread(() => CaptureLoopGuarded(gen)) { IsBackground = true, Name = $"Camera-{Id}" };
            _thread.Start();
        }

        public void Stop()
        {
            _running = false;
            Interlocked.Increment(ref _generation);   // invalidate the current thread even if Join times out
            var t = _thread;
            if (t != null && t != Thread.CurrentThread && !t.Join(StopJoinMs))
                Console.WriteLine($"[Camera] {Id} capture thread is blocked in a device read; it will release the camera when the read returns");
            FailPendingProbe();
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

        // ── Resolution probing ─────────────────────────────────────────────────

        /// <summary>
        /// Asks the capture thread to probe the supported resolutions on its open capture
        /// and waits for the answer. Returns [] if the camera is not connected, the probe
        /// times out, or the device is stopped while the probe is pending.
        /// </summary>
        public List<CameraResolution> ProbeResolutions()
        {
            if (!Connected || !_running) return new List<CameraResolution>();

            TaskCompletionSource<List<CameraResolution>> request;
            lock (_probeLock)
            {
                _probeRequest ??= new TaskCompletionSource<List<CameraResolution>>(TaskCreationOptions.RunContinuationsAsynchronously);
                request = _probeRequest;
            }

            try
            {
                return request.Task.Wait(ProbeTimeoutMs) ? request.Task.Result : new List<CameraResolution>();
            }
            catch (AggregateException) { return new List<CameraResolution>(); }
        }

        private TaskCompletionSource<List<CameraResolution>>? TakePendingProbe()
        {
            lock (_probeLock)
            {
                var r = _probeRequest;
                _probeRequest = null;
                return r;
            }
        }

        private void FailPendingProbe() => TakePendingProbe()?.TrySetResult(new List<CameraResolution>());

        /// <summary>Runs on the capture thread only. Tries each candidate size and records what the driver accepts.</summary>
        private List<CameraResolution> ProbeOnCaptureThread(VideoCapture cap)
        {
            var results = new List<CameraResolution>();
            try
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
            }
            catch (Exception ex) { Console.WriteLine($"[Camera] {Id} probe error: {ex}"); }
            finally
            {
                // Always restore the configured size, even if the probe threw part-way.
                if (Width > 0 && Height > 0)
                {
                    try
                    {
                        cap.Set(VideoCaptureProperties.FrameWidth,  Width);
                        cap.Set(VideoCaptureProperties.FrameHeight, Height);
                    }
                    catch (Exception ex) { Console.WriteLine($"[Camera] {Id} could not restore resolution after probe: {ex.Message}"); }
                }
            }
            return results.OrderBy(r => r.Width).ThenBy(r => r.Height).ToList();
        }

        /// <summary>Capture thread: services a pending probe (or the first-connect auto-probe) between frames.</summary>
        private void ServiceProbes(VideoCapture cap)
        {
            var request = TakePendingProbe();
            bool auto   = !_autoProbeDone && SupportedResolutions.Count == 0;
            if (request == null && !auto) return;

            var results = ProbeOnCaptureThread(cap);
            request?.TrySetResult(results);

            if (results.Count > 0)
            {
                _autoProbeDone       = true;
                SupportedResolutions = results;
                try { OnResolutionsDetected?.Invoke(results); }
                catch (Exception ex) { Console.WriteLine($"[Camera] {Id} OnResolutionsDetected failed: {ex}"); }
            }
            // An empty auto-probe result is retried on the next frame that follows a reconnect.
        }

        // ── Capture ────────────────────────────────────────────────────────────

        // On Windows, calling Release() on a partially-initialised DSHOW VideoCapture can
        // terminate the process — suppress the finalizer instead and let GC clean up.
        // On Linux, V4L2 file descriptors must be explicitly released or the device stays
        // locked and can't be reopened.
        private static void AbandonCapture(ref VideoCapture? cap)
        {
            if (cap == null) return;
            if (OperatingSystem.IsWindows())
            {
                GC.SuppressFinalize(cap);
            }
            else
            {
                try { cap.Release(); cap.Dispose(); } catch { }
            }
            cap = null;
        }

        private static VideoCapture? OpenCapture(int index)
        {
            VideoCapture? cap = null;
            try
            {
                // Open by device path on Linux — more reliable than index with V4L2 and avoids
                // "can't open camera by index" after a reconnect.
                cap = OperatingSystem.IsLinux()
                    ? new VideoCapture($"/dev/video{index}", VideoCaptureAPIs.V4L2)
                    : new VideoCapture(index, VideoCaptureAPIs.MSMF);
                if (cap.IsOpened()) return cap;
                GC.SuppressFinalize(cap);
                cap = null;
            }
            catch (Exception) { if (cap != null) { GC.SuppressFinalize(cap); cap = null; } }
            return null;
        }

        private void CaptureLoopGuarded(int generation)
        {
            try   { CaptureLoop(generation); }
            catch (Exception ex) { Console.WriteLine($"[Camera] {Id} thread died: {ex}"); }
            Connected = false;
            FailPendingProbe();
        }

        private bool ShouldRun(int generation) => _running && generation == _generation;

        private void CaptureLoop(int generation)
        {
            var intervalMs = Math.Max(1, 1000 / Math.Max(1, TargetFps));
            VideoCapture? cap = null;   // owned by this thread; never shared

            try
            {
                while (ShouldRun(generation))
                {
                    try
                    {
                        if (cap == null || !cap.IsOpened())
                        {
                            Connected = false;
                            AbandonCapture(ref cap);
                            cap = OpenCapture(DeviceIndex);

                            if (cap == null)
                            {
                                Thread.Sleep(ReopenDelayMs);
                                continue;
                            }

                            if (Width > 0 && Height > 0)
                            {
                                cap.Set(VideoCaptureProperties.FrameWidth,  Width);
                                cap.Set(VideoCaptureProperties.FrameHeight, Height);
                            }

                            Console.WriteLine($"[Camera] {Id} opened on device {DeviceIndex}");
                        }

                        using var frame = new Mat();
                        if (!cap.Read(frame) || frame.Empty())
                        {
                            Connected = false;
                            AbandonCapture(ref cap);
                            Thread.Sleep(ReadFailDelayMs);
                            continue;
                        }

                        Connected = true;

                        Cv2.ImEncode(".jpg", frame, out var buf, JpegParams);
                        lock (_frameLock) _latestFrame = buf;

                        // Probe only after a good frame, so the driver has settled after the open.
                        ServiceProbes(cap);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"[Camera] {Id} error: {ex}");
                        Connected = false;
                        AbandonCapture(ref cap);
                        Thread.Sleep(ReopenDelayMs);
                        continue;
                    }

                    Thread.Sleep(intervalMs);
                }
            }
            finally
            {
                Connected = false;
                AbandonCapture(ref cap);
            }
        }
    }
}
