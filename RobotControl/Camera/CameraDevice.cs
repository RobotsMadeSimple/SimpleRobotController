using Controller.RobotControl.Camera.Sofia;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;

namespace Controller.RobotControl.Camera
{
    /// <summary>
    /// One camera: a USB device, a network (RTSP/HTTP) stream opened through FFmpeg, or a
    /// Sofia/DVRIP (XMeye) camera (docs/network-cameras.md). A single capture thread owns the
    /// OpenCV <see cref="VideoCapture"/> (for Sofia: the <see cref="SofiaSession"/>)
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

        // Network source (docs/network-cameras.md). DeviceIndex is ignored when IsNetwork.
        public string SourceType   { get; private set; } = NetworkCameraSource.SourceUsb;
        public string Url          { get; private set; } = "";
        public string Username     { get; private set; } = "";
        public string Password     { get; private set; } = "";
        public string Transport    { get; private set; } = NetworkCameraSource.TransportTcp;
        public bool   IsNetwork    => SourceType == NetworkCameraSource.SourceNetwork;
        /// <summary>Sofia / DVRIP (XMeye) camera; username/password are its Sofia login.</summary>
        public bool   IsSofia      => SourceType == NetworkCameraSource.SourceSofia;
        public bool   IsUsb        => SourceType == NetworkCameraSource.SourceUsb;
        public string Host         { get; private set; } = "";
        public int    Port         { get; private set; } = SofiaCameraSource.DefaultPort;
        public string Stream       { get; private set; } = SofiaCameraSource.StreamMain;
        public string Codec        { get; private set; } = DvripClient.CodecH264;
        public string Decoder      { get; private set; } = SofiaDecoder.DecoderOpenCv;
        public string FfmpegPath   { get; private set; } = SofiaCameraSource.DefaultFfmpegPath;
        public string Hwaccel      { get; private set; } = "";
        /// <summary>Size of the frames actually delivered (0 until the first frame).</summary>
        public int    StreamWidth  { get; private set; }
        public int    StreamHeight { get; private set; }
        /// <summary>Best-effort network decode latency (see <see cref="UpdateLatency"/>); 0 when unknown.</summary>
        public int    LatencyMs    { get; private set; }

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
            ApplySource(cfg);
        }

        private void ApplySource(CameraConfig cfg)
        {
            SourceType = NetworkCameraSource.NormalizeSourceType(cfg.SourceType);
            Url        = cfg.Url?.Trim() ?? "";
            Username   = cfg.Username ?? "";
            Password   = cfg.Password ?? "";
            Transport  = NetworkCameraSource.NormalizeTransport(cfg.Transport);
            Host       = cfg.Host?.Trim() ?? "";
            Port       = SofiaCameraSource.NormalizePort(cfg.Port);
            Stream     = SofiaCameraSource.NormalizeStream(cfg.Stream);
            Codec      = SofiaCameraSource.NormalizeCodec(cfg.Codec);
            Decoder    = SofiaCameraSource.NormalizeDecoder(cfg.Decoder);
            FfmpegPath = SofiaCameraSource.NormalizeFfmpegPath(cfg.FfmpegPath);
            Hwaccel    = SofiaCameraSource.NormalizeHwaccel(cfg.Hwaccel);
        }

        /// <summary>True when applying <paramref name="cfg"/> changes how the stream must be opened.</summary>
        public bool SourceDiffers(CameraConfig cfg) =>
               SourceType != NetworkCameraSource.NormalizeSourceType(cfg.SourceType)
            || Url        != (cfg.Url?.Trim() ?? "")
            || Username   != (cfg.Username ?? "")
            || Password   != (cfg.Password ?? "")
            || Transport  != NetworkCameraSource.NormalizeTransport(cfg.Transport)
            || Host       != (cfg.Host?.Trim() ?? "")
            || Port       != SofiaCameraSource.NormalizePort(cfg.Port)
            || Stream     != SofiaCameraSource.NormalizeStream(cfg.Stream)
            || Codec      != SofiaCameraSource.NormalizeCodec(cfg.Codec)
            || Decoder    != SofiaCameraSource.NormalizeDecoder(cfg.Decoder)
            || FfmpegPath != SofiaCameraSource.NormalizeFfmpegPath(cfg.FfmpegPath)
            || Hwaccel    != SofiaCameraSource.NormalizeHwaccel(cfg.Hwaccel);

        /// <summary>The stream URL with credentials injected (what FFmpeg opens). Never log this.</summary>
        private string? EffectiveUrl() => NetworkCameraSource.BuildUrl(Url, Username, Password);

        /// <summary>The URL safe to log: password replaced by <c>***</c>.</summary>
        public string MaskedUrl() => IsSofia
            ? $"sofia://{SofiaSettings().Username}@{Host}:{Port}/{Stream}"
            : NetworkCameraSource.MaskUrl(EffectiveUrl() ?? Url);

        /// <summary>The Sofia connection settings (an empty username logs in as admin).</summary>
        public SofiaSettings SofiaSettings() =>
            SofiaCameraSource.Settings(Host, Port, Username, Password, Stream, Codec, Decoder, FfmpegPath, Hwaccel);

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Start()
        {
            if (!Enabled) return;
            if (_thread is { IsAlive: true } && _running) return;   // idempotent

            int gen  = Interlocked.Increment(ref _generation);
            var prev = _thread;
            _running = true;
            // If the previous thread is still inside a blocking MSMF/V4L2 call, wait for it
            // to finish and release the device before this one opens it. Two threads holding
            // the same camera is exactly what made opens slow and connection state flap.
            _thread  = new Thread(() => { prev?.Join(); CaptureLoopGuarded(gen); }) { IsBackground = true, Name = $"Camera-{Id}" };
            _thread.Start();
        }

        public void Stop()
        {
            _running = false;
            Interlocked.Increment(ref _generation);   // invalidate the current thread even if Join times out
            var t = _thread;
            if (t != null && t != Thread.CurrentThread && !t.Join(StopJoinMs))
                Console.WriteLine($"[Camera] {Id} capture thread is still inside a device call (open/read); it will release the camera when that returns");
            FailPendingProbe();
            // A Sofia session blocks in socket/decoder reads; closing it lets the thread exit now.
            Interlocked.Exchange(ref _sofiaSession, null)?.Abort();
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
            ApplySource(cfg);
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
            SourceType           = SourceType,
            Url                  = Url,
            Username             = Username,
            Password             = Password,
            Transport            = Transport,
            StreamWidth          = StreamWidth,
            StreamHeight         = StreamHeight,
            LatencyMs            = LatencyMs,
            Host                 = Host,
            Port                 = Port,
            Stream               = Stream,
            Codec                = Codec,
            Decoder              = Decoder,
            FfmpegPath           = FfmpegPath,
            Hwaccel              = Hwaccel,
        };

        // ── Resolution probing ─────────────────────────────────────────────────

        /// <summary>
        /// Asks the capture thread to probe the supported resolutions on its open capture
        /// and waits for the answer. Returns [] if the camera is not connected, the probe
        /// times out, or the device is stopped while the probe is pending.
        /// </summary>
        public List<CameraResolution> ProbeResolutions()
        {
            if (!IsUsb || !Connected || !_running) return new List<CameraResolution>();

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

        // Releases a capture that was fully opened (we read frames from it). The device must
        // be released on every platform: a V4L2 descriptor or an MSMF source that is merely
        // abandoned keeps the camera busy, so the next open of the same device blocks or
        // fails. (Partially-initialised captures are handled in OpenCapture, where the
        // finalizer is suppressed instead — releasing those can crash on Windows.)
        private void ReleaseCapture(ref VideoCapture? cap)
        {
            if (cap == null) return;
            try { cap.Release(); }
            catch (Exception ex) { Console.WriteLine($"[Camera] {Id} release failed: {ex.Message}"); }
            try { cap.Dispose(); } catch { }
            cap = null;
        }

        private static VideoCapture? OpenCapture(int index)
        {
            VideoCapture? cap = null;
            try
            {
                // Open by device path on Linux — more reliable than index with V4L2 and avoids
                // "can't open camera by index" after a reconnect.
                // On Windows use DirectShow: measured on a dev box, Media Foundation took ~7 s
                // to open camera 0 and ~13 s more for the resolution Set calls (every open),
                // so an added camera stayed "not connected" for ~20 s; DirectShow does the
                // same in ~1.2 s. The static resolution probe in CameraManager uses DSHOW too.
                cap = OperatingSystem.IsLinux()
                    ? new VideoCapture($"/dev/video{index}", VideoCaptureAPIs.V4L2)
                    : new VideoCapture(index, VideoCaptureAPIs.DSHOW);
                if (cap.IsOpened()) return cap;
                GC.SuppressFinalize(cap);
                cap = null;
            }
            catch (Exception) { if (cap != null) { GC.SuppressFinalize(cap); cap = null; } }
            return null;
        }

        private VideoCapture? OpenNetworkCapture()
        {
            var url = EffectiveUrl();
            return url == null ? null : NetworkCameraSource.Open(url, Transport);
        }

        // Latency estimate: how far wall-clock time has run ahead of the stream's own
        // presentation clock since the open, relative to the best (smallest) gap seen. A
        // stream decoded in real time stays near 0; a growing backlog shows up here.
        // Touched only by the capture thread.
        private readonly Stopwatch _latencyClock = new();
        private double _pts0   = double.NaN;
        private double _minLag = double.MaxValue;

        private void ResetLatency()
        {
            _latencyClock.Restart();
            _pts0     = double.NaN;
            _minLag   = double.MaxValue;
            LatencyMs = 0;
        }

        private void UpdateLatency(VideoCapture cap)
        {
            try
            {
                double pts = cap.Get(VideoCaptureProperties.PosMsec);
                if (pts <= 0 || double.IsNaN(pts)) { LatencyMs = 0; return; }
                if (double.IsNaN(_pts0)) { _pts0 = pts; _latencyClock.Restart(); }
                double lag = _latencyClock.Elapsed.TotalMilliseconds - (pts - _pts0);
                if (lag < _minLag) _minLag = lag;
                LatencyMs = (int)Math.Max(0, Math.Round(lag - _minLag));
            }
            catch { LatencyMs = 0; }
        }

        private void CaptureLoopGuarded(int generation)
        {
            try
            {
                if (IsSofia) SofiaCaptureLoop(generation);
                else         CaptureLoop(generation);
            }
            catch (Exception ex) { Console.WriteLine($"[Camera] {Id} thread died: {ex}"); }
            SetConnected(generation, false);
            if (generation == _generation) FailPendingProbe();
        }

        private bool ShouldRun(int generation) => _running && generation == _generation;

        /// <summary>A retired thread (superseded by a later Start) must not clobber the live thread's state.</summary>
        private void SetConnected(int generation, bool value)
        {
            if (generation == _generation) Connected = value;
        }

        private void CaptureLoop(int generation)
        {
            var intervalMs = Math.Max(1, 1000 / Math.Max(1, TargetFps));
            VideoCapture? cap = null;   // owned by this thread; never shared
            bool openFailureLogged = false;
            // Network sources are drained as fast as frames arrive (no latency builds up in
            // FFmpeg's buffer); only the publish (JPEG encode) is throttled to targetFps.
            bool network = IsNetwork;
            var  publishClock = Stopwatch.StartNew();
            long lastPublishMs = long.MinValue / 2;

            try
            {
                while (ShouldRun(generation))
                {
                    try
                    {
                        if (cap == null || !cap.IsOpened())
                        {
                            SetConnected(generation, false);
                            ReleaseCapture(ref cap);

                            if (network && !NetworkCameraSource.FfmpegAvailable)
                            {
                                if (!openFailureLogged)
                                {
                                    Console.WriteLine($"[Camera] ERROR: {Id} is a network camera but this OpenCV build has no FFmpeg backend; it stays disconnected");
                                    openFailureLogged = true;
                                }
                                Thread.Sleep(ReopenDelayMs);
                                continue;
                            }

                            cap = network ? OpenNetworkCapture() : OpenCapture(DeviceIndex);

                            if (cap == null && network)
                            {
                                if (!openFailureLogged)
                                {
                                    Console.WriteLine($"[Camera] {Id} could not open {MaskedUrl()}; retrying every {ReopenDelayMs / 1000}s");
                                    openFailureLogged = true;
                                }
                                Thread.Sleep(ReopenDelayMs);
                                continue;
                            }

                            if (cap == null)
                            {
                                if (!openFailureLogged)
                                {
                                    Console.WriteLine($"[Camera] {Id} could not open device {DeviceIndex}; retrying every {ReopenDelayMs / 1000}s");
                                    openFailureLogged = true;
                                }
                                Thread.Sleep(ReopenDelayMs);
                                continue;
                            }
                            openFailureLogged = false;

                            if (network)
                            {
                                // The stream's own size is used; no resolution Set calls.
                                ResetLatency();
                                Console.WriteLine($"[Camera] {Id} opened {MaskedUrl()}");
                            }
                            else
                            {
                                if (Width > 0 && Height > 0)
                                {
                                    cap.Set(VideoCaptureProperties.FrameWidth,  Width);
                                    cap.Set(VideoCaptureProperties.FrameHeight, Height);
                                }

                                Console.WriteLine($"[Camera] {Id} opened on device {DeviceIndex}");
                            }
                        }

                        // A Stop/Start may have happened during the (possibly long) open above.
                        if (!ShouldRun(generation)) break;

                        using var frame = new Mat();
                        if (!cap.Read(frame) || frame.Empty())
                        {
                            SetConnected(generation, false);
                            ReleaseCapture(ref cap);
                            Thread.Sleep(ReadFailDelayMs);
                            continue;
                        }

                        SetConnected(generation, true);

                        if (network)
                        {
                            if (generation == _generation)
                            {
                                StreamWidth  = frame.Width;
                                StreamHeight = frame.Height;
                                UpdateLatency(cap);
                            }
                            long now = publishClock.ElapsedMilliseconds;
                            if (now - lastPublishMs < intervalMs) continue;   // decoded, not published
                            lastPublishMs = now;
                            Cv2.ImEncode(".jpg", frame, out var nbuf, JpegParams);
                            lock (_frameLock) _latestFrame = nbuf;
                            continue;   // no sleep: the next Read blocks until a frame arrives
                        }

                        Cv2.ImEncode(".jpg", frame, out var buf, JpegParams);
                        lock (_frameLock) _latestFrame = buf;

                        // Probe only after a good frame, so the driver has settled after the open.
                        if (ShouldRun(generation)) ServiceProbes(cap);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine($"[Camera] {Id} error: {ex}");
                        SetConnected(generation, false);
                        ReleaseCapture(ref cap);
                        Thread.Sleep(ReopenDelayMs);
                        continue;
                    }

                    Thread.Sleep(intervalMs);
                }
            }
            finally
            {
                SetConnected(generation, false);
                ReleaseCapture(ref cap);
            }
        }

        // ── Sofia / DVRIP ──────────────────────────────────────────────────────

        // The live session of the Sofia capture thread, so Stop() can abort its blocking reads.
        private SofiaSession? _sofiaSession;

        private const int SofiaOpenTimeoutMs = NetworkCameraSource.DefaultOpenTimeoutMs;

        /// <summary>
        /// Capture thread for a Sofia camera. Each pass owns one <see cref="SofiaSession"/>
        /// (DVRIP client + pump thread + decoder): connect/login/claim, read the first video
        /// frame (codec detection), start the decoder, then publish decoded pictures until the
        /// session ends. Any failure — connect, login, a decoder that yields no picture within
        /// the open timeout, a socket error or the resync giving up — tears the session down and
        /// reconnects after the usual backoff, logging once per failure streak.
        /// </summary>
        private void SofiaCaptureLoop(int generation)
        {
            var intervalMs     = Math.Max(1, 1000 / Math.Max(1, TargetFps));
            var publishClock   = Stopwatch.StartNew();
            long lastPublishMs = long.MinValue / 2;
            bool streakLogged  = false;

            while (ShouldRun(generation))
            {
                var settings = SofiaSettings();
                var session  = new SofiaSession(settings, Id);
                Volatile.Write(ref _sofiaSession, session);
                string? failure = null;
                bool unavailable = false;
                int  gotPicture  = 0;
                try
                {
                    // A Stop() that ran before the session was published could not abort it.
                    if (!ShouldRun(generation)) break;
                    if (string.IsNullOrWhiteSpace(settings.Host))
                        throw new DvripException(DvripStage.Connect, "no host configured");

                    session.Connect(SofiaOpenTimeoutMs);
                    if (!ShouldRun(generation)) break;
                    var first = session.ReadFirstFrame(SofiaCameraSource.StreamReadTimeoutMs);
                    if (!ShouldRun(generation)) break;
                    session.StartDecoder(first, SofiaOpenTimeoutMs, SofiaCameraSource.StreamReadTimeoutMs);
                    Console.WriteLine($"[Sofia] {Id} streaming {MaskedUrl()} ({session.EffectiveCodec}, {settings.Decoder} decoder)");

                    // No picture within the open timeout (probe/decoder never produced one): reconnect.
                    using var watchdog = new Timer(_ =>
                    {
                        if (Volatile.Read(ref gotPicture) == 0) session.Abort();
                    }, null, SofiaOpenTimeoutMs, Timeout.Infinite);

                    while (ShouldRun(generation))
                    {
                        using var pic = session.ReadDecoded();
                        if (pic == null) break;
                        if (Interlocked.Exchange(ref gotPicture, 1) == 0)
                        {
                            streakLogged = false;
                            Console.WriteLine($"[Sofia] {Id} first picture {pic.Width}x{pic.Height}");
                        }

                        SetConnected(generation, true);
                        if (generation == _generation)
                        {
                            StreamWidth  = pic.Width;
                            StreamHeight = pic.Height;
                            LatencyMs    = session.LatencyMs;
                        }
                        // Decoded as fast as frames arrive; only the publish is throttled to targetFps.
                        long now = publishClock.ElapsedMilliseconds;
                        if (now - lastPublishMs < intervalMs) continue;
                        lastPublishMs = now;
                        var jpeg = pic.ToJpeg(JpegParams);
                        lock (_frameLock) _latestFrame = jpeg;
                    }

                    failure = Volatile.Read(ref gotPicture) == 0
                        ? $"decoder produced no picture within {SofiaOpenTimeoutMs / 1000}s"
                        : session.PumpError?.Message ?? "stream ended";
                }
                catch (DecoderUnavailableException ex) { failure = ex.Message; unavailable = true; }
                catch (Exception ex)                   { failure = ex.Message; }
                finally
                {
                    Interlocked.CompareExchange(ref _sofiaSession, null, session);
                    session.Dispose();
                    SetConnected(generation, false);
                    if (generation == _generation) LatencyMs = 0;
                }

                if (!ShouldRun(generation)) break;
                if (!streakLogged && failure != null)
                {
                    Console.WriteLine(unavailable
                        ? $"[Sofia] ERROR: {Id} {failure}; it stays disconnected (retrying every {ReopenDelayMs / 1000}s)"
                        : $"[Sofia] {Id} {MaskedUrl()}: {failure}; retrying every {ReopenDelayMs / 1000}s");
                    streakLogged = true;
                }
                SleepWhileRunning(generation, ReopenDelayMs);
            }
            SetConnected(generation, false);
        }

        private void SleepWhileRunning(int generation, int ms)
        {
            var sw = Stopwatch.StartNew();
            while (ShouldRun(generation) && sw.ElapsedMilliseconds < ms) Thread.Sleep(50);
        }
    }
}
