using Controller.RobotControl.Vision.Tools;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Threading;

namespace Controller.RobotControl.Vision
{
    /// <summary>
    /// Runs a vision program on each new camera frame on a background thread, annotates the
    /// frame, and exposes the latest JPEGs and results for streaming. The per-frame work is
    /// <see cref="VisionEngine"/>; this class owns the thread, the native caches
    /// (<see cref="VisionRuntime"/>) and publication.
    /// </summary>
    public class VisionProcessor
    {
        private const int NoFrameSleepMs     = 50;
        private const int SameFrameSleepMs   = 10;
        private const int ErrorSleepMs       = 200;
        private const int FramePeriodSleepMs = 50;   // ~20 fps
        private const int StopJoinTimeoutMs  = 2000;

        // Written by UpdateProgram (any thread), read once per frame by the processing thread.
        private volatile VisionProgram _program;
        private readonly Camera.CameraDevice _camera;
        private readonly VisionRuntime _runtime = new();
        private readonly Action<VisionProgram>? _onReferenceCaptured;

        private byte[]?       _latestAnnotated;
        private VisionResult? _latestResult;
        private byte[]?       _latestRaw;
        private readonly object _lock = new();

        private Thread?       _thread;
        private volatile bool _running;
        private byte[]?       _lastProcessedJpeg;

        // Logs a tool's error only when its message changes, not once per frame.
        private readonly Dictionary<string, string> _lastLoggedError = new();

        public string ProgramId => _program.Id;
        public VisionProgram Program => _program;

        /// <param name="onReferenceCaptured">Called (on the processing thread) when a pose tool's
        /// reference was captured automatically, so the owner can persist the program.</param>
        public VisionProcessor(VisionProgram program, Camera.CameraDevice camera, Action<VisionProgram>? onReferenceCaptured = null)
        {
            _program = program;
            _camera  = camera;
            _onReferenceCaptured = onReferenceCaptured;
        }

        public void Start()
        {
            if (_thread is { IsAlive: true }) return;
            _running = true;
            _thread  = new Thread(ProcessLoop) { IsBackground = true, Name = $"Vision-{ProgramId}" };
            _thread.Start();
        }

        public void Stop()
        {
            _running = false;
            if (_thread != null && !_thread.Join(StopJoinTimeoutMs))
            {
                // Still mid-frame: releasing the native caches now could free one in use.
                Console.WriteLine($"[Vision] {ProgramId} processing thread did not stop within timeout");
                return;
            }
            _runtime.Dispose();
        }

        /// <summary>Hot-swaps the program. Takes effect from the next frame.</summary>
        public void UpdateProgram(VisionProgram updated) => _program = updated;

        public byte[]? GetLatestAnnotated() { lock (_lock) return _latestAnnotated; }
        public byte[]? GetLatestRaw()       { lock (_lock) return _latestRaw; }
        public VisionResult? GetLatestResult() { lock (_lock) return _latestResult; }

        private void ProcessLoop()
        {
            while (_running)
            {
                try
                {
                    var jpeg = _camera.GetLatestFrame();
                    if (jpeg == null) { Thread.Sleep(NoFrameSleepMs); continue; }
                    // The camera publishes a new byte[] per encoded frame: same reference = same frame.
                    if (ReferenceEquals(jpeg, _lastProcessedJpeg)) { Thread.Sleep(SameFrameSleepMs); continue; }

                    using var src = Cv2.ImDecode(jpeg, ImreadModes.Color);
                    if (src.Empty()) { Thread.Sleep(NoFrameSleepMs); continue; }

                    var program = _program;
                    var run = RunOnFrame(program, src, _runtime, out var annotated);
                    LogErrors(run.Result);
                    lock (_lock)
                    {
                        _latestRaw       = jpeg;
                        _latestAnnotated = annotated;
                        _latestResult    = run.Result;
                    }
                    _lastProcessedJpeg = jpeg;
                    if (run.CapturedReferences.Count > 0) _onReferenceCaptured?.Invoke(program);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[Vision] {ProgramId} error: {ex.Message}");
                    Thread.Sleep(ErrorSleepMs);
                    continue;
                }
                Thread.Sleep(FramePeriodSleepMs);
            }
        }

        private void LogErrors(VisionResult result)
        {
            foreach (var tr in result.Tools)
            {
                if (tr.Status != ToolStatus.Error) { _lastLoggedError.Remove(tr.Id); continue; }
                if (_lastLoggedError.TryGetValue(tr.Id, out var last) && last == tr.Message) continue;
                _lastLoggedError[tr.Id] = tr.Message ?? "";
                Console.WriteLine($"[Vision] {result.ProgramId} tool {tr.Id}: {tr.Message}");
            }
        }

        /// <summary>One frame through the engine plus the annotated JPEG.</summary>
        internal static VisionEngine.Run RunOnFrame(VisionProgram program, Mat src, VisionRuntime runtime, out byte[] annotatedJpeg)
        {
            var run = VisionEngine.Execute(program, src, runtime);
            using var annotated = src.Clone();
            VisionOverlay.DrawAll(annotated, program, run.Result);
            annotatedJpeg = VisionDrawing.EncodeJpeg(annotated, VisionDrawing.AnnotatedJpegQuality);
            return run;
        }

        /// <summary>
        /// Runs a program once on a supplied image — no camera, no thread. For RunVisionOnImage
        /// and tests. A reference captured on this run is set on <paramref name="program"/> in
        /// place (<paramref name="capturedReference"/> = true); the caller decides whether to persist it.
        /// </summary>
        public static VisionResult RunProgramOnce(VisionProgram program, Mat src, out byte[] annotatedJpeg, out bool capturedReference)
        {
            using var runtime = new VisionRuntime();
            var run = RunOnFrame(program, src, runtime, out annotatedJpeg);
            capturedReference = run.CapturedReferences.Count > 0;
            return run.Result;
        }

        public static VisionResult RunProgramOnce(VisionProgram program, Mat src, out byte[] annotatedJpeg) =>
            RunProgramOnce(program, src, out annotatedJpeg, out _);

        // ── Previews ──────────────────────────────────────────────────────────────

        /// <summary>A layer of <paramref name="program"/> on a JPEG frame, at filter index <paramref name="upTo"/> (null = all filters).</summary>
        public static byte[]? RenderLayer(VisionProgram program, byte[]? jpeg, string layerId, int? upTo)
        {
            if (jpeg == null) return null;
            using var src = Cv2.ImDecode(jpeg, ImreadModes.Color);
            if (src.Empty()) return null;
            using var cache = new LayerCache(src, program.Layers);
            if (layerId == VisionTool.CameraInput) return VisionDrawing.EncodeJpeg(src, VisionDrawing.DebugJpegQuality);
            using var img = cache.Preview(layerId, upTo);
            using var shown = new Mat();
            if (img.Depth() != MatType.CV_8U) img.ConvertTo(shown, MatType.CV_8U); else img.CopyTo(shown);
            return VisionDrawing.EncodeJpeg(shown, VisionDrawing.DebugJpegQuality);
        }

        /// <summary>
        /// One tool's result drawn over its own input layer (gray layers shown in colour), on a
        /// JPEG frame. The tool's result is <paramref name="result"/>'s when given, else the
        /// program is run on the frame.
        /// </summary>
        public static byte[]? RenderToolDebug(VisionProgram program, byte[]? jpeg, string toolId, VisionResult? result)
        {
            if (jpeg == null) return null;
            var tool = program.Tool(toolId);
            if (tool == null) return null;
            using var src = Cv2.ImDecode(jpeg, ImreadModes.Color);
            if (src.Empty()) return null;
            if (result?.Tool(toolId) == null)
            {
                using var rt = new VisionRuntime();
                // A throwaway copy: a debug render must not capture references.
                var copy = System.Text.Json.JsonSerializer.Deserialize<VisionProgram>(System.Text.Json.JsonSerializer.Serialize(program))!;
                result = VisionEngine.Execute(copy, src, rt).Result;
            }
            var tr = result!.Tool(toolId);
            using var cache = new LayerCache(src, program.Layers);
            var input = cache.Exists(tool.Input) ? cache.Get(tool.Input) : src;
            using var canvas = new Mat();
            if (input.Channels() == 1)
            {
                using var u8 = new Mat();
                input.ConvertTo(u8, MatType.CV_8U);
                Cv2.CvtColor(u8, canvas, ColorConversionCodes.GRAY2BGR);
            }
            else input.CopyTo(canvas);
            if (tr != null)
            {
                VisionOverlay.DrawTool(canvas, tr);
                var text = $"{(string.IsNullOrEmpty(tool.Name) ? tool.Id : tool.Name)}: {tr.Status}{(tr.Message != null ? " - " + tr.Message : "")}";
                VisionDrawing.DrawOutlinedText(canvas, text, new OpenCvSharp.Point(VisionDrawing.LeftMargin, 20), VisionDrawing.StatusFontScale,
                    tr.Passed ? VisionPalette.Pass : VisionPalette.Fail);
            }
            return VisionDrawing.EncodeJpeg(canvas, VisionDrawing.DebugJpegQuality);
        }
    }
}
