using Controller.RobotControl.Vision.Inspections;
using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Threading;

namespace Controller.RobotControl.Vision
{
    /// <summary>
    /// Runs a vision program's inspections against each new camera frame on a background
    /// thread, annotates the frame, and exposes the latest JPEGs and results for streaming.
    ///
    /// The per-inspection work lives in the strategies under Vision/Inspections; this class
    /// owns the thread, the frame pipeline (decode → shared FrameContext → inspections in
    /// InspectionOrder → encode) and publication.
    /// </summary>
    public class VisionProcessor
    {
        private const int NoFrameSleepMs     = 50;
        private const int SameFrameSleepMs   = 10;
        private const int ErrorSleepMs       = 200;
        private const int FramePeriodSleepMs = 50;   // ~20 fps
        private const int StopJoinTimeoutMs  = 2000;

        // Written by UpdateProgram (any thread) and read once per iteration by ProcessLoop
        // (the processing thread) — volatile so a write is guaranteed visible to the reader
        // without both sides needing Interlocked. The plan carries the program it was built from.
        private volatile InspectionPlan _plan;
        private readonly Camera.CameraDevice _camera;

        // ArUco detectors survive program updates (they are keyed by dictionary, not by
        // inspection) and are only released once the processing thread has stopped.
        private readonly ArucoStrategy _aruco = new();

        private byte[]?       _latestAnnotated;
        private VisionResult? _latestResult;
        private readonly object _lock = new();

        private Thread?       _thread;
        private volatile bool _running;

        private byte[]?       _latestRaw;
        private byte[]?       _lastProcessedJpeg;

        // Rate-limits per-inspection error logging: only re-logs when the message for a
        // given inspection id actually changes, so a persistently failing inspection does
        // not spam the console once per frame.
        private readonly Dictionary<string, string> _lastLoggedError = new();
        private readonly object _errorLogLock = new();

        public string ProgramId => _plan.Program.Id;

        public VisionProcessor(VisionProgram program, Camera.CameraDevice camera)
        {
            _plan   = InspectionPlan.Build(program, _aruco);
            _camera = camera;
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
                // Still mid-frame: releasing the detectors now could free one in use.
                Console.WriteLine($"[Vision] {ProgramId} processing thread did not stop within timeout");
                return;
            }
            _aruco.ReleaseDetectors();
        }

        /// <summary>Hot-swaps the program. Takes effect from the next frame.</summary>
        public void UpdateProgram(VisionProgram updated) =>
            _plan = InspectionPlan.Build(updated, _aruco);

        public byte[]? GetLatestAnnotated()
        {
            lock (_lock) return _latestAnnotated;
        }

        public byte[]? GetLatestRaw()
        {
            lock (_lock) return _latestRaw;
        }

        public VisionResult? GetLatestResult()
        {
            lock (_lock) return _latestResult;
        }

        /// <summary>
        /// Records a per-inspection failure on the result and logs it — but only when the
        /// message differs from the last one logged for that inspection id, so a steadily
        /// failing inspection logs once instead of once per frame.
        /// </summary>
        private void LogInspectionError(VisionResult result, string inspectionId, Exception ex)
        {
            var message = $"{inspectionId}: {ex.Message}";
            result.Errors.Add(message);

            bool changed;
            lock (_errorLogLock)
            {
                changed = !_lastLoggedError.TryGetValue(inspectionId, out var last) || last != message;
                _lastLoggedError[inspectionId] = message;
            }

            if (changed)
                Console.WriteLine($"[Vision] {result.ProgramId} inspection {message}");
        }

        // ── Processing loop ───────────────────────────────────────────────────────

        private void ProcessLoop()
        {
            while (_running)
            {
                try
                {
                    var jpeg = _camera.GetLatestFrame();
                    if (jpeg == null) { Thread.Sleep(NoFrameSleepMs); continue; }

                    // The camera thread publishes a new byte[] only when it actually encodes a
                    // new frame, so a reference match means we've already processed this exact
                    // frame — reprocessing it would just burn CPU for an identical result.
                    if (ReferenceEquals(jpeg, _lastProcessedJpeg)) { Thread.Sleep(SameFrameSleepMs); continue; }

                    using var src = Cv2.ImDecode(jpeg, ImreadModes.Color);
                    if (src.Empty()) { Thread.Sleep(NoFrameSleepMs); continue; }

                    var plan   = _plan;
                    var result = ProcessFrame(plan, src, out var annotatedJpeg);

                    lock (_lock)
                    {
                        _latestRaw       = jpeg;
                        _latestAnnotated = annotatedJpeg;
                        _latestResult    = result;
                    }

                    _lastProcessedJpeg = jpeg;
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

        /// <summary>Runs every step of the plan on one decoded frame (live loop: logs + records failures).</summary>
        private VisionResult ProcessFrame(InspectionPlan plan, Mat src, out byte[] annotatedJpeg) =>
            RunPlan(plan, src, out annotatedJpeg, LogInspectionError);

        /// <summary>
        /// Runs a saved program's inspections against a single supplied image — no camera and no
        /// processing thread, just the same per-frame pipeline the live loop uses. For the
        /// RunVisionOnImage API / testing: decode an image, get back the VisionResult (including
        /// per-cell color coverage) that a camera frame would have produced. Builds a throwaway
        /// plan and releases its ArUco detectors; a failing inspection is recorded on the result's
        /// Errors rather than logged.
        /// </summary>
        public static VisionResult RunProgramOnce(VisionProgram program, Mat src, out byte[] annotatedJpeg)
        {
            var aruco = new ArucoStrategy();
            try
            {
                var plan = InspectionPlan.Build(program, aruco);
                return RunPlan(plan, src, out annotatedJpeg,
                    (r, id, ex) => r.Errors.Add($"{id}: {ex.Message}"));
            }
            finally
            {
                aruco.ReleaseDetectors();
            }
        }

        /// <summary>
        /// Runs every step of a plan on one decoded frame: shared by the live loop and the
        /// one-shot <see cref="RunProgramOnce"/> path. <paramref name="onError"/> handles a step
        /// that throws (the loop logs and records; the one-shot only records on the result).
        /// </summary>
        private static VisionResult RunPlan(InspectionPlan plan, Mat src, out byte[] annotatedJpeg,
                                            Action<VisionResult, string, Exception>? onError)
        {
            var result = new VisionResult
            {
                ProgramId      = plan.Program.Id,
                TimestampMs    = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds(),
                ImageWidth     = src.Width,
                ImageHeight    = src.Height,
                Inspections    = new List<InspectionResult>(),
                ColorResults   = new List<ColorCoverageResult>(),
                PolygonResults = new List<PolygonResult>(),
                ArucoResults   = new List<ArucoResult>(),
                LineResults    = new List<LineResult>(),
                BarcodeResults = new List<BarcodeResult>(),
            };

            using var ctx       = new FrameContext(src);
            using var annotated = src.Clone();

            foreach (var zone in plan.BorderZones)
                VisionDrawing.DrawZoneBorder(annotated, zone.Geometry, ctx.Width, ctx.Height, zone.Name, VisionPalette.ZoneBorder);

            var labels = new LabelStack(ctx.Height);
            foreach (var step in plan.Steps)
            {
                var sw = Stopwatch.StartNew();
                try
                {
                    step.Execute(ctx, annotated, labels, result);
                }
                catch (Exception ex)
                {
                    step.RecordFailure(result);
                    onError?.Invoke(result, step.Id, ex);
                }
                result.Timings[step.Id] = Math.Round(sw.Elapsed.TotalMilliseconds, 1);
            }

            annotatedJpeg = VisionDrawing.EncodeJpeg(annotated, VisionDrawing.AnnotatedJpegQuality);
            return result;
        }

        // ── Debug frames ──────────────────────────────────────────────────────────

        /// <summary>The polygon pipeline's threshold mask and contour classification on the latest frame.</summary>
        public byte[]? GetPolygonDebugFrame(string inspectionId) =>
            RenderDebug(p => p.PolygonInspections, inspectionId, InspectionPlan.Polygon);

        /// <summary>The line pipeline's masked edges and Hough segments on the latest frame.</summary>
        public byte[]? GetLineDebugFrame(string inspectionId) =>
            RenderDebug(p => p.LineInspections, inspectionId, InspectionPlan.Line);

        private byte[]? RenderDebug<TInsp, TResult>(
            Func<VisionProgram, IEnumerable<TInsp>?> inspections, string inspectionId,
            IInspectionStrategy<TInsp, TResult> strategy)
            where TInsp : IVisionInspection
        {
            var prog = _plan.Program;
            var insp = (inspections(prog) ?? []).FirstOrDefault(i => i.Id == inspectionId);
            if (insp == null) return null;

            byte[]? raw;
            lock (_lock) raw = _latestRaw;
            if (raw == null) return null;

            using var src = Cv2.ImDecode(raw, ImreadModes.Color);
            if (src.Empty()) return null;

            using var ctx = new FrameContext(src);
            return strategy.RenderDebug(ctx, insp, InspectionPlan.ResolveZone(prog, insp.ZoneId));
        }
    }
}
