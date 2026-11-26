using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;

namespace Controller.RobotControl.Camera.Sofia
{
    /// <summary>
    /// Linux in-process decoder (<c>decoder: "opencv"</c>): the elementary stream is written to a
    /// named FIFO and opened by OpenCV's FFmpeg backend as a plain path (the <c>file</c> protocol).
    ///
    /// <para>The self-contained OpenCvSharp FFmpeg build shipped on Linux has <b>no <c>tcp</c>
    /// network protocol</b>, so the loopback-TCP transport used on Windows
    /// (<see cref="LoopbackEsDecoder"/>) can never open there (<c>VideoCapture</c> returns closed
    /// with "can't be used to capture by name"). A FIFO is read through the <c>file</c> protocol,
    /// which the same build decodes fine. Probing, prefeed and latency options mirror
    /// <see cref="LoopbackEsDecoder"/> exactly.</para>
    ///
    /// <para><b>Rendezvous.</b> Opening a FIFO write-only blocks until a reader opens it, and
    /// read-only blocks until a writer opens it, so the writer thread and OpenCV's open meet at the
    /// FIFO. Frames that arrive before the reader connects are held back from the latest key frame
    /// (so the probe starts on SPS/PPS/IDR) and flushed the instant the write side opens.</para>
    /// </summary>
    public sealed class FifoEsDecoder : SofiaDecoder
    {
        public const string CaptureOptions = LoopbackEsDecoder.CaptureOptions;

        // cv::CAP_PROP_N_THREADS (OpenCV >= 4.6) as an open parameter
        private const int CapPropNThreads = 70;
        private const int PrefeedFrames   = 3;
        private const int MaxPendingBytes = 16 * 1024 * 1024;

        [DllImport("libc", SetLastError = true)] private static extern int mkfifo(string path, uint mode);
        [DllImport("libc", SetLastError = true)] private static extern int open(string path, int flags);
        [DllImport("libc", SetLastError = true)] private static extern int close(int fd);
        private const int O_RDONLY  = 0;
        private const int O_NONBLOCK = 2048;   // Linux value for O_NONBLOCK

        private readonly string _fifoPath;
        private readonly string _tag;
        private readonly int    _readTimeoutMs;
        private readonly object _lock = new();
        private readonly List<byte[]> _pending = new();
        private int          _pendingBytes;
        private bool         _pendingHasKey;
        private FileStream?  _writeFs;        // set once OpenCV opens the read side; owner: writer thread then any
        private volatile bool _aborted;
        private VideoCapture? _cap;           // owner thread only
        private readonly ManualResetEventSlim _prefed = new(false);
        private readonly Thread _writerOpen;

        public override string Kind => DecoderOpenCv;

        public FifoEsDecoder(string tag, int readTimeoutMs = NetworkCameraSource.DefaultReadTimeoutMs)
        {
            if (!NetworkCameraSource.FfmpegAvailable)
                throw new DecoderUnavailableException("this OpenCV build has no FFmpeg backend");
            _tag           = tag;
            _readTimeoutMs = readTimeoutMs;
            _fifoPath      = Path.Combine(Path.GetTempPath(), $"rmses_{Guid.NewGuid():N}.h264");
            if (mkfifo(_fifoPath, 0x180 /* 0600 */) != 0)
                throw new IOException($"mkfifo failed for {_fifoPath} (errno {Marshal.GetLastWin32Error()})");
            _writerOpen = new Thread(OpenWriteSide) { IsBackground = true, Name = $"FifoEsWriter-{tag}" };
            _writerOpen.Start();
        }

        // Blocks in open(O_WRONLY) until OpenCV opens the read side, then flushes buffered frames.
        private void OpenWriteSide()
        {
            FileStream fs;
            try { fs = new FileStream(_fifoPath, FileMode.Open, FileAccess.Write, FileShare.ReadWrite); }
            catch { return; }   // aborted / fifo removed before a reader arrived
            lock (_lock)
            {
                if (_aborted) { try { fs.Dispose(); } catch { } return; }
                _writeFs = fs;
                try { foreach (var f in _pending) fs.Write(f, 0, f.Length); fs.Flush(); }
                catch { /* the next Write reports it */ }
                _pending.Clear();
                _pendingBytes = 0;
            }
        }

        public override void Write(byte[] frame, bool isKeyFrame)
        {
            if (_aborted) throw new IOException("decoder closed");
            FileStream? ws;
            lock (_lock)
            {
                ws = _writeFs;
                if (ws == null)
                {
                    // Not connected yet: keep from the latest key frame so the probe starts on SPS/PPS/IDR.
                    if (isKeyFrame) { _pending.Clear(); _pendingBytes = 0; _pendingHasKey = true; }
                    if (_pendingBytes + frame.Length > MaxPendingBytes) { _pending.Clear(); _pendingBytes = 0; _pendingHasKey = false; }
                    _pending.Add(frame);
                    _pendingBytes += frame.Length;
                    if (_pendingHasKey && _pending.Count >= PrefeedFrames) _prefed.Set();
                    return;
                }
            }
            try { ws.Write(frame, 0, frame.Length); ws.Flush(); }
            catch (Exception ex) when (ex is IOException or ObjectDisposedException)
            {
                throw new IOException("fifo decoder connection closed", ex);
            }
        }

        public override void Open(int timeoutMs)
        {
            timeoutMs = Math.Max(500, timeoutMs);
            var sw = Stopwatch.StartNew();
            // Give the probe data to chew on: a key frame and a couple more, or half the budget.
            _prefed.Wait(timeoutMs / 2);
            if (_aborted) throw new IOException("decoder closed");

            int remaining = (int)Math.Max(500, timeoutMs - sw.ElapsedMilliseconds);
            NetworkCameraSource.SetCaptureOptions(CaptureOptions);
            // Opening the FIFO for read unblocks the writer thread, which flushes the buffered key frame.
            _cap = NetworkCameraSource.OpenFfmpeg(_fifoPath, remaining, _readTimeoutMs, CapPropNThreads, 1);
            if (_cap == null)
                throw new IOException($"OpenCV could not open the loopback elementary stream (probe failed){(_aborted ? " — aborted" : "")}");
        }

        public override SofiaDecodedFrame? Read()
        {
            var cap = _cap;
            if (cap == null || _aborted) return null;
            var mat = new Mat();
            try
            {
                if (cap.Read(mat) && !mat.Empty()) return SofiaDecodedFrame.FromMat(mat);
            }
            catch (Exception ex) { Console.WriteLine($"[Sofia] {_tag} decode error: {ex.Message}"); }
            mat.Dispose();
            return null;
        }

        public override void Abort()
        {
            _aborted = true;
            _prefed.Set();
            FileStream? ws;
            lock (_lock) { ws = _writeFs; _pending.Clear(); _pendingBytes = 0; }
            if (ws != null)
            {
                // Closing our end gives FFmpeg EOF, so a blocked cap.Read returns.
                try { ws.Dispose(); } catch { }
            }
            else
            {
                // OpenCV never opened the read side: the writer thread is still blocked in open(O_WRONLY).
                // A non-blocking read-open releases it so it sees _aborted and exits.
                int fd = open(_fifoPath, O_RDONLY | O_NONBLOCK);
                if (fd >= 0) close(fd);
            }
        }

        public override void Dispose()
        {
            Abort();
            var cap = _cap;
            _cap = null;
            if (cap != null)
            {
                try { cap.Release(); } catch { }
                try { cap.Dispose(); } catch { }
            }
            try { File.Delete(_fifoPath); } catch { }
            // _prefed is left to the GC: the pump thread may still Set it while unwinding.
        }
    }
}
