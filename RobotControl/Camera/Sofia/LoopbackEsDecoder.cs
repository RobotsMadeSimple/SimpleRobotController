using OpenCvSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace Controller.RobotControl.Camera.Sofia
{
    /// <summary>
    /// In-process decoder (<c>decoder: "opencv"</c>): the elementary stream is served on a
    /// loopback TCP socket and opened by OpenCV's FFmpeg backend as
    /// <c>tcp://127.0.0.1:&lt;port&gt;</c>.
    ///
    /// <para><b>Probing.</b> OpenCV cannot force an input format, so FFmpeg's raw
    /// <c>h264</c>/<c>hevc</c> demuxer has to <i>probe</i> the Annex-B bytes to recognise them
    /// (it scores SPS/PPS/IDR NALs). Frames written before OpenCV connects are therefore held
    /// back, starting from the latest key frame (an XMeye I-frame carries the parameter sets),
    /// and flushed the moment the connection is accepted, so the probe sees
    /// SPS/PPS/IDR immediately. <see cref="Open"/> waits for a key frame plus a few more frames
    /// (or half its budget) before opening. If the probe or the decoder never produces a
    /// picture, the capture loop tears the session down and reconnects.</para>
    ///
    /// <para><b>Latency options</b> (process-wide <c>OPENCV_FFMPEG_CAPTURE_OPTIONS</c>, set just
    /// before the open, last writer wins — see <see cref="NetworkCameraSource.SetCaptureOptions"/>):
    /// <c>probesize;32768|analyzeduration;0|flags;low_delay</c>, and the open parameter
    /// <c>CAP_PROP_N_THREADS = 1</c>. Measured with an MSMF-encoded 320x240 stream:
    /// FFmpeg's frame-threaded decoder held ~8 frames back until single-threading was requested;
    /// <c>fflags;nobuffer</c> is deliberately left out because it discards the packets read
    /// while probing — including the key frame — so the first picture then waits a whole GOP.
    /// The RTSP transport option of the network path is not included (it does not apply to a
    /// <c>tcp://</c> input); network opens write their own value before opening.</para>
    /// </summary>
    public sealed class LoopbackEsDecoder : SofiaDecoder
    {
        public const string CaptureOptions = "probesize;32768|analyzeduration;0|flags;low_delay";

        // cv::CAP_PROP_N_THREADS (OpenCV >= 4.6) as an open parameter
        private const int CapPropNThreads = 70;

        private const int PrefeedFrames   = 3;
        private const int MaxPendingBytes = 16 * 1024 * 1024;

        private readonly TcpListener _listener;
        private readonly string      _tag;
        private readonly object      _lock = new();
        private readonly List<byte[]> _pending = new();
        private int          _pendingBytes;
        private bool         _pendingHasKey;
        private Socket?      _conn;
        private volatile bool _aborted;
        private VideoCapture? _cap;          // owner thread only
        private readonly int  _readTimeoutMs;
        private readonly ManualResetEventSlim _prefed = new(false);

        public int Port { get; }
        public override string Kind => DecoderOpenCv;

        public LoopbackEsDecoder(string tag, int readTimeoutMs = NetworkCameraSource.DefaultReadTimeoutMs)
        {
            if (!NetworkCameraSource.FfmpegAvailable)
                throw new DecoderUnavailableException("this OpenCV build has no FFmpeg backend");
            _tag           = tag;
            _readTimeoutMs = readTimeoutMs;
            _listener      = new TcpListener(IPAddress.Loopback, 0);
            _listener.Start(1);
            Port = ((IPEndPoint)_listener.LocalEndpoint).Port;
            _listener.BeginAcceptSocket(OnAccept, null);
        }

        private void OnAccept(IAsyncResult ar)
        {
            Socket s;
            try { s = _listener.EndAcceptSocket(ar); }
            catch { return; }   // listener stopped

            lock (_lock)
            {
                if (_aborted || _conn != null) { try { s.Close(0); } catch { } return; }
                s.NoDelay = true;
                _conn = s;
                // Flush what arrived before FFmpeg connected, starting at the key frame.
                try
                {
                    foreach (var f in _pending) SendAll(s, f);
                }
                catch (Exception) { /* the next Write reports it */ }
                _pending.Clear();
                _pendingBytes = 0;
            }
            // Only one connection is ever served.
            try { _listener.Stop(); } catch { }
        }

        private static void SendAll(Socket s, byte[] data)
        {
            int off = 0;
            while (off < data.Length)
            {
                int n = s.Send(data, off, data.Length - off, SocketFlags.None);
                if (n <= 0) throw new IOException("loopback decoder connection closed");
                off += n;
            }
        }

        public override void Write(byte[] frame, bool isKeyFrame)
        {
            if (_aborted) throw new IOException("decoder closed");
            Socket? conn;
            lock (_lock)
            {
                conn = _conn;
                if (conn == null)
                {
                    // Not connected yet: keep from the latest key frame so the probe starts on SPS/PPS/IDR.
                    if (isKeyFrame) { _pending.Clear(); _pendingBytes = 0; _pendingHasKey = true; }
                    if (_pendingBytes + frame.Length > MaxPendingBytes) { _pending.Clear(); _pendingBytes = 0; _pendingHasKey = false; }
                    _pending.Add(frame);
                    _pendingBytes += frame.Length;
                    if ((_pendingHasKey && _pending.Count >= PrefeedFrames)) _prefed.Set();
                    return;
                }
            }
            try { SendAll(conn, frame); }
            catch (Exception ex) when (ex is SocketException or ObjectDisposedException)
            {
                throw new IOException("loopback decoder connection closed", ex);
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
            _cap = NetworkCameraSource.OpenFfmpeg($"tcp://127.0.0.1:{Port}", remaining, _readTimeoutMs, CapPropNThreads, 1);
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
            Socket? conn;
            lock (_lock) { conn = _conn; _pending.Clear(); _pendingBytes = 0; }
            try { _listener.Stop(); } catch { }
            // Closing our end makes FFmpeg's read return EOF, so a blocked cap.Read returns.
            if (conn != null)
            {
                try { conn.Shutdown(SocketShutdown.Both); } catch { }
                try { conn.Close(0); } catch { }
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
            // _prefed is left to the GC: the pump thread may still Set it while unwinding.
        }
    }
}
