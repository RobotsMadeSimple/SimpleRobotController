using System;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Threading;

namespace Controller.RobotControl.Camera.Sofia
{
    /// <summary>Connection settings of a Sofia camera (normalised by <see cref="SofiaCameraSource.Settings"/>).</summary>
    public sealed record SofiaSettings(
        string Host, int Port, string Username, string Password, string Stream,
        string Codec, string Decoder, string FfmpegPath, string Hwaccel);

    /// <summary>
    /// One live Sofia connection: the <see cref="DvripClient"/>, the decoder, and the DVRIP pump
    /// thread that feeds the decoder. Used by the camera capture thread and by
    /// <c>TestCameraSource</c>.
    ///
    /// <para>Order: <see cref="Connect"/> → <see cref="ReadFirstFrame"/> (codec detection) →
    /// <see cref="StartDecoder"/> (creates the decoder for the effective codec, feeds it the first
    /// frame, starts the pump, opens the decoder) → <see cref="ReadDecoded"/> in a loop. The
    /// owner thread calls all of these and <see cref="Dispose"/>; <see cref="Abort"/> may be
    /// called from any thread to unblock everything (Stop, test timeout). A pump failure
    /// (socket error, resync giving up, decoder gone) aborts the session, so the owner's
    /// <see cref="ReadDecoded"/> returns null and the owner reconnects.</para>
    /// </summary>
    public sealed class SofiaSession : IDisposable
    {
        private readonly SofiaSettings _s;
        private readonly string        _tag;
        private readonly DvripClient   _client;
        private volatile SofiaDecoder? _decoder;
        private Thread?                _pump;
        private volatile bool          _aborted;
        private volatile Exception?    _pumpError;

        // Written-frame timestamps for the latency estimate (pump enqueues, owner dequeues).
        private readonly ConcurrentQueue<long> _written = new();
        private readonly Stopwatch _clock = Stopwatch.StartNew();

        public DvripClient Client        => _client;
        public SofiaDecoder? Decoder     => _decoder;
        /// <summary>Codec detected from the first video frame (<c>h264</c>/<c>hevc</c>/<c>unknown</c>).</summary>
        public string   DetectedCodec  { get; private set; } = DvripClient.CodecUnknown;
        /// <summary>Codec used for decoding: the detected one when known, else the configured one.</summary>
        public string   EffectiveCodec { get; private set; }
        public byte[]?  FirstFrame     { get; private set; }
        /// <summary>The error that ended the pump, if any.</summary>
        public Exception? PumpError    => _pumpError;
        /// <summary>Latest decode latency estimate (ms): wall time from a frame's arrival to its picture.</summary>
        public int LatencyMs { get; private set; }

        public SofiaSession(SofiaSettings settings, string tag)
        {
            _s   = settings;
            _tag = tag;
            EffectiveCodec = settings.Codec;
            _client = new DvripClient(settings.Host, settings.Port, settings.Username, settings.Password, settings.Stream);
        }

        /// <summary>TCP connect + login + claim/start within <paramref name="timeoutMs"/>. Throws <see cref="DvripException"/>.</summary>
        public void Connect(int timeoutMs)
        {
            if (_aborted) throw new DvripException(DvripStage.Connect, "aborted");
            _client.Connect(timeoutMs);
        }

        /// <summary>
        /// Blocks for the first video frame (the socket read timeout applies), detects its
        /// codec and logs it; a mismatch with the configured codec is a warning and the detected
        /// codec wins.
        /// </summary>
        public DvripVideoFrame ReadFirstFrame(int readTimeoutMs)
        {
            _client.ReadTimeoutMs = readTimeoutMs;
            var frame = _client.ReadVideoFrame();
            FirstFrame    = frame.Data;
            DetectedCodec = DvripClient.DetectCodec(frame.Data);
            DvripClient.LogFirstFrame(frame.Data, _tag);
            if (DetectedCodec != DvripClient.CodecUnknown)
            {
                if (DetectedCodec != _s.Codec)
                    Console.WriteLine($"[Sofia] WARNING: {_tag} is configured as {_s.Codec} but streams {DetectedCodec}; using {DetectedCodec}");
                EffectiveCodec = DetectedCodec;
            }
            return frame;
        }

        /// <summary>
        /// Creates the configured decoder, feeds it <paramref name="first"/>, starts the DVRIP pump
        /// thread and opens the decoder. Throws <see cref="DecoderUnavailableException"/> when the
        /// decoder cannot run, or an I/O error when it does not open.
        /// </summary>
        public void StartDecoder(DvripVideoFrame first, int openTimeoutMs, int streamReadTimeoutMs)
        {
            _decoder = _s.Decoder == SofiaDecoder.DecoderFfmpeg
                ? new FfmpegProcessDecoder(_tag, _s.FfmpegPath, EffectiveCodec, _s.Hwaccel)
                : new LoopbackEsDecoder(_tag, streamReadTimeoutMs);
            if (_aborted) { _decoder.Abort(); throw new System.IO.IOException("aborted"); }

            _client.ReadTimeoutMs = streamReadTimeoutMs;
            WriteFrame(first);
            _pump = new Thread(PumpLoop) { IsBackground = true, Name = $"Sofia-{_tag}-pump" };
            _pump.Start();
            _decoder.Open(openTimeoutMs);
        }

        private void WriteFrame(DvripVideoFrame f)
        {
            _decoder!.Write(f.Data, f.IsKeyFrame);
            _written.Enqueue(_clock.ElapsedMilliseconds);
            // A decoder that drops frames (e.g. before the first key frame) would make the
            // estimate drift; keep the queue bounded.
            while (_written.Count > 120 && _written.TryDequeue(out _)) { }
        }

        // DVRIP → decoder until the socket fails, the decoder goes away, or the session is aborted.
        private void PumpLoop()
        {
            try
            {
                while (!_aborted) WriteFrame(_client.ReadVideoFrame());
            }
            catch (Exception ex)
            {
                if (!_aborted) _pumpError = ex;
            }
            finally
            {
                // Whatever ended the pump ends the session: unblock the reader.
                Abort();
            }
        }

        /// <summary>Next decoded picture (owner thread); null when the session ended.</summary>
        public SofiaDecodedFrame? ReadDecoded()
        {
            var d = _decoder;
            if (d == null) return null;
            var frame = d.Read();
            if (frame != null && _written.TryDequeue(out long t))
                LatencyMs = (int)Math.Max(0, _clock.ElapsedMilliseconds - t);
            return frame;
        }

        /// <summary>Closes the DVRIP socket and the decoder input from any thread (idempotent).</summary>
        public void Abort()
        {
            _aborted = true;
            try { _client.Dispose(); } catch { }
            try { _decoder?.Abort(); } catch { }
        }

        public bool Aborted => _aborted;

        /// <summary>Owner thread: aborts, waits briefly for the pump and releases the decoder.</summary>
        public void Dispose()
        {
            Abort();
            var pump = _pump;
            if (pump != null && pump != Thread.CurrentThread) pump.Join(2000);
            try { _decoder?.Dispose(); } catch { }
        }
    }
}
