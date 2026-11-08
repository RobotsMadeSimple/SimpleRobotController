using Controller.RobotControl.Camera.Sofia;
using OpenCvSharp;
using SIPSorcery.Net;
using SIPSorceryMedia.Abstractions;
using System;
using System.Collections.Generic;
using System.Net;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace Controller.RobotControl.Camera.WebRtc
{
    /// <summary>
    /// Receives a phone's <b>H.264 WebRTC</b> video stream and feeds it into the existing camera
    /// pipeline. It is an alternate <i>transport</i> for a <c>"phone"</c> camera (not a new source
    /// type): <see cref="H264RtpAssembler"/> turns the RTP packets into Annex-B access units (only
    /// complete, decodable ones), the reused Sofia decoder (<see cref="LoopbackEsDecoder"/>, OpenCV's
    /// bundled FFmpeg — no external exe) turns them into JPEGs, and each JPEG is handed to
    /// <see cref="CameraDevice.PushFrame"/>, so snapshot, <c>/camera/{id}/ws</c>, vision and
    /// calibration all see it unchanged.
    ///
    /// <para><b>Loss recovery.</b> A lost packet is NACKed (the phone retransmits it); if it still
    /// doesn't come, the broken frame and everything after it are held back until a complete key
    /// frame, which is requested at once with RTCP PLI (and again, backing off, until it comes). The
    /// picture freezes on the last clean frame for a moment instead of smearing. A periodic PLI is a
    /// slow safety refresh only.</para>
    ///
    /// <para><b>Threading.</b> SIPSorcery's single RTP receive thread hands packets to the assembler
    /// (<see cref="OnRtp"/>), which must never block — a blocked receive thread overflows the UDP
    /// buffer and loses packets. Frames go into a small queue; a feeder thread writes them into the
    /// decoder, and the reader thread pumps decoded pictures to <see cref="CameraDevice.PushFrame"/>.
    /// If the decoder falls behind, the queue is dropped and the stream resyncs on a key frame.
    /// One receiver per signaling socket; disposed when that socket closes or the peer connection fails.</para>
    /// </summary>
    public sealed class WebRtcCameraReceiver : IDisposable
    {
        private readonly CameraDevice _cam;
        private readonly string       _tag;
        private readonly RTCPeerConnection _pc;

        private SofiaDecoder? _decoder;
        private Thread?       _reader;
        private Thread?       _feeder;
        private volatile bool _running = true;
        private System.Threading.Timer? _watchdogTimer;
        private System.Threading.Timer? _rtcpTimer;
        private int           _ended;        // 1 once the session has stopped (End ran)
        private long          _startedMs;    // when the answer went out (watchdog grace starts here)
        private long          _lastRtpMs;    // last video packet from the phone (0 = none yet)
        private long          _lastDecodedMs; // last decoded picture (0 = none yet)
        private long          _frames;       // decoded pictures pushed to the camera
        private int           _localIce;
        private int           _remoteIce;

        // RTP → frames. Guarded by _asmLock (RTP thread + the RTCP timer).
        private readonly object _asmLock = new();
        private readonly H264RtpAssembler _asm = new();
        private readonly List<H264RtpAssembler.Frame> _asmFrames = new();
        private readonly List<ushort> _asmNacks = new();

        // Frames waiting for the decoder (feeder thread). Guarded by _queueLock.
        private readonly object _queueLock = new();
        private readonly Queue<H264RtpAssembler.Frame> _queue = new();
        private readonly AutoResetEvent _queueSignal = new(false);
        private long _decoderOverflows;

        // Key-frame requests.
        private long _lastPliMs;
        private int  _pliAttempt;           // consecutive PLIs while waiting (drives the backoff)
        private long _plisSent;
        private long _brokenSinceMs;        // when the picture last broke (0 = clean)
        private string? _brokenReason;

        // Once-a-second stats.
        private long _statsAtMs;
        private Snapshot _prevStats;

        // 90 (not 75): the decoded H.264 is re-encoded to JPEG for snapshot + vision/calibration,
        // so this is the round-trip quality ceiling — 75 stacked visible artifacts on top of H.264.
        private static readonly int[] JpegParams = { (int)ImwriteFlags.JpegQuality, 90 };

        /// <summary>Safety refresh: ask for a key frame this often even when nothing was lost. Loss is
        /// handled on demand (NACK, then PLI), so this stays slow — every key frame at a high bitrate is a
        /// big burst, and bursts are what Wi-Fi drops.</summary>
        internal const int KeyframeRefreshMs = 5000;
        /// <summary>While waiting for a key frame, repeat the PLI after 250 ms, doubling up to this.</summary>
        internal const int PliRetryMaxMs = 2000;
        internal const int PliRetryBaseMs = 250;
        private  const int RtcpTickMs = 50;
        private  const int StatsEveryMs = 1000;

        /// <summary>Frames queued for the decoder beyond this mean it can't keep up: drop them and resync.</summary>
        internal const int MaxQueuedFrames = 45;

        /// <summary>Kernel receive buffer for the RTP socket. A 1080p key frame at 16 Mbps is ~180 KB in
        /// one burst; give the receive thread room to breathe.</summary>
        private const int RtpSocketBufferBytes = 4 * 1024 * 1024;

        // The live /ws preview gets a downscaled, lighter JPEG so it doesn't flood the network at full
        // 1080p (a full-res MJPEG round-trip is ~5x the phone's upload bitrate and lags over WiFi).
        // Snapshot + vision still read the full-resolution frame.
        private const int PreviewMaxEdge = 720;
        private static readonly int[] PreviewJpegParams = { (int)ImwriteFlags.JpegQuality, 80 };
        private const int OpenTimeoutMs = 10000;

        // Stall watchdog. The peer connection can stay "connected" (ICE/DTLS keep-alives still flow)
        // while no video does — e.g. Android hands the phone's camera to another app — or while video
        // arrives but nothing decodable does (packets lost and no key frame comes). Either way the
        // session is ended and the signaling socket closed, so the phone sees the drop and reconnects
        // instead of silently streaming nothing.
        internal const int StallMs        = 6000;    // no new clean picture for this long once streaming
        internal const int FirstPictureMs = 15000;   // grace for the first picture after the answer
        private  const int WatchdogTickMs = 1000;

        /// <summary>A local ICE candidate was gathered — forward it to the phone (JSON per the W3C shape).</summary>
        public event Action<string>? OnLocalIceCandidate;
        /// <summary>The peer connection state changed (connected / failed / closed …).</summary>
        public event Action<RTCPeerConnectionState>? OnConnectionStateChanged;
        /// <summary>The session stopped on its own (stall, decoder end, peer failure). Carries a short reason.
        /// Not raised by <see cref="Dispose"/>.</summary>
        public event Action<string>? OnEnded;
        /// <summary>Once a second: the receive side's stats as a JSON object (see <see cref="StatsJson"/>),
        /// for the phone to show next to its own sender stats.</summary>
        public event Action<string>? OnStats;

        public long FrameCount => Interlocked.Read(ref _frames);
        /// <summary>Why the session stopped, once it has.</summary>
        public string? EndReason { get; private set; }

        public WebRtcCameraReceiver(CameraDevice cam, RTCConfiguration? config = null)
        {
            _cam = cam;
            _tag = cam.Id;
            _pc  = new RTCPeerConnection(config ?? new RTCConfiguration());

            // Receive-only H.264 video. The payload id is a placeholder; negotiation against the
            // phone's offer settles the actual one.
            var videoTrack = new MediaStreamTrack(
                new List<VideoFormat> { new VideoFormat(VideoCodecsEnum.H264, 102) },
                MediaStreamStatusEnum.RecvOnly);
            _pc.addTrack(videoTrack);

            // Raw packets, not OnVideoFrameReceived: SIPSorcery's framer can't tell a complete frame
            // from one with lost packets (and with no frame subscriber it skips that work entirely).
            _pc.OnRtpPacketReceived += OnRtp;
            _asm.KeyFrameNeeded += OnPictureBroke;
            _pc.onicecandidate += c =>
            {
                Interlocked.Increment(ref _localIce);
                Console.WriteLine($"[WebRTC] {_tag} local cand: {c.candidate}");
                try { OnLocalIceCandidate?.Invoke(c.toJSON()); } catch { /* best effort */ }
            };
            _pc.oniceconnectionstatechange += s => Console.WriteLine($"[WebRTC] {_tag} ICE state={s}");
            _pc.onconnectionstatechange += state =>
            {
                Console.WriteLine($"[WebRTC] {_tag} pc state={state} (frames={FrameCount}, localICE={_localIce}, remoteICE={_remoteIce})");
                try { OnConnectionStateChanged?.Invoke(state); } catch { }
                if (state is RTCPeerConnectionState.failed
                          or RTCPeerConnectionState.disconnected
                          or RTCPeerConnectionState.closed)
                    End($"peer connection {state}");
            };
        }

        /// <summary>
        /// Stops the session once, logs why, and (unless <paramref name="notify"/> is false) raises
        /// <see cref="OnEnded"/> so the signaling socket can be closed and the phone reconnects.
        /// Aborting the decoder unblocks a reader stuck in a read.
        /// </summary>
        private void End(string reason, bool notify = true)
        {
            if (Interlocked.Exchange(ref _ended, 1) == 1) return;
            _running = false;
            EndReason = reason;
            var s = TakeSnapshot();
            Console.WriteLine($"[WebRTC] {_tag} stream stopped: {reason} (packets={s.Packets}, lost={s.Lost}, recovered={s.Recovered}, " +
                              $"frames={s.FramesOut + s.Dropped}, dropped={s.Dropped}, decoded={FrameCount}, keyRequests={s.Plis})");
            try { _watchdogTimer?.Dispose(); } catch { }
            try { _rtcpTimer?.Dispose(); } catch { }
            try { _queueSignal.Set(); } catch { }
            try { _decoder?.Abort(); } catch { }
            if (notify) { try { OnEnded?.Invoke(reason); } catch { } }
        }

        /// <summary>
        /// The watchdog's decision: null while healthy, else why the stream counts as stopped.
        /// All times are <see cref="Environment.TickCount64"/> ms; 0 means "never".
        /// <paramref name="awaitingKeyFrame"/>: packets were lost and the picture is waiting for a key frame.
        /// </summary>
        internal static string? StallReason(long nowMs, long startedMs, long lastRtpMs, long lastDecodedMs, bool awaitingKeyFrame = false)
        {
            if (startedMs == 0) return null;   // not negotiated yet
            if (lastDecodedMs == 0)
            {
                if (nowMs - startedMs < FirstPictureMs) return null;
                return lastRtpMs == 0
                    ? $"no video arrived from the phone within {FirstPictureMs / 1000} s"
                    : awaitingKeyFrame
                        ? $"video arrived but no complete key frame within {FirstPictureMs / 1000} s (packets lost)"
                        : $"video arrived but no picture was decoded within {FirstPictureMs / 1000} s";
            }
            if (nowMs - lastDecodedMs < StallMs) return null;
            if (lastRtpMs == 0 || nowMs - lastRtpMs >= StallMs)
                return $"no video from the phone for {StallMs / 1000} s (its camera stopped or the network dropped)";
            return awaitingKeyFrame
                ? $"video is arriving but no clean picture for {StallMs / 1000} s (packets lost; no complete key frame came)"
                : $"video is arriving but the decoder produced no picture for {StallMs / 1000} s";
        }

        private void CheckStall()
        {
            if (!_running) return;
            bool awaiting;
            lock (_asmLock) awaiting = _asm.AwaitingKeyFrame;
            var reason = StallReason(Environment.TickCount64, Interlocked.Read(ref _startedMs),
                                     Interlocked.Read(ref _lastRtpMs), Interlocked.Read(ref _lastDecodedMs), awaiting);
            if (reason != null) End(reason);
        }

        /// <summary>Applies the phone's SDP offer and returns the controller's SDP answer. Starts the decode loop.</summary>
        public async Task<string> AnswerAsync(string offerSdp)
        {
            var result = _pc.setRemoteDescription(new RTCSessionDescriptionInit
            {
                type = RTCSdpType.offer,
                sdp  = offerSdp,
            });
            if (result != SetDescriptionResultEnum.OK)
                throw new InvalidOperationException($"WebRTC offer rejected: {result}");

            var answer = _pc.createAnswer(null);
            await _pc.setLocalDescription(answer);

            try
            {
                var sock = _pc.VideoStream?.GetRTPChannel()?.RtpSocket;
                if (sock != null) sock.ReceiveBufferSize = Math.Max(sock.ReceiveBufferSize, RtpSocketBufferBytes);
            }
            catch { /* keep SIPSorcery's default */ }

            // Tell the phone we NACK and PLI, so its sender keeps a retransmission history.
            var sdp = WithVideoFeedback(answer.sdp ?? "");
            Console.WriteLine($"[WebRTC] {_tag} offer applied, answer sent (answer has H264={sdp.Contains("H264")}, nack={sdp.Contains(" nack")})");
            Interlocked.Exchange(ref _startedMs, Environment.TickCount64);
            StartReader();
            _rtcpTimer     = new System.Threading.Timer(_ => RtcpTick(), null, RtcpTickMs, RtcpTickMs);
            _watchdogTimer = new System.Threading.Timer(_ => CheckStall(), null, WatchdogTickMs, WatchdogTickMs);
            return sdp;
        }

        /// <summary>
        /// Adds <c>a=rtcp-fb:&lt;pt&gt; nack</c> and <c>nack pli</c> for every H.264 payload of the
        /// video section that lacks them. The phone's sender only keeps packets for retransmission
        /// (and answers PLI by the book) when the answer says the receiver uses them.
        /// </summary>
        internal static string WithVideoFeedback(string sdp)
        {
            if (string.IsNullOrEmpty(sdp)) return sdp;
            var nl = sdp.Contains("\r\n") ? "\r\n" : "\n";
            var lines = sdp.Split(nl);
            var outp = new List<string>(lines.Length + 8);
            bool inVideo = false;
            var h264 = new List<string>();
            var have = new HashSet<string>();
            // First pass: H.264 payload ids and the feedback already present.
            foreach (var l in lines)
            {
                if (l.StartsWith("m=")) inVideo = l.StartsWith("m=video");
                if (!inVideo) continue;
                if (l.StartsWith("a=rtpmap:") && l.Contains(" H264/", StringComparison.OrdinalIgnoreCase))
                    h264.Add(l["a=rtpmap:".Length..].Split(' ')[0]);
                if (l.StartsWith("a=rtcp-fb:")) have.Add(l.TrimEnd());
            }
            inVideo = false;
            foreach (var l in lines)
            {
                if (l.StartsWith("m=")) inVideo = l.StartsWith("m=video");
                outp.Add(l);
                if (!inVideo || !l.StartsWith("a=rtpmap:")) continue;
                var pt = l["a=rtpmap:".Length..].Split(' ')[0];
                if (!h264.Contains(pt)) continue;
                foreach (var fb in new[] { $"a=rtcp-fb:{pt} nack", $"a=rtcp-fb:{pt} nack pli" })
                    if (!have.Contains(fb)) outp.Add(fb);
            }
            return string.Join(nl, outp);
        }

        /// <summary>Adds a remote ICE candidate (W3C JSON: <c>{candidate, sdpMid, sdpMLineIndex}</c>).</summary>
        public void AddRemoteIceCandidate(string candidateJson)
        {
            try
            {
                using var doc = JsonDocument.Parse(candidateJson);
                var root = doc.RootElement;
                var init = new RTCIceCandidateInit
                {
                    candidate     = Str(root, "candidate"),
                    sdpMid        = Str(root, "sdpMid"),
                    sdpMLineIndex = (ushort)(root.TryGetProperty("sdpMLineIndex", out var i) && i.ValueKind == JsonValueKind.Number ? i.GetUInt16() : 0),
                };
                Console.WriteLine($"[WebRTC] {_tag} remote cand: {init.candidate}");
                if (!string.IsNullOrEmpty(init.candidate)) { _pc.addIceCandidate(init); Interlocked.Increment(ref _remoteIce); }
            }
            catch (Exception ex) { Console.WriteLine($"[WebRTC] {_tag} bad ICE candidate: {ex.Message}"); }
        }

        private static string Str(JsonElement e, string name) =>
            e.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() ?? "" : "";

        // ── RTP in (SIPSorcery's receive thread: never block here) ─────────────────

        private void OnRtp(IPEndPoint ep, SDPMediaTypesEnum media, RTPPacket pkt)
        {
            if (media != SDPMediaTypesEnum.video || !_running || _decoder == null) return;
            Interlocked.Exchange(ref _lastRtpMs, Environment.TickCount64);
            var h = pkt.Header;
            var payload = pkt.GetPayloadBytes();
            bool paddingOnly = h.PaddingFlag == 1 && h.PaddingCount >= payload.Length;
            List<ushort>? nacks = null;
            lock (_asmLock)
            {
                _asm.Push(h.SequenceNumber, h.Timestamp, h.MarkerBit == 1, payload, paddingOnly,
                          Environment.TickCount64, _asmFrames, _asmNacks);
                FlushAssembled(ref nacks);
            }
            if (nacks != null) SendNacks(nacks);
        }

        /// <summary>Moves assembled frames to the decoder queue and takes the NACK list. Caller holds _asmLock.</summary>
        private void FlushAssembled(ref List<ushort>? nacks)
        {
            if (_asmNacks.Count > 0) { nacks = new List<ushort>(_asmNacks); _asmNacks.Clear(); }
            if (_asmFrames.Count == 0) return;
            bool overflow = false;
            lock (_queueLock)
            {
                foreach (var f in _asmFrames)
                {
                    if (_queue.Count >= MaxQueuedFrames && !f.IsKey) { overflow = true; break; }
                    if (f.IsKey) _queue.Clear();   // nothing before a key frame is needed any more
                    _queue.Enqueue(f);
                }
                if (overflow) _queue.Clear();
            }
            _asmFrames.Clear();
            _queueSignal.Set();
            if (overflow)
            {
                Interlocked.Increment(ref _decoderOverflows);
                _asm.RequireKeyFrame("the decoder fell behind (frames dropped)");
            }
        }

        private void SendNacks(List<ushort> seqs)
        {
            try
            {
                var vs = _pc.VideoStream;
                var remote = vs?.RemoteTrack?.Ssrc ?? 0;
                if (vs == null || remote == 0) return;
                var local = vs.LocalTrack?.Ssrc ?? 0;
                foreach (var (pid, blp) in H264RtpAssembler.PackNacks(seqs))
                    vs.SendRtcpFeedback(new RTCPFeedback(local, remote, RTCPFeedbackTypesEnum.NACK, pid, blp));
            }
            catch { /* connection not ready / torn down */ }
        }

        // ── Key frames, give-up timers, stats (timer thread) ─────────────────────────

        private void OnPictureBroke(string reason)
        {
            // Runs under _asmLock (raised by the assembler). Ask right away; the tick retries with backoff.
            if (_brokenSinceMs == 0) _brokenSinceMs = Environment.TickCount64;
            _brokenReason = reason;
            Console.WriteLine($"[WebRTC] {_tag} picture broke: {reason} — holding the last clean frame, asking the phone for a key frame");
            _pliAttempt = 0;
            _lastPliMs = 0;
        }

        /// <summary>
        /// When to send the next PLI: while waiting for a key frame, at once and then backing off from
        /// <see cref="PliRetryBaseMs"/> to <see cref="PliRetryMaxMs"/>; otherwise every
        /// <see cref="KeyframeRefreshMs"/> as a safety refresh. Returns true when one is due now.
        /// </summary>
        internal static bool PliDue(long nowMs, long lastPliMs, bool awaitingKeyFrame, int attempt, bool videoFlowing)
        {
            if (!videoFlowing) return false;                 // nobody to ask (the stall watchdog handles it)
            if (lastPliMs == 0) return true;
            if (!awaitingKeyFrame) return nowMs - lastPliMs >= KeyframeRefreshMs;
            long wait = Math.Min(PliRetryMaxMs, (long)PliRetryBaseMs << Math.Min(attempt, 6));
            return nowMs - lastPliMs >= wait;
        }

        private int _inTick;

        private void RtcpTick()
        {
            if (!_running || Interlocked.Exchange(ref _inTick, 1) == 1) return;   // timer callbacks can overlap
            try { RtcpTickCore(); } finally { Volatile.Write(ref _inTick, 0); }
        }

        private void RtcpTickCore()
        {
            long now = Environment.TickCount64;
            List<ushort>? nacks = null;
            bool awaiting, wasBroken = _brokenSinceMs != 0;
            lock (_asmLock)
            {
                _asm.Poll(now, _asmFrames, _asmNacks);
                FlushAssembled(ref nacks);
                awaiting = _asm.AwaitingKeyFrame;
            }
            if (nacks != null) SendNacks(nacks);

            long lastRtp = Interlocked.Read(ref _lastRtpMs);
            bool flowing = lastRtp != 0 && now - lastRtp < 2000;
            if (PliDue(now, _lastPliMs, awaiting, _pliAttempt, flowing))
            {
                if (RequestKeyFrame()) { _lastPliMs = now; if (awaiting) _pliAttempt++; }
            }
            if (!awaiting)
            {
                _pliAttempt = 0;
                if (wasBroken)
                {
                    Console.WriteLine($"[WebRTC] {_tag} picture recovered after {now - _brokenSinceMs} ms (key frame)");
                    _brokenSinceMs = 0;
                    _brokenReason = null;
                }
            }

            if (now - _statsAtMs >= StatsEveryMs) EmitStats(now, awaiting);
        }

        /// <summary>Ask the phone to emit a key frame now (RTCP PLI), once the remote video SSRC is known.</summary>
        private bool RequestKeyFrame()
        {
            if (!_running) return false;
            try
            {
                var vs = _pc.VideoStream;
                var remote = vs?.RemoteTrack?.Ssrc ?? 0;
                if (vs == null || remote == 0) return false;   // not flowing yet
                var local = vs.LocalTrack?.Ssrc ?? 0;
                vs.SendRtcpFeedback(new RTCPFeedback(local, remote, PSFBFeedbackTypesEnum.PLI));
                Interlocked.Increment(ref _plisSent);
                return true;
            }
            catch { return false; /* connection not ready / torn down — the timer stops on End */ }
        }

        private readonly record struct Snapshot(long Packets, long Lost, long Recovered, long Nacks, long FramesOut,
                                                long Dropped, long Decoded, long Plis, long KeyFrames, long Overflows);

        private Snapshot TakeSnapshot()
        {
            lock (_asmLock)
                return new Snapshot(_asm.PacketsReceived, _asm.PacketsLost, _asm.PacketsRecovered, _asm.NacksSent,
                                    _asm.FramesOut, _asm.FramesDropped, Interlocked.Read(ref _frames),
                                    Interlocked.Read(ref _plisSent), _asm.KeyFramesOut, Interlocked.Read(ref _decoderOverflows));
        }

        private void EmitStats(long now, bool awaiting)
        {
            var cur = TakeSnapshot();
            var prev = _prevStats;
            double secs = _statsAtMs == 0 ? 1 : Math.Max(0.001, (now - _statsAtMs) / 1000.0);
            _prevStats = cur;
            _statsAtMs = now;
            long lastPic = Interlocked.Read(ref _lastDecodedMs);
            var json = StatsJson(
                receivedFps: (cur.FramesOut + cur.Dropped - prev.FramesOut - prev.Dropped) / secs,
                decodedFps:  (cur.Decoded - prev.Decoded) / secs,
                droppedTotal: cur.Dropped, lostTotal: cur.Lost, recoveredTotal: cur.Recovered,
                keyRequests: cur.Plis, keyFrames: cur.KeyFrames, awaitingKeyFrame: awaiting,
                msSincePicture: lastPic == 0 ? -1 : now - lastPic,
                reason: awaiting ? _brokenReason : null);
            try { OnStats?.Invoke(json); } catch { }

            // A log line every 10 s while things go wrong, so the console tells the story.
            if ((cur.Lost > prev.Lost || cur.Dropped > prev.Dropped) && now - _lastLossLogMs >= 10_000)
            {
                _lastLossLogMs = now;
                Console.WriteLine($"[WebRTC] {_tag} loss: packets lost={cur.Lost} recovered={cur.Recovered} nacked={cur.Nacks}, " +
                                  $"frames dropped={cur.Dropped} decoded={cur.Decoded}, key requests={cur.Plis} key frames={cur.KeyFrames}, decoder overflows={cur.Overflows}");
            }
        }
        private long _lastLossLogMs;

        /// <summary>The receive-side stats the phone shows: <c>{type:"stats", rxFps, decodedFps, dropped,
        /// lost, recovered, keyRequests, keyFrames, waitingForKeyFrame, msSincePicture, reason?}</c>.</summary>
        internal static string StatsJson(double receivedFps, double decodedFps, long droppedTotal, long lostTotal, long recoveredTotal,
                                         long keyRequests, long keyFrames, bool awaitingKeyFrame, long msSincePicture, string? reason)
        {
            var sb = new StringBuilder(200);
            sb.Append("{\"type\":\"stats\"");
            sb.Append(",\"rxFps\":").Append(Math.Round(receivedFps, 1).ToString(System.Globalization.CultureInfo.InvariantCulture));
            sb.Append(",\"decodedFps\":").Append(Math.Round(decodedFps, 1).ToString(System.Globalization.CultureInfo.InvariantCulture));
            sb.Append(",\"dropped\":").Append(droppedTotal);
            sb.Append(",\"lost\":").Append(lostTotal);
            sb.Append(",\"recovered\":").Append(recoveredTotal);
            sb.Append(",\"keyRequests\":").Append(keyRequests);
            sb.Append(",\"keyFrames\":").Append(keyFrames);
            sb.Append(",\"waitingForKeyFrame\":").Append(awaitingKeyFrame ? "true" : "false");
            sb.Append(",\"msSincePicture\":").Append(msSincePicture);
            if (reason != null) sb.Append(",\"reason\":").Append(JsonSerializer.Serialize(reason));
            sb.Append('}');
            return sb.ToString();
        }

        // ── Decoder ───────────────────────────────────────────────────────────────

        private void StartReader()
        {
            // OpenCV/FFmpeg loopback decoder: no external ffmpeg binary, reuses the bundled plugin.
            _decoder = new LoopbackEsDecoder(_tag, NetworkCameraSource.DefaultReadTimeoutMs);
            _feeder  = new Thread(FeederLoop) { IsBackground = true, Name = $"WebRtcFeed-{_tag}" };
            _feeder.Start();
            _reader  = new Thread(ReaderLoop) { IsBackground = true, Name = $"WebRtc-{_tag}" };
            _reader.Start();
        }

        /// <summary>Writes queued access units into the decoder (may block when FFmpeg is busy — that's
        /// why it's off the RTP thread).</summary>
        private void FeederLoop()
        {
            var dec = _decoder;
            if (dec == null) return;
            while (_running)
            {
                H264RtpAssembler.Frame f;
                lock (_queueLock)
                {
                    if (_queue.Count == 0) f = default;
                    else f = _queue.Dequeue();
                }
                if (f.AnnexB == null) { _queueSignal.WaitOne(200); continue; }
                try { dec.Write(f.AnnexB, f.IsKey); }
                catch { return; /* decoder gone away — the reader loop ends the session */ }
            }
        }

        private void ReaderLoop()
        {
            var dec = _decoder;
            if (dec == null) return;
            try
            {
                dec.Open(OpenTimeoutMs);   // waits for the first key frame, then opens the decoder
                Console.WriteLine($"[WebRTC] {_tag} decoder open; streaming");
                while (_running)
                {
                    using var pic = dec.Read();
                    if (pic == null)
                    {
                        // Read gives up after the decoder's read timeout with no data — the phone stopped
                        // sending (camera taken by another app, network drop), or everything that came was
                        // held back waiting for a key frame that never arrived.
                        var stale = Environment.TickCount64 - Interlocked.Read(ref _lastRtpMs);
                        bool awaiting; lock (_asmLock) awaiting = _asm.AwaitingKeyFrame;
                        End(_lastRtpMs == 0 || stale >= NetworkCameraSource.DefaultReadTimeoutMs - 500
                            ? $"no video from the phone for {NetworkCameraSource.DefaultReadTimeoutMs / 1000} s (its camera stopped or the network dropped)"
                            : awaiting
                                ? $"video is arriving but no clean picture for {NetworkCameraSource.DefaultReadTimeoutMs / 1000} s (packets lost; no complete key frame came)"
                                : "the decoder stopped producing pictures");
                        break;
                    }
                    _cam.PushFrame(pic.ToJpeg(JpegParams), pic.Width, pic.Height, MakePreview(pic));
                    Interlocked.Increment(ref _frames);
                    Interlocked.Exchange(ref _lastDecodedMs, Environment.TickCount64);
                }
            }
            catch (Exception ex)
            {
                End(Interlocked.Read(ref _lastRtpMs) == 0
                    ? $"no video arrived from the phone within {OpenTimeoutMs / 1000} s"
                    : $"decoder failed: {ex.Message}");
            }
            finally { _running = false; }
        }

        /// <summary>
        /// A downscaled, lighter JPEG of the decoded frame for the live preview (null when the frame
        /// is already small enough to reuse the full one). Cheap: one Area-interpolated resize + encode.
        /// </summary>
        private static byte[]? MakePreview(SofiaDecodedFrame pic)
        {
            var mat = pic.Mat;
            if (mat == null) return null;
            int longEdge = Math.Max(pic.Width, pic.Height);
            if (longEdge <= PreviewMaxEdge) return null;   // small enough — the full frame is the preview
            double scale = (double)PreviewMaxEdge / longEdge;
            int w = Math.Max(1, (int)Math.Round(pic.Width * scale));
            int h = Math.Max(1, (int)Math.Round(pic.Height * scale));
            try
            {
                using var small = new Mat();
                OpenCvSharp.Cv2.Resize(mat, small, new OpenCvSharp.Size(w, h), 0, 0, OpenCvSharp.InterpolationFlags.Area);
                OpenCvSharp.Cv2.ImEncode(".jpg", small, out var buf, PreviewJpegParams);
                return buf;
            }
            catch { return null; }   // on any encode error, fall back to the full frame
        }

        /// <summary>True when the Annex-B access unit carries an SPS (type 7) or IDR (type 5) NAL.</summary>
        internal static bool IsKeyFrame(byte[] annexB)
        {
            // Walk start codes (00 00 01 / 00 00 00 01) and inspect each NAL header's type (b &amp; 0x1F).
            int i = 0, n = annexB.Length;
            while (i + 3 < n)
            {
                bool sc3 = annexB[i] == 0 && annexB[i + 1] == 0 && annexB[i + 2] == 1;
                bool sc4 = i + 4 < n && annexB[i] == 0 && annexB[i + 1] == 0 && annexB[i + 2] == 0 && annexB[i + 3] == 1;
                if (!sc3 && !sc4) { i++; continue; }
                int hdr = i + (sc4 ? 4 : 3);
                if (hdr >= n) break;
                int type = annexB[hdr] & 0x1F;
                if (type == 5 || type == 7) return true;   // IDR or SPS
                i = hdr + 1;
            }
            return false;
        }

        public void Dispose()
        {
            End("signaling socket closed", notify: false);   // logs why, unless it already stopped
            try { _watchdogTimer?.Dispose(); } catch { }
            try { _rtcpTimer?.Dispose(); } catch { }
            try { _decoder?.Abort(); } catch { }
            try { if (_reader is { IsAlive: true } && _reader != Thread.CurrentThread) _reader.Join(1500); } catch { }
            try { if (_feeder is { IsAlive: true } && _feeder != Thread.CurrentThread) _feeder.Join(500); } catch { }
            try { _decoder?.Dispose(); } catch { }
            try { _pc.close(); } catch { }
            try { _pc.Dispose(); } catch { }
        }
    }
}
