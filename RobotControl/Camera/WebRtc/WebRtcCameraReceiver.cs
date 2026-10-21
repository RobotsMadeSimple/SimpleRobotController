using Controller.RobotControl.Camera.Sofia;
using OpenCvSharp;
using SIPSorcery.Net;
using SIPSorceryMedia.Abstractions;
using System;
using System.Collections.Generic;
using System.Net;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace Controller.RobotControl.Camera.WebRtc
{
    /// <summary>
    /// Receives a phone's <b>H.264 WebRTC</b> video stream and feeds it into the existing camera
    /// pipeline. It is an alternate <i>transport</i> for a <c>"phone"</c> camera (not a new source
    /// type): SIPSorcery reassembles the RTP into Annex-B H.264 access units, the reused Sofia
    /// decoder (<see cref="LoopbackEsDecoder"/>, OpenCV's bundled FFmpeg — no external exe) turns
    /// them into JPEGs, and each JPEG is handed to <see cref="CameraDevice.PushFrame"/>, so snapshot,
    /// <c>/camera/{id}/ws</c>, vision and calibration all see it unchanged.
    ///
    /// <para>Threading mirrors <c>SofiaSession</c>: SIPSorcery's RTP thread writes access units into
    /// the decoder (<see cref="OnFrame"/>); a single owner thread opens the decoder and pumps
    /// decoded pictures to <see cref="CameraDevice.PushFrame"/>. One receiver per signaling socket;
    /// disposed when that socket closes or the peer connection fails.</para>
    /// </summary>
    public sealed class WebRtcCameraReceiver : IDisposable
    {
        private readonly CameraDevice _cam;
        private readonly string       _tag;
        private readonly RTCPeerConnection _pc;

        private SofiaDecoder? _decoder;
        private Thread?       _reader;
        private System.Threading.Timer? _keyframeTimer;
        private volatile bool _running = true;
        private long          _frames;
        private long          _rtpFrames;    // video access units received from the peer
        private long          _keyFrames;    // key frames received (to verify PLI cadence)
        private int           _localIce;
        private int           _remoteIce;

        // 90 (not 75): the decoded H.264 is re-encoded to JPEG for snapshot + vision/calibration,
        // so this is the round-trip quality ceiling — 75 stacked visible artifacts on top of H.264.
        private static readonly int[] JpegParams = { (int)ImwriteFlags.JpegQuality, 90 };

        // How often to ask the phone for a fresh key frame (PLI). The encoder otherwise uses a long GOP
        // (key frames almost never), so motion-prediction error accumulates and any dropped frame
        // smears until the next key frame. Periodic PLIs cap that and speed recovery. Keep it modest —
        // key frames are large, and this controller decodes single-threaded.
        private const int KeyframeRequestMs = 1000;

        // The live /ws preview gets a downscaled, lighter JPEG so it doesn't flood the network at full
        // 1080p (a full-res MJPEG round-trip is ~5x the phone's upload bitrate and lags over WiFi).
        // Snapshot + vision still read the full-resolution frame.
        private const int PreviewMaxEdge = 720;
        private static readonly int[] PreviewJpegParams = { (int)ImwriteFlags.JpegQuality, 80 };
        private const int OpenTimeoutMs = 10000;

        /// <summary>A local ICE candidate was gathered — forward it to the phone (JSON per the W3C shape).</summary>
        public event Action<string>? OnLocalIceCandidate;
        /// <summary>The peer connection state changed (connected / failed / closed …).</summary>
        public event Action<RTCPeerConnectionState>? OnConnectionStateChanged;

        public long FrameCount => Interlocked.Read(ref _frames);

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

            _pc.OnVideoFrameReceived += OnFrame;
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
                    _running = false;
            };
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

            Console.WriteLine($"[WebRTC] {_tag} offer applied, answer sent (answer has H264={answer.sdp?.Contains("H264") == true})");
            StartReader();
            // Periodically request a key frame so motion stays crisp and losses recover fast.
            _keyframeTimer = new System.Threading.Timer(_ => RequestKeyFrame(), null, KeyframeRequestMs, KeyframeRequestMs);
            return answer.sdp;
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

        // SIPSorcery hands a full Annex-B access unit per decoded video frame; feed it straight to
        // the decoder (same bytes the Sofia FFmpeg path expects). Runs on the RTP receive thread.
        private void OnFrame(IPEndPoint endpoint, uint timestamp, byte[] frame, VideoFormat format)
        {
            var dec = _decoder;
            if (dec == null || !_running || frame.Length == 0) return;
            var n = Interlocked.Increment(ref _rtpFrames);
            bool key = IsKeyFrame(frame);
            if (key) Interlocked.Increment(ref _keyFrames);
            if (n == 1 || n % 60 == 0) Console.WriteLine($"[WebRTC] {_tag} rx video frame #{n} ({frame.Length}B, key={key}, keyframes={Interlocked.Read(ref _keyFrames)})");
            try { dec.Write(frame, key); }
            catch { /* decoder gone away — the reader loop ends the session */ }
        }

        /// <summary>Ask the phone to emit a key frame now (RTCP PLI), once the remote video SSRC is known.</summary>
        private void RequestKeyFrame()
        {
            if (!_running) return;
            try
            {
                var vs = _pc.VideoStream;
                var remote = vs?.RemoteTrack?.Ssrc ?? 0;
                if (vs == null || remote == 0) return;   // not flowing yet
                var local = vs.LocalTrack?.Ssrc ?? 0;
                vs.SendRtcpFeedback(new RTCPFeedback(local, remote, PSFBFeedbackTypesEnum.PLI));
            }
            catch { /* connection not ready / torn down — the timer stops on Dispose */ }
        }

        private void StartReader()
        {
            // OpenCV/FFmpeg loopback decoder: no external ffmpeg binary, reuses the bundled plugin.
            _decoder = new LoopbackEsDecoder(_tag, NetworkCameraSource.DefaultReadTimeoutMs);
            _reader  = new Thread(ReaderLoop) { IsBackground = true, Name = $"WebRtc-{_tag}" };
            _reader.Start();
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
                    if (pic == null) break;
                    _cam.PushFrame(pic.ToJpeg(JpegParams), pic.Width, pic.Height, MakePreview(pic));
                    Interlocked.Increment(ref _frames);
                }
            }
            catch (Exception ex) { Console.WriteLine($"[WebRTC] {_tag} decode loop ended: {ex.Message}"); }
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
            _running = false;
            try { _keyframeTimer?.Dispose(); } catch { }
            try { _decoder?.Abort(); } catch { }
            try { if (_reader is { IsAlive: true } && _reader != Thread.CurrentThread) _reader.Join(1500); } catch { }
            try { _decoder?.Dispose(); } catch { }
            try { _pc.close(); } catch { }
            try { _pc.Dispose(); } catch { }
        }
    }
}
