using System;
using System.Collections.Generic;
using System.IO;

namespace Controller.RobotControl.Camera.WebRtc
{
    /// <summary>
    /// Turns the phone's H.264 RTP packets (RFC 6184) into Annex-B access units, and only hands on
    /// pictures the decoder can decode cleanly.
    ///
    /// <para><b>Why.</b> SIPSorcery's own depacketiser glues together whatever packets arrive: a frame
    /// with a lost packet goes to the decoder with a hole in it, FFmpeg conceals it by copying from the
    /// previous picture, and every later P-frame paints on top of that damage — the smeared, blocky,
    /// "frozen" picture — while the stream still counts frames as arriving. This assembler tracks
    /// RTP sequence numbers instead:</para>
    /// <list type="bullet">
    /// <item>a missing packet is NACKed (the phone retransmits from its history) and waited for for up
    /// to <see cref="NackWaitMs"/>;</item>
    /// <item>if it never comes, the frame is dropped and so is everything after it until a
    /// <i>complete</i> key frame (SPS + IDR) arrives — the decoder holds the last clean picture
    /// instead of showing damage — and <see cref="KeyFrameNeeded"/> is raised so the receiver asks
    /// the phone for one (RTCP PLI).</item>
    /// </list>
    /// <para>Not thread-safe: the receiver calls it from SIPSorcery's RTP thread and its timer under one lock.
    /// Times are caller-supplied milliseconds (<see cref="Environment.TickCount64"/> in production).</para>
    /// </summary>
    public sealed class H264RtpAssembler
    {
        /// <summary>One access unit ready for the decoder.</summary>
        public readonly record struct Frame(byte[] AnnexB, bool IsKey, uint Timestamp);

        /// <summary>How long a missing packet is waited for (retransmission) before it counts as lost.</summary>
        public int NackWaitMs { get; init; } = 150;
        /// <summary>Re-NACK a still-missing packet after this long.</summary>
        public int NackRetryMs { get; init; } = 40;
        /// <summary>NACK a missing packet at most this many times.</summary>
        public int MaxNacksPerPacket { get; init; } = 3;
        /// <summary>A jump in sequence numbers larger than this is a stream reset, not loss.</summary>
        public int MaxGap { get; init; } = 1000;
        /// <summary>Never buffer more packets than this (a hole that pins the buffer is given up).</summary>
        public int MaxBufferedPackets { get; init; } = 4000;

        // ── Stats (monotonic counters; read by the receiver for the once-a-second report) ──
        public long PacketsReceived   { get; private set; }
        public long PacketsLost       { get; private set; }   // gave up on (never arrived in time)
        public long PacketsRecovered  { get; private set; }   // were missing, then arrived (retransmission / reorder)
        public long NacksSent         { get; private set; }   // packets NACKed (counting retries)
        public long FramesOut         { get; private set; }   // complete access units handed to the decoder
        public long KeyFramesOut      { get; private set; }
        public long FramesDropped     { get; private set; }   // incomplete, or skipped while waiting for a key frame
        public long KeyFramesNeeded   { get; private set; }   // times the picture broke and needed a key frame

        /// <summary>True until a complete key frame has been handed on, and again after any loss.</summary>
        public bool AwaitingKeyFrame { get; private set; } = true;

        /// <summary>Raised when the picture breaks (loss / reset) and a key frame is needed to resume.</summary>
        public event Action<string>? KeyFrameNeeded;

        private sealed class Pkt
        {
            public uint Ts;
            public bool Marker;
            public byte[] Payload = Array.Empty<byte>();   // empty = padding-only (occupies a sequence number)
        }
        private sealed class Miss { public long SinceMs; public long LastNackMs; public int Nacks; }

        private readonly Dictionary<long, Pkt>  _buf  = new();
        private readonly SortedDictionary<long, Miss> _miss = new();
        private bool _started;
        private long _highest;   // highest extended sequence number seen
        private long _next;      // next extended sequence number to consume (a frame boundary while synced)

        /// <summary>
        /// Adds one RTP packet. Complete frames are appended to <paramref name="frames"/>, sequence
        /// numbers to NACK to <paramref name="nacks"/> (both may be empty).
        /// </summary>
        public void Push(ushort seq, uint timestamp, bool marker, byte[] payload, bool paddingOnly, long nowMs,
                         List<Frame> frames, List<ushort> nacks)
        {
            PacketsReceived++;
            long ext;
            if (!_started)
            {
                _started = true;
                ext = 1L << 32 | seq;   // headroom so unwrapping never goes negative
                _highest = ext;
                _next = ext;
            }
            else
            {
                ext = _highest + (short)(ushort)(seq - (ushort)_highest);
                if (ext > _highest + MaxGap || ext < _next - MaxGap)
                {
                    Reset(ext, "the phone's packet sequence jumped");
                }
                else if (ext > _highest)
                {
                    for (long m = _highest + 1; m < ext; m++)
                        _miss[m] = new Miss { SinceMs = nowMs };
                    _highest = ext;
                }
            }

            if (ext < _next || _buf.ContainsKey(ext)) { Drain(nowMs, frames); CollectNacks(nowMs, nacks); return; }   // late or duplicate
            if (_miss.Remove(ext)) PacketsRecovered++;
            _buf[ext] = new Pkt { Ts = timestamp, Marker = marker, Payload = paddingOnly ? Array.Empty<byte>() : payload };

            Drain(nowMs, frames);
            CollectNacks(nowMs, nacks);
        }

        /// <summary>Re-runs the give-up timers without a new packet (call a few times a second).</summary>
        public void Poll(long nowMs, List<Frame> frames, List<ushort> nacks)
        {
            if (!_started) return;
            Drain(nowMs, frames);
            CollectNacks(nowMs, nacks);
        }

        /// <summary>Forces a resync on the next key frame (e.g. the decoder had to drop queued frames).</summary>
        public void RequireKeyFrame(string reason) => Break(reason);

        private void Reset(long ext, string reason)
        {
            _buf.Clear();
            _miss.Clear();
            _highest = ext;
            _next = ext;
            Break(reason);
        }

        /// <summary>The picture can't continue: wait for a key frame. Raises <see cref="KeyFrameNeeded"/> once per break.</summary>
        private void Break(string reason)
        {
            if (AwaitingKeyFrame) return;   // already waiting (the receiver keeps asking while it does)
            AwaitingKeyFrame = true;
            KeyFramesNeeded++;
            try { KeyFrameNeeded?.Invoke(reason); } catch { /* listener's problem */ }
        }

        private bool MissingTooLong(long seq, long nowMs) =>
            !_miss.TryGetValue(seq, out var m) || nowMs - m.SinceMs >= NackWaitMs;

        private void GiveUp(long seq)
        {
            if (_miss.Remove(seq)) PacketsLost++;
        }

        /// <summary>Drops buffered packets in [from, to) (counted once per frame timestamp).</summary>
        private void Discard(long from, long to)
        {
            for (long s = from; s < to; s++)
            {
                if (_buf.Remove(s, out var p)) { Consumed(s, p); if (p.Payload.Length > 0) CountDropped(p.Ts); }
                else GiveUp(s);
            }
        }

        private void Drain(long nowMs, List<Frame> frames)
        {
            // Too much buffered behind a hole: give the hole up.
            if (_buf.Count > MaxBufferedPackets && _miss.Count > 0)
            {
                long hole = FirstMissingAtOrAfter(_next);
                if (hole >= 0) { Discard(_next, hole + 1); _next = hole + 1; Break("too many packets were waiting on a lost one"); }
            }

            while (_next <= _highest)
            {
                if (!_buf.TryGetValue(_next, out var first))
                {
                    // Hole at a frame boundary.
                    if (AwaitingKeyFrame && HasKeyStartAfter(_next)) { GiveUp(_next); _next++; continue; }   // nothing we need
                    if (!MissingTooLong(_next, nowMs)) return;   // wait for the retransmission
                    GiveUp(_next);
                    _next++;
                    Break("a video packet was lost");
                    continue;
                }

                if (first.Payload.Length == 0) { Consumed(_next, first); _buf.Remove(_next); _next++; continue; }   // padding

                if (AwaitingKeyFrame && !IsKeyStart(_next, first))
                {
                    // Can't decode anything before a key frame: skip this packet (its frame counts once).
                    CountDropped(first.Ts);
                    Consumed(_next, first);
                    _buf.Remove(_next);
                    _next++;
                    continue;
                }

                // Collect a contiguous run with one timestamp, ending at the marker bit.
                long end = -1;         // exclusive end of a complete frame
                long hole = -1;        // first missing seq inside the frame
                for (long s = _next; ; s++)
                {
                    if (!_buf.TryGetValue(s, out var q))
                    {
                        if (s <= _highest) hole = s;   // a packet of this frame (or the next) is missing
                        break;                         // else: the rest hasn't arrived yet
                    }
                    if (q.Payload.Length == 0) continue;                 // padding inside? just skip it
                    if (q.Ts != first.Ts) { end = s; break; }            // next frame began: this one ended (marker lost/unset)
                    if (q.Marker) { end = s + 1; break; }
                }

                if (end < 0)
                {
                    if (hole < 0) return;                                 // still arriving
                    if (!MissingTooLong(hole, nowMs)) return;             // waiting on a retransmission
                    // The frame can't be completed: drop it, and resync on a key frame.
                    long upTo = hole + 1;
                    Discard(_next, upTo);
                    _next = upTo;
                    Break("a video frame was incomplete (lost packets)");
                    continue;
                }

                var au = Depacketise(_next, end, out bool idr, out bool sps);
                for (long s = _next; s < end; s++) { Consumed(s, _buf[s]); _buf.Remove(s); }
                _next = end;
                if (au == null) { CountDropped(first.Ts); Break("a video frame could not be unpacked"); continue; }
                if (sps) _haveSps = true;
                bool key = idr && (sps || _haveSps);
                if (AwaitingKeyFrame && !key) { CountDropped(first.Ts); continue; }   // e.g. SPS without an IDR
                if (key) { KeyFramesOut++; AwaitingKeyFrame = false; }
                FramesOut++;
                frames.Add(new Frame(au, key, first.Ts));
            }
        }

        private uint? _lastDroppedTs;

        /// <summary>Counts a dropped frame once, however many of its packets are dropped.</summary>
        private void CountDropped(uint ts)
        {
            if (_lastDroppedTs == ts) return;
            _lastDroppedTs = ts;
            FramesDropped++;
        }

        private long FirstMissingAtOrAfter(long seq)
        {
            foreach (var k in _miss.Keys) if (k >= seq) return k;
            return -1;
        }

        private bool HasKeyStartAfter(long seq)
        {
            for (long s = seq + 1; s <= _highest; s++)
                if (_buf.TryGetValue(s, out var p) && IsKeyStart(s, p)) return true;
            return false;
        }

        private long _lastSeq = -1;   // the last packet consumed (handed on, skipped or dropped)…
        private uint _lastTs;         // …and its timestamp, to tell where a frame begins
        private bool _haveSps;        // the decoder has been given parameter sets

        private void Consumed(long seq, Pkt p) { _lastSeq = seq; _lastTs = p.Ts; }

        /// <summary>
        /// Could this packet begin a key frame? libwebrtc sends SPS + PPS ahead of every IDR (single NALs
        /// or one STAP-A), so a key frame normally starts with an SPS — which also proves no earlier
        /// packet of the frame is missing. A frame led by an IDR / SEI / PPS / AUD instead counts only
        /// when the packet before it is known to belong to an earlier frame.
        /// </summary>
        private bool IsKeyStart(long seq, Pkt p)
        {
            var b = p.Payload;
            if (b.Length < 2) return false;
            int type = b[0] & 0x1F;
            int lead = type switch
            {
                24 => b.Length > 3 ? b[3] & 0x1F : 0,                        // STAP-A: first aggregated NAL
                28 => (b[1] & 0x80) != 0 ? b[1] & 0x1F : 0,                  // FU-A: only its start fragment
                _  => type,
            };
            if (lead == 7) return true;                                      // SPS
            if (lead is 5 or 6 or 8 or 9)
            {
                uint? prev = _buf.TryGetValue(seq - 1, out var q) ? q.Ts : _lastSeq == seq - 1 ? _lastTs : null;
                return prev.HasValue && prev.Value != p.Ts;
            }
            return false;
        }

        /// <summary>Builds the Annex-B access unit from packets [from, to). Null if malformed.</summary>
        private byte[]? Depacketise(long from, long to, out bool idr, out bool sps)
        {
            idr = sps = false;
            using var ms = new MemoryStream();
            bool inFu = false;
            for (long s = from; s < to; s++)
            {
                var b = _buf[s].Payload;
                if (b.Length == 0) continue;
                int type = b[0] & 0x1F;
                if (type >= 1 && type <= 23)
                {
                    if (inFu) return null;
                    Note(type, ref idr, ref sps);
                    StartCode(ms); ms.Write(b, 0, b.Length);
                }
                else if (type == 24)                                     // STAP-A
                {
                    if (inFu) return null;
                    int i = 1;
                    while (i + 2 <= b.Length)
                    {
                        int len = b[i] << 8 | b[i + 1];
                        i += 2;
                        if (len == 0 || i + len > b.Length) return null;
                        Note(b[i] & 0x1F, ref idr, ref sps);
                        StartCode(ms); ms.Write(b, i, len);
                        i += len;
                    }
                }
                else if (type == 28)                                     // FU-A
                {
                    if (b.Length < 3) return null;
                    bool start = (b[1] & 0x80) != 0, endBit = (b[1] & 0x40) != 0;
                    int nalType = b[1] & 0x1F;
                    if (start)
                    {
                        if (inFu) return null;
                        Note(nalType, ref idr, ref sps);
                        StartCode(ms);
                        ms.WriteByte((byte)(b[0] & 0xE0 | nalType));
                        inFu = true;
                    }
                    else if (!inFu) return null;                         // middle fragment without its start
                    ms.Write(b, 2, b.Length - 2);
                    if (endBit) inFu = false;
                }
                else return null;                                        // STAP-B / MTAP / FU-B: not used by WebRTC
            }
            if (inFu) return null;                                       // last fragment missing
            return ms.Length > 0 ? ms.ToArray() : null;
        }

        private static void Note(int nalType, ref bool idr, ref bool sps)
        {
            if (nalType == 5) idr = true;
            else if (nalType == 7) sps = true;
        }

        private static void StartCode(MemoryStream ms) { ms.WriteByte(0); ms.WriteByte(0); ms.WriteByte(0); ms.WriteByte(1); }

        private void CollectNacks(long nowMs, List<ushort> nacks)
        {
            if (_miss.Count == 0) return;
            List<long>? stale = null;
            foreach (var (seq, m) in _miss)
            {
                if (seq < _next) { (stale ??= new()).Add(seq); continue; }
                if (m.Nacks >= MaxNacksPerPacket) continue;
                if (m.Nacks > 0 && nowMs - m.LastNackMs < NackRetryMs) continue;
                m.Nacks++;
                m.LastNackMs = nowMs;
                NacksSent++;
                nacks.Add((ushort)seq);
                if (nacks.Count >= 256) break;
            }
            if (stale != null) foreach (var s in stale) GiveUp(s);
        }

        /// <summary>Packs sequence numbers into RFC 4585 generic-NACK (PID, BLP) pairs.</summary>
        public static List<(ushort Pid, ushort Blp)> PackNacks(IReadOnlyList<ushort> seqs)
        {
            var result = new List<(ushort, ushort)>();
            if (seqs.Count == 0) return result;
            var sorted = new List<ushort>(seqs);
            // Sort in wrap-aware order relative to the first one.
            ushort basis = sorted[0];
            sorted.Sort((a, b) => ((ushort)(a - basis)).CompareTo((ushort)(b - basis)));
            int i = 0;
            while (i < sorted.Count)
            {
                ushort pid = sorted[i];
                ushort blp = 0;
                int j = i + 1;
                while (j < sorted.Count)
                {
                    int d = (ushort)(sorted[j] - pid);
                    if (d == 0) { j++; continue; }
                    if (d > 16) break;
                    blp |= (ushort)(1 << (d - 1));
                    j++;
                }
                result.Add((pid, blp));
                i = j;
            }
            return result;
        }
    }
}
