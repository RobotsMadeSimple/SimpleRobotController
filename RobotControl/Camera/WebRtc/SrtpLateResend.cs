using SIPSorcery.Net;
using SIPSorcery.Net.SharpSRTP.SRTP;
using System;
using System.Collections.Generic;
using System.Reflection;
using System.Threading;

namespace Controller.RobotControl.Camera.WebRtc
{
    /// <summary>
    /// Lets a <b>late NACK resend</b> through SIPSorcery's SRTP replay check.
    ///
    /// <para><b>Why.</b> SIPSorcery's SRTP receiver (<see cref="SsrcSrtpContext"/>) has a fixed
    /// 64-packet replay window: a packet whose index is more than 64 behind the newest one is rejected
    /// (<c>UnprotectRtp</c> returns -4) even though it authenticates. A phone's resend of a lost packet
    /// comes back one round trip later — and at 1080p, 8–16 Mbps, a key frame is ~150 packets leaving
    /// in one paced burst, so by the time the resend arrives 64+ newer packets are already in. Every
    /// such resend was thrown away before the H.264 assembler saw it: the NACK "worked", the phone
    /// resent, and the frame still counted as lost, so the picture froze waiting for a key frame that
    /// itself usually lost a packet the same way. libwebrtc's own receiver uses a much wider window
    /// for exactly this reason.</para>
    ///
    /// <para><b>How.</b> The video stream's unprotect delegate is wrapped. When the original rejects a
    /// packet as a replay, and the packet is older than the window (not a duplicate inside it) but
    /// still within <see cref="MaxLatePackets"/>, and it hasn't been let through before, the replay
    /// state is moved back just for that packet, the original unprotect runs again (so it is still
    /// fully authenticated and decrypted by SIPSorcery), and the state is put back. Duplicates are
    /// still rejected. Everything else behaves exactly as before.</para>
    /// </summary>
    internal sealed class SrtpLateResend
    {
        /// <summary>How far behind the newest packet a resend may be and still be let through.</summary>
        public const uint MaxLatePackets = 8192;
        private const int SipsorceryReplayRejected = -4;
        private const int RememberLate = 2048;

        private static readonly PropertyInfo? BitmapProp = typeof(SsrcSrtpContext).GetProperty(nameof(SsrcSrtpContext.Bitmap));
        private static readonly PropertyInfo? UnprotectProp = typeof(SecureContext).GetProperty(nameof(SecureContext.UnprotectRtpPacket));

        private readonly ProtectRtpPacket _inner;
        private readonly Func<uint, SsrcSrtpContext?> _replayFor;
        private readonly object _lock;
        private readonly HashSet<ulong> _late = new();       // (ssrc, index) let through already
        private readonly Queue<ulong> _lateOrder = new();
        private long _rescued;

        /// <summary>Packets let through that the 64-packet window would have dropped.</summary>
        public long Rescued => Interlocked.Read(ref _rescued);

        internal SrtpLateResend(ProtectRtpPacket inner, Func<uint, SsrcSrtpContext?> replayFor, object? sync = null)
        {
            _inner = inner;
            _replayFor = replayFor;
            _lock = sync ?? new object();
        }

        /// <summary>
        /// Wraps <paramref name="stream"/>'s SRTP unprotect once its DTLS handshake has installed one.
        /// Returns null while there is nothing to wrap yet (call again later), or if this SIPSorcery
        /// build is shaped differently (then the stream keeps its own behaviour).
        /// </summary>
        public static SrtpLateResend? TryInstall(MediaStream stream, out bool unsupported)
        {
            unsupported = false;
            var sc = stream.GetSecurityContext();
            if (sc?.UnprotectRtpPacket == null) return null;
            try
            {
                var inner = sc.UnprotectRtpPacket;
                if (inner.Target is SrtpLateResend) { unsupported = true; return null; }   // already wrapped
                var (decode, sync) = FindDecodeContext(inner.Target);
                if (decode == null || BitmapProp?.SetMethod == null || UnprotectProp?.SetMethod == null) { unsupported = true; return null; }
                var w = new SrtpLateResend(inner, ssrc => decode.ReplayProtection.TryGetValue(ssrc, out var c) ? c : null, sync);
                UnprotectProp.SetValue(sc, (ProtectRtpPacket)w.Unprotect);
                return w;
            }
            catch { unsupported = true; return null; }
        }

        /// <summary>DtlsSrtpTransport / SrtpHandler → ThreadSafeSrtpSessionContext → SrtpSessionContext → DecodeRtpContext.</summary>
        private static (SrtpContext? decode, object? sync) FindDecodeContext(object? transport)
        {
            if (transport == null) return (null, null);
            var ctx = transport.GetType().GetProperty("Context", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic)?.GetValue(transport);
            object? sync = null;
            if (ctx is ThreadSafeSrtpSessionContext ts)
            {
                sync = typeof(ThreadSafeSrtpSessionContext).GetField("_syncDecodeRtpContext", BindingFlags.Instance | BindingFlags.NonPublic)?.GetValue(ts);
                ctx = typeof(ThreadSafeSrtpSessionContext).GetField("_srtpSessionContext", BindingFlags.Instance | BindingFlags.NonPublic)?.GetValue(ts);
            }
            return ((ctx as SrtpSessionContext)?.DecodeRtpContext, sync);
        }

        /// <summary>The wrapped unprotect (SIPSorcery's RTP receive thread).</summary>
        internal int Unprotect(byte[] buf, int length, out int outLength)
        {
            int r = _inner(buf, length, out outLength);
            if (r != SipsorceryReplayRejected || length < 12) return r;

            uint ssrc = (uint)(buf[8] << 24 | buf[9] << 16 | buf[10] << 8 | buf[11]);
            ushort seq = (ushort)(buf[2] << 8 | buf[3]);
            lock (_lock)
            {
                var ctx = _replayFor(ssrc);
                if (ctx == null || !ctx.S_l_set) return r;
                uint sl = ctx.S_l;
                uint idx = SrtpContext.DetermineRtpIndex((ushort)(sl & 0xFFFF), seq, sl >> 16);
                if (idx >= sl || sl - idx < SsrcSrtpContext.REPLAY_WINDOW_SIZE || sl - idx > MaxLatePackets || idx == 0)
                    return r;                                   // a duplicate inside the window, or far too old
                ulong key = (ulong)ssrc << 32 | idx;
                if (_late.Contains(key)) return r;              // this late packet already came through once

                ulong bitmap = ctx.Bitmap;
                try
                {
                    ctx.SetSequence(idx - 1);                   // the packet is now "newest" — the check passes
                    BitmapProp!.SetValue(ctx, 0UL);
                    r = _inner(buf, length, out outLength);     // SIPSorcery authenticates + decrypts it as usual
                }
                finally
                {
                    ctx.SetSequence(sl);                        // put the window back exactly as it was
                    BitmapProp!.SetValue(ctx, bitmap);
                }
                if (r == 0)
                {
                    _late.Add(key);
                    _lateOrder.Enqueue(key);
                    if (_lateOrder.Count > RememberLate) _late.Remove(_lateOrder.Dequeue());
                    Interlocked.Increment(ref _rescued);
                }
                return r;
            }
        }
    }
}
