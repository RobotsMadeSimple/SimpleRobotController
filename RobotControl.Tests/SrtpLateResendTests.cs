using System.Collections.Generic;
using Controller.RobotControl.Camera.WebRtc;
using SIPSorcery.Net.SharpSRTP.SRTP;

namespace RobotControl.Tests;

/// <summary>
/// The SRTP replay-window widening for late NACK resends: SIPSorcery rejects (-4) any packet more than
/// 64 behind the newest, which is exactly where a resend lands at 1080p. The fake "inner" unprotect
/// below applies SIPSorcery's real <see cref="SsrcSrtpContext"/> replay rules the way
/// <c>SrtpContext.UnprotectRtp</c> does (check, then update after "decrypting").
/// </summary>
public sealed class SrtpLateResendTests
{
    private const uint Ssrc = 0x11223344;

    private sealed class Rig
    {
        public readonly SsrcSrtpContext Ctx = new();
        public readonly SrtpLateResend Wrap;
        public readonly List<ushort> Decrypted = new();
        public Rig()
        {
            Wrap = new SrtpLateResend(Inner, s => s == Ssrc ? Ctx : null);
        }

        private int Inner(byte[] buf, int length, out int outLength)
        {
            ushort seq = (ushort)(buf[2] << 8 | buf[3]);
            Ctx.SetInitialSequence(seq);
            uint sl = Ctx.S_l;
            uint idx = SrtpContext.DetermineRtpIndex((ushort)(sl & 0xFFFF), seq, sl >> 16);
            if (!Ctx.CheckReplayWindow(idx)) { outLength = 0; return -4; }
            Ctx.UpdateReplayWindow(idx);
            Decrypted.Add(seq);
            outLength = length;
            return 0;
        }

        public int Receive(ushort seq)
        {
            var b = new byte[20];
            b[0] = 0x80; b[1] = 102;
            b[2] = (byte)(seq >> 8); b[3] = (byte)seq;
            b[8] = 0x11; b[9] = 0x22; b[10] = 0x33; b[11] = 0x44;
            return Wrap.Unprotect(b, b.Length, out _);
        }
    }

    [Fact]
    public void AResendFarBehindTheNewestPacketIsLetThrough()
    {
        var r = new Rig();
        for (ushort s = 1000; s < 1200; s++) if (s != 1010) Assert.Equal(0, r.Receive(s));
        // 1010 is 189 behind the newest: SIPSorcery alone would answer -4.
        Assert.Equal(0, r.Receive(1010));
        Assert.Equal(1, r.Wrap.Rescued);
        Assert.Equal(1199u, r.Ctx.S_l & 0xFFFF);           // the window is back where it was…
        Assert.Equal(0, r.Receive(1200));                   // …and the stream carries on normally
    }

    [Fact]
    public void ADuplicateIsStillRejected()
    {
        var r = new Rig();
        for (ushort s = 1000; s < 1200; s++) if (s != 1010) r.Receive(s);
        Assert.Equal(0, r.Receive(1010));
        Assert.Equal(-4, r.Receive(1010));                  // the same late packet twice
        Assert.Equal(-4, r.Receive(1190));                  // a duplicate inside the 64-packet window
        Assert.Equal(-4, r.Receive(1150));                  // an old packet that already came through
        Assert.Equal(1, r.Wrap.Rescued);
    }

    [Fact]
    public void FarTooOldIsRejected()
    {
        var r = new Rig();
        r.Receive(100);
        for (ushort s = 101; s < 100 + SrtpLateResend.MaxLatePackets + 100; s++) r.Receive(s);
        Assert.Equal(-4, r.Receive(100));
    }

    [Fact]
    public void WorksAcrossTheSequenceWrap()
    {
        var r = new Rig();
        for (int i = 65400; i < 65536 + 200; i++) if (i != 65500) r.Receive((ushort)i);
        Assert.Equal(0, r.Receive(65500));
        Assert.Equal(1, r.Wrap.Rescued);
        Assert.Equal(0, r.Receive(200));                    // past the wrap, still in step
    }

    [Fact]
    public void OtherFailuresPassStraightThrough()
    {
        var w = new SrtpLateResend((byte[] b, int l, out int o) => { o = 0; return -3; }, _ => new SsrcSrtpContext());
        var buf = new byte[20];
        Assert.Equal(-3, w.Unprotect(buf, buf.Length, out _));
        Assert.Equal(0, w.Rescued);
    }
}
