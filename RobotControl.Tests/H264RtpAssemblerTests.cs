using System.Collections.Generic;
using System.Linq;
using Controller.RobotControl.Camera.WebRtc;

namespace RobotControl.Tests;

/// <summary>
/// The phone stream's RTP → H.264 assembler: complete frames only, NACK on a hole, hold back
/// everything after an unrecoverable loss until a complete key frame, and say when one is needed.
/// </summary>
public sealed class H264RtpAssemblerTests
{
    // ── Packet builders (RFC 6184) ──────────────────────────────────────────
    private static byte[] Nal(int type, int len = 6) =>
        new[] { (byte)(0x60 | type) }.Concat(Enumerable.Range(1, len - 1).Select(i => (byte)(i + type))).ToArray();

    private static byte[] StapA(params byte[][] nals)
    {
        var b = new List<byte> { 24 };
        foreach (var n in nals) { b.Add((byte)(n.Length >> 8)); b.Add((byte)n.Length); b.AddRange(n); }
        return b.ToArray();
    }

    /// <summary>Splits one NAL into FU-A fragments of <paramref name="chunk"/> payload bytes.</summary>
    private static List<byte[]> FuA(byte[] nal, int chunk)
    {
        var res = new List<byte[]>();
        int type = nal[0] & 0x1F;
        var body = nal.Skip(1).ToArray();
        for (int off = 0; off < body.Length; off += chunk)
        {
            int n = System.Math.Min(chunk, body.Length - off);
            byte hdr = (byte)((off == 0 ? 0x80 : 0) | (off + n >= body.Length ? 0x40 : 0) | type);
            res.Add(new[] { (byte)(nal[0] & 0xE0 | 28), hdr }.Concat(body.Skip(off).Take(n)).ToArray());
        }
        return res;
    }

    /// <summary>A key frame as libwebrtc sends it: STAP-A(SPS, PPS) then the IDR in FU-A fragments.</summary>
    private static List<byte[]> KeyFrame(int idrLen = 40, int chunk = 10) =>
        new List<byte[]> { StapA(Nal(7), Nal(8)) }.Concat(FuA(Nal(5, idrLen), chunk)).ToList();

    private static List<byte[]> PFrame(int len = 30, int chunk = 10) => FuA(Nal(1, len), chunk);

    private sealed class Rig
    {
        public readonly H264RtpAssembler A = new();
        public readonly List<H264RtpAssembler.Frame> Frames = new();
        public readonly List<ushort> Nacks = new();
        public readonly List<string> Breaks = new();
        public ushort Seq;
        public uint Ts = 90000;
        public long Now = 1000;
        public Rig(ushort startSeq = 100) { Seq = startSeq; A.KeyFrameNeeded += r => Breaks.Add(r); }

        /// <summary>Sends one frame's packets, skipping the indexes in <paramref name="drop"/> (their seq is still used).</summary>
        public List<ushort> Send(List<byte[]> pkts, params int[] drop)
        {
            var seqs = new List<ushort>();
            for (int i = 0; i < pkts.Count; i++)
            {
                var s = Seq++;
                seqs.Add(s);
                if (drop.Contains(i)) continue;
                A.Push(s, Ts, i == pkts.Count - 1, pkts[i], false, Now, Frames, Nacks);
            }
            Ts += 6000;
            return seqs;
        }

        public void Push(ushort seq, uint ts, bool marker, byte[] p) => A.Push(seq, ts, marker, p, false, Now, Frames, Nacks);
        public void Advance(long ms) { Now += ms; A.Poll(Now, Frames, Nacks); }
    }

    [Fact]
    public void CompleteKeyFrameThenPFramesComeOutInOrder()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        r.Send(PFrame());
        r.Send(PFrame());
        Assert.Equal(3, r.Frames.Count);
        Assert.True(r.Frames[0].IsKey);
        Assert.False(r.Frames[1].IsKey);
        Assert.False(r.A.AwaitingKeyFrame);
        Assert.Empty(r.Nacks);
        Assert.Equal(0, r.A.FramesDropped);
    }

    [Fact]
    public void ReassembledKeyFrameIsValidAnnexB()
    {
        var r = new Rig();
        r.Send(KeyFrame(idrLen: 25, chunk: 7));
        var au = r.Frames.Single().AnnexB;
        Assert.True(WebRtcCameraReceiver.IsKeyFrame(au));
        // SPS, PPS and IDR each behind a 4-byte start code, IDR rebuilt byte-for-byte.
        var idr = Nal(5, 25);
        Assert.Equal(idr, au.Skip(au.Length - idr.Length).ToArray());
        Assert.Equal(new byte[] { 0, 0, 0, 1 }, au.Take(4).ToArray());
    }

    [Fact]
    public void NothingIsDecodedBeforeTheFirstKeyFrame()
    {
        var r = new Rig();
        r.Send(PFrame());   // joined mid-stream
        r.Send(PFrame());
        Assert.Empty(r.Frames);
        Assert.True(r.A.AwaitingKeyFrame);
        Assert.Equal(2, r.A.FramesDropped);
        r.Send(KeyFrame());
        Assert.Single(r.Frames);
        Assert.True(r.Frames[0].IsKey);
    }

    [Fact]
    public void AMissingPacketIsNackedAndARetransmissionCompletesTheFrame()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        var p = PFrame();
        var seqs = r.Send(p, 1);                 // middle fragment lost
        Assert.Single(r.Frames);                 // the P-frame waits
        Assert.Contains(seqs[1], r.Nacks);
        r.Push(seqs[1], r.Ts - 6000, false, p[1]);   // retransmission
        Assert.Equal(2, r.Frames.Count);
        Assert.Equal(1, r.A.PacketsRecovered);
        Assert.Equal(0, r.A.PacketsLost);
        Assert.Empty(r.Breaks);
    }

    [Fact]
    public void ALostPacketDropsTheFrameAndHoldsEverythingUntilACompleteKeyFrame()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        r.Send(PFrame(), 1);                     // lost for good
        r.Send(PFrame());
        r.Send(PFrame());
        Assert.Single(r.Frames);                 // all held behind the hole
        r.Advance(r.A.NackWaitMs + 1);           // give up on the hole
        Assert.Single(r.Frames);                 // P-frames after the loss are not decodable either
        Assert.True(r.A.AwaitingKeyFrame);
        Assert.Single(r.Breaks);
        Assert.Equal(1, r.A.PacketsLost);
        Assert.Equal(3, r.A.FramesDropped);
        r.Send(PFrame());
        Assert.Single(r.Frames);
        r.Send(KeyFrame());
        Assert.Equal(2, r.Frames.Count);
        Assert.True(r.Frames[1].IsKey);
        Assert.False(r.A.AwaitingKeyFrame);
        r.Send(PFrame());
        Assert.Equal(3, r.Frames.Count);
    }

    [Fact]
    public void ALostMarkerPacketIsCaughtByTheTimestampChange()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        var p = PFrame();
        r.Send(p, p.Count - 1);                  // last fragment (with the marker) lost
        r.Send(PFrame());
        r.Advance(r.A.NackWaitMs + 1);
        Assert.Single(r.Frames);
        Assert.True(r.A.AwaitingKeyFrame);
    }

    [Fact]
    public void AKeyFrameWithALostPacketIsNotUsed()
    {
        var r = new Rig();
        r.Send(KeyFrame(), 2);                   // a fragment of the IDR lost
        r.Advance(r.A.NackWaitMs + 1);
        r.Send(PFrame());
        Assert.Empty(r.Frames);
        Assert.True(r.A.AwaitingKeyFrame);
        r.Send(KeyFrame());
        Assert.Single(r.Frames);
    }

    [Fact]
    public void AKeyFrameMissingItsParameterSetsIsNotUsed()
    {
        var r = new Rig();
        r.Send(KeyFrame(), 0);                   // the STAP-A with SPS/PPS lost
        r.Advance(r.A.NackWaitMs + 1);
        Assert.Empty(r.Frames);
        Assert.True(r.A.AwaitingKeyFrame);
    }

    [Fact]
    public void AnIdrLedKeyFrameResyncsWhenItsStartIsProvable()
    {
        var r = new Rig();
        r.Send(KeyFrame());                      // parameter sets known from here
        r.Send(PFrame(), 1);
        r.Advance(r.A.NackWaitMs + 1);
        Assert.True(r.A.AwaitingKeyFrame);
        r.Send(PFrame());                        // skipped, but marks where the next frame begins
        r.Send(FuA(Nal(5, 40), 10));             // IDR without in-band SPS/PPS
        Assert.Equal(2, r.Frames.Count);
        Assert.True(r.Frames[1].IsKey);
        Assert.False(r.A.AwaitingKeyFrame);
    }

    [Fact]
    public void AnIdrFragmentAfterAHoleIsNotTrustedAsAFrameStart()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        var idr = FuA(Nal(5, 40), 10);
        r.Send(new List<byte[]> { Nal(1) }.Concat(idr).ToList(), 0);   // packet before the IDR lost: can't prove the start
        r.Advance(r.A.NackWaitMs + 1);
        Assert.Single(r.Frames);
    }

    [Fact]
    public void SequenceNumbersWrapAround()
    {
        var r = new Rig(startSeq: 65530);
        r.Send(KeyFrame());                      // crosses 65535 → 0
        r.Send(PFrame());
        r.Send(PFrame());
        Assert.Equal(3, r.Frames.Count);
        Assert.Empty(r.Nacks);
    }

    [Fact]
    public void ReorderedPacketsInsideAFrameAreFine()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        var p = PFrame();
        ushort s0 = r.Seq; uint ts = r.Ts;
        r.Push((ushort)(s0 + 1), ts, false, p[1]);
        r.Push(s0, ts, false, p[0]);
        r.Push((ushort)(s0 + 2), ts, true, p[2]);
        Assert.Equal(2, r.Frames.Count);
        Assert.Equal(0, r.A.PacketsLost);
    }

    [Fact]
    public void PaddingPacketsOccupySequenceNumbersWithoutBreakingThePicture()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        r.A.Push(r.Seq++, r.Ts - 6000, false, new byte[] { 0, 0, 0, 4 }, true, r.Now, r.Frames, r.Nacks);   // padding-only
        r.Send(PFrame());
        Assert.Equal(2, r.Frames.Count);
        Assert.False(r.A.AwaitingKeyFrame);
    }

    [Fact]
    public void AHugeSequenceJumpResetsAndWaitsForAKeyFrame()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        r.Seq += 5000;
        r.Send(PFrame());
        Assert.Single(r.Frames);
        Assert.True(r.A.AwaitingKeyFrame);
        Assert.Single(r.Breaks);
        r.Send(KeyFrame());
        Assert.Equal(2, r.Frames.Count);
    }

    [Fact]
    public void RequireKeyFrameForcesAResync()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        r.A.RequireKeyFrame("decoder overflow");
        r.Send(PFrame());
        Assert.Single(r.Frames);
        Assert.Single(r.Breaks);
        r.Send(KeyFrame());
        Assert.Equal(2, r.Frames.Count);
    }

    [Fact]
    public void NacksAreRetriedThenStop()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        var seqs = r.Send(PFrame(), 1);
        r.Send(PFrame());
        int first = r.Nacks.Count(s => s == seqs[1]);
        Assert.Equal(1, first);
        for (int i = 0; i < r.A.MaxNacksPerPacket + 2; i++) r.Advance(r.A.RetryAfterMs);
        Assert.Equal(r.A.MaxNacksPerPacket, r.Nacks.Count(s => s == seqs[1]));
    }

    [Fact]
    public void AResendThatTakesARoundTripStillCompletesTheFrame()
    {
        // Over Wi-Fi a resend comes back a round trip plus the phone's pacer queue later — 150 ms
        // (the old give-up) was too short; anything inside GiveUpAfterMs must still count.
        var r = new Rig();
        r.Send(KeyFrame());
        var pkts = PFrame();
        var seqs = r.Send(pkts, 1);
        r.Send(PFrame());
        r.Advance(200);
        Assert.Single(r.Frames);
        Assert.Empty(r.Breaks);
        r.Push(seqs[1], r.Ts - 12000, false, pkts[1]);   // the resend
        Assert.Equal(3, r.Frames.Count);
        Assert.Empty(r.Breaks);
        Assert.Equal(1, r.A.PacketsRecovered);
        Assert.Equal(1, r.A.PacketsMissing);
    }

    [Fact]
    public void SlowResendsStretchTheWaitAndTheRetryGap()
    {
        var r = new Rig();
        r.Send(KeyFrame());
        Assert.Equal(r.A.NackWaitMs, r.A.GiveUpAfterMs);   // nothing measured yet: the floor
        Assert.Equal(r.A.NackRetryMs, r.A.RetryAfterMs);
        for (int i = 0; i < 20; i++)
        {
            var pkts = PFrame();
            var seqs = r.Send(pkts, 1);
            r.Send(PFrame());
            r.Now += 150;                                     // each resend takes 150 ms (one NACK each)
            r.Push(seqs[1], r.Ts - 12000, false, pkts[1]);
        }
        Assert.InRange(r.A.ResendRttMs, 120, 151);
        Assert.True(r.A.GiveUpAfterMs > r.A.NackWaitMs);
        Assert.True(r.A.GiveUpAfterMs <= r.A.MaxNackWaitMs);
        Assert.True(r.A.RetryAfterMs > r.A.NackRetryMs);
        Assert.Empty(r.Breaks);
    }

    [Fact]
    public void NacksPackIntoPidAndBitmask()
    {
        var packed = H264RtpAssembler.PackNacks(new ushort[] { 10, 11, 13, 26, 27, 40 });
        // 26 is 16 after 10 → bit 15; 27 is 17 after → its own entry, and 40 is 13 after 27 → bit 12.
        Assert.Equal(new (ushort, ushort)[] { (10, 0b1000000000000101), (27, 1 << 12) }, packed);
        var wrap = H264RtpAssembler.PackNacks(new ushort[] { 65535, 0, 1 });
        Assert.Equal(new (ushort, ushort)[] { (65535, 0b11) }, wrap);
    }
}
