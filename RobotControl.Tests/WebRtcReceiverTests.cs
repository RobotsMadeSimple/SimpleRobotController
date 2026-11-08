using Controller.RobotControl.Camera.WebRtc;

namespace RobotControl.Tests;

/// <summary>
/// WebRTC phone camera receiver. The full peer-connection negotiation + H.264 decode is validated
/// on-device (it needs real ICE/DTLS and the OpenCV FFmpeg plugin); here we pin the one piece of
/// custom parsing — Annex-B key-frame detection — which drives the decoder's prefeed/buffering.
/// </summary>
public sealed class WebRtcReceiverTests
{
    // NAL header byte: type = b & 0x1F. 5 = IDR, 7 = SPS, 8 = PPS, 1 = non-IDR (P/B) slice.
    private static byte Nal(int type) => (byte)(type & 0x1F);

    [Fact]
    public void IdrAccessUnitIsAKeyFrame()
    {
        var au = new byte[] { 0, 0, 0, 1, Nal(5), 0xAA, 0xBB };   // 4-byte start code + IDR
        Assert.True(WebRtcCameraReceiver.IsKeyFrame(au));
    }

    [Fact]
    public void SpsMakesItAKeyFrame()
    {
        var au = new byte[] { 0, 0, 1, Nal(7), 0x42, 0x00 };      // 3-byte start code + SPS
        Assert.True(WebRtcCameraReceiver.IsKeyFrame(au));
    }

    [Fact]
    public void SpsPpsIdrSequenceIsAKeyFrame()
    {
        var au = new byte[]
        {
            0, 0, 0, 1, Nal(7), 0x42,    // SPS
            0, 0, 0, 1, Nal(8), 0xCE,    // PPS
            0, 0, 0, 1, Nal(5), 0x88,    // IDR
        };
        Assert.True(WebRtcCameraReceiver.IsKeyFrame(au));
    }

    [Fact]
    public void NonIdrSliceIsNotAKeyFrame()
    {
        var au = new byte[] { 0, 0, 0, 1, Nal(1), 0x9A, 0x00, 0, 0, 1, Nal(1), 0x12 };   // only P-slices
        Assert.False(WebRtcCameraReceiver.IsKeyFrame(au));
    }

    [Fact]
    public void GarbageAndEmptyAreNotKeyFrames()
    {
        Assert.False(WebRtcCameraReceiver.IsKeyFrame(System.Array.Empty<byte>()));
        Assert.False(WebRtcCameraReceiver.IsKeyFrame(new byte[] { 0xFF, 0x00, 0x13, 0x37 }));   // no start code
    }

    // ── Stall watchdog ──────────────────────────────────────────────────────

    private const long T0 = 1_000_000;

    [Fact]
    public void WatchdogIsQuietBeforeNegotiationAndWhileFramesFlow()
    {
        Assert.Null(WebRtcCameraReceiver.StallReason(T0 + 60_000, 0, 0, 0));          // no answer yet
        Assert.Null(WebRtcCameraReceiver.StallReason(T0 + 5_000, T0, 0, 0));          // inside first-picture grace
        Assert.Null(WebRtcCameraReceiver.StallReason(T0 + 30_000, T0, T0 + 29_900, T0 + 29_950)); // streaming
    }

    [Fact]
    public void WatchdogEndsAStreamThatNeverStarted()
    {
        var now = T0 + WebRtcCameraReceiver.FirstPictureMs + 1;
        Assert.Contains("no video arrived", WebRtcCameraReceiver.StallReason(now, T0, 0, 0));
        Assert.Contains("no picture was decoded", WebRtcCameraReceiver.StallReason(now, T0, now - 100, 0));
    }

    [Fact]
    public void WatchdogEndsAStreamWhosePhoneStoppedSending()
    {
        // The phone's camera was taken away: no video and no pictures for longer than the stall limit,
        // while the peer connection itself still looks connected.
        long last = T0 + 10_000;
        var now = last + WebRtcCameraReceiver.StallMs + 1;
        Assert.Contains("no video from the phone", WebRtcCameraReceiver.StallReason(now, T0, last, last));
        Assert.Null(WebRtcCameraReceiver.StallReason(last + WebRtcCameraReceiver.StallMs - 1, T0, last, last));
    }

    [Fact]
    public void WatchdogEndsAStuckDecoder()
    {
        long lastPic = T0 + 10_000;
        var now = lastPic + WebRtcCameraReceiver.StallMs + 1;
        Assert.Contains("decoder", WebRtcCameraReceiver.StallReason(now, T0, now - 50, lastPic));
    }

    [Fact]
    public void WatchdogNamesLostPacketsWhenWaitingForAKeyFrame()
    {
        long lastPic = T0 + 10_000;
        var now = lastPic + WebRtcCameraReceiver.StallMs + 1;
        var r = WebRtcCameraReceiver.StallReason(now, T0, now - 50, lastPic, awaitingKeyFrame: true);
        Assert.Contains("no clean picture", r);
        Assert.Contains("key frame", r);
    }

    // ── Key-frame requests (PLI) ────────────────────────────────────────────

    [Fact]
    public void PliGoesOutAtOnceThenBacksOffWhileWaiting()
    {
        Assert.True(WebRtcCameraReceiver.PliDue(T0, 0, awaitingKeyFrame: true, attempt: 0, videoFlowing: true));
        Assert.False(WebRtcCameraReceiver.PliDue(T0 + 200, T0, true, 0, true));
        Assert.True(WebRtcCameraReceiver.PliDue(T0 + WebRtcCameraReceiver.PliRetryBaseMs, T0, true, 0, true));
        Assert.False(WebRtcCameraReceiver.PliDue(T0 + 400, T0, true, 1, true));      // 2nd retry waits 500
        Assert.True(WebRtcCameraReceiver.PliDue(T0 + 500, T0, true, 1, true));
        Assert.True(WebRtcCameraReceiver.PliDue(T0 + WebRtcCameraReceiver.PliRetryMaxMs, T0, true, 30, true));   // capped
        Assert.False(WebRtcCameraReceiver.PliDue(T0 + WebRtcCameraReceiver.PliRetryMaxMs - 1, T0, true, 30, true));
    }

    [Fact]
    public void HealthyStreamOnlyGetsTheSlowSafetyRefresh()
    {
        Assert.False(WebRtcCameraReceiver.PliDue(T0 + 1000, T0, awaitingKeyFrame: false, attempt: 0, videoFlowing: true));
        Assert.True(WebRtcCameraReceiver.PliDue(T0 + WebRtcCameraReceiver.KeyframeRefreshMs, T0, false, 0, true));
        Assert.False(WebRtcCameraReceiver.PliDue(T0 + 60_000, T0, true, 0, videoFlowing: false));   // nobody to ask
    }

    // ── Answer SDP: tell the phone we NACK and PLI ──────────────────────────

    [Fact]
    public void AnswerGetsNackAndPliFeedbackForH264()
    {
        var sdp = string.Join("\r\n",
            "v=0", "o=- 1 1 IN IP4 127.0.0.1", "s=-", "t=0 0",
            "m=audio 9 UDP/TLS/RTP/SAVPF 111", "a=rtpmap:111 opus/48000/2",
            "m=video 9 UDP/TLS/RTP/SAVPF 102", "c=IN IP4 0.0.0.0", "a=rtpmap:102 H264/90000",
            "a=fmtp:102 packetization-mode=1", "a=recvonly", "");
        var outp = WebRtcCameraReceiver.WithVideoFeedback(sdp);
        Assert.Contains("a=rtpmap:102 H264/90000\r\na=rtcp-fb:102 nack\r\na=rtcp-fb:102 nack pli\r\n", outp);
        Assert.DoesNotContain("rtcp-fb:111", outp);
        // Idempotent: nothing is added twice.
        Assert.Equal(outp, WebRtcCameraReceiver.WithVideoFeedback(outp));
    }

    [Fact]
    public void AnswerKeepsFeedbackThatIsAlreadyThere()
    {
        var sdp = "m=video 9 RTP/AVP 102\na=rtpmap:102 H264/90000\na=rtcp-fb:102 nack pli\n";
        var outp = WebRtcCameraReceiver.WithVideoFeedback(sdp);
        Assert.Single(outp.Split('\n'), l => l == "a=rtcp-fb:102 nack pli");
        Assert.Contains("a=rtcp-fb:102 nack\n", outp);
    }

    [Fact]
    public void StatsJsonIsParseableAndHonest()
    {
        var json = WebRtcCameraReceiver.StatsJson(15.04, 0, 12, 3, 40, 5, 2, true, 2500, "a video packet was lost");
        using var doc = System.Text.Json.JsonDocument.Parse(json);
        var r = doc.RootElement;
        Assert.Equal("stats", r.GetProperty("type").GetString());
        Assert.Equal(15.0, r.GetProperty("rxFps").GetDouble());
        Assert.Equal(0, r.GetProperty("decodedFps").GetDouble());
        Assert.Equal(12, r.GetProperty("dropped").GetInt64());
        Assert.True(r.GetProperty("waitingForKeyFrame").GetBoolean());
        Assert.Equal("a video packet was lost", r.GetProperty("reason").GetString());
        Assert.False(r.TryGetProperty("capKbps", out _));   // no cap: the phone keeps its own setting
    }

    [Fact]
    public void StatsJsonCarriesTheBitrateCap()
    {
        var json = WebRtcCameraReceiver.StatsJson(15, 15, 0, 0, 40, 1, 1, false, 30, null,
                                                  rxKbps: 6012.4, capKbps: 6000, capLowered: 2, resendRttMs: 57.6);
        using var doc = System.Text.Json.JsonDocument.Parse(json);
        var r = doc.RootElement;
        Assert.Equal(6000, r.GetProperty("capKbps").GetInt32());
        Assert.Equal(2, r.GetProperty("capLowered").GetInt32());
        Assert.Equal(6012, r.GetProperty("rxKbps").GetDouble());
        Assert.Equal(58, r.GetProperty("resendRttMs").GetDouble());
        Assert.Equal(40, r.GetProperty("recovered").GetInt64());
    }

    [Fact]
    public void CloseReasonFitsTheWebSocketLimit()
    {
        var r = Controller.RobotControl.Hosting.HttpEndpoints.CloseReason(new string('x', 500));
        Assert.True(System.Text.Encoding.UTF8.GetByteCount(r) <= 123);
        Assert.Equal("short", Controller.RobotControl.Hosting.HttpEndpoints.CloseReason("short"));
    }
}
