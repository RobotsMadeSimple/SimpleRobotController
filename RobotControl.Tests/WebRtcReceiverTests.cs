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
    public void CloseReasonFitsTheWebSocketLimit()
    {
        var r = Controller.RobotControl.Hosting.HttpEndpoints.CloseReason(new string('x', 500));
        Assert.True(System.Text.Encoding.UTF8.GetByteCount(r) <= 123);
        Assert.Equal("short", Controller.RobotControl.Hosting.HttpEndpoints.CloseReason("short"));
    }
}
