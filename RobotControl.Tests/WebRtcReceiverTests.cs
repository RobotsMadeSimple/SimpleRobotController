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
}
