using OpenCvSharp;
using System;

namespace Controller.RobotControl.Camera.Sofia
{
    /// <summary>The configured decoder cannot run here (ffmpeg executable missing, OpenCV without FFmpeg).</summary>
    public sealed class DecoderUnavailableException : Exception
    {
        public DecoderUnavailableException(string message, Exception? inner = null) : base(message, inner) { }
    }

    /// <summary>
    /// One decoded picture: a BGR <see cref="Mat"/> (in-process decoder) or a ready JPEG
    /// (ffmpeg decoder, which already outputs MJPEG). The receiver disposes it.
    /// </summary>
    public sealed class SofiaDecodedFrame : IDisposable
    {
        public Mat?    Mat    { get; }
        public byte[]? Jpeg   { get; }
        public int     Width  { get; }
        public int     Height { get; }

        private SofiaDecodedFrame(Mat? mat, byte[]? jpeg, int width, int height)
        {
            Mat = mat; Jpeg = jpeg; Width = width; Height = height;
        }

        public static SofiaDecodedFrame FromMat(Mat mat) => new(mat, null, mat.Width, mat.Height);

        public static SofiaDecodedFrame FromJpeg(byte[] jpeg)
        {
            JpegInfo.TryGetSize(jpeg, out int w, out int h);
            return new SofiaDecodedFrame(null, jpeg, w, h);
        }

        /// <summary>The frame as JPEG bytes: passed through for ffmpeg output, encoded for a Mat.</summary>
        public byte[] ToJpeg(int[] encodeParams)
        {
            if (Jpeg != null) return Jpeg;
            Cv2.ImEncode(".jpg", Mat!, out var buf, encodeParams);
            return buf;
        }

        public void Dispose() => Mat?.Dispose();
    }

    /// <summary>
    /// Turns the Annex-B elementary stream from <see cref="DvripClient"/> into pictures.
    /// Threading: <see cref="Write"/> is called by the DVRIP pump thread only;
    /// <see cref="Open"/>, <see cref="Read"/> and <see cref="Dispose"/> by the owning (capture)
    /// thread only; <see cref="Abort"/> from any thread — it closes the input side so a
    /// blocked <see cref="Read"/> returns (null) and a blocked <see cref="Write"/> throws.
    /// </summary>
    public abstract class SofiaDecoder : IDisposable
    {
        public const string DecoderOpenCv = "opencv";
        public const string DecoderFfmpeg = "ffmpeg";

        /// <summary><c>opencv</c> or <c>ffmpeg</c>.</summary>
        public abstract string Kind { get; }

        /// <summary>Feeds one video frame (pump thread). Throws <see cref="System.IO.IOException"/> when the decoder has gone away.</summary>
        public abstract void Write(byte[] frame, bool isKeyFrame);

        /// <summary>Gets the decoder ready to <see cref="Read"/> (owner thread). Throws on failure.</summary>
        public abstract void Open(int timeoutMs);

        /// <summary>Blocks for the next decoded picture (owner thread); null when the stream ended or failed.</summary>
        public abstract SofiaDecodedFrame? Read();

        /// <summary>Closes the input side from any thread (unblocks the pump and the reader).</summary>
        public abstract void Abort();

        public abstract void Dispose();
    }

    /// <summary>Reads the frame size out of a JPEG's SOFn header without decoding it.</summary>
    public static class JpegInfo
    {
        public static bool TryGetSize(byte[] jpeg, out int width, out int height)
        {
            width = height = 0;
            if (jpeg.Length < 4 || jpeg[0] != 0xFF || jpeg[1] != 0xD8) return false;
            int i = 2;
            while (i + 3 < jpeg.Length)
            {
                if (jpeg[i] != 0xFF) { i++; continue; }
                byte marker = jpeg[i + 1];
                if (marker == 0xFF) { i++; continue; }                         // fill byte
                if (marker == 0xD8 || marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) { i += 2; continue; }
                if (marker == 0xD9 || marker == 0xDA) return false;           // no SOF before the scan
                int len = (jpeg[i + 2] << 8) | jpeg[i + 3];
                bool sof = marker >= 0xC0 && marker <= 0xCF && marker != 0xC4 && marker != 0xC8 && marker != 0xCC;
                if (sof && i + 8 < jpeg.Length)
                {
                    height = (jpeg[i + 5] << 8) | jpeg[i + 6];
                    width  = (jpeg[i + 7] << 8) | jpeg[i + 8];
                    return width > 0 && height > 0;
                }
                i += 2 + len;
            }
            return false;
        }
    }
}
