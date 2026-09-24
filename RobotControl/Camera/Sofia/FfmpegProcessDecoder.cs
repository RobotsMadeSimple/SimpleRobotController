using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;

namespace Controller.RobotControl.Camera.Sofia
{
    /// <summary>
    /// External-process decoder (<c>decoder: "ffmpeg"</c>), ported from octanecoffeebotv2's
    /// DVRIP backend: the elementary stream is written to <c>ffmpeg</c>'s stdin and each
    /// picture comes back on stdout as a self-delimiting JPEG (SOI..EOI), which is published
    /// as-is (no decode/re-encode). stderr is drained so a full pipe cannot stall ffmpeg.
    ///
    /// <code>ffmpeg -hide_banner -loglevel error [-hwaccel X] -f h264|hevc -fflags nobuffer
    ///        -flags low_delay -i pipe:0 -an -f mjpeg -q:v 4 -flush_packets 1 pipe:1</code>
    /// (<c>-flush_packets 1</c>, from the reference, pushes every JPEG down the pipe at once.)
    /// </summary>
    public sealed class FfmpegProcessDecoder : SofiaDecoder
    {
        private readonly Process _proc;
        private readonly Stream  _stdin;
        private readonly Stream  _stdout;
        private readonly string  _tag;
        private volatile bool    _aborted;

        // Rolling stdout buffer (owner thread only)
        private byte[] _buf = new byte[1 << 20];
        private int    _len;
        private readonly byte[] _chunk = new byte[64 * 1024];

        public override string Kind => DecoderFfmpeg;

        /// <summary>The ffmpeg command-line arguments for a codec / hwaccel (exposed for tests).</summary>
        public static List<string> BuildArguments(string codec, string? hwaccel)
        {
            var args = new List<string> { "-hide_banner", "-loglevel", "error" };
            // Hardware-accelerated decode when configured
            if (!string.IsNullOrWhiteSpace(hwaccel)) { args.Add("-hwaccel"); args.Add(hwaccel.Trim()); }
            // The DVRIP client feeds a raw elementary stream on stdin (h264 or hevc); no input buffering
            args.AddRange(new[] { "-f", codec == DvripClient.CodecHevc ? "hevc" : "h264",
                                  "-fflags", "nobuffer", "-flags", "low_delay", "-i", "pipe:0" });
            // Frames only, MJPEG out, flushed per frame
            args.AddRange(new[] { "-an", "-f", "mjpeg", "-q:v", "4", "-flush_packets", "1", "pipe:1" });
            return args;
        }

        /// <summary>Starts ffmpeg. Throws <see cref="DecoderUnavailableException"/> when the executable cannot be started.</summary>
        public FfmpegProcessDecoder(string tag, string ffmpegPath, string codec, string? hwaccel)
        {
            _tag = tag;
            var path = string.IsNullOrWhiteSpace(ffmpegPath) ? "ffmpeg" : ffmpegPath.Trim();
            var startInfo = new ProcessStartInfo
            {
                FileName               = path,
                RedirectStandardInput  = true,
                RedirectStandardOutput = true,
                RedirectStandardError  = true,
                UseShellExecute        = false,
                CreateNoWindow         = true,
            };
            // ArgumentList avoids any quoting issues
            foreach (var a in BuildArguments(codec, hwaccel)) startInfo.ArgumentList.Add(a);

            _proc = new Process { StartInfo = startInfo };
            try
            {
                if (!_proc.Start()) throw new DecoderUnavailableException($"ffmpeg not found at {path}");
            }
            catch (Exception ex) when (ex is Win32Exception or FileNotFoundException or InvalidOperationException or PlatformNotSupportedException)
            {
                _proc.Dispose();
                throw new DecoderUnavailableException($"ffmpeg not found at {path}", ex);
            }

            // Drain stderr on its own thread so a full pipe can never stall ffmpeg
            _proc.ErrorDataReceived += (_, e) =>
            {
                if (!string.IsNullOrEmpty(e.Data)) Console.WriteLine($"[Sofia] {_tag} ffmpeg: {e.Data}");
            };
            _proc.BeginErrorReadLine();
            _stdin  = _proc.StandardInput.BaseStream;
            _stdout = _proc.StandardOutput.BaseStream;
        }

        public override void Write(byte[] frame, bool isKeyFrame)
        {
            if (_aborted) throw new IOException("decoder closed");
            try
            {
                _stdin.Write(frame, 0, frame.Length);
                _stdin.Flush();
            }
            catch (ObjectDisposedException ex) { throw new IOException("ffmpeg stdin closed", ex); }
        }

        public override void Open(int timeoutMs)
        {
            // The process is already running; nothing to wait for.
            if (_aborted) throw new IOException("decoder closed");
        }

        /// <summary>Reads stdout until the next complete JPEG; null when ffmpeg closed the pipe.</summary>
        public override SofiaDecodedFrame? Read()
        {
            while (!_aborted)
            {
                var jpeg = ExtractJpeg();
                if (jpeg != null) return SofiaDecodedFrame.FromJpeg(jpeg);

                int read;
                try { read = _stdout.Read(_chunk, 0, _chunk.Length); }
                catch (Exception ex) when (ex is IOException or ObjectDisposedException) { return null; }
                if (read <= 0) return null;

                if (_len + read > _buf.Length)
                {
                    if (_buf.Length >= 64 * 1024 * 1024) _len = 0;                   // runaway: drop it
                    else Array.Resize(ref _buf, Math.Max(_buf.Length * 2, _len + read));
                }
                Buffer.BlockCopy(_chunk, 0, _buf, _len, read);
                _len += read;
            }
            return null;
        }

        // Pulls the first complete SOI..EOI out of the buffer, dropping leading garbage.
        private byte[]? ExtractJpeg()
        {
            int soi = IndexOfMarker(0xD8, 0);
            if (soi < 0)
            {
                // Keep only a trailing byte in case an FF was split across chunks
                if (_len > 1) { _buf[0] = _buf[_len - 1]; _len = 1; }
                return null;
            }
            int eoi = IndexOfMarker(0xD9, soi + 2);
            if (eoi < 0)
            {
                // Incomplete frame; discard bytes before the start to bound growth
                if (soi > 0) { Buffer.BlockCopy(_buf, soi, _buf, 0, _len - soi); _len -= soi; }
                return null;
            }
            int end = eoi + 2;
            var jpeg = new byte[end - soi];
            Buffer.BlockCopy(_buf, soi, jpeg, 0, jpeg.Length);
            Buffer.BlockCopy(_buf, end, _buf, 0, _len - end);
            _len -= end;
            return jpeg;
        }

        private int IndexOfMarker(byte second, int start)
        {
            for (int i = Math.Max(start, 0); i < _len - 1; i++)
                if (_buf[i] == 0xFF && _buf[i + 1] == second) return i;
            return -1;
        }

        public override void Abort()
        {
            _aborted = true;
            // Killing the process closes both pipes: the pump's write and the reader's read fail.
            try { if (!_proc.HasExited) _proc.Kill(entireProcessTree: true); } catch { }
        }

        public override void Dispose()
        {
            Abort();
            try { _proc.WaitForExit(1000); } catch { }
            try { _proc.Dispose(); } catch { }
        }
    }
}
