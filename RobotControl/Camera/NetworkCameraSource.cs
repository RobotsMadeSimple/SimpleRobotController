using OpenCvSharp;
using System;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;

namespace Controller.RobotControl.Camera
{
    /// <summary>Result of <see cref="NetworkCameraSource.Test"/> (the <c>TestCameraSource</c> command).</summary>
    public sealed class CameraSourceTestResult
    {
        public bool    Ok           { get; init; }
        public int     Width        { get; init; }
        public int     Height       { get; init; }
        public long    OpenMs       { get; init; }
        public long    FirstFrameMs { get; init; }
        /// <summary>null on success, else <c>invalidUrl</c> | <c>openFailed</c> | <c>noFrame</c> | <c>timeout</c>.</summary>
        public string? Error        { get; init; }
    }

    /// <summary>
    /// Network (RTSP / HTTP) camera helpers: URL building with injected credentials,
    /// password masking for logs, the FFmpeg open, and the one-shot source test.
    /// See docs/network-cameras.md.
    /// </summary>
    public static class NetworkCameraSource
    {
        public const string SourceUsb     = "usb";
        public const string SourceNetwork = "network";
        public const string TransportTcp  = "tcp";
        public const string TransportUdp  = "udp";

        public const int DefaultOpenTimeoutMs = 8000;
        public const int DefaultReadTimeoutMs = 5000;

        // cv::CAP_PROP_OPEN_TIMEOUT_MSEC / CAP_PROP_READ_TIMEOUT_MSEC (OpenCV >= 4.5.3).
        // Passed as open parameters so they apply to the open itself; builds that do not
        // know them ignore them.
        private const int CapPropOpenTimeoutMsec = 53;
        private const int CapPropReadTimeoutMsec = 54;

        private const string CaptureOptionsEnv = "OPENCV_FFMPEG_CAPTURE_OPTIONS";

        private static readonly string[] AllowedSchemes = { "rtsp", "rtsps", "http", "https" };

        public static string NormalizeSourceType(string? s) =>
            string.Equals(s?.Trim(), SourceNetwork, StringComparison.OrdinalIgnoreCase) ? SourceNetwork : SourceUsb;

        public static string NormalizeTransport(string? t) =>
            string.Equals(t?.Trim(), TransportUdp, StringComparison.OrdinalIgnoreCase) ? TransportUdp : TransportTcp;

        // ── URL handling ───────────────────────────────────────────────────────

        /// <summary>True when <paramref name="url"/> is an absolute rtsp(s)/http(s) URL with a host.</summary>
        public static bool IsValidUrl(string? url)
        {
            if (string.IsNullOrWhiteSpace(url)) return false;
            if (!Uri.TryCreate(url.Trim(), UriKind.Absolute, out var uri)) return false;
            if (Array.IndexOf(AllowedSchemes, uri.Scheme.ToLowerInvariant()) < 0) return false;
            return !string.IsNullOrEmpty(uri.Host);
        }

        /// <summary>
        /// The URL actually opened: <paramref name="url"/> with <c>user:pass@</c> injected
        /// after the scheme when a username is set (both URL-encoded, so <c>@ : /</c> in a
        /// password survive). Credentials already present in the URL are replaced when
        /// separate credentials are given, kept otherwise. Returns null for an invalid URL.
        /// </summary>
        public static string? BuildUrl(string? url, string? username, string? password)
        {
            if (!IsValidUrl(url)) return null;
            url = url!.Trim();
            if (string.IsNullOrEmpty(username)) return url;

            var (prefix, authority, rest) = SplitAuthority(url);
            int at = authority.LastIndexOf('@');
            var hostPart = at >= 0 ? authority[(at + 1)..] : authority;

            var userInfo = Uri.EscapeDataString(username)
                         + (string.IsNullOrEmpty(password) ? "" : ":" + Uri.EscapeDataString(password));
            return prefix + userInfo + "@" + hostPart + rest;
        }

        /// <summary>The URL with any password replaced by <c>***</c> — the only form that may be logged.</summary>
        public static string MaskUrl(string? url)
        {
            if (string.IsNullOrEmpty(url)) return "";
            var (prefix, authority, rest) = SplitAuthority(url);
            if (prefix.Length == 0) return url;
            int at = authority.LastIndexOf('@');
            if (at < 0) return url;
            var userInfo = authority[..at];
            int colon    = userInfo.IndexOf(':');
            if (colon < 0) return url;
            return prefix + userInfo[..colon] + ":***" + authority[at..] + rest;
        }

        // "scheme://" | "user:pass@host:port" | "/path?query"
        private static (string Prefix, string Authority, string Tail) SplitAuthority(string url)
        {
            int s = url.IndexOf("://", StringComparison.Ordinal);
            if (s < 0) return ("", url, "");
            int start = s + 3;
            // The authority ends at the first '/', '?' or '#' after the last '@' (a raw '/'
            // is not allowed in userinfo, but be lenient with hand-typed URLs).
            int at     = url.LastIndexOf('@');
            int search = at >= start ? at : start;
            int end    = url.IndexOfAny(new[] { '/', '?', '#' }, search);
            if (end < 0) end = url.Length;
            return (url[..start], url[start..end], url[end..]);
        }

        // ── FFmpeg backend ─────────────────────────────────────────────────────

        private static readonly Lazy<bool> _ffmpeg = new(DetectFfmpeg);

        /// <summary>True when the loaded OpenCV build has the FFmpeg video backend.</summary>
        public static bool FfmpegAvailable => _ffmpeg.Value;

        private static bool DetectFfmpeg()
        {
            try
            {
                foreach (var raw in Cv2.GetBuildInformation().Split('\n'))
                {
                    var line = raw.Trim();
                    if (line.StartsWith("FFMPEG", StringComparison.OrdinalIgnoreCase)
                        && line.Contains("YES", StringComparison.OrdinalIgnoreCase))
                        return true;
                }
            }
            catch (Exception ex) { Console.WriteLine($"[Camera] Could not read OpenCV build information: {ex.Message}"); }
            return false;
        }

        // glibc's soname first ("libc" alone can resolve to the libc.so linker script), then
        // the generic name (musl and others).
        [DllImport("libc.so.6", EntryPoint = "setenv")]
        private static extern int glibc_setenv(string name, string value, int overwrite);

        [DllImport("libc", EntryPoint = "setenv")]
        private static extern int libc_setenv(string name, string value, int overwrite);

        private static void unix_setenv(string name, string value, int overwrite)
        {
            try   { glibc_setenv(name, value, overwrite); }
            catch (DllNotFoundException) { libc_setenv(name, value, overwrite); }
        }

        [DllImport("ucrtbase.dll", EntryPoint = "_putenv_s", CharSet = CharSet.Ansi)]
        private static extern int win_putenv_s(string name, string value);

        private static readonly object _envLock = new();

        /// <summary>
        /// Selects the RTSP transport for the next FFmpeg open. OpenCV reads it from the
        /// process-wide <c>OPENCV_FFMPEG_CAPTURE_OPTIONS</c> environment variable, so this is
        /// set before every network open and the last writer wins; opens that race with a
        /// different transport may pick up the other one. (.NET keeps its own copy of the
        /// environment on Unix, so the native environment is written directly.)
        /// </summary>
        public static void SetTransport(string transport)
        {
            var value = "rtsp_transport;" + NormalizeTransport(transport);
            lock (_envLock)
            {
                try { Environment.SetEnvironmentVariable(CaptureOptionsEnv, value); } catch { }
                try
                {
                    if (OperatingSystem.IsWindows()) win_putenv_s(CaptureOptionsEnv, value);
                    else                             unix_setenv(CaptureOptionsEnv, value, 1);
                }
                catch { /* best effort — the managed variable is still set */ }
            }
        }

        /// <summary>
        /// Opens <paramref name="effectiveUrl"/> with the FFmpeg backend. Returns null when it
        /// does not open. The caller owns (and must release) the returned capture.
        /// </summary>
        public static VideoCapture? Open(string effectiveUrl, string transport,
                                         int openTimeoutMs = DefaultOpenTimeoutMs,
                                         int readTimeoutMs = DefaultReadTimeoutMs)
        {
            SetTransport(transport);
            VideoCapture? cap = null;
            try
            {
                var prms = new[] { CapPropOpenTimeoutMsec, openTimeoutMs, CapPropReadTimeoutMsec, readTimeoutMs };
                try   { cap = new VideoCapture(effectiveUrl, VideoCaptureAPIs.FFMPEG, prms); }
                catch { cap = null; }
                // Builds without open-parameter support: plain open.
                if (cap == null) cap = new VideoCapture(effectiveUrl, VideoCaptureAPIs.FFMPEG);

                if (!cap.IsOpened())
                {
                    GC.SuppressFinalize(cap);
                    return null;
                }
                try { cap.Set(VideoCaptureProperties.BufferSize, 1); } catch { }
                return cap;
            }
            catch (Exception)
            {
                if (cap != null) GC.SuppressFinalize(cap);
                return null;
            }
        }

        // ── TestCameraSource ───────────────────────────────────────────────────

        /// <summary>
        /// Opens the stream once on a worker thread, grabs one frame and closes it. Never
        /// blocks longer than about <paramref name="timeoutMs"/>: when the open or read
        /// overruns, <c>timeout</c> is returned and the worker releases the capture itself
        /// once the native call returns.
        /// </summary>
        public static async Task<CameraSourceTestResult> Test(string? url, string? username, string? password,
                                                              string? transport, int timeoutMs = DefaultOpenTimeoutMs)
        {
            var effective = BuildUrl(url, username, password);
            if (effective == null) return new CameraSourceTestResult { Error = "invalidUrl" };
            if (!FfmpegAvailable)  return new CameraSourceTestResult { Error = "openFailed" };

            timeoutMs = Math.Clamp(timeoutMs, 500, 60000);
            var tcs = new TaskCompletionSource<CameraSourceTestResult>(TaskCreationOptions.RunContinuationsAsynchronously);
            var worker = new Thread(() =>
            {
                var sw = Stopwatch.StartNew();
                VideoCapture? cap = null;
                try
                {
                    cap = Open(effective, NormalizeTransport(transport), timeoutMs, timeoutMs);
                    long openMs = sw.ElapsedMilliseconds;
                    if (cap == null) { tcs.TrySetResult(new CameraSourceTestResult { OpenMs = openMs, Error = "openFailed" }); return; }

                    using var frame = new Mat();
                    bool got = cap.Read(frame) && !frame.Empty();
                    long firstMs = sw.ElapsedMilliseconds;
                    tcs.TrySetResult(got
                        ? new CameraSourceTestResult { Ok = true, Width = frame.Width, Height = frame.Height, OpenMs = openMs, FirstFrameMs = firstMs }
                        : new CameraSourceTestResult { OpenMs = openMs, FirstFrameMs = firstMs, Error = "noFrame" });
                }
                catch (Exception) { tcs.TrySetResult(new CameraSourceTestResult { OpenMs = sw.ElapsedMilliseconds, Error = "openFailed" }); }
                finally
                {
                    if (cap != null)
                    {
                        try { cap.Release(); } catch { }
                        try { cap.Dispose(); } catch { }
                    }
                }
            }) { IsBackground = true, Name = "CameraSourceTest" };
            worker.Start();

            // timeoutMs is the whole budget. The native open/read timeouts use it too, so a
            // silent source normally fails as openFailed/noFrame; the margin covers builds that
            // ignore those properties (and the rare open that overruns its own timeout).
            var done = await Task.WhenAny(tcs.Task, Task.Delay(timeoutMs + 1000));
            return done == tcs.Task ? tcs.Task.Result : new CameraSourceTestResult { Error = "timeout" };
        }
    }
}
