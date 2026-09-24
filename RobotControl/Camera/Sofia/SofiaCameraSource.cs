using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;

namespace Controller.RobotControl.Camera.Sofia
{
    /// <summary>Result of <see cref="SofiaCameraSource.Test"/> (<c>TestCameraSource</c> with <c>sourceType: "sofia"</c>).</summary>
    public sealed class SofiaSourceTestResult
    {
        public bool    Ok              { get; init; }
        /// <summary>Time to finish the handshake (TCP connect, login, monitor claim/start).</summary>
        public long    LoginMs         { get; init; }
        /// <summary>Time until the first video frame arrived (from the start of the test).</summary>
        public long    FirstFrameMs    { get; init; }
        /// <summary><c>h264</c> | <c>hevc</c> | <c>unknown</c>.</summary>
        public string  DetectedCodec   { get; init; } = DvripClient.CodecUnknown;
        public int     FirstFrameBytes { get; init; }
        /// <summary>Decoded size; 0 when the decoder could not be exercised within the budget.</summary>
        public int     Width           { get; init; }
        public int     Height          { get; init; }
        /// <summary>null on success, else <c>connectFailed</c> | <c>loginFailed</c> | <c>claimFailed</c> | <c>noFrame</c> | <c>timeout</c> | <c>decoderUnavailable</c>.</summary>
        public string? Error           { get; init; }
        /// <summary>Detail for the error (e.g. <c>DVRIP login failed (Ret=203)</c>); never contains the password.</summary>
        public string? Message         { get; init; }
    }

    /// <summary>Sofia / DVRIP (XMeye) camera helpers: field normalisation and the one-shot source test.</summary>
    public static class SofiaCameraSource
    {
        public const int    DefaultPort       = DvripClient.DefaultPort;
        public const string DefaultUsername   = "admin";
        public const string StreamMain        = "Main";
        public const string StreamExtra1      = "Extra1";
        public const string DefaultFfmpegPath = "ffmpeg";
        public const int    DefaultTimeoutMs  = 8000;
        /// <summary>Socket read timeout while streaming: a camera silent this long is reconnected.</summary>
        public const int    StreamReadTimeoutMs = 10000;

        public static int NormalizePort(int? port) => port is > 0 and <= 65535 ? port.Value : DefaultPort;

        /// <summary><c>Main</c> or <c>Extra1</c> (<c>sub</c>/<c>extra</c>/<c>extra1</c> in any case map to <c>Extra1</c>).</summary>
        public static string NormalizeStream(string? s)
        {
            var v = s?.Trim().ToLowerInvariant();
            return v is "extra1" or "extra" or "sub" ? StreamExtra1 : StreamMain;
        }

        /// <summary><c>h264</c> or <c>hevc</c> (<c>h265</c>/<c>hevc</c> map to <c>hevc</c>).</summary>
        public static string NormalizeCodec(string? c)
        {
            var v = c?.Trim().ToLowerInvariant();
            return v is "hevc" or "h265" or "h.265" ? DvripClient.CodecHevc : DvripClient.CodecH264;
        }

        /// <summary><c>opencv</c> (default) or <c>ffmpeg</c>.</summary>
        public static string NormalizeDecoder(string? d) =>
            string.Equals(d?.Trim(), SofiaDecoder.DecoderFfmpeg, StringComparison.OrdinalIgnoreCase)
                ? SofiaDecoder.DecoderFfmpeg : SofiaDecoder.DecoderOpenCv;

        public static string NormalizeFfmpegPath(string? p) => string.IsNullOrWhiteSpace(p) ? DefaultFfmpegPath : p.Trim();

        public static string NormalizeHwaccel(string? h) => h?.Trim() ?? "";

        /// <summary>Normalised connection settings. An empty username logs in as <c>admin</c>.</summary>
        public static SofiaSettings Settings(string? host, int? port, string? username, string? password, string? stream,
                                             string? codec, string? decoder, string? ffmpegPath, string? hwaccel) =>
            new(host?.Trim() ?? "", NormalizePort(port),
                string.IsNullOrEmpty(username) ? DefaultUsername : username,
                password ?? "", NormalizeStream(stream), NormalizeCodec(codec), NormalizeDecoder(decoder),
                NormalizeFfmpegPath(ffmpegPath), NormalizeHwaccel(hwaccel));

        /// <summary>
        /// Connect → login → claim → first video frame within <paramref name="timeoutMs"/>; then,
        /// if time remains, decode one frame with the configured decoder to report its size.
        /// Runs on a worker thread; <paramref name="timeoutMs"/> is the whole budget (socket
        /// timeouts use what is left of it). When the decode overruns the budget, the handshake
        /// result is returned with width/height 0; when the handshake or first frame overruns it
        /// (plus a 1 s margin), <c>timeout</c>.
        /// </summary>
        public static async Task<SofiaSourceTestResult> Test(SofiaSettings s, int timeoutMs = DefaultTimeoutMs)
        {
            if (string.IsNullOrWhiteSpace(s.Host))
                return new SofiaSourceTestResult { Error = "connectFailed", Message = "host is required" };

            timeoutMs = Math.Clamp(timeoutMs, 500, 60000);
            var tcs = new TaskCompletionSource<SofiaSourceTestResult>(TaskCreationOptions.RunContinuationsAsynchronously);
            SofiaSourceTestResult? partial = null;   // handshake + first frame done, decode pending
            var session = new SofiaSession(s, $"test {s.Host}:{s.Port}");

            var worker = new Thread(() =>
            {
                var sw = Stopwatch.StartNew();
                int Remaining() => (int)(timeoutMs - sw.ElapsedMilliseconds);
                try
                {
                    // 1. TCP connect + login + claim/start
                    try { session.Connect(timeoutMs); }
                    catch (DvripException ex)
                    {
                        string code = ex.Stage switch
                        {
                            DvripStage.Connect => "connectFailed",
                            DvripStage.Login   => "loginFailed",
                            _                  => "claimFailed",
                        };
                        // A reply that never came within the budget is a timeout, not a refusal.
                        if (ex.Ret == null && ex.Stage != DvripStage.Connect && Remaining() <= 0) code = "timeout";
                        tcs.TrySetResult(new SofiaSourceTestResult { LoginMs = sw.ElapsedMilliseconds, Error = code, Message = ex.Message });
                        return;
                    }
                    long loginMs = sw.ElapsedMilliseconds;

                    // 2. First video frame within what is left of the budget
                    if (Remaining() <= 0) { tcs.TrySetResult(new SofiaSourceTestResult { LoginMs = loginMs, Error = "timeout" }); return; }
                    DvripVideoFrame first;
                    try { first = session.ReadFirstFrame(Math.Max(1, Remaining())); }
                    catch (Exception ex)
                    {
                        tcs.TrySetResult(new SofiaSourceTestResult
                        {
                            LoginMs = loginMs, FirstFrameMs = sw.ElapsedMilliseconds, Error = "noFrame", Message = ex.Message,
                        });
                        return;
                    }
                    long firstMs = sw.ElapsedMilliseconds;
                    var ok = new SofiaSourceTestResult
                    {
                        Ok = true, LoginMs = loginMs, FirstFrameMs = firstMs,
                        DetectedCodec = session.DetectedCodec, FirstFrameBytes = first.Data.Length,
                    };
                    Volatile.Write(ref partial, ok);

                    // 3. Decode one picture if time remains
                    if (Remaining() < 200) { tcs.TrySetResult(ok); return; }
                    try
                    {
                        session.StartDecoder(first, Remaining(), Math.Max(200, Remaining()));
                        using var pic = session.ReadDecoded();
                        tcs.TrySetResult(pic != null && pic.Width > 0
                            ? new SofiaSourceTestResult
                              {
                                  Ok = true, LoginMs = loginMs, FirstFrameMs = firstMs, DetectedCodec = ok.DetectedCodec,
                                  FirstFrameBytes = ok.FirstFrameBytes, Width = pic.Width, Height = pic.Height,
                              }
                            : ok);
                    }
                    catch (DecoderUnavailableException ex)
                    {
                        tcs.TrySetResult(new SofiaSourceTestResult
                        {
                            LoginMs = loginMs, FirstFrameMs = firstMs, DetectedCodec = ok.DetectedCodec,
                            FirstFrameBytes = ok.FirstFrameBytes, Error = "decoderUnavailable", Message = ex.Message,
                        });
                    }
                    catch (Exception) { tcs.TrySetResult(ok); }   // decoder did not produce a picture in time
                }
                catch (Exception ex)
                {
                    tcs.TrySetResult(new SofiaSourceTestResult { LoginMs = sw.ElapsedMilliseconds, Error = "connectFailed", Message = ex.Message });
                }
                finally
                {
                    session.Dispose();
                }
            }) { IsBackground = true, Name = "SofiaSourceTest" };
            worker.Start();

            var done = await Task.WhenAny(tcs.Task, Task.Delay(timeoutMs)).ConfigureAwait(false);
            if (done == tcs.Task) return tcs.Task.Result;

            // Budget spent. With the first frame in hand, report it (decoder not exercised).
            var got = Volatile.Read(ref partial);
            if (got != null)
            {
                session.Abort();
                return tcs.Task.IsCompleted && tcs.Task.Result.Width > 0 ? tcs.Task.Result : got;
            }

            // Socket timeouts use the same budget, so a natural failure lands just after it.
            done = await Task.WhenAny(tcs.Task, Task.Delay(1000)).ConfigureAwait(false);
            if (done == tcs.Task) return tcs.Task.Result;
            session.Abort();
            return new SofiaSourceTestResult { Error = "timeout" };
        }
    }
}
