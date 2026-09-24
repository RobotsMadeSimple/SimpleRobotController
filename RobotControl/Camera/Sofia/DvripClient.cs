using System;
using System.Collections.Generic;
using System.IO;
using System.Net.Sockets;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace Controller.RobotControl.Camera.Sofia
{
    /// <summary>Which step of the DVRIP handshake failed (maps to the TestCameraSource error codes).</summary>
    public enum DvripStage { Connect, Login, Claim, Stream }

    /// <summary>A DVRIP handshake failure: the stage it failed at and, for a refused login/claim, the camera's Ret code.</summary>
    public sealed class DvripException : Exception
    {
        public DvripStage Stage { get; }
        public int?       Ret   { get; }

        public DvripException(DvripStage stage, string message, int? ret = null, Exception? inner = null)
            : base(message, inner)
        {
            Stage = stage;
            Ret   = ret;
        }
    }

    /// <summary>One reassembled Sofia video frame (H.264/H.265 Annex-B access unit).</summary>
    public readonly record struct DvripVideoFrame(byte[] Data, bool IsKeyFrame);

    /// <summary>
    /// Speaks the XMeye / Sofia "DVRIP" protocol (TCP port 34567) that the vendor VMS uses,
    /// which buffers far less than the cameras' RTSP server. Logs in, claims and starts the
    /// monitor stream, then yields the raw H.264/H.265 Annex-B elementary stream one video
    /// frame at a time (<see cref="ReadVideoFrame"/>) or copies it into a stream
    /// (<see cref="CopyVideoTo"/>). Mirrors OpenIPC/python-dvr's framing.
    ///
    /// <para>Port of octanecoffeebotv2's <c>CCamera/DvripClient</c> (docs/network-cameras.md,
    /// "Sofia / DVRIP"). Additions: a connect timeout, typed handshake errors
    /// (<see cref="DvripException"/>), a socket read timeout so a silent camera surfaces as an
    /// error instead of a hang, and a thread-safe <see cref="Dispose"/> that unblocks a read in
    /// progress on another thread. The password is never logged.</para>
    /// </summary>
    public sealed class DvripClient : IDisposable
    {
        public const int DefaultPort = 34567;

        // Message ids from the protocol
        private const int LoginRequest     = 1000;
        private const int KeepAliveRequest = 1006;
        private const int MonitorClaim     = 1413;
        private const int MonitorStart     = 1410;

        // Sofia media frame magics (read big-endian from the start of each frame)
        internal const uint FrameIFrame = 0x000001FC;
        internal const uint FrameJpeg   = 0x000001FE;
        internal const uint FramePFrame = 0x000001FD;

        // Ret codes the camera treats as success
        private static readonly int[] OkCodes = { 100, 515 };

        // Hard ceiling on a single DVRIP header/payload read; a length beyond this (or a
        // negative one, from a uint length field that overflowed an int) means the stream
        // desynced or the header was corrupt, not a real 8 MB+ packet
        internal const int MaxPacketBytes = 8 * 1024 * 1024;

        // How far the resync will scan for the next frame boundary before giving up and
        // forcing a reconnect; bounds the work so a truly broken stream can't hang the loop
        internal const int MaxResyncScanBytes = 4 * 1024 * 1024;

        // A reassembled frame larger than this is treated as a desync (no camera frame is 32 MB)
        private const int MaxFrameBytes = 32 * 1024 * 1024;

        // Connection settings
        private readonly string _host;
        private readonly int    _port;
        private readonly string _username;
        private readonly string _password;
        private readonly string _stream;

        // The live socket and its stream
        private TcpClient?     _client;
        private NetworkStream? _net;

        // Buffered read view of the socket, so the resync can scan a byte at a time cheaply
        private Stream? _in;

        // Bytes the resync read past the header and must replay before the next socket read
        private byte[] _pushback = Array.Empty<byte>();

        // Session id from login and the rolling send sequence number
        private uint _session;
        private uint _packetCount;

        // Keepalive pacing
        private Timer? _keepAlive;
        private int    _aliveSeconds = 20;

        // Guards concurrent sends (the keepalive timer vs the setup calls)
        private readonly object _sendLock = new();

        private volatile bool _disposed;

        /// <summary>Session id the camera assigned at login.</summary>
        public uint SessionId => _session;
        /// <summary>Keepalive interval the camera advertised (seconds; 20 when it did not say).</summary>
        public int  AliveIntervalSeconds => _aliveSeconds;
        /// <summary>Number of garbage bytes skipped by resyncs so far (diagnostics / tests).</summary>
        public long ResyncSkippedBytes { get; private set; }
        /// <summary>Number of resyncs performed so far.</summary>
        public int  ResyncCount { get; private set; }

        /// <summary>
        /// Socket receive timeout once streaming (ms). A camera that goes silent for this long
        /// surfaces as an <see cref="IOException"/> from <see cref="ReadVideoFrame"/>. 0 = none.
        /// </summary>
        public int ReadTimeoutMs
        {
            get => _readTimeoutMs;
            set
            {
                _readTimeoutMs = Math.Max(0, value);
                try { if (_client != null) _client.ReceiveTimeout = _readTimeoutMs; } catch { }
            }
        }
        private int _readTimeoutMs = 10000;

        public DvripClient(string host, int port, string username, string password, string stream)
        {
            _host     = host;
            _port     = port > 0 && port <= 65535 ? port : DefaultPort;
            _username = username;
            _password = password ?? "";
            _stream   = stream;
        }

        /// <summary>Connects, logs in, and starts the monitor stream. Throws <see cref="DvripException"/> on any failure.</summary>
        public void Connect(int timeoutMs = 8000) => ConnectAsync(timeoutMs).GetAwaiter().GetResult();

        /// <summary>
        /// Connects (TCP within <paramref name="timeoutMs"/>), logs in, claims and starts the
        /// monitor stream. The login and claim replies must also arrive within the timeout.
        /// Throws <see cref="DvripException"/> with the failing <see cref="DvripStage"/>.
        /// </summary>
        public async Task ConnectAsync(int timeoutMs = 8000, CancellationToken cancellationToken = default)
        {
            timeoutMs = Math.Max(1, timeoutMs);

            // Open the TCP socket with Nagle off so frames are not coalesced
            _client = new TcpClient { NoDelay = true };
            using (var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken))
            {
                cts.CancelAfter(timeoutMs);
                try
                {
                    await _client.ConnectAsync(_host, _port, cts.Token).ConfigureAwait(false);
                }
                catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
                {
                    throw new DvripException(DvripStage.Connect, $"DVRIP connect to {_host}:{_port} timed out after {timeoutMs} ms");
                }
                catch (Exception ex) when (ex is SocketException or IOException or ObjectDisposedException or ArgumentException)
                {
                    throw new DvripException(DvripStage.Connect, $"DVRIP connect to {_host}:{_port} failed: {ex.Message}", inner: ex);
                }
            }
            if (_disposed) throw new DvripException(DvripStage.Connect, "DVRIP client closed");

            _client.NoDelay = true;
            // The handshake replies must arrive within the connect budget too
            _client.ReceiveTimeout = timeoutMs;
            _client.SendTimeout    = timeoutMs;
            _net = _client.GetStream();

            // Read through a buffer so the resync can scan the stream one byte at a time
            _in = new BufferedStream(_net, 1 << 16);

            // Authenticate, then claim and start the live stream
            Login();
            StartMonitor();

            // Streaming: a silent camera shows up as a read timeout, not a hang
            _client.ReceiveTimeout = _readTimeoutMs;

            // Begin keepalives so the camera does not drop the session
            _keepAlive = new Timer(_ => SendKeepAlive(), null, _aliveSeconds * 1000, _aliveSeconds * 1000);
        }

        /// <summary>
        /// Reads Sofia frames until the next video frame and returns its reassembled payload.
        /// Audio / metadata / JPEG frames are skipped. Throws on socket errors, end of stream,
        /// a read timeout, or when the resync gives up.
        /// </summary>
        public DvripVideoFrame ReadVideoFrame()
        {
            while (true)
            {
                SofiaFrame frame = ReadFrame();
                if (frame.IsVideo) return new DvripVideoFrame(frame.Payload, frame.IsKeyFrame);
            }
        }

        /// <summary>
        /// Reads monitor frames and writes the H.264/H.265 payload of each video frame to output.
        /// Returns when running() goes false or the socket ends; throws on socket errors.
        /// </summary>
        public void CopyVideoTo(Stream output, Func<bool> running)
        {
            bool loggedFirst = false;

            while (running())
            {
                // Pull the next complete video frame and forward it
                DvripVideoFrame frame = ReadVideoFrame();

                // Log the first video frame's leading bytes once so the codec can be identified
                if (!loggedFirst)
                {
                    LogFirstFrame(frame.Data);
                    loggedFirst = true;
                }

                output.Write(frame.Data, 0, frame.Data.Length);
                output.Flush();
            }
        }

        /// <summary>Prints the start of the first video frame; the NAL header reveals h264 vs hevc.</summary>
        public static void LogFirstFrame(byte[] payload, string? tag = null)
        {
            // Show up to the first 8 bytes as hex (e.g. 00 00 00 01 67 -> h264 SPS, 40/42 -> hevc)
            int count = Math.Min(8, payload.Length);
            string hex = BitConverter.ToString(payload, 0, count).Replace("-", " ");
            Console.WriteLine($"[Sofia]{(tag == null ? "" : " " + tag)} first video frame: {payload.Length} bytes, starts with {hex} ({DescribeNal(payload)})");
        }

        // ── Codec detection ────────────────────────────────────────────────────

        public const string CodecH264    = "h264";
        public const string CodecHevc    = "hevc";
        public const string CodecUnknown = "unknown";

        /// <summary>
        /// Identifies the codec of an Annex-B frame from its NAL headers: after a
        /// <c>00 00 01</c> / <c>00 00 00 01</c> start code, an H.264 SPS (NAL type 7, e.g.
        /// <c>0x67</c>) means <c>h264</c>; an HEVC VPS (<c>0x40</c>) or SPS (<c>0x42</c>) means
        /// <c>hevc</c>. The first recognised parameter set in the first 64 KB wins; anything
        /// else (a P-frame, garbage) is <c>unknown</c>.
        /// </summary>
        public static string DetectCodec(byte[] firstFrame)
        {
            var (codec, _) = Detect(firstFrame);
            return codec;
        }

        /// <summary>Human-readable name of the NAL that <see cref="DetectCodec"/> keyed on (for logs).</summary>
        public static string DescribeNal(byte[] frame) => Detect(frame).Nal;

        private static (string Codec, string Nal) Detect(byte[]? data)
        {
            if (data == null) return (CodecUnknown, "empty");
            int limit = Math.Min(data.Length, 64 * 1024);
            for (int i = 0; i + 3 < limit; i++)
            {
                if (data[i] != 0 || data[i + 1] != 0 || data[i + 2] != 1) continue;
                byte h = data[i + 3];
                // HEVC first: 0x40 / 0x42 would read as H.264 types 0 / 2, which never start a stream.
                if (h == 0x40) return (CodecHevc, "hevc VPS");
                if (h == 0x42) return (CodecHevc, "hevc SPS");
                if ((h & 0x80) == 0 && (h & 0x1F) == 7) return (CodecH264, "h264 SPS");
                i += 2;
            }
            return (CodecUnknown, "no SPS/VPS");
        }

        // ── Handshake ──────────────────────────────────────────────────────────

        /// <summary>Sends the login request and parses the session id and keepalive interval.</summary>
        private void Login()
        {
            JsonElement reply;
            try
            {
                // The password is hashed with the Sofia digest, not sent in clear
                var payload = new
                {
                    EncryptType = "MD5",
                    LoginType   = "DVRIP-Web",
                    UserName    = _username,
                    PassWord    = SofiaHash(_password),
                };
                Send(LoginRequest, payload);

                // The reply carries the session id (hex) and how often to keepalive
                reply = ReceiveJson();
            }
            catch (Exception ex) when (IsIoFailure(ex))
            {
                throw new DvripException(DvripStage.Login, $"DVRIP login failed: {ex.Message}", inner: ex);
            }

            int ret = ReadRet(reply);
            if (Array.IndexOf(OkCodes, ret) < 0)
                throw new DvripException(DvripStage.Login, $"DVRIP login failed (Ret={ret})", ret);

            // Session id arrives as a "0x........" string
            try
            {
                var sid = reply.GetProperty("SessionID").GetString() ?? "";
                _session = Convert.ToUInt32(sid.StartsWith("0x", StringComparison.OrdinalIgnoreCase) ? sid[2..] : sid, 16);
            }
            catch (Exception ex) when (ex is KeyNotFoundException or FormatException or OverflowException or InvalidOperationException or ArgumentException)
            {
                throw new DvripException(DvripStage.Login, "DVRIP login reply has no usable SessionID", ret, ex);
            }

            // Use the camera's advertised keepalive interval when present
            if (reply.TryGetProperty("AliveInterval", out var alive) && alive.ValueKind == JsonValueKind.Number
                && alive.TryGetInt32(out int seconds) && seconds > 0)
            {
                _aliveSeconds = seconds;
            }
        }

        /// <summary>Claims then starts the monitor stream for the configured channel and stream type.</summary>
        private void StartMonitor()
        {
            // The parameter block is identical for the claim and the start
            var parameters = new
            {
                Channel    = 0,
                CombinMode = "NONE",
                StreamType = _stream,
                TransMode  = "TCP",
            };

            JsonElement claim;
            try
            {
                // Claim reserves the stream; it must succeed before starting
                Send(MonitorClaim, new
                {
                    Name      = "OPMonitor",
                    SessionID = SessionHex(),
                    OPMonitor = new { Action = "Claim", Parameter = parameters },
                });
                claim = ReceiveJson();
            }
            catch (Exception ex) when (IsIoFailure(ex))
            {
                throw new DvripException(DvripStage.Claim, $"DVRIP monitor claim failed: {ex.Message}", inner: ex);
            }

            int ret = ReadRet(claim);
            if (Array.IndexOf(OkCodes, ret) < 0)
                throw new DvripException(DvripStage.Claim, $"DVRIP monitor claim failed (Ret={ret})", ret);

            try
            {
                // Start has no reply; media frames begin flowing right after
                Send(MonitorStart, new
                {
                    Name      = "OPMonitor",
                    SessionID = SessionHex(),
                    OPMonitor = new { Action = "Start", Parameter = parameters },
                });
            }
            catch (Exception ex) when (IsIoFailure(ex))
            {
                throw new DvripException(DvripStage.Claim, $"DVRIP monitor start failed: {ex.Message}", inner: ex);
            }
        }

        private static int ReadRet(JsonElement reply) =>
            reply.ValueKind == JsonValueKind.Object && reply.TryGetProperty("Ret", out var r)
            && r.ValueKind == JsonValueKind.Number && r.TryGetInt32(out int v) ? v : 0;

        private static bool IsIoFailure(Exception ex) =>
            ex is IOException or SocketException or ObjectDisposedException or InvalidDataException or JsonException or InvalidOperationException;

        // ── Media framing ──────────────────────────────────────────────────────

        /// <summary>Reads one full Sofia frame, reassembling it across DVRIP packets.</summary>
        private SofiaFrame ReadFrame()
        {
            var payload = new MemoryStream();
            int remaining = 0;
            bool isVideo = false, isKey = false;
            bool inFrame = false;

            while (true)
            {
                // Every DVRIP packet is a 20-byte header then len_data bytes; ReadHeader
                // re-anchors to the next frame if the stream has desynced
                byte[] header = ReadHeader();
                int lenData = (int)BitConverter.ToUInt32(header, 16);
                byte[] packet = ReadExact(lenData);

                // A non-media packet (e.g. a stray JSON reply) is skipped, not parsed as a frame
                if (!inFrame && !LooksLikeFrame(packet))
                {
                    continue;
                }

                // The first packet of a frame carries the Sofia header; later packets are pure payload
                int headerLen = 0;
                if (!inFrame)
                {
                    headerLen = ParseFrameHeader(packet, out remaining, out isVideo, out isKey);
                    if (headerLen > packet.Length || remaining < 0 || remaining > MaxFrameBytes)
                        throw new InvalidDataException($"DVRIP frame header invalid (length {remaining})");
                    inFrame = true;
                }

                // Append the payload portion and track how much of this frame is left
                int body = packet.Length - headerLen;
                payload.Write(packet, headerLen, body);
                remaining -= body;

                // The frame ends exactly on a packet boundary
                if (remaining <= 0)
                {
                    return new SofiaFrame(payload.ToArray(), isVideo, isKey);
                }
            }
        }

        /// <summary>Returns true if the packet starts with the Sofia 00 00 01 xx frame magic.</summary>
        private static bool LooksLikeFrame(byte[] packet)
        {
            return packet.Length >= 8 && packet[0] == 0x00 && packet[1] == 0x00 && packet[2] == 0x01;
        }

        /// <summary>Reads the Sofia frame header, returning its byte length and the payload length + video flag.</summary>
        private static int ParseFrameHeader(byte[] packet, out int payloadLength, out bool isVideo, out bool isKey)
        {
            // The magic is stored big-endian at the very start of the frame
            uint dataType = (uint)((packet[0] << 24) | (packet[1] << 16) | (packet[2] << 8) | packet[3]);

            // I-frame and JPEG share a 16-byte header with the length at offset 12 (little-endian)
            if (dataType == FrameIFrame || dataType == FrameJpeg)
            {
                if (packet.Length < 16) throw new InvalidDataException("DVRIP I-frame header truncated");
                payloadLength = (int)BitConverter.ToUInt32(packet, 12);
                isVideo = dataType == FrameIFrame;
                isKey   = isVideo;
                return 16;
            }

            // P-frame has an 8-byte header with the length at offset 4
            if (dataType == FramePFrame)
            {
                payloadLength = (int)BitConverter.ToUInt32(packet, 4);
                isVideo = true;
                isKey   = false;
                return 8;
            }

            // Audio / metadata frames: 8-byte header, length at offset 4, not forwarded
            payloadLength = (int)BitConverter.ToUInt32(packet, 4);
            isVideo = false;
            isKey   = false;
            return 8;
        }

        /// <summary>Sends the periodic keepalive; failures are swallowed since the read loop owns errors.</summary>
        private void SendKeepAlive()
        {
            if (_disposed) return;
            // A dead socket throws here; the read loop will surface the disconnect and reconnect
            try
            {
                Send(KeepAliveRequest, new { Name = "KeepAlive", SessionID = SessionHex() });
            }
            catch (IOException) { }
            catch (ObjectDisposedException) { }
            catch (SocketException) { }
            catch (InvalidOperationException) { }
        }

        /// <summary>Serializes a JSON command and writes it with the 20-byte DVRIP header.</summary>
        private void Send(int msgId, object payload)
        {
            // Build the JSON body and the null-terminated trailer the protocol expects
            byte[] json = JsonSerializer.SerializeToUtf8Bytes(payload);
            byte[] body = new byte[json.Length + 2];
            Array.Copy(json, body, json.Length);
            body[json.Length]     = 0x0A;
            body[json.Length + 1] = 0x00;

            // One writer at a time; the keepalive timer can race the setup calls
            lock (_sendLock)
            {
                // Header: 0xFF, version 0, pad, session, sequence, pad, msgid, length
                byte[] header = new byte[20];
                header[0] = 0xFF;
                BitConverter.GetBytes(_session).CopyTo(header, 4);
                BitConverter.GetBytes(_packetCount).CopyTo(header, 8);
                BitConverter.GetBytes((ushort)msgId).CopyTo(header, 14);
                BitConverter.GetBytes((uint)body.Length).CopyTo(header, 16);

                _packetCount++;
                var net = _net ?? throw new ObjectDisposedException(nameof(DvripClient));
                net.Write(header, 0, header.Length);
                net.Write(body, 0, body.Length);
                net.Flush();
            }
        }

        /// <summary>Reads a 20-byte header plus its JSON payload and returns the parsed root element.</summary>
        private JsonElement ReceiveJson()
        {
            // Pull the header to learn the payload length
            byte[] header = ReadExact(20);
            int lenData = (int)BitConverter.ToUInt32(header, 16);
            byte[] body = ReadExact(lenData);

            // Trim the trailing newline / null the camera appends before parsing
            int end = body.Length;
            while (end > 0 && (body[end - 1] == 0x00 || body[end - 1] == 0x0A))
            {
                end--;
            }

            // Clone the document's root so it stays valid after the document is disposed
            using var document = JsonDocument.Parse(new ReadOnlyMemory<byte>(body, 0, end));
            return document.RootElement.Clone();
        }

        /// <summary>
        /// Reads a 20-byte transport header, re-anchoring to the next frame when the stream
        /// has desynced so a single bad packet does not tear down the whole connection.
        /// </summary>
        private byte[] ReadHeader()
        {
            // Read the next candidate header off the stream
            byte[] header = ReadExact(20);

            // A header whose length field is sane is taken as-is (the common path)
            if (LengthInRange(header))
            {
                return header;
            }

            // Otherwise the stream desynced; scan forward to the next real frame boundary
            return ResyncToFrameHeader(header);
        }

        /// <summary>
        /// Slides a window forward until it sees a sane-length header immediately followed by
        /// the Sofia 00 00 01 frame magic, discarding the garbled bytes in between. Throws to
        /// force a reconnect only if no boundary turns up within the scan cap.
        /// </summary>
        private byte[] ResyncToFrameHeader(byte[] header)
        {
            // Window is the 20 header bytes plus the 3 magic bytes that must follow them
            byte[] window = new byte[23];
            Array.Copy(header, window, 20);
            window[20] = ReadScanByte();
            window[21] = ReadScanByte();
            window[22] = ReadScanByte();

            int scanned = 0;
            while (true)
            {
                // A valid frame start: in-range length, then the 00 00 01 frame magic
                if (LengthInRange(window) && window[20] == 0x00 && window[21] == 0x00 && window[22] == 0x01)
                {
                    // The magic bytes belong to the next payload, so replay them on the next read
                    // (ahead of anything still pushed back from the scan itself)
                    var replay = new byte[3 + _pushback.Length];
                    replay[0] = window[20]; replay[1] = window[21]; replay[2] = window[22];
                    _pushback.CopyTo(replay, 3);
                    _pushback = replay;

                    // Hand back just the re-anchored 20-byte header
                    byte[] resynced = new byte[20];
                    Array.Copy(window, resynced, 20);
                    ResyncCount++;
                    ResyncSkippedBytes += scanned;
                    Console.WriteLine($"[Sofia] resynced to frame after skipping {scanned} bytes");
                    return resynced;
                }

                // Give up after a bounded scan and let the caller reconnect cleanly
                if (scanned >= MaxResyncScanBytes)
                {
                    throw new InvalidDataException($"DVRIP resync failed after {scanned} bytes");
                }

                // Slide the window forward one byte and test the next position
                Array.Copy(window, 1, window, 0, 22);
                window[22] = ReadScanByte();
                scanned++;
            }
        }

        /// <summary>True when a header's length field is within the sane packet-size range.</summary>
        private static bool LengthInRange(byte[] header)
        {
            return BitConverter.ToUInt32(header, 16) <= (uint)MaxPacketBytes;
        }

        /// <summary>Reads one byte for the resync scan, draining pushback first, throwing on close.</summary>
        private byte ReadScanByte()
        {
            // Serve a pushed-back byte before touching the socket
            if (_pushback.Length > 0)
            {
                byte first = _pushback[0];
                _pushback = _pushback.Length == 1 ? Array.Empty<byte>() : _pushback[1..];
                return first;
            }

            // Otherwise pull the next byte from the buffered stream
            var input = _in ?? throw new ObjectDisposedException(nameof(DvripClient));
            int value = input.ReadByte();
            if (value < 0)
            {
                throw new EndOfStreamException("DVRIP socket closed");
            }

            return (byte)value;
        }

        /// <summary>Reads exactly count bytes, throwing if the socket closes first.</summary>
        private byte[] ReadExact(int count)
        {
            // Reject an out-of-range length before allocating; new byte[count] would throw an
            // OverflowException on a negative size
            if (count < 0 || count > MaxPacketBytes)
            {
                throw new InvalidDataException($"DVRIP length out of range: {count}");
            }

            byte[] buffer = new byte[count];
            int offset = 0;

            // Replay any bytes the resync read past the header before reading the socket
            if (_pushback.Length > 0)
            {
                int take = Math.Min(_pushback.Length, count);
                Array.Copy(_pushback, 0, buffer, 0, take);
                _pushback = take == _pushback.Length ? Array.Empty<byte>() : _pushback[take..];
                offset += take;
            }

            // Loop until the buffer is full; a zero read means the peer closed
            var input = _in ?? throw new ObjectDisposedException(nameof(DvripClient));
            while (offset < count)
            {
                int read = input.Read(buffer, offset, count - offset);
                if (read <= 0)
                {
                    throw new EndOfStreamException("DVRIP socket closed");
                }
                offset += read;
            }

            return buffer;
        }

        /// <summary>Formats the session id the way the protocol expects it in JSON.</summary>
        private string SessionHex()
        {
            return "0x" + _session.ToString("X8");
        }

        /// <summary>Computes the Sofia password hash: 8 chars from the MD5 of the password.</summary>
        internal static string SofiaHash(string password)
        {
            // Hash the password and fold byte pairs into the 62-char alphabet
            byte[] md5 = MD5.HashData(Encoding.UTF8.GetBytes(password ?? ""));
            const string chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";

            var hash = new StringBuilder(8);
            for (int i = 0; i < md5.Length; i += 2)
            {
                hash.Append(chars[(md5[i] + md5[i + 1]) % 62]);
            }
            return hash.ToString();
        }

        /// <summary>
        /// Closes the session and releases the socket. Safe to call from any thread, and more
        /// than once: closing the socket unblocks a read in progress on the owning thread,
        /// which then fails with an I/O error.
        /// </summary>
        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            // Stop keepalives first so they cannot fire mid-teardown
            try { _keepAlive?.Dispose(); } catch { }
            // Close the socket first: that is what unblocks a reader on another thread
            try { _client?.Client?.Close(0); } catch { }
            try { _client?.Dispose(); } catch { }
            try { _net?.Dispose(); } catch { }
        }

        /// <summary>One reassembled Sofia frame and whether it is video (vs audio/metadata).</summary>
        private readonly record struct SofiaFrame(byte[] Payload, bool IsVideo, bool IsKeyFrame);
    }
}
