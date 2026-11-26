using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;

namespace Controller.RobotControl.SimplePlc
{
    // The seam that lets the device logic be tested without a real head: a line-oriented transport
    // plus the factory that opens one, and a discovery probe. The production implementations below use
    // real TCP/UDP sockets; the tests provide a fake head that speaks the same JSON lines.

    /// <summary>A connected newline-JSON session to one head. Reads block until a line, EOF, or Dispose.</summary>
    public interface ISimplePlcTransport : IDisposable
    {
        /// <summary>Next line (without the terminator), or null at end of stream.</summary>
        string? ReadLine();
        /// <summary>Send one JSON object as a line (the terminator is added).</summary>
        void WriteLine(string json);
    }

    /// <summary>Opens a transport to a head, or throws if it cannot connect in time.</summary>
    public interface ISimplePlcTransportFactory
    {
        ISimplePlcTransport Connect(string host, int port, int timeoutMs);
    }

    /// <summary>Probes the network for heads (UDP discovery, §4.1).</summary>
    public interface ISimplePlcDiscovery
    {
        IReadOnlyList<SimplePlcDiscovered> Discover(int timeoutMs);
    }

    // ── Real TCP transport ──────────────────────────────────────────────────────

    public sealed class TcpSimplePlcTransport : ISimplePlcTransport
    {
        private readonly TcpClient _client;
        private readonly NetworkStream _stream;
        private readonly byte[] _buf = new byte[SimplePlcProtocol.MaxLineBytes + 1];

        private TcpSimplePlcTransport(TcpClient client)
        {
            _client = client;
            _stream = client.GetStream();
        }

        public static TcpSimplePlcTransport Connect(string host, int port, int timeoutMs)
        {
            var client = new TcpClient { NoDelay = true };
            try
            {
                if (!client.ConnectAsync(host, port).Wait(timeoutMs))
                    throw new TimeoutException($"connect to {host}:{port} timed out");
            }
            catch
            {
                client.Dispose();
                throw;
            }
            return new TcpSimplePlcTransport(client);
        }

        /// <summary>Reads one '\n'-terminated line as UTF-8, capped at the protocol's max line length.</summary>
        public string? ReadLine()
        {
            int n = 0;
            while (n <= SimplePlcProtocol.MaxLineBytes)
            {
                int b;
                try { b = _stream.ReadByte(); }
                catch { return null; }   // socket closed / disposed
                if (b < 0) return n > 0 ? Encoding.UTF8.GetString(_buf, 0, n) : null;
                if (b == '\n') return Encoding.UTF8.GetString(_buf, 0, n);
                if (b == '\r') continue;
                _buf[n++] = (byte)b;
            }
            // Over-long line: drain to the next newline and report it as empty (ignored by the caller).
            int d;
            do { try { d = _stream.ReadByte(); } catch { return null; } } while (d >= 0 && d != '\n');
            return "";
        }

        public void WriteLine(string json)
        {
            var bytes = Encoding.UTF8.GetBytes(json + "\n");
            _stream.Write(bytes, 0, bytes.Length);
            _stream.Flush();
        }

        public void Dispose()
        {
            try { _stream.Dispose(); } catch { }
            try { _client.Dispose(); } catch { }
        }
    }

    public sealed class TcpSimplePlcTransportFactory : ISimplePlcTransportFactory
    {
        public ISimplePlcTransport Connect(string host, int port, int timeoutMs) =>
            TcpSimplePlcTransport.Connect(host, port, timeoutMs);
    }

    // ── Real UDP discovery ──────────────────────────────────────────────────────

    public sealed class UdpSimplePlcDiscovery : ISimplePlcDiscovery
    {
        public IReadOnlyList<SimplePlcDiscovered> Discover(int timeoutMs)
        {
            var found = new Dictionary<string, SimplePlcDiscovered>(StringComparer.OrdinalIgnoreCase);
            using var udp = new UdpClient { EnableBroadcast = true };
            udp.Client.ReceiveTimeout = Math.Max(50, Math.Min(timeoutMs, 1000));

            var probe = Encoding.UTF8.GetBytes(SimplePlcProtocol.Encode(new { op = "discover", proto = SimplePlcProtocol.ProtoVersion }));
            var targets = new[]
            {
                new IPEndPoint(IPAddress.Broadcast, SimplePlcProtocol.DiscoveryPort),
                new IPEndPoint(IPAddress.Loopback,  SimplePlcProtocol.DiscoveryPort), // local simulator
            };
            foreach (var t in targets) { try { udp.Send(probe, probe.Length, t); } catch { } }

            var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
            while (DateTime.UtcNow < deadline)
            {
                IPEndPoint remote = new(IPAddress.Any, 0);
                byte[] data;
                try { data = udp.Receive(ref remote); }
                catch (SocketException) { continue; }   // receive timeout — keep waiting until the deadline
                catch { break; }

                SimplePlcDiscovered? head = Parse(data, remote);
                if (head != null) found[head.Mac.Length > 0 ? head.Mac : head.Host] = head;
            }
            return new List<SimplePlcDiscovered>(found.Values);
        }

        private static SimplePlcDiscovered? Parse(byte[] data, IPEndPoint remote)
        {
            try
            {
                using var doc = JsonDocument.Parse(data);
                var m = doc.RootElement;
                if (SimplePlcProtocol.Op(m) != "here") return null;
                string Str(string k) => m.TryGetProperty(k, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() ?? "" : "";
                int Num(string k, int d) => m.TryGetProperty(k, out var v) && v.ValueKind == JsonValueKind.Number ? v.GetInt32() : d;
                // Prefer the sender's real address; the head's self-reported ip may be a static fallback.
                var host = remote.Address.ToString();
                if (host == "0.0.0.0" || string.IsNullOrEmpty(host)) host = Str("ip");
                return new SimplePlcDiscovered
                {
                    Name     = Str("name"),
                    Mac      = Str("mac"),
                    Host     = host,
                    Port     = Num("port", SimplePlcProtocol.DefaultTcpPort),
                    Firmware = Str("fw"),
                    Modules  = Num("modules", 0),
                };
            }
            catch { return null; }
        }
    }
}
