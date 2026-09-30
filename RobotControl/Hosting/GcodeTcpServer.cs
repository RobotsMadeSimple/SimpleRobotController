using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using Controller.RobotControl.Gcode;

namespace Controller.RobotControl.Hosting
{
    /// <summary>
    /// A GRBL-style raw-TCP G-code stream: point any sender (UGS, Candle, bCNC, netcat, a script)
    /// at this port, send one line per move, get "ok" / "error:…" back. Realtime single bytes are
    /// honoured anywhere in the stream: '?' status, '!' or 0x18 stop. One active session at a time
    /// (enforced by <see cref="GcodeStreamSession"/>); a second connection is told and closed.
    /// </summary>
    internal sealed class GcodeTcpServer
    {
        private readonly RobotController _robot;
        private readonly int _port;
        private TcpListener? _listener;
        private readonly CancellationTokenSource _cts = new();

        public GcodeTcpServer(RobotController robot, int port)
        {
            _robot = robot;
            _port  = port;
        }

        public void Start()
        {
            try
            {
                _listener = new TcpListener(IPAddress.Any, _port);
                _listener.Start();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[Gcode] TCP listener failed on :{_port} — {ex.Message}");
                return;
            }
            Console.WriteLine($"[Gcode] TCP stream listening on :{_port}");
            new Thread(AcceptLoop) { IsBackground = true, Name = "GcodeTcp" }.Start();
        }

        public void Stop()
        {
            _cts.Cancel();
            try { _listener?.Stop(); } catch { }
        }

        private void AcceptLoop()
        {
            while (!_cts.IsCancellationRequested)
            {
                TcpClient client;
                try { client = _listener!.AcceptTcpClient(); }
                catch { return; } // listener stopped
                new Thread(() => Serve(client)) { IsBackground = true }.Start();
            }
        }

        private void Serve(TcpClient client)
        {
            using (client)
            using (var stream = client.GetStream())
            {
                void Write(string s)
                {
                    var b = Encoding.ASCII.GetBytes(s + "\r\n");
                    try { stream.Write(b, 0, b.Length); } catch { }
                }

                if (!GcodeStreamSession.TryCreate(_robot, out var session, out var err))
                {
                    Write("error:" + err);
                    return;
                }

                using (var s = session!)
                using (var linkCts = CancellationTokenSource.CreateLinkedTokenSource(_cts.Token))
                {
                    Write(GcodeStreamSession.Banner);
                    var buffer  = new byte[2048];
                    var pending = new StringBuilder();
                    try
                    {
                        int n;
                        while ((n = stream.Read(buffer, 0, buffer.Length)) > 0)
                        {
                            for (int i = 0; i < n; i++)
                            {
                                char c = (char)buffer[i];
                                if (c == '?')                    { Write(s.Status()); continue; }
                                if (c == '!' || c == '\x18')     { s.Stop(); Write("ok"); continue; }
                                if (c == '\n')                   { Write(s.Feed(pending.ToString(), linkCts.Token)); pending.Clear(); }
                                else if (c != '\r')              pending.Append(c);
                            }
                        }
                    }
                    catch { /* client disconnected or reset */ }
                    finally { linkCts.Cancel(); }
                }
            }
        }
    }
}
