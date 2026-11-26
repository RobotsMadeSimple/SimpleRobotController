using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using System.Threading;

namespace Controller.RobotControl.SimplePlc
{
    /// <summary>
    /// One live newline-JSON session over an <see cref="ISimplePlcTransport"/>. A background reader
    /// parses each line: a reply (it carries the request's <c>id</c>) completes the matching pending
    /// request; anything else is a pushed event (§4.5) raised on <see cref="OnEvent"/>. Writes are
    /// serialized. Request() blocks for the reply up to a timeout; Send() is fire-and-forget (used for
    /// <c>ping</c>, which only needs to reach the head to feed its watchdog).
    /// </summary>
    public sealed class SimplePlcConnection : IDisposable
    {
        private readonly ISimplePlcTransport _t;
        private readonly Thread _reader;
        private readonly object _writeLock = new();
        private readonly ConcurrentDictionary<long, TaskCompletionSourceLike> _pending = new();
        private long _nextId;
        private volatile bool _closed;

        /// <summary>A pushed message from the head (no id): in / module / modules / fault / fault_clear.</summary>
        public event Action<JsonElement>? OnEvent;
        /// <summary>Raised once when the stream ends (remote close, error, or Dispose).</summary>
        public event Action? OnClosed;

        public bool Closed => _closed;

        public SimplePlcConnection(ISimplePlcTransport transport)
        {
            _t = transport;
            _reader = new Thread(ReadLoop) { IsBackground = true, Name = "SimplePlcRead" };
            _reader.Start();
        }

        private void ReadLoop()
        {
            try
            {
                while (!_closed)
                {
                    var line = _t.ReadLine();
                    if (line == null) break;           // end of stream
                    if (line.Length == 0) continue;    // blank / over-long line, ignored

                    JsonElement msg;
                    try { using var doc = JsonDocument.Parse(line); msg = doc.RootElement.Clone(); }
                    catch { continue; }                // not JSON — skip

                    if (msg.ValueKind == JsonValueKind.Object
                        && msg.TryGetProperty("id", out var idEl) && TryGetId(idEl, out var id)
                        && _pending.TryRemove(id, out var waiter))
                        waiter.SetResult(msg);
                    else
                        try { OnEvent?.Invoke(msg); } catch { /* handler must not take the reader down */ }
                }
            }
            catch { /* fall through to close */ }
            finally { FailPending(); }
        }

        private void FailPending()
        {
            _closed = true;
            foreach (var kv in _pending)
                kv.Value.SetException(new IOException("SimplePLC connection closed"));
            _pending.Clear();
            try { OnClosed?.Invoke(); } catch { }
        }

        /// <summary>Send a request and wait for its reply. Throws on timeout, write failure, or close.</summary>
        public JsonElement Request(string op, IReadOnlyDictionary<string, object?>? fields, int timeoutMs)
        {
            if (_closed) throw new IOException("SimplePLC connection closed");
            long id = Interlocked.Increment(ref _nextId);
            var waiter = new TaskCompletionSourceLike();
            _pending[id] = waiter;
            try { lock (_writeLock) _t.WriteLine(Build(op, id, fields)); }
            catch (Exception ex) { _pending.TryRemove(id, out _); throw new IOException("SimplePLC write failed", ex); }

            if (!waiter.Wait(timeoutMs)) { _pending.TryRemove(id, out _); throw new TimeoutException($"SimplePLC: no reply to '{op}'"); }
            return waiter.Result;
        }

        /// <summary>Send a request without waiting for its reply (still feeds the head watchdog).</summary>
        public void Send(string op, IReadOnlyDictionary<string, object?>? fields = null)
        {
            if (_closed) throw new IOException("SimplePLC connection closed");
            long id = Interlocked.Increment(ref _nextId);
            lock (_writeLock) _t.WriteLine(Build(op, id, fields));
        }

        private static string Build(string op, long id, IReadOnlyDictionary<string, object?>? fields)
        {
            var obj = new Dictionary<string, object?> { ["op"] = op, ["id"] = id };
            if (fields != null) foreach (var kv in fields) obj[kv.Key] = kv.Value;
            return SimplePlcProtocol.Encode(obj);
        }

        private static bool TryGetId(JsonElement idEl, out long id)
        {
            id = 0;
            if (idEl.ValueKind == JsonValueKind.Number) return idEl.TryGetInt64(out id);
            if (idEl.ValueKind == JsonValueKind.String) return long.TryParse(idEl.GetString(), out id);
            return false;
        }

        public void Dispose()
        {
            _closed = true;
            try { _t.Dispose(); } catch { }   // unblocks the reader's ReadLine
        }

        /// <summary>A tiny manual-reset reply slot (avoids pulling Task machinery into the hot path).</summary>
        private sealed class TaskCompletionSourceLike
        {
            private readonly ManualResetEventSlim _done = new(false);
            private JsonElement _result;
            private Exception? _error;

            public void SetResult(JsonElement value) { _result = value; _done.Set(); }
            public void SetException(Exception ex) { _error ??= ex; _done.Set(); }
            public bool Wait(int timeoutMs) => _done.Wait(timeoutMs);
            public JsonElement Result => _error != null ? throw _error : _result;
        }
    }
}
