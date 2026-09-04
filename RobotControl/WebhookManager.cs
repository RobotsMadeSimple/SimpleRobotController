using System;
using System.Collections.Generic;
using System.Text.Json;
using System.Threading;
using System.Threading.Channels;

public class WebhookManager
{
    private readonly object _lock = new();
    private readonly Dictionary<string, List<(Guid Id, Channel<Dictionary<string, JsonElement>> Chan)>> _subs
        = new(StringComparer.OrdinalIgnoreCase);

    public Guid Subscribe(string name, out Channel<Dictionary<string, JsonElement>> channel)
    {
        var id   = Guid.NewGuid();
        var chan  = Channel.CreateBounded<Dictionary<string, JsonElement>>(
            new BoundedChannelOptions(1) { FullMode = BoundedChannelFullMode.DropOldest, SingleReader = true });
        channel = chan;
        lock (_lock)
        {
            if (!_subs.TryGetValue(name, out var list))
            {
                list = new();
                _subs[name] = list;
            }
            list.Add((id, chan));
        }
        return id;
    }

    public void Unsubscribe(string name, Guid id)
    {
        lock (_lock)
        {
            if (!_subs.TryGetValue(name, out var list)) return;
            list.RemoveAll(e => e.Id == id);
        }
    }

    public void Deliver(string name, Dictionary<string, JsonElement> payload)
    {
        List<(Guid Id, Channel<Dictionary<string, JsonElement>> Chan)>? copy;
        lock (_lock)
        {
            if (!_subs.TryGetValue(name, out var list) || list.Count == 0) return;
            copy = new(list);
        }
        foreach (var (_, chan) in copy)
            chan.Writer.TryWrite(payload);
    }
}
