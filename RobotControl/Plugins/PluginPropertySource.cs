namespace Controller.RobotControl.Plugins;

/// <summary>
/// <see cref="IPropertySource"/> over every running plugin's property table:
/// <c>$scale.weight</c> → plugin <c>scale</c>, property <c>weight</c>. Case-insensitive,
/// lock-free reads. A property that has not been set (or whose plugin disconnected) is
/// unknown. Sits in the chain computed → plugin → robot (wired by the executor).
/// </summary>
public sealed class PluginPropertySource : IPropertySource
{
    /// <summary>Description given to properties a plugin sets without declaring them.</summary>
    public const string Undocumented = "undocumented";

    private readonly PluginManager _manager;

    public PluginPropertySource(PluginManager manager) => _manager = manager;

    public bool TryGet(string name, out double value)
    {
        value = 0;
        if (string.IsNullOrEmpty(name)) return false;
        int dot = name.IndexOf('.');
        if (dot <= 0 || dot == name.Length - 1) return false;
        var host = _manager.Get(name[..dot]);
        if (host is null || !host.Connected) return false;
        return host.Properties.TryGetValue(name[(dot + 1)..], out value);
    }

    public IEnumerable<(string Name, string Description, string Type)> List()
    {
        foreach (var host in _manager.Plugins)
        {
            if (!host.IsRunning || host.Manifest is not { } m) continue;
            var declared = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var p in m.Properties)
            {
                declared.Add(p.Name);
                yield return ($"{host.Id}.{p.Name}", p.Description ?? "", p.Type);
            }
            foreach (var name in host.Properties.Keys.OrderBy(k => k, StringComparer.OrdinalIgnoreCase))
                if (!declared.Contains(name))
                    yield return ($"{host.Id}.{name}", Undocumented, "number");
        }
    }
}
