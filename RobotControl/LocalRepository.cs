/// <summary>
/// Persists named work-frame coordinate systems (Local).
/// All CRUD and history logic lives in NamedVectorRepository&lt;Local, LocalHistoryEntry&gt;.
/// </summary>
public class LocalRepository : NamedVectorRepository<Local, LocalHistoryEntry>
{
    public Dictionary<string, Local> Locals => Items;
    public string? localsJson               => ItemsJson;

    public LocalRepository(
        string localsFile   = "locals.json",
        string historyFile  = "localsHistory.json")
        : base(localsFile, historyFile) { }

    protected override LocalHistoryEntry CreateHistoryEntry(Local item) =>
        new() { TimestampUnixMs = NowUnixMs(), Local = item };

    public Local SaveLocal(string name, Vector6 vector)                                                     => SaveItem(name, vector);
    public void EditLocal(string name, System.Collections.Generic.Dictionary<string, object?> values)       => EditItem(name, values);
    public void DeleteLocal(string name)                                                                     => DeleteItem(name);
}
