/// <summary>
/// Persists named TCP-offset tool frames (Tool).
/// All CRUD and history logic lives in NamedVectorRepository&lt;Tool, ToolHistoryEntry&gt;.
/// </summary>
public class ToolRepository : NamedVectorRepository<Tool, ToolHistoryEntry>
{
    public Dictionary<string, Tool> Tools => Items;
    public string? toolsJson              => ItemsJson;

    public ToolRepository(
        string toolsFile   = "tools.json",
        string historyFile = "toolsHistory.json")
        : base(toolsFile, historyFile) { }

    protected override ToolHistoryEntry CreateHistoryEntry(Tool item) =>
        new() { TimestampUnixMs = NowUnixMs(), Tool = item };

    public Tool SaveTool(string name, Vector6 vector)                                                    => SaveItem(name, vector);
    public void EditTool(string name, System.Collections.Generic.Dictionary<string, object?> values)     => EditItem(name, values);
    public void DeleteTool(string name)                                                                  => DeleteItem(name);
}
