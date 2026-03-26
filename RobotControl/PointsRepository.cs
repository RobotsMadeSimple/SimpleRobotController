/// <summary>
/// Persists named robot positions (Point).
/// All CRUD and history logic lives in NamedVectorRepository&lt;Point, PointHistoryEntry&gt;.
/// Backward-compatible aliases are kept so RobotController.cs needs no edits.
/// </summary>
public class PointRepository : NamedVectorRepository<Point, PointHistoryEntry>
{
    // ── Backward-compatible aliases ───────────────────────────────────────────
    public Dictionary<string, Point> Points => Items;
    public string? pointsJson               => ItemsJson;

    public PointRepository(
        string pointsFile  = "points.json",
        string historyFile = "pointsHistory.json")
        : base(pointsFile, historyFile) { }

    protected override PointHistoryEntry CreateHistoryEntry(Point item) =>
        new() { TimestampUnixMs = NowUnixMs(), Point = item };

    // ── Backward-compatible method names ─────────────────────────────────────
    public Point SavePoint(string name, Vector6 vector)                           => SaveItem(name, vector);
    public void  EditPoint(string name, System.Collections.Generic.Dictionary<string, object?> values) => EditItem(name, values);
    public void  DeletePoint(string name)                                         => DeleteItem(name);
}
