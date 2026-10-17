namespace Controller.RobotControl.Persistence;
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

    // ── Stable identity ───────────────────────────────────────────────────────

    /// <summary>
    /// Give every point a stable id: keep the prior id when re-teaching the same
    /// name (so program-step links survive), otherwise mint a new one. Returns true
    /// when an id was assigned (used to persist a backfill at load).
    /// </summary>
    protected override bool EnsureIdentity(Point item, Point? existing)
    {
        if (existing != null && !string.IsNullOrWhiteSpace(existing.Id))
            item.Id = existing.Id;

        if (string.IsNullOrWhiteSpace(item.Id))
        {
            item.Id = System.Guid.NewGuid().ToString("N");
            return true;
        }
        return false;
    }

    /// <summary>The point with this stable id, or null. Resolves a step's pointId
    /// regardless of the point's current name (rename-proof).</summary>
    public Point? GetById(string? id)
    {
        if (string.IsNullOrWhiteSpace(id)) return null;
        foreach (var p in Items.Values)
            if (p.Id == id) return p;
        return null;
    }

    // ── Backward-compatible method names ─────────────────────────────────────
    public Point SavePoint(string name, Vector6 vector)                           => SaveItem(name, vector);
    public void  EditPoint(string name, System.Collections.Generic.Dictionary<string, object?> values) => EditItem(name, values);
    public void  DeletePoint(string name)                                         => DeleteItem(name);
}
