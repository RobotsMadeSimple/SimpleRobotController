namespace Controller.RobotControl.Persistence;

/// <summary>
/// Read side of a point source for move-target resolution: by stable id (preferred) or by name.
/// Implemented by the global <see cref="PointRepository"/> and by the per-program composite the
/// executor builds, so the resolver treats "this program's points" and the global library the same.
/// </summary>
public interface IPointLookup
{
    Point? Get(string name);
    Point? GetById(string? id);
}

/// <summary>
/// Persists named robot positions (Point).
/// All CRUD and history logic lives in NamedVectorRepository&lt;Point, PointHistoryEntry&gt;.
/// Backward-compatible aliases are kept so RobotController.cs needs no edits.
/// </summary>
public class PointRepository : NamedVectorRepository<Point, PointHistoryEntry>, IPointLookup
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

    // Stable identity (EnsureIdentity) and GetById now live in NamedVectorRepository —
    // shared by Point, Tool and Local.

    // ── Backward-compatible method names ─────────────────────────────────────
    public Point SavePoint(string name, Vector6 vector)                           => SaveItem(name, vector);
    public void  EditPoint(string name, System.Collections.Generic.Dictionary<string, object?> values) => EditItem(name, values);
    public void  DeletePoint(string name)                                         => DeleteItem(name);
}
