using Controller.RobotControl;
using Controller.RobotControl.Persistence;

namespace RobotControl.Tests;

/// <summary>Point descriptions: editable, and preserved across a re-teach so moving a point
/// never wipes the note describing what it is for.</summary>
public class PointDescriptionTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "rms-ptdesc-" + Guid.NewGuid().ToString("N"));

    public PointDescriptionTests() => Directory.CreateDirectory(_dir);
    public void Dispose() { try { Directory.Delete(_dir, recursive: true); } catch { } }

    private PointRepository NewRepo() =>
        new(Path.Combine(_dir, "points.json"), Path.Combine(_dir, "pointsHistory.json"));

    [Fact]
    public void EditPoint_SetsDescription_AndReTeachKeepsIt()
    {
        var repo = NewRepo();
        var p = repo.SavePoint("pickup", new Vector6(100, 200, 50, 0, 0, 90));
        Assert.Equal("", p.Description);
        var id = p.Id;

        // Edit sets the description.
        repo.EditPoint("pickup", new Dictionary<string, object?> { ["Description"] = "part in-feed" });
        Assert.Equal("part in-feed", repo.Get("pickup")!.Description);

        // Re-teaching the same name replaces the pose but keeps the id AND the description.
        var reTaught = repo.SavePoint("pickup", new Vector6(1, 2, 3, 0, 0, 0));
        Assert.Equal(id, reTaught.Id);
        Assert.Equal("part in-feed", reTaught.Description);
        Assert.Equal(1, reTaught.X);
    }
}
