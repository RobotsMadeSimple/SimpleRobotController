using Controller.RobotControl;
using System.Text.Json.Serialization;
using Controller.RobotControl.Persistence;

public class PersistenceTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "rms-tests-" + Guid.NewGuid().ToString("N"));

    public PersistenceTests() => Directory.CreateDirectory(_dir);
    public void Dispose() { try { Directory.Delete(_dir, recursive: true); } catch { } }

    private string P(string name) => Path.Combine(_dir, name);

    private class Item : IStoredItem
    {
        [JsonPropertyName("id")] public string Id { get; set; } = "";
        [JsonPropertyName("name")] public string Name { get; set; } = "";
        [JsonPropertyName("lastUpdatedUnixMs")] public long LastUpdatedUnixMs { get; set; }
    }

    [Fact]
    public void AtomicFile_WritesContentAndLeavesNoTempFile()
    {
        var path = P("a.json");
        AtomicFile.WriteAllText(path, "{\"x\":1}");
        Assert.Equal("{\"x\":1}", File.ReadAllText(path));
        Assert.False(File.Exists(path + ".tmp"));

        AtomicFile.WriteAllText(path, "{\"x\":2}");
        Assert.Equal("{\"x\":2}", File.ReadAllText(path));
    }

    [Fact]
    public void JsonFiles_CorruptFileIsQuarantinedNotOverwritten()
    {
        var path = P("bad.json");
        File.WriteAllText(path, "{ this is not json");
        var loaded = JsonFiles.Load<List<Item>>(path);
        Assert.Null(loaded);
        Assert.False(File.Exists(path));
        Assert.Single(Directory.GetFiles(_dir, "bad.json.corrupt-*"));
    }

    [Fact]
    public void JsonListRepository_UpsertAssignsIdAndPersists()
    {
        var file = P("items.json");
        var repo = new JsonListRepository<Item>(file);
        var saved = repo.Upsert(new Item { Name = "one" });
        Assert.False(string.IsNullOrEmpty(saved.Id));
        Assert.True(saved.LastUpdatedUnixMs > 0);
        Assert.True(repo.LastUpdatedUnixMs > 0);

        var reloaded = new JsonListRepository<Item>(file);
        Assert.Equal("one", reloaded.Get(saved.Id)!.Name);
        Assert.Single(reloaded.GetAll());
    }

    [Fact]
    public void JsonListRepository_DeleteRemovesAndReturnsFalseWhenMissing()
    {
        var repo = new JsonListRepository<Item>(P("items.json"));
        var saved = repo.Upsert(new Item { Name = "gone" });
        Assert.True(repo.Delete(saved.Id));
        Assert.False(repo.Delete(saved.Id));
        Assert.Null(repo.Get(saved.Id));
    }

    [Fact]
    public void GridAndStackRepositories_RoundTrip()
    {
        var grids = new GridRepository(P("grids.json"));
        var g = grids.Upsert(new Grid { Name = "g", RowCount = 2, ColCount = 3 });
        Assert.Equal(2, new GridRepository(P("grids.json")).Get(g.Id)!.RowCount);

        var stacks = new StackRepository(P("stacks.json"));
        var s = stacks.Upsert(new RobotStack { Name = "s", MaxCount = 4 });
        Assert.Equal(4, new StackRepository(P("stacks.json")).Get(s.Id)!.MaxCount);
    }
}
