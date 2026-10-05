using Controller.RobotControl;
using Controller.RobotControl.Persistence;

namespace RobotControl.Tests;

public class BuiltProgramRevisionTests : IDisposable
{
    private readonly string _dir = Path.Combine(Path.GetTempPath(), "rms-tests-" + Guid.NewGuid().ToString("N"));

    public BuiltProgramRevisionTests() => Directory.CreateDirectory(_dir);
    public void Dispose() { try { Directory.Delete(_dir, recursive: true); } catch { } }

    private static BuiltProgram MakeProgram(string name, int stepCount, string? id = null) => new()
    {
        Id    = id ?? Guid.NewGuid().ToString(),
        Name  = name,
        Steps = Enumerable.Range(0, stepCount)
            .Select(i => new ProgramStep { Id = "s" + i, Type = StepType.Wait, WaitMs = i })
            .ToList(),
    };

    private string RevisionDirFor(string name) =>
        Path.Combine(_dir, "builtPrograms", ".revisions", name);

    [Fact]
    public void Save_UnchangedContent_CreatesNoRevision()
    {
        var repo = new BuiltProgramRepository(_dir);
        var p = MakeProgram("Prog", 2);
        repo.Save(p);

        // Save again with identical steps/name/description — only LastUpdatedUnixMs
        // differs internally, but that isn't part of what the app sends back in, so
        // re-saving the same logical content must not create a revision.
        var same = MakeProgram("Prog", 2, p.Id);
        same.Steps = p.Steps;
        repo.Save(same);

        Assert.False(Directory.Exists(RevisionDirFor("Prog")));
        Assert.Empty(repo.ListRevisions("Prog"));
    }

    [Fact]
    public void Save_ChangedContent_ArchivesPreviousVersionAsRevision()
    {
        var repo = new BuiltProgramRepository(_dir);
        var p = MakeProgram("Prog", 2);
        repo.Save(p);

        var changed = MakeProgram("Prog", 3, p.Id);
        repo.Save(changed);

        var revisions = repo.ListRevisions("Prog");
        Assert.Single(revisions);
        Assert.Equal(2, revisions[0].StepCount); // the archived (pre-change) content had 2 steps
    }

    [Fact]
    public void Save_MoreThan30ChangedSaves_KeepsOnlyNewest30Revisions()
    {
        var repo = new BuiltProgramRepository(_dir);
        var id = Guid.NewGuid().ToString();
        repo.Save(MakeProgram("Prog", 0, id));

        for (var i = 1; i <= 35; i++)
            repo.Save(MakeProgram("Prog", i, id));

        var revisions = repo.ListRevisions("Prog");
        Assert.Equal(30, revisions.Count);
        // Newest-first: the most recently archived revision was the pre-overwrite
        // content of the last save, i.e. the program with 34 steps.
        Assert.Equal(34, revisions[0].StepCount);
        // Oldest kept revision is the 6th archived one (content with 5 steps),
        // since revisions from the 0..4-step saves were trimmed away.
        Assert.Equal(5, revisions[^1].StepCount);
    }

    [Fact]
    public void ListRevisions_NewestFirst()
    {
        var repo = new BuiltProgramRepository(_dir);
        var id = Guid.NewGuid().ToString();
        repo.Save(MakeProgram("Prog", 0, id));
        repo.Save(MakeProgram("Prog", 1, id));
        repo.Save(MakeProgram("Prog", 2, id));

        var revisions = repo.ListRevisions("Prog");
        Assert.Equal(2, revisions.Count);
        Assert.True(revisions[0].SavedUnixMs >= revisions[1].SavedUnixMs);
    }

    [Fact]
    public void Rename_MovesRevisionFolder()
    {
        var repo = new BuiltProgramRepository(_dir);
        var id = Guid.NewGuid().ToString();
        repo.Save(MakeProgram("Old", 0, id));
        repo.Save(MakeProgram("Old", 1, id)); // creates a revision under Old

        Assert.True(Directory.Exists(RevisionDirFor("Old")));

        var renamed = MakeProgram("New", 1, id);
        repo.Save(renamed);

        Assert.False(Directory.Exists(RevisionDirFor("Old")));
        Assert.True(Directory.Exists(RevisionDirFor("New")));
        Assert.Single(repo.ListRevisions("New"));
        Assert.Empty(repo.ListRevisions("Old"));
    }

    [Fact]
    public void Delete_RemovesRevisionFolder()
    {
        var repo = new BuiltProgramRepository(_dir);
        var id = Guid.NewGuid().ToString();
        repo.Save(MakeProgram("Prog", 0, id));
        repo.Save(MakeProgram("Prog", 1, id));
        Assert.True(Directory.Exists(RevisionDirFor("Prog")));

        repo.Delete("Prog");

        Assert.False(Directory.Exists(RevisionDirFor("Prog")));
    }

    [Fact]
    public void RestoreRevision_KeepsCurrentNameAndId_AndArchivesPreRestoreState()
    {
        var repo = new BuiltProgramRepository(_dir);
        var id = Guid.NewGuid().ToString();
        repo.Save(MakeProgram("Prog", 2, id));   // v1: 2 steps
        repo.Save(MakeProgram("Prog", 5, id));   // v2: 5 steps (archives v1 as a revision)

        var revisions = repo.ListRevisions("Prog");
        Assert.Single(revisions);
        var v1Id = revisions[0].Id;

        var restored = repo.RestoreRevision("Prog", v1Id);
        Assert.NotNull(restored);
        Assert.Equal(2, restored!.Steps.Count);
        Assert.Equal("Prog", restored.Name);
        Assert.Equal(id, restored.Id);

        // The pre-restore state (v2, 5 steps) must itself now be archived as a revision.
        var afterRestore = repo.ListRevisions("Prog");
        Assert.Equal(2, afterRestore.Count);
        Assert.Contains(afterRestore, r => r.StepCount == 5);
        Assert.Contains(afterRestore, r => r.StepCount == 2);

        // Current on-disk program reflects the restored content.
        var current = repo.Get("Prog");
        Assert.NotNull(current);
        Assert.Equal(2, current!.Steps.Count);
    }

    [Fact]
    public void RestoreRevision_KeepsCurrentNameEvenIfRevisionPredatesRename()
    {
        var repo = new BuiltProgramRepository(_dir);
        var id = Guid.NewGuid().ToString();
        repo.Save(MakeProgram("Old", 2, id));
        repo.Save(MakeProgram("Old", 3, id)); // archives the 2-step revision under Old, then rename below moves it

        var revisions = repo.ListRevisions("Old");
        var revId = revisions[0].Id;

        repo.Save(MakeProgram("New", 3, id)); // rename — same content, moves revision folder to New

        var restored = repo.RestoreRevision("New", revId);
        Assert.NotNull(restored);
        Assert.Equal("New", restored!.Name);
        Assert.Equal(id, restored.Id);
    }

    [Fact]
    public void GetRevision_UnknownIdOrProgram_ReturnsNull()
    {
        var repo = new BuiltProgramRepository(_dir);
        repo.Save(MakeProgram("Prog", 1));

        Assert.Null(repo.GetRevision("Prog", "999999999999"));
        Assert.Null(repo.GetRevision("NoSuchProgram", "1"));
        Assert.Null(repo.RestoreRevision("Prog", "999999999999"));
        Assert.Null(repo.RestoreRevision("NoSuchProgram", "1"));
    }
}
