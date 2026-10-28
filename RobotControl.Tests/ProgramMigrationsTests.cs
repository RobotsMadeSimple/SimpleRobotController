using Controller.RobotControl;
using Controller.RobotControl.Io;
using Controller.RobotControl.Persistence;
using System.Text.Json;
using System.Text.Json.Serialization;
using Xunit;

namespace RobotControl.Tests;

/// <summary>
/// "Move with Force" no longer exists in the editor: a saved one becomes a MoveL from the
/// current position with a stop condition on the load cell, and one that cannot convert is
/// left for its legacy handler.
/// </summary>
public class ProgramMigrationsTests
{
    private static readonly ProgramMigrations.LoadCellResolver Cells = ProgramMigrations.Resolver(new[]
    {
        new IoDeviceState
        {
            Id = "nano-1", Name = "Arduino Nano",
            Pins = new List<IoPinState>
            {
                new() { Pin = 4, Name = "Load Cell", Type = PinType.LoadCell },
                new() { Pin = 7, Name = "",          Type = PinType.LoadCell },
                new() { Pin = 9, Name = "Button",    Type = PinType.Input },
            },
        },
    });

    private static ProgramStep ForceStep(Action<ProgramStep>? mutate = null)
    {
        var s = new ProgramStep
        {
            Id = "force-1", Type = StepType.MoveWithForce, Name = "Press down", Comment = "note",
            LoadCellDeviceId = "nano-1", LoadCellPin = 4,
            ForceAxis = "Z", ForceDirection = -1, ForceTargetGrams = 2000,
            ForceMaxTravelMm = 12.5, ForceApproachSpeed = 4, ForceToleranceGrams = 20, ForceTimeoutMs = 15000,
        };
        mutate?.Invoke(s);
        return s;
    }

    [Fact]
    public void ForceMove_BecomesMoveLWithStopCondition()
    {
        var s = ForceStep();
        var notes = new List<string>();

        Assert.True(ProgramMigrations.ForceMoveToMoveL(s, Cells, notes));

        Assert.Equal(StepType.MoveL, s.Type);
        Assert.Equal("force-1", s.Id);
        Assert.Equal("Press down", s.Name);
        Assert.Equal("note", s.Comment);
        Assert.Null(s.PointName);
        Assert.Equal(-12.5, s.OffsetZ);
        Assert.Null(s.OffsetX);
        Assert.Equal(4, s.Speed);
        Assert.Equal("stop", s.MoveStopMode);
        var item = Assert.Single(s.MoveStopCondition!.Items);
        Assert.Equal("$io.Arduino_Nano.Load_Cell >= 2000", item.Expr);
        Assert.False(string.IsNullOrEmpty(item.Id));
        // The force and load-cell fields are gone so nothing re-reads them as a force move.
        Assert.Null(s.ForceTargetGrams); Assert.Null(s.ForceAxis); Assert.Null(s.LoadCellDeviceId); Assert.Null(s.LoadCellPin);
        Assert.Empty(notes);
    }

    [Fact]
    public void ForceMove_AxisDirectionAndDefaults()
    {
        var s = ForceStep(x => { x.ForceAxis = "x"; x.ForceDirection = 1; x.ForceMaxTravelMm = null; x.ForceApproachSpeed = null; });
        Assert.True(ProgramMigrations.ForceMoveToMoveL(s, Cells, new List<string>()));
        Assert.Equal(10, s.OffsetX);   // default max travel, pushing +X
        Assert.Null(s.OffsetZ);
        Assert.Equal(5, s.Speed);      // default approach speed
    }

    [Fact]
    public void ForceMove_HoldIsDroppedAndNoted()
    {
        var s = ForceStep(x => x.ForceHoldMs = 1500);
        var notes = new List<string>();
        Assert.True(ProgramMigrations.ForceMoveToMoveL(s, Cells, notes));
        Assert.Equal(StepType.MoveL, s.Type);
        var note = Assert.Single(notes);
        Assert.Contains("1500 ms", note);
        Assert.Contains("Press down", note);
    }

    [Fact]
    public void ForceMove_UnnamedCellOrNoTarget_IsLeftAlone()
    {
        var unnamed = ForceStep(x => x.LoadCellPin = 7);
        var noTarget = ForceStep(x => x.ForceTargetGrams = 0);
        var notCell  = ForceStep(x => x.LoadCellPin = 9);
        var notes = new List<string>();

        Assert.False(ProgramMigrations.ForceMoveToMoveL(unnamed, Cells, notes));
        Assert.False(ProgramMigrations.ForceMoveToMoveL(noTarget, Cells, notes));
        Assert.False(ProgramMigrations.ForceMoveToMoveL(notCell, Cells, notes));

        Assert.All(new[] { unnamed, noTarget, notCell }, s => Assert.Equal(StepType.MoveWithForce, s.Type));
        Assert.Equal(2000, unnamed.ForceTargetGrams);
        Assert.Equal(3, notes.Count);
        Assert.Contains(notes, n => n.Contains("no target force"));
    }

    [Fact]
    public void MigrateSteps_RecursesIntoBodiesAndCountsOnlyChanges()
    {
        var steps = new List<ProgramStep>
        {
            new() { Id = "w", Type = StepType.Wait, WaitMs = 10 },
            new() { Id = "loop", Type = StepType.Loop, LoopSteps = new List<ProgramStep> { ForceStep(x => x.Id = "in-loop") } },
            new()
            {
                Id = "if", Type = StepType.IfCondition,
                IfSteps = new List<ProgramStep> { ForceStep(x => x.Id = "in-if") },
                ElseIfBranches = new List<ElseIfBranch> { new() { Id = "elif", Steps = new List<ProgramStep> { ForceStep(x => x.Id = "in-elif") } } },
                ElseSteps = new List<ProgramStep> { ForceStep(x => { x.Id = "kept"; x.ForceTargetGrams = null; }) },
            },
        };
        var notes = new List<string>();

        Assert.Equal(3, ProgramMigrations.MigrateSteps(steps, Cells, notes));

        Assert.Equal(StepType.MoveL, steps[1].LoopSteps![0].Type);
        Assert.Equal(StepType.MoveL, steps[2].IfSteps![0].Type);
        Assert.Equal(StepType.MoveL, steps[2].ElseIfBranches![0].Steps[0].Type);
        Assert.Equal(StepType.MoveWithForce, steps[2].ElseSteps![0].Type);
        Assert.Single(notes);
        // Nothing left to do on a second pass.
        Assert.Equal(0, ProgramMigrations.MigrateSteps(steps, Cells, new List<string>()));
    }

    [Fact]
    public void Repository_MigratesStoredProgramsOnceAndArchivesTheOldContent()
    {
        var dir = Path.Combine(Path.GetTempPath(), "rms-migrate-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(Path.Combine(dir, "builtPrograms"));
        var opts = new JsonSerializerOptions { Converters = { new JsonStringEnumConverter() } };
        var program = new BuiltProgram { Id = "p1", Name = "Old Press", Steps = new List<ProgramStep> { ForceStep() } };
        File.WriteAllText(Path.Combine(dir, "builtPrograms", "Old Press.json"), JsonSerializer.Serialize(program, opts));
        try
        {
            var repo = new BuiltProgramRepository(dir);
            var notes = repo.MigrateLegacySteps(Cells);

            Assert.Empty(notes);
            var loaded = repo.Get("Old Press")!;
            Assert.Equal(StepType.MoveL, loaded.Steps[0].Type);
            Assert.Equal("$io.Arduino_Nano.Load_Cell >= 2000", loaded.Steps[0].MoveStopCondition!.Items[0].Expr);
            // Persisted in the new shape, with the pre-migration file kept as a revision.
            Assert.Contains("\"MoveL\"", File.ReadAllText(Path.Combine(dir, "builtPrograms", "Old Press.json")));
            Assert.Single(repo.ListRevisions("Old Press"));

            // A fresh repository over the same folder finds nothing left to migrate.
            var again = new BuiltProgramRepository(dir);
            Assert.Empty(again.MigrateLegacySteps(Cells));
            Assert.Single(again.ListRevisions("Old Press"));
        }
        finally
        {
            try { Directory.Delete(dir, recursive: true); } catch { }
        }
    }
}
