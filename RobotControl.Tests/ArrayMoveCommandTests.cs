using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Commands;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Persistence;
using Controller.RobotControl.Robots.ASTRO;
using Controller.RobotControl.Safety;

namespace RobotControl.Tests;

/// <summary>
/// The WebSocket MoveL / MoveJ grid-cell and stack-slot targets ({ grid, row, col } / { stack, index }):
/// resolved by stable id exactly as a program's grid / stack move (MoveTargetResolver + the active
/// local), queued as a raw world-frame vector, reach-checked (outOfReach) and refused by the e-stop.
/// </summary>
[Collection("RobotController")]
public sealed class ArrayMoveCommandTests : IDisposable
{
    private readonly string _tmp = Path.Combine(Path.GetTempPath(), "rms-arraymove-" + Guid.NewGuid().ToString("N"));
    private readonly RobotController _robot;

    public ArrayMoveCommandTests() : this(identity: true) { }

    private ArrayMoveCommandTests(bool identity)
    {
        Directory.CreateDirectory(_tmp);
        _robot = identity ? new RobotController().WithIdentityKinematics() : new RobotController();
        _robot.pointRepo = new PointRepository(P("points.json"), P("pointsHistory.json"));
        _robot.localRepo = new LocalRepository(P("locals.json"), P("localsHistory.json"));
        _robot.gridRepo  = new GridRepository(P("grids.json"));
        _robot.stackRepo = new StackRepository(P("stacks.json"));
        _robot.pointRepo.SavePoint("TrayBase", new Vector6(100, 200, 50, 0, 0, 30));
        _robot.gridRepo.Upsert(new Grid
        {
            Id = "g-tray", Name = "Tray", BasePointName = "TrayBase",
            RowOffsetY = 20, ColOffsetX = 30, ColOffsetZ = 1, RowCount = 3, ColCount = 4, Rotation = 90,
        });
        _robot.stackRepo.Upsert(new RobotStack { Id = "s-parts", Name = "Parts", BasePointName = "TrayBase", OffsetZ = 12, MaxCount = 5 });
    }

    public void Dispose() { try { Directory.Delete(_tmp, recursive: true); } catch { } }

    private string P(string name) => Path.Combine(_tmp, name);

    private JsonElement Send(string command, object parameters)
    {
        var msg = new CommandMessage
        {
            Type = "Command", Id = "t", Command = command,
            Params = JsonSerializer.SerializeToElement(parameters),
        };
        return JsonSerializer.SerializeToElement(_robot.AddCommand(msg).GetAwaiter().GetResult());
    }

    private static string? Error(JsonElement r) =>
        r.ValueKind == JsonValueKind.Object && r.TryGetProperty("error", out var e) ? e.GetString() : null;

    private RobotCommand Queued()
    {
        Assert.True(_robot.QueuedCommands.TryDequeue(out var c));
        Assert.True(_robot.QueuedCommands.IsEmpty);
        return c!;
    }

    /// <summary>What a program's MoveL to the same cell resolves to (with the given active local).</summary>
    private Vector6 ProgramTarget(ProgramStep step, Vector6? local = null)
    {
        var src = new MoveTargetSources(_robot.pointRepo, _robot.pointRepo, _robot.gridRepo, _robot.stackRepo, _robot.localRepo);
        Assert.True(MoveTargetResolver.TryResolve(step, src, new VariableScope(), local, () => Vector6.Zero, out var t, out var err), err);
        return t;
    }

    private static void Same(Vector6 want, RobotCommand got)
    {
        Assert.Null(got.Name);
        Assert.Equal(want.X, got.X!.Value, 6); Assert.Equal(want.Y, got.Y!.Value, 6); Assert.Equal(want.Z, got.Z!.Value, 6);
        Assert.Equal(want.RX, got.RX!.Value, 6); Assert.Equal(want.RY, got.RY!.Value, 6); Assert.Equal(want.RZ, got.RZ!.Value, 6);
    }

    [Fact]
    public void Grid_cell_lands_where_a_program_grid_move_does()
    {
        // x/y/z are what an older controller would use: the grid target wins over them.
        var ack = Send("MoveL", new { grid = "g-tray", row = 2, col = 3, speed = 40, x = 1, y = 2, z = 3 });
        Assert.Null(Error(ack));
        Assert.Equal("controller", ack.GetProperty("resolvedBy").GetString());

        var cmd = Queued();
        Assert.Equal("MoveL", cmd.CommandType);
        Assert.Equal(40, cmd.Speed);
        var want = ProgramTarget(new ProgramStep
        {
            Type = StepType.MoveL, GridPoint = new GridPointRef { GridId = "g-tray", RowIndex = 2, ColIndex = 3 },
        });
        Same(want, cmd);
        // Rotation 90: col offset +X turns into +Y; row offset +Y into −X.
        Assert.Equal(100 - 40, cmd.X!.Value, 6);
        Assert.Equal(200 + 90, cmd.Y!.Value, 6);
        Assert.Equal(53, cmd.Z!.Value, 6);
        Assert.Equal(30, cmd.RZ!.Value, 6);
        Assert.Equal(cmd.X!.Value, ack.GetProperty("target").GetProperty("x").GetDouble(), 6);
    }

    [Fact]
    public void Stack_slot_and_the_active_local_resolve_like_a_program_move()
    {
        var local = new Vector6(500, -100, 20, 0, 0, 45);
        _robot.localRepo.SaveLocal("Fixture", local);
        _robot.ApplyLocal("Fixture");

        Assert.Null(Error(Send("MoveJ", new { stack = "s-parts", index = 4 })));
        var cmd = Queued();
        Assert.Equal("MoveJ", cmd.CommandType);
        Same(ProgramTarget(new ProgramStep { Type = StepType.MoveJ, StackPoint = new StackPointRef { StackId = "s-parts", Index = 4 } }, local), cmd);
        Assert.Equal(30 + 45, cmd.RZ!.Value, 6);
    }

    [Fact]
    public void Unknown_ids_and_cells_outside_the_array_are_refused_without_queueing()
    {
        Assert.Equal("pointNotFound", Error(Send("MoveL", new { grid = "Tray", row = 0, col = 0 })));   // by id, never by name
        Assert.Equal("pointNotFound", Error(Send("MoveL", new { stack = "nope", index = 0 })));
        Assert.Equal("indexOutOfRange", Error(Send("MoveL", new { grid = "g-tray", row = 3, col = 0 })));
        Assert.Equal("indexOutOfRange", Error(Send("MoveL", new { grid = "g-tray", row = 0, col = -1 })));
        Assert.Equal("indexOutOfRange", Error(Send("MoveJ", new { stack = "s-parts", index = 5 })));
        Assert.Equal("badParams", Error(Send("MoveL", new { grid = "g-tray", row = 1 })));
        Assert.True(_robot.QueuedCommands.IsEmpty);
    }

    [Fact]
    public void Unlimited_arrays_accept_any_non_negative_cell()
    {
        _robot.gridRepo.Upsert(new Grid { Id = "g-open", Name = "Open", BasePointName = "TrayBase", ColOffsetX = 10 });
        _robot.stackRepo.Upsert(new RobotStack { Id = "s-open", Name = "Open", BasePointName = "TrayBase", OffsetZ = 5 });
        Assert.Null(Error(Send("MoveL", new { grid = "g-open", row = 0, col = 7 })));
        Assert.Equal(170, Queued().X!.Value, 6);
        Assert.Null(Error(Send("MoveL", new { stack = "s-open", index = 9 })));
        Assert.Equal(95, Queued().Z!.Value, 6);
    }

    [Fact]
    public void A_plain_named_move_is_unchanged()
    {
        var ack = Send("MoveL", new { name = "TrayBase" });
        Assert.Null(Error(ack));
        Assert.False(ack.TryGetProperty("resolvedBy", out _));
        Assert.Equal("TrayBase", Queued().Name);
    }

    [Fact]
    public void Estop_refuses_a_grid_move()
    {
        _robot.UseAlarmHistoryForTest(new Controller.RobotControl.Alarms.AlarmHistory(P("alarms.json")));
        _robot.Config.Estop = new EstopConfig { Enabled = true, Source = EstopConfig.SourceStb, StbInput = 4 };
        bool raw = true;
        long now = 1_000;
        _robot.EstopReaderOverride = () => new EstopReading(true, raw, "STB input 4");
        _robot.SafetyClock = () => now += 200;
        _robot.SafetyTick();
        raw = false;
        _robot.SafetyTick();

        Assert.Equal("estopActive", Error(Send("MoveL", new { grid = "g-tray", row = 0, col = 0 })));
        Assert.Equal("estopActive", Error(Send("MoveJ", new { stack = "s-parts", index = 0 })));
        Assert.True(_robot.QueuedCommands.IsEmpty);
    }

    // ── Reach (real ASTRO kinematics, joint limits on) ────────────────────────

    [Fact]
    public void A_cell_out_of_reach_fails_outOfReach_and_does_not_queue()
    {
        using var t = new ArrayMoveCommandTests(identity: false);
        var r = t._robot;
        r.SetConfig(new RobotConfig
        {
            JointLimitsEnabled = true,
            Joint1Min = -170, Joint1Max = 170, Joint2Min = 60, Joint2Max = 450,
            Joint3Min = 0, Joint3Max = 400, Joint4Min = -360, Joint4Max = 360,
        });
        var astro = new ASTROKinematics();
        var near = astro.ForwardFromJoints(new Vector6(30, 300, 200, 0, 0, 10), Vector6.Zero);
        r.pointRepo.SavePoint("Reach", near);
        r.gridRepo.Upsert(new Grid { Id = "g-reach", Name = "Reach", BasePointName = "Reach", ColOffsetX = 100, ColCount = 6 });

        Assert.Null(Error(t.Send("MoveJ", new { grid = "g-reach", row = 0, col = 0 })));
        Assert.False(r.QueuedCommands.IsEmpty);
        r.QueuedCommands.Clear();

        var refused = t.Send("MoveJ", new { grid = "g-reach", row = 0, col = 5 });   // +500 mm
        Assert.Equal("outOfReach", Error(refused));
        Assert.Contains("out of reach", refused.GetProperty("message").GetString());
        Assert.True(r.QueuedCommands.IsEmpty);
    }
}
