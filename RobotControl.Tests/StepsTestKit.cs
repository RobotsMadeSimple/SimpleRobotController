using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Persistence;

namespace RobotControl.Tests;

/// <summary>
/// A private executor for package C's step tests (MoveC, SavePosition, SetSpeedFactor, ListOp,
/// Prompt): its own point / local / grid / stack files in a temp folder, a never-connected
/// controller, and helpers to build programs, tick them and play the motion thread (draining
/// <see cref="RobotController.QueuedCommands"/> so a dispatched move counts as finished).
/// </summary>
internal sealed class StepsRig : IDisposable
{
    public readonly string              Tmp;
    public readonly RobotController     Robot    = new();
    public readonly ProgramCycleManager Programs = new();
    public readonly PointRepository     Points;
    public readonly LocalRepository     Locals;
    public readonly ProgramExecutor     Exec;

    /// <summary>Every command the executor queued for the motion thread, in order.</summary>
    public readonly List<RobotCommand> Dispatched = new();

    public StepsRig()
    {
        Tmp = Path.Combine(Path.GetTempPath(), "rms-steps-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(Tmp);
        Points = new PointRepository(Path.Combine(Tmp, "points.json"), Path.Combine(Tmp, "pointsHistory.json"));
        Locals = new LocalRepository(Path.Combine(Tmp, "locals.json"), Path.Combine(Tmp, "localsHistory.json"));
        var grids  = new GridRepository(Path.Combine(Tmp, "grids.json"));
        var stacks = new StackRepository(Path.Combine(Tmp, "stacks.json"));
        Exec = new ProgramExecutor(Robot, Programs, Points, Robot.toolRepo, Locals, Robot.builtProgramRepo, grids, stacks);
    }

    public void Dispose()
    {
        if (Exec.IsRunning) Exec.Reset();
        try { Directory.Delete(Tmp, recursive: true); } catch { /* best effort */ }
    }

    // ── Builders ──────────────────────────────────────────────────────────────

    private static int _ids;

    public static ProgramStep Step(StepType type, Action<ProgramStep>? init = null)
    {
        var s = new ProgramStep { Id = $"c{Interlocked.Increment(ref _ids)}", Type = type };
        init?.Invoke(s);
        return s;
    }

    public static ProgramStep Set(string name, string expr) =>
        Step(StepType.SetVariable, s => { s.VariableName = name; s.VariableExpr = expr; });

    public static ProgramVariable Num(string name, double value = 0) => new() { Id = name, Name = name, Value = value };
    public static ProgramVariable Bool(string name) => new() { Id = name, Name = name, IsBoolean = true };
    public static ProgramVariable Str(string name, string value = "") => new() { Id = name, Name = name, IsString = true, StringValue = value };

    public static ProgramVariable Numbers(string name, params double[] values) =>
        new() { Id = name, Name = name, ElementType = ListElementType.Number, Items = values.Select(ObjectRecord.FromScalar).ToList() };

    public static ProgramVariable Booleans(string name, params bool[] values) =>
        new() { Id = name, Name = name, ElementType = ListElementType.Boolean, Items = values.Select(b => ObjectRecord.FromScalar(b ? 1 : 0)).ToList() };

    public static ProgramVariable PointList(string name, params Vector6Val[] values) =>
        new() { Id = name, Name = name, ElementType = ListElementType.Point, Items = values.Select(ObjectRecord.FromPoint).ToList() };

    public static ProgramVariable Records(string name, params ObjectRecord[] values) =>
        new() { Id = name, Name = name, ElementType = ListElementType.Record, Items = values.ToList() };

    public static Vector6Val P(double x, double y, double z, double rz = 0) => new() { X = x, Y = y, Z = z, RZ = rz };

    public static ObjectRecord Rec(params (string Field, double Value)[] fields)
    {
        var r = new ObjectRecord();
        foreach (var (f, v) in fields) r[f] = v;
        return r;
    }

    public static BuiltProgram Prog(IEnumerable<ProgramVariable> vars, params ProgramStep[] steps) =>
        new() { Id = "main", Name = "Main", Steps = steps.ToList(), Variables = vars.ToList() };

    public static BuiltProgram Prog(params ProgramStep[] steps) => Prog([], steps);

    // ── Running ───────────────────────────────────────────────────────────────

    /// <summary>Plays the motion thread: takes every queued command (a move then counts as finished).</summary>
    public List<RobotCommand> Drain()
    {
        var got = new List<RobotCommand>();
        while (Robot.QueuedCommands.TryDequeue(out var cmd)) got.Add(cmd);
        Dispatched.AddRange(got);
        return got;
    }

    public void Start(BuiltProgram p) => Exec.Start(p);

    /// <summary>One executor tick, then the motion thread's part.</summary>
    public void Tick(int times = 1)
    {
        for (int i = 0; i < times; i++)
        {
            Exec.Update();
            Drain();
        }
    }

    /// <summary>Starts <paramref name="p"/> and ticks until it ends (fails if it never does).</summary>
    public void Run(BuiltProgram p, int maxTicks = 500)
    {
        Start(p);
        for (int i = 0; i < maxTicks && Exec.IsRunning; i++) Tick();
        Assert.False(Exec.IsRunning, "program did not finish");
    }

    public double Live(string expr) => Exec.EvaluateLive(expr);

    public JsonElement Summary(string name = "Main") =>
        JsonSerializer.SerializeToElement(Programs.GetProgramsSummary())
            .EnumerateArray().First(p => p.GetProperty("name").GetString() == name);

    public string Status() => Summary().GetProperty("status").GetString()!;
    public string Error()  => Summary().GetProperty("errorDescription").GetString() ?? "";

    /// <summary>Asserts the run finished Complete (with its error in the failure message otherwise).</summary>
    public void AssertComplete() => Assert.True(Status() == "Complete", $"status {Status()}: {Error()}");

    public void AssertError(string contains)
    {
        Assert.Equal("Error", Status());
        Assert.Contains(contains, Error());
    }

    /// <summary>The list's elements as the program sees them now (via expressions).</summary>
    public int Count(string list) => (int)Live($"len(${list})");
}
