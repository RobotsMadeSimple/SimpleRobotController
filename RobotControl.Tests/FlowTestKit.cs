using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Persistence;

namespace RobotControl.Tests;

/// <summary>
/// A simulated executor for the flow tests (FlowControlTests, TryCatchTests, RoutineParamTests,
/// DebuggerTests): a controller with no hardware, its own temp repositories (routines are saved
/// there), step builders and run / status helpers.
/// </summary>
internal sealed class FlowTestKit : IDisposable
{
    private readonly string _tmp;
    public RobotController        Robot    { get; } = new();
    public ProgramCycleManager    Programs { get; } = new();
    public BuiltProgramRepository Repo     { get; }
    public ProgramExecutor        Exec     { get; }

    public FlowTestKit()
    {
        _tmp = Path.Combine(Path.GetTempPath(), "rms-flow-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(_tmp);
        var points = new PointRepository(Path.Combine(_tmp, "points.json"), Path.Combine(_tmp, "pointsHistory.json"));
        var grids  = new GridRepository(Path.Combine(_tmp, "grids.json"));
        var stacks = new StackRepository(Path.Combine(_tmp, "stacks.json"));
        Repo = new BuiltProgramRepository(Path.Combine(_tmp, "programs"));
        Exec = new ProgramExecutor(Robot, Programs, points, Robot.toolRepo, Robot.localRepo, Repo, grids, stacks);
    }

    public void Dispose()
    {
        try { Directory.Delete(_tmp, recursive: true); } catch { /* best effort */ }
    }

    // ── Builders ──────────────────────────────────────────────────────────────

    private static int _ids;
    public static ProgramStep Step(StepType type, Action<ProgramStep>? init = null)
    {
        var s = new ProgramStep { Id = $"f{Interlocked.Increment(ref _ids)}", Type = type };
        init?.Invoke(s);
        return s;
    }

    public static ProgramStep Set(string name, string expr) =>
        Step(StepType.SetVariable, s => { s.VariableName = name; s.VariableExpr = expr; });

    public static ProgramStep Loop(int count, params ProgramStep[] body) =>
        Step(StepType.Loop, s => { s.LoopCount = count; s.LoopSteps = body.ToList(); });

    public static ProgramStep While(string cond, params ProgramStep[] body) =>
        Step(StepType.Loop, s =>
        {
            s.LoopMode = "while";
            s.LoopWhileCondition = Cond(cond);
            s.LoopSteps = body.ToList();
        });

    public static ProgramStep If(string cond, ProgramStep[] then, ProgramStep[]? otherwise = null) =>
        Step(StepType.IfCondition, s =>
        {
            s.Condition = Cond(cond);
            s.IfSteps   = then.ToList();
            s.ElseSteps = otherwise?.ToList();
        });

    public static ConditionGroup Cond(string expr) =>
        new() { Items = [new ConditionItem { Id = "c" + Interlocked.Increment(ref _ids), Expr = expr }] };

    public static ProgramStep Break()    => Step(StepType.LoopControl, s => s.LoopControl = "break");
    public static ProgramStep Continue() => Step(StepType.LoopControl, s => s.LoopControl = "continue");

    public static ProgramStep Raise(string message, string? code = null) =>
        Step(StepType.RaiseError, s => { s.ErrorMessage = message; s.ErrorCode = code; });

    public static ProgramStep Try(ProgramStep[] body, ProgramStep[]? catchSteps = null, Action<ProgramStep>? init = null) =>
        Step(StepType.Try, s =>
        {
            s.TrySteps   = body.ToList();
            s.CatchSteps = catchSteps?.ToList();
            init?.Invoke(s);
        });

    public static ProgramStep Call(BuiltProgram routine, Action<ProgramStep>? init = null) =>
        Step(StepType.CallRoutine, s => { s.RoutineId = routine.Id; s.RoutineName = routine.Name; init?.Invoke(s); });

    public static ProgramVariable Num(string name, double value = 0, string? paramMode = null) =>
        new() { Id = name, Name = name, Value = value, ParamMode = paramMode };

    public static ProgramVariable Bool(string name, string? paramMode = null) =>
        new() { Id = name, Name = name, IsBoolean = true, ParamMode = paramMode };

    public static ProgramVariable Text(string name, string value = "", string? paramMode = null) =>
        new() { Id = name, Name = name, IsString = true, StringValue = value, ParamMode = paramMode };

    public static BuiltProgram Prog(IEnumerable<ProgramVariable> vars, params ProgramStep[] steps) =>
        new() { Id = "main", Name = "Main", Steps = steps.ToList(), Variables = vars.ToList() };

    /// <summary>Main with number variables x, y, n, i.</summary>
    public static BuiltProgram Prog(params ProgramStep[] steps) =>
        Prog([Num("x"), Num("y"), Num("n"), Num("i")], steps);

    public BuiltProgram Routine(string name, IEnumerable<ProgramVariable> vars, params ProgramStep[] steps)
    {
        var r = new BuiltProgram { Id = "r-" + name, Name = name, IsRoutine = true, Steps = steps.ToList(), Variables = vars.ToList() };
        Repo.Save(r);
        return Repo.GetById(r.Id)!;
    }

    // ── Running ───────────────────────────────────────────────────────────────

    /// <summary>Starts and ticks until the program stops running (finished or paused).</summary>
    public void Run(BuiltProgram p, int maxTicks = 500, ProgramExecutor.DebugOptions? debug = null)
    {
        Exec.Start(p, debug: debug);
        Tick(maxTicks);
    }

    /// <summary>Ticks until the program stops running.</summary>
    public void Tick(int maxTicks = 500)
    {
        for (int i = 0; i < maxTicks && Exec.IsRunning; i++) Exec.Update();
    }

    public void RunToEnd(BuiltProgram p, int maxTicks = 500, ProgramExecutor.DebugOptions? debug = null)
    {
        Run(p, maxTicks, debug);
        Assert.False(Exec.IsRunning, "program did not finish");
    }

    public double Value(string expr) => Exec.EvaluateLive(expr);

    public string? Text(string name) =>
        Exec.SnapshotLiveValues().TryGetValue(name, out var v) ? v as string : null;

    public JsonElement Summary(string name = "Main") =>
        JsonSerializer.SerializeToElement(Programs.GetProgramsSummary())
            .EnumerateArray().First(p => p.GetProperty("name").GetString() == name);

    public string Status() => Summary().GetProperty("status").GetString() ?? "";
    public string Error()  => Summary().GetProperty("errorDescription").GetString() ?? "";

    public string? PauseReason() =>
        Summary().GetProperty("pauseReason") is { ValueKind: JsonValueKind.String } r ? r.GetString() : null;

    public string? PausedAt() =>
        Summary().GetProperty("pausedAtStepId") is { ValueKind: JsonValueKind.String } r ? r.GetString() : null;
}
