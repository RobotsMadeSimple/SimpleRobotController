using System.Text.Json;
using Controller.RobotControl;
using Controller.RobotControl.Execution;

namespace RobotControl.Tests;

/// <summary>
/// Launch-time input seeding: a program display page passes a <c>variables</c> map to
/// StartProgram, which overrides the declared initial values of variables flagged
/// <see cref="ProgramVariable.IsInput"/> before the run. Driven straight off a VariableScope
/// (the same seam StartCore calls after Initialize).
/// </summary>
public sealed class ProgramInputSeedingTests
{
    private static BuiltProgram Program(params ProgramVariable[] vars) =>
        new() { Id = "prog-1", Name = "P", Variables = vars.ToList() };

    private static ProgramVariable Num(string name, double value, bool input, bool boolean = false) =>
        new() { Name = name, Value = value, IsInput = input ? true : null, IsBoolean = boolean ? true : null };

    private static Dictionary<string, JsonElement> Seed(object anon) =>
        JsonSerializer.Deserialize<Dictionary<string, JsonElement>>(JsonSerializer.Serialize(anon))!;

    [Fact]
    public void Seed_overrides_an_input_scalars_declared_value()
    {
        var prog  = Program(Num("speed", 1, input: true));
        var scope = new VariableScope();
        scope.Initialize(prog);
        scope.SeedInputs(prog, Seed(new { speed = 42 }));

        Assert.True(scope.TryGetLocal("speed", out var v));
        Assert.Equal(42, v);
    }

    [Fact]
    public void Non_input_variables_are_left_at_their_declared_value()
    {
        var prog  = Program(Num("locked", 1, input: false));
        var scope = new VariableScope();
        scope.Initialize(prog);
        scope.SeedInputs(prog, Seed(new { locked = 99 }));

        Assert.True(scope.TryGetLocal("locked", out var v));
        Assert.Equal(1, v); // ignored — not flagged as an input
    }

    [Fact]
    public void Boolean_input_accepts_true_false_as_one_zero()
    {
        var prog  = Program(Num("run", 0, input: true, boolean: true));
        var scope = new VariableScope();
        scope.Initialize(prog);
        scope.SeedInputs(prog, Seed(new { run = true }));

        Assert.True(scope.TryGetLocal("run", out var v));
        Assert.Equal(1, v);
    }

    [Fact]
    public void Numeric_string_seeds_a_scalar_input()
    {
        var prog  = Program(Num("count", 0, input: true));
        var scope = new VariableScope();
        scope.Initialize(prog);
        scope.SeedInputs(prog, Seed(new { count = "7" })); // a text field delivers a string

        Assert.True(scope.TryGetLocal("count", out var v));
        Assert.Equal(7, v);
    }

    [Fact]
    public void String_input_is_seeded_as_text()
    {
        var prog = Program(new ProgramVariable { Name = "label", IsString = true, IsInput = true, StringValue = "" });
        var scope = new VariableScope();
        scope.Initialize(prog);
        scope.SeedInputs(prog, Seed(new { label = "Bin A" }));

        Assert.True(scope.TryGetString("label", out var s));
        Assert.Equal("Bin A", s);
    }

    [Fact]
    public void Computed_input_is_never_written_as_a_stored_scalar()
    {
        var prog = Program(new ProgramVariable { Name = "calc", IsComputed = true, IsInput = true, ValueExpression = "1 + 1" });
        var scope = new VariableScope();
        scope.Initialize(prog);
        scope.SeedInputs(prog, Seed(new { calc = 5 })); // must be ignored, not throw

        Assert.False(scope.TryGetLocal("calc", out _)); // computed — no stored value to override
    }

    [Fact]
    public void Unknown_and_absent_seed_names_are_ignored()
    {
        var prog  = Program(Num("speed", 3, input: true));
        var scope = new VariableScope();
        scope.Initialize(prog);
        scope.SeedInputs(prog, Seed(new { nonexistent = 123 })); // name not in the program

        Assert.True(scope.TryGetLocal("speed", out var v));
        Assert.Equal(3, v);
    }
}
