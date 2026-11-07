using System.Text.Json;
using System.Text.Json.Nodes;
using Controller.RobotControl.Language;
using Controller.RobotControl.Validation;

namespace Controller.RobotControl.Commands;

/// <summary>
/// The text language (SRS) commands (docs/program-text.md, docs/program-language.md §6.9):
/// <list type="bullet">
/// <item><c>FormatProgramText { program, includeIds?, includeHeader? }</c> → <see cref="Language.FormatProgramTextResult"/></item>
/// <item><c>ParseProgramText { text, base?, validate? }</c> → <see cref="Language.ParseProgramTextResult"/>
/// (validation problems are mapped onto the text, source "validate")</item>
/// </list>
/// Names in the text resolve against this controller's points, tools, locals, grids, stacks,
/// programs, vision programs, devices and cameras.
/// </summary>
internal sealed class LanguageCommands(RobotController robot, Func<ValidationContext> validationContext)
{
    private static readonly JsonSerializerOptions ProgramJson = new()
    {
        Converters = { new System.Text.Json.Serialization.JsonStringEnumConverter() },
        PropertyNameCaseInsensitive = true,
    };

    public void Register(CommandDispatcher d)
    {
        d.Add("FormatProgramText", Format);
        d.Add("ParseProgramText",  Parse);
    }

    private object? Format(CommandMessage msg)
    {
        var program = ProgramParam(msg, "program")
            ?? throw new InvalidOperationException("FormatProgramText needs a 'program' param");
        return SrsLanguage.Format(program, new ControllerSrsNames(robot),
            BoolParam(msg, "includeIds") ?? false, BoolParam(msg, "includeHeader") ?? false);
    }

    private object? Parse(CommandMessage msg)
    {
        var text = Param(msg, "text") is { ValueKind: JsonValueKind.String } t ? t.GetString() ?? "" : "";
        var @base = ProgramParam(msg, "base");
        Func<BuiltProgram, IEnumerable<ValidationProblem>>? validate = BoolParam(msg, "validate") ?? true
            ? prog => ProgramValidator.Validate(prog, validationContext())
            : null;
        return SrsLanguage.Parse(text, @base, new ControllerSrsNames(robot), validate);
    }

    private static JsonElement? Param(CommandMessage msg, string name)
    {
        if (msg.Params is not { ValueKind: JsonValueKind.Object } ps) return null;
        foreach (var prop in ps.EnumerateObject())
            if (string.Equals(prop.Name, name, StringComparison.OrdinalIgnoreCase)) return prop.Value;
        return null;
    }

    private static bool? BoolParam(CommandMessage msg, string name) => Param(msg, name) switch
    {
        { ValueKind: JsonValueKind.True } => true,
        { ValueKind: JsonValueKind.False } => false,
        _ => null,
    };

    /// <summary>A program param, read the way ValidateBuiltProgram reads one: the object or a
    /// JSON string of it, with step types this controller does not know kept as Unknown.</summary>
    private static BuiltProgram? ProgramParam(CommandMessage msg, string name)
    {
        if (Param(msg, name) is not { } raw || raw.ValueKind is JsonValueKind.Null or JsonValueKind.Undefined) return null;
        var json = raw.ValueKind == JsonValueKind.String ? raw.GetString() ?? "{}" : raw.GetRawText();
        var node = JsonNode.Parse(json);
        PatchUnknownStepTypes(node);
        return node?.Deserialize<BuiltProgram>(ProgramJson);
    }

    private static readonly HashSet<string> StepArrays =
        new(["steps", StepTree.Roles.LoopSteps, StepTree.Roles.IfSteps, StepTree.Roles.ElseSteps,
             StepTree.Roles.SwitchDefault, StepTree.Roles.TrySteps, StepTree.Roles.CatchSteps,
             StepTree.Roles.CncProgramSteps], StringComparer.OrdinalIgnoreCase);

    private static void PatchUnknownStepTypes(JsonNode? node)
    {
        switch (node)
        {
            case JsonObject obj:
                foreach (var kv in obj.ToList())
                {
                    if (StepArrays.Contains(kv.Key) && kv.Value is JsonArray steps)
                        foreach (var step in steps)
                            if (step is JsonObject so && so["type"] is JsonValue tv && tv.TryGetValue<string>(out var type) &&
                                !Enum.TryParse<StepType>(type, ignoreCase: true, out _))
                            {
                                so["unknownStepType"] = type;
                                so["type"] = nameof(StepType.Unknown);
                            }
                    PatchUnknownStepTypes(kv.Value);
                }
                break;
            case JsonArray arr:
                foreach (var item in arr) PatchUnknownStepTypes(item);
                break;
        }
    }
}
