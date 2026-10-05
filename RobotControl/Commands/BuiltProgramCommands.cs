using System.Text.Json;
using System.Text.Json.Nodes;
using System.Text.Json.Serialization;
using Controller.RobotControl.Execution;
using Controller.RobotControl.Validation;

namespace Controller.RobotControl.Commands;

/// <summary>
/// Built (in-app authored) programs: the repository, foreground execution,
/// background programs, live variables/images and the active CNC toolpath.
/// </summary>
internal sealed class BuiltProgramCommands
{
    private readonly RobotController _robot;
    private readonly ProgramCycleManager _programs;
    private readonly ProgramExecutor? _executor;
    private readonly BackgroundProgramManager _background;

    public BuiltProgramCommands(RobotController robot, ProgramCycleManager programs,
        ProgramExecutor? executor, BackgroundProgramManager background)
    {
        _robot      = robot;
        _programs   = programs;
        _executor   = executor;
        _background = background;
    }

    public void Register(CommandDispatcher d)
    {
        d.Add("GetBuiltPrograms",        GetBuiltPrograms);
        d.Add("SaveBuiltProgram",        SaveBuiltProgram);
        d.Add("DeleteBuiltProgram",      DeleteBuiltProgram);
        d.Add("SaveBuiltProgramImage",   SaveBuiltProgramImage);
        d.Add("ExecuteBuiltProgram",     ExecuteBuiltProgram);
        d.Add("StopBuiltProgram",        _ => _executor?.Stop());
        d.Add("StartBackgroundProgram",  StartBackgroundProgram);
        d.Add("StopBackgroundProgram",   StopBackgroundProgram);
        d.Add("GetProgramVariables",     GetProgramVariables);
        // Not "GetProgramImage" — GetProgramImages is the program *thumbnail*
        // list, an unrelated thing, and the two would be a singular/plural apart.
        d.Add("GetProgramVariableImage", GetProgramVariableImage);
        d.Add("GetCncToolpath",          GetCncToolpath);
        // Program editor support — docs/expressions-and-variables.md §4.
        d.Add("ValidateBuiltProgram",    ValidateBuiltProgram);
        d.Add("EvaluateExpression",      EvaluateExpression);
        d.Add("GetExpressionSymbols",    GetExpressionSymbols);
    }

    private object? GetBuiltPrograms(CommandMessage msg)
    {
        var list = _robot.builtProgramRepo.GetAll();
        var json = JsonSerializer.Serialize(list, CommandJson.CamelCaseWithEnums);
        return new { programs = json };
    }

    private void SaveBuiltProgram(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SaveBuiltProgramParams>(msg);
        _robot.builtProgramRepo.Save(new BuiltProgram
        {
            Id                  = p.Id,
            Name                = p.Name,
            Description         = p.Description,
            Steps               = p.Steps,
            Variables           = p.Variables,
            IsRoutine           = p.IsRoutine,
            IsBackground        = p.IsBackground,
            KillBackgroundOnStop = p.KillBackgroundOnStop,
        });
    }

    private void DeleteBuiltProgram(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<BuiltProgramNameParams>(msg);
        _robot.builtProgramRepo.Delete(p.Name);
        _programs.RemoveProgram(p.Name);
    }

    private void SaveBuiltProgramImage(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<SaveBuiltProgramImageParams>(msg);
        var bytes = Convert.FromBase64String(p.Image);
        _robot.builtProgramRepo.SaveImage(p.Name, bytes);
    }

    private void ExecuteBuiltProgram(CommandMessage msg)
    {
        var p    = CommandJson.LoadParams<BuiltProgramNameParams>(msg);
        var prog = _robot.builtProgramRepo.Get(p.Name);
        if (prog != null)
        {
            _robot.DisplaceRunningBuiltProgram(p.Name);
            // A jog left running (the app relies on the 1 s watchdog rather than StopJog)
            // would otherwise hold the motion queue until it times out and decelerates,
            // delaying the program's first move by a second or more.
            _robot.StopJog();
            var imgBytes = _robot.builtProgramRepo.GetImage(p.Name);
            _executor?.Start(prog, imgBytes != null ? Convert.ToBase64String(imgBytes) : null);
        }
    }

    private void StartBackgroundProgram(CommandMessage msg)
    {
        var p     = CommandJson.LoadParams<ProgramActionParams>(msg);
        var built = _robot.builtProgramRepo.Get(p.ProgramName);
        if (built != null && built.IsBackground)
            _background.TryStart(built);
    }

    private void StopBackgroundProgram(CommandMessage msg)
    {
        var p     = CommandJson.LoadParams<ProgramActionParams>(msg);
        var built = _robot.builtProgramRepo.Get(p.ProgramName);
        if (built != null) _background.Stop(built.Id);
    }

    private bool IsForeground(string name) =>
        _executor?.CurrentProgramName?.Equals(name, StringComparison.OrdinalIgnoreCase) == true;

    private object? GetProgramVariables(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<BuiltProgramNameParams>(msg);
        bool foreground = IsForeground(p.Name);
        var vars = foreground
            ? _executor!.GetDisplayVariables()
            : _background.GetDisplayVariables(p.Name);
        // Images are listed by name and revision only — see GetDisplayImages.
        // The monitor fetches the bytes with GetProgramImage when a revision
        // moves, which keeps this poll the same size whether or not the
        // program holds a camera frame.
        var images = foreground
            ? _executor!.GetDisplayImages()
            : _background.GetDisplayImages(p.Name);
        return new
        {
            variables = vars.Select(v => new { name = v.Name, value = v.Value, isBoolean = v.IsBoolean }).ToList(),
            images    = images.Select(i => new { name = i.Name, revision = i.Revision }).ToList()
        };
    }

    private object? GetProgramVariableImage(CommandMessage msg)
    {
        var p = CommandJson.LoadParams<ProgramImageParams>(msg);
        var data = IsForeground(p.Name)
            ? _executor!.GetDisplayImage(p.Variable)
            : _background.GetDisplayImage(p.Name, p.Variable);
        // Empty rather than an error when there is nothing to send: a monitor
        // asking about a program that has just stopped is ordinary, not a fault.
        return new { name = p.Name, variable = p.Variable, image = data };
    }

    private object? GetCncToolpath(CommandMessage msg)
    {
        // Resolved toolpath of the CNC block currently executing —
        // anchor and variables applied. Null when no block is active.
        var tp = _robot.ActiveCncToolpath;
        return new
        {
            toolpath = tp == null ? null : new
            {
                programName = tp.ProgramName,
                paths       = tp.Paths,
                holes       = tp.Holes.Select(h => new { x = h.X, y = h.Y }).ToList(),
            },
        };
    }

    // ── Expressions and validation ────────────────────────────────────────────

    private static readonly JsonSerializerOptions ProgramJson = new()
    {
        Converters = { new JsonStringEnumConverter() },
        PropertyNameCaseInsensitive = true,
    };

    private static JsonElement? Param(CommandMessage msg, string name)
    {
        if (msg.Params is not { ValueKind: JsonValueKind.Object } p) return null;
        foreach (var prop in p.EnumerateObject())
            if (string.Equals(prop.Name, name, StringComparison.OrdinalIgnoreCase)) return prop.Value;
        return null;
    }

    private static string? StringParam(CommandMessage msg, string name) =>
        Param(msg, name) is { ValueKind: JsonValueKind.String } v ? v.GetString() : null;

    /// <summary>
    /// ValidateBuiltProgram { program } → { problems: [...] }. The program may be unsaved;
    /// a step type this controller does not know is reported rather than failing the parse.
    /// </summary>
    private object? ValidateBuiltProgram(CommandMessage msg)
    {
        var raw = Param(msg, "program")
            ?? throw new InvalidOperationException("ValidateBuiltProgram needs a 'program' param");
        // Accept the object itself or a JSON string of it (what some clients send).
        var json = raw.ValueKind == JsonValueKind.String ? raw.GetString() ?? "{}" : raw.GetRawText();

        var node = JsonNode.Parse(json) ?? throw new InvalidOperationException("'program' is null");
        PatchUnknownStepTypes(node);
        var program = node.Deserialize<BuiltProgram>(ProgramJson)
            ?? throw new InvalidOperationException("'program' could not be read");

        var problems = ProgramValidator.Validate(program, BuildValidationContext());
        return new { problems };
    }

    // The arrays that hold steps; only their elements are step objects.
    private static readonly HashSet<string> StepArrays =
        new(["steps", "loopSteps", "ifSteps", "elseSteps", "cncProgramSteps"], StringComparer.OrdinalIgnoreCase);

    /// <summary>Same idea as the repository's loader: a step whose "type" this controller
    /// does not know becomes Unknown (with the original kept in unknownStepType).</summary>
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

    private ValidationContext BuildValidationContext()
    {
        var repo   = _robot.builtProgramRepo;
        var vision = _robot.VisionRepo;
        return new ValidationContext
        {
            PointExists         = n => _robot.pointRepo.Get(n) != null,
            ToolExists          = n => _robot.toolRepo.Get(n) != null,
            LocalExists         = n => _robot.localRepo.Get(n) != null,
            GridExists          = id => _robot.gridRepo.Get(id) != null,
            StackExists         = id => _robot.stackRepo.Get(id) != null,
            VisionProgramExists = vision == null ? null : id => vision.Get(id) != null,
            VisionProgramCamera = vision == null || _robot.CalibrationRepo == null ? null
                : id => vision.Get(id) is { } vp ? (vp.CameraId, _robot.CalibrationRepo.IsCalibrated(vp.CameraId)) : null,
            FindProgram         = (id, name) =>
                (!string.IsNullOrEmpty(id) ? repo.GetById(id) : null)
                ?? (!string.IsNullOrEmpty(name) ? repo.Get(name) : null),
            IoNames       = new HashSet<string>(IoSymbols().Select(i => i.Name), StringComparer.OrdinalIgnoreCase),
            PropertyNames = new HashSet<string>(new RobotPropertySource(_robot, null).List().Select(p => p.Name),
                                                StringComparer.OrdinalIgnoreCase),
        };
    }

    /// <summary>
    /// The executor whose live variables belong to <paramref name="name"/>: the foreground
    /// executor while it holds the program (running, paused or just finished), else a
    /// running background executor with that name, else null.
    /// </summary>
    private ProgramExecutor? LiveExecutorFor(string? name)
    {
        if (string.IsNullOrEmpty(name)) return null;
        if (IsForeground(name!)) return _executor;
        return _background.FindRunning(name!);
    }

    /// <summary>
    /// EvaluateExpression { expression, programName? } → { ok, value, error?, isBoolean? }.
    /// Against the named program's live variables when the foreground executor holds it,
    /// otherwise against globals + IO + properties.
    /// </summary>
    private object? EvaluateExpression(CommandMessage msg)
    {
        var expr        = StringParam(msg, "expression") ?? "";
        var programName = StringParam(msg, "programName");
        try
        {
            double value;
            if (LiveExecutorFor(programName) is { } live)
                value = live.EvaluateLive(expr);
            else
            {
                var vars = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
                _background.GlobalVars.CopyInto(vars);
                vars["time_ms"] = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
                ProgramExecutor.AddIoVariables(_robot, vars);
                // Global computed variables resolve too, ahead of the system properties.
                var props = ComputedPropertySource.ForGlobals(_background.GlobalVars,
                    io => ProgramExecutor.AddIoVariables(_robot, io), new RobotPropertySource(_robot, null));
                value = ExpressionEvaluator.Evaluate(expr, vars, null, props);
            }
            bool isBoolean = ExpressionEvaluator.IsBooleanExpression(expr);
            if (!double.IsFinite(value))
                return new { ok = false, value = (double?)null, error = $"Result is not a finite number ({value})", isBoolean };
            return new { ok = true, value = (double?)value, error = (string?)null, isBoolean };
        }
        catch (ExpressionParseException ex)
        {
            return new { ok = false, value = (double?)null, error = ex.Position < 0 ? ex.Message : $"{ex.Message} (at position {ex.Position})",
                         isBoolean = false, position = ex.Position, code = ex.Code };
        }
        catch (UnknownVariableException ex)
        {
            return new { ok = false, value = (double?)null, error = ex.Message, isBoolean = false,
                         position = -1, code = "unknownVariable" };
        }
    }

    /// <summary>
    /// GetExpressionSymbols { programName? } → { variables, properties, functions, io }.
    /// Variables come from the program definition (with live values while the foreground
    /// executor holds it); without a program, the current global variables.
    /// </summary>
    private object? GetExpressionSymbols(CommandMessage msg)
    {
        var programName = StringParam(msg, "programName");
        var program = string.IsNullOrEmpty(programName) ? null : _robot.builtProgramRepo.Get(programName!);
        var live = program != null ? LiveExecutorFor(program.Name)?.SnapshotLiveValues() : null;
        var globalProps = ComputedPropertySource.ForGlobals(_background.GlobalVars,
            io => ProgramExecutor.AddIoVariables(_robot, io), new RobotPropertySource(_robot, null));
        var variables = VariableSymbols(program, live, _background.GlobalVars, globalProps);

        var properties = new RobotPropertySource(_robot, null).List()
            .Select(p => new { name = p.Name, description = p.Description, type = p.Type })
            .Append(new { name = "time_ms", description = "Unix time in milliseconds (built-in variable; same as $time.now)", type = "number" })
            .ToList();

        var functions = ExpressionEvaluator.Functions
            .Select(f => new { name = f.Name, signature = f.Signature, description = f.Description })
            .ToList();

        var io = IoSymbols().Select(i => new { name = i.Name, description = i.Description }).ToList();

        return new { variables, properties, functions, io };
    }

    /// <summary>
    /// The <c>variables</c> of GetExpressionSymbols. With a program: its declared variables,
    /// computed ones as <c>kind: "computed"</c> with their <c>expression</c>, valued from
    /// <paramref name="live"/> when the program is held by an executor (a global computed
    /// one is evaluated from the global store otherwise). Without: the current global values
    /// and the registered global computed variables.
    /// </summary>
    internal static List<object> VariableSymbols(BuiltProgram? program, IReadOnlyDictionary<string, object?>? live,
                                                 GlobalVariableStore globals, ComputedPropertySource globalProps)
    {
        var variables = new List<object>();

        object? GlobalComputedValue(string name)
        {
            try { return globalProps.TryGetComputed(name, out var v) && double.IsFinite(v) ? v : null; }
            catch { return null; }
        }

        if (program != null)
        {
            foreach (var v in program.Variables ?? [])
            {
                if (string.IsNullOrWhiteSpace(v.Name)) continue;
                if (v.IsComputed == true)
                {
                    object? cv = null;
                    if (live != null && live.TryGetValue(v.Name, out var lcv)) cv = lcv;
                    else if (v.IsGlobal == true && globals.TryGetComputed(v.Name, out _)) cv = GlobalComputedValue(v.Name);
                    variables.Add(new
                    {
                        name         = v.Name,
                        kind         = "computed",
                        elementType  = (string?)null,
                        isGlobal     = v.IsGlobal == true,
                        isPersistent = false,
                        isBoolean    = v.IsBoolean == true,
                        expression   = v.ValueExpression ?? "",
                        value        = cv,
                    });
                    continue;
                }
                var list = v.ToListVar();
                string kind = list != null ? "list"
                            : v.IsString == true ? "string"
                            : v.IsImage == true ? "image"
                            : v.IsBoolean == true ? "boolean"
                            : "number";
                object? value = kind switch
                {
                    "list"   => list!.Count,
                    "string" => v.StringValue ?? "",
                    "image"  => null,
                    _        => v.Value,
                };
                if (live != null && kind != "image" && live.TryGetValue(v.Name, out var lv)) value = lv;
                variables.Add(new
                {
                    name         = v.Name,
                    kind,
                    elementType  = list != null ? list.ElementType.ToString() : null,
                    isGlobal     = v.IsGlobal == true,
                    isPersistent = v.IsPersistent == true,
                    value,
                });
            }
        }
        else
        {
            foreach (var kv in globals.Snapshot().OrderBy(k => k.Key, StringComparer.OrdinalIgnoreCase))
                variables.Add(new
                {
                    name = kv.Key, kind = "number", elementType = (string?)null,
                    isGlobal = true, isPersistent = false, value = (object?)kv.Value,
                });
            foreach (var kv in globals.ComputedSnapshot().OrderBy(k => k.Key, StringComparer.OrdinalIgnoreCase))
                variables.Add(new
                {
                    name = kv.Key, kind = "computed", elementType = (string?)null,
                    isGlobal = true, isPersistent = false,
                    // The store keeps formulas only; the flag lives in the declaring program.
                    isBoolean = LooksBoolean(kv.Value),
                    expression = kv.Value, value = GlobalComputedValue(kv.Key),
                });
        }
        return variables;

        static bool LooksBoolean(string expr)
        {
            try { return ExpressionEvaluator.IsBooleanExpression(expr); }
            catch (ExpressionParseException) { return false; }
        }
    }

    /// <summary>Every IO name an expression can read, as AddIoVariables writes them.</summary>
    private List<(string Name, string Description)> IoSymbols()
    {
        var list = new List<(string, string)>();
        for (int n = 1; n <= 4; n++) list.Add(($"stb.in{n}",  $"Driver board input {n}"));
        for (int n = 1; n <= 4; n++) list.Add(($"stb.out{n}", $"Driver board output {n}"));
        for (int n = 1; n <= 4; n++) list.Add(($"relay.{n}",  $"USB relay {n}"));
        try
        {
            foreach (var nano in _robot.NanoManager.GetAllStates())
                if (!string.IsNullOrEmpty(nano.Name))
                    foreach (var pin in nano.Pins)
                        if (!string.IsNullOrEmpty(pin.Name))
                            list.Add(($"nano.{nano.Name}.{pin.Name}", $"Nano '{nano.Name}' pin {pin.Pin} ({pin.Type})"));
        }
        catch { /* no nano manager yet — the fixed names still stand */ }
        return list;
    }
}
