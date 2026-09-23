using System.Text.Json.Serialization;
using System.Text.RegularExpressions;
using Controller.RobotControl.Execution;

namespace Controller.RobotControl.Validation
{
    /// <summary>One thing wrong (or suspect) with a program, as ValidateBuiltProgram reports it.</summary>
    public sealed class ValidationProblem
    {
        /// <summary>The step's id; null for program-level problems (variables).</summary>
        [JsonPropertyName("stepId")]   public string? StepId   { get; init; }
        /// <summary>Where the step sits: "steps[2].loopSteps[0]", "variables[1]",
        /// "steps[3].routine(Pick).steps[0]" for a routine body reached through a call.</summary>
        [JsonPropertyName("stepPath")] public string  StepPath { get; init; } = "";
        /// <summary>The offending field ("expressions.offsetZ", "condition.items[0].left"), when there is one.</summary>
        [JsonPropertyName("field")]    public string? Field    { get; init; }
        /// <summary>"error" or "warning".</summary>
        [JsonPropertyName("severity")] public string  Severity { get; init; } = ValidationSeverity.Error;
        [JsonPropertyName("code")]     public string  Code     { get; init; } = "";
        [JsonPropertyName("message")]  public string  Message  { get; init; } = "";

        public override string ToString() => $"{Severity} {Code} @ {StepPath}{(Field != null ? "." + Field : "")}: {Message}";
    }

    public static class ValidationSeverity
    {
        public const string Error   = "error";
        public const string Warning = "warning";
    }

    /// <summary>The validation codes (docs/expressions-and-variables.md §4), plus the few extras noted.</summary>
    public static class ValidationCodes
    {
        public const string UnknownPoint         = "unknownPoint";
        public const string UnknownTool          = "unknownTool";
        public const string UnknownLocal         = "unknownLocal";
        public const string UnknownRoutine       = "unknownRoutine";
        public const string UnknownVisionProgram = "unknownVisionProgram";
        public const string UnknownGrid          = "unknownGrid";
        public const string UnknownStack         = "unknownStack";
        public const string UnknownLabel         = "unknownLabel";
        public const string DuplicateLabel       = "duplicateLabel";
        public const string UnknownVariable      = "unknownVariable";
        public const string UnknownProperty      = "unknownProperty";
        public const string ExpressionSyntax     = "expressionSyntax";
        public const string EmptyLoop            = "emptyLoop";
        public const string EmptyBranch          = "emptyBranch";
        public const string MissingField         = "missingField";
        public const string RoutineRecursion     = "routineRecursion";
        public const string DisabledStep         = "disabledStep";     // warning
        public const string UnreachableStep      = "unreachableStep";  // warning
        public const string UnusedVariable       = "unusedVariable";   // warning

        // Beyond the §4 list:
        /// <summary>A function name the evaluator does not know (§2 names it as a parse error).</summary>
        public const string UnknownFunction      = "unknownFunction";
        /// <summary>A function called with the wrong number of arguments (§2).</summary>
        public const string BadArity             = "badArity";
        /// <summary>A write (SetVariable, loop/vision/HTTP target, variable declaration) to a property or IO name.</summary>
        public const string ReadOnlyProperty     = "readOnlyProperty";
        /// <summary>Start/Stop/WaitForBackground naming a program that does not exist.</summary>
        public const string UnknownProgram       = "unknownProgram";
        /// <summary>A step type this controller does not know (it would be skipped) — warning.</summary>
        public const string UnknownStepType      = "unknownStepType";

        // Computed variables (§7):
        /// <summary>Computed variables whose formulas depend on each other in a loop.</summary>
        public const string ComputedCycle        = "computedCycle";
        /// <summary>A write target (Set Variable, loop/forEach variable, vision/HTTP output, stopwatch) that is a computed variable.</summary>
        public const string ComputedVariable     = "computedVariable";
        /// <summary>A global computed formula that references a non-global program variable.</summary>
        public const string ComputedGlobalScope  = "computedGlobalScope";
        /// <summary>isComputed together with isPersistent / isString / isImage / isStopwatch / items.</summary>
        public const string ComputedKindConflict = "computedKindConflict";
    }

    /// <summary>
    /// What the validator checks names against. Every lookup is optional: a null predicate
    /// means "cannot check", and that check is skipped rather than reported.
    /// </summary>
    internal sealed class ValidationContext
    {
        public Func<string, bool>? PointExists         { get; init; }
        public Func<string, bool>? ToolExists          { get; init; }
        public Func<string, bool>? LocalExists         { get; init; }
        /// <summary>By grid id (what GridPointRef carries).</summary>
        public Func<string, bool>? GridExists          { get; init; }
        /// <summary>By stack id (what StackPointRef carries).</summary>
        public Func<string, bool>? StackExists         { get; init; }
        /// <summary>By vision program id.</summary>
        public Func<string, bool>? VisionProgramExists { get; init; }
        /// <summary>A built program / routine by id first, then name — the executor's own order.</summary>
        public Func<string?, string?, BuiltProgram?>? FindProgram { get; init; }

        /// <summary>Exact IO names (live nano pins, …). The stb/relay/nano patterns are always accepted too.</summary>
        public ISet<string> IoNames { get; init; } = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        /// <summary>Every property name (§3), including the configured <c>aux.*</c> ones.</summary>
        public ISet<string> PropertyNames { get; init; } =
            new HashSet<string>(RobotPropertySource.StaticNames, StringComparer.OrdinalIgnoreCase);

        /// <summary>The expression function table.</summary>
        public IReadOnlyList<ExpressionFunction> Functions { get; init; } = ExpressionEvaluator.Functions;

        /// <summary>No repositories: names of points/tools/… are not checked.</summary>
        public static ValidationContext Offline() => new();
    }

    /// <summary>
    /// Static checks on a <see cref="BuiltProgram"/> before it runs — the rules of
    /// docs/expressions-and-variables.md §5. Pure: reads only the program and the context.
    /// </summary>
    internal static class ProgramValidator
    {
        public static List<ValidationProblem> Validate(BuiltProgram program, ValidationContext ctx) =>
            new Walker(program, ctx).Run();

        // stb.in1-4 / stb.out1-4 / relay.1-4 / nano.<board>.<pin>
        private static readonly Regex IoPattern = new(
            @"^(stb\.(in|out)[1-4]|relay\.[1-4]|nano\.[^.\s]+\.[^.\s]+)$",
            RegexOptions.IgnoreCase | RegexOptions.CultureInvariant);

        private static readonly string[] ConditionOps =
            ["==", "!=", ">", ">=", "<", "<=", "contains", "startsWith", "endsWith"];

        private enum SymKind { Number, Boolean, Stopwatch, String, Image, List, Computed }

        private sealed record Sym(SymKind Kind, ListElementType ElementType = ListElementType.Record);

        private sealed class Walker
        {
            private readonly BuiltProgram _main;
            private readonly ValidationContext _ctx;
            private readonly List<ValidationProblem> _problems = new();

            // Every variable any reachable program/routine declares, and the scalars/lists
            // steps create implicitly (vision outputs, HTTP inbound, SetVariable, stopwatches).
            private readonly Dictionary<string, Sym> _declared = new(StringComparer.OrdinalIgnoreCase);
            private readonly Dictionary<string, Sym> _implicit = new(StringComparer.OrdinalIgnoreCase);
            private readonly HashSet<string> _used = new(StringComparer.OrdinalIgnoreCase);
            private readonly HashSet<string> _functionNames;

            // Declared variables flagged isGlobal, and every computed variable's formula (for
            // the cycle and global-scope checks) — across the program and its routines.
            private readonly HashSet<string> _globalNames = new(StringComparer.OrdinalIgnoreCase);
            private readonly Dictionary<string, (string Formula, bool IsGlobal)> _computedFormulas =
                new(StringComparer.OrdinalIgnoreCase);

            // Routines already validated, and the chain currently being walked (for recursion).
            private readonly HashSet<string> _validatedRoutines = new(StringComparer.OrdinalIgnoreCase);
            private readonly List<BuiltProgram> _callStack = new();

            // Names in scope only inside a loop body (count index, forEach value/index).
            private readonly List<string> _loopScope = new();

            // When non-null, variable lookups see only these declarations (initialisers run in
            // declaration order, so one can reference the variables above it but not below).
            private Dictionary<string, Sym>? _initScope;

            public Walker(BuiltProgram main, ValidationContext ctx)
            {
                _main = main;
                _ctx  = ctx;
                _functionNames = new HashSet<string>(ctx.Functions.Select(f => f.Name), StringComparer.OrdinalIgnoreCase);
            }

            private static string Key(BuiltProgram p) => !string.IsNullOrEmpty(p.Id) ? "id:" + p.Id : "name:" + p.Name;

            // ── Entry ─────────────────────────────────────────────────────────

            public List<ValidationProblem> Run()
            {
                CollectSymbols();

                ValidateVariables(_main, "", baseScope: null);

                _callStack.Add(_main);
                var body = new Body();
                WalkList(_main.Steps ?? new(), "steps", new List<List<ProgramStep>>(), body);
                _callStack.RemoveAt(_callStack.Count - 1);

                ReportUnusedVariables();
                return _problems;
            }

            // ── Problems ──────────────────────────────────────────────────────

            private void Add(string code, string message, string path, string? stepId = null,
                             string? field = null, string severity = ValidationSeverity.Error) =>
                _problems.Add(new ValidationProblem
                {
                    StepId = stepId, StepPath = path, Field = field,
                    Severity = severity, Code = code, Message = message,
                });

            /// <summary>Where a problem is reported: a step (id + path) or a program-level location.</summary>
            private readonly record struct At(string Path, string? StepId)
            {
                public static At Step(ProgramStep s, string path) => new(path, s.Id);
            }

            private void Add(At at, string code, string message, string? field = null,
                             string severity = ValidationSeverity.Error) =>
                Add(code, message, at.Path, at.StepId, field, severity);

            // ── Symbols ───────────────────────────────────────────────────────

            private BuiltProgram? FindProgram(string? id, string? name)
            {
                // The program being validated may be unsaved — a call to itself must see this
                // version, not the one on disk.
                if (!string.IsNullOrEmpty(id) && id == _main.Id) return _main;
                if (string.IsNullOrEmpty(id) && !string.IsNullOrEmpty(name) &&
                    string.Equals(name, _main.Name, StringComparison.OrdinalIgnoreCase)) return _main;
                if (_ctx.FindProgram == null) return null;
                var p = _ctx.FindProgram(string.IsNullOrEmpty(id) ? null : id, name);
                if (p != null && ((!string.IsNullOrEmpty(p.Id) && p.Id == _main.Id) ||
                                  (string.IsNullOrEmpty(_main.Id) && string.Equals(p.Name, _main.Name, StringComparison.OrdinalIgnoreCase))))
                    return _main;
                return p;
            }

            private static Sym? SymOf(ProgramVariable v)
            {
                if (string.IsNullOrWhiteSpace(v.Name)) return null;
                // First, as at run time: a computed variable never gets storage of another kind.
                if (v.IsComputed == true) return new Sym(SymKind.Computed);
                if (v.ToListVar() is { } lv) return new Sym(SymKind.List, lv.ElementType);
                if (v.IsStopwatch == true) return new Sym(SymKind.Stopwatch);
                if (v.IsString == true)    return new Sym(SymKind.String);
                if (v.IsImage == true)     return new Sym(SymKind.Image);
                return new Sym(v.IsBoolean == true ? SymKind.Boolean : SymKind.Number);
            }

            /// <summary>
            /// First pass: the variables of the program and every routine it reaches (they
            /// share one scope at run time), and the names steps write without a declaration.
            /// </summary>
            private void CollectSymbols()
            {
                var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                var queue = new Queue<BuiltProgram>();
                queue.Enqueue(_main);
                seen.Add(Key(_main));
                while (queue.Count > 0)
                {
                    var p = queue.Dequeue();
                    foreach (var v in p.Variables ?? [])
                    {
                        if (SymOf(v) is not { } sym) continue;
                        var name = v.Name.Trim();
                        _declared.TryAdd(name, sym);
                        if (v.IsGlobal == true) _globalNames.Add(name);
                        if (sym.Kind == SymKind.Computed)
                            _computedFormulas.TryAdd(name, (v.ValueExpression ?? "", v.IsGlobal == true));
                    }

                    foreach (var s in Descend(p.Steps ?? new()))
                    {
                        CollectImplicit(s);
                        if (s.Type == StepType.CallRoutine && FindProgram(s.RoutineId, s.RoutineName) is { } r
                            && seen.Add(Key(r)))
                            queue.Enqueue(r);
                    }
                }
            }

            /// <summary>Every enabled step in a list, nested bodies included (routine bodies not).</summary>
            private static IEnumerable<ProgramStep> Descend(List<ProgramStep> steps)
            {
                foreach (var s in steps)
                {
                    if (s == null || !s.IsEnabled) continue;
                    yield return s;
                    foreach (var child in Children(s))
                        foreach (var c in Descend(child.Steps)) yield return c;
                }
            }

            private static IEnumerable<(string Field, List<ProgramStep> Steps)> Children(ProgramStep s)
            {
                if (s.LoopSteps != null) yield return ("loopSteps", s.LoopSteps);
                if (s.IfSteps   != null) yield return ("ifSteps",   s.IfSteps);
                if (s.ElseIfBranches != null)
                    for (int j = 0; j < s.ElseIfBranches.Count; j++)
                        if (s.ElseIfBranches[j]?.Steps != null)
                            yield return ($"elseIfBranches[{j}].steps", s.ElseIfBranches[j].Steps);
                if (s.ElseSteps != null) yield return ("elseSteps", s.ElseSteps);
                if (s.CncSpec == null && s.CncProgramSteps != null) yield return ("cncProgramSteps", s.CncProgramSteps);
            }

            private void Implicit(string? name, SymKind kind, ListElementType et = ListElementType.Record)
            {
                if (string.IsNullOrWhiteSpace(name)) return;
                name = name.Trim().TrimStart('$');
                _implicit.TryAdd(name, new Sym(kind, et));
            }

            private void CollectImplicit(ProgramStep s)
            {
                switch (s.Type)
                {
                    case StepType.SetVariable:      Implicit(s.VariableName, SymKind.Number); break;
                    case StepType.StopwatchControl: Implicit(s.StopwatchVariableName, SymKind.Stopwatch); break;
                    case StepType.CaptureImage:     Implicit(s.CaptureImageVariableName, SymKind.Image); break;
                    case StepType.Wait:             Implicit(s.WaitTimeoutVariableName, SymKind.Boolean); break;
                }
                foreach (var o in s.VisionOutputs ?? [])
                {
                    Implicit(o.CountVar, SymKind.Number);
                    Implicit(o.PointsVar, SymKind.List, ListElementType.Point);
                    Implicit(o.DetectedVar, SymKind.Boolean);
                }
                foreach (var o in s.ColorOutputs ?? [])
                {
                    Implicit(o.CoverageVar, SymKind.Number);
                    Implicit(o.PassedVar, SymKind.Boolean);
                    Implicit(o.CellsVar, SymKind.List, ListElementType.Record);
                    Implicit(o.CellsPassedVar, SymKind.Number);
                }
                foreach (var o in s.PolygonOutputs ?? [])
                {
                    Implicit(o.CountVar, SymKind.Number); Implicit(o.FoundVar, SymKind.Boolean);
                    Implicit(o.AngleVar, SymKind.Number); Implicit(o.CenterXVar, SymKind.Number);
                    Implicit(o.CenterYVar, SymKind.Number);
                }
                foreach (var o in s.ArucoOutputs ?? [])
                {
                    Implicit(o.CountVar, SymKind.Number); Implicit(o.FoundVar, SymKind.Boolean);
                    Implicit(o.FirstIdVar, SymKind.Number); Implicit(o.FirstCenterXVar, SymKind.Number);
                    Implicit(o.FirstCenterYVar, SymKind.Number);
                }
                foreach (var m in s.JsonInbound ?? [])        Implicit(m.VariableName, SymKind.Number);
                foreach (var m in s.HttpReceiveInbound ?? []) Implicit(m.VariableName, SymKind.Number);
            }

            private Sym? Lookup(string name)
            {
                if (_initScope != null)
                    return _initScope.TryGetValue(name, out var i) ? i : null;
                if (_declared.TryGetValue(name, out var d)) return d;
                if (_implicit.TryGetValue(name, out var m)) return m;
                foreach (var l in _loopScope)
                    if (string.Equals(l, name, StringComparison.OrdinalIgnoreCase)) return new Sym(SymKind.Number);
                return null;
            }

            private bool IsScalar(string name) =>
                name.Equals("time_ms", StringComparison.OrdinalIgnoreCase) ||
                Lookup(name) is { Kind: SymKind.Number or SymKind.Boolean or SymKind.Stopwatch or SymKind.Computed };

            private bool IsList(string name) => Lookup(name) is { Kind: SymKind.List };

            private bool IsIo(string name) => _ctx.IoNames.Contains(name) || IoPattern.IsMatch(name);

            private bool IsProperty(string name) => _ctx.PropertyNames.Contains(name);

            private static bool LooksLikeProperty(string name)
            {
                int dot = name.IndexOf('.');
                return dot > 0 && RobotPropertySource.Roots.Contains(name[..dot]);
            }

            private void Use(string name)
            {
                if (string.IsNullOrWhiteSpace(name)) return;
                name = name.Trim().TrimStart('$');
                _used.Add(name);
                int dot = name.IndexOf('.');
                if (dot > 0) _used.Add(name[..dot]);
            }

            // ── Variables ─────────────────────────────────────────────────────

            /// <param name="pathPrefix">"" for the main program, "steps[3].routine(X)." for a routine.</param>
            /// <param name="baseScope">What a routine's initialisers can already see (the caller's
            /// variables); null for the main program.</param>
            private void ValidateVariables(BuiltProgram p, string pathPrefix, Dictionary<string, Sym>? baseScope)
            {
                var vars = p.Variables ?? [];
                var scope = baseScope != null
                    ? new Dictionary<string, Sym>(baseScope, StringComparer.OrdinalIgnoreCase)
                    : new Dictionary<string, Sym>(StringComparer.OrdinalIgnoreCase);

                for (int i = 0; i < vars.Count; i++)
                {
                    var v = vars[i];
                    var at = new At($"{pathPrefix}variables[{i}]", null);
                    if (v == null) continue;
                    if (string.IsNullOrWhiteSpace(v.Name))
                    {
                        Add(at, ValidationCodes.MissingField, "Variable has no name", "name");
                        continue;
                    }
                    var name = v.Name.Trim();
                    if (IsProperty(name) || LooksLikeProperty(name) || IsIo(name))
                        Add(at, ValidationCodes.ReadOnlyProperty,
                            $"'{name}' is a read-only {(IsIo(name) ? "IO value" : "property")} and cannot be declared as a variable", "name");

                    if (!string.IsNullOrWhiteSpace(v.ValueExpression) && SymOf(v) is { Kind: SymKind.Number or SymKind.Boolean })
                    {
                        // Initialisers run as each variable is registered, in declaration order.
                        var saved = _initScope;
                        _initScope = scope;
                        try { CheckExpr(v.ValueExpression, at, "valueExpression"); }
                        finally { _initScope = saved; }
                    }

                    if (v.IsComputed == true) ValidateComputed(v, name, at);

                    if (SymOf(v) is { } sym) scope[name] = sym;
                }
            }

            // ── Computed variables (§7) ───────────────────────────────────────

            private void ValidateComputed(ProgramVariable v, string name, At at)
            {
                var conflicts = new List<string>();
                if (v.IsPersistent == true) conflicts.Add("persistent");
                if (v.IsString == true)     conflicts.Add("text");
                if (v.IsImage == true)      conflicts.Add("image");
                if (v.IsStopwatch == true)  conflicts.Add("stopwatch");
                if (v.Items != null || v.Values != null || v.Points != null || v.Objects != null) conflicts.Add("list");
                if (conflicts.Count > 0)
                    Add(at, ValidationCodes.ComputedKindConflict,
                        $"'${name}' is computed (a formula with no stored value) and cannot also be {string.Join(", ", conflicts)}",
                        "isComputed");

                var formula = v.ValueExpression;
                if (string.IsNullOrWhiteSpace(formula))
                {
                    Add(at, ValidationCodes.MissingField, $"Computed variable '${name}' has no formula", "valueExpression");
                    return;
                }

                // A formula is evaluated whenever it is read, so it sees every variable — not
                // just the ones declared above it, as an initial value does.
                var saved = _initScope;
                _initScope = null;
                try { CheckExpr(formula, at, "valueExpression"); }
                finally { _initScope = saved; }

                if (v.IsGlobal == true) CheckGlobalComputedScope(formula, name, at);

                if (ComputedCycleThrough(name) is { } cycle)
                    Add(at, ValidationCodes.ComputedCycle,
                        $"Computed variable '${name}' depends on itself ({string.Join(" → ", cycle.Select(n => "$" + n))})",
                        "valueExpression");
            }

            /// <summary>
            /// A global computed formula is evaluated against the global store, IO and properties
            /// only, so anything program-local it names would be unknown in another program.
            /// </summary>
            private void CheckGlobalComputedScope(string formula, string name, At at)
            {
                List<ExprRef> refs;
                try { refs = ExpressionEvaluator.References(formula); }
                catch (ExpressionParseException) { return; } // already reported
                var reported = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                foreach (var r in refs)
                {
                    if (r.Kind is not (ExprRefKind.Plain or ExprRefKind.Indexed or ExprRefKind.ListArgument)) continue;
                    string refName = r.Name;
                    if (r.Kind == ExprRefKind.Plain && IsList(r.Parts[0])) refName = r.Parts[0];
                    else if (r.Kind == ExprRefKind.Plain &&
                             (IsIo(refName) || IsProperty(refName) || refName.Equals("time_ms", StringComparison.OrdinalIgnoreCase)))
                        continue;

                    var sym = Lookup(refName);
                    if (sym == null) continue; // unknown — reported by CheckExpr
                    // Lists are never shared between programs; a scalar is when declared global.
                    bool shared = sym.Kind != SymKind.List && _globalNames.Contains(refName);
                    if (shared || !reported.Add(refName)) continue;
                    Add(at, ValidationCodes.ComputedGlobalScope,
                        $"Global computed variable '${name}' uses '${refName}', which is not a global variable — " +
                        "other programs cannot see it", "valueExpression");
                }
            }

            /// <summary>The dependency loop from <paramref name="start"/> back to itself, or null.</summary>
            private List<string>? ComputedCycleThrough(string start)
            {
                var path = new List<string> { start };
                var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

                bool Walk(string n)
                {
                    foreach (var dep in ComputedDependencies(n))
                    {
                        if (string.Equals(dep, start, StringComparison.OrdinalIgnoreCase))
                        {
                            path.Add(dep);
                            return true;
                        }
                        if (!visited.Add(dep)) continue;
                        path.Add(dep);
                        if (Walk(dep)) return true;
                        path.RemoveAt(path.Count - 1);
                    }
                    return false;
                }

                return Walk(start) ? path : null;
            }

            /// <summary>The computed variables a computed variable's formula reads directly.</summary>
            private IEnumerable<string> ComputedDependencies(string name)
            {
                if (!_computedFormulas.TryGetValue(name, out var c)) return [];
                try
                {
                    return ExpressionEvaluator.References(c.Formula)
                        .Where(r => r.Kind is ExprRefKind.Plain or ExprRefKind.Indexed or ExprRefKind.ListArgument)
                        .Select(r => r.Name)
                        .Where(_computedFormulas.ContainsKey)
                        .Distinct(StringComparer.OrdinalIgnoreCase)
                        .ToList();
                }
                catch (ExpressionParseException) { return []; }
            }

            private void ReportUnusedVariables()
            {
                var vars = _main.Variables ?? [];
                for (int i = 0; i < vars.Count; i++)
                {
                    var v = vars[i];
                    if (v == null || string.IsNullOrWhiteSpace(v.Name)) continue;
                    // A global is shared with other programs; this one need not touch it.
                    if (v.IsGlobal == true) continue;
                    if (_used.Contains(v.Name.Trim())) continue;
                    Add(ValidationCodes.UnusedVariable, $"Variable '${v.Name.Trim()}' is never used",
                        $"variables[{i}]", null, "name", ValidationSeverity.Warning);
                }
            }

            // ── Steps ─────────────────────────────────────────────────────────

            /// <summary>Label bookkeeping for one program or routine body.</summary>
            private sealed class Body
            {
                public readonly Dictionary<string, string> LabelIds   = new(StringComparer.Ordinal);
                public readonly Dictionary<string, string> LabelNames = new(StringComparer.OrdinalIgnoreCase);
            }

            private void WalkList(List<ProgramStep> steps, string path, List<List<ProgramStep>> ancestors, Body body)
            {
                var chain = new List<List<ProgramStep>>(ancestors) { steps };

                // Labels first, so duplicates are reported whatever order the walk takes.
                for (int i = 0; i < steps.Count; i++)
                {
                    var s = steps[i];
                    if (s?.Type != StepType.Label) continue;
                    var at = At.Step(s, $"{path}[{i}]");
                    if (!string.IsNullOrEmpty(s.LabelId))
                    {
                        if (body.LabelIds.TryGetValue(s.LabelId, out var first))
                            Add(at, ValidationCodes.DuplicateLabel, $"Label id '{s.LabelId}' is also used at {first}", "labelId");
                        else body.LabelIds[s.LabelId] = at.Path;
                    }
                    if (!string.IsNullOrWhiteSpace(s.LabelName))
                    {
                        if (body.LabelNames.TryGetValue(s.LabelName.Trim(), out var first))
                            Add(at, ValidationCodes.DuplicateLabel, $"Label '{s.LabelName.Trim()}' is also defined at {first}", "labelName");
                        else body.LabelNames[s.LabelName.Trim()] = at.Path;
                    }
                }

                bool unreachable = false;
                string? unreachableCause = null;
                for (int i = 0; i < steps.Count; i++)
                {
                    var s = steps[i];
                    string sp = $"{path}[{i}]";
                    if (s == null) continue;
                    var at = At.Step(s, sp);

                    // A Label can be jumped to, so it ends an unreachable run.
                    if (s.Type == StepType.Label) unreachable = false;
                    else if (unreachable)
                    {
                        Add(at, ValidationCodes.UnreachableStep,
                            $"This step can never run: it follows {unreachableCause} with no Label in between",
                            severity: ValidationSeverity.Warning);
                        unreachable = false; // one warning per run of dead steps
                        unreachableCause = null;
                        // keep checking the step itself — it may become reachable once edited
                        SetTerminator(s, ref unreachable, ref unreachableCause);
                    }

                    if (!s.IsEnabled)
                    {
                        Add(at, ValidationCodes.DisabledStep, "Step is disabled and will be skipped",
                            "enabled", ValidationSeverity.Warning);
                        continue;
                    }

                    ValidateStep(s, sp, at, chain, body);

                    if (!unreachable) SetTerminator(s, ref unreachable, ref unreachableCause);
                }
            }

            /// <summary>A step after which the rest of the list cannot run: an unconditional
            /// GoToLabel, or a counted loop that runs forever.</summary>
            private static void SetTerminator(ProgramStep s, ref bool unreachable, ref string? cause)
            {
                if (!s.IsEnabled) return;
                if (s.Type == StepType.GoToLabel)
                {
                    unreachable = true;
                    cause = $"Go To '{s.LabelName ?? s.LabelId}'";
                }
                else if (s.Type == StepType.Loop && (s.LoopMode is null or "count") && s.LoopCount == 0
                         && s.Expressions?.ContainsKey("loopCount") != true && (s.LoopSteps?.Count ?? 0) > 0)
                {
                    unreachable = true;
                    cause = "an infinite loop";
                }
            }

            private void ValidateStep(ProgramStep s, string sp, At at, List<List<ProgramStep>> chain, Body body)
            {
                // Every step may carry numeric field expressions.
                if (s.Expressions != null)
                    foreach (var (key, expr) in s.Expressions)
                    {
                        if (string.IsNullOrWhiteSpace(expr))
                            Add(at, ValidationCodes.MissingField,
                                $"Expression for '{key}' is empty — it evaluates to 0 instead of the typed value",
                                $"expressions.{key}", ValidationSeverity.Warning);
                        else
                            CheckExpr(expr, at, $"expressions.{key}");
                    }

                switch (s.Type)
                {
                    case StepType.MoveL: case StepType.MoveJ: case StepType.JumpL: case StepType.JumpJ:
                        CheckMoveTarget(s, at);
                        if (!string.IsNullOrEmpty(s.LocalName) && _ctx.LocalExists != null && !_ctx.LocalExists(s.LocalName))
                            Add(at, ValidationCodes.UnknownLocal, $"Local '{s.LocalName}' does not exist", "localName");
                        break;

                    case StepType.SetOutput:
                        if (s.OutputNumber == null)
                            Add(at, ValidationCodes.MissingField, "No output number is set", "outputNumber");
                        break;

                    case StepType.Wait:
                        if (s.WaitMode == "condition")
                            CheckCondition(s.WaitCondition, at, "waitCondition", required: true);
                        CheckTarget(s.WaitTimeoutVariableName, at, "waitTimeoutVariableName", requireDeclared: false);
                        break;

                    case StepType.Loop:
                        ValidateLoop(s, sp, at, chain, body);
                        return; // children walked there, with the loop variables in scope

                    case StepType.IfCondition:
                        CheckCondition(s.Condition, at, "condition", required: true);
                        for (int j = 0; j < (s.ElseIfBranches?.Count ?? 0); j++)
                            if (s.ElseIfBranches![j] is { } b)
                                CheckCondition(b.Condition, at, $"elseIfBranches[{j}].condition", required: true);
                        bool anyElse = (s.ElseSteps?.Count ?? 0) > 0 || (s.ElseIfBranches?.Any(b => b?.Steps?.Count > 0) ?? false);
                        if ((s.IfSteps?.Count ?? 0) == 0 && !anyElse)
                            Add(at, ValidationCodes.EmptyBranch, "If has no steps in any branch", "ifSteps");
                        break;

                    case StepType.StatusUpdate:
                        CheckTemplate(s.StatusMessage, at, "statusMessage", ValidationSeverity.Warning);
                        CheckTemplate(s.StatusWarning, at, "statusWarning", ValidationSeverity.Warning);
                        CheckTemplate(s.StatusError,   at, "statusError",   ValidationSeverity.Warning);
                        break;

                    case StepType.CallRoutine:
                        ValidateCall(s, sp, at, chain);
                        break;

                    case StepType.SetVariable:
                        ValidateSetVariable(s, at);
                        break;

                    case StepType.Label:
                        if (string.IsNullOrEmpty(s.LabelId))
                            Add(at, ValidationCodes.MissingField, "Label has no id", "labelId");
                        break;

                    case StepType.GoToLabel:
                        if (string.IsNullOrEmpty(s.LabelId))
                            Add(at, ValidationCodes.MissingField, "Go To has no label selected", "labelId");
                        else if (!chain.Any(list => list.Any(x => x?.Type == StepType.Label && x.LabelId == s.LabelId)))
                            Add(at, ValidationCodes.UnknownLabel,
                                $"Label '{s.LabelName ?? s.LabelId}' is not in this step list or any list enclosing it",
                                "labelId");
                        break;

                    case StepType.SetTool:
                        if (!string.IsNullOrEmpty(s.ToolName) && s.ToolName != "none" &&
                            _ctx.ToolExists != null && !_ctx.ToolExists(s.ToolName))
                            Add(at, ValidationCodes.UnknownTool, $"Tool '{s.ToolName}' does not exist", "toolName");
                        break;

                    case StepType.SetLocal:
                        if (!string.IsNullOrEmpty(s.LocalName) && _ctx.LocalExists != null && !_ctx.LocalExists(s.LocalName))
                            Add(at, ValidationCodes.UnknownLocal, $"Local '{s.LocalName}' does not exist", "localName");
                        break;

                    case StepType.RunVision:
                        if (string.IsNullOrEmpty(s.VisionProgramId))
                            Add(at, ValidationCodes.MissingField, "No vision program is selected", "visionProgramId");
                        else if (_ctx.VisionProgramExists != null && !_ctx.VisionProgramExists(s.VisionProgramId))
                            Add(at, ValidationCodes.UnknownVisionProgram,
                                $"Vision program '{s.VisionProgramName ?? s.VisionProgramId}' does not exist", "visionProgramId");
                        if (!string.IsNullOrWhiteSpace(s.VisionZoneVar)) CheckRead(s.VisionZoneVar, at, "visionZoneVar");
                        CheckVisionTargets(s, at);
                        break;

                    case StepType.StartBackground: case StepType.StopBackground: case StepType.WaitForBackground:
                        if (string.IsNullOrEmpty(s.BackgroundProgramId) && string.IsNullOrEmpty(s.BackgroundProgramName))
                            Add(at, ValidationCodes.MissingField, "No background program is selected", "backgroundProgramName");
                        else if (_ctx.FindProgram != null && FindProgram(s.BackgroundProgramId, s.BackgroundProgramName) == null)
                            Add(at, ValidationCodes.UnknownProgram,
                                $"Program '{s.BackgroundProgramName ?? s.BackgroundProgramId}' does not exist", "backgroundProgramName");
                        break;

                    case StepType.StopwatchControl:
                        if (string.IsNullOrWhiteSpace(s.StopwatchVariableName))
                            Add(at, ValidationCodes.MissingField, "No stopwatch variable is selected", "stopwatchVariableName");
                        else CheckTarget(s.StopwatchVariableName, at, "stopwatchVariableName", requireDeclared: false);
                        if (s.StopwatchAction is not ("Start" or "Stop" or "Reset"))
                            Add(at, ValidationCodes.MissingField, "Stopwatch action must be Start, Stop or Reset", "stopwatchAction");
                        break;

                    case StepType.SaveImage:
                        if (string.IsNullOrWhiteSpace(s.SaveImagePath))
                            Add(at, ValidationCodes.MissingField, "No image path is set", "saveImagePath");
                        else CheckTemplate(s.SaveImagePath, at, "saveImagePath", ValidationSeverity.Warning);
                        break;

                    case StepType.CaptureImage:
                        if (string.IsNullOrWhiteSpace(s.CaptureImageVariableName))
                            Add(at, ValidationCodes.MissingField, "No image variable is selected", "captureImageVariableName");
                        else CheckTarget(s.CaptureImageVariableName, at, "captureImageVariableName", requireDeclared: false);
                        break;

                    case StepType.HttpRequest:
                        if (string.IsNullOrWhiteSpace(s.JsonUrl))
                            Add(at, ValidationCodes.MissingField, "No URL is set", "jsonUrl");
                        CheckOutbound(s, at);
                        for (int j = 0; j < (s.JsonInbound?.Count ?? 0); j++)
                            CheckTarget(s.JsonInbound![j]?.VariableName, at, $"jsonInbound[{j}].variableName", requireDeclared: false);
                        break;

                    case StepType.HttpReceive:
                        if (string.IsNullOrWhiteSpace(s.HttpReceiveName))
                            Add(at, ValidationCodes.MissingField, "No receive name is set", "httpReceiveName");
                        for (int j = 0; j < (s.HttpReceiveInbound?.Count ?? 0); j++)
                            CheckTarget(s.HttpReceiveInbound![j]?.VariableName, at, $"httpReceiveInbound[{j}].variableName", requireDeclared: false);
                        break;

                    case StepType.CncProgram:
                        if (s.CncSpec == null && (s.CncProgramSteps == null || s.CncProgramSteps.Count == 0))
                            Add(at, ValidationCodes.MissingField, "CNC block has no toolpath", "cncSpec");
                        if (s.CncSpec?.Expressions != null)
                            foreach (var (key, expr) in s.CncSpec.Expressions)
                                if (!string.IsNullOrWhiteSpace(expr))
                                    CheckExpr(expr, at, $"cncSpec.expressions.{key}");
                        break;

                    case StepType.Unknown:
                        Add(at, ValidationCodes.UnknownStepType,
                            $"Step type '{s.UnknownStepType ?? "?"}' is not known to this controller; it will be skipped",
                            "type", ValidationSeverity.Warning);
                        break;
                }

                // Nested bodies (if/else branches, legacy CNC steps).
                foreach (var (field, steps) in Children(s))
                    WalkList(steps, $"{sp}.{field}", chain, body);
            }

            private void ValidateLoop(ProgramStep s, string sp, At at, List<List<ProgramStep>> chain, Body body)
            {
                int pushed = 0;
                void Scope(string? name)
                {
                    if (string.IsNullOrWhiteSpace(name)) return;
                    _loopScope.Add(name.Trim().TrimStart('$'));
                    pushed++;
                }

                if (s.LoopMode == "forEach")
                {
                    if (string.IsNullOrWhiteSpace(s.ForEachVariableName))
                        Add(at, ValidationCodes.MissingField, "For-each loop has no list selected", "forEachVariableName");
                    else
                    {
                        Use(s.ForEachVariableName);
                        if (!IsList(s.ForEachVariableName.Trim()))
                            Add(at, ValidationCodes.UnknownVariable,
                                Lookup(s.ForEachVariableName.Trim()) != null
                                    ? $"'${s.ForEachVariableName.Trim()}' is not a list"
                                    : $"Unknown list variable '${s.ForEachVariableName.Trim()}'",
                                "forEachVariableName");
                    }
                    CheckTarget(s.ForEachValueVariableName, at, "forEachValueVariableName", requireDeclared: false);
                    CheckTarget(s.ForEachIndexVariableName, at, "forEachIndexVariableName", requireDeclared: false);
                    Scope(s.ForEachValueVariableName);
                    Scope(s.ForEachIndexVariableName);
                }
                else if (s.LoopMode == "while")
                {
                    CheckCondition(s.LoopWhileCondition, at, "loopWhileCondition", required: true);
                }
                else
                {
                    CheckTarget(s.ForEachIndexVariableName, at, "forEachIndexVariableName", requireDeclared: false);
                    Scope(s.ForEachIndexVariableName);
                }

                if (s.LoopSteps == null || s.LoopSteps.Count == 0)
                    Add(at, ValidationCodes.EmptyLoop, "Loop has no steps", "loopSteps");
                else
                    WalkList(s.LoopSteps, $"{sp}.loopSteps", chain, body);

                _loopScope.RemoveRange(_loopScope.Count - pushed, pushed);
            }

            private void ValidateCall(ProgramStep s, string sp, At at, List<List<ProgramStep>> chain)
            {
                if (string.IsNullOrEmpty(s.RoutineId) && string.IsNullOrEmpty(s.RoutineName))
                {
                    Add(at, ValidationCodes.MissingField, "No routine is selected", "routineName");
                    return;
                }
                if (_ctx.FindProgram == null && !ReferencesMain(s)) return; // cannot check offline
                var routine = FindProgram(s.RoutineId, s.RoutineName);
                if (routine == null)
                {
                    Add(at, ValidationCodes.UnknownRoutine, $"Routine '{s.RoutineName ?? s.RoutineId}' does not exist", "routineName");
                    return;
                }

                if (_callStack.Any(p => Key(p) == Key(routine)))
                {
                    var loop = string.Join(" → ", _callStack.SkipWhile(p => Key(p) != Key(routine)).Select(p => p.Name).Append(routine.Name));
                    Add(at, ValidationCodes.RoutineRecursion,
                        $"Calling '{routine.Name}' here recurses ({loop}); the program would never finish", "routineName");
                    return;
                }

                // Each routine body is checked once — its problems are its own, not the caller's.
                if (!_validatedRoutines.Add(Key(routine))) return;

                string prefix = $"{sp}.routine({routine.Name}).";
                ValidateVariables(routine, prefix, baseScope: new Dictionary<string, Sym>(
                    _declared.Concat(_implicit).GroupBy(kv => kv.Key, StringComparer.OrdinalIgnoreCase)
                             .ToDictionary(g => g.Key, g => g.First().Value, StringComparer.OrdinalIgnoreCase),
                    StringComparer.OrdinalIgnoreCase));

                _callStack.Add(routine);
                // Routine frames sit on top of the caller's at run time, so a GoToLabel in the
                // routine can reach the caller's labels too.
                WalkList(routine.Steps ?? new(), prefix + "steps", chain, new Body());
                _callStack.RemoveAt(_callStack.Count - 1);
            }

            private bool ReferencesMain(ProgramStep s) =>
                (!string.IsNullOrEmpty(s.RoutineId) && s.RoutineId == _main.Id) ||
                (string.IsNullOrEmpty(s.RoutineId) && string.Equals(s.RoutineName, _main.Name, StringComparison.OrdinalIgnoreCase));

            private void ValidateSetVariable(ProgramStep s, At at)
            {
                if (string.IsNullOrWhiteSpace(s.VariableName))
                {
                    Add(at, ValidationCodes.MissingField, "No variable is selected", "variableName");
                    return;
                }
                var name = s.VariableName.Trim().TrimStart('$');
                if (!CheckTarget(name, at, "variableName", requireDeclared: true)) return;

                var sym = Lookup(name);
                if (sym is { Kind: SymKind.List or SymKind.Image })
                    Add(at, ValidationCodes.UnknownVariable,
                        $"'${name}' is {(sym.Kind == SymKind.List ? "a list" : "an image")} — Set Variable assigns numbers and text only",
                        "variableName");

                if (string.IsNullOrWhiteSpace(s.VariableExpr))
                    Add(at, ValidationCodes.MissingField, "No value expression is set", "variableExpr");
                else if (sym is { Kind: SymKind.String })
                    CheckTemplate(s.VariableExpr, at, "variableExpr", ValidationSeverity.Warning);
                else
                    CheckExpr(s.VariableExpr, at, "variableExpr");
            }

            private void CheckVisionTargets(ProgramStep s, At at)
            {
                void T(string? n, string f) => CheckTarget(n, at, f, requireDeclared: false);
                for (int j = 0; j < (s.VisionOutputs?.Count ?? 0); j++)
                {
                    var o = s.VisionOutputs![j]; if (o == null) continue;
                    T(o.CountVar, $"visionOutputs[{j}].countVar"); T(o.PointsVar, $"visionOutputs[{j}].pointsVar");
                    T(o.DetectedVar, $"visionOutputs[{j}].detectedVar");
                }
                for (int j = 0; j < (s.ColorOutputs?.Count ?? 0); j++)
                {
                    var o = s.ColorOutputs![j]; if (o == null) continue;
                    T(o.CoverageVar, $"colorOutputs[{j}].coverageVar"); T(o.PassedVar, $"colorOutputs[{j}].passedVar");
                    T(o.CellsVar, $"colorOutputs[{j}].cellsVar"); T(o.CellsPassedVar, $"colorOutputs[{j}].cellsPassedVar");
                }
                for (int j = 0; j < (s.PolygonOutputs?.Count ?? 0); j++)
                {
                    var o = s.PolygonOutputs![j]; if (o == null) continue;
                    T(o.CountVar, $"polygonOutputs[{j}].countVar"); T(o.FoundVar, $"polygonOutputs[{j}].foundVar");
                    T(o.AngleVar, $"polygonOutputs[{j}].angleVar"); T(o.CenterXVar, $"polygonOutputs[{j}].centerXVar");
                    T(o.CenterYVar, $"polygonOutputs[{j}].centerYVar");
                }
                for (int j = 0; j < (s.ArucoOutputs?.Count ?? 0); j++)
                {
                    var o = s.ArucoOutputs![j]; if (o == null) continue;
                    T(o.CountVar, $"arucoOutputs[{j}].countVar"); T(o.FoundVar, $"arucoOutputs[{j}].foundVar");
                    T(o.FirstIdVar, $"arucoOutputs[{j}].firstIdVar"); T(o.FirstCenterXVar, $"arucoOutputs[{j}].firstCenterXVar");
                    T(o.FirstCenterYVar, $"arucoOutputs[{j}].firstCenterYVar");
                }
            }

            private void CheckOutbound(ProgramStep s, At at)
            {
                for (int j = 0; j < (s.JsonOutbound?.Count ?? 0); j++)
                {
                    var kv = s.JsonOutbound![j];
                    if (kv == null || string.IsNullOrWhiteSpace(kv.Key)) continue;
                    string f = $"jsonOutbound[{j}]";
                    if (!string.IsNullOrWhiteSpace(kv.ListVar))
                    {
                        Use(kv.ListVar);
                        if (!IsList(kv.ListVar.Trim()))
                            Add(at, ValidationCodes.UnknownVariable, $"'${kv.ListVar.Trim()}' is not a list variable", f + ".listVar");
                    }
                    else if (!string.IsNullOrWhiteSpace(kv.ImageVar))
                    {
                        Use(kv.ImageVar);
                        if (Lookup(kv.ImageVar.Trim()) is not { Kind: SymKind.Image })
                            Add(at, ValidationCodes.UnknownVariable, $"'${kv.ImageVar.Trim()}' is not an image variable", f + ".imageVar");
                    }
                    else if (!string.IsNullOrWhiteSpace(kv.Expr))
                        CheckExpr(kv.Expr, at, f + ".expr");
                }
                for (int j = 0; j < (s.JsonImageOutbound?.Count ?? 0); j++)
                {
                    var m = s.JsonImageOutbound![j];
                    if (m == null || string.IsNullOrWhiteSpace(m.VariableName)) continue;
                    Use(m.VariableName);
                    if (Lookup(m.VariableName.Trim()) is not { Kind: SymKind.Image })
                        Add(at, ValidationCodes.UnknownVariable, $"'${m.VariableName.Trim()}' is not an image variable",
                            $"jsonImageOutbound[{j}].variableName");
                }
            }

            private void CheckMoveTarget(ProgramStep s, At at)
            {
                // Only the target the executor will use is checked — same precedence order.
                if (s.GridPoint != null)
                {
                    if (string.IsNullOrEmpty(s.GridPoint.GridId))
                        Add(at, ValidationCodes.MissingField, "No grid is selected", "gridPoint.gridId");
                    else if (_ctx.GridExists != null && !_ctx.GridExists(s.GridPoint.GridId))
                        Add(at, ValidationCodes.UnknownGrid, $"Grid '{s.GridPoint.GridId}' does not exist", "gridPoint.gridId");
                }
                else if (s.StackPoint != null)
                {
                    if (string.IsNullOrEmpty(s.StackPoint.StackId))
                        Add(at, ValidationCodes.MissingField, "No stack is selected", "stackPoint.stackId");
                    else if (_ctx.StackExists != null && !_ctx.StackExists(s.StackPoint.StackId))
                        Add(at, ValidationCodes.UnknownStack, $"Stack '{s.StackPoint.StackId}' does not exist", "stackPoint.stackId");
                }
                else if (!string.IsNullOrEmpty(s.VarPointName))
                {
                    CheckPointList(s.VarPointName, at, "varPointName");
                    if (!string.IsNullOrWhiteSpace(s.VarPointIndex)) CheckExpr(s.VarPointIndex, at, "varPointIndex");
                }
                else if (!string.IsNullOrEmpty(s.PointNameExpr))
                {
                    var expr = s.PointNameExpr;
                    if (MoveTargetResolver.TryParsePointsRef(expr, out var refName, out var idxExpr) && IsList(refName))
                    {
                        CheckPointList(refName, at, "pointNameExpr");
                        if (!string.IsNullOrWhiteSpace(idxExpr)) CheckExpr(idxExpr, at, "pointNameExpr");
                    }
                    else if (expr.Contains('$') || expr.Contains('{'))
                        CheckTemplate(expr, at, "pointNameExpr", ValidationSeverity.Error);
                    else if (_ctx.PointExists != null && !_ctx.PointExists(expr.Trim()))
                        Add(at, ValidationCodes.UnknownPoint, $"Point '{expr.Trim()}' does not exist", "pointNameExpr");
                }
                else if (!string.IsNullOrEmpty(s.PointName))
                {
                    if (_ctx.PointExists != null && !_ctx.PointExists(s.PointName))
                        Add(at, ValidationCodes.UnknownPoint, $"Point '{s.PointName}' does not exist", "pointName");
                }
            }

            private void CheckPointList(string name, At at, string field)
            {
                name = name.Trim().TrimStart('$');
                Use(name);
                var sym = Lookup(name);
                if (sym is not { Kind: SymKind.List })
                    Add(at, ValidationCodes.UnknownVariable,
                        sym == null ? $"Unknown points variable '${name}'" : $"'${name}' is not a points list", field);
                else if (sym.ElementType != ListElementType.Point)
                    Add(at, ValidationCodes.UnknownVariable, $"'${name}' is a {sym.ElementType} list, not a points list", field);
            }

            // ── Names written ─────────────────────────────────────────────────

            /// <summary>
            /// A name a step writes. Properties and IO are read-only. With
            /// <paramref name="requireDeclared"/> (Set Variable) the name must also be declared.
            /// Returns false when the name cannot be written at all.
            /// </summary>
            private bool CheckTarget(string? name, At at, string field, bool requireDeclared)
            {
                if (string.IsNullOrWhiteSpace(name)) return true;
                name = name.Trim().TrimStart('$');
                Use(name);
                if (IsProperty(name) || LooksLikeProperty(name))
                {
                    Add(at, ValidationCodes.ReadOnlyProperty, $"'${name}' is a read-only property and cannot be assigned", field);
                    return false;
                }
                if (IsIo(name))
                {
                    Add(at, ValidationCodes.ReadOnlyProperty,
                        $"'${name}' is an IO value and cannot be assigned — use a Set Output step", field);
                    return false;
                }
                if (_declared.TryGetValue(name, out var declared) && declared.Kind == SymKind.Computed)
                {
                    Add(at, ValidationCodes.ComputedVariable,
                        $"'${name}' is a computed variable (a formula) and cannot be assigned", field);
                    return false;
                }
                if (requireDeclared && !_declared.ContainsKey(name) && !_loopScope.Contains(name, StringComparer.OrdinalIgnoreCase))
                    Add(at, ValidationCodes.UnknownVariable,
                        $"'${name}' is not declared in the program's variables", field);
                return true;
            }

            /// <summary>A plain variable name a step reads (not an expression).</summary>
            private void CheckRead(string name, At at, string field)
            {
                name = name.Trim().TrimStart('$');
                Use(name);
                if (Lookup(name) == null && !IsIo(name) && !IsProperty(name))
                    Add(at, ValidationCodes.UnknownVariable, $"Unknown variable '${name}'", field);
            }

            // ── Expressions ───────────────────────────────────────────────────

            private void CheckExpr(string? expr, At at, string field, string severity = ValidationSeverity.Error)
            {
                if (string.IsNullOrWhiteSpace(expr)) return;
                List<ExprRef> refs;
                try { refs = ExpressionEvaluator.References(expr); }
                catch (ExpressionParseException ex)
                {
                    Add(at, ex.Code, $"{ex.Message} (at position {ex.Position} in '{expr}')", field, severity);
                    return;
                }
                foreach (var r in refs) CheckRef(r, expr, at, field, severity);
            }

            private void CheckRef(ExprRef r, string expr, At at, string field, string severity)
            {
                switch (r.Kind)
                {
                    case ExprRefKind.Function:
                        if (!_functionNames.Contains(r.Name))
                            Add(at, ValidationCodes.UnknownFunction, $"Unknown function '{r.Name}'", field, severity);
                        return;

                    case ExprRefKind.BareWord:
                    {
                        var hint = Lookup(r.Name) != null ? $" — did you mean ${r.Name}?" : "";
                        Add(at, ValidationCodes.ExpressionSyntax,
                            $"'{r.Name}' has no $ — a bare word evaluates to 0{hint}", field, ValidationSeverity.Warning);
                        return;
                    }

                    case ExprRefKind.Indexed:
                    case ExprRefKind.ListArgument:
                    {
                        Use(r.Name);
                        if (IsList(r.Name)) return;
                        var what = r.Kind == ExprRefKind.Indexed ? $"'${r.Name}[…]'" : $"'${r.Name}'";
                        Add(at, ValidationCodes.UnknownVariable,
                            Lookup(r.Name) != null || IsIo(r.Name) || IsProperty(r.Name)
                                ? $"{what}: '${r.Name}' is not a list"
                                : $"Unknown list variable '${r.Name}'", field, severity);
                        return;
                    }

                    case ExprRefKind.Plain:
                    {
                        var name  = r.Name;
                        var parts = r.Parts;
                        Use(name);

                        if (IsScalar(name) || IsIo(name) || IsProperty(name)) return;

                        if (IsList(parts[0]))
                        {
                            if (parts.Length == 1 || (parts.Length == 2 &&
                                (parts[1].Equals("length", StringComparison.OrdinalIgnoreCase) ||
                                 parts[1].Equals("count",  StringComparison.OrdinalIgnoreCase))))
                                return;
                            Add(at, ValidationCodes.UnknownVariable,
                                $"'${name}' — '${parts[0]}' is a list: use ${parts[0]}[i].{parts[^1]} or ${parts[0]}.length",
                                field, severity);
                            return;
                        }

                        if (parts.Length > 1 && RobotPropertySource.Roots.Contains(parts[0]) && Lookup(parts[0]) == null)
                        {
                            Add(at, ValidationCodes.UnknownProperty, $"Unknown property '${name}'", field, severity);
                            return;
                        }

                        var sym = Lookup(name);
                        if (sym is { Kind: SymKind.String or SymKind.Image })
                        {
                            Add(at, ValidationCodes.UnknownVariable,
                                $"'${name}' is {(sym.Kind == SymKind.String ? "a text" : "an image")} variable and has no numeric value",
                                field, severity);
                            return;
                        }

                        if (_initScope != null && (_declared.ContainsKey(name) || _implicit.ContainsKey(name)))
                        {
                            Add(at, ValidationCodes.UnknownVariable,
                                $"'${name}' is not set yet when this initial value is computed — variables initialise in declaration order",
                                field, severity);
                            return;
                        }

                        Add(at, ValidationCodes.UnknownVariable, $"Unknown variable '${name}'", field, severity);
                        return;
                    }
                }
            }

            /// <summary>
            /// A text template ($name, $list[i].x, {expression}) as VariableScope.Interpolate
            /// expands it. At run time an unknown name is left as written rather than failing,
            /// so problems here default to warnings.
            /// </summary>
            private void CheckTemplate(string? text, At at, string field, string severity)
            {
                if (string.IsNullOrEmpty(text)) return;
                foreach (Match m in VariableScope.TemplateToken.Matches(text))
                {
                    if (m.Groups["body"].Success)
                    {
                        var body = m.Groups["body"].Value.Trim();
                        if (body.Length == 0) continue;
                        var bare = VariableScope.BareWord.Matches(body).FirstOrDefault(w => !VariableScope.IsAllowedBareWord(body, w));
                        if (bare != null)
                        {
                            // Not an expression at all (JSON, prose in braces) unless it has a $.
                            if (body.Contains('$'))
                                Add(at, ValidationCodes.ExpressionSyntax,
                                    $"'{m.Value}' contains '{bare.Value}' without $ — it is left as written, not substituted",
                                    field, ValidationSeverity.Warning);
                            continue;
                        }
                        var lone = VariableScope.LoneRef.Match(body);
                        if (lone.Success) CheckTemplateRef(lone, at, field, severity);
                        else CheckExpr(body, at, field, severity);
                    }
                    else
                        CheckTemplateRef(m, at, field, severity);
                }
            }

            private void CheckTemplateRef(Match r, At at, string field, string severity)
            {
                var name = r.Groups["name"].Value;
                Use(name);
                bool hasIndex = r.Groups["idx"].Success;
                if (hasIndex)
                {
                    var idx = r.Groups["idx"].Value.Trim();
                    if (idx.Length > 0) CheckExpr(idx, at, field, severity);
                    if (!IsList(name))
                        Add(at, ValidationCodes.UnknownVariable,
                            Lookup(name) != null ? $"'${name}' is not a list" : $"Unknown list variable '${name}'", field, severity);
                    return;
                }
                if (Lookup(name) != null || name.Equals("time_ms", StringComparison.OrdinalIgnoreCase)) return;
                var hint = RobotPropertySource.Roots.Contains(name)
                    ? $" — properties need braces in text: {{${name}.…}}"
                    : "";
                Add(at, ValidationCodes.UnknownVariable, $"Unknown variable '${name}' — it is left as written{hint}", field, severity);
            }

            // ── Conditions ────────────────────────────────────────────────────

            private void CheckCondition(ConditionGroup? g, At at, string field, bool required)
            {
                if (g == null || g.Items == null || g.Items.Count == 0)
                {
                    if (required)
                        Add(at, ValidationCodes.MissingField, "Condition has no rows", field);
                    return;
                }
                for (int k = 0; k < g.Items.Count; k++)
                {
                    var item = g.Items[k];
                    if (item == null) continue;
                    string f = $"{field}.items[{k}]";
                    if (Array.IndexOf(ConditionOps, item.Operator) < 0)
                        Add(at, ValidationCodes.ExpressionSyntax, $"Unknown comparison operator '{item.Operator}'", f + ".operator");

                    bool isStringOp = item.Operator is "contains" or "startsWith" or "endsWith";
                    var left = item.Left?.Trim() ?? "";
                    bool leftIsString = left.StartsWith('$') && Lookup(left[1..]) is { Kind: SymKind.String };
                    if (isStringOp || leftIsString)
                    {
                        // Text comparison: both sides are templates, and plain text is fine.
                        CheckTemplate(item.Left,  at, f + ".left",  ValidationSeverity.Warning);
                        CheckTemplate(item.Right, at, f + ".right", ValidationSeverity.Warning);
                        continue;
                    }

                    if (string.IsNullOrWhiteSpace(item.Left))
                        Add(at, ValidationCodes.MissingField, "Condition row has no left side", f + ".left");
                    else CheckExpr(item.Left, at, f + ".left");

                    if (string.IsNullOrWhiteSpace(item.Right))
                        Add(at, ValidationCodes.MissingField, "Condition row has no right side", f + ".right");
                    else CheckExpr(item.Right, at, f + ".right");
                }
            }
        }
    }
}
