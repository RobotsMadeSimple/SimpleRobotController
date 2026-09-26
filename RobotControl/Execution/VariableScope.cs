using System.Text.RegularExpressions;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Every program variable one executor can see: scalars (with booleans flagged),
    /// lists, strings, images and stopwatches, plus which of them are shared through the
    /// global stores and which persist to disk.
    /// </summary>
    /// <remarks>
    /// <para><b>Not thread-safe.</b> A scope belongs to one <see cref="ProgramExecutor"/> and is
    /// only ever touched under that executor's control lock — from <c>Update()</c> on the loop
    /// thread and from the locked <c>GetDisplay*</c> readers. The global stores it forwards to
    /// are thread-safe on their own, since several executors share them.</para>
    /// <para>All name lookups are case-insensitive. A scalar declared global lives in the
    /// shared store only; <see cref="Set"/> routes writes there and <see cref="MergedVars"/>
    /// lets the global value win over any local of the same name.</para>
    /// </remarks>
    internal sealed class VariableScope
    {
        private readonly GlobalVariableStore? _globalVars;
        private readonly GlobalImageStore?    _globalImages;
        private readonly Action<Dictionary<string, double>>? _ioSource;

        private readonly Dictionary<string, double> _variables = new(StringComparer.OrdinalIgnoreCase);

        /// <summary>
        /// Every list variable, whatever its elements are. One dictionary rather than three
        /// because a number and a point are both records of named doubles — see ListVar.
        /// </summary>
        private readonly Dictionary<string, ListVar> _listVariables = new(StringComparer.OrdinalIgnoreCase);

        private readonly HashSet<string>            _booleanVariables = new(StringComparer.OrdinalIgnoreCase);
        private readonly Dictionary<string, string> _stringVariables  = new(StringComparer.OrdinalIgnoreCase);
        private readonly Dictionary<string, string> _imageVariables   = new(StringComparer.OrdinalIgnoreCase);

        // How many times each image variable has been written. Deliberately not cleared by
        // Clear(): the counter is what the monitor compares against to decide whether to
        // re-fetch, and resetting it on a re-run would make the second run's first frame
        // look like a revision the monitor had already drawn.
        private readonly Dictionary<string, long> _imageRevisions = new(StringComparer.OrdinalIgnoreCase);

        private readonly HashSet<string> _globalVarNames   = new(StringComparer.OrdinalIgnoreCase);
        private readonly HashSet<string> _globalImageNames = new(StringComparer.OrdinalIgnoreCase);

        // Persistent variables — name → id of the program (main or routine) that declared
        // it, which is the key prefix it was loaded under and must be saved under.
        private readonly Dictionary<string, string> _persistentVarOwners = new(StringComparer.OrdinalIgnoreCase);

        private struct StopwatchEntry { public bool Running; public long AccumMs; public long StartTick; }
        private readonly Dictionary<string, StopwatchEntry> _stopwatches = new(StringComparer.OrdinalIgnoreCase);

        // Computed variables (docs/expressions-and-variables.md §7): name → formula. Never
        // stored as values; resolved on every read through _computedSource. Global computed
        // formulas live in the GlobalVariableStore; _globalComputedNames are the ones this
        // program declared (for the monitor and the write guard).
        private readonly Dictionary<string, string> _computed = new(StringComparer.OrdinalIgnoreCase);
        private readonly HashSet<string> _globalComputedNames = new(StringComparer.OrdinalIgnoreCase);
        private readonly ComputedPropertySource _computedSource;
        private IPropertySource? _robotProperties;

        /// <param name="ioSource">Adds live IO values (stb.in1, relay.1, nano.x.y …) to a
        /// dictionary; null when there is no hardware (tests).</param>
        public VariableScope(GlobalVariableStore? globalVars = null, GlobalImageStore? globalImages = null,
                             Action<Dictionary<string, double>>? ioSource = null)
        {
            _globalVars   = globalVars;
            _globalImages = globalImages;
            _ioSource     = ioSource;
            Eval          = new EvalContext(this);
            _computedSource = new ComputedPropertySource(
                _computed, () => Eval.Vars, () => _listVariables, globalVars, ioSource, () => _robotProperties);
        }

        /// <summary>The per-tick evaluation snapshot over this scope.</summary>
        public EvalContext Eval { get; }

        /// <summary>
        /// The property chain every expression evaluated through this scope consults when a
        /// name is not a variable or IO value: computed variables first (this program's, then
        /// global ones), then the read-only system properties ($robot.x, $program.runCount,
        /// $time.hour …). Resolved lazily at lookup — never copied into the per-tick snapshot.
        /// </summary>
        /// <remarks>
        /// The setter sets the system-property source at the end of the chain (null in tests:
        /// then only variables, IO and computed variables resolve); the getter always returns
        /// the whole chain, so every evaluation path sees computed variables.
        /// </remarks>
        public IPropertySource? Properties
        {
            get => _computedSource;
            set => _robotProperties = value;
        }

        /// <summary>
        /// Whether assigning <paramref name="name"/> is refused because it is a computed
        /// variable: one this program declared (local or global), or a global computed
        /// variable registered by another program that this one has no stored variable for.
        /// </summary>
        public bool IsComputed(string name) =>
            _computed.ContainsKey(name) || _globalComputedNames.Contains(name) ||
            (_globalVars != null && !_variables.ContainsKey(name) && !_globalVarNames.Contains(name)
             && _globalVars.TryGetComputed(name, out _));

        /// <summary>Evaluates a computed variable now; false when <paramref name="name"/> is not one.
        /// Evaluation errors (unknown variable, cycle) propagate.</summary>
        public bool TryEvaluateComputed(string name, out double value) =>
            _computedSource.TryGetComputed(name, out value);

        private void RefuseComputedWrite(string name)
        {
            if (IsComputed(name)) throw new ComputedVariableWriteException(name);
        }

        /// <summary>
        /// Bumped by every change to a scalar value or registration, so <see cref="EvalContext"/>
        /// knows when its cached snapshot is out of date.
        /// </summary>
        public long Version { get; private set; }

        /// <summary>List variables, by reference — expressions index into them directly.</summary>
        public Dictionary<string, ListVar> Lists => _listVariables;

        public bool IsString(string name) => _stringVariables.ContainsKey(name);
        public bool IsImage(string name)  => _imageVariables.ContainsKey(name);

        public void SetString(string name, string value) => _stringVariables[name] = value;
        public bool TryGetString(string name, out string value) => _stringVariables.TryGetValue(name, out value!);

        /// <summary>A program-local scalar only — globals are not consulted.</summary>
        public bool TryGetLocal(string name, out double value) => _variables.TryGetValue(name, out value);

        public void SetList(string name, ListVar list)
        {
            RefuseComputedWrite(name);
            _listVariables[name] = list;
        }

        /// <summary>Clears all program variables (and their registrations). Image revisions survive.</summary>
        public void Clear()
        {
            Version++;
            _variables.Clear();
            _listVariables.Clear();
            _booleanVariables.Clear();
            _stringVariables.Clear();
            _imageVariables.Clear();
            _globalVarNames.Clear();
            _globalImageNames.Clear();
            _persistentVarOwners.Clear();
            _stopwatches.Clear();
            _computed.Clear();
            _globalComputedNames.Clear();
        }

        // ── Scalars ───────────────────────────────────────────────────────────

        /// <summary>
        /// The only writer of scalar values — every step, vision output, stopwatch refresh
        /// and initialiser goes through here so globals are honoured. A computed variable
        /// cannot be assigned: <see cref="ComputedVariableWriteException"/>.
        /// </summary>
        public void Set(string name, double value)
        {
            RefuseComputedWrite(name);
            Version++;
            if (_globalVarNames.Contains(name) && _globalVars != null)
                _globalVars.Set(name, value);
            else
                _variables[name] = value;
        }

        /// <summary>
        /// A fresh merged snapshot of local + global scalars (globals win), plus the built-in
        /// <c>time_ms</c> (current Unix time in milliseconds).
        /// </summary>
        public Dictionary<string, double> MergedVars()
        {
            var merged = new Dictionary<string, double>(_variables, StringComparer.OrdinalIgnoreCase);
            if (_globalVars != null)
                foreach (var kv in _globalVars.Snapshot()) merged[kv.Key] = kv.Value;
            merged["time_ms"] = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            return merged;
        }

        /// <summary>Adds local + global scalars and <c>time_ms</c> to <paramref name="target"/> — the
        /// allocation-free form of <see cref="MergedVars"/> used by <see cref="EvalContext"/>.</summary>
        internal void FillMerged(Dictionary<string, double> target)
        {
            foreach (var kv in _variables) target[kv.Key] = kv.Value;
            _globalVars?.CopyInto(target);
            target["time_ms"] = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
        }

        /// <summary>Adds live IO values to <paramref name="target"/>.</summary>
        internal void FillIo(Dictionary<string, double> target) => _ioSource?.Invoke(target);

        /// <summary><see cref="MergedVars"/> plus live IO values — the full dictionary for
        /// expression evaluation (matches what conditions see).</summary>
        public Dictionary<string, double> EvalVars()
        {
            var vars = MergedVars();
            _ioSource?.Invoke(vars);
            return vars;
        }

        // ── Lists ─────────────────────────────────────────────────────────────

        /// <summary>
        /// A list usable as a move target. Point elements only — a record list may happen to
        /// carry x/y/z, but treating it as a pose was never allowed and guessing here would
        /// turn a typo'd variable name into a move to somewhere unintended.
        /// </summary>
        public bool TryGetPointList(string name, out ListVar list)
        {
            if (_listVariables.TryGetValue(name, out var lv) && lv.ElementType == ListElementType.Point)
            {
                list = lv;
                return true;
            }
            list = null!;
            return false;
        }

        // ── Images ────────────────────────────────────────────────────────────

        public void SetImage(string name, string value)
        {
            _imageVariables[name] = value;
            _imageRevisions[name] = _imageRevisions.TryGetValue(name, out var r) ? r + 1 : 1;
            if (_globalImageNames.Contains(name) && _globalImages != null)
                _globalImages.Set(name, value);
        }

        public string GetImage(string name)
        {
            if (_globalImageNames.Contains(name) && _globalImages != null
                && _globalImages.TryGet(name, out var gv))
                return gv;
            return _imageVariables.TryGetValue(name, out var lv) ? lv : "";
        }

        /// <summary>Write-count for one image, preferring the shared store for globals.</summary>
        public long ImageRevision(string name)
        {
            // Same precedence as GetImage: a global is written by whichever program got
            // there last, which may not be this one, so its count lives in the store.
            if (_globalImageNames.Contains(name) && _globalImages != null
                && _globalImages.TryGetRevision(name, out var gr))
                return gr;
            return _imageRevisions.TryGetValue(name, out var lr) ? lr : 0;
        }

        // ── Stopwatches ───────────────────────────────────────────────────────

        /// <summary>Refreshes every stopwatch variable so expressions see the current elapsed time.</summary>
        public void RefreshStopwatches()
        {
            if (_stopwatches.Count == 0) return;
            var nowTick = Environment.TickCount64;
            foreach (var (name, sw) in _stopwatches)
                Set(name, sw.Running ? sw.AccumMs + (nowTick - sw.StartTick) : sw.AccumMs);
        }

        /// <summary>Applies a Start / Stop / Reset action and writes the new elapsed value.</summary>
        public void ControlStopwatch(string name, string? action)
        {
            RefuseComputedWrite(name);
            if (!_stopwatches.TryGetValue(name, out var sw))
                sw = new StopwatchEntry { Running = false, AccumMs = 0, StartTick = 0 };

            var now = Environment.TickCount64;
            sw = action switch
            {
                "Start" when !sw.Running => new StopwatchEntry { Running = true,  AccumMs = sw.AccumMs, StartTick = now },
                "Stop"  when  sw.Running => new StopwatchEntry { Running = false, AccumMs = sw.AccumMs + (now - sw.StartTick), StartTick = 0 },
                "Reset"                  => new StopwatchEntry { Running = false, AccumMs = 0, StartTick = 0 },
                _                        => sw, // Start when already running / Stop when already stopped — no-op
            };
            _stopwatches[name] = sw;
            Set(name, sw.Running ? sw.AccumMs + (now - sw.StartTick) : sw.AccumMs);
        }

        // ── Declaration / persistence ─────────────────────────────────────────

        /// <summary>
        /// Registers a program's declared variables (lists, stopwatches, strings, images and
        /// scalars, with global/persistent handling). Used for the main program on Start and
        /// for each routine when it is entered, so routine-local variables exist at runtime.
        /// </summary>
        public void Initialize(BuiltProgram program)
        {
            var savedPersistent = PersistentVariableStore.Load();
            var persistPrefix = string.IsNullOrEmpty(program.Id) ? "" : program.Id + ":";

            foreach (var v in program.Variables ?? [])
            {
                bool isGlobal     = v.IsGlobal == true && _globalVars != null;
                bool isPersistent = v.IsPersistent == true;

                // Computed: a formula, not a value. Checked first so a stored-kind flag set
                // alongside it (the validator's computedKindConflict) cannot give it storage.
                if (v.IsComputed == true)
                {
                    var formula = v.ValueExpression ?? "";
                    if (isGlobal)
                    {
                        _globalComputedNames.Add(v.Name);
                        _globalVars!.RegisterComputed(v.Name, formula);
                    }
                    else
                        _computed[v.Name] = formula;
                    if (v.IsBoolean == true) _booleanVariables.Add(v.Name);
                    Version++;
                    continue;
                }

                // Lists of every element type, including ones saved before the list types
                // were unified — ToListVar folds the legacy points/objects/values fields in.
                if (v.ToListVar() is { } declaredList)
                    _listVariables[v.Name] = declaredList;
                else if (v.IsStopwatch == true)
                {
                    _stopwatches[v.Name] = new StopwatchEntry { Running = false, AccumMs = 0, StartTick = 0 };
                    Set(v.Name, 0); // elapsed ms, updated each tick
                }
                else if (v.IsString == true)
                {
                    _stringVariables[v.Name] = v.StringValue ?? "";
                }
                else if (v.IsImage == true)
                {
                    bool isGlobalImage = v.IsGlobal == true && _globalImages != null;
                    if (isGlobalImage)
                    {
                        _globalImageNames.Add(v.Name);
                        _globalImages!.InitIfAbsent(v.Name, "");
                        _imageVariables[v.Name] = ""; // local shadow, kept in sync on read
                    }
                    else
                        _imageVariables[v.Name] = ""; // populated at runtime by CaptureImage steps
                }
                else
                {
                    // Persistent: restore saved value if available (keyed by programId:varName), else use declared default
                    double initialValue = isPersistent && savedPersistent.TryGetValue(persistPrefix + v.Name, out var saved)
                        ? saved
                        : ResolveInitialValue(v);

                    if (isGlobal)
                    {
                        _globalVarNames.Add(v.Name);
                        _globalVars!.InitIfAbsent(v.Name, initialValue);
                        Version++;
                    }
                    else
                        Set(v.Name, initialValue);

                    // Remember which program declared it: that is the key it was loaded
                    // under above, so SavePersistent must write it back there too.
                    if (isPersistent) _persistentVarOwners[v.Name] = program.Id ?? "";
                    if (v.IsBoolean == true) _booleanVariables.Add(v.Name);
                }
            }
        }

        /// <summary>
        /// A scalar's starting value: its expression if it declares one, otherwise its plain
        /// number.
        ///
        /// Evaluated here, once, as the variable is registered — and variables are registered
        /// in declaration order, so an expression can reference a variable declared above it
        /// but not one below. A persistent variable with a restored value never reaches this:
        /// the point of persistence is to carry the last value across runs.
        /// </summary>
        public double ResolveInitialValue(ProgramVariable v)
        {
            if (string.IsNullOrWhiteSpace(v.ValueExpression)) return v.Value;

            double result;
            // An unknown variable propagates, as it does everywhere else an expression is
            // evaluated — a typo'd name quietly starting at 0 is how a clearance height
            // becomes a collision. Value is the fallback for anything else that goes wrong,
            // since it holds the last result the editor computed.
            try { result = ExpressionEvaluator.Evaluate(v.ValueExpression, Eval.Vars, _listVariables, Properties); }
            catch (UnknownVariableException) { throw; }
            catch (ExpressionParseException) { throw; }
            catch { return v.Value; }

            // A boolean holds 0 or 1, and an expression can produce any number — "$count"
            // on its own, say. Comparisons already yield 1 or 0, so this only bites the
            // cases that would otherwise store something no boolean step expects.
            return v.IsBoolean == true ? (result != 0 ? 1 : 0) : result;
        }

        /// <summary>Writes every persistent variable's current value back to disk.</summary>
        public void SavePersistent()
        {
            if (_persistentVarOwners.Count == 0) return;
            var existing = PersistentVariableStore.Load();
            foreach (var (name, ownerId) in _persistentVarOwners)
            {
                // Saved under the id of the program that declared it (a routine's own id for
                // routine variables) — the same key Initialize loads it from.
                var prefix = string.IsNullOrEmpty(ownerId) ? "" : ownerId + ":";
                var val = _globalVarNames.Contains(name) && _globalVars != null && _globalVars.TryGet(name, out var gv)
                    ? gv
                    : _variables.TryGetValue(name, out var v) ? v : 0;
                existing[prefix + name] = val;
            }
            PersistentVariableStore.Write(existing);
        }

        /// <summary>
        /// Every variable's current value for the expression symbol list: scalars (globals
        /// winning) as numbers, strings as text, lists as their element count. Images are
        /// left out — their value is a camera frame.
        /// </summary>
        public Dictionary<string, object?> SnapshotValues()
        {
            var result = new Dictionary<string, object?>(StringComparer.OrdinalIgnoreCase);
            foreach (var kv in MergedVars()) result[kv.Key] = kv.Value;
            foreach (var kv in _stringVariables) result[kv.Key] = kv.Value;
            foreach (var kv in _listVariables) result[kv.Key] = kv.Value.Count;

            // Computed variables: their formula's current result; null when it fails to evaluate.
            var computedNames = new HashSet<string>(_computed.Keys, StringComparer.OrdinalIgnoreCase);
            computedNames.UnionWith(_globalComputedNames);
            if (_globalVars != null) computedNames.UnionWith(_globalVars.ComputedSnapshot().Keys);
            foreach (var name in computedNames)
            {
                // A stored variable of the same name reads first in expressions; keep that value.
                if (result.ContainsKey(name) && !_computed.ContainsKey(name)) continue;
                double v = EvaluateComputedOrNaN(name);
                result[name] = double.IsNaN(v) ? null : (object?)v;
            }
            return result;
        }

        /// <summary>A computed variable's current value, or NaN when it is not one or its formula fails.</summary>
        private double EvaluateComputedOrNaN(string name)
        {
            try { return _computedSource.TryGetComputed(name, out var v) ? v : double.NaN; }
            catch { return double.NaN; }
        }

        // ── Monitor display ───────────────────────────────────────────────────

        /// <summary>Current values for all scalar variables of <paramref name="program"/> flagged DisplayOnMonitor.</summary>
        public IReadOnlyList<(string Name, double Value, bool IsBoolean)> GetDisplayVariables(BuiltProgram? program)
        {
            if (program?.Variables == null) return [];
            var merged = MergedVars();
            var result = new List<(string, double, bool)>();
            foreach (var v in program.Variables)
            {
                if (v.DisplayOnMonitor != true) continue;
                // A computed variable shows its formula's current result — NaN when it fails.
                if (v.IsComputed == true)
                {
                    result.Add((v.Name, EvaluateComputedOrNaN(v.Name), v.IsBoolean == true));
                    continue;
                }
                // Non-scalar types are not supported in numeric display. The legacy list
                // fields are still tested alongside Items so that an empty saved list —
                // which ToListVar deliberately reads as a scalar — stays excluded here,
                // exactly as it was before the list types were unified.
                if (v.Items != null || v.Values != null || v.Points != null || v.Objects != null
                    || v.IsString == true || v.IsImage == true) continue;
                merged.TryGetValue(v.Name, out double val);
                result.Add((v.Name, val, v.IsBoolean == true));
            }
            return result;
        }

        /// <summary>Names and write-counts of image variables flagged DisplayOnMonitor.</summary>
        public IReadOnlyList<(string Name, long Revision)> GetDisplayImages(BuiltProgram? program)
        {
            if (program?.Variables == null) return [];
            var result = new List<(string, long)>();
            foreach (var v in program.Variables)
            {
                if (v.DisplayOnMonitor != true || v.IsImage != true) continue;
                result.Add((v.Name, ImageRevision(v.Name)));
            }
            return result;
        }

        /// <summary>
        /// The bytes of one DisplayOnMonitor image variable, or "" when it is not listed —
        /// so what can be fetched is exactly what <see cref="GetDisplayImages"/> lists.
        /// </summary>
        public string GetDisplayImage(BuiltProgram? program, string name)
        {
            if (program?.Variables == null) return "";
            bool listed = program.Variables.Any(v =>
                v.DisplayOnMonitor == true && v.IsImage == true
                && string.Equals(v.Name, name, StringComparison.OrdinalIgnoreCase));
            return listed ? GetImage(name) : "";
        }

        // ── Conditions ────────────────────────────────────────────────────────

        public bool EvaluateCondition(ConditionGroup group, Dictionary<string, double> vars)
        {
            if (group.Items.Count == 0) return true;
            bool isAny = group.Combinator == "ANY";
            foreach (var item in group.Items)
            {
                bool result = EvaluateConditionItem(item, vars);
                if (isAny && result)  return true;
                if (!isAny && !result) return false;
            }
            return !isAny;
        }

        private bool EvaluateConditionItem(ConditionItem item, Dictionary<string, double> vars)
        {
            // String operator path — also used when left side is a string variable
            bool isStringOp = item.Operator is "contains" or "startsWith" or "endsWith";
            if (isStringOp || IsStringVarRef(item.Left))
            {
                string ls = ResolveStringValue(item.Left);
                string rs = ResolveStringValue(item.Right);
                return item.Operator switch
                {
                    "==" => string.Equals(ls, rs, StringComparison.Ordinal),
                    "!=" => !string.Equals(ls, rs, StringComparison.Ordinal),
                    "contains"   => ls.Contains(rs, StringComparison.Ordinal),
                    "startsWith" => ls.StartsWith(rs, StringComparison.Ordinal),
                    "endsWith"   => ls.EndsWith(rs, StringComparison.Ordinal),
                    _    => false,
                };
            }

            // Unknown variables propagate (and error the program) — a typo'd condition
            // silently comparing 0 could take the wrong branch on a machine that moves.
            double left, right;
            try { left  = ExpressionEvaluator.Evaluate(item.Left,  vars, _listVariables, Properties); }
            catch (UnknownVariableException) { throw; }
            catch (ExpressionParseException) { throw; }
            catch { left  = 0; }
            try { right = ExpressionEvaluator.Evaluate(item.Right, vars, _listVariables, Properties); }
            catch (UnknownVariableException) { throw; }
            catch (ExpressionParseException) { throw; }
            catch { right = 0; }
            // Shared with the comparison operators inside expressions, so "==" cannot come
            // to mean one thing in a condition row and another in "$a == $b".
            return ExpressionEvaluator.Compare(left, item.Operator, right);
        }

        private bool IsStringVarRef(string expr) =>
            !string.IsNullOrEmpty(expr) && expr.StartsWith('$') &&
            _stringVariables.ContainsKey(expr.Substring(1));

        private string ResolveStringValue(string expr)
        {
            if (string.IsNullOrEmpty(expr)) return "";
            if (expr.StartsWith('$') && _stringVariables.TryGetValue(expr.Substring(1), out var sv))
                return sv;
            return Interpolate(expr);
        }

        // ── Interpolation ─────────────────────────────────────────────────────

        // Matches $name, $name[expr] and $name[expr].component.
        private const string VarRef = @"(?<name>\w+)(?:\[(?<idx>[^\]]*)\](?:\.(?<comp>\w+))?)?";
        internal static readonly Regex TemplateToken = new(@"\{(?<body>[^{}]*)\}|\$" + VarRef);
        internal static readonly Regex LoneRef       = new(@"^\$" + VarRef + "$");
        internal static readonly Regex BareWord      = new(@"(?<![$.\w])[A-Za-z_]\w*");

        /// <summary>
        /// A word inside {…} that is not a forgotten $: a literal (true/false), a word-spelled
        /// operator (and/or/not), or a function name followed by "(" — "{round($x, 2)}".
        /// </summary>
        internal static bool IsAllowedBareWord(string body, Match w)
        {
            var word = w.Value;
            if (word.Equals("true", StringComparison.OrdinalIgnoreCase) ||
                word.Equals("false", StringComparison.OrdinalIgnoreCase) ||
                word.Equals("and", StringComparison.OrdinalIgnoreCase) ||
                word.Equals("or", StringComparison.OrdinalIgnoreCase) ||
                word.Equals("not", StringComparison.OrdinalIgnoreCase))
                return true;
            if (!ExpressionEvaluator.IsFunctionName(word)) return false;
            int i = w.Index + w.Length;
            while (i < body.Length && char.IsWhiteSpace(body[i])) i++;
            return i < body.Length && body[i] == '(';
        }

        /// <summary>A pose as it appears interpolated into a status message.</summary>
        private static string FormatPoint(Vector6Val pt) =>
            $"(x={pt.X:G6}, y={pt.Y:G6}, z={pt.Z:G6}, rx={pt.RX:G6}, ry={pt.RY:G6}, rz={pt.RZ:G6})";

        /// <summary>
        /// Expands variable references in a text template.
        /// </summary>
        /// <remarks>
        /// Matches $name, $name[expr], $name[expr].component and the braced form {…}.
        /// Braces delimit a reference so it can butt straight up against surrounding text —
        /// "{$prefix}{$index}" has no bare equivalent, because while "$prefix$index" works,
        /// "$prefix_2" swallows the underscore into the name — and they may hold any math
        /// expression, so "{$index + 1}" and "{$row * 3 + $col}" also interpolate. The $
        /// stays required inside braces, matching expressions everywhere else.
        /// </remarks>
        public string Interpolate(string template)
        {
            var allVarsForTemplate = Eval.Merged;

            // Expands a plain variable reference; null when the name isn't a known variable.
            string? ExpandRef(Match r)
            {
                var name     = r.Groups["name"].Value;
                var hasIndex = r.Groups["idx"].Success;
                var idxExpr  = r.Groups["idx"].Value.Trim();
                var hasComp  = r.Groups["comp"].Success;
                var compName = r.Groups["comp"].Value.ToLower();

                if (!hasIndex)
                {
                    // Plain $name — scalar → value, list → count, points → "N points", string → value
                    if (allVarsForTemplate.TryGetValue(name, out var sv))
                        return _booleanVariables.Contains(name) ? (sv != 0 ? "True" : "False") : sv.ToString("G6");
                    if (_stringVariables.TryGetValue(name, out var strVal))
                        return strVal;
                    if (_listVariables.TryGetValue(name, out var bare))
                        return bare.Describe();
                    // A computed variable: its formula's result now. A failing formula leaves the
                    // reference as written, like any other unresolvable name in text.
                    try
                    {
                        if (_computedSource.TryGetComputed(name, out var cv))
                            return _booleanVariables.Contains(name) ? (cv != 0 ? "True" : "False") : cv.ToString("G6");
                    }
                    catch { /* left as written */ }
                    return null;
                }

                // Evaluate index expression (literal int or variable expression)
                int idx = 0;
                if (!string.IsNullOrEmpty(idxExpr))
                {
                    try { idx = (int)Math.Round(ExpressionEvaluator.Evaluate(idxExpr, allVarsForTemplate, _listVariables, Properties)); }
                    catch { idx = 0; }
                }

                if (_listVariables.TryGetValue(name, out var lv))
                {
                    if (lv.Count == 0) return "(empty)";
                    idx = Math.Clamp(idx, 0, lv.Count - 1);
                    var item = lv.Items[idx];

                    if (hasComp)
                        return item.TryGetValue(compName, out var fv) ? fv.ToString("G6") : "0";

                    // No field named — render the whole element. A point keeps its familiar
                    // axis-ordered form rather than dictionary order, which is what makes
                    // "$pts[0]" readable in a status message. A boolean prints True/False to
                    // match how a scalar boolean variable interpolates, not the 0/1 it is
                    // stored as — that spelling is the reason the element type exists.
                    return lv.ElementType switch
                    {
                        ListElementType.Number  => item.Scalar.ToString("G6"),
                        ListElementType.Boolean => item.Scalar != 0 ? "True" : "False",
                        ListElementType.Point   => FormatPoint(item.ToPoint()),
                        _ => "(" + string.Join(", ", item.Select(kv => $"{kv.Key}={kv.Value:G6}")) + ")",
                    };
                }

                return null;
            }

            return TemplateToken.Replace(template, m =>
            {
                if (!m.Groups["body"].Success)
                    return ExpandRef(m) ?? m.Value;

                var body = m.Groups["body"].Value.Trim();

                // A name written without its $ would tokenize as a word and evaluate to 0,
                // turning a forgotten sigil into a plausible-looking wrong answer. Leave the
                // braces written as-is instead, so it surfaces downstream. Words after a dot
                // are components (.z, .length) and true/false are literals — both fine bare.
                var bare = BareWord.Matches(body).Any(w => !IsAllowedBareWord(body, w));
                if (body.Length == 0 || bare) return m.Value;

                var inner = LoneRef.Match(body);
                if (inner.Success)
                    // A lone reference. When the name isn't a known variable, leave it written
                    // as-is rather than handing a typo to the evaluator.
                    return ExpandRef(inner) ?? m.Value;

                // Anything else is an expression. A failure leaves the braces in place,
                // which surfaces downstream (a point lookup, say) rather than silently
                // substituting something wrong.
                try { return ExpressionEvaluator.Evaluate(body, allVarsForTemplate, _listVariables, Properties).ToString("G6"); }
                catch { return m.Value; }
            });
        }
    }
}
