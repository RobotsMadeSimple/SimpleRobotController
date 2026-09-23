namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Thrown when something tries to assign a computed variable (a named formula with no
    /// stored value — docs/expressions-and-variables.md §7). The executor turns it into a
    /// program error, as it does <see cref="UnknownVariableException"/>.
    /// </summary>
    public class ComputedVariableWriteException : Exception
    {
        public string VariableName { get; }
        public ComputedVariableWriteException(string variableName)
            : base($"'${variableName}' is a computed variable and cannot be assigned")
        {
            VariableName = variableName;
        }
    }

    /// <summary>
    /// Resolves computed variables (user-defined properties) by evaluating their formula each
    /// time they are read, then defers to the next source in the chain (the robot property
    /// source) for everything else: computed → robot.
    /// </summary>
    /// <remarks>
    /// <para>Lookup order: this program's own computed variables, then global computed
    /// variables registered in the <see cref="GlobalVariableStore"/>, then
    /// <see cref="Inner"/>. A local formula is evaluated against the program's live variables,
    /// lists and this source (so it may reference other computed variables and properties);
    /// a global formula against the global values, IO and properties only.</para>
    /// <para>A formula that reaches itself again while it is being evaluated throws an
    /// <see cref="ExpressionParseException"/> with code <c>computedCycle</c>, which every
    /// evaluation path already turns into a program error.</para>
    /// <para>Not thread-safe (the in-progress set): one instance per scope, used under the
    /// owning executor's lock, or a fresh one per command.</para>
    /// </remarks>
    internal sealed class ComputedPropertySource : IPropertySource
    {
        public const string CycleCode = "computedCycle";

        private readonly IReadOnlyDictionary<string, string>? _local;
        private readonly Func<Dictionary<string, double>>? _localVars;
        private readonly Func<Dictionary<string, ListVar>?>? _localLists;
        private readonly GlobalVariableStore? _globals;
        private readonly Action<Dictionary<string, double>>? _io;
        private readonly Func<IPropertySource?> _inner;

        private readonly HashSet<string> _inProgress = new(StringComparer.OrdinalIgnoreCase);
        private readonly List<string> _stack = new();
        private ComputedPropertySource? _globalOnly;

        /// <param name="localFormulas">This program's computed variables (name → formula); null for
        /// a globals-only source.</param>
        /// <param name="localVars">The variables (with IO) local formulas are evaluated against.</param>
        /// <param name="localLists">The list variables local formulas can index.</param>
        /// <param name="globals">The shared store holding global values and global formulas.</param>
        /// <param name="io">Adds live IO values to a dictionary (global formula evaluation).</param>
        /// <param name="inner">The next source in the chain (robot properties); may return null.</param>
        public ComputedPropertySource(
            IReadOnlyDictionary<string, string>? localFormulas,
            Func<Dictionary<string, double>>? localVars,
            Func<Dictionary<string, ListVar>?>? localLists,
            GlobalVariableStore? globals,
            Action<Dictionary<string, double>>? io,
            Func<IPropertySource?> inner)
        {
            _local      = localFormulas;
            _localVars  = localVars;
            _localLists = localLists;
            _globals    = globals;
            _io         = io;
            _inner      = inner;
        }

        /// <summary>A source that resolves only global computed variables, then <paramref name="inner"/> —
        /// for evaluating outside any program.</summary>
        public static ComputedPropertySource ForGlobals(GlobalVariableStore? globals,
                                                        Action<Dictionary<string, double>>? io,
                                                        IPropertySource? inner) =>
            new(null, null, null, globals, io, () => inner);

        /// <summary>The next source in the chain.</summary>
        public IPropertySource? Inner => _inner();

        public bool TryGet(string name, out double value)
        {
            if (TryGetComputed(name, out value)) return true;
            var inner = _inner();
            if (inner != null) return inner.TryGet(name, out value);
            value = 0;
            return false;
        }

        /// <summary>
        /// Resolves <paramref name="name"/> only if it is a computed variable (local first, then
        /// global); false otherwise. Evaluation errors propagate.
        /// </summary>
        public bool TryGetComputed(string name, out double value)
        {
            if (_local != null && _local.TryGetValue(name, out var formula))
            {
                var vars = _localVars?.Invoke() ?? new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
                value = EvaluateFormula(name, formula, vars, _localLists?.Invoke());
                return true;
            }
            if (_globals != null && _globals.TryGetComputed(name, out var globalFormula))
            {
                var g = GlobalOnly;
                value = g.EvaluateFormula(name, globalFormula, g.GlobalVars(), null);
                return true;
            }
            value = 0;
            return false;
        }

        /// <summary>Whether <paramref name="name"/> is a computed variable this source resolves.</summary>
        public bool IsComputed(string name) =>
            (_local != null && _local.ContainsKey(name)) ||
            (_globals != null && _globals.TryGetComputed(name, out _));

        public IEnumerable<(string Name, string Description, string Type)> List() =>
            _inner()?.List() ?? [];

        // Global formulas never see a program's locals: they are evaluated by a source with
        // no local formulas, against the global store's values only. It shares nothing with
        // this one but the chain, so a global cycle is caught in its own in-progress set.
        private ComputedPropertySource GlobalOnly =>
            _local == null ? this : _globalOnly ??= new ComputedPropertySource(null, null, null, _globals, _io, _inner);

        private Dictionary<string, double> GlobalVars()
        {
            var vars = new Dictionary<string, double>(StringComparer.OrdinalIgnoreCase);
            _globals?.CopyInto(vars);
            vars["time_ms"] = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds();
            _io?.Invoke(vars);
            return vars;
        }

        private double EvaluateFormula(string name, string formula, Dictionary<string, double> vars,
                                       Dictionary<string, ListVar>? lists)
        {
            if (!_inProgress.Add(name))
            {
                int start = _stack.FindIndex(n => string.Equals(n, name, StringComparison.OrdinalIgnoreCase));
                var chain = string.Join(" → ", _stack.Skip(Math.Max(0, start)).Append(name).Select(n => "$" + n));
                throw new ExpressionParseException(
                    $"Computed variable '${name}' depends on itself ({chain})", -1, CycleCode, formula);
            }
            _stack.Add(name);
            try
            {
                return ExpressionEvaluator.Evaluate(formula, vars, lists, this);
            }
            finally
            {
                _stack.RemoveAt(_stack.Count - 1);
                _inProgress.Remove(name);
            }
        }
    }
}
