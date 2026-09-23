namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// The variable dictionaries expressions are evaluated against, built at most once per
    /// executor tick instead of once per field.
    /// </summary>
    /// <remarks>
    /// <para>Between <see cref="BeginTick"/> and <see cref="EndTick"/> the merged scalar
    /// dictionary (<see cref="Merged"/>) and the merged-plus-IO dictionary (<see cref="Vars"/>)
    /// are built lazily on first use and then reused, so every field of a step — a move has
    /// two dozen — sees one consistent snapshot of variables, IO and <c>time_ms</c>. A write
    /// through <see cref="VariableScope.Set"/> (or anything else that bumps
    /// <see cref="VariableScope.Version"/>) invalidates the snapshot, so a value written earlier
    /// in the same tick — variable initialisers that reference the one declared above them,
    /// say — is always seen.</para>
    /// <para>Outside a tick (Start, Resume, the display readers) every access builds a fresh
    /// dictionary, exactly as before caching existed, so a snapshot can never go stale across
    /// the gap between runs.</para>
    /// <para>The cached dictionaries are reused between ticks to avoid allocating on the 1 ms
    /// loop; callers must treat them as read-only and must not keep them past the call.</para>
    /// <para>Not thread-safe: used under the owning executor's control lock only.</para>
    /// </remarks>
    internal sealed class EvalContext
    {
        private readonly VariableScope _scope;

        private bool _inTick;
        private readonly Dictionary<string, double> _merged = new(StringComparer.OrdinalIgnoreCase);
        private readonly Dictionary<string, double> _vars   = new(StringComparer.OrdinalIgnoreCase);
        private long _mergedVersion = -1;
        private long _varsVersion   = -1;

        public EvalContext(VariableScope scope) => _scope = scope;

        /// <summary>Starts caching: the next access builds the snapshot, later ones reuse it.</summary>
        public void BeginTick()
        {
            _inTick        = true;
            _mergedVersion = -1;
            _varsVersion   = -1;
        }

        /// <summary>Stops caching; accesses until the next <see cref="BeginTick"/> build fresh.</summary>
        public void EndTick()
        {
            _inTick        = false;
            _mergedVersion = -1;
            _varsVersion   = -1;
        }

        /// <summary>Local + global scalars and <c>time_ms</c>. Read-only.</summary>
        public Dictionary<string, double> Merged
        {
            get
            {
                if (!_inTick) return _scope.MergedVars();
                if (_mergedVersion != _scope.Version)
                {
                    _merged.Clear();
                    _scope.FillMerged(_merged);
                    _mergedVersion = _scope.Version;
                }
                return _merged;
            }
        }

        /// <summary><see cref="Merged"/> plus live IO — what expressions and conditions see. Read-only.</summary>
        public Dictionary<string, double> Vars
        {
            get
            {
                if (!_inTick) return _scope.EvalVars();
                if (_varsVersion != _scope.Version)
                {
                    _vars.Clear();
                    _scope.FillMerged(_vars);
                    _scope.FillIo(_vars);
                    _varsVersion = _scope.Version;
                }
                return _vars;
            }
        }

        /// <summary>Evaluates an expression against <see cref="Vars"/>, the list variables and the
        /// scope's properties (looked up lazily — never copied into the snapshot).</summary>
        public double Evaluate(string expr) => ExpressionEvaluator.Evaluate(expr, Vars, _scope.Lists, _scope.Properties);

        public bool EvaluateCondition(ConditionGroup group) => _scope.EvaluateCondition(group, Vars);

        /// <summary>
        /// Returns the evaluated value for a numeric field.
        /// If the step has an expression keyed by <paramref name="fieldName"/>, that expression is
        /// evaluated against the current variable dictionary; otherwise <paramref name="fallback"/> is returned.
        /// An unknown variable or a syntax error in the expression propagates (and errors the
        /// program) rather than silently falling back — a typo'd offset must never move the
        /// robot to the wrong place.
        /// </summary>
        public double EvalField(ProgramStep step, string fieldName, double fallback)
        {
            if (step.Expressions != null && step.Expressions.TryGetValue(fieldName, out var expr))
            {
                try { return Evaluate(expr); }
                catch (UnknownVariableException) { throw; }
                // A syntax error errors the program too — the validator reports it before a run.
                catch (ExpressionParseException) { throw; }
                catch { /* anything else — fall through to the literal */ }
            }
            return fallback;
        }

        /// <summary>
        /// An optional numeric field: null when the step sets neither a literal nor an
        /// expression for <paramref name="key"/>, otherwise its evaluated value (expression
        /// first, then the literal — see <see cref="EvalField"/>).
        /// </summary>
        public double? OptionalField(ProgramStep step, string key, double? literal) =>
            literal.HasValue || step.Expressions?.ContainsKey(key) == true
                ? EvalField(step, key, literal ?? 0)
                : null;
    }
}
