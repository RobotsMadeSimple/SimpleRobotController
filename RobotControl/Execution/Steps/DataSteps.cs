using System.Runtime.CompilerServices;
using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// Program editor v2 data blocks (docs/program-language.md §4.3): ListOp — append, insert,
    /// removeAt, clear, set, setField, sort, reverse and copyFrom on a list variable.
    /// </summary>
    internal static class DataSteps
    {
        public static void Register(Dictionary<StepType, IStepHandler> r)
        {
            r[StepType.ListOp] = ListOpStep.Instance;
        }
    }

    /// <summary>
    /// One List step. Everything that can fail (the index, every value) is worked out before the
    /// list changes, so a failed step leaves the list as it was. An index outside the list fails
    /// <c>indexOutOfRange</c> (catchable); a step that does not fit the list's element type fails
    /// <c>badListOperation</c> (an authoring bug, not catchable).
    /// </summary>
    /// <remarks>
    /// Element rules: a Number / Boolean element is <c>listValueExpr</c> (a Boolean stores 0/1);
    /// a Point element takes its axes (x y z rx ry rz) from <c>listFields</c> — append with no
    /// fields appends the current position (in the active local), and set keeps the axes it does
    /// not name; a Record element takes any fields from <c>listFields</c> (set merges into the
    /// element). Sort orders by value (Number/Boolean) or by <c>listSortField</c>, stably.
    /// </remarks>
    internal sealed class ListOpStep : IStepHandler
    {
        public static readonly ListOpStep Instance = new();

        private static readonly string[] PointAxes = ["x", "y", "z", "rx", "ry", "rz"];

        // Lists this handler made its own copy of, and may therefore change in place. A list
        // straight from a program's declaration shares its element objects with the program
        // definition (which must never change), and one written by another step may be held
        // elsewhere, so the first List step on it copies it — later ones reuse the copy.
        private static readonly ConditionalWeakTable<ListVar, object> Owned = new();
        private static readonly object Mark = new();

        /// <summary>An evaluated index: rounded to the nearest whole number; a non-number is -1.</summary>
        internal static int ToIndex(double v) =>
            double.IsFinite(v) && Math.Abs(v) < int.MaxValue ? (int)Math.Round(v) : -1;

        public StepOutcome Execute(ProgramStep step, StepListFrame frame, ExecutionContext ctx)
        {
            string desc = ProgressReporter.StepDescription(step);
            string name = VarName(step.VariableName);
            if (name.Length == 0) return ctx.Finish(ProgramStatus.Error, $"{desc}: no list variable is selected");
            if (!ctx.Vars.Lists.TryGetValue(name, out var current))
                return ctx.Fail(ErrorCodes.UnknownVariable, $"{desc}: '${name}' is not a list variable");

            string action = string.IsNullOrEmpty(step.ListAction) ? "append" : step.ListAction;
            var type = current.ElementType;
            bool scalar = current.HasScalarElements;
            int count = current.Count;

            StepOutcome Bad(string why) => ctx.Fail(ErrorCodes.BadListOperation, $"{desc}: {why}");
            StepOutcome OutOfRange(int index, int max) => ctx.Fail(ErrorCodes.IndexOutOfRange,
                count == 0 && max < 0
                    ? $"{desc}: index {index} is outside '${name}' — the list is empty"
                    : $"{desc}: index {index} is outside '${name}' ({Describe(current)}; valid {0}–{max})");

            int Index() => ToIndex(ctx.Eval.EvalField(step, "listIndex", step.ListIndex ?? 0));

            switch (action)
            {
                case "append":
                case "insert":
                {
                    int index = action == "insert" ? Index() : count;
                    if (index < 0 || index > count) return OutOfRange(index, count);
                    if (!TryBuild(step, ctx, type, null, allowHere: action == "append", out var element, out var why))
                        return Bad(why);
                    Writable(ctx, name, current).Items.Insert(index, element);
                    return StepOutcome.Advance;
                }

                case "set":
                {
                    int index = Index();
                    if (index < 0 || index >= count) return OutOfRange(index, count - 1);
                    if (!TryBuild(step, ctx, type, current.Items[index], allowHere: false, out var element, out var why))
                        return Bad(why);
                    Writable(ctx, name, current).Items[index] = element;
                    return StepOutcome.Advance;
                }

                case "setField":
                {
                    if (scalar) return Bad($"'${name}' is a {type} list — its elements have no fields; use Set element");
                    if (step.ListFields is not { Count: > 0 } fields) return Bad("no field is set");
                    int index = Index();
                    if (index < 0 || index >= count) return OutOfRange(index, count - 1);
                    if (!TryEvalFields(fields, ctx, type, out var values, out var why)) return Bad(why);
                    var rec = Writable(ctx, name, current).Items[index];
                    foreach (var (field, value) in values) rec[field] = value;
                    return StepOutcome.Advance;
                }

                case "removeAt":
                {
                    int index = Index();
                    if (index < 0 || index >= count) return OutOfRange(index, count - 1);
                    Writable(ctx, name, current).Items.RemoveAt(index);
                    return StepOutcome.Advance;
                }

                case "clear":
                    Writable(ctx, name, current).Items.Clear();
                    return StepOutcome.Advance;

                case "reverse":
                    Writable(ctx, name, current).Items.Reverse();
                    return StepOutcome.Advance;

                case "sort":
                {
                    string? field = step.ListSortField?.Trim();
                    if (scalar)
                    {
                        if (!string.IsNullOrEmpty(field) && !field.Equals(ObjectRecord.ScalarKey, StringComparison.OrdinalIgnoreCase))
                            return Bad($"'${name}' is a {type} list — it sorts by value, not by '{field}'");
                        field = ObjectRecord.ScalarKey;
                    }
                    else if (string.IsNullOrEmpty(field))
                        return Bad($"'${name}' is a {type} list — choose the field to sort by");
                    else if (type == ListElementType.Point && !IsAxis(field))
                        return Bad($"'{field}' is not a point axis (x, y, z, rx, ry, rz)");

                    var items = Writable(ctx, name, current).Items;
                    string key = field;
                    double Key(ObjectRecord r) => r.TryGetValue(key, out var v) ? v : 0;
                    // LINQ ordering is stable: equal keys keep their order.
                    var sorted = step.ListSortDescending == true ? items.OrderByDescending(Key).ToList() : items.OrderBy(Key).ToList();
                    items.Clear();
                    items.AddRange(sorted);
                    return StepOutcome.Advance;
                }

                case "copyFrom":
                {
                    string source = VarName(step.ListSourceVariable);
                    if (source.Length == 0) return Bad("no list to copy from is selected");
                    if (!ctx.Vars.Lists.TryGetValue(source, out var src))
                        return ctx.Fail(ErrorCodes.UnknownVariable, $"{desc}: '${source}' is not a list variable");
                    if (!CanCopy(src.ElementType, type))
                        return Bad($"cannot copy a {src.ElementType} list into '${name}', a {type} list");
                    var copy = src.Items.Select(r => Normalise(Clone(r), type)).ToList();
                    var target = Writable(ctx, name, current).Items;
                    target.Clear();
                    target.AddRange(copy);
                    return StepOutcome.Advance;
                }

                default:
                    return Bad($"unknown list action '{action}'");
            }
        }

        // ── Elements ──────────────────────────────────────────────────────────

        /// <summary>
        /// The element an append / insert / set writes. <paramref name="existing"/> is the element
        /// being replaced (set), whose fields a Point / Record element starts from.
        /// </summary>
        private static bool TryBuild(ProgramStep step, ExecutionContext ctx, ListElementType type, ObjectRecord? existing,
            bool allowHere, out ObjectRecord element, out string why)
        {
            element = null!;
            why = "";
            var fields = step.ListFields;
            switch (type)
            {
                case ListElementType.Number:
                case ListElementType.Boolean:
                {
                    string? expr = !string.IsNullOrWhiteSpace(step.ListValueExpr) ? step.ListValueExpr
                        : fields != null && TryGetField(fields, ObjectRecord.ScalarKey, out var fv) ? fv : null;
                    if (expr == null) { why = "no value is set"; return false; }
                    if (fields != null && fields.Keys.Any(k => !k.Equals(ObjectRecord.ScalarKey, StringComparison.OrdinalIgnoreCase)))
                    {
                        why = $"a {type} list has no fields — set its value instead";
                        return false;
                    }
                    double v = ctx.Eval.Evaluate(expr);
                    element = ObjectRecord.FromScalar(type == ListElementType.Boolean ? (v != 0 ? 1 : 0) : v);
                    return true;
                }

                case ListElementType.Point:
                {
                    if (fields is not { Count: > 0 })
                    {
                        if (!string.IsNullOrWhiteSpace(step.ListValueExpr))
                        {
                            why = "a Points list element needs axes (x, y, z, rx, ry, rz), not a single value";
                            return false;
                        }
                        if (!allowHere) { why = "no axes are set"; return false; }
                        // "append Here": the current position, as Save Position would store it.
                        element = ObjectRecord.FromPoint(MotionExtSteps.ToVal(MotionExtSteps.ProgramPose(ctx)));
                        return true;
                    }
                    if (!TryEvalFields(fields, ctx, type, out var values, out why)) return false;
                    element = existing != null ? Clone(existing) : ObjectRecord.FromPoint(new Vector6Val());
                    foreach (var (f, value) in values) element[f] = value;
                    return true;
                }

                default: // Record
                {
                    if (fields is not { Count: > 0 }) { why = "no fields are set"; return false; }
                    if (!TryEvalFields(fields, ctx, type, out var values, out why)) return false;
                    element = existing != null ? Clone(existing) : new ObjectRecord();
                    foreach (var (f, value) in values) element[f] = value;
                    return true;
                }
            }
        }

        /// <summary>Evaluates every field expression (Point lists: only the six axes, stored lower-case).</summary>
        private static bool TryEvalFields(Dictionary<string, string> fields, ExecutionContext ctx, ListElementType type,
            out List<(string Field, double Value)> values, out string why)
        {
            values = new();
            why = "";
            foreach (var (rawKey, expr) in fields)
            {
                var key = rawKey?.Trim() ?? "";
                if (key.Length == 0) { why = "a field has no name"; return false; }
                if (type == ListElementType.Point)
                {
                    if (!IsAxis(key)) { why = $"'{key}' is not a point axis (x, y, z, rx, ry, rz)"; return false; }
                    key = key.ToLowerInvariant();
                }
                values.Add((key, ctx.Eval.Evaluate(expr ?? "")));
            }
            return true;
        }

        private static bool IsAxis(string field) => PointAxes.Contains(field, StringComparer.OrdinalIgnoreCase);

        private static bool TryGetField(Dictionary<string, string> fields, string key, out string value)
        {
            foreach (var (k, v) in fields)
                if (k.Equals(key, StringComparison.OrdinalIgnoreCase)) { value = v; return true; }
            value = "";
            return false;
        }

        /// <summary>Which element types copy into which: the same type, Number ↔ Boolean, and
        /// anything into a Record list (a record is any named numbers).</summary>
        private static bool CanCopy(ListElementType from, ListElementType to) =>
            from == to || to == ListElementType.Record ||
            (from is ListElementType.Number or ListElementType.Boolean && to is ListElementType.Number or ListElementType.Boolean);

        private static ObjectRecord Normalise(ObjectRecord r, ListElementType to)
        {
            if (to == ListElementType.Boolean) r[ObjectRecord.ScalarKey] = r.Scalar != 0 ? 1 : 0;
            return r;
        }

        // ── Ownership ─────────────────────────────────────────────────────────

        /// <summary>The list under <paramref name="name"/>, safe to change in place (copied once).</summary>
        private static ListVar Writable(ExecutionContext ctx, string name, ListVar current)
        {
            if (Owned.TryGetValue(current, out _)) return current;
            var copy = new ListVar { ElementType = current.ElementType, Items = current.Items.Select(Clone).ToList() };
            Owned.AddOrUpdate(copy, Mark);
            ctx.Vars.SetList(name, copy);
            return copy;
        }

        private static ObjectRecord Clone(ObjectRecord r)
        {
            var c = new ObjectRecord();
            foreach (var (k, v) in r) c[k] = v;
            return c;
        }

        private static string VarName(string? raw) => raw?.Trim().TrimStart('$') ?? "";

        private static string Describe(ListVar l) => l.Describe();
    }
}
