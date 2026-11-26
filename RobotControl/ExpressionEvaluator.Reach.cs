using Controller.RobotControl.Programs;

namespace Controller.RobotControl
{
    /// <summary>
    /// A runtime failure inside an expression that a Try can catch — <see cref="ExpressionParseException.Code"/>
    /// is a catalog error code (<c>outOfReach</c>, <c>pointNotFound</c>, <c>indexOutOfRange</c>). The
    /// executor reports it through <c>ExecutionContext.Fail</c>; every other path treats it as the
    /// expression error it also is.
    /// </summary>
    public class ExpressionRuntimeException : ExpressionParseException
    {
        public ExpressionRuntimeException(string code, string message, int position = -1, string expression = "")
            : base(message, position, code, expression) { }
    }

    /// <summary>
    /// What the reach functions need from the robot: saved geometry by name, the active (or a named)
    /// tool and local frame, and the robot's own IK / limit check and FK. Implemented by the
    /// property source a program evaluates against (<c>RobotPropertySource</c>), so the answers are
    /// the controller's real kinematics and the joint windows the motion thread enforces.
    /// </summary>
    internal interface IMotionQueries
    {
        /// <summary>False when there is no robot behind the source (tests, offline): the reach functions then error.</summary>
        bool MotionAvailable { get; }
        bool TryPoint(string name, out Vector6 point);
        bool TryGridCell(string grid, int row, int col, out Vector6 point, out string error);
        bool TryGridIndex(string grid, int index, out Vector6 point, out string error);
        bool TryStackSlot(string stack, int index, out Vector6 point, out string error);
        /// <summary>null / "" = the active tool, "none" = no tool, else a saved tool by name.</summary>
        bool TryTool(string? name, out Vector6 tool, out string error);
        /// <summary>null / "" = the program's active local, "none" = world, else a saved local by name.</summary>
        bool TryLocal(string? name, out Vector6? local, out string error);
        /// <summary>IK + joint limits for a world-frame target with <paramref name="tool"/>.</summary>
        ReachResult Reach(Vector6 worldTarget, Vector6 tool);
        /// <summary>The world TCP pose of a joint vector with <paramref name="tool"/>.</summary>
        Vector6 Pose(Vector6 joints, Vector6 tool);
    }

    internal static partial class ExpressionEvaluator
    {
        internal const string ReachableFn = "reachable";
        internal const string JointsOfFn  = "jointsOf";
        internal const string PoseOfFn    = "poseOf";

        private static bool IsReachFunction(string name) =>
            name is ReachableFn or JointsOfFn or PoseOfFn;

        private static readonly string[] JointFields = ["j1", "j2", "j3", "j4"];
        private static readonly string[] PoseFields  = ["x", "y", "z", "rx", "ry", "rz"];

        /// <summary>
        /// Evaluates an expression that stands for a whole value rather than a number: a list element
        /// (<c>$home[0]</c>, <c>$pts[$i]</c>), <c>jointsOf(…)</c> or <c>poseOf(…)</c>. Used where a step
        /// takes a joints value or a pose (MoveJoints, a List step's value). False when the expression
        /// is anything else (a number) — the caller then treats it as one.
        /// </summary>
        /// <param name="kind">"joints", "pose" or the list's element type name for a list element.</param>
        public static bool TryEvaluateValue(string expr, Dictionary<string, double> variables,
            Dictionary<string, ListVar>? listVariables, IPropertySource? properties,
            Func<string, string>? strings, Func<string, string?>? stringVariables,
            out ObjectRecord value, out string kind)
        {
            value = new ObjectRecord();
            kind  = "";
            var node = GetOrParse(expr);
            var env = new Env(variables, listVariables, properties, strings, stringVariables);
            switch (node)
            {
                case ReachCallNode call when call.IsWholeValue:
                    value = call.EvalValue(env);
                    kind  = call.Kind == ReachKind.JointsOf ? "joints" : "pose";
                    return true;
                case ElementValueNode el:
                    (value, var et) = el.Element(env);
                    kind = et.ToString();
                    return true;
            }
            return false;
        }

        /// <summary>True when the expression stands for a whole value (<c>$list[i]</c>, <c>jointsOf(…)</c>,
        /// <c>poseOf(…)</c> with no accessor) — what <see cref="TryEvaluateValue"/> accepts. False on a syntax error.</summary>
        public static bool IsValueExpression(string expr)
        {
            try { return GetOrParse(expr) is ElementValueNode or ReachCallNode { IsWholeValue: true }; }
            catch (ExpressionParseException) { return false; }
        }

        private enum ReachKind { Reachable, JointsOf, PoseOf }

        private static IMotionQueries Queries(in Env e, string fn, int pos)
        {
            // The robot source sits at the end of the chain: computed variables → plugins → robot.
            var p = e.Props;
            for (int depth = 0; depth < 8 && p != null; depth++)
            {
                if (p is IMotionQueries q && q.MotionAvailable) return q;
                p = (p as Execution.ComputedPropertySource)?.Inner;
            }
            throw new ExpressionParseException($"{fn}() needs the robot — it is only available while a program or the controller evaluates it", pos);
        }

        // ── Arguments ─────────────────────────────────────────────────────────

        /// <summary>A point (or joints) argument of a reach function.</summary>
        private abstract class ReachArg
        {
            public int Pos;
            /// <summary>The pose in program (local-frame) coordinates.</summary>
            public abstract Vector6 Pose(in Env e, IMotionQueries q, string fn);
            /// <summary>The joint vector (X = J1, Y = J2, Z = J3, RZ = J4).</summary>
            public abstract Vector6 Joints(in Env e, IMotionQueries q, string fn);
            public virtual void Collect(List<ExprRef> refs) { }
        }

        /// <summary><c>$list[i]</c> — an element of a Points list (a pose) or of an Objects list (a pose
        /// with x/y/z, or a joints value with j1…j4).</summary>
        private sealed class ListElementArg : ReachArg
        {
            private readonly string _name;
            private readonly Node _index;
            public ListElementArg(string name, Node index, int pos) { _name = name; _index = index; Pos = pos; }

            private (ObjectRecord Item, ListVar List) Item(in Env e, string fn)
            {
                if (e.Lists == null || !e.Lists.TryGetValue(_name, out var lv))
                {
                    if (e.Vars.ContainsKey(_name)) throw new ExpressionParseException($"{fn}(): ${_name} is not a list", Pos);
                    throw new UnknownVariableException(_name);
                }
                int idx = (int)Math.Round(_index.Eval(e));
                if (idx < 0 || idx >= lv.Count)
                    throw new ExpressionRuntimeException(ErrorCodes.IndexOutOfRange,
                        $"{fn}(): index {idx} is outside ${_name} ({lv.Describe()})", Pos);
                return (lv.Items[idx], lv);
            }

            public override Vector6 Pose(in Env e, IMotionQueries q, string fn)
            {
                var (item, lv) = Item(e, fn);
                if (lv.ElementType == ListElementType.Point ||
                    (lv.ElementType == ListElementType.Record && (item.ContainsKey("x") || item.ContainsKey("y"))))
                {
                    var p = item.ToPoint();
                    return new Vector6(p.X, p.Y, p.Z, p.RX, p.RY, p.RZ);
                }
                throw new ExpressionParseException($"{fn}(): ${_name} holds {lv.ElementType} values, not points", Pos);
            }

            public override Vector6 Joints(in Env e, IMotionQueries q, string fn)
            {
                var (item, lv) = Item(e, fn);
                if (lv.ElementType == ListElementType.Record && Reachability.IsJointsRecord(item))
                    return Reachability.FromRecord(item);
                throw new ExpressionParseException(
                    $"{fn}(): ${_name} does not hold joint values — use an Objects list whose elements have j1…j4", Pos);
            }

            public override void Collect(List<ExprRef> refs)
            {
                refs.Add(new ExprRef(ExprRefKind.Indexed, _name, _name.Split('.'), Pos));
                _index.Collect(refs);
            }
        }

        /// <summary>A text value: a <c>"literal"</c> or a string variable <c>$s</c>.</summary>
        private readonly record struct TextArg(string Text, bool IsVar, int Pos)
        {
            public string Resolve(in Env e)
            {
                if (!IsVar) return Text;
                if (e.TryString(Text, out var s)) return s;
                throw new UnknownVariableException(Text);
            }

            public void Collect(List<ExprRef> refs)
            {
                if (IsVar) refs.Add(new ExprRef(ExprRefKind.Plain, Text, [Text], Pos));
            }
        }

        /// <summary><c>"Name"</c> / <c>$name</c> — a saved point by name.</summary>
        private sealed class SavedPointArg : ReachArg
        {
            private readonly TextArg _name;
            public SavedPointArg(TextArg name) { _name = name; Pos = name.Pos; }

            public override Vector6 Pose(in Env e, IMotionQueries q, string fn)
            {
                var name = _name.Resolve(e).Trim();
                if (name.Length > 0 && q.TryPoint(name, out var p)) return p;
                throw new ExpressionRuntimeException(ErrorCodes.PointNotFound, $"{fn}(): no saved point named '{name}'", Pos);
            }

            public override Vector6 Joints(in Env e, IMotionQueries q, string fn) =>
                throw new ExpressionParseException($"{fn}() takes joint values ($j[i] or jointsOf(…)), not a saved point", Pos);

            public override void Collect(List<ExprRef> refs) => _name.Collect(refs);
        }

        /// <summary><c>grid("Tray", row, col)</c>, <c>grid("Tray", index)</c> or <c>stack("Pallet", index)</c> —
        /// the same cell a move to that grid / stack goes to.</summary>
        private sealed class CellArg : ReachArg
        {
            private readonly bool _stack;
            private readonly TextArg _name;
            private readonly Node _a;
            private readonly Node? _b;
            public CellArg(bool stack, TextArg name, Node a, Node? b, int pos) { _stack = stack; _name = name; _a = a; _b = b; Pos = pos; }

            public override Vector6 Pose(in Env e, IMotionQueries q, string fn)
            {
                var name = _name.Resolve(e).Trim();
                int a = (int)Math.Round(_a.Eval(e));
                bool ok; Vector6 p; string err;
                if (_stack) ok = q.TryStackSlot(name, a, out p, out err);
                else if (_b != null) ok = q.TryGridCell(name, a, (int)Math.Round(_b.Eval(e)), out p, out err);
                else ok = q.TryGridIndex(name, a, out p, out err);
                if (ok) return p;
                throw new ExpressionRuntimeException(ErrorCodes.PointNotFound, $"{fn}(): {err}", Pos);
            }

            public override Vector6 Joints(in Env e, IMotionQueries q, string fn) =>
                throw new ExpressionParseException($"{fn}() takes joint values ($j[i] or jointsOf(…)), not a {(_stack ? "stack" : "grid")} cell", Pos);

            public override void Collect(List<ExprRef> refs)
            {
                _name.Collect(refs);
                _a.Collect(refs);
                _b?.Collect(refs);
            }
        }

        /// <summary>A nested <c>jointsOf(…)</c> / <c>poseOf(…)</c> as the argument of another reach function.</summary>
        private sealed class NestedArg : ReachArg
        {
            private readonly ReachCallNode _call;
            public NestedArg(ReachCallNode call) { _call = call; Pos = call.Pos; }

            public override Vector6 Pose(in Env e, IMotionQueries q, string fn)
            {
                if (_call.Kind != ReachKind.PoseOf)
                    throw new ExpressionParseException($"{fn}() takes a point; {_call.Name}(…) gives joint values — use poseOf(…)", Pos);
                var r = _call.EvalValue(e);
                var p = r.ToPoint();
                return new Vector6(p.X, p.Y, p.Z, p.RX, p.RY, p.RZ);
            }

            public override Vector6 Joints(in Env e, IMotionQueries q, string fn)
            {
                if (_call.Kind != ReachKind.JointsOf)
                    throw new ExpressionParseException($"{fn}() takes joint values; {_call.Name}(…) gives a pose — use jointsOf(…)", Pos);
                return Reachability.FromRecord(_call.EvalValue(e));
            }

            public override void Collect(List<ExprRef> refs) => _call.Collect(refs);
        }

        // ── The call ──────────────────────────────────────────────────────────

        /// <summary>
        /// <c>reachable(point[, tool][, local])</c> → 1/0; <c>jointsOf(point[, tool][, local])</c> → a joints
        /// value; <c>poseOf(joints[, tool][, local])</c> → a pose. A value is read one component at a time
        /// in a numeric expression (<c>jointsOf(…).j2</c>, <c>poseOf(…).z</c>, or <c>[n]</c>), or taken
        /// whole where a step wants a value (<see cref="TryEvaluateValue"/>).
        /// </summary>
        /// <remarks>
        /// The point resolves exactly like a move target: program (local-frame) coordinates, moved to
        /// the world by the given local (default: the program's active local) and solved with the given
        /// tool (default: the active tool). Reach is the robot's real IK plus the enforced joint windows.
        /// </remarks>
        private sealed class ReachCallNode : Node
        {
            public readonly ReachKind Kind;
            public readonly string Name;
            private readonly ReachArg _target;
            private readonly TextArg? _tool, _local;
            private readonly string? _field;
            private readonly Node? _posIdx;
            private readonly string _src;

            public ReachCallNode(ReachKind kind, string name, ReachArg target, TextArg? tool, TextArg? local,
                                 string? field, Node? posIdx, int pos, string src)
            {
                Kind = kind; Name = name; _target = target; _tool = tool; _local = local;
                _field = field; _posIdx = posIdx; Pos = pos; _src = src;
            }

            /// <summary>jointsOf / poseOf written without a component accessor.</summary>
            public bool IsWholeValue => Kind != ReachKind.Reachable && _field == null && _posIdx == null;

            public override bool IsBoolean => Kind == ReachKind.Reachable;

            private (Vector6 Tool, Vector6? Local) Frames(in Env e, IMotionQueries q)
            {
                string? toolName  = _tool?.Resolve(e);
                string? localName = _local?.Resolve(e);
                if (!q.TryTool(toolName, out var tool, out var terr))
                    throw new ExpressionRuntimeException(ErrorCodes.PointNotFound, $"{Name}(): {terr}", Pos, _src);
                if (!q.TryLocal(localName, out var local, out var lerr))
                    throw new ExpressionRuntimeException(ErrorCodes.PointNotFound, $"{Name}(): {lerr}", Pos, _src);
                return (tool, local);
            }

            private ReachResult Solve(in Env e, IMotionQueries q)
            {
                var (tool, local) = Frames(e, q);
                var p = _target.Pose(e, q, Name);
                var world = local is { } l ? LocalFrame.Apply(l, p) : p;
                return q.Reach(world, tool);
            }

            /// <summary>The whole value: a joints record (j1…j4) or a pose record (x…rz).</summary>
            public ObjectRecord EvalValue(in Env e)
            {
                var q = Queries(e, Name, Pos);
                if (Kind == ReachKind.PoseOf)
                {
                    var (tool, local) = Frames(e, q);
                    var joints = _target.Joints(e, q, Name);
                    var world = q.Pose(joints, tool);
                    var p = local is { } l ? LocalFrame.Inverse(l, world) : world;
                    return ObjectRecord.FromPoint(new Vector6Val { X = p.X, Y = p.Y, Z = p.Z, RX = p.RX, RY = p.RY, RZ = p.RZ });
                }

                var r = Solve(e, q);
                if (!r.Ok)
                    throw new ExpressionRuntimeException(ErrorCodes.OutOfReach, $"{Name}(): the point is out of reach — {r.Reason}", Pos, _src);
                return Reachability.ToRecord(r.Joints);
            }

            public override double Eval(in Env e)
            {
                if (Kind == ReachKind.Reachable)
                {
                    var q = Queries(e, Name, Pos);
                    return Solve(e, q).Ok ? 1 : 0;
                }

                if (IsWholeValue)
                    throw new ExpressionParseException(Kind == ReachKind.JointsOf
                        ? "jointsOf(…) gives four joint values — read one with .j1, .j2, .j3 or .j4"
                        : "poseOf(…) gives a pose — read one axis with .x, .y, .z, .rx, .ry or .rz", Pos, "expressionSyntax", _src);

                var value = EvalValue(e);
                var fields = Kind == ReachKind.JointsOf ? JointFields : PoseFields;
                string? key = _field;
                if (_posIdx != null)
                {
                    int i = (int)Math.Round(_posIdx.Eval(e));
                    key = i >= 0 && i < fields.Length ? fields[i] : null;
                    if (key == null) return 0;
                }
                return value.TryGetValue(key!, out double v) ? v : 0;
            }

            public override void Collect(List<ExprRef> refs)
            {
                int args = 1 + (_tool != null ? 1 : 0) + (_local != null ? 1 : 0);
                refs.Add(new ExprRef(ExprRefKind.Function, Name, [Name], Pos, args));
                _target.Collect(refs);
                _tool?.Collect(refs);
                _local?.Collect(refs);
                _posIdx?.Collect(refs);
            }
        }

        /// <summary>A bare <c>$list[i]</c> (no accessor) at the top of an expression — a whole element.
        /// Evaluates numerically exactly as before (0 for a point / record element).</summary>
        private sealed class ElementValueNode : Node
        {
            private readonly Node _inner;
            private readonly string _name;
            private readonly Node _index;
            public ElementValueNode(Node inner, string name, Node index, int pos) { _inner = inner; _name = name; _index = index; Pos = pos; }

            public override double Eval(in Env e) => _inner.Eval(e);
            public override bool IsBoolean => _inner.IsBoolean;
            public override void Collect(List<ExprRef> refs) => _inner.Collect(refs);

            public (ObjectRecord, ListElementType) Element(in Env e)
            {
                if (e.Lists == null || !e.Lists.TryGetValue(_name, out var lv))
                {
                    if (e.Vars.ContainsKey(_name)) throw new ExpressionParseException($"${_name} is not a list", Pos);
                    throw new UnknownVariableException(_name);
                }
                int idx = (int)Math.Round(_index.Eval(e));
                if (idx < 0 || idx >= lv.Count)
                    throw new ExpressionRuntimeException(ErrorCodes.IndexOutOfRange, $"index {idx} is outside ${_name} ({lv.Describe()})", Pos);
                var copy = new ObjectRecord();
                foreach (var (k, v) in lv.Items[idx]) copy[k] = v;
                return (copy, lv.ElementType);
            }
        }

        private sealed partial class Parser
        {
            /// <summary>Parses a reach call from its name (the '(' is current).</summary>
            private Node ParseReachCall(ExpressionFunction fn, Token nameTok, bool allowAccessor)
            {
                var kind = fn.Name switch
                {
                    ReachableFn => ReachKind.Reachable,
                    JointsOfFn  => ReachKind.JointsOf,
                    _           => ReachKind.PoseOf,
                };
                _i++; // '('
                if (Cur.Type == TokType.RParen)
                    throw Error(ArityMessage(fn, 0), nameTok.Pos, "badArity");

                var target = ParseReachTarget(fn, kind);
                TextArg? tool = null, local = null;
                if (Cur.Type == TokType.Comma) { _i++; tool  = ParseTextArg(fn, "tool"); }
                if (Cur.Type == TokType.Comma) { _i++; local = ParseTextArg(fn, "local"); }
                if (Cur.Type == TokType.Comma)
                    throw Error(ArityMessage(fn, 4), nameTok.Pos, "badArity");
                if (Cur.Type != TokType.RParen)
                    throw Error(Cur.Type == TokType.End
                        ? $"Missing ')' after the arguments of {fn.Name}()"
                        : $"Expected ',' or ')' in {fn.Name}() but found {Describe(Cur)}", Cur.Pos);
                _i++;

                string? field = null;
                Node? posIdx = null;
                if (allowAccessor && kind != ReachKind.Reachable)
                {
                    var fields = kind == ReachKind.JointsOf ? JointFields : PoseFields;
                    if (Cur.Type == TokType.Dot && _t[_i + 1].Type == TokType.Word)
                    {
                        var w = _t[_i + 1];
                        field = fields.FirstOrDefault(f => f.Equals(w.Value, StringComparison.OrdinalIgnoreCase));
                        if (field == null)
                            throw Error($"{fn.Name}(…) has no '.{w.Value}' — use {string.Join(", ", fields.Select(f => "." + f))}", w.Pos);
                        _i += 2;
                    }
                    else if (Cur.Type == TokType.LBracket)
                    {
                        _i++;
                        posIdx = ParseTernary();
                        Expect(TokType.RBracket, "]");
                    }
                }
                return new ReachCallNode(kind, fn.Name, target, tool, local, field, posIdx, nameTok.Pos, _src);
            }

            private ReachArg ParseReachTarget(ExpressionFunction fn, ReachKind kind)
            {
                var tok = Cur;
                string what = kind == ReachKind.PoseOf
                    ? "joint values: $j[i] (an Objects list with j1…j4) or jointsOf(…)"
                    : "a point: $pts[i], \"Name\", grid(\"Grid\", row, col), stack(\"Stack\", i) or poseOf(…)";

                if (tok.Type == TokType.Variable)
                {
                    if (_t[_i + 1].Type == TokType.LBracket)
                    {
                        string name = string.Join(".", tok.Parts!);
                        _i += 2; // $name '['
                        var index = ParseTernary();
                        Expect(TokType.RBracket, "]");
                        if (Cur.Type is TokType.Dot or TokType.LBracket)
                            throw Error($"{fn.Name}() takes the whole element — write ${name}[…] without .field", Cur.Pos);
                        return new ListElementArg(name, index, tok.Pos);
                    }
                    if (tok.Parts!.Length == 1 && kind != ReachKind.PoseOf)
                    {
                        _i++;
                        return new SavedPointArg(new TextArg(tok.Parts[0], true, tok.Pos)); // a string variable naming a saved point
                    }
                    throw Error($"{fn.Name}() takes {what}", tok.Pos);
                }

                if (tok.Type == TokType.String && kind != ReachKind.PoseOf)
                {
                    _i++;
                    return new SavedPointArg(new TextArg(tok.Value, false, tok.Pos));
                }

                if (tok.Type == TokType.Word && _t[_i + 1].Type == TokType.LParen)
                {
                    if (kind != ReachKind.PoseOf &&
                        (tok.Value.Equals("grid", StringComparison.OrdinalIgnoreCase) || tok.Value.Equals("stack", StringComparison.OrdinalIgnoreCase)))
                    {
                        bool stack = tok.Value.Equals("stack", StringComparison.OrdinalIgnoreCase);
                        _i += 2; // word '('
                        var name = ParseTextArg(fn, stack ? "stack" : "grid");
                        if (Cur.Type != TokType.Comma) throw Error($"{tok.Value}(\"Name\", {(stack ? "index" : "row, col")}) needs {(stack ? "an index" : "a row and a column")}", Cur.Pos);
                        _i++;
                        var a = ParseTernary();
                        Node? b = null;
                        if (!stack && Cur.Type == TokType.Comma) { _i++; b = ParseTernary(); }
                        Expect(TokType.RParen, ")");
                        return new CellArg(stack, name, a, b, tok.Pos);
                    }
                    if (ResolveFunction(tok.Value) is { } inner && IsReachFunction(inner.Name) && inner.Name != ReachableFn)
                    {
                        _i++; // the word; ParseReachCall starts at '('
                        var call = (ReachCallNode)ParseReachCall(inner, tok, allowAccessor: false);
                        return new NestedArg(call);
                    }
                }

                throw Error($"{fn.Name}() takes {what}", tok.Pos);
            }

            private TextArg ParseTextArg(ExpressionFunction fn, string what)
            {
                var tok = Cur;
                if (tok.Type == TokType.String) { _i++; return new TextArg(tok.Value, false, tok.Pos); }
                if (tok.Type == TokType.Variable && tok.Parts!.Length == 1 && _t[_i + 1].Type != TokType.LBracket)
                {
                    _i++;
                    return new TextArg(tok.Parts[0], true, tok.Pos);
                }
                throw Error($"{fn.Name}(): the {what} is a name in quotes (\"\" = the active one, \"none\" = none) or a text variable", tok.Pos);
            }
        }
    }
}
