using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;

namespace Controller.RobotControl
{
    /// <summary>
    /// Thrown by ExpressionEvaluator when a <c>$variable</c> reference cannot be resolved
    /// in any of the supplied variable dictionaries (or the property source).
    /// </summary>
    public class UnknownVariableException : Exception
    {
        public string VariableName { get; }
        public UnknownVariableException(string variableName)
            : base($"Unknown variable '${variableName}'")
        {
            VariableName = variableName;
        }
    }

    /// <summary>
    /// A syntax error in an expression: unbalanced parentheses, a stray token, an unknown
    /// function, a function called with the wrong number of arguments. Thrown by
    /// <see cref="ExpressionEvaluator.Evaluate"/> instead of evaluating to a fallback.
    /// </summary>
    public class ExpressionParseException : Exception
    {
        /// <summary>0-based character offset into <see cref="Expression"/> where the problem is.</summary>
        public int Position { get; }

        /// <summary><c>expressionSyntax</c>, <c>unknownFunction</c> or <c>badArity</c> — the validation code.</summary>
        public string Code { get; }

        /// <summary>The expression text that failed to parse.</summary>
        public string Expression { get; }

        public ExpressionParseException(string message, int position, string code = "expressionSyntax", string expression = "")
            : base(message)
        {
            Position   = position;
            Code       = code;
            Expression = expression;
        }
    }

    /// <summary>
    /// Read-only system values (<c>$robot.x</c>, <c>$program.runCount</c>, <c>$time.hour</c> …)
    /// that the evaluator consults when a <c>$name</c> is not a variable or IO value.
    /// Resolved lazily at lookup time; never materialised into the variable snapshot.
    /// </summary>
    public interface IPropertySource
    {
        /// <summary>Case-insensitive lookup of a dotted property name (without the <c>$</c>).</summary>
        bool TryGet(string name, out double value);

        /// <summary>Every property this source can currently answer, for pickers and validation.</summary>
        IEnumerable<(string Name, string Description, string Type)> List();
    }

    /// <summary>One entry of the expression function table.</summary>
    public sealed class ExpressionFunction
    {
        internal delegate double NumericFn(ReadOnlySpan<double> args);

        public string Name        { get; }
        public int    MinArgs     { get; }
        /// <summary><see cref="int.MaxValue"/> for variadic functions.</summary>
        public int    MaxArgs     { get; }
        public string Signature   { get; }
        public string Description { get; }
        /// <summary>True for the list functions (len/sum/avg/minOf/maxOf), whose one argument is a list variable.</summary>
        public bool   TakesList   { get; }

        internal NumericFn?          Numeric { get; }
        internal Func<ListVar, double>? OverList { get; }

        internal ExpressionFunction(string name, int min, int max, string signature, string description,
                                    NumericFn? numeric = null, Func<ListVar, double>? overList = null)
        {
            Name = name; MinArgs = min; MaxArgs = max; Signature = signature; Description = description;
            Numeric = numeric; OverList = overList; TakesList = overList != null;
        }
    }

    /// <summary>What an expression reference is, as reported by <see cref="ExpressionEvaluator.References"/>.</summary>
    internal enum ExprRefKind
    {
        /// <summary><c>$name</c> or <c>$a.b.c</c> (IO, property, <c>$list.length</c>).</summary>
        Plain,
        /// <summary><c>$name[…]</c> — must be a list.</summary>
        Indexed,
        /// <summary>The argument of a list function, <c>len($name)</c> — must be a list.</summary>
        ListArgument,
        /// <summary>A word with no <c>$</c> that is not a literal, operator or function — evaluates to 0.</summary>
        BareWord,
        /// <summary>A function call (the name is the function name).</summary>
        Function,
    }

    /// <summary>One reference found in an expression.</summary>
    internal readonly record struct ExprRef(ExprRefKind Kind, string Name, string[] Parts, int Position);

    /// <summary>
    /// Evaluates numeric expressions that may reference program variables, IO and properties.
    ///
    /// Supported syntax:
    ///   Literals     : 3.14  -5  100  1e3  true  false
    ///   Variables    : $varName
    ///   IO           : $stb.in1  $relay.2  $nano.Board.pin    (dotted names, longest match wins)
    ///   Properties   : $robot.x  $program.runCount  $time.hour (read-only; see <see cref="IPropertySource"/>)
    ///   List count   : $listName.length  or  $listName.count
    ///   Number elem. : $listName[indexExpr]
    ///   Point comp.  : $pointsVar[indexExpr].x   (x/y/z/rx/ry/rz)
    ///   Point comp.  : $pointsVar[indexExpr][0]  (0=x 1=y 2=z 3=rx 4=ry 5=rz)
    ///   Record field : $objVar[indexExpr].fieldName
    ///   Arithmetic   : +  -  *  /  %  ^     (% remainder, ^ power — right-associative)
    ///   Comparison   : ==  !=  &lt;  &lt;=  &gt;  &gt;=   (yield 1 or 0; == within 1e-9)
    ///   Logic        : and  or  not   (also &amp;&amp;  ||  !)
    ///   Conditional  : cond ? a : b   (lowest precedence, right-associative)
    ///   Functions    : abs(x) min(a,b,…) round(x,2) sin(deg) len($list) … — see <see cref="Functions"/>
    ///   Grouping     : (expr)       Braces { } are ignored, so "{$x + 1}" works in numeric fields.
    ///
    /// Precedence, tightest first:
    ///   ^  ·  unary - +  ·  * / %  ·  + -  ·  comparison  ·  not  ·  and  ·  or  ·  ? :
    /// A leading "not" sits above comparison, so "not $a &gt; 5" is "not ($a &gt; 5)" (as it
    /// always has been); "not" written as an operand ("$a == not $b") binds like unary minus.
    /// "-2 ^ 2" is -(2 ^ 2) = -4, and "2 ^ 3 ^ 2" is 2 ^ 9.
    ///
    /// Comparison and logic yield 1 or 0, which is exactly how a boolean variable is
    /// stored, so a comparison can be assigned to one directly. Anything non-zero counts
    /// as true on the way in. Both sides of and/or and both branches of ?: / if() are
    /// always evaluated — nothing has side effects, and it means a typo'd variable fails
    /// the run whichever way the condition goes.
    ///
    /// A bare word (no $) that is not a literal, operator or function evaluates to 0 —
    /// legacy behaviour kept on purpose; the validator warns about it.
    ///
    /// Syntax errors throw <see cref="ExpressionParseException"/>; an unresolvable name throws
    /// <see cref="UnknownVariableException"/>. Parsed expressions are cached, so evaluating
    /// the same text every tick allocates nothing beyond what the lookups do.
    ///
    /// Number, point and record lists all arrive in one <see cref="ListVar"/> dictionary;
    /// the element type decides which of the index forms above applies.
    /// </summary>
    internal static class ExpressionEvaluator
    {
        // ── Public entry points ───────────────────────────────────────────────

        public static double Evaluate(
            string expr,
            Dictionary<string, double> variables,
            Dictionary<string, ListVar>? listVariables = null,
            IPropertySource? properties = null)
        {
            var node = GetOrParse(expr);
            if (node == null) return 0; // empty expression — 0, as it always was
            return node.Eval(new Env(variables, listVariables, properties));
        }

        /// <summary>Syntax check only — no variable resolution.</summary>
        public static bool TryParse(string expr, out string? error) => TryParse(expr, out error, out _);

        /// <summary>Syntax check only — no variable resolution. <paramref name="position"/> is -1 when valid.</summary>
        public static bool TryParse(string expr, out string? error, out int position)
        {
            try
            {
                GetOrParse(expr);
                error = null;
                position = -1;
                return true;
            }
            catch (ExpressionParseException ex)
            {
                error = ex.Message;
                position = ex.Position;
                return false;
            }
        }

        /// <summary>Parses <paramref name="expr"/>; throws <see cref="ExpressionParseException"/> on a syntax error.</summary>
        internal static void Validate(string expr) => GetOrParse(expr);

        /// <summary>
        /// Every <c>$name</c> in the expression, dotted names joined ("stb.in1", "robot.x",
        /// "list.length"). List index internals (".x", "[0]") are not included, but
        /// variables used inside an index expression are. Throws on a syntax error.
        /// </summary>
        public static IEnumerable<string> ReferencedNames(string expr) =>
            References(expr)
                .Where(r => r.Kind is ExprRefKind.Plain or ExprRefKind.Indexed or ExprRefKind.ListArgument)
                .Select(r => r.Name)
                .Distinct(StringComparer.OrdinalIgnoreCase)
                .ToList();

        /// <summary>Every reference (variables, list arguments, bare words, function calls) with positions.</summary>
        internal static List<ExprRef> References(string expr)
        {
            var list = new List<ExprRef>();
            GetOrParse(expr)?.Collect(list);
            return list;
        }

        /// <summary>
        /// True when the expression's top-level operation is a comparison or logic operator,
        /// i.e. it naturally reads as true/false. Throws on a syntax error.
        /// </summary>
        public static bool IsBooleanExpression(string expr) => GetOrParse(expr)?.IsBoolean ?? false;

        /// <summary>
        /// The numeric comparison behind both the operators below and <c>ConditionGroup</c>
        /// items, so <c>==</c> means one thing no matter which of the two a program uses.
        ///
        /// Equality is within 1e-9 rather than exact: these are doubles that have usually
        /// been through arithmetic first, and someone writing a program reasonably expects
        /// 0.1 + 0.2 to equal 0.3.
        /// </summary>
        public static bool Compare(double left, string op, double right)
        {
            const double eps = 1e-9;
            return op switch
            {
                "==" => Math.Abs(left - right) < eps,
                "!=" => Math.Abs(left - right) >= eps,
                ">"  => left > right,
                ">=" => left >= right,
                "<"  => left < right,
                "<=" => left <= right,
                _    => false,
            };
        }

        /// <summary>Whether <paramref name="name"/> is a function in <see cref="Functions"/> (case-insensitive).</summary>
        public static bool IsFunctionName(string name) => FunctionsByName.ContainsKey(name);

        // ── Parse cache ───────────────────────────────────────────────────────

        private const int CacheLimit = 4096;
        private static readonly ConcurrentDictionary<string, Node?> Cache = new(StringComparer.Ordinal);

        private static Node? GetOrParse(string expr)
        {
            expr ??= "";
            if (Cache.TryGetValue(expr, out var cached)) return cached;
            var node = new Parser(expr).ParseAll();
            // Crude bound: expressions come from saved programs, so the set is small in
            // practice; clearing keeps a pathological stream of distinct texts from growing it.
            if (Cache.Count >= CacheLimit) Cache.Clear();
            Cache[expr] = node;
            return node;
        }

        // ── Function table ────────────────────────────────────────────────────

        private const double DegToRad = Math.PI / 180.0;
        private const double RadToDeg = 180.0 / Math.PI;
        private const int Variadic = int.MaxValue;

        private static double RoundAway(double x, double digits)
        {
            int d = (int)Math.Clamp(Math.Round(digits), -15, 15);
            if (d == 0) return Math.Round(x, MidpointRounding.AwayFromZero);
            double f = Math.Pow(10, d);
            return Math.Round(x * f, MidpointRounding.AwayFromZero) / f;
        }

        private static double Rem(double a, double b) => b != 0 ? a % b : 0;

        private static IEnumerable<double> Scalars(ListVar l)
        {
            foreach (var item in l.Items) yield return item.Scalar;
        }

        /// <summary>The function table, in documentation order. Names are case-insensitive.</summary>
        public static IReadOnlyList<ExpressionFunction> Functions { get; } = new List<ExpressionFunction>
        {
            new("abs",   1, 1, "abs(x)",   "Absolute value",                      a => Math.Abs(a[0])),
            new("sign",  1, 1, "sign(x)",  "-1, 0 or 1",                          a => Math.Sign(a[0])),
            new("sqrt",  1, 1, "sqrt(x)",  "Square root",                         a => Math.Sqrt(a[0])),
            new("pow",   2, 2, "pow(x, y)", "x raised to y (same as x ^ y)",      a => Math.Pow(a[0], a[1])),
            new("min",   1, Variadic, "min(a, b, …)", "Smallest argument",        a => { double m = a[0]; for (int i = 1; i < a.Length; i++) m = Math.Min(m, a[i]); return m; }),
            new("max",   1, Variadic, "max(a, b, …)", "Largest argument",         a => { double m = a[0]; for (int i = 1; i < a.Length; i++) m = Math.Max(m, a[i]); return m; }),
            new("clamp", 3, 3, "clamp(x, lo, hi)", "x limited to lo…hi",          a => Math.Min(Math.Max(a[0], a[1]), a[2])),
            new("round", 1, 2, "round(x) / round(x, digits)", "Round half away from zero, optionally to digits decimals",
                                                                                   a => RoundAway(a[0], a.Length > 1 ? a[1] : 0)),
            new("floor", 1, 1, "floor(x)", "Round down",                          a => Math.Floor(a[0])),
            new("ceil",  1, 1, "ceil(x)",  "Round up",                            a => Math.Ceiling(a[0])),
            new("trunc", 1, 1, "trunc(x)", "Drop the fraction (toward zero)",     a => Math.Truncate(a[0])),
            new("mod",   2, 2, "mod(a, b)", "Remainder, same as a % b (0 when b is 0)", a => Rem(a[0], a[1])),
            new("sin",   1, 1, "sin(deg)", "Sine of an angle in degrees",         a => Math.Sin(a[0] * DegToRad)),
            new("cos",   1, 1, "cos(deg)", "Cosine of an angle in degrees",       a => Math.Cos(a[0] * DegToRad)),
            new("tan",   1, 1, "tan(deg)", "Tangent of an angle in degrees",      a => Math.Tan(a[0] * DegToRad)),
            new("asin",  1, 1, "asin(x)",  "Arcsine, in degrees",                 a => Math.Asin(a[0]) * RadToDeg),
            new("acos",  1, 1, "acos(x)",  "Arccosine, in degrees",               a => Math.Acos(a[0]) * RadToDeg),
            new("atan",  1, 1, "atan(x)",  "Arctangent, in degrees",              a => Math.Atan(a[0]) * RadToDeg),
            new("atan2", 2, 2, "atan2(y, x)", "Angle of (x, y) in degrees",       a => Math.Atan2(a[0], a[1]) * RadToDeg),
            new("deg",   1, 1, "deg(rad)", "Radians to degrees",                  a => a[0] * RadToDeg),
            new("rad",   1, 1, "rad(deg)", "Degrees to radians",                  a => a[0] * DegToRad),
            new("hypot", 2, 2, "hypot(x, y)", "sqrt(x² + y²)",                    a => Math.Sqrt(a[0] * a[0] + a[1] * a[1])),
            new("dist",  4, 4, "dist(x1, y1, x2, y2)", "Distance between two XY points",
                                                                                   a => Math.Sqrt((a[2] - a[0]) * (a[2] - a[0]) + (a[3] - a[1]) * (a[3] - a[1]))),
            new("dist3", 6, 6, "dist3(x1, y1, z1, x2, y2, z2)", "Distance between two XYZ points",
                                                                                   a => Math.Sqrt((a[3] - a[0]) * (a[3] - a[0]) + (a[4] - a[1]) * (a[4] - a[1]) + (a[5] - a[2]) * (a[5] - a[2]))),
            new("if",    3, 3, "if(cond, a, b)", "a when cond is non-zero, else b (same as cond ? a : b)",
                                                                                   a => a[0] != 0 ? a[1] : a[2]),
            new("len",   1, 1, "len($list)", "Number of elements in a list",      overList: l => l.Count),
            new("sum",   1, 1, "sum($list)", "Sum of a number list",              overList: l => Scalars(l).Sum()),
            new("avg",   1, 1, "avg($list)", "Average of a number list (0 when empty)",
                                                                                   overList: l => l.Count == 0 ? 0 : Scalars(l).Average()),
            new("minOf", 1, 1, "minOf($list)", "Smallest element of a number list (0 when empty)",
                                                                                   overList: l => l.Count == 0 ? 0 : Scalars(l).Min()),
            new("maxOf", 1, 1, "maxOf($list)", "Largest element of a number list (0 when empty)",
                                                                                   overList: l => l.Count == 0 ? 0 : Scalars(l).Max()),
            new("rand",  0, 2, "rand() / rand(lo, hi)", "Uniform random number in [0, 1) or [lo, hi)",
                                                                                   a => a.Length == 0 ? Random.Shared.NextDouble()
                                                                                       : a.Length == 2 ? a[0] + (a[1] - a[0]) * Random.Shared.NextDouble()
                                                                                       : double.NaN),
            new("map",   5, 5, "map(x, inLo, inHi, outLo, outHi)", "Re-map x from one range to another",
                                                                                   a => a[2] == a[1] ? a[3] : a[3] + (a[0] - a[1]) * (a[4] - a[3]) / (a[2] - a[1])),
            new("lerp",  3, 3, "lerp(a, b, t)", "a + (b - a) × t",                a => a[0] + (a[1] - a[0]) * a[2]),
        };

        private static readonly Dictionary<string, ExpressionFunction> FunctionsByName =
            Functions.ToDictionary(f => f.Name, StringComparer.OrdinalIgnoreCase);

        // ── Evaluation environment and AST ────────────────────────────────────

        private readonly struct Env
        {
            public readonly Dictionary<string, double> Vars;
            public readonly Dictionary<string, ListVar>? Lists;
            public readonly IPropertySource? Props;
            public Env(Dictionary<string, double> vars, Dictionary<string, ListVar>? lists, IPropertySource? props)
            { Vars = vars; Lists = lists; Props = props; }
        }

        /// <summary>Non-zero is true, matching how boolean variables are stored.</summary>
        private static bool Truthy(double v) => v != 0;

        private abstract class Node
        {
            public int Pos;
            public abstract double Eval(in Env e);
            public virtual bool IsBoolean => false;
            public virtual void Collect(List<ExprRef> refs) { }
        }

        private sealed class NumNode : Node
        {
            private readonly double _v;
            public NumNode(double v, int pos) { _v = v; Pos = pos; }
            public override double Eval(in Env e) => _v;
        }

        /// <summary>A word with no $ — legacy: evaluates to 0.</summary>
        private sealed class WordNode : Node
        {
            private readonly string _word;
            public WordNode(string w, int pos) { _word = w; Pos = pos; }
            public override double Eval(in Env e) => 0;
            public override void Collect(List<ExprRef> refs) =>
                refs.Add(new ExprRef(ExprRefKind.BareWord, _word, [_word], Pos));
        }

        private static bool IsCountWord(string s) =>
            s.Equals("length", StringComparison.OrdinalIgnoreCase) ||
            s.Equals("count",  StringComparison.OrdinalIgnoreCase);

        private sealed class VarNode : Node
        {
            private readonly string[] _parts;
            private readonly string[] _joined; // _joined[n-1] = first n parts joined with '.'

            public VarNode(string[] parts, int pos)
            {
                _parts  = parts;
                _joined = new string[parts.Length];
                for (int n = 1; n <= parts.Length; n++) _joined[n - 1] = string.Join(".", parts, 0, n);
                Pos = pos;
            }

            public string FullName => _joined[^1];

            public override double Eval(in Env e)
            {
                var lists = e.Lists;

                // .length / .count — element count of any list variable. Only when the name
                // really is a list; otherwise dotted-name lookup below gets a try (an IO key
                // could end in ".count").
                if (_parts.Length == 2 && lists != null && IsCountWord(_parts[1]) &&
                    lists.TryGetValue(_parts[0], out var counted))
                    return counted.Count;

                // Dotted-name lookup — IO variables are injected with dotted keys (stb.in1,
                // relay.1, nano.Board.pin1). Longest candidate first; properties are tried
                // for the full name only, after variables/IO.
                for (int n = _parts.Length; n >= 1; n--)
                {
                    if (e.Vars.TryGetValue(_joined[n - 1], out double val)) return val;
                    if (n == _parts.Length && e.Props != null && e.Props.TryGet(_joined[n - 1], out val)) return val;
                }

                // Known list variable referenced without an index — the legacy 0.
                if (lists != null && lists.ContainsKey(_parts[0])) return 0;

                // Truly unknown identifier — fail loudly. Silently coercing a typo'd
                // variable to 0 can turn a clearance offset into a collision.
                throw new UnknownVariableException(FullName);
            }

            public override void Collect(List<ExprRef> refs) =>
                refs.Add(new ExprRef(ExprRefKind.Plain, FullName, _parts, Pos));
        }

        private sealed class IndexNode : Node
        {
            private readonly string  _name;
            private readonly Node    _index;
            private readonly string? _field;   // .field accessor
            private readonly Node?   _posIdx;  // [n] accessor

            public IndexNode(string name, Node index, string? field, Node? posIdx, int pos)
            { _name = name; _index = index; _field = field; _posIdx = posIdx; Pos = pos; }

            public override double Eval(in Env e)
            {
                if (e.Lists != null && e.Lists.TryGetValue(_name, out var lv))
                {
                    int idx = (int)Math.Round(_index.Eval(e));
                    var item = (idx >= 0 && idx < lv.Count) ? lv.Items[idx] : null;

                    // A number or boolean element *is* the value; any accessor written after
                    // it is ignored ("$nums[0].foo" is "$nums[0]"), as it always was.
                    if (lv.HasScalarElements)
                        return item?.Scalar ?? 0;

                    // .field — a point's axis (.x) and a record's field (.coverage) are the same
                    // lookup. A missing field is 0 rather than a throw: the field set depends on
                    // what produced the record.
                    if (_field != null)
                        return item != null && item.TryGetValue(_field, out double fv) ? fv : 0;

                    // [n] — positional, and only a point list has a defined axis order.
                    if (_posIdx != null)
                    {
                        double compIdx = _posIdx.Eval(e);
                        if (lv.ElementType != ListElementType.Point) return 0;
                        string? axis = ListVar.AxisName((int)Math.Round(compIdx));
                        return axis != null && item != null && item.TryGetValue(axis, out double av) ? av : 0;
                    }

                    return 0; // point or record indexed without an accessor → 0
                }

                // Not a list: the index is ignored and the name resolves like any other.
                if (e.Vars.TryGetValue(_name, out double v)) return v;
                if (e.Props != null && e.Props.TryGet(_name, out v)) return v;
                throw new UnknownVariableException(_name);
            }

            public override void Collect(List<ExprRef> refs)
            {
                refs.Add(new ExprRef(ExprRefKind.Indexed, _name, _name.Split('.'), Pos));
                _index.Collect(refs);
                _posIdx?.Collect(refs);
            }
        }

        private sealed class UnaryNode : Node
        {
            private readonly string _op; // "-", "+", "not"
            private readonly Node _operand;
            public UnaryNode(string op, Node operand, int pos) { _op = op; _operand = operand; Pos = pos; }
            public override double Eval(in Env e)
            {
                double v = _operand.Eval(e);
                return _op switch { "-" => -v, "not" => Truthy(v) ? 0 : 1, _ => v };
            }
            public override bool IsBoolean => _op == "not";
            public override void Collect(List<ExprRef> refs) => _operand.Collect(refs);
        }

        private sealed class BinaryNode : Node
        {
            private readonly string _op;
            private readonly Node _l, _r;
            public BinaryNode(string op, Node l, Node r, int pos) { _op = op; _l = l; _r = r; Pos = pos; }

            public override double Eval(in Env e)
            {
                double a = _l.Eval(e);
                double b = _r.Eval(e);
                switch (_op)
                {
                    case "+": return a + b;
                    case "-": return a - b;
                    case "*": return a * b;
                    case "/": return b != 0 ? a / b : 0;   // div-by-zero → 0 (intentional, long-standing)
                    case "%": return Rem(a, b);
                    case "^": return Math.Pow(a, b);
                    // Both sides are always evaluated — see the class remarks.
                    case "and": return Truthy(a) && Truthy(b) ? 1 : 0;
                    case "or":  return Truthy(a) || Truthy(b) ? 1 : 0;
                    default:    return Compare(a, _op, b) ? 1 : 0;
                }
            }

            public override bool IsBoolean => _op is "and" or "or" or "==" or "!=" or "<" or "<=" or ">" or ">=";

            public override void Collect(List<ExprRef> refs) { _l.Collect(refs); _r.Collect(refs); }
        }

        private sealed class TernaryNode : Node
        {
            private readonly Node _c, _a, _b;
            public TernaryNode(Node c, Node a, Node b, int pos) { _c = c; _a = a; _b = b; Pos = pos; }
            public override double Eval(in Env e)
            {
                bool cond = Truthy(_c.Eval(e));
                double a = _a.Eval(e);
                double b = _b.Eval(e);
                return cond ? a : b;
            }
            public override void Collect(List<ExprRef> refs) { _c.Collect(refs); _a.Collect(refs); _b.Collect(refs); }
        }

        private sealed class CallNode : Node
        {
            private readonly ExpressionFunction _fn;
            private readonly Node[] _args;
            public CallNode(ExpressionFunction fn, Node[] args, int pos) { _fn = fn; _args = args; Pos = pos; }

            public override double Eval(in Env e)
            {
                int n = _args.Length;
                Span<double> buf = n <= 8 ? stackalloc double[8] : new double[n];
                buf = buf[..n];
                for (int i = 0; i < n; i++) buf[i] = _args[i].Eval(e);
                return _fn.Numeric!(buf);
            }

            public override bool IsBoolean => false;

            public override void Collect(List<ExprRef> refs)
            {
                refs.Add(new ExprRef(ExprRefKind.Function, _fn.Name, [_fn.Name], Pos));
                foreach (var a in _args) a.Collect(refs);
            }
        }

        private sealed class ListCallNode : Node
        {
            private readonly ExpressionFunction _fn;
            private readonly string _list;
            private readonly int _argPos;
            public ListCallNode(ExpressionFunction fn, string list, int pos, int argPos)
            { _fn = fn; _list = list; Pos = pos; _argPos = argPos; }

            public override double Eval(in Env e)
            {
                if (e.Lists != null && e.Lists.TryGetValue(_list, out var lv))
                    return _fn.OverList!(lv);
                throw new UnknownVariableException(_list);
            }

            public override void Collect(List<ExprRef> refs)
            {
                refs.Add(new ExprRef(ExprRefKind.Function, _fn.Name, [_fn.Name], Pos));
                refs.Add(new ExprRef(ExprRefKind.ListArgument, _list, [_list], _argPos));
            }
        }

        // ── Tokenizer ─────────────────────────────────────────────────────────

        private enum TokType { Number, Variable, Word, Op, LParen, RParen, LBracket, RBracket, Dot, Comma, Question, Colon, End }

        private readonly record struct Token(TokType Type, string Value, int Pos, string[]? Parts = null, double Num = 0);

        private static bool IsIdentChar(char c) => char.IsLetterOrDigit(c) || c == '_';

        private static bool IsLogicWord(string w) =>
            w.Equals("and", StringComparison.OrdinalIgnoreCase) ||
            w.Equals("or",  StringComparison.OrdinalIgnoreCase) ||
            w.Equals("not", StringComparison.OrdinalIgnoreCase);

        private static List<Token> Tokenize(string expr)
        {
            var tokens = new List<Token>();
            int i = 0;

            while (i < expr.Length)
            {
                char c = expr[i];

                // Whitespace, and braces: the shared template input lets users type "{$x + 1}"
                // into numeric fields, so braces have always been skipped here.
                if (char.IsWhiteSpace(c) || c == '{' || c == '}') { i++; continue; }

                // Variable: $identifier with an optional dotted chain ($stb.in1, $relay.1,
                // $nano.Board.pin, $aux.dev.0.position). The chain only continues across a
                // '.' that is immediately followed by an identifier character.
                if (c == '$')
                {
                    int start = i;
                    i++;
                    int s = i;
                    while (i < expr.Length && IsIdentChar(expr[i])) i++;
                    var parts = new List<string> { expr[s..i] };
                    while (i + 1 < expr.Length && expr[i] == '.' && IsIdentChar(expr[i + 1]))
                    {
                        i++;
                        s = i;
                        while (i < expr.Length && IsIdentChar(expr[i])) i++;
                        parts.Add(expr[s..i]);
                    }
                    tokens.Add(new Token(TokType.Variable, "$" + string.Join(".", parts), start, parts.ToArray()));
                    continue;
                }

                // Bare word — a field name after a dot, a function name, true/false, or one
                // of the word-spelled logic operators. After a dot it is always a field name:
                // record fields are named by whatever produced the record, so "$r[0].not"
                // has to keep working.
                if (char.IsLetter(c) || c == '_')
                {
                    int start = i;
                    while (i < expr.Length && IsIdentChar(expr[i])) i++;
                    string word = expr[start..i];
                    bool afterDot = tokens.Count > 0 && tokens[^1].Type == TokType.Dot;
                    tokens.Add(!afterDot && IsLogicWord(word)
                        ? new Token(TokType.Op,   word.ToLowerInvariant(), start)
                        : new Token(TokType.Word, word, start));
                    continue;
                }

                // Number: digits with an optional fraction and exponent (".5" too)
                if (char.IsDigit(c) || (c == '.' && i + 1 < expr.Length && char.IsDigit(expr[i + 1])))
                {
                    int start = i;
                    while (i < expr.Length && char.IsDigit(expr[i])) i++;
                    if (i < expr.Length && expr[i] == '.')
                    {
                        i++;
                        while (i < expr.Length && char.IsDigit(expr[i])) i++;
                    }
                    if (i < expr.Length && (expr[i] == 'e' || expr[i] == 'E'))
                    {
                        int save = i;
                        i++;
                        if (i < expr.Length && (expr[i] == '+' || expr[i] == '-')) i++;
                        if (i < expr.Length && char.IsDigit(expr[i]))
                            while (i < expr.Length && char.IsDigit(expr[i])) i++;
                        else
                            i = save; // not an exponent — "2e" is 2 then a word
                    }
                    if (i < expr.Length && expr[i] == '.' && i + 1 < expr.Length && char.IsDigit(expr[i + 1]))
                        throw new ExpressionParseException($"Invalid number '{expr[start..(i + 1)]}…'", start, expression: expr);
                    string text = expr[start..i];
                    if (!double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out double num))
                        throw new ExpressionParseException($"Invalid number '{text}'", start, expression: expr);
                    tokens.Add(new Token(TokType.Number, text, start, Num: num));
                    continue;
                }

                // Two-character operators, before the single-character ones so ">=" is never
                // read as ">" followed by "=".
                if (i + 1 < expr.Length)
                {
                    string pair = expr.Substring(i, 2);
                    if (pair is "==" or "!=" or "<=" or ">=")
                    {
                        tokens.Add(new Token(TokType.Op, pair, i));
                        i += 2;
                        continue;
                    }
                    // The C-style spellings fold into the word forms, so the parser and any
                    // error message only ever deal with one name per operator.
                    if (pair == "&&") { tokens.Add(new Token(TokType.Op, "and", i)); i += 2; continue; }
                    if (pair == "||") { tokens.Add(new Token(TokType.Op, "or",  i)); i += 2; continue; }
                }

                switch (c)
                {
                    case '+': case '-': case '*': case '/': case '%': case '^': case '<': case '>':
                        tokens.Add(new Token(TokType.Op, c.ToString(), i)); i++; continue;
                    case '!': tokens.Add(new Token(TokType.Op, "not", i)); i++; continue;
                    // A lone '=' means '=='. Nothing in this language assigns, so there is no
                    // other thing it could mean.
                    case '=': tokens.Add(new Token(TokType.Op, "==", i)); i++; continue;
                    case '(': tokens.Add(new Token(TokType.LParen,   "(", i)); i++; continue;
                    case ')': tokens.Add(new Token(TokType.RParen,   ")", i)); i++; continue;
                    case '[': tokens.Add(new Token(TokType.LBracket, "[", i)); i++; continue;
                    case ']': tokens.Add(new Token(TokType.RBracket, "]", i)); i++; continue;
                    case '.': tokens.Add(new Token(TokType.Dot,      ".", i)); i++; continue;
                    case ',': tokens.Add(new Token(TokType.Comma,    ",", i)); i++; continue;
                    case '?': tokens.Add(new Token(TokType.Question, "?", i)); i++; continue;
                    case ':': tokens.Add(new Token(TokType.Colon,    ":", i)); i++; continue;
                }

                throw new ExpressionParseException($"Unexpected character '{c}'", i, expression: expr);
            }

            tokens.Add(new Token(TokType.End, "", expr.Length));
            return tokens;
        }

        // ── Recursive descent ─────────────────────────────────────────────────

        private sealed class Parser
        {
            private readonly string _src;
            private List<Token> _t = null!;
            private int _i;

            public Parser(string src) => _src = src;

            private Token Cur => _t[_i];

            private ExpressionParseException Error(string msg, int pos, string code = "expressionSyntax") =>
                new(msg, pos, code, _src);

            private static string Describe(Token t) => t.Type == TokType.End ? "end of expression" : $"'{t.Value}'";

            private bool IsOp(string op) => Cur.Type == TokType.Op && Cur.Value == op;

            public Node? ParseAll()
            {
                _t = Tokenize(_src);
                if (_t.Count == 1) return null; // nothing but whitespace/braces

                var node = ParseTernary();
                if (Cur.Type != TokType.End)
                {
                    if (Cur.Type == TokType.RParen)
                        throw Error("Unbalanced ')' — no matching '('", Cur.Pos);
                    throw Error($"Unexpected {Describe(Cur)}", Cur.Pos);
                }
                return node;
            }

            private Node ParseTernary()
            {
                var cond = ParseOr();
                if (Cur.Type != TokType.Question) return cond;
                int pos = Cur.Pos;
                _i++;
                var a = ParseTernary();
                if (Cur.Type != TokType.Colon)
                    throw Error($"Expected ':' in 'cond ? a : b' but found {Describe(Cur)}", Cur.Pos);
                _i++;
                var b = ParseTernary();
                return new TernaryNode(cond, a, b, pos);
            }

            private Node ParseOr()
            {
                var left = ParseAnd();
                while (IsOp("or"))
                {
                    int pos = Cur.Pos; _i++;
                    left = new BinaryNode("or", left, ParseAnd(), pos);
                }
                return left;
            }

            private Node ParseAnd()
            {
                var left = ParseNot();
                while (IsOp("and"))
                {
                    int pos = Cur.Pos; _i++;
                    left = new BinaryNode("and", left, ParseNot(), pos);
                }
                return left;
            }

            // A leading "not" sits above comparison: "not $a > 5" reads as "not ($a > 5)".
            private Node ParseNot()
            {
                if (IsOp("not"))
                {
                    int pos = Cur.Pos; _i++;
                    return new UnaryNode("not", ParseNot(), pos);
                }
                return ParseComparison();
            }

            private static readonly string[] CompareOps = ["==", "!=", "<=", ">=", "<", ">"];

            private Node ParseComparison()
            {
                var left = ParseAddSub();
                // Left-associative, as in C and JavaScript: "1 < 2 < 3" is "(1 < 2) < 3".
                while (Cur.Type == TokType.Op && Array.IndexOf(CompareOps, Cur.Value) >= 0)
                {
                    var op = Cur.Value; int pos = Cur.Pos; _i++;
                    left = new BinaryNode(op, left, ParseAddSub(), pos);
                }
                return left;
            }

            private Node ParseAddSub()
            {
                var left = ParseMulDiv();
                while (IsOp("+") || IsOp("-"))
                {
                    var op = Cur.Value; int pos = Cur.Pos; _i++;
                    left = new BinaryNode(op, left, ParseMulDiv(), pos);
                }
                return left;
            }

            private Node ParseMulDiv()
            {
                var left = ParseUnary();
                while (IsOp("*") || IsOp("/") || IsOp("%"))
                {
                    var op = Cur.Value; int pos = Cur.Pos; _i++;
                    left = new BinaryNode(op, left, ParseUnary(), pos);
                }
                return left;
            }

            private Node ParseUnary()
            {
                if (IsOp("-") || IsOp("+") || IsOp("not"))
                {
                    var op = Cur.Value; int pos = Cur.Pos; _i++;
                    return new UnaryNode(op, ParseUnary(), pos);
                }
                return ParsePower();
            }

            // Right-associative and tighter than unary minus: -2^2 = -4, 2^3^2 = 2^9, and the
            // exponent may carry its own sign (2^-1).
            private Node ParsePower()
            {
                var b = ParsePrimary();
                if (IsOp("^"))
                {
                    int pos = Cur.Pos; _i++;
                    return new BinaryNode("^", b, ParseUnary(), pos);
                }
                return b;
            }

            private void Expect(TokType type, string what)
            {
                if (Cur.Type != type)
                    throw Error($"Expected '{what}' but found {Describe(Cur)}", Cur.Pos);
                _i++;
            }

            private Node ParsePrimary()
            {
                var tok = Cur;
                switch (tok.Type)
                {
                    case TokType.Number:
                        _i++;
                        return new NumNode(tok.Num, tok.Pos);

                    case TokType.LParen:
                    {
                        _i++;
                        // Back to the top of the chain — parentheses hold a full expression.
                        var inner = ParseTernary();
                        if (Cur.Type != TokType.RParen)
                            throw Error(Cur.Type == TokType.End
                                ? "Missing ')' — unbalanced parentheses"
                                : $"Expected ')' but found {Describe(Cur)}", Cur.Type == TokType.End ? tok.Pos : Cur.Pos);
                        _i++;
                        return inner;
                    }

                    case TokType.Variable:
                        _i++;
                        return ParseVariableTail(tok);

                    case TokType.Word:
                    {
                        _i++;
                        if (Cur.Type == TokType.LParen) return ParseCall(tok);
                        if (tok.Value.Equals("true",  StringComparison.OrdinalIgnoreCase)) return new NumNode(1, tok.Pos);
                        if (tok.Value.Equals("false", StringComparison.OrdinalIgnoreCase)) return new NumNode(0, tok.Pos);
                        // Legacy: a bare word is 0. The validator flags it.
                        return new WordNode(tok.Value, tok.Pos);
                    }

                    case TokType.End:
                        throw Error("Expected a value but the expression ended", tok.Pos);

                    case TokType.RParen:
                        throw Error("Expected a value before ')'", tok.Pos);

                    default:
                        throw Error($"Expected a value but found {Describe(tok)}", tok.Pos);
                }
            }

            private Node ParseVariableTail(Token tok)
            {
                var parts = tok.Parts!;
                if (Cur.Type != TokType.LBracket)
                    return new VarNode(parts, tok.Pos);

                // $name[indexExpr] then an optional accessor: .field or [n]
                string name = string.Join(".", parts);
                _i++; // '['
                var index = ParseTernary();
                Expect(TokType.RBracket, "]");

                string? field = null;
                Node? posIdx = null;
                if (Cur.Type == TokType.Dot && _t[_i + 1].Type == TokType.Word)
                {
                    field = _t[_i + 1].Value;
                    _i += 2;
                }
                else if (Cur.Type == TokType.LBracket)
                {
                    _i++;
                    posIdx = ParseTernary();
                    Expect(TokType.RBracket, "]");
                }
                return new IndexNode(name, index, field, posIdx, tok.Pos);
            }

            private Node ParseCall(Token nameTok)
            {
                if (!FunctionsByName.TryGetValue(nameTok.Value, out var fn))
                    throw Error($"Unknown function '{nameTok.Value}'", nameTok.Pos, "unknownFunction");

                _i++; // '('
                var args = new List<Node>();
                var argTokens = new List<Token>();
                if (Cur.Type != TokType.RParen)
                {
                    while (true)
                    {
                        argTokens.Add(Cur);
                        args.Add(ParseTernary());
                        if (Cur.Type == TokType.Comma) { _i++; continue; }
                        break;
                    }
                }
                if (Cur.Type != TokType.RParen)
                    throw Error(Cur.Type == TokType.End
                        ? $"Missing ')' after the arguments of {fn.Name}()"
                        : $"Expected ',' or ')' in {fn.Name}() but found {Describe(Cur)}", Cur.Pos);
                _i++;

                // rand() takes none or two — one bound on its own means nothing.
                bool badRand = fn.Name == "rand" && args.Count == 1;
                if (args.Count < fn.MinArgs || args.Count > fn.MaxArgs || badRand)
                {
                    if (badRand)
                        throw Error($"rand() takes 0 or 2 arguments, got 1 — {fn.Signature}", nameTok.Pos, "badArity");
                    string want = fn.MaxArgs == int.MaxValue ? $"at least {fn.MinArgs}"
                                : fn.MinArgs == fn.MaxArgs  ? $"{fn.MinArgs}"
                                : $"{fn.MinArgs} to {fn.MaxArgs}";
                    throw Error($"{fn.Name}() takes {want} argument{(want == "1" ? "" : "s")}, got {args.Count} — {fn.Signature}",
                                nameTok.Pos, "badArity");
                }

                if (fn.TakesList)
                {
                    // The one argument must be a plain list reference: len($pts), not len($pts[0]).
                    var at = argTokens[0];
                    if (args[0] is not VarNode vn || at.Type != TokType.Variable || at.Parts!.Length != 1)
                        throw Error($"{fn.Name}() takes a list variable, e.g. {fn.Signature}", at.Pos);
                    return new ListCallNode(fn, vn.FullName, nameTok.Pos, at.Pos);
                }

                return new CallNode(fn, args.ToArray(), nameTok.Pos);
            }
        }
    }
}
