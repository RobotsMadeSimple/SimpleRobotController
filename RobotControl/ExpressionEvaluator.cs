using System;
using System.Collections.Generic;
using System.Globalization;

namespace Controller.RobotControl
{
    /// <summary>
    /// Thrown by ExpressionEvaluator when a <c>$variable</c> reference cannot be resolved
    /// in any of the supplied variable dictionaries.
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
    /// Evaluates simple math expressions that may reference program variables.
    ///
    /// Supported syntax:
    ///   Variables    : $varName
    ///   List count   : $listName.length  or  $listName.count
    ///   Number elem. : $listName[indexExpr]
    ///   Point comp.  : $pointsVar[indexExpr].x   (x/y/z/rx/ry/rz)
    ///   Point comp.  : $pointsVar[indexExpr][0]  (0=x 1=y 2=z 3=rx 4=ry 5=rz)
    ///   Record field : $objVar[indexExpr].fieldName
    ///   Literals     : 3.14  -5  100  true  false
    ///   Arithmetic   : +  -  *  /
    ///   Comparison   : ==  !=  &lt;  &lt;=  &gt;  &gt;=   (yield 1 or 0)
    ///   Logic        : and  or  not   (also &amp;&amp;  ||  !)
    ///   Grouping     : (expr)
    ///   Precedence   : tightest first — * /  then  + -  then  comparison  then  not  then
    ///                  and  then  or. So "not $a &gt; 5" is "not ($a &gt; 5)", and
    ///                  "$a &gt; 1 and $b &gt; 2" needs no parentheses.
    ///
    /// Comparison and logic yield 1 or 0, which is exactly how a boolean variable is
    /// stored, so a comparison can be assigned to one directly. Anything non-zero counts
    /// as true on the way in.
    ///
    /// Number, point and record lists all arrive in one <see cref="ListVar"/> dictionary;
    /// the element type decides which of the three index forms above applies.
    /// </summary>
    internal static class ExpressionEvaluator
    {
        private enum TokType { Number, Variable, Op, LParen, RParen, LBracket, RBracket, Dot, Word }
        private readonly record struct Token(TokType Type, string Value);

        // ── Public entry point ────────────────────────────────────────────────

        public static double Evaluate(
            string expr,
            Dictionary<string, double> variables,
            Dictionary<string, ListVar>? listVariables = null)
        {
            var tokens = Tokenize(expr.Trim());
            int idx = 0;
            return ParseOr(tokens, ref idx, variables, listVariables);
        }

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

        /// <summary>Non-zero is true, matching how boolean variables are stored.</summary>
        private static bool Truthy(double v) => v != 0;

        private static bool IsLogicWord(string w) =>
            w.Equals("and", StringComparison.OrdinalIgnoreCase) ||
            w.Equals("or",  StringComparison.OrdinalIgnoreCase) ||
            w.Equals("not", StringComparison.OrdinalIgnoreCase);

        // ── Recursive descent ─────────────────────────────────────────────────

        private static double ParseOr(List<Token> t, ref int i,
            Dictionary<string, double> vars, Dictionary<string, ListVar>? lists)
        {
            double left = ParseAnd(t, ref i, vars, lists);
            while (i < t.Count && t[i].Type == TokType.Op && t[i].Value == "or")
            {
                i++;
                // Both sides are always evaluated. Nothing in this language has a side
                // effect, so short-circuiting would only change how fast a wrong answer
                // arrives — and skipping the right side would leave its tokens unconsumed.
                double right = ParseAnd(t, ref i, vars, lists);
                left = Truthy(left) || Truthy(right) ? 1 : 0;
            }
            return left;
        }

        private static double ParseAnd(List<Token> t, ref int i,
            Dictionary<string, double> vars, Dictionary<string, ListVar>? lists)
        {
            double left = ParseNot(t, ref i, vars, lists);
            while (i < t.Count && t[i].Type == TokType.Op && t[i].Value == "and")
            {
                i++;
                double right = ParseNot(t, ref i, vars, lists);
                left = Truthy(left) && Truthy(right) ? 1 : 0;
            }
            return left;
        }

        private static double ParseNot(List<Token> t, ref int i,
            Dictionary<string, double> vars, Dictionary<string, ListVar>? lists)
        {
            if (i < t.Count && t[i].Type == TokType.Op && t[i].Value == "not")
            {
                i++;
                // Recurses into itself rather than into the comparison layer, so "not" sits
                // above comparison: "not $a > 5" reads as "not ($a > 5)".
                return Truthy(ParseNot(t, ref i, vars, lists)) ? 0 : 1;
            }
            return ParseComparison(t, ref i, vars, lists);
        }

        private static readonly string[] CompareOps = ["==", "!=", "<=", ">=", "<", ">"];

        private static double ParseComparison(List<Token> t, ref int i,
            Dictionary<string, double> vars, Dictionary<string, ListVar>? lists)
        {
            double left = ParseAddSub(t, ref i, vars, lists);
            while (i < t.Count && t[i].Type == TokType.Op && Array.IndexOf(CompareOps, t[i].Value) >= 0)
            {
                string op = t[i++].Value;
                double right = ParseAddSub(t, ref i, vars, lists);
                // Left-associative, as in C and JavaScript: "1 < 2 < 3" is "(1 < 2) < 3",
                // which is 1 < 3. Chained comparisons do not mean what they do in maths.
                left = Compare(left, op, right) ? 1 : 0;
            }
            return left;
        }

        private static double ParseAddSub(List<Token> t, ref int i,
            Dictionary<string, double> vars, Dictionary<string, ListVar>? lists)
        {
            double left = ParseMulDiv(t, ref i, vars, lists);
            while (i < t.Count && t[i].Type == TokType.Op && (t[i].Value == "+" || t[i].Value == "-"))
            {
                string op = t[i++].Value;
                double right = ParseMulDiv(t, ref i, vars, lists);
                left = op == "+" ? left + right : left - right;
            }
            return left;
        }

        private static double ParseMulDiv(List<Token> t, ref int i,
            Dictionary<string, double> vars, Dictionary<string, ListVar>? lists)
        {
            double left = ParseUnary(t, ref i, vars, lists);
            while (i < t.Count && t[i].Type == TokType.Op && (t[i].Value == "*" || t[i].Value == "/"))
            {
                string op = t[i++].Value;
                double right = ParseUnary(t, ref i, vars, lists);
                left = op == "*" ? left * right : (right != 0 ? left / right : 0);
            }
            return left;
        }

        private static double ParseUnary(List<Token> t, ref int i,
            Dictionary<string, double> vars, Dictionary<string, ListVar>? lists)
        {
            if (i < t.Count && t[i].Type == TokType.Op && t[i].Value == "-")
            {
                i++;
                return -ParsePrimary(t, ref i, vars, lists);
            }
            return ParsePrimary(t, ref i, vars, lists);
        }

        private static double ParsePrimary(List<Token> t, ref int i,
            Dictionary<string, double> vars, Dictionary<string, ListVar>? lists)
        {
            if (i >= t.Count) return 0;

            var tok = t[i];

            if (tok.Type == TokType.Variable)
            {
                i++;
                string name = tok.Value;

                // .length / .count — element count of any list variable. Only consumed when
                // the name really is a list; otherwise fall through so dotted-name lookup
                // below can try (e.g. an IO key ending ".count").
                if (i + 1 < t.Count && t[i].Type == TokType.Dot && t[i + 1].Type == TokType.Word)
                {
                    string prop = t[i + 1].Value;
                    if ((prop.Equals("length", StringComparison.OrdinalIgnoreCase) ||
                         prop.Equals("count",  StringComparison.OrdinalIgnoreCase)) &&
                        lists != null && lists.TryGetValue(name, out var counted))
                    {
                        i += 2;
                        return counted.Count;
                    }
                }

                // Array indexing: $name[indexExpr], then an element-type-specific accessor.
                if (i < t.Count && t[i].Type == TokType.LBracket &&
                    lists != null && lists.TryGetValue(name, out var lv))
                {
                    i++; // consume '['
                    double idxVal = ParseAddSub(t, ref i, vars, lists);
                    if (i < t.Count && t[i].Type == TokType.RBracket) i++; // consume ']'
                    int idx = (int)Math.Round(idxVal);
                    var item = (idx >= 0 && idx < lv.Count) ? lv.Items[idx] : null;

                    // A number or boolean element *is* the value, so it stops here and
                    // deliberately consumes no accessor — "$nums[0].foo" stays "$nums[0]", as
                    // it was before the list types were unified. A boolean reaches expressions
                    // as the 0/1 it is stored as, so conditions need no special case.
                    if (lv.HasScalarElements)
                        return item?.Scalar ?? 0;

                    // .field  — a point's axis (.x) and a record's field (.coverage) are the
                    // same lookup once a point is stored as named doubles.
                    if (i + 1 < t.Count && t[i].Type == TokType.Dot && t[i + 1].Type == TokType.Word)
                    {
                        string field = t[i + 1].Value;
                        i += 2;
                        // A missing field is 0 rather than a throw: the field set depends on
                        // what produced the record, so the expression cannot be sure.
                        return item != null && item.TryGetValue(field, out double fv) ? fv : 0;
                    }

                    // [n] — positional, and only a point list has a defined axis order.
                    if (i < t.Count && t[i].Type == TokType.LBracket)
                    {
                        i++; // consume '['
                        double compIdx = ParseAddSub(t, ref i, vars, lists);
                        if (i < t.Count && t[i].Type == TokType.RBracket) i++; // consume ']'
                        if (lv.ElementType != ListElementType.Point) return 0;
                        string? axis = ListVar.AxisName((int)Math.Round(compIdx));
                        return axis != null && item != null && item.TryGetValue(axis, out double av) ? av : 0;
                    }

                    return 0; // point or record indexed without an accessor → 0
                }

                // Dotted-name lookup — IO variables are injected with dotted keys
                // (e.g. "stb.in1", "relay.1", "nano.Board.pin1") but the tokenizer splits
                // on '.', so re-join the dotted chain and try the longest candidate first.
                // Note: a purely numeric suffix like ".1" tokenizes as a single Number
                // token with a leading dot, not Dot + Number — handle both shapes.
                {
                    var parts    = new List<string> { name };
                    var consumed = new List<int> { 0 };   // tokens consumed beyond the name per part depth
                    int look = i;
                    while (look < t.Count)
                    {
                        if (look + 1 < t.Count && t[look].Type == TokType.Dot &&
                            (t[look + 1].Type == TokType.Word || t[look + 1].Type == TokType.Number))
                        {
                            parts.Add(t[look + 1].Value);
                            look += 2;
                        }
                        else if (t[look].Type == TokType.Number && t[look].Value.StartsWith('.'))
                        {
                            parts.Add(t[look].Value[1..]);
                            look += 1;
                        }
                        else break;
                        consumed.Add(look - i);
                    }
                    for (int n = parts.Count; n >= 1; n--)
                    {
                        string candidate = string.Join(".", parts.GetRange(0, n));
                        if (vars.TryGetValue(candidate, out double val))
                        {
                            i += consumed[n - 1]; // consume exactly the tokens we matched
                            return val;
                        }
                    }
                }

                // Known list variable referenced without an index — preserve the legacy 0
                // (it's a declared variable, just used without [i]/.length).
                if (lists != null && lists.ContainsKey(name))
                    return 0;

                // Truly unknown identifier — fail loudly. Silently coercing a typo'd
                // variable to 0 can turn a clearance offset into a collision.
                throw new UnknownVariableException(name);
            }

            if (tok.Type == TokType.Number)
            {
                i++;
                return double.TryParse(tok.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out double n) ? n : 0;
            }

            if (tok.Type == TokType.LParen)
            {
                i++;
                // Back to the top of the precedence chain, not just arithmetic — parentheses
                // are how you write "($a > 1) and ($b > 2)", so what is inside them has to be
                // able to hold a full expression.
                double val = ParseOr(t, ref i, vars, lists);
                if (i < t.Count && t[i].Type == TokType.RParen) i++;
                return val;
            }

            // Bare words: True → 1, False → 0 (case-insensitive)
            if (tok.Type == TokType.Word)
            {
                i++;
                if (tok.Value.Equals("true",  StringComparison.OrdinalIgnoreCase)) return 1;
                if (tok.Value.Equals("false", StringComparison.OrdinalIgnoreCase)) return 0;
                return 0;
            }

            return 0;
        }

        // ── Tokenizer ─────────────────────────────────────────────────────────

        private static List<Token> Tokenize(string expr)
        {
            var tokens = new List<Token>();
            int i = 0;

            while (i < expr.Length)
            {
                char c = expr[i];

                // Whitespace
                if (char.IsWhiteSpace(c)) { i++; continue; }

                // Variable: $identifier (no dot — dots are Dot tokens)
                if (c == '$')
                {
                    i++;
                    int start = i;
                    while (i < expr.Length && (char.IsLetterOrDigit(expr[i]) || expr[i] == '_')) i++;
                    tokens.Add(new Token(TokType.Variable, expr[start..i]));
                    continue;
                }

                // Bare word (component name after dot: x, y, z, rx, ry, rz), or one of the
                // word-spelled logic operators. Those become Op tokens here so the parser
                // has a single shape to match — "and" and "&&" are the same token after this.
                if (char.IsLetter(c) || c == '_')
                {
                    int start = i;
                    while (i < expr.Length && (char.IsLetterOrDigit(expr[i]) || expr[i] == '_')) i++;
                    string word = expr[start..i];
                    // Straight after a dot it is a field name, never an operator: record
                    // fields are named by whatever produced the record, so "$r[0].not" has
                    // to keep working even though "not" is spelled like the operator.
                    bool afterDot = tokens.Count > 0 && tokens[^1].Type == TokType.Dot;
                    tokens.Add(!afterDot && IsLogicWord(word)
                        ? new Token(TokType.Op,   word.ToLowerInvariant())
                        : new Token(TokType.Word, word));
                    continue;
                }

                // Number: digits and decimal point
                if (char.IsDigit(c) || (c == '.' && i + 1 < expr.Length && char.IsDigit(expr[i + 1])))
                {
                    int start = i;
                    while (i < expr.Length && (char.IsDigit(expr[i]) || expr[i] == '.')) i++;
                    tokens.Add(new Token(TokType.Number, expr[start..i]));
                    continue;
                }

                // Two-character operators, matched before the single-character ones below so
                // ">=" is never read as ">" followed by a skipped "=".
                if (i + 1 < expr.Length)
                {
                    string pair = expr.Substring(i, 2);
                    if (pair is "==" or "!=" or "<=" or ">=")
                    {
                        tokens.Add(new Token(TokType.Op, pair));
                        i += 2;
                        continue;
                    }
                    // The C-style spellings fold into the word forms, so the parser and any
                    // error message only ever deal with one name per operator.
                    if (pair == "&&") { tokens.Add(new Token(TokType.Op, "and")); i += 2; continue; }
                    if (pair == "||") { tokens.Add(new Token(TokType.Op, "or"));  i += 2; continue; }
                }

                // Operators
                if (c == '+' || c == '-' || c == '*' || c == '/' || c == '<' || c == '>')
                {
                    tokens.Add(new Token(TokType.Op, c.ToString()));
                    i++;
                    continue;
                }

                if (c == '!') { tokens.Add(new Token(TokType.Op, "not")); i++; continue; }

                // A lone '=' means '=='. Nothing in this language assigns, so there is no
                // other thing it could mean — and the alternative is silently dropping it,
                // which would turn "$a = 5" into the bare "$a" and quietly read as true.
                if (c == '=') { tokens.Add(new Token(TokType.Op, "==")); i++; continue; }

                if (c == '(') { tokens.Add(new Token(TokType.LParen,   "(")); i++; continue; }
                if (c == ')') { tokens.Add(new Token(TokType.RParen,   ")")); i++; continue; }
                if (c == '[') { tokens.Add(new Token(TokType.LBracket, "[")); i++; continue; }
                if (c == ']') { tokens.Add(new Token(TokType.RBracket, "]")); i++; continue; }
                if (c == '.') { tokens.Add(new Token(TokType.Dot,      ".")); i++; continue; }

                i++; // skip unrecognised characters
            }

            return tokens;
        }
    }
}
