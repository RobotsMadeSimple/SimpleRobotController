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
    ///   List index   : $listName[indexExpr]
    ///   Point count  : $pointsVar.length  or  $pointsVar.count
    ///   Point comp.  : $pointsVar[indexExpr].x   (x/y/z/rx/ry/rz)
    ///   Point comp.  : $pointsVar[indexExpr][0]  (0=x 1=y 2=z 3=rx 4=ry 5=rz)
    ///   Literals     : 3.14  -5  100
    ///   Operators    : +  -  *  /
    ///   Grouping     : (expr)
    ///   Precedence   : standard (* / before + -)
    /// </summary>
    internal static class ExpressionEvaluator
    {
        private enum TokType { Number, Variable, Op, LParen, RParen, LBracket, RBracket, Dot, Word }
        private readonly record struct Token(TokType Type, string Value);

        // ── Public entry point ────────────────────────────────────────────────

        public static double Evaluate(
            string expr,
            Dictionary<string, double> variables,
            Dictionary<string, List<double>>? listVariables = null,
            Dictionary<string, List<Vector6Val>>? pointVariables = null)
        {
            var tokens = Tokenize(expr.Trim());
            int idx = 0;
            return ParseAddSub(tokens, ref idx, variables, listVariables, pointVariables);
        }

        // ── Recursive descent ─────────────────────────────────────────────────

        private static double ParseAddSub(List<Token> t, ref int i,
            Dictionary<string, double> vars,
            Dictionary<string, List<double>>? listVars,
            Dictionary<string, List<Vector6Val>>? ptVars)
        {
            double left = ParseMulDiv(t, ref i, vars, listVars, ptVars);
            while (i < t.Count && t[i].Type == TokType.Op && (t[i].Value == "+" || t[i].Value == "-"))
            {
                string op = t[i++].Value;
                double right = ParseMulDiv(t, ref i, vars, listVars, ptVars);
                left = op == "+" ? left + right : left - right;
            }
            return left;
        }

        private static double ParseMulDiv(List<Token> t, ref int i,
            Dictionary<string, double> vars,
            Dictionary<string, List<double>>? listVars,
            Dictionary<string, List<Vector6Val>>? ptVars)
        {
            double left = ParseUnary(t, ref i, vars, listVars, ptVars);
            while (i < t.Count && t[i].Type == TokType.Op && (t[i].Value == "*" || t[i].Value == "/"))
            {
                string op = t[i++].Value;
                double right = ParseUnary(t, ref i, vars, listVars, ptVars);
                left = op == "*" ? left * right : (right != 0 ? left / right : 0);
            }
            return left;
        }

        private static double ParseUnary(List<Token> t, ref int i,
            Dictionary<string, double> vars,
            Dictionary<string, List<double>>? listVars,
            Dictionary<string, List<Vector6Val>>? ptVars)
        {
            if (i < t.Count && t[i].Type == TokType.Op && t[i].Value == "-")
            {
                i++;
                return -ParsePrimary(t, ref i, vars, listVars, ptVars);
            }
            return ParsePrimary(t, ref i, vars, listVars, ptVars);
        }

        private static double ParsePrimary(List<Token> t, ref int i,
            Dictionary<string, double> vars,
            Dictionary<string, List<double>>? listVars,
            Dictionary<string, List<Vector6Val>>? ptVars)
        {
            if (i >= t.Count) return 0;

            var tok = t[i];

            if (tok.Type == TokType.Variable)
            {
                i++;
                string name = tok.Value;

                // .length / .count — returns the number of elements in a list or points variable.
                // Only consumed when the name is actually a list/points var; otherwise fall
                // through so dotted-name lookup below can try (e.g. an IO key ending ".count").
                if (i + 1 < t.Count && t[i].Type == TokType.Dot && t[i + 1].Type == TokType.Word)
                {
                    string prop = t[i + 1].Value;
                    if (prop.Equals("length", StringComparison.OrdinalIgnoreCase) ||
                        prop.Equals("count",  StringComparison.OrdinalIgnoreCase))
                    {
                        if (listVars != null && listVars.TryGetValue(name, out var countList)) { i += 2; return countList.Count; }
                        if (ptVars   != null && ptVars  .TryGetValue(name, out var countPts))  { i += 2; return countPts.Count; }
                    }
                }

                // Array indexing: $name[indexExpr]
                if (i < t.Count && t[i].Type == TokType.LBracket)
                {
                    // Try scalar list first
                    if (listVars != null && listVars.TryGetValue(name, out var list))
                    {
                        i++; // consume '['
                        double idxVal = ParseAddSub(t, ref i, vars, listVars, ptVars);
                        if (i < t.Count && t[i].Type == TokType.RBracket) i++; // consume ']'
                        int idx = (int)Math.Round(idxVal);
                        return (idx >= 0 && idx < list.Count) ? list[idx] : 0;
                    }

                    // Try points variable
                    if (ptVars != null && ptVars.TryGetValue(name, out var ptList))
                    {
                        i++; // consume '['
                        double idxVal = ParseAddSub(t, ref i, vars, listVars, ptVars);
                        if (i < t.Count && t[i].Type == TokType.RBracket) i++; // consume ']'
                        int ptIdx = (int)Math.Round(idxVal);
                        var pt = (ptIdx >= 0 && ptIdx < ptList.Count) ? ptList[ptIdx] : null;

                        // .component  (e.g. .x .y .rx)
                        if (i + 1 < t.Count && t[i].Type == TokType.Dot && t[i + 1].Type == TokType.Word)
                        {
                            string comp = t[i + 1].Value;
                            i += 2;
                            return pt?.GetComponent(comp) ?? 0;
                        }

                        // [compIndex]  (e.g. [0]=x [1]=y)
                        if (i < t.Count && t[i].Type == TokType.LBracket)
                        {
                            i++; // consume '['
                            double compIdxVal = ParseAddSub(t, ref i, vars, listVars, ptVars);
                            if (i < t.Count && t[i].Type == TokType.RBracket) i++; // consume ']'
                            return pt?.GetComponent((int)Math.Round(compIdxVal)) ?? 0;
                        }

                        return 0; // point var without component → 0
                    }
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

                // Known list/points variable referenced without an index — preserve the
                // legacy 0 (it's a declared variable, just used without [i]/.length).
                if ((listVars != null && listVars.ContainsKey(name)) ||
                    (ptVars   != null && ptVars.ContainsKey(name)))
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
                double val = ParseAddSub(t, ref i, vars, listVars, ptVars);
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

                // Bare word (component name after dot: x, y, z, rx, ry, rz)
                if (char.IsLetter(c) || c == '_')
                {
                    int start = i;
                    while (i < expr.Length && (char.IsLetterOrDigit(expr[i]) || expr[i] == '_')) i++;
                    tokens.Add(new Token(TokType.Word, expr[start..i]));
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

                // Operators
                if (c == '+' || c == '-' || c == '*' || c == '/')
                {
                    tokens.Add(new Token(TokType.Op, c.ToString()));
                    i++;
                    continue;
                }

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
