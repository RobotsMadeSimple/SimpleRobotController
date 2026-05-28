using System;
using System.Collections.Generic;
using System.Globalization;

namespace Controller.RobotControl
{
    /// <summary>
    /// Evaluates simple math expressions that may reference program variables.
    ///
    /// Supported syntax:
    ///   Variables    : $varName
    ///   List index   : $listName[indexExpr]  or  $listName[3]
    ///   Literals     : 3.14  -5  100
    ///   Operators    : +  -  *  /
    ///   Grouping     : (expr)
    ///   Precedence   : standard (* / before + -)
    ///
    /// Examples:
    ///   "$speed * 0.8"
    ///   "($pickHeight + $offset) / 2"
    ///   "$counter + 1"
    ///   "$positions[$index]"
    ///   "$offsets[2] + $base"
    /// </summary>
    internal static class ExpressionEvaluator
    {
        private enum TokType { Number, Variable, Op, LParen, RParen, LBracket, RBracket }
        private readonly record struct Token(TokType Type, string Value);

        // ── Public entry point ────────────────────────────────────────────────

        public static double Evaluate(string expr, Dictionary<string, double> variables,
            Dictionary<string, List<double>>? listVariables = null)
        {
            var tokens = Tokenize(expr.Trim());
            int idx = 0;
            return ParseAddSub(tokens, ref idx, variables, listVariables);
        }

        // ── Recursive descent ─────────────────────────────────────────────────

        private static double ParseAddSub(List<Token> t, ref int i, Dictionary<string, double> vars, Dictionary<string, List<double>>? listVars)
        {
            double left = ParseMulDiv(t, ref i, vars, listVars);
            while (i < t.Count && t[i].Type == TokType.Op && (t[i].Value == "+" || t[i].Value == "-"))
            {
                string op = t[i++].Value;
                double right = ParseMulDiv(t, ref i, vars, listVars);
                left = op == "+" ? left + right : left - right;
            }
            return left;
        }

        private static double ParseMulDiv(List<Token> t, ref int i, Dictionary<string, double> vars, Dictionary<string, List<double>>? listVars)
        {
            double left = ParseUnary(t, ref i, vars, listVars);
            while (i < t.Count && t[i].Type == TokType.Op && (t[i].Value == "*" || t[i].Value == "/"))
            {
                string op = t[i++].Value;
                double right = ParseUnary(t, ref i, vars, listVars);
                left = op == "*" ? left * right : (right != 0 ? left / right : 0);
            }
            return left;
        }

        private static double ParseUnary(List<Token> t, ref int i, Dictionary<string, double> vars, Dictionary<string, List<double>>? listVars)
        {
            if (i < t.Count && t[i].Type == TokType.Op && t[i].Value == "-")
            {
                i++;
                return -ParsePrimary(t, ref i, vars, listVars);
            }
            return ParsePrimary(t, ref i, vars, listVars);
        }

        private static double ParsePrimary(List<Token> t, ref int i, Dictionary<string, double> vars, Dictionary<string, List<double>>? listVars)
        {
            if (i >= t.Count) return 0;

            var tok = t[i];

            if (tok.Type == TokType.Variable)
            {
                i++;
                string name = tok.Value;
                // List indexing: $name[indexExpr]
                if (i < t.Count && t[i].Type == TokType.LBracket && listVars != null && listVars.TryGetValue(name, out var list))
                {
                    i++; // consume '['
                    double idxVal = ParseAddSub(t, ref i, vars, listVars);
                    if (i < t.Count && t[i].Type == TokType.RBracket) i++; // consume ']'
                    int idx = (int)Math.Round(idxVal);
                    return (idx >= 0 && idx < list.Count) ? list[idx] : 0;
                }
                return vars.TryGetValue(name, out double v) ? v : 0;
            }

            if (tok.Type == TokType.Number)
            {
                i++;
                return double.TryParse(tok.Value, NumberStyles.Any, CultureInfo.InvariantCulture, out double n) ? n : 0;
            }

            if (tok.Type == TokType.LParen)
            {
                i++;
                double val = ParseAddSub(t, ref i, vars, listVars);
                if (i < t.Count && t[i].Type == TokType.RParen) i++;
                return val;
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

                // Variable: $identifier
                if (c == '$')
                {
                    i++;
                    int start = i;
                    while (i < expr.Length && (char.IsLetterOrDigit(expr[i]) || expr[i] == '_' || expr[i] == '.')) i++;
                    tokens.Add(new Token(TokType.Variable, expr[start..i]));
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

                i++; // skip unrecognised characters
            }

            return tokens;
        }
    }
}
