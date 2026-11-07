using System.Globalization;
using System.Text;

namespace Controller.RobotControl.Language;

// The text language's (SRS) lexical layer (docs/program-text.md): physical lines → logical lines
// (continuation while a bracket is open or after a trailing comma), comments split off, and a
// cursor with the scanning helpers the parser uses. Every character of a logical line keeps its
// absolute offset in the source, so problems point at the exact UTF-16 column.

internal enum SrsLineKind { Blank, Pragma, Comment, Section, Content }

/// <summary>One logical line.</summary>
internal sealed class SrsLine
{
    public SrsLineKind Kind;
    /// <summary>Content lines: the statement text (comment removed, continuation lines joined
    /// with one space). Comment / section lines: the comment text (one space after the marker dropped).</summary>
    public string Text = "";
    /// <summary>Absolute source offset of every <see cref="Text"/> index, plus one past the end.</summary>
    public int[] Map = [];
    /// <summary>First and last physical line, 1-based.</summary>
    public int Line, EndLine;
    public int Indent;
    /// <summary>Content lines: the text of a trailing <c># comment</c> (null when none).</summary>
    public string? TrailingComment;
    /// <summary>A lexical problem on this line (unterminated string, unclosed bracket).</summary>
    public string? Error;
    public int ErrorOffset;

    public int Abs(int index) => Map.Length == 0 ? 0 : Map[Math.Clamp(index, 0, Map.Length - 1)];
}

/// <summary>The source text with line starts, for offset → (line, col).</summary>
internal sealed class SrsSource
{
    public readonly string Text;
    private readonly List<int> _lineStarts = [0];

    public SrsSource(string text)
    {
        Text = text ?? "";
        for (int i = 0; i < Text.Length; i++)
            if (Text[i] == '\n') _lineStarts.Add(i + 1);
    }

    public int LineCount => _lineStarts.Count;

    /// <summary>1-based line and UTF-16 column of an absolute offset.</summary>
    public (int Line, int Col) Pos(int offset)
    {
        offset = Math.Clamp(offset, 0, Text.Length);
        int lo = 0, hi = _lineStarts.Count - 1;
        while (lo < hi)
        {
            int mid = (lo + hi + 1) / 2;
            if (_lineStarts[mid] <= offset) lo = mid; else hi = mid - 1;
        }
        return (lo + 1, offset - _lineStarts[lo] + 1);
    }

    public SrsSpan Span(int start, int end)
    {
        if (end < start) end = start;
        var (l, c) = Pos(start);
        var (el, ec) = Pos(end);
        if (el == l && ec <= c) ec = c + 1;
        return new SrsSpan(l, c, el, ec);
    }

    /// <summary>Physical line <paramref name="line"/> (1-based) without its line break.</summary>
    public (int Start, int End) LineBounds(int line)
    {
        int s = _lineStarts[line - 1];
        int e = line < _lineStarts.Count ? _lineStarts[line] - 1 : Text.Length;
        if (e > s && Text[e - 1] == '\r') e--;
        return (s, e);
    }

    /// <summary>Splits the source into logical lines.</summary>
    public List<SrsLine> Lines()
    {
        var result = new List<SrsLine>();
        int n = _lineStarts.Count;
        int ln = 1;
        while (ln <= n)
        {
            var (s, e) = LineBounds(ln);
            int i = s;
            while (i < e && (Text[i] == ' ' || Text[i] == '\t')) i++;
            int indent = i - s;
            if (i >= e)
            {
                result.Add(new SrsLine { Kind = SrsLineKind.Blank, Line = ln, EndLine = ln, Map = [i] });
                ln++; continue;
            }
            if (Text[i] == '#')
            {
                SrsLineKind kind; int ts;
                if (ln == 1 && Text.AsSpan(i, e - i).StartsWith("#!srs")) { kind = SrsLineKind.Pragma; ts = i + 2; }
                else if (i + 1 < e && Text[i + 1] == '#') { kind = SrsLineKind.Section; ts = i + 2; }
                else { kind = SrsLineKind.Comment; ts = i + 1; }
                if (kind != SrsLineKind.Pragma && ts < e && Text[ts] == ' ') ts++;
                var line = new SrsLine { Kind = kind, Line = ln, EndLine = ln, Indent = indent };
                line.Text = Text[ts..e];
                line.Map = Enumerable.Range(ts, e - ts + 1).ToArray();
                result.Add(line);
                ln++; continue;
            }
            result.Add(ReadContent(ref ln, i, indent));
        }
        return result;
    }

    private SrsLine ReadContent(ref int ln, int start, int indent)
    {
        var line = new SrsLine { Kind = SrsLineKind.Content, Line = ln, Indent = indent };
        var sb = new StringBuilder();
        var map = new List<int>();
        var openers = new Stack<(char C, int At)>();
        int n = _lineStarts.Count;
        int segStart = start;
        while (true)
        {
            var (_, e) = LineBounds(ln);
            bool inStr = false; int strAt = 0;
            int contentEnd = e;
            for (int i = segStart; i < e; i++)
            {
                char c = Text[i];
                if (inStr)
                {
                    if (c == '\\' && i + 1 < e) { i++; continue; }
                    if (c == '"') inStr = false;
                    continue;
                }
                if (c == '"') { inStr = true; strAt = i; continue; }
                if (c == '#') { contentEnd = i; break; }
                if (c is '(' or '[' or '{') openers.Push((c, i));
                else if (c is ')' or ']' or '}')
                {
                    char want = c == ')' ? '(' : c == ']' ? '[' : '{';
                    if (openers.Count > 0 && openers.Peek().C == want) openers.Pop();
                }
            }
            if (inStr && line.Error == null) { line.Error = "Unterminated string — close it with \""; line.ErrorOffset = strAt; }
            if (contentEnd < e && line.TrailingComment == null)
            {
                int cs = contentEnd + 1;
                if (cs < e && Text[cs] == ' ') cs++;
                line.TrailingComment = Text[cs..e];
            }
            int ce = contentEnd;
            while (ce > segStart && (Text[ce - 1] == ' ' || Text[ce - 1] == '\t')) ce--;
            if (sb.Length > 0) { sb.Append(' '); map.Add(segStart); }
            for (int i = segStart; i < ce; i++) { sb.Append(Text[i]); map.Add(i); }
            line.EndLine = ln;
            int endAbs = ce;

            bool trailingComma = sb.Length > 0 && sb[^1] == ',';
            if (inStr || (openers.Count == 0 && !trailingComma)) { map.Add(endAbs); break; }

            // Continue on the next non-comment line; a blank line or the end of the file stops it.
            int next = ln + 1;
            int nextStart = -1;
            while (next <= n)
            {
                var (ns, ne) = LineBounds(next);
                int j = ns;
                while (j < ne && (Text[j] == ' ' || Text[j] == '\t')) j++;
                if (j >= ne) break;                        // blank
                if (Text[j] == '#') { next++; continue; }  // comment inside a continuation: skipped
                nextStart = j;
                break;
            }
            if (nextStart < 0)
            {
                map.Add(endAbs);
                if (openers.Count > 0 && line.Error == null)
                {
                    var (c, at) = openers.Peek();
                    line.Error = $"'{c}' is never closed";
                    line.ErrorOffset = at;
                }
                break;
            }
            ln = next;
            segStart = nextStart;
        }
        line.Text = sb.ToString();
        line.Map = map.ToArray();
        ln++;
        return line;
    }
}

/// <summary>A 1-based source span; <see cref="EndCol"/> exclusive.</summary>
internal readonly record struct SrsSpan(int Line, int Col, int EndLine, int EndCol);

/// <summary>Scanning helpers over one logical line's text.</summary>
internal sealed class SrsCursor
{
    public readonly SrsLine Line;
    public readonly string S;
    public int P;

    public SrsCursor(SrsLine line, int start = 0) { Line = line; S = line.Text; P = start; }

    public void Ws() { while (P < S.Length && (S[P] == ' ' || S[P] == '\t')) P++; }
    public bool AtEnd { get { Ws(); return P >= S.Length; } }
    public char Peek(int k = 0) => P + k < S.Length ? S[P + k] : '\0';
    public string Rest => P < S.Length ? S[P..] : "";

    public bool TryChar(char c)
    {
        Ws();
        if (Peek() != c) return false;
        P++;
        return true;
    }

    public static bool IsIdentStart(char c) => char.IsAsciiLetter(c) || c == '_';
    public static bool IsIdentChar(char c) => char.IsAsciiLetterOrDigit(c) || c == '_';

    public static bool IsIdent(string? s) =>
        !string.IsNullOrEmpty(s) && IsIdentStart(s[0]) && s.All(IsIdentChar);

    /// <summary>The identifier at the cursor (after whitespace), without consuming it.</summary>
    public string? PeekIdent()
    {
        Ws();
        if (!IsIdentStart(Peek())) return null;
        int e = P;
        while (e < S.Length && IsIdentChar(S[e])) e++;
        return S[P..e];
    }

    public string? TryIdent()
    {
        var id = PeekIdent();
        if (id != null) P += id.Length;
        return id;
    }

    /// <summary>Consumes <paramref name="word"/> (case-insensitive) when it is the next whole word.</summary>
    public bool TryWord(string word)
    {
        var id = PeekIdent();
        if (id == null || !id.Equals(word, StringComparison.OrdinalIgnoreCase)) return false;
        P += id.Length;
        return true;
    }

    /// <summary>True when the cursor is at <c>ident =</c> (not <c>==</c>): a named argument.</summary>
    public bool AtNamedArg()
    {
        int save = P;
        try
        {
            if (TryIdent() == null) return false;
            Ws();
            return Peek() == '=' && Peek(1) != '=';
        }
        finally { P = save; }
    }

    /// <summary>Reads a <c>"…"</c> string with the escapes <c>\" \\ \n \t \r</c> (any other
    /// backslash is kept as written). Returns null when the cursor is not at a string.</summary>
    public string? TryString(out int start)
    {
        Ws();
        start = P;
        if (Peek() != '"') return null;
        var sb = new StringBuilder();
        int i = P + 1;
        while (i < S.Length)
        {
            char c = S[i];
            if (c == '\\' && i + 1 < S.Length)
            {
                char d = S[i + 1];
                switch (d)
                {
                    case '"': sb.Append('"'); break;
                    case '\\': sb.Append('\\'); break;
                    case 'n': sb.Append('\n'); break;
                    case 't': sb.Append('\t'); break;
                    case 'r': sb.Append('\r'); break;
                    default: sb.Append(c).Append(d); break;
                }
                i += 2;
                continue;
            }
            if (c == '"') { P = i + 1; return sb.ToString(); }
            sb.Append(c);
            i++;
        }
        return null; // unterminated: the lexer already reported it
    }

    /// <summary>
    /// Scans an expression-like run from the cursor to the first top-level stop: a <c>,</c>
    /// (when <paramref name="stopComma"/>), <c>-&gt;</c> (when <paramref name="stopArrow"/>), a
    /// closing bracket that was not opened in the run, or the end of the line. Strings and
    /// brackets are skipped as units. Returns the trimmed text and its start index.
    /// </summary>
    public (string Text, int Start) ScanExpr(bool stopComma = true, bool stopArrow = false)
    {
        Ws();
        int start = P;
        int depth = 0;
        int i = P;
        while (i < S.Length)
        {
            char c = S[i];
            if (c == '"')
            {
                i++;
                while (i < S.Length && S[i] != '"') { if (S[i] == '\\' && i + 1 < S.Length) i++; i++; }
                i++;
                continue;
            }
            if (c is '(' or '[' or '{') { depth++; i++; continue; }
            if (c is ')' or ']' or '}')
            {
                if (depth == 0) break;
                depth--; i++; continue;
            }
            if (depth == 0)
            {
                if (stopComma && c == ',') break;
                if (stopArrow && c == '-' && i + 1 < S.Length && S[i + 1] == '>') break;
            }
            i++;
        }
        int end = Math.Min(i, S.Length);
        P = end;
        int e = end;
        while (e > start && (S[e - 1] == ' ' || S[e - 1] == '\t')) e--;
        return (S[start..e], start);
    }

    /// <summary>
    /// One positional token: a string, a <c>$var</c> reference with <c>[…]</c> / <c>.field</c>
    /// parts, a parenthesised group, a number with an optional unit suffix, or a word.
    /// Returns the raw text (strings unescaped through <paramref name="isString"/>).
    /// </summary>
    public string? TryToken(out int start, out bool isString)
    {
        Ws();
        start = P;
        isString = false;
        if (P >= S.Length) return null;
        char c = S[P];
        if (c == '"')
        {
            isString = true;
            return TryString(out start);
        }
        if (c == '$')
        {
            int i = P + 1;
            while (i < S.Length && IsIdentChar(S[i])) i++;
            while (i < S.Length)
            {
                if (S[i] == '.' && i + 1 < S.Length && IsIdentChar(S[i + 1]))
                {
                    i++;
                    while (i < S.Length && IsIdentChar(S[i])) i++;
                    continue;
                }
                if (S[i] == '[')
                {
                    int close = MatchClose(i);
                    if (close < 0) { i = S.Length; break; }
                    i = close + 1;
                    continue;
                }
                break;
            }
            P = i;
            return S[start..i];
        }
        if (c == '(')
        {
            int close = MatchClose(P);
            P = close < 0 ? S.Length : close + 1;
            return S[start..P];
        }
        if (char.IsAsciiDigit(c) || ((c == '+' || c == '-' || c == '.') && P + 1 < S.Length && (char.IsAsciiDigit(S[P + 1]) || S[P + 1] == '.')))
        {
            int i = P + 1;
            while (i < S.Length && (char.IsAsciiLetterOrDigit(S[i]) || S[i] == '.' ||
                   ((S[i] == '+' || S[i] == '-') && (S[i - 1] == 'e' || S[i - 1] == 'E')))) i++;
            P = i;
            return S[start..i];
        }
        if (IsIdentStart(c)) return TryIdent();
        return null;
    }

    /// <summary>Index of the bracket closing the one at <paramref name="open"/>, or -1.</summary>
    public int MatchClose(int open)
    {
        int depth = 0;
        for (int i = open; i < S.Length; i++)
        {
            char c = S[i];
            if (c == '"')
            {
                i++;
                while (i < S.Length && S[i] != '"') { if (S[i] == '\\' && i + 1 < S.Length) i++; i++; }
                continue;
            }
            if (c is '(' or '[' or '{') depth++;
            else if (c is ')' or ']' or '}') { depth--; if (depth == 0) return i; }
        }
        return -1;
    }
}

/// <summary>Text helpers shared by the printer and the parser.</summary>
internal static class SrsText
{
    public static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    /// <summary>Words that are never printed bare as a name (they mean something in a value slot).</summary>
    private static readonly HashSet<string> Reserved = new(StringComparer.OrdinalIgnoreCase)
    {
        "on", "off", "true", "false", "here", "grid", "stack", "point", "and", "or", "not", "forever",
        "as", "in", "out", "to", "toggle", "list", "image", "send", "receive", "end", "_",
    };

    /// <summary>A name as text: bare when it is a plain identifier, otherwise quoted.</summary>
    public static string Name(string name) =>
        SrsCursor.IsIdent(name) && !Reserved.Contains(name) ? name : Quote(name);

    /// <summary>True when <paramref name="s"/> can be written as a string literal (no control
    /// characters other than \n \t \r).</summary>
    public static bool Quotable(string? s) => s != null && s.All(c => c >= ' ' || c is '\n' or '\t' or '\r');

    public static string Quote(string s)
    {
        var sb = new StringBuilder(s.Length + 2).Append('"');
        foreach (char c in s)
        {
            switch (c)
            {
                case '"': sb.Append("\\\""); break;
                case '\\': sb.Append("\\\\"); break;
                case '\n': sb.Append("\\n"); break;
                case '\t': sb.Append("\\t"); break;
                case '\r': sb.Append("\\r"); break;
                default: sb.Append(c); break;
            }
        }
        return sb.Append('"').ToString();
    }

    public static string Num(double d) => d.ToString("R", Inv);

    /// <summary>A millisecond value with its canonical suffix: 1500ms, 2s.</summary>
    public static string Ms(double v) =>
        v != 0 && v % 1000 == 0 && Math.Abs(v) < 1e15 ? Num(v / 1000) + "s" : Num(v) + "ms";

    /// <summary>A plain numeric literal (optionally signed); with <paramref name="ms"/>, also
    /// <c>500ms</c> / <c>1.5s</c> (converted to milliseconds).</summary>
    public static bool TryLiteral(string text, bool ms, out double value)
    {
        value = 0;
        if (string.IsNullOrEmpty(text)) return false;
        string t = text;
        double scale = 1;
        if (ms)
        {
            if (t.EndsWith("ms", StringComparison.OrdinalIgnoreCase)) t = t[..^2];
            else if (t.EndsWith('s') || t.EndsWith('S')) { t = t[..^1]; scale = 1000; }
        }
        if (!IsNumberText(t)) return false;
        if (!double.TryParse(t, NumberStyles.Float, Inv, out value)) return false;
        value *= scale;
        return double.IsFinite(value);
    }

    private static bool IsNumberText(string t)
    {
        int i = 0;
        if (i < t.Length && (t[i] == '+' || t[i] == '-')) i++;
        int digits = 0;
        while (i < t.Length && char.IsAsciiDigit(t[i])) { i++; digits++; }
        if (i < t.Length && t[i] == '.')
        {
            i++;
            while (i < t.Length && char.IsAsciiDigit(t[i])) { i++; digits++; }
        }
        if (digits == 0) return false;
        if (i < t.Length && (t[i] == 'e' || t[i] == 'E'))
        {
            i++;
            if (i < t.Length && (t[i] == '+' || t[i] == '-')) i++;
            int ed = 0;
            while (i < t.Length && char.IsAsciiDigit(t[i])) { i++; ed++; }
            if (ed == 0) return false;
        }
        return i == t.Length;
    }

    public static bool IsBoolWord(string t, out bool value)
    {
        value = false;
        switch (t.ToLowerInvariant())
        {
            case "on": case "true": value = true; return true;
            case "off": case "false": return true;
            default: return false;
        }
    }

    /// <summary>True when the whole text is one parenthesised group: <c>(…)</c>.</summary>
    public static bool IsWrapped(string t)
    {
        if (t.Length < 2 || t[0] != '(' || t[^1] != ')') return false;
        var c = new SrsCursor(new SrsLine { Text = t, Map = new int[t.Length + 1] });
        return c.MatchClose(0) == t.Length - 1;
    }

    /// <summary>A <c>$name</c> reference token with optional <c>[…]</c> / <c>.field</c> parts.</summary>
    public static bool IsVarRefToken(string t)
    {
        if (t.Length < 2 || t[0] != '$') return false;
        var c = new SrsCursor(new SrsLine { Text = t, Map = new int[t.Length + 1] });
        var tok = c.TryToken(out _, out bool isStr);
        return !isStr && tok == t && c.P == t.Length && SrsCursor.IsIdentStart(t[1]);
    }

    /// <summary>
    /// Whether an expression text can be written raw on one line and read back unchanged:
    /// non-empty, trimmed, single-line, brackets balanced, no <c>#</c> or backslash-escape
    /// outside its strings, and (for a comma-terminated slot) no top-level comma or <c>-&gt;</c>.
    /// </summary>
    public static bool RawSafe(string? e, bool stopComma = true, bool stopArrow = false)
    {
        if (string.IsNullOrEmpty(e) || e.Trim() != e) return false;
        if (e.Contains('\n') || e.Contains('\r') || e.Contains('\t')) return false;
        int depth = 0;
        for (int i = 0; i < e.Length; i++)
        {
            char c = e[i];
            if (c == '"')
            {
                i++;
                while (i < e.Length && e[i] != '"') { if (e[i] == '\\') return false; i++; }
                if (i >= e.Length) return false;
                continue;
            }
            if (c == '\\' || c == '#') return false;
            if (c is '(' or '[' or '{') depth++;
            else if (c is ')' or ']' or '}') { depth--; if (depth < 0) return false; }
            else if (depth == 0)
            {
                if (stopComma && c == ',') return false;
                if (stopArrow && c == '-' && i + 1 < e.Length && e[i + 1] == '>') return false;
            }
        }
        return depth == 0;
    }

    /// <summary>Edit distance, for "did you mean".</summary>
    public static int Distance(string a, string b)
    {
        a = a.ToLowerInvariant(); b = b.ToLowerInvariant();
        var d = new int[a.Length + 1, b.Length + 1];
        for (int i = 0; i <= a.Length; i++) d[i, 0] = i;
        for (int j = 0; j <= b.Length; j++) d[0, j] = j;
        for (int i = 1; i <= a.Length; i++)
            for (int j = 1; j <= b.Length; j++)
                d[i, j] = Math.Min(Math.Min(d[i - 1, j] + 1, d[i, j - 1] + 1), d[i - 1, j - 1] + (a[i - 1] == b[j - 1] ? 0 : 1));
        return d[a.Length, b.Length];
    }

    public static string? DidYouMean(string word, IEnumerable<string> candidates)
    {
        string? best = null; int bestD = int.MaxValue;
        foreach (var c in candidates)
        {
            int dd = Distance(word, c);
            if (dd < bestD) { bestD = dd; best = c; }
        }
        return best != null && bestD <= Math.Max(2, word.Length / 3) ? best : null;
    }
}
