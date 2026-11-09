using System.Reflection;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Language;

/// <summary>
/// Gives parsed steps the ids of the steps they came from (docs/program-text.md, "Ids").
/// Text carries no ids unless <c>@id</c> is written, but revisions, the monitor, validation,
/// breakpoints, labels and UI state all key on them, so each parsed step list is matched
/// against the base program's list at the same role and path:
/// <list type="number">
/// <item>key = <c>type | primaryRef</c> (point, target expression, grid/stack cell, routine,
/// variable, label, vision program, plugin step, background program — else "");</item>
/// <item>the longest common subsequence of keys gives anchors; between anchors the leftover old
/// and new steps of the same type pair up in order;</item>
/// <item>a matched step takes the old id, the literals its expressions shadow, its label's
/// labelId, and (by position) the ids of its condition items, else-if branches and switch
/// cases; then child lists recurse by role;</item>
/// <item>an id already on the step (<c>@id</c>, or JSON from <c>@with</c>) always wins; anything
/// unmatched gets a fresh GUID.</item>
/// </list>
/// Variables match by name.
/// </summary>
internal static class IdReconciler
{
    public static string NewId() => Guid.NewGuid().ToString();

    /// <param name="claimed">Ids written explicitly somewhere in the new program: never inherited
    /// by another step.</param>
    /// <param name="nodeOf">The text node a step was parsed from; null for a step that arrived as
    /// JSON inside an <c>@with</c> — such steps keep exactly what the JSON says.</param>
    public static void Reconcile(List<ProgramStep>? olds, List<ProgramStep> news, ISet<string> claimed,
                                 Func<ProgramStep, StepNode?> nodeOf)
    {
        olds ??= [];
        var explicitHere = news.Where(n => !string.IsNullOrEmpty(n.Id)).Select(n => n.Id).ToHashSet();
        string Key(ProgramStep s, bool isNew)
        {
            if (isNew && !string.IsNullOrEmpty(s.Id)) return $"{s.Type}|#id:{s.Id}";
            if (!isNew && explicitHere.Contains(s.Id)) return $"{s.Type}|#id:{s.Id}";
            return $"{s.Type}|{PrimaryRef(s)}";
        }
        var ok = olds.Select(o => Key(o, false)).ToArray();
        var nk = news.Select(n => Key(n, true)).ToArray();

        // LCS
        int a = ok.Length, b = nk.Length;
        var dp = new int[a + 1, b + 1];
        for (int i = a - 1; i >= 0; i--)
            for (int j = b - 1; j >= 0; j--)
                dp[i, j] = ok[i] == nk[j] ? dp[i + 1, j + 1] + 1 : Math.Max(dp[i + 1, j], dp[i, j + 1]);
        var anchors = new List<(int O, int N)>();
        {
            int i = 0, j = 0;
            while (i < a && j < b)
            {
                if (ok[i] == nk[j]) { anchors.Add((i, j)); i++; j++; }
                else if (dp[i + 1, j] >= dp[i, j + 1]) i++;
                else j++;
            }
        }
        var pairs = new List<(int O, int N)>();
        int po = 0, pn = 0;
        foreach (var anchor in anchors.Append((O: a, N: b)))
        {
            // Between anchors: same-type steps pair in order.
            var oldLeft = Enumerable.Range(po, anchor.O - po).ToList();
            for (int j = pn; j < anchor.N; j++)
            {
                if (!string.IsNullOrEmpty(news[j].Id)) continue;
                int k = oldLeft.FindIndex(i => olds[i].Type == news[j].Type && !explicitHere.Contains(olds[i].Id));
                if (k < 0) continue;
                pairs.Add((oldLeft[k], j));
                oldLeft.RemoveRange(0, k + 1);
            }
            if (anchor.O < a) pairs.Add(anchor);
            po = anchor.O + 1; pn = anchor.N + 1;
        }

        // A step moved past others (cut and paste): same key, still unmatched on both sides.
        var usedOld = pairs.Select(p => p.O).ToHashSet();
        var usedNew = pairs.Select(p => p.N).ToHashSet();
        for (int j = 0; j < b; j++)
        {
            if (usedNew.Contains(j) || nk[j].EndsWith('|')) continue;
            for (int i = 0; i < a; i++)
                if (!usedOld.Contains(i) && ok[i] == nk[j]) { pairs.Add((i, j)); usedOld.Add(i); usedNew.Add(j); break; }
        }

        var matched = new HashSet<int>();
        foreach (var (o, n) in pairs)
        {
            matched.Add(n);
            Inherit(olds[o], news[n], claimed, nodeOf);
        }
        for (int j = 0; j < news.Count; j++)
            if (!matched.Contains(j)) Fresh(news[j], claimed, nodeOf);
    }

    /// <summary>The reference that identifies a step among its siblings.</summary>
    public static string PrimaryRef(ProgramStep s)
    {
        if (!string.IsNullOrEmpty(s.PointId)) return "p:" + s.PointId;
        if (!string.IsNullOrEmpty(s.PointNameExpr)) return "e:" + s.PointNameExpr;
        if (s.GridPoint is { } g)
            return $"g:{g.GridId}[{g.RowIndex}{Ex(s, "gridRowIndex")},{g.ColIndex}{Ex(s, "gridColIndex")},{g.GridIndex}{Ex(s, "gridGridIndex")}]";
        if (s.StackPoint is { } st) return $"s:{st.StackId}[{st.Index}{Ex(s, "stackIndex")}]";
        if (!string.IsNullOrEmpty(s.RoutineId)) return "r:" + s.RoutineId;
        if (!string.IsNullOrEmpty(s.VariableName)) return "v:" + s.VariableName;
        if (!string.IsNullOrEmpty(s.LabelName)) return "l:" + s.LabelName;
        if (!string.IsNullOrEmpty(s.VisionProgramId)) return "vp:" + s.VisionProgramId;
        if (!string.IsNullOrEmpty(s.PluginId)) return $"pl:{s.PluginId}.{s.PluginStepId}";
        if (!string.IsNullOrEmpty(s.BackgroundProgramId)) return "b:" + s.BackgroundProgramId;
        return "";
    }

    private static string Ex(ProgramStep s, string key) =>
        s.Expressions != null && s.Expressions.TryGetValue(key, out var e) ? "=" + e : "";

    private static void Inherit(ProgramStep old, ProgramStep @new, ISet<string> claimed, Func<ProgramStep, StepNode?> nodeOf)
    {
        var node = nodeOf(@new);
        if (node == null) return;
        bool W(string key) => node.With?.ContainsKey(key) == true;
        if (string.IsNullOrEmpty(@new.Id))
            @new.Id = !string.IsNullOrEmpty(old.Id) && !claimed.Contains(old.Id) ? old.Id : NewId();

        InheritShadowed(old, @new);
        if (@new.Type == StepType.Label && @new.LabelId == null && !W("labelId")) @new.LabelId = old.Type == StepType.Label ? old.LabelId : NewId();

        Items(old.Condition, @new.Condition);
        Items(old.LoopWhileCondition, @new.LoopWhileCondition);
        Items(old.WaitCondition, @new.WaitCondition);
        Items(old.MoveStopCondition, @new.MoveStopCondition);
        if (@new.ElseIfBranches != null && !W("elseIfBranches"))
            for (int i = 0; i < @new.ElseIfBranches.Count; i++)
            {
                var nb = @new.ElseIfBranches[i];
                var ob = old.ElseIfBranches != null && i < old.ElseIfBranches.Count ? old.ElseIfBranches[i] : null;
                if (string.IsNullOrEmpty(nb.Id)) nb.Id = ob != null ? ob.Id : NewId();
                Items(ob?.Condition, nb.Condition);
            }
        if (@new.SwitchCases != null && !W("switchCases"))
            for (int i = 0; i < @new.SwitchCases.Count; i++)
            {
                var nc = @new.SwitchCases[i];
                var oc = old.SwitchCases != null && i < old.SwitchCases.Count ? old.SwitchCases[i] : null;
                if (string.IsNullOrEmpty(nc.Id)) nc.Id = oc != null ? oc.Id : NewId();
            }

        var oldLists = old.ChildLists().ToList();
        foreach (var child in @new.ChildLists())
        {
            var match = oldLists.FirstOrDefault(o => o.Role == child.Role && o.Index == child.Index);
            Reconcile(match.Steps, child.Steps, claimed, nodeOf);
        }
    }

    private static void Items(ConditionGroup? old, ConditionGroup? @new)
    {
        if (@new?.Items == null) return;
        for (int i = 0; i < @new.Items.Count; i++)
        {
            var it = @new.Items[i];
            if (!string.IsNullOrEmpty(it.Id)) continue;
            var oi = old?.Items != null && i < old.Items.Count ? old.Items[i] : null;
            it.Id = oi != null ? oi.Id : NewId();
        }
    }

    /// <summary>A step with nothing to match: fresh ids all the way down.</summary>
    public static void Fresh(ProgramStep s, ISet<string> claimed, Func<ProgramStep, StepNode?> nodeOf)
    {
        var node = nodeOf(s);
        if (node == null) return;
        bool W(string key) => node.With?.ContainsKey(key) == true;
        if (string.IsNullOrEmpty(s.Id)) s.Id = NewId();
        if (s.Type == StepType.Label && s.LabelId == null && !W("labelId")) s.LabelId = NewId();
        Items(null, s.Condition);
        Items(null, s.LoopWhileCondition);
        Items(null, s.WaitCondition);
        Items(null, s.MoveStopCondition);
        if (!W("elseIfBranches"))
            foreach (var b in s.ElseIfBranches ?? [])
            {
                if (string.IsNullOrEmpty(b.Id)) b.Id = NewId();
                Items(null, b.Condition);
            }
        if (!W("switchCases"))
            foreach (var c in s.SwitchCases ?? []) if (string.IsNullOrEmpty(c.Id)) c.Id = NewId();
        foreach (var child in s.ChildLists())
            foreach (var cs in child.Steps) if (cs != null) Fresh(cs, claimed, nodeOf);
    }

    // ── Shadowed literals ─────────────────────────────────────────────────

    private static readonly Dictionary<string, PropertyInfo> Props =
        typeof(ProgramStep).GetProperties()
            .Select(p => (p, n: p.GetCustomAttribute<JsonPropertyNameAttribute>()?.Name))
            .Where(x => x.n != null)
            .ToDictionary(x => x.n!, x => x.p);

    /// <summary>Copies, for every <c>expressions</c> key of the new step whose literal is empty,
    /// the old step's literal (Normalize rule 4).</summary>
    public static void InheritShadowed(ProgramStep old, ProgramStep @new)
    {
        if (@new.Expressions == null) return;
        foreach (var key in @new.Expressions.Keys)
        {
            switch (key)
            {
                case "gridRowIndex":  if (@new.GridPoint != null && @new.GridPoint.RowIndex == null) @new.GridPoint.RowIndex = old.GridPoint?.RowIndex; continue;
                case "gridColIndex":  if (@new.GridPoint != null && @new.GridPoint.ColIndex == null) @new.GridPoint.ColIndex = old.GridPoint?.ColIndex; continue;
                case "gridGridIndex": if (@new.GridPoint != null && @new.GridPoint.GridIndex == null) @new.GridPoint.GridIndex = old.GridPoint?.GridIndex; continue;
                case "stackIndex":    if (@new.StackPoint != null && @new.StackPoint.Index == null) @new.StackPoint.Index = old.StackPoint?.Index; continue;
            }
            if (!Props.TryGetValue(key, out var p) || !p.CanWrite) continue;
            var t = p.PropertyType;
            if (Nullable.GetUnderlyingType(t) == null) continue; // only nullable scalars hold an optional literal
            if (p.GetValue(@new) == null && p.GetValue(old) is { } v) p.SetValue(@new, v);
        }
    }

    // ── Variables ──────────────────────────────────────────────────────────

    public static void ReconcileVariables(List<ProgramVariable>? olds, List<ProgramVariable> news, ISet<string> explicitIds)
    {
        var used = new HashSet<ProgramVariable>();
        foreach (var v in news)
        {
            var o = (olds ?? []).FirstOrDefault(x => !used.Contains(x) && x.Name == v.Name)
                 ?? (olds ?? []).FirstOrDefault(x => !used.Contains(x) && string.Equals(x.Name, v.Name, StringComparison.OrdinalIgnoreCase));
            if (o != null)
            {
                used.Add(o);
                if (string.IsNullOrEmpty(v.Id)) v.Id = !string.IsNullOrEmpty(o.Id) && !explicitIds.Contains(o.Id) ? o.Id : NewId();
                if (v.ValueExpression != null && v.Value == 0) v.Value = o.Value;
            }
            else if (string.IsNullOrEmpty(v.Id)) v.Id = NewId();
        }
    }
}
