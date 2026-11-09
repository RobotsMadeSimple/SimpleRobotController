using System.Globalization;
using System.Text.RegularExpressions;
using Controller.RobotControl.Programs;

namespace Controller.RobotControl.Validation
{
    // Package B (docs/program-language.md §8): checks for the flow blocks — LoopControl
    // (loopControlOutsideLoop), Switch (duplicateCaseValue, emptyBranch), Try (emptyBranch, catch
    // codes, error variables), RaiseError, EndProgram and routine parameters (unknownParam,
    // paramMissing, paramOutputType). Nested bodies (switchCases[i].steps, switchDefault,
    // trySteps, catchSteps) are walked by ValidateStep through StepTree; these hooks only add the
    // per-step checks.
    internal static partial class ProgramValidator
    {
        private static readonly Regex Identifier = new(@"^[A-Za-z_][A-Za-z0-9_]*$", RegexOptions.CultureInvariant);

        private sealed partial class Walker
        {
            partial void CheckFlowExt(ProgramStep s, string sp, At at, List<List<ProgramStep>> chain, Body body)
            {
                switch (s.Type)
                {
                    case StepType.LoopControl:
                        if (s.LoopControl is not (null or "break" or "continue"))
                            Add(at, ValidationCodes.MissingField, "Action must be break or continue", "loopControl");
                        if (!InsideLoop(chain))
                            Add(at, ValidationCodes.LoopControlOutsideLoop,
                                (s.LoopControl == "continue" ? "Next iteration" : "Break loop") +
                                " is not inside a loop" + (_callStack.Count > 1 ? " in this routine" : ""),
                                "loopControl");
                        break;

                    case StepType.EndProgram:
                        CheckTemplate(s.StatusMessage, at, "statusMessage", ValidationSeverity.Warning);
                        break;

                    case StepType.RaiseError:
                        if (string.IsNullOrWhiteSpace(s.ErrorMessage))
                            Add(at, ValidationCodes.MissingField, "No error message is set", "errorMessage");
                        else CheckTemplate(s.ErrorMessage, at, "errorMessage", ValidationSeverity.Warning);
                        if (!string.IsNullOrWhiteSpace(s.ErrorCode))
                        {
                            var code = s.ErrorCode.Trim();
                            if (!Identifier.IsMatch(code))
                                Add(at, ValidationCodes.MissingField,
                                    $"Error code '{code}' must be an identifier (letters, digits, _)", "errorCode");
                            else if (!ErrorCodes.IsCatchable(code))
                                Add(at, ValidationCodes.MissingField,
                                    $"'{code}' is a fatal error code: no Try can catch it", "errorCode", ValidationSeverity.Warning);
                        }
                        break;

                    case StepType.Switch:
                        CheckSwitch(s, at);
                        break;

                    case StepType.Try:
                        CheckTry(s, at);
                        break;

                    case StepType.CallRoutine:
                        CheckRoutineParams(s, at);
                        break;
                }
            }

            /// <summary>
            /// Whether a step list on <paramref name="chain"/> (root first, the step's own list
            /// last) is a loop body. A routine body starts a fresh chain, so a loop in the caller
            /// does not count — Break / Next never cross a routine boundary.
            /// </summary>
            private static bool InsideLoop(List<List<ProgramStep>> chain)
            {
                for (int k = 1; k < chain.Count; k++)
                {
                    var list = chain[k];
                    if (chain[k - 1].Any(p => p?.Type == StepType.Loop && ReferenceEquals(p.LoopSteps, list)))
                        return true;
                }
                return false;
            }

            private void CheckSwitch(ProgramStep s, At at)
            {
                if (string.IsNullOrWhiteSpace(s.SwitchExpr))
                    Add(at, ValidationCodes.MissingField, "No value to switch on", "switchExpr");
                else CheckExpr(s.SwitchExpr, at, "switchExpr");

                var cases = s.SwitchCases ?? [];
                // value (normalised literal) → where it first appears
                var seen = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                for (int i = 0; i < cases.Count; i++)
                {
                    var c = cases[i];
                    if (c == null) continue;
                    var values = c.Values ?? [];
                    if (values.All(string.IsNullOrWhiteSpace))
                    {
                        Add(at, ValidationCodes.MissingField, $"Case {i + 1} has no value, so it never runs",
                            $"switchCases[{i}].values", ValidationSeverity.Warning);
                        continue;
                    }
                    for (int j = 0; j < values.Count; j++)
                    {
                        var v = values[j];
                        if (string.IsNullOrWhiteSpace(v)) continue;
                        string field = $"switchCases[{i}].values[{j}]";
                        CheckExpr(v, at, field);
                        var key = double.TryParse(v.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out var d)
                            ? d.ToString("R", CultureInfo.InvariantCulture)
                            : Regex.Replace(v, @"\s+", "");
                        if (seen.TryGetValue(key, out var first))
                            Add(at, ValidationCodes.DuplicateCaseValue,
                                $"Value {v.Trim()} is already matched by {first}, so this one never runs", field,
                                ValidationSeverity.Warning);
                        else seen[key] = $"case {i + 1}";
                    }
                }

                bool anySteps = cases.Any(c => c?.Steps is { Count: > 0 }) || s.SwitchDefault is { Count: > 0 };
                if (!anySteps)
                    Add(at, ValidationCodes.EmptyBranch, "Switch has no steps in any case", "switchCases");
            }

            private void CheckTry(ProgramStep s, At at)
            {
                if (s.TrySteps is not { Count: > 0 })
                    Add(at, ValidationCodes.EmptyBranch, "Try has no steps to try", "trySteps");
                if (s.TryRetries is < 0)
                    Add(at, ValidationCodes.MissingField, "Retries cannot be negative", "tryRetries");
                if (s.TryRetryDelayMs is < 0)
                    Add(at, ValidationCodes.MissingField, "Retry delay cannot be negative", "tryRetryDelayMs");

                var codes = s.CatchCodes ?? [];
                for (int i = 0; i < codes.Count; i++)
                {
                    var c = codes[i]?.Trim();
                    if (string.IsNullOrEmpty(c)) continue;
                    if (!Identifier.IsMatch(c))
                        Add(at, ValidationCodes.MissingField, $"Error code '{c}' must be an identifier", $"catchCodes[{i}]");
                    else if (!ErrorCodes.IsCatchable(c))
                        Add(at, ValidationCodes.MissingField,
                            $"'{c}' is a fatal error and is never caught", $"catchCodes[{i}]", ValidationSeverity.Warning);
                }

                CheckTextTarget(s.ErrorVariableName, at, "errorVariableName");
                CheckTextTarget(s.ErrorCodeVariableName, at, "errorCodeVariableName");
            }

            /// <summary>A variable the step writes text into: writable, and not declared as a
            /// non-text kind (it would then hold two values under one name).</summary>
            private void CheckTextTarget(string? name, At at, string field)
            {
                if (string.IsNullOrWhiteSpace(name)) return;
                if (!CheckTarget(name, at, field, requireDeclared: false)) return;
                var n = name.Trim().TrimStart('$');
                if (_declared.TryGetValue(n, out var sym) && sym.Kind != SymKind.String)
                    Add(at, ValidationCodes.ParamOutputType,
                        $"'${n}' is {Describe(sym)} — the error text needs a text variable", field);
            }

            /// <summary>CallRoutine arguments and outputs against the routine's declared parameters.</summary>
            private void CheckRoutineParams(ProgramStep s, At at)
            {
                bool any = s.RoutineArgs is { Count: > 0 } || s.RoutineOutputs is { Count: > 0 };
                var routine = FindProgram(s.RoutineId, s.RoutineName);
                if (routine == null) return; // unknownRoutine (or offline) is reported by ValidateCall

                var parameters = (routine.Variables ?? [])
                    .Where(v => v != null && !string.IsNullOrWhiteSpace(v.Name) &&
                                v.ParamMode is ParamModes.In or ParamModes.Out or ParamModes.InOut)
                    .ToDictionary(v => v.Name.Trim(), StringComparer.OrdinalIgnoreCase);
                if (parameters.Count == 0 && !any) return;

                // Arguments: in / inOut parameters only.
                var given = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
                foreach (var (key, expr) in s.RoutineArgs ?? [])
                {
                    var k = key?.Trim() ?? "";
                    string field = $"routineArgs.{k}";
                    if (!parameters.TryGetValue(k, out var p))
                    {
                        Add(at, ValidationCodes.UnknownParam, $"Routine '{routine.Name}' has no parameter '{k}'", field);
                        continue;
                    }
                    if (p.ParamMode == ParamModes.Out)
                    {
                        Add(at, ValidationCodes.UnknownParam,
                            $"'{k}' is an out parameter of '{routine.Name}': it takes no argument (map it under outputs)",
                            field, ValidationSeverity.Warning);
                        continue;
                    }
                    if (string.IsNullOrWhiteSpace(expr)) continue;
                    given.Add(k);
                    if (p.IsString == true) CheckTemplate(expr, at, field, ValidationSeverity.Warning);
                    else if (p.ToListVar() != null)
                    {
                        var ln = expr.Trim().TrimStart('$');
                        Use(ln);
                        if (!IsList(ln))
                            Add(at, ValidationCodes.ParamOutputType,
                                $"'{k}' is a list parameter: its argument must be a list variable ($name)", field);
                    }
                    else CheckExpr(expr, at, field);
                }
                foreach (var p in parameters.Values)
                    if (p.ParamMode is ParamModes.In or ParamModes.InOut && !given.Contains(p.Name.Trim()))
                        Add(at, ValidationCodes.ParamMissing,
                            $"No argument for '{p.Name}': the routine uses its declared value", $"routineArgs.{p.Name.Trim()}",
                            ValidationSeverity.Warning);

                // Outputs: out / inOut parameters into a caller variable of a compatible kind.
                for (int j = 0; j < (s.RoutineOutputs?.Count ?? 0); j++)
                {
                    var m = s.RoutineOutputs![j];
                    if (m == null || string.IsNullOrWhiteSpace(m.VariableName)) continue;
                    var k = m.Key?.Trim() ?? "";
                    if (!parameters.TryGetValue(k, out var p) || p.ParamMode == ParamModes.In)
                    {
                        Add(at, ValidationCodes.UnknownParam,
                            p == null ? $"Routine '{routine.Name}' has no parameter '{k}'"
                                      : $"'{k}' is an in parameter of '{routine.Name}': nothing is copied back",
                            $"routineOutputs[{j}].key");
                        continue;
                    }
                    string field = $"routineOutputs[{j}].variableName";
                    var name = m.VariableName.Trim().TrimStart('$');
                    if (!CheckTarget(name, at, field, requireDeclared: false)) continue;
                    var target = Lookup(name);
                    if (target == null || SymOf(p) is not { } ps) continue;
                    if (!Compatible(ps, target))
                        Add(at, ValidationCodes.ParamOutputType,
                            $"'${name}' is {Describe(target)} but '{k}' is {Describe(ps)}", field);
                }
            }

            private static bool Compatible(Sym param, Sym target) => param.Kind switch
            {
                SymKind.String => target.Kind == SymKind.String,
                SymKind.List   => target.Kind == SymKind.List && target.ElementType == param.ElementType,
                SymKind.Image  => target.Kind == SymKind.Image,
                _              => target.Kind is SymKind.Number or SymKind.Boolean or SymKind.Stopwatch,
            };

            /// <summary>Names the flow blocks write without a declaration: a Try's error text and
            /// code variables, and a CallRoutine's outputs (with the parameter's kind).</summary>
            private void CollectImplicitFlow(ProgramStep s)
            {
                if (s.Type == StepType.Try)
                {
                    Implicit(s.ErrorVariableName, SymKind.String);
                    Implicit(s.ErrorCodeVariableName, SymKind.String);
                }
                else if (s.Type == StepType.CallRoutine && s.RoutineOutputs is { Count: > 0 } outs
                         && FindProgram(s.RoutineId, s.RoutineName) is { } r)
                {
                    foreach (var m in outs)
                    {
                        if (m == null) continue;
                        var p = r.Variables?.FirstOrDefault(v =>
                            string.Equals(v?.Name?.Trim(), m.Key?.Trim(), StringComparison.OrdinalIgnoreCase));
                        if (p == null || SymOf(p) is not { } ps || ps.Kind == SymKind.Computed) continue;
                        Implicit(m.VariableName, ps.Kind == SymKind.Stopwatch ? SymKind.Number : ps.Kind, ps.ElementType);
                    }
                }
            }
        }
    }
}
