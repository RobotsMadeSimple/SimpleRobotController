# Expressions and Conditions

Almost every numeric field of a step can be an **expression** instead of a plain number — a speed,
an offset, a count, an index. In Blocks, tap the **fx** toggle on a field; in text, write the
expression after `name=` or in parentheses in a positional slot:

```srs
MoveL Pick1, speed=$base * 2, offset=(z=$layer * 5)
SetSpeedL ($base * 2)          # positional values need parentheses
Wait ($delay)
```

Expressions are evaluated when the step runs, so they always use the current values.

## What an expression can contain

| Element | Examples |
|---|---|
| Numbers | `3.14` `-5` `1e3` |
| Booleans | `true` `false`, or `on` `off` (1 and 0) |
| Variables | `$count`, `$pts[$i].x`, `$vals[0]`, `$list.length` |
| IO | `$stb.in1` `$stb.out2` `$relay.3` `$io.Arduino_Nano.Input_1` |
| Properties | `$robot.z` `$program.runCount` `$time.hour` |
| Arithmetic | `+ - * / % ^` — `%` is the remainder, `^` is power; `/` and `%` by zero give 0 |
| Comparison | `== != < <= > >=` — yield 1 or 0 |
| Logic | `and or not` (also `&& \|\| !`) |
| Conditional | `cond ? a : b`, or `if(cond, a, b)` |
| Functions | `abs(x)`, `round(x, 2)`, `min(a, b, …)`, `dist(x1, y1, x2, y2)`, `len($list)` … |
| Text helpers | `$s.contains("x")`, `$s.startsWith("x")`, `$s.indexOf("x")`, `$s.toNumber()`, `$s.length` |
| Plugin calls | `scale.tare()`, `$scale.weight` |
| Grouping | `( … )` |

The full list of functions and properties is in [Functions and Properties](#functions).

> **Caution** Variables need the `$`. A bare word such as `count` is not a lookup — it evaluates to
> **0**, and the validator warns about it. `true`, `false`, `on` and `off` are not bare words: they are
> the values 1 and 0.

## Precedence

Tightest first:

| Level | Operators | Associativity |
|---|---|---|
| 1 | `^` | right (`2 ^ 3 ^ 2` is `2 ^ 9`) |
| 2 | unary `-` | — (`-2 ^ 2` is `-4`) |
| 3 | `*` `/` `%` | left |
| 4 | `+` `-` | left |
| 5 | `==` `!=` `<` `<=` `>` `>=` | left (`1 < 2 < 3` is `(1 < 2) < 3`) |
| 6 | `not` | — (`not $a > 5` is `not ($a > 5)`) |
| 7 | `and` | left |
| 8 | `or` | left |
| 9 | `? :` | right |

So `$count > 5 and $count < 10` needs no parentheses, and neither does `$n > 0 ? $sum / $n : 0`.

## Truth and equality

- Any non-zero value is **true**; comparisons and logic produce exactly 1 or 0.
- `==` compares within `1e-9`, so `0.1 + 0.2 == 0.3` holds.
- Both sides of `and` / `or` and both branches of `?:` are always evaluated. Nothing has side effects,
  and a misspelt variable fails the run whichever way the condition goes.

## Conditions

`if`, `elif`, `while`, Wait until, stop conditions (`until=`) and computed booleans all take a
**condition**, which is simply an expression read as true or false.

```srs
if $count > 5 and $stb.in1
  …
end
Wait until $io.Arduino_Nano.Input_1 == 1, timeout=5s
MoveL Pick1, until=$io.Arduino_Nano.Load_Cell >= 2000
```

In Blocks a condition is shown as **rows** (`left · operator · right`) joined by **ALL** (and) or
**ANY** (or). The text form's top-level `and` / `or` become those rows, so both views stay in step.

## IO names

| Name | Value |
|---|---|
| `$stb.in1` … `$stb.in4` | Driver board inputs, 1 / 0 |
| `$stb.out1` … `$stb.out4` | Driver board outputs (as last set), 1 / 0 |
| `$relay.1` … `$relay.4` | USB relay channels, 1 / 0 |
| `$io.<device>.<pin>` | An IO device pin: a digital pin as 1 / 0, a load cell as its weight in **grams** |

Device and pin names with spaces or other characters use their underscore form: device
"Arduino Nano", pin "Load Cell" → `$io.Arduino_Nano.Load_Cell`.

## When an expression fails

| Problem | When found | Result at run time |
|---|---|---|
| A misspelt `$name` | validator: `unknownVariable` | the program stops (`unknownVariable`, fatal) |
| A syntax error: `1 2`, unbalanced `(`, unknown function, wrong number of arguments | validator: `expressionSyntax`, `unknownFunction`, `badArity` | the program stops (`expressionSyntax`, fatal) |
| A list index out of range, an unknown field | — | reads as 0 |
| Division by zero | — | 0 |

These failures are **fatal** on purpose: a typo in an offset must never move the robot somewhere
unintended, and a Try cannot hide it.

> **Tip** The expression tester in the editor evaluates an expression against the live controller
> (`EvaluateExpression`) — use it to check a formula before running.
