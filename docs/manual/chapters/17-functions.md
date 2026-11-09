# Functions and Properties Reference

Everything an expression can read. Operators and precedence are described in
[Expressions and Conditions](#expressions); this chapter lists the functions, the read-only
properties and the IO names.

## Operators

| Operator | Meaning | Example |
|---|---|---|
| `+ - * /` | arithmetic (`/` by zero gives 0) | `$w * $h` |
| `%` | remainder, with the sign of the left side (by zero gives 0) | `$i % 4` |
| `^` | power, right-associative | `2 ^ 10` |
| `== != < <= > >=` | comparison → 1 / 0 (`==` within 1e-9) | `$grams >= 100` |
| `and or not` (`&& \|\| !`) | logic → 1 / 0 | `$a and not $b` |
| `c ? a : b` | conditional | `$n > 0 ? $sum / $n : 0` |
| `( )` | grouping | `($a + $b) / 2` |

## Notes on functions

- Function names are case-insensitive.
- Angles are in **degrees**, in and out.
- `round` rounds half away from zero: `round(2.5)` = 3, `round(-2.5)` = -3; negative digits round to
  tens, hundreds …
- The list functions take a bare list reference: `sum($vals)`, `avg($samples.v)` (a field of each
  element of a Point or Record list). An empty list gives 0.
- An unknown function is `unknownFunction`; a wrong number of arguments is `badArity`.
- A running plugin's functions are called as `<pluginId>.<name>(…)`, its properties read as
  `$<pluginId>.<name>` — see [Plugins](#plugins).
