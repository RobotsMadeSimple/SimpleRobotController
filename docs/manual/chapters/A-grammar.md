# Simple Robot Script Grammar

The grammar of Simple Robot Script, language version 1, in EBNF. `COND` and `EXPR` are the
expression language of [Expressions and Conditions](#expressions).

## Lexical rules

| Element | Rule |
|---|---|
| Line | One statement per line. A line continues while a `(`, `[` or `{` is open, or after a trailing `,`. A blank line ends a continuation. |
| Comment | `#` to the end of the line, outside strings. `##` starts a section header. |
| Version | The first line `#!srs 1` (optional when parsing). |
| Strings | `"…"` with the escapes `\"` `\\` `\n` `\t` `\r`; any other backslash is kept (`"C:\data"` works). Template braces `{$x}` are kept and filled in at run time. |
| Numbers | `3`, `-2.5`, `1e3`. Fields in milliseconds also take `500ms` and `1.5s`. |
| Booleans | `on` `off` `true` `false` (as values of step arguments and declarations). |
| Names | Bare when an identifier (`Pick1`), else quoted (`"Pick 1"`) or in underscore form (`Pick_1`). |
| Variables | `$name`, with `[…]` indexes and `.field` accessors. |
| Positional values | Right after the keyword, separated by spaces: a name, string, number, `on`/`off`, a `$var`, or an expression **in parentheses**. |
| Named values | `name=value`, comma-separated; a named value runs to the next top-level comma, so it can be any expression. |
| Literal or expression | A plain number is stored as the number; anything else as an expression. Parentheses always mean an expression: `speed=(200)`. |

## Grammar

```ebnf
file        = [ "#!srs" INT NL ] { line } ;
line        = ( decl | attr | stmt | comment | blank ) [ "#" text ] NL ;
attr        = "@" ( "name" STRING | "off" | "id" STRING | "with" JSON | "program" JSON ) ;
comment     = "#" text | "##" text ;                 (* paragraph → Note; "##" → section *)

decl        = { "global" | "persistent" | "input" | "monitor" }
              ( "var" | "computed" | "param" ( "in" | "out" | "inout" ) ) NAME [ ":" type ] [ "=" init ] ;
type        = "number" | "bool" | "string" | "image" | "stopwatch"
            | "list" ( "number" | "bool" | "point" | "record" ) ;
init        = EXPR | STRING | "(" [ value { "," value } ] ")" ;

stmt        = if | loop | switch | try | assign | listop | flow | keyword-stmt ;
if          = "if" COND NL body { ( "elif" | "else if" ) COND NL body } [ "else" NL body ] "end" ;
loop        = ( "repeat" [ ( "forever" | POS ) ] [ "as" VAR ]
              | "while" COND
              | "for" ( VAR | "_" ) [ "," VAR ] "in" VAR ) [ "," args ] NL body "end" ;
switch      = "switch" EXPR NL { "case" EXPR { "," EXPR } NL body } [ "default" NL body ] "end" ;
try         = "try" [ args ] NL body [ "catch" [ VAR ] [ "," args ] NL body ] "end" ;
assign      = VAR "=" ( EXPR | STRING )
            | VAR "[" EXPR "]" [ "." IDENT ] "=" ( EXPR | "(" IDENT "=" EXPR { "," IDENT "=" EXPR } ")" ) ;
listop      = VAR "." ( "append" | "insert" | "removeAt" | "clear" | "sort" | "reverse" | "copyFrom" )
              "(" [ arglist ] ")" ;
flow        = "break" | "continue" | "return" | "pause"
            | "exit" [ STRING ] | "raise" STRING [ "," args ] | "label" NAME | "goto" NAME
            | "call" NAME [ "(" IDENT "=" value { "," IDENT "=" value } ")" ] { "," "out" IDENT "=" VAR }
            | "status" [ "info" | "warning" | "error" ] [ STRING ] [ "," args ]
            | "stopwatch" ( "start" | "stop" | "reset" ) VAR
            | "prompt" STRING [ "," args ]
            | "plugin" NAME "." NAME [ "," args ] [ rows ]
            | "unknown" STRING ;
keyword-stmt= KEYWORD { POS } [ [ "," ] args ] [ rows ] ;    (* KEYWORD: a step type name *)
rows        = NL { row NL } "end" ;     (* RunVision, HttpRequest, HttpReceive, ExportData, plugin *)
row         = [ "send" ] KEY "=" ( EXPR | "list" VAR | "image" VAR )
            | [ "receive" ] KEY "->" VAR
            | KEY "." KEY "->" VAR ;                          (* RunVision: tool.value -> $var *)
args        = named { "," named } ;
named       = IDENT "=" value ;
value       = EXPR | STRING | BOOL | NUMBER [ "ms" | "s" ] | tuple | list | target ;
tuple       = "(" IDENT "=" EXPR { "," IDENT "=" EXPR } ")" ;
list        = "(" NAME { "," NAME } ")" ;
target      = "here" | NAME | STRING | VARREF | ( "grid" | "stack" ) NAME "[" EXPR [ "," EXPR ] "]" ;
POS         = NAME | STRING | NUMBER [ unit ] | BOOL | VARREF | "(" EXPR ")" | target | "J" INT | "toggle" ;
NAME        = IDENT | STRING ;
VAR         = "$" IDENT ;
VARREF      = VAR { "[" EXPR "]" | "." IDENT } ;
COND, EXPR  = the expression language, up to the next top-level "," or the end of the line ;
```

Every step type name is also a keyword (`IfCondition`, `Loop`, `SetVariable`, `Note` …): the formatter
uses the bare type name plus `@with` for a step it cannot otherwise write.

## Expression grammar

```ebnf
expr        = cond ;
cond        = or [ "?" cond ":" cond ] ;
or          = and { ( "or" | "||" ) and } ;
and         = not { ( "and" | "&&" ) not } ;
not         = ( "not" | "!" ) not | cmp ;
cmp         = add { ( "==" | "!=" | "<" | "<=" | ">" | ">=" ) add } ;
add         = mul { ( "+" | "-" ) mul } ;
mul         = unary { ( "*" | "/" | "%" ) unary } ;
unary       = ( "-" | "not" | "!" ) unary | pow ;      (* "not" as an operand binds like "-" *)
pow         = atom [ "^" unary ] ;
atom        = NUMBER | "true" | "false" | VARREF | call | "(" expr ")" ;
call        = IDENT { "." IDENT } "(" [ expr { "," expr } ] ")" ;
```

This expression grammar is informative — a summary of the precedence table in [Expressions and Conditions](#expressions)
(`^` binds tightest and is right-associative; `? :` binds loosest).
