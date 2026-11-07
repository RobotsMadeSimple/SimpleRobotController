# Error and Problem Codes

Three kinds of codes:

| Kind | When | Effect |
|---|---|---|
| **Runtime error codes** | a step fails while the program runs | Catchable codes can be handled with a [Try](#errors); fatal codes always end the program with Error. The code is in the program status (`errorCode`) and the Try's code variable. |
| **Validation problems** | the controller checks the program (every edit, and before a run) | Errors block running; warnings are hints. |
| **Parse problems** | the text view is read back into a program | Errors block switching to Blocks and saving; warnings (unknown names) do not. Each has a line and column. |

A Raise Error step may use any identifier as its code (default `userError`); such codes are always
catchable.
