# Live tests

End-to-end checks against a **running** controller: each suite saves real built programs over the
WebSocket API, runs them, reads back their variables and final status, and deletes them again
(all are named `zz live …`). They complement the xUnit suite, which cannot catch wire-format,
persistence or executor-timing bugs.

Run with Node 24+ (built-in `WebSocket`, no install):

```
node live-tests/stopcond.mjs            # all cases
node live-tests/stopcond.mjs jump       # cases whose name contains "jump"
V=1 node live-tests/stopcond.mjs        # print every observed value
CTRL=192.168.4.52:9000 node live-tests/stopcond.mjs
```

Safe on a PC with no motor driver attached: robot motion is simulated and aux axes simulate
offline. **They move the robot** — do not point them at an arm with hardware connected unless it
is clear to move (relative moves of up to 40 mm in X and 20 mm in Z, plus the gripper).

| Suite | Covers |
|---|---|
| `stopcond.mjs` | Stop / record on condition: MoveL stop, record and never-fires, MoveJ, JumpL stop (legs skipped) and record, no-blend on a watched MoveL, bad-expression error, AuxMove stop and record, Gripper close stopped halfway. |
| `flow.mjs` | Program flow and debugging (no motion): Try/catch/retry, Raise Error with errorCode + failing step, Wait onTimeout=error, Switch, break/continue, routine in/out/inOut params + Return, SetOutput toggle/expression, breakpoints, step mode, StepBuiltProgram, SetBreakpoints, run from a step in a loop, refused start step. |
| `steps.mjs` | Motion, data and operator steps: Move Arc path sampled against the circle and a full circle back to the start, collinear → `arcDegenerate`; Save Position into a Points list and a saved point; Set Speed Factor (`$program.speedFactor`, a 25 % run takes > 2.5× as long); List actions on number / point / record lists, `indexOutOfRange` caught, `badListOperation` fatal; `inBox` / `inCircle` / text helpers; Prompt + AnswerPrompt (published fields, badValue / badButton / stalePrompt, timeout button, `promptTimeout` caught, refused while stopped and re-shown on Continue); validator + catalog over the wire. Relative moves of up to 40 mm in X and 20 mm in Y. |

`harness.mjs` holds the shared pieces (`Ctrl`, `runProgram`, variable and step helpers) for new suites.
