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

`harness.mjs` holds the shared pieces (`Ctrl`, `runProgram`, variable and step helpers) for new suites.
