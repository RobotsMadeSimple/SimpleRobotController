# Robot MCP Server

An MCP server that puts SimpleRobotController in front of an agent: read robot
state, author and save built programs, and — behind an explicit gate — drive the
hardware.

It talks to the controller over the same WebSocket the app uses
(`ws://<robot-ip>:9000/control`, see [`../docs/websocket-api.md`](../docs/websocket-api.md)),
so it works against any running controller with no changes on the controller side.

---

## Safety

Programs are data; running them moves a machine. The server splits on that line.

**Always available** — status, points, program read/write, validation, teaching a
point at the current position, and `robot_stop`.

**Gated** behind `RMS_MCP_ALLOW_MOTION=1` — `robot_move`, `robot_home`,
`robot_run_program`, `robot_set_output`, and any non-`Get*` command through
`robot_raw_command`. With the gate closed these tools are still listed (so the
agent knows they exist and can tell you how to enable them) but refuse to run.

`robot_stop` **ignores the gate** — it halts the running program, hard-stops
motion, and cancels any jog. A stop must never be the thing that is unavailable.

There is deliberately no jog tool. Continuous motion that only stops on a second
command is a bad fit for an agent.

Keep the gate closed by default. Open it for a session where you are watching the
robot, then close it again.

---

## Setup

Requires `websocket-client` (and `zeroconf` for mDNS discovery, optional if you
set the host explicitly):

```
pip install websocket-client zeroconf
```

Register it by creating `.mcp.json` in the workspace root
(`C:\Users\lucas\Desktop\RobotsMadeSimple\`):

```json
{
  "mcpServers": {
    "robot": {
      "command": "python",
      "args": ["SimpleRobotController/mcp/robot_mcp.py"],
      "env": {
        "RMS_MCP_ALLOW_MOTION": "0"
      }
    }
  }
}
```

Flip `RMS_MCP_ALLOW_MOTION` to `"1"` and restart the session when you want the
robot to actually move.

### Agent context

`skill/SKILL.md` is the context that makes the tools usable without re-deriving the
program model every session — order of operations, the `$`-sigil rule, move-target
precedence, program shape, and the safety posture. Drop it in as a skill so it loads
on demand rather than sitting in every conversation:

```powershell
Copy-Item -Recurse SimpleRobotController\mcp\skill `
  "$HOME\Desktop\RobotsMadeSimple\.claude\skills\robot-programming"
```

### Environment

| Variable | Effect |
|---|---|
| `RMS_ROBOT_URL` | Full endpoint, e.g. `ws://192.168.4.23:9000/control`. Highest priority. |
| `RMS_ROBOT_HOST` | Host or IP; the URL becomes `ws://<host>:9000/control`. |
| `RMS_MCP_ALLOW_MOTION` | `1` unlocks the motion / execution / output tools. |
| `RMS_MCP_TIMEOUT` | Seconds to wait for a command ACK. Default `10`. |

With neither URL nor host set, the server finds the robot over mDNS (`_robot._tcp`),
the same way the app does. Set `RMS_ROBOT_HOST` if discovery is slow or the robot
is on another subnet (e.g. over Tailscale).

---

## Tools

| Tool | Gated | What it does |
|---|---|---|
| `robot_status` | | Pose, homed/faulted flags, motion defaults, IO, running programs. Also reports the endpoint in use and whether the gate is open. |
| `robot_points` | | Taught points — names by default, coordinates on request. |
| `robot_programs` | | Summary of every built program. |
| `robot_get_program` | | Full JSON of one program. |
| `robot_step_schema` | | Field reference for step types. No args → type list + program shape; `types=[...]` → full fields. |
| `robot_validate_program` | | Static checks without saving. |
| `robot_save_program` | | Create or replace a program. Validates first, refuses on errors. Does **not** run it. |
| `robot_teach_point` | | Save the current position under a name. Records, does not move. |
| `robot_stop` | never | Stop program + motion + jog. |
| `robot_move` | ✓ | Single MoveL/MoveJ to a point or coordinates. |
| `robot_home` | ✓ | Clear faults and run homing. |
| `robot_run_program` | ✓ | Execute a saved program. |
| `robot_set_output` | ✓ | Drive a digital output. |
| `robot_raw_command` | partly | Any other WebSocket command. `Get*` ungated; everything else needs the gate. |

---

## Validation

`robot_validate_program` (and the check `robot_save_program` runs first) splits
findings two ways, and the split is the point:

**Errors** block a save — things that are structurally broken or *silently* wrong:

- a step `type` the executor does not recognise (it stores these as `Unknown` and skips them)
- duplicate or missing step ids
- a `GoToLabel` with no matching `Label`
- `SetVariable` assigning a name not declared in `variables`
- a `forEach` loop with no variable, a `while` loop with no condition
- **a variable reference missing its `$`** — `baseSpeed * 2` evaluates to `0`, not to the
  variable, so a forgotten sigil is a plausible-looking wrong answer rather than an error
- motion steps inside a program marked `isBackground` (the executor skips them)

**Warnings** do not block — legal but probably unintended:

- a move with no target (falls back to the current TCP, so it does not move)
- a point that is not taught yet — writing the program before teaching its points is a
  normal way to work, and the run fails loudly rather than moving somewhere wrong
- unconfigured steps, empty loop bodies, `loopCount: 0` (which means forever)

Validated against all 14 programs currently on the robot: zero false-positive errors.

---

## Files

| File | |
|---|---|
| `robot_mcp.py` | The server — MCP over stdio, the WebSocket link, the tools, the validator. |
| `step_schema.py` | Per-step-type field reference served by `robot_step_schema`. Mirrors `RobotControl/Models.cs`; **nothing enforces the link**, so a field added there needs adding here or the agent writes steps the executor ignores. |
| `stdio_check.py` | End-to-end check — spawns the server as a subprocess and drives it over stdio the way a client does. Covers reads, the save round trip, validation, and every gate refusal. `python stdio_check.py [host]`, default `localhost`. Moves nothing. |
| `selftest.py` | Narrower in-process round trip: saves `_mcp_selftest`, reads it back, deletes it. Moves nothing. |
| `skill/SKILL.md` | Agent context — copy into `.claude/skills/robot-programming/`. |

MCP is spoken directly over stdio rather than through the `mcp` SDK, so the only
dependency is `websocket-client` and setup is one file.
