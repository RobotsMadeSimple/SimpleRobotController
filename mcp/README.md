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
point at the current position, reading and **configuring** IO (pin roles, HX711 load
cells and their calibration via `robot_io` / `robot_configure_pin` / `robot_load_cell`),
and `robot_stop`. Configuration sets channels up and persists to `nano_config.json`;
it moves nothing and drives no load, so it is ungated like teaching a point. *Actuating*
an output is the gated `robot_set_output`.

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
| `robot_validate_program` | | The controller's checks (offline structural checks if it is unreachable), without saving. |
| `robot_save_program` | | Create or replace a program. Validates on the controller first and refuses on errors; `allow_untaught` saves missing points/grids/routines as *pending*; `dry_run` only checks. Reads the program back and lists `droppedFields`. Does **not** run it. |
| `robot_modify_program` | | Targeted edits to a saved program (`set` / `replace` / `insert` / `remove` steps by id, `setVariable` / `removeVariable`, `setProgram`), validated like a save, with `newProblems` / `fixedProblems` against the saved version. `dry_run` previews. |
| `robot_program_text_get` | | One program as Simple Robot Script text (`docs/program-text.md`): one statement per step, names instead of ids. `include_ids`, `header` (the `@program` line for a file). |
| `robot_program_text_set` | | Create or replace a program from `.srs` text: parsed on the controller (problems as `line:col`), step ids kept by matching against the saved version, then validated and saved like `robot_save_program` (`allow_untaught`, `dry_run`, `force`). |
| `robot_teach_point` | | Save the current position under a name. Records, does not move. |
| `robot_io` | | Read IO config + state: every IO device (Nano etc.: id, connected, pins) and the relay board. `deviceId` and `nanoId` are accepted interchangeably. |
| `robot_configure_pin` | | Set a Nano pin's role (Input/Output/Neopixel/Unconfigured) and/or name. Persists; applies on the connected device. |
| `robot_load_cell` | | Configure/tare/calibrate an HX711 load cell, or change its display unit. |
| `robot_stop` | never | Stop program + motion + jog. |
| `robot_move` | ✓ | Single MoveL/MoveJ to a point or coordinates. |
| `robot_home` | ✓ | Clear faults and run homing. |
| `robot_run_program` | ✓ | Execute a saved program. |
| `robot_set_output` | ✓ | Drive a digital output. |
| `robot_raw_command` | partly | Any other WebSocket command. `Get*` ungated; everything else needs the gate. |

---

## Validation

`robot_save_program` and `robot_modify_program` run every program through the
**controller's** `ValidateBuiltProgram` before saving — it knows the expression
language, the property/function tables, plugin manifests and every point, tool,
local, grid, stack, routine and vision program on the robot — plus this server's
offline structural checks (missing/duplicate step ids, `$`-less names, …).

- **Errors block the save.** Expression syntax, unknown variables/properties/functions,
  read-only writes, routine recursion, labels, empty loops, missing fields, plugin
  param/output mismatches, a grid whose base point is missing, …
- **Missing references** — `unknownPoint`, `unknownGrid`, `unknownStack`, `unknownTool`,
  `unknownLocal`, `unknownRoutine`, `unknownVisionProgram` — are errors too (the run
  would fault there), but writing a program before teaching its points is normal:
  pass `allow_untaught: true` and they are saved and reported as `pending`.
- **Warnings never block**: disabled/unreachable steps, unused variables, a grid or
  stack with no spacing (`untaughtGrid`), a plugin that is not running, …
- `dry_run: true` checks (and diffs) without saving; `force: true` skips the gate and
  should only be used when a check is provably wrong.

After a save the program is **read back** and compared with what was sent:
`droppedFields` lists anything the controller did not keep — a misspelt field name or
a value of the wrong type is otherwise silently discarded, so the program saves "fine"
and the setting just isn't there.

When a save replaces an existing program, or `robot_modify_program` edits one, the
result carries `newProblems` (introduced by the change) and `fixedProblems` (resolved
by it), so an edit can be judged on its own effect rather than on the program's
existing debt.

If the controller cannot be reached, validation falls back to the offline checks
(and says so with `controllerChecked: false`); with the controller up, an offline
"unknown step type" defers to the controller, which knows its own step types.

`program_tools_check.py` exercises all of this against a live controller (and checks
that `step_schema.py` lists every `StepType` the controller has).

---

## Files

| File | |
|---|---|
| `robot_mcp.py` | The server — MCP over stdio, the WebSocket link, the tools, the validator. |
| `step_schema.py` | Per-step-type field reference served by `robot_step_schema`. Mirrors `RobotControl/Models/ProgramSteps.cs`; `program_tools_check.py` fails when a step type is missing, and `droppedFields` catches a misspelt field at save time. |
| `stdio_check.py` | End-to-end check — spawns the server as a subprocess and drives it over stdio the way a client does. Covers reads, the save round trip, validation, and every gate refusal. `python stdio_check.py [host]`, default `localhost`. Moves nothing. |
| `selftest.py` | Narrower in-process round trip: saves `_mcp_selftest`, reads it back, deletes it. Moves nothing. |
| `program_tools_check.py` | The save gate, `allow_untaught`, `droppedFields`, `robot_modify_program` edits/diffs, and step-schema drift against `ProgramSteps.cs`. Writes and deletes `_mcp_program_check`. Moves nothing. |
| `program_text_check.py` | The text tools: every program formats and parses back unchanged (dry run), an edit keeps step ids, parse errors come back with positions. Writes and deletes `_mcp_text_check`. Moves nothing. |
| `skill/SKILL.md` | Agent context — copy into `.claude/skills/robot-programming/`. |

MCP is spoken directly over stdio rather than through the `mcp` SDK, so the only
dependency is `websocket-client` and setup is one file.
