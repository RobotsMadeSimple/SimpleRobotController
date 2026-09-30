# G-code support

The controller runs common G-code three ways:

1. **From a stored file** — upload over HTTP, run with `RunGcodeFile`.
2. **As a program step** — `GcodeProgram` references a stored file (or inline text) and expands
   to moves at runtime, like the `CncProgram` step.
3. **Over a live stream** — a raw **TCP** line protocol (GRBL-style, for senders like UGS / Candle
   / bCNC / netcat) and a **WebSocket** channel (`/gcode/stream`) for the app or web tooling.

All three share one interpreter (`RobotControl/Gcode/GcodeInterpreter.cs`). Moves go through the
normal `MoveL` path, so G-code works on every robot type (ASTRO via IK, CNC4Axis as identity).

## Supported codes

| Code | Meaning |
|------|---------|
| `G0` | Rapid move (uses the configured rapid speed) |
| `G1` | Linear feed move (`F` word, mm/min) |
| `G2` / `G3` | Clockwise / counter-clockwise arc (XY plane), flattened to blended segments |
| `G4 P<sec>` | Dwell |
| `G17` | XY plane (the only plane supported for arcs) |
| `G20` / `G21` | Units: inch / millimetre |
| `G28` | Run the homing sequence |
| `G90` / `G91` | Absolute / relative distance mode |
| `G92` | Set current work position (coordinate offset) |
| `G94` | Feed per minute (the only feed mode) |
| `M0` / `M1` | Pause (program/step mode); ignored in a live stream |
| `M2` / `M30` | Program end |
| `M3` / `M4` | Spindle/laser on (routed to a configurable output) |
| `M5` | Spindle/laser off |

Words: `X Y Z A F S I J R P N`. `A` maps to the **RZ** rotary axis. Comments `; …` and `( … )`
and line numbers `N…` are ignored. Unsupported words **error** in a stream (the sender sees
`error:…`) and are **skipped with a log** when running a file/step (one bad line won't abort it).

## Coordinates, units, feed

- Coordinates are **base-frame** targets (the active local is ignored), the same as a raw-vector
  `MoveL`. Home/position the robot so machine coordinates mean what your file expects.
- An axis not named on a line **holds its current position** (safe for axes a file never touches).
- `F` is **mm/min**; the controller runs mm/s, so feed is divided by 60. `G20` scales coordinates
  and feed by 25.4.
- Arcs are flattened to line segments at the configured chord tolerance and blended into one path.

## Configuration (Robot › Configure → G-code)

| Field | Default | Notes |
|-------|---------|-------|
| Spindle output type | `none` | `none` / `stb` / `relay` — where `M3/M4/M5` is routed |
| Spindle output pin | `1` | Output number on that card |
| Rapid speed | `100` mm/s | G0 speed |
| Default feed | `50` mm/s | Used when a G1 has no `F` yet |
| Arc tolerance | `0.1` mm | Max chord error when flattening arcs |
| Stream TCP port | `8500` | Raw-TCP listener port (restart to change) |
| TCP stream enabled | `true` | Toggle the raw-TCP listener (restart to change) |

`S` (spindle speed) is parsed but not applied as PWM — `M3/M4` simply turn the output on, `M5` off.

## Files (HTTP)

Same shape as `/dxf`, stored per-robot in the `gcode/` data folder:

- `POST /gcode?name=<file>` — body is the raw G-code text (`.nc .gcode .tap .ngc .txt`)
- `GET /gcode` — list file names
- `GET /gcode/<name>` — download
- `DELETE /gcode/<name>` — delete

Commands: `RunGcodeFile { name }` (wraps the file in a one-step program and runs it — stop with
`StopBuiltProgram`), `ValidateGcodeFile { name }` → `{ ok, lines, moves, error? }`.

## Streaming

A stream **owns the motion queue**: opening one is refused while a program runs, and program
runs are refused while a stream is open. Flow control is GRBL-style — a motion line only gets
`ok` once the planner queue has room (≤ 16 queued moves), so the sender paces itself. Non-motion
lines (`M3`, `G4`, …) `ok` immediately.

**Realtime bytes** (sent on their own): `?` → a status line `<Idle|Run,MPos:x,y,z|A:rz>`;
`!` or `0x18` → stop motion. We have no resumable feed-hold, so both `!` and Ctrl-X stop.

### Raw TCP (`GcodeTcpServer`, default `:8500`)

```
$ nc <robot-ip> 8500
SimpleRobot G-code stream ready
G21 G90
ok
G1 X10 Y5 F600
ok
?
<Idle|MPos:10.000,5.000,0.000|A:0.000>
```

Point any GRBL-style sender at `<robot-ip>:8500`.

### WebSocket (`/gcode/stream`)

Connect to `ws://<robot-ip>:9000/gcode/stream`, send G-code lines as text frames, receive
`ok` / `error:…` / status lines as text. Single-character frames `?` / `!` are realtime controls.

## Safety notes

- A dropped stream connection stops motion and turns the spindle output off.
- The STB4100 must be connected for physical motion; without it the logic runs and acks but the
  robot does not move.
