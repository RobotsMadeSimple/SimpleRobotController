# Checkers Opponent plugin

An American-checkers opponent that runs **inside the controller** as a hosted
plugin — the controller launches and supervises it (`runtime: "exe"`). There is
no separate server to start. It replaces the old standalone `CheckersServer`
(the alpha-beta engine is a faithful C# port of `CheckersServer/checkers.py`).

The plugin is pure game logic. It speaks in **dark-square indices 0..31** and
never touches the arm itself — the robot *program* maps a square to a physical
point and drives the motion. That keeps the plugin portable across board sizes
and robot setups.

## Square numbering

Only the 32 dark squares are playable, numbered left-to-right, top-to-bottom:

```
row 0   .  0  .  1  .  2  .  3
row 1   4  .  5  .  6  .  7  .
row 2   .  8  .  9  . 10  . 11
...
row 7  28  . 29  . 30  . 31  .
```

Black starts on rows 0–2 (squares 0–11) and moves down; white on rows 5–7
(20–31) and moves up. Black promotes on row 7, white on row 0.

## What it contributes

**Steps** (builder → Plugins category):

| Step | Params | Key outputs |
|---|---|---|
| `newGame` | `firstPlayer` (black/white) | `turn`, `moveText` |
| `setBoard` | `white`,`black`,`whiteKings`,`blackKings` (32-bool lists), `turn` | `legal`, `whiteCount`, `blackCount`, `winner` |
| `applyMove` | `from`, `to` (squares) | `legal`, `captured` (list), `path`, `kinged`, `gameOver`, `winner` |
| `detectMove` | `whiteCells`, `blackCells` (vision cell lists), `minCoverage?`, `orientation` (normal/rotated90/rotated180/rotated270), `bothMeans` (white/black/neither) | `status` (same/move/unclear), `from`, `to`, `captured`, `changed`, `moveText` |
| `aiMove` | `maxDepth?`, `timeBudgetMs?` | `fromSquare`, `toSquare`, `path` (list), `captured` (list), `kinged`, `gameOver`, `winner`, `moveText` |

**Functions:** `checkers.pieceAt(square)` (0 empty, ±1 man, ±2 king; + white / − black),
`checkers.legalMoves()` (count for side to move).

**Properties:** `$checkers.gameOver`, `$checkers.turn` (0 white / 1 black),
`$checkers.whiteCount`, `$checkers.blackCount`, `$checkers.lastFrom`, `$checkers.lastTo`.

## Typical robot program

```
Plugin: checkers.newGame
Loop:
    (observe the board — vision — into 32-element lists)
    Plugin: checkers.setBoard  → winner, legal
    If $checkers.gameOver: break
    Plugin: checkers.aiMove    → fromSquare, toSquare, captured[], kinged
    For each s in captured[]:  MoveL to point(s) ; (lift piece off board)
    MoveL to point(fromSquare) ; pick ; MoveL to point(toSquare) ; place
    If kinged: (place a second piece / flip to a king)
    If $checkers.gameOver: break
    (wait for the human's move, then) Plugin: checkers.applyMove(from,to)
```

### Spotting the human's move with the camera

`detectMove` takes the per-cell results of two vision tools (white and black
pieces) gridded **8x8** over the board, works out which dark squares hold
which colour, and compares that with the tracked game. If the change is exactly what
one legal move of the side to move would produce, it reports `status: "move"` with
`from`/`to`/`captured` — but does not apply it. Read it a few times in a row (so a hand
still over the board, or a half-finished multi-jump, can't trigger it), then call
`applyMove`. Anything else is `"same"` (nothing moved) or `"unclear"` (changed, no
legal match: mid-move, hand in view, a jumped piece not yet removed, an illegal move).

A cell — a RunVision `cells` record `{ row, col, index, value, passed }` — counts as a
piece when it **passed** its tool's limit (each colour keeps its own tuned range); set
`minCoverage` > 0 to compare the cell's `value` against one threshold instead. A cell that
reads as both colours is white by default (`bothMeans`), which is what a board with
black pieces on black squares needs (below). The camera
grid's row 0 is the top of the image; `orientation` says where black's side (board row 0)
is in it: `normal` = top, `rotated90` = right (the players sit left and right of the
camera, as in a side view across the table), `rotated180` = bottom, `rotated270` = left.

**Black pieces on black squares.** Colour can't tell them apart, so black pieces are read
by their texture instead: a ringed checker has fine detail, a plain square has none. The
`Fine detail` layer subtracts a blurred copy of the image (slow shading — glare, shadows —
drops out) and an **intensity** tool measures the spread (`stdDev`) of what is left in the
middle of each square: on the real photos black pieces score 7+ and empty dark squares
under 1.5, at 1280 px. White pieces have texture too, which is why `bothMeans` defaults to
white.

By default the 8 × 8 grid is **fixed**: one skewed (four-corner) area on the squares that
stays where you put it — no edge finding. *Black pieces* shares *White pieces*' area
(`regionFrom`), so the four corners are set once. To set them: **Vision editor → White pieces
(or Black pieces) → Area → Draw**, drag each corner onto the corner of the playing squares
(the `1,1` corner is row 1, column 1; a magnifier shows the spot under your finger), then
**Save**. Do it again whenever the camera or the board moves.

The program lives in `program/checkers-vision.json` (built only from the generic tools —
nothing in the controller knows about checkers); `program/checkers-vision.mjs` saves it
to a controller (`CTRL=host:port node checkers-vision.mjs save [cameraId]`):

| Tool | Type | Settings |
|---|---|---|
| White pieces | pixelCount (colour), skewed area on the squares, no fixture | 8 × 8, inset 15 %, cream (232, 230, 200) ± 20 %, `percent` ≥ 30 |
| Black pieces | intensity `stdDev` on the `Fine detail` layer (backgroundRemove 17 px), same area as *White pieces* | 8 × 8, inset 35 %, `stdDev` ≥ 4 |

**Tracked version** (`save --tracked`, `program/checkers-vision-tracked.json`, same id and
outputs): the grid follows the board through a **fixture** — four **findLine** tools find
the board's outer top, right and left edges (white table → black frame) and the bottom edge
of the squares, a **makeFixture** (`fourLines`) intersects them into the board's four
corners, and both piece tools are drawn relative to it, so a bumped camera or a pushed board
moves the grid with full perspective. If an edge can't be found (a hand over it), the piece
tools report *not run* that frame. When the camera has moved a lot, redraw the four edge
regions and the squares region, then press **Set reference** on *Board*.

The ready-made program **"Checkers Game (camera)"** does all of this: the robot plays
black on a taught 8x8 grid named *Checkers Board* (row r, col c = the board square),
clears jumped pieces to the point *Checkers Discard*, parks above it while you move,
and replies once the camera has seen your move.

### Checker silos (crowning)

Two **silos** — tubes of spare checkers on a lifting platform, one per colour — supply
the crowns. The controller drives them with the generic aux **Silo** role
([docs/aux-silo.md](../../../docs/aux-silo.md)): `SiloAction home | present | refill |
setcount`. `kinged` (from `aiMove` / `applyMove`) means *the piece that just moved* was
promoted, so it is the robot's piece after `aiMove` and yours after `applyMove`:

- **Robot's king** (black): `SiloAction present` lifts the top black checker to the rim,
  the robot picks it at the point **Checkers Silo Robot**, puts it on the new king (the
  square + `$checkerMm`, 3.9 mm) and releases. If the silo is empty or not homed
  (`siloEmpty` / `siloNotHomed`, caught) it falls back to asking you to crown it.
- **Your king** (white): the white silo presents a checker and a status asks you to stack
  it on your piece — you crown your own.
- **Start of the game**: both silos are homed. Set the program variable `$siloRefill` to 1
  when you have just loaded both tubes full — they are refilled (count = capacity) and homed.

**Setup:**

1. **Wiring** — one Uno CNC-shield axis per silo (X = 0, Y = 1, Z = 2, A = 3) on the aux
   stepper board. The builder defaults to device `AUX_STEPPER_001`, black (robot) silo on
   axis 0, white silo on axis 1; change them when building:
   `SILO_DEVICE=… SILO_ROBOT_AXIS=… SILO_HUMAN_AXIS=… node program/checkers-game.mjs save`.
   The SiloAction steps link the silo by that device id + axis (they are not expressions).
2. **Driver current** — homing has no switch: the platform drives into the bottom stop and
   the motor skips. Turn each driver's current pot down until the lift still raises a full
   stack but skips gently against the stop (keep `siloHomeSpeed` slow).
3. **Microstepping** — the shield's M0/M1/M2 jumpers under each driver: no jumpers = full
   step, all three = 1/16 (A4988) or 1/32 (DRV8825). `stepsPerRev` = 200 × microsteps
   (1.8° motors), e.g. 3200 at 1/16.
4. **Configure each axis** (I/O → aux board → axis): role **Silo**, axis type Linear and:

   | Field | Value |
   |---|---|
   | `mmPerRev` | 40 |
   | `stepsPerRev` | 200 × microsteps |
   | `invertDirection` | set so **positive = platform up**. The twin motors are mirrored, so one silo is inverted and the other not — jog +1 mm and check |
   | `siloPitchMm` | 3.9 (one checker) |
   | `siloCapacity` | 12 |
   | `siloClearanceMm` | 0.5 |
   | `siloHomeOverTravelMm` | 5 |
   | `siloHomeSpeed` | 5 mm/s |
   | `actionVelocity` | 20 mm/s |

5. **Load and home** — fill each tube (platform at the bottom), then run `SiloAction refill`
   on it (or the game with `$siloRefill` = 1). The count is kept across restarts; any
   manual jog clears "homed", so home again after jogging.
6. **Teach the point *Checkers Silo Robot*** — the gripper's pick pose on the top checker of
   the black silo **after a Present** (present one, jog onto it, save the point). The program
   only references it.

`point(square)` is the integration point you own: a saved **grid** over the
board, or 32 named points `sq0..sq31`. The plugin only ever gives you the square
numbers; mapping them to reachable poses — and any off-board "captured pile" and
"king supply" locations — lives in the program and the robot's taught points.

## Build & install

Self-contained (no runtime needed on the robot PC), matching `plugin.json`'s
`runtime: "exe"` / `entry: "Checkers.exe"`:

```sh
dotnet publish examples/dotnet/Checkers -c Release -r win-x64 --self-contained
# or -r linux-x64 for a Linux controller host
```

Zip the publish output (it contains `plugin.json` next to `Checkers.exe`) and
install from the app: **IO → Plugins → Install**, or `POST /plugins/install`
(`application/zip`). The controller unpacks it to `<data dir>/plugins/checkers/`,
starts it, and `$checkers.*` / the Plugins step category appear once it reports
ready. Tune strength in the plugin's **Config** (`maxDepth`, `timeBudgetMs`).
