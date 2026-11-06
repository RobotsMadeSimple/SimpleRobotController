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

The grid follows the board through a **fixture**: four **findLine** tools find the board's
outer top, right and left edges (white table → black frame) and the bottom edge of the
squares (the board's own bottom edge is often cut off), a **makeFixture** (`fourLines`)
intersects them into the board's four corners, and both piece tools are drawn on the
squares relative to it — so a bumped camera, or the board pushed or turned, moves the
8 × 8 grid with full perspective. If an edge can't be found (a hand over it), the piece
tools report *not run* that frame rather than read squares from a guessed position.

The program lives in `program/checkers-vision.json` (built only from the generic tools —
nothing in the controller knows about checkers); `program/checkers-vision.mjs` saves it
to a controller:

| Tool | Type | Settings |
|---|---|---|
| Board top / right / left edge | findLine on the `Gray` layer | outer edge, light → dark from outside, 16 probes, pick first, ignore 4 |
| Squares bottom edge | findLine on the `Gray` layer | dark → light from the frame inwards, pick strongest |
| Board | makeFixture `fourLines` | sources top, right, bottom, left |
| White pieces | pixelCount (colour) on the squares, fixture *Board* | 8 × 8, inset 15 %, cream (232, 230, 200) ± 20 %, `percent` ≥ 30 |
| Black pieces | intensity `stdDev` on the `Fine detail` layer (backgroundRemove 17 px), fixture *Board* | 8 × 8, inset 35 %, `stdDev` ≥ 4 |

The edge regions and the squares region are drawn on the default camera view. When the
camera has moved a lot, redraw the four edge regions over the board's edges and the
squares region over the squares, then press **Set reference** on *Board* (or clear its
reference: it is captured again on the next good frame).

The ready-made program **"Checkers Game (camera)"** does all of this: the robot plays
black on a taught 8x8 grid named *Checkers Board* (row r, col c = the board square),
clears jumped pieces to the point *Checkers Discard*, parks above it while you move,
and replies once the camera has seen your move.

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
