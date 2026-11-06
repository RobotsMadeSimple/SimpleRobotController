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

`detectMove` takes the per-cell results of two vision inspections (white and black
pieces) run on an **8x8 grid zone** over the board, works out which dark squares hold
which colour, and compares that with the tracked game. If the change is exactly what
one legal move of the side to move would produce, it reports `status: "move"` with
`from`/`to`/`captured` — but does not apply it. Read it a few times in a row (so a hand
still over the board, or a half-finished multi-jump, can't trigger it), then call
`applyMove`. Anything else is `"same"` (nothing moved) or `"unclear"` (changed, no
legal match: mid-move, hand in view, a jumped piece not yet removed, an illegal move).

A cell counts as a piece when it **passed** its inspection (each colour keeps its own
tuned coverage range); set `minCoverage` > 0 to use one threshold instead. A cell that
reads as both colours is white by default (`bothMeans`), which is what a board with
black pieces on black squares needs (below). The camera
grid's row 0 is the top of the image; `orientation` says where black's side (board row 0)
is in it: `normal` = top, `rotated90` = right (the players sit left and right of the
camera, as in a side view across the table), `rotated180` = bottom, `rotated270` = left.

**Black pieces on black squares.** A colour inspection can't tell them apart, so read
black pieces with a **Texture** inspection instead: a ringed checker has fine detail, a
plain square has none (on real photos black pieces score 6.5+ and empty squares under 3,
at 1280 px). White pieces pass a texture check too, which is why `bothMeans` defaults to
white. Align the grid with a **Checker grid** frame on the same zone so it snaps onto the
squares and follows the camera. The vision program builder
`program/checkers-vision.mjs` creates this setup:

| Inspection | Type | Settings |
|---|---|---|
| Squares | Frame, method *Checker grid*, on the Board zone | 8 × 8, min edge ratio 4 |
| White pieces | Colour coverage on the Board zone | cream (232, 230, 200) ± 20 %, min 30 % |
| Black pieces | Texture on the Board zone | cell inset 25 %, min score 4 |

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
