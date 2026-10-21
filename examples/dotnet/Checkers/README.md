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
