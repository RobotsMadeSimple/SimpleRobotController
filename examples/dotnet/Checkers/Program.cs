using System.Text.Json;
using Checkers;
using SimpleRobot.PluginSdk;

// Checkers opponent as a controller-hosted plugin. The controller launches and
// supervises this process (runtime: "exe"); there is no separate server to run.
// It holds one game (board + side to move), computes the robot's move with an
// alpha-beta search, and hands the robot program plain 0..31 square indices —
// the program maps a square to a physical point (a grid or named points) and
// drives the arm. See docs/plugins.md and CheckersServer/checkers.py (the engine
// this is ported from).

if (args.Length > 0 && args[0] == "--selftest") return CheckersSelfTest.Run();

var game = new Game();
var plugin = new PluginHost();   // reads SRC_PLUGIN_* from the environment (set by the controller)

plugin.OnReady(ctx =>
{
    game.Configure(ctx.Config);
    ctx.Log($"checkers ready (controller {ctx.ControllerVersion}) — depth {game.MaxDepth}, budget {game.TimeBudgetMs} ms");
    game.Publish(ctx);
    return Task.CompletedTask;
});

plugin.OnConfigChanged((ctx, cfg) =>
{
    game.Configure(cfg);
    ctx.Log($"config: depth {game.MaxDepth}, budget {game.TimeBudgetMs} ms");
    return Task.CompletedTask;
});

// ── Steps ───────────────────────────────────────────────────────────────────

// Reset to the standard opening. Black (rows 0-2) moves first.
plugin.Step("newGame", (ctx, p) =>
{
    string side = p.GetString("firstPlayer", "black");
    var result = game.NewGame(side);
    game.Publish(ctx);
    ctx.Log(result["moveText"]?.ToString() ?? "new game");
    return Task.FromResult(result);
});

// Load an externally-observed board (e.g. from the vision pipeline) as the truth,
// replacing the tracked game. Lists are 32-element boolean (or 0/1) arrays — or, given
// whiteCells/blackCells, a camera reading (the same params as detectMove), read as men.
// A camera reading can be held back while something is over the board:
//   maxStrayCells — pieces only sit on dark squares, so light squares showing something are a
//                   hand (or arm) in view: more than this many beyond the few the loaded board
//                   itself lights (pieces spill into their neighbours); -1 (default) = don't check.
//   settleReads   — only load a reading once this many reads in a row agree (default 1),
//                   so a hand moving over the board never lands on it.
// Outputs applied (loaded this read), blocked (held back: something over the board),
// strayCells and steadyReads; whiteCount/blackCount are always the board as loaded.
plugin.Step("setBoard", (ctx, p) =>
{
    var turn = p.GetString("turn", "black");
    StepResult result;
    if (p.Has("whiteCells") || p.Has("blackCells"))
    {
        var (white, black, turns) = ReadCamera(p);
        result = game.SetBoardFromCamera(white, black, turns, turn,
            p.GetInt("settleReads", 1), p.GetInt("maxStrayCells", -1));
    }
    else
        result = game.SetBoard(
            ReadBoolList(p, "white"), ReadBoolList(p, "black"),
            ReadBoolList(p, "whiteKings"), ReadBoolList(p, "blackKings"), turn);
    game.Publish(ctx);
    return Task.FromResult(result);
});

// Apply the opponent's (human's) move by its from/to squares. Outputs legal=false
// instead of failing, so the program can re-prompt; a multi-jump only needs its
// start and end squares (the engine fills in the forced path and captures).
plugin.Step("applyMove", (ctx, p) =>
{
    var result = game.ApplyHumanMove(p.GetInt("from", -1), p.GetInt("to", -1));
    game.Publish(ctx);
    return Task.FromResult(result);
});

// Compare a camera view of the board with the tracked game and work out the move the
// human (the side to move) made, without applying it. whiteCells/blackCells are the
// per-cell results of a vision tool gridded 8x8 (RunVision cells records with
// row/col/index/value/passed, or plain numbers / booleans, row-major).
// status: "same" (board unchanged), "move" (matches exactly one legal move — from/to/captured
// are set; confirm it over a few reads, then applyMove; a jump is also matched when its captured
// pieces are still on the board, so the robot can clear them afterwards), or "unclear" (changed
// but matches no single legal move: mid-move, a hand over the board, or an illegal move).
// orientation: where black's side (board row 0) is in the camera grid — normal (top),
// rotated90 (right), rotated180 (bottom), rotated270 (left).
// bothMeans: what a cell that reads as both colours is — "white" (default: black pieces
// found by their texture, which white pieces also have), "black", or "neither".
plugin.Step("detectMove", (ctx, p) =>
{
    var (white, black, turns) = ReadCamera(p);
    return Task.FromResult(game.DetectMove(white, black, turns));
});

// Compute and apply the robot's move for the side to move. Outputs the squares the
// arm must act on: fromSquare/toSquare, the full path (multi-jumps), captured
// squares to clear, whether the piece was crowned, and game-over/winner.
plugin.Step("aiMove", async (ctx, p) =>
{
    int depth = p.Has("maxDepth") ? p.GetInt("maxDepth") : game.MaxDepth;
    int budget = p.Has("timeBudgetMs") ? p.GetInt("timeBudgetMs") : game.TimeBudgetMs;
    ctx.Progress("thinking…");
    // The search is CPU-bound; run it off the request-dispatch thread so progress
    // and cancellation stay responsive.
    var result = await Task.Run(() => game.AiMove(depth, budget));
    game.Publish(ctx);
    ctx.Log(result["moveText"]?.ToString() ?? "ai move");
    return result;
});

// ── Expression functions ──────────────────────────────────────────────────────

// $checkers piece code at a square: 0 empty, +1 white man, +2 white king,
// -1 black man, -2 black king.
plugin.Function("pieceAt", (ctx, args) =>
    Task.FromResult(game.PieceAt(args.Length > 0 ? (int)Math.Round(args[0]) : -1)));

// Number of legal moves for the side to move (0 = game over).
plugin.Function("legalMoves", (ctx, args) => Task.FromResult((double)game.LegalMoveCount()));

// ── UI actions (from ui/index.html via the plugin UI socket) ───────────────────

// New game. The GUI always plays black-first unless told otherwise.
plugin.OnAction("newGame", (ctx, args) =>
{
    game.NewGame(ReadString(args, "firstPlayer", "black"));
    game.Publish(ctx);
    return Task.FromResult<object?>(game.BuildView());
});

// Tap a square: select a piece, move to a legal target, or deselect.
plugin.OnAction("tapSquare", (ctx, args) =>
{
    game.Tap(ReadInt(args, "index", ReadInt(args, "square", -1)));
    game.Publish(ctx);
    return Task.FromResult<object?>(game.BuildView());
});

// Let the robot (the side to move) pick and play its move.
plugin.OnAction("aiMove", async (ctx, args) =>
{
    await game.AiMoveForUi(ctx);
    return (object?)game.BuildView();
});

// ── Live properties ($checkers.*) + board view ─────────────────────────────────

plugin.Background(async (ctx, ct) =>
{
    while (!ct.IsCancellationRequested)
    {
        game.Publish(ctx);
        await Task.Delay(500, ct);
    }
});

await plugin.RunAsync();
return 0;

// Reads an int field from a UI-action args object, tolerating numeric strings.
static int ReadInt(JsonElement args, string key, int fallback)
{
    if (args.ValueKind != JsonValueKind.Object || !args.TryGetProperty(key, out var v)) return fallback;
    return v.ValueKind switch
    {
        JsonValueKind.Number => v.TryGetInt32(out var n) ? n : (int)Math.Round(v.GetDouble()),
        JsonValueKind.String => int.TryParse(v.GetString(), out var n) ? n : fallback,
        _ => fallback,
    };
}

static string ReadString(JsonElement args, string key, string fallback) =>
    args.ValueKind == JsonValueKind.Object && args.TryGetProperty(key, out var v) && v.ValueKind == JsonValueKind.String
        ? v.GetString() ?? fallback : fallback;

// Reads a 32-element boolean list param, tolerating numeric 1/0 as well as true/false.
static bool[] ReadBoolList(StepParams p, string key)
{
    var outList = new bool[CheckersEngine.BoardSquares];
    if (!p.TryGet(key, out var v) || v.ValueKind != JsonValueKind.Array) return outList;
    int i = 0;
    foreach (var e in v.EnumerateArray())
    {
        if (i >= outList.Length) break;
        outList[i++] = e.ValueKind switch
        {
            JsonValueKind.True => true,
            JsonValueKind.False => false,
            JsonValueKind.Number => e.GetDouble() != 0,
            JsonValueKind.String => e.GetString() is var s && (s == "1" || string.Equals(s, "true", StringComparison.OrdinalIgnoreCase)),
            _ => false,
        };
    }
    return outList;
}

// The camera params detectMove and setBoard share: whiteCells/blackCells (8x8 vision cells),
// minCoverage, orientation (where black's side is in the camera grid) and bothMeans (what a
// cell read as both colours is). Returns the per-cell colours with bothMeans resolved.
static (bool[] White, bool[] Black, int QuarterTurns) ReadCamera(StepParams p)
{
    double minCoverage = p.GetDouble("minCoverage", 0);   // 0 = use each tool's own pass/fail
    var white = ReadCells(p, "whiteCells", minCoverage);
    var black = ReadCells(p, "blackCells", minCoverage);
    int turns = p.GetString("orientation", "normal").Trim().ToLowerInvariant() switch
    {
        "rotated90" => 1, "rotated180" => 2, "rotated270" => 3, _ => 0,
    };
    switch (p.GetString("bothMeans", "white").Trim().ToLowerInvariant())
    {
        case "black":   for (int i = 0; i < 64; i++) if (black[i]) white[i] = false; break;
        case "neither": for (int i = 0; i < 64; i++) if (white[i] && black[i]) white[i] = black[i] = false; break;
        default:        for (int i = 0; i < 64; i++) if (white[i]) black[i] = false; break;
    }
    return (white, black, turns);
}

// Reads an 8x8 row-major cell list into "occupied" per 8x8 cell index. A record cell
// ({row, col, index, value, passed}) counts when it passed its vision tool's limit (each
// colour has its own tuned range), or, when minCoverage > 0, when its value reaches that;
// plain numbers are values (or 0/1 flags when minCoverage is 0) and booleans are as-is.
static bool[] ReadCells(StepParams p, string key, double minCoverage)
{
    var cells = new bool[64];
    if (!p.TryGet(key, out var v) || v.ValueKind != JsonValueKind.Array) return cells;
    int i = 0;
    foreach (var e in v.EnumerateArray())
    {
        int idx = i++;
        bool on;
        switch (e.ValueKind)
        {
            case JsonValueKind.Object:
                if (e.TryGetProperty("index", out var ix) && ix.ValueKind == JsonValueKind.Number) idx = ix.GetInt32();
                else if (e.TryGetProperty("row", out var r) && e.TryGetProperty("col", out var c)
                         && r.ValueKind == JsonValueKind.Number && c.ValueKind == JsonValueKind.Number)
                    idx = r.GetInt32() * 8 + c.GetInt32();
                on = minCoverage > 0 && e.TryGetProperty("value", out var val) && val.ValueKind == JsonValueKind.Number
                    ? val.GetDouble() >= minCoverage
                    // A RunVision cells list stores passed as 1/0 (program values are numbers).
                    : e.TryGetProperty("passed", out var ps) && (ps.ValueKind == JsonValueKind.True
                        || (ps.ValueKind == JsonValueKind.Number && ps.GetDouble() != 0));
                break;
            case JsonValueKind.Number: on = minCoverage > 0 ? e.GetDouble() >= minCoverage : e.GetDouble() != 0; break;
            case JsonValueKind.True:   on = true; break;
            default:                   on = false; break;
        }
        if (idx >= 0 && idx < 64) cells[idx] = on;
    }
    return cells;
}

/// <summary>One tracked game. All mutation goes through the lock because the session
/// dispatches inbound step/function requests on the thread pool.</summary>
sealed class Game
{
    private readonly object _gate = new();
    private Board _board = Board.Starting();
    private Side _turn = Side.Black;
    private int _lastFrom = -1, _lastTo = -1;
    private int _selected = -1;       // square the GUI has selected (-1 none)
    private bool _thinking;           // the robot is searching for its move
    private string _moveText = "New game — black to move";

    public int MaxDepth { get; private set; } = 8;
    public int TimeBudgetMs { get; private set; } = 1000;

    public void Configure(JsonElement cfg)
    {
        lock (_gate)
        {
            if (cfg.ValueKind == JsonValueKind.Object)
            {
                if (cfg.TryGetProperty("maxDepth", out var d) && d.ValueKind == JsonValueKind.Number)
                    MaxDepth = Math.Clamp(d.GetInt32(), 1, 20);
                if (cfg.TryGetProperty("timeBudgetMs", out var t) && t.ValueKind == JsonValueKind.Number)
                    TimeBudgetMs = Math.Clamp(t.GetInt32(), 50, 20000);
            }
        }
    }

    public StepResult NewGame(string firstPlayer)
    {
        lock (_gate)
        {
            _board = Board.Starting();
            _turn = ParseSide(firstPlayer, Side.Black);
            _lastFrom = _lastTo = -1;
            _selected = -1;
            _thinking = false;
            _moveText = $"New game — {Name(_turn)} to move";
            return new StepResult
            {
                ["moveText"] = _moveText,
                ["turn"] = (double)(int)_turn,
                ["gameOver"] = false,
            };
        }
    }

    public StepResult SetBoard(bool[] white, bool[] black, bool[] whiteKings, bool[] blackKings, string turn)
    {
        lock (_gate)
        {
            _board = CheckersEngine.BoardFromLists(white, black, whiteKings, blackKings);
            _turn = ParseSide(turn, _turn);
            _lastFrom = _lastTo = -1;
            var (over, winner) = Result(_board, _turn);
            return new StepResult
            {
                ["legal"] = CheckersEngine.LegalMoves(_board, _turn).Count > 0,
                ["whiteCount"] = (double)CheckersEngine.PopCount(_board.White),
                ["blackCount"] = (double)CheckersEngine.PopCount(_board.Black),
                ["turn"] = (double)(int)_turn,
                ["gameOver"] = over,
                ["winner"] = winner,
            };
        }
    }

    /// <summary>SetBoard from a camera reading: per-cell colours of the 8x8 camera grid (bothMeans
    /// already resolved), mapped to the 32 dark squares. The camera cannot tell kings, so all are men.
    /// The reading is held back (the board keeps its last one) while more than maxStray light squares
    /// show something — a hand over the board — or until settleReads reads in a row agree.</summary>
    public StepResult SetBoardFromCamera(bool[] whiteCells, bool[] blackCells, int quarterTurns, string turn,
        int settleReads = 1, int maxStray = -1)
    {
        var white = new bool[CheckersEngine.BoardSquares];
        var black = new bool[CheckersEngine.BoardSquares];
        uint seenW = 0, seenB = 0;
        for (int sq = 0; sq < CheckersEngine.BoardSquares; sq++)
        {
            var (r, c) = CameraCell(CheckersEngine.IdxToRc(sq), quarterTurns);
            white[sq] = whiteCells[r * 8 + c];
            black[sq] = blackCells[r * 8 + c] && !white[sq];
            if (white[sq]) seenW |= 1u << sq;
            if (black[sq]) seenB |= 1u << sq;
        }
        int stray = 0;
        for (int r = 0; r < 8; r++)
            for (int c = 0; c < 8; c++)
            {
                if ((r + c) % 2 != 0) continue;   // dark square: pieces belong there
                var (cr, cc) = CameraCell((r, c), quarterTurns);
                if (whiteCells[cr * 8 + cc] || blackCells[cr * 8 + cc]) stray++;
            }

        bool blocked, apply;
        int steady;
        lock (_gate)
        {
            // Pieces spill into the light squares beside them (perspective, tall white pieces), so
            // a hand is light squares lit beyond what the loaded board itself showed.
            blocked = maxStray >= 0 && _baseStray >= 0 && stray > _baseStray + maxStray;
            if (blocked) _steadyReads = 0;
            else if (_steadyReads > 0 && seenW == _steadyW && seenB == _steadyB) _steadyReads++;
            else { _steadyW = seenW; _steadyB = seenB; _steadyReads = 1; }
            steady = _steadyReads;
            // A reading of the board already loaded has nothing to confirm.
            bool same = seenW == _board.White && seenB == _board.Black;
            apply = !blocked && (same || steady >= Math.Max(1, settleReads));
            // A new board resets the spill baseline; the same board only ever lowers it, so a hand
            // creeping in a square or two per read can't raise it.
            if (apply) _baseStray = same && _baseStray >= 0 ? Math.Min(_baseStray, stray) : stray;
        }

        StepResult result;
        if (apply)
        {
            var none = new bool[CheckersEngine.BoardSquares];
            result = SetBoard(white, black, none, none, turn);
            lock (_gate) _moveText = $"Camera sees {result["whiteCount"]} white, {result["blackCount"]} black ({DateTime.Now:HH:mm:ss})";
        }
        else
        {
            lock (_gate)
            {
                _moveText = blocked
                    ? "Hand over the board — waiting for it to clear"
                    : $"Board changed — checking ({steady}/{settleReads})";
                result = new StepResult
                {
                    ["whiteCount"] = (double)CheckersEngine.PopCount(_board.White),
                    ["blackCount"] = (double)CheckersEngine.PopCount(_board.Black),
                    ["turn"] = (double)(int)_turn,
                };
            }
        }
        result["applied"] = apply;
        result["blocked"] = blocked;
        result["strayCells"] = (double)stray;
        result["steadyReads"] = (double)steady;
        return result;
    }

    // The camera reading setBoard is waiting to see repeat (settleReads), and how often it has.
    private uint _steadyW, _steadyB;
    private int _steadyReads;
    // Light squares lit when the board was loaded (piece spill); -1 = nothing loaded from the camera yet.
    private int _baseStray = -1;

    public StepResult ApplyHumanMove(int from, int to)
    {
        lock (_gate)
        {
            var mv = CheckersEngine.FindMove(_board, _turn, from, to);
            if (mv == null)
                return new StepResult { ["legal"] = false, ["moveText"] = $"Illegal move {from}->{to}", ["gameOver"] = false };
            return Commit(mv, legal: true);
        }
    }

    /// <summary>
    /// Where board square (row, col) lands in the camera's 8x8 grid (row 0 = the top of the
    /// image). Board row 0 is black's side: normal = black's side at the top, rotated90 = at
    /// the right (players sit left and right of the camera), rotated180 = at the bottom,
    /// rotated270 = at the left.
    /// </summary>
    internal static (int Row, int Col) CameraCell((int Row, int Col) board, int quarterTurns) => (quarterTurns & 3) switch
    {
        1 => (board.Col, 7 - board.Row),
        2 => (7 - board.Row, 7 - board.Col),
        3 => (7 - board.Col, board.Row),
        _ => board,
    };

    /// <summary>Works out which legal move of the side to move turns the tracked board into the
    /// observed one (piece colours per dark square). Does not apply it.</summary>
    public StepResult DetectMove(bool[] whiteCells, bool[] blackCells, int quarterTurns)
    {
        uint seenW = 0, seenB = 0;
        for (int sq = 0; sq < CheckersEngine.BoardSquares; sq++)
        {
            var (r, c) = CameraCell(CheckersEngine.IdxToRc(sq), quarterTurns);
            int cell = r * 8 + c;
            // The step resolves cells read as both colours (bothMeans); any left are ambiguous.
            if (whiteCells[cell] && !blackCells[cell]) seenW |= 1u << sq;
            if (blackCells[cell] && !whiteCells[cell]) seenB |= 1u << sq;
        }
        lock (_gate)
        {
            int changed = CheckersEngine.PopCount((seenW ^ _board.White) | (seenB ^ _board.Black));
            var result = new StepResult
            {
                ["changed"] = (double)changed,
                ["whiteSeen"] = (double)CheckersEngine.PopCount(seenW),
                ["blackSeen"] = (double)CheckersEngine.PopCount(seenB),
                ["turn"] = (double)(int)_turn,
                ["from"] = -1.0,
                ["to"] = -1.0,
                ["captured"] = Array.Empty<double>(),
            };
            if (changed == 0)
            {
                result["status"] = "same";
                result["moveText"] = "Board unchanged";
                return result;
            }
            var legal = CheckersEngine.LegalMoves(_board, _turn);
            // Exact: the board already shows the move's result (any jumped pieces removed).
            var match = legal.FirstOrDefault(m =>
            {
                var after = CheckersEngine.ApplyMove(_board, _turn, m);
                return after.White == seenW && after.Black == seenB;
            });
            // Lenient: a jump whose captured pieces are still on the board — the robot clears them
            // afterwards. The mover's pieces must match exactly, every surviving foe piece must be
            // present, and any extra foe pieces must sit exactly on this move's captured squares.
            // Accepted only when a single legal jump fits, so it never guesses between two moves.
            if (match == null)
            {
                var fits = legal.Where(m =>
                {
                    if (m.Captures.Length == 0) return false;   // only jumps leave extra pieces
                    var after = CheckersEngine.ApplyMove(_board, _turn, m);
                    uint capMask = 0;
                    foreach (var cap in m.Captures) capMask |= 1u << cap;
                    return _turn == Side.White
                        ? seenW == after.White && (after.Black & ~seenB) == 0 && (seenB & ~after.Black & ~capMask) == 0
                        : seenB == after.Black && (after.White & ~seenW) == 0 && (seenW & ~after.White & ~capMask) == 0;
                }).ToList();
                if (fits.Count == 1) match = fits[0];
            }
            if (match == null)
            {
                result["status"] = "unclear";
                result["moveText"] = $"{changed} square(s) differ but no legal {Name(_turn)} move matches";
                return result;
            }
            result["status"] = "move";
            result["from"] = (double)match.Start;
            result["to"] = (double)match.End;
            result["captured"] = match.Captures.Select(x => (double)x).ToArray();
            result["moveText"] = $"Saw {Name(_turn)} {string.Join(match.Captures.Length > 0 ? "x" : "->", match.Path)}";
            return result;
        }
    }

    public StepResult AiMove(int maxDepth, int timeBudgetMs)
    {
        lock (_gate)
        {
            var choice = CheckersEngine.ChooseMove(_board, _turn, maxDepth, timeBudgetMs);
            if (choice.Move == null)
            {
                var (_, winner) = Result(_board, _turn);
                return new StepResult
                {
                    ["legal"] = false,
                    ["gameOver"] = true,
                    ["winner"] = winner,
                    ["moveText"] = $"{Name(_turn)} has no move — {winner} wins",
                    ["fromSquare"] = -1.0,
                    ["toSquare"] = -1.0,
                };
            }
            return Commit(choice.Move, legal: true);
        }
    }

    // Applies a move for the side to move, flips the turn, and builds the output.
    private StepResult Commit(Move mv, bool legal)
    {
        Side mover = _turn;
        _board = CheckersEngine.ApplyMove(_board, mover, mv);
        _turn = CheckersEngine.Opponent(mover);
        _lastFrom = mv.Start;
        _lastTo = mv.End;
        _selected = -1;

        var (over, winner) = Result(_board, _turn);
        string text = BuildMoveText(mover, mv, over, winner);
        _moveText = text;
        return new StepResult
        {
            ["legal"] = legal,
            ["fromSquare"] = (double)mv.Start,
            ["toSquare"] = (double)mv.End,
            ["path"] = mv.Path.Select(x => (double)x).ToArray(),
            ["captured"] = mv.Captures.Select(x => (double)x).ToArray(),
            ["kinged"] = mv.Promoted,
            ["gameOver"] = over,
            ["winner"] = winner,
            ["moveText"] = text,
        };
    }

    public double PieceAt(int sq)
    {
        if (sq < 0 || sq >= CheckersEngine.BoardSquares) return 0;
        lock (_gate)
        {
            uint bit = 1u << sq;
            if ((_board.White & bit) != 0) return (_board.WhiteKings & bit) != 0 ? 2 : 1;
            if ((_board.Black & bit) != 0) return (_board.BlackKings & bit) != 0 ? -2 : -1;
            return 0;
        }
    }

    public int LegalMoveCount()
    {
        lock (_gate) return CheckersEngine.LegalMoves(_board, _turn).Count;
    }

    public void PublishProperties(IPluginContext ctx)
    {
        bool over; string winner; int turn, w, b, lf, lt;
        lock (_gate)
        {
            (over, winner) = Result(_board, _turn);
            turn = (int)_turn;
            w = CheckersEngine.PopCount(_board.White);
            b = CheckersEngine.PopCount(_board.Black);
            lf = _lastFrom; lt = _lastTo;
        }
        ctx.SetProperties(new
        {
            gameOver = over ? 1 : 0,
            turn,                 // 0 = white, 1 = black
            whiteCount = w,
            blackCount = b,
            lastFrom = lf,
            lastTo = lt,
        });
    }

    // ── Custom GUI (ui/index.html via the plugin UI socket) ───────────────────

    /// <summary>Pushes both the numeric properties and the rich board view to the GUI.</summary>
    public void Publish(IPluginContext ctx)
    {
        PublishProperties(ctx);
        ctx.SetView(BuildView());
    }

    /// <summary>The full board view the GUI renders (piece codes, whose turn, selection, legal targets, …).</summary>
    public object BuildView()
    {
        lock (_gate)
        {
            var board = new int[CheckersEngine.BoardSquares];
            for (int i = 0; i < board.Length; i++)
            {
                uint bit = 1u << i;
                board[i] = (_board.White & bit) != 0 ? ((_board.WhiteKings & bit) != 0 ? 2 : 1)
                         : (_board.Black & bit) != 0 ? ((_board.BlackKings & bit) != 0 ? -2 : -1)
                         : 0;
            }
            var legal = CheckersEngine.LegalMoves(_board, _turn);
            var movable = legal.Select(m => m.Start).Distinct().OrderBy(x => x).ToArray();
            var targets = _selected >= 0
                ? legal.Where(m => m.Start == _selected).Select(m => m.End).Distinct().OrderBy(x => x).ToArray()
                : Array.Empty<int>();
            var (over, winner) = Result(_board, _turn);
            return new
            {
                board,
                turn = (int)_turn,            // 0 = white, 1 = black
                whiteCount = CheckersEngine.PopCount(_board.White),
                blackCount = CheckersEngine.PopCount(_board.Black),
                selected = _selected,
                legalTargets = targets,
                movable,
                gameOver = over,
                winner,
                lastFrom = _lastFrom,
                lastTo = _lastTo,
                thinking = _thinking,
                moveText = _moveText,
            };
        }
    }

    /// <summary>A tap on a board square from the GUI: select a movable piece, move to a legal
    /// target (the engine fills in a multi-jump's path), or deselect.</summary>
    public void Tap(int sq)
    {
        lock (_gate)
        {
            if (_thinking) return;
            var (over, _) = Result(_board, _turn);
            if (over || sq < 0 || sq >= CheckersEngine.BoardSquares) { _selected = -1; return; }

            var legal = CheckersEngine.LegalMoves(_board, _turn);
            if (_selected >= 0)
            {
                var mv = legal.FirstOrDefault(m => m.Start == _selected && m.End == sq);
                if (mv != null) { Commit(mv, legal: true); return; }   // applies + flips + clears selection
                if (sq == _selected) { _selected = -1; return; }
            }
            // (Re)select only a piece that actually has a legal move this turn.
            _selected = legal.Any(m => m.Start == sq) ? sq : -1;
        }
    }

    /// <summary>Compute and play the robot's move, pushing a "thinking" view first so the GUI can
    /// show progress. The search runs off the lock; the result is discarded if the board changed.</summary>
    public async Task AiMoveForUi(IPluginContext ctx)
    {
        Board snap; Side side; int depth, budget;
        lock (_gate)
        {
            if (_thinking) return;
            var (over, _) = Result(_board, _turn);
            if (over) return;
            _thinking = true;
            _selected = -1;
            snap = _board; side = _turn; depth = MaxDepth; budget = TimeBudgetMs;
        }
        Publish(ctx);   // show "thinking…"
        var choice = await Task.Run(() => CheckersEngine.ChooseMove(snap, side, depth, budget));
        lock (_gate)
        {
            _thinking = false;
            bool unchanged = snap.White == _board.White && snap.Black == _board.Black
                          && snap.WhiteKings == _board.WhiteKings && snap.BlackKings == _board.BlackKings && _turn == side;
            if (unchanged && choice.Move != null) Commit(choice.Move, legal: true);
        }
        Publish(ctx);
    }

    // Side to move with no legal move has lost; also a side with no pieces loses.
    private static (bool over, string winner) Result(Board board, Side toMove)
    {
        if (CheckersEngine.PopCount(board.White) == 0) return (true, "black");
        if (CheckersEngine.PopCount(board.Black) == 0) return (true, "white");
        if (CheckersEngine.LegalMoves(board, toMove).Count == 0)
            return (true, Name(CheckersEngine.Opponent(toMove)));
        return (false, "");
    }

    private static string BuildMoveText(Side mover, Move mv, bool over, string winner)
    {
        string sep = mv.Captures.Length > 0 ? "x" : "->";
        string path = string.Join(sep, mv.Path);
        string s = $"{Name(mover)} {path}";
        if (mv.Captures.Length > 0) s += $" (took {string.Join(",", mv.Captures)})";
        if (mv.Promoted) s += " — king!";
        if (over) s += $" — {winner} wins";
        return s;
    }

    private static Side ParseSide(string s, Side fallback) => s?.Trim().ToLowerInvariant() switch
    {
        "white" => Side.White,
        "black" => Side.Black,
        _ => fallback,
    };

    private static string Name(Side s) => s == Side.White ? "white" : "black";
}

// Offline check of DetectMove's two match tiers (run: Checkers.exe --selftest). White (to move)
// has a single forced jump 16->9 over the black man at 13. Verifies the jump is recognised both
// when the captured piece has been removed (exact) and when it is still on the board (lenient —
// the robot clears it afterwards), and that an unchanged board still reads "same".
static class CheckersSelfTest
{
    static bool[] Men(params int[] sq)   { var a = new bool[32]; foreach (var s in sq) a[s] = true; return a; }
    static bool[] Cells(params int[] sq) { var a = new bool[64]; foreach (var s in sq) { var (r, c) = CheckersEngine.IdxToRc(s); a[r * 8 + c] = true; } return a; }

    public static int Run()
    {
        int fails = 0;
        void Check(bool ok, string name) { Console.WriteLine((ok ? "ok   " : "FAIL ") + name); if (!ok) fails++; }
        string? Status(StepResult r) => r["status"] as string;

        var g = new Game();

        // Jump played, captured black (13) still physically on the board.
        g.SetBoard(Men(16), Men(13), new bool[32], new bool[32], "white");
        var left = g.DetectMove(Cells(9), Cells(13), 0);
        var cap  = (double[])left["captured"]!;
        Check(Status(left) == "move" && (double)left["from"]! == 16 && (double)left["to"]! == 9, "jump detected with captured piece still on the board");
        Check(cap.Length == 1 && cap[0] == 13, "captured square reported (13)");

        // Same jump, captured piece already removed (the exact match).
        g.SetBoard(Men(16), Men(13), new bool[32], new bool[32], "white");
        var gone = g.DetectMove(Cells(9), Cells(), 0);
        Check(Status(gone) == "move" && (double)gone["to"]! == 9, "jump detected with captured piece removed");

        // Nothing moved.
        g.SetBoard(Men(16), Men(13), new bool[32], new bool[32], "white");
        Check(Status(g.DetectMove(Cells(16), Cells(13), 0)) == "same", "unchanged board reads same");

        Console.WriteLine(fails == 0 ? "SELFTEST PASS" : $"SELFTEST FAIL ({fails})");
        return fails == 0 ? 0 : 1;
    }
}
