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
    game.PublishProperties(ctx);
    ctx.Log(result["moveText"]?.ToString() ?? "new game");
    return Task.FromResult(result);
});

// Load an externally-observed board (e.g. from the vision pipeline) as the truth,
// replacing the tracked game. Lists are 32-element boolean (or 0/1) arrays.
plugin.Step("setBoard", (ctx, p) =>
{
    var result = game.SetBoard(
        ReadBoolList(p, "white"), ReadBoolList(p, "black"),
        ReadBoolList(p, "whiteKings"), ReadBoolList(p, "blackKings"),
        p.GetString("turn", "black"));
    game.PublishProperties(ctx);
    return Task.FromResult(result);
});

// Apply the opponent's (human's) move by its from/to squares. Outputs legal=false
// instead of failing, so the program can re-prompt; a multi-jump only needs its
// start and end squares (the engine fills in the forced path and captures).
plugin.Step("applyMove", (ctx, p) =>
{
    var result = game.ApplyHumanMove(p.GetInt("from", -1), p.GetInt("to", -1));
    game.PublishProperties(ctx);
    return Task.FromResult(result);
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
    game.PublishProperties(ctx);
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
