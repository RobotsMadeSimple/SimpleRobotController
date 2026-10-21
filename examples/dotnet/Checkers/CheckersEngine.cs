using System.Diagnostics;

namespace Checkers;

// American (English draughts) checkers rules engine and alpha-beta search.
// Pure logic — no I/O, no robot, no plugin protocol. Ported faithfully from the
// Python reference engine (CheckersServer/checkers.py) so the two stay in sync.
//
// Board model
// -----------
// 8x8 board, only the 32 dark squares are playable, indexed 0..31: index 0 is the
// leftmost dark square of row 0, index 31 the rightmost of row 7. A 32-element
// list maps one-to-one onto the physical squares the arm can reach.
//
//     row 0   .  0  .  1  .  2  .  3
//     row 1   4  .  5  .  6  .  7  .
//     row 2   .  8  .  9  . 10  . 11
//     ...
//
// Black occupies rows 0-2 and advances down the board (increasing row); white
// occupies rows 5-7 and advances up. Black promotes on row 7, white on row 0.

public enum Side { White, Black }

/// <summary>One turn. <see cref="Path"/> is the squares the piece occupies in order,
/// starting where it began: a quiet move has two entries, a double jump three.
/// <see cref="Captures"/> are the squares of removed pieces, in the order jumped
/// (the order an arm would clear them).</summary>
public sealed record Move(int[] Path, int[] Captures, bool Promoted)
{
    public int Start => Path[0];
    public int End => Path[^1];
}

/// <summary>Immutable board as four 32-bit masks (bit i set = a piece on square i).
/// Kings are a subset of the matching colour's men mask.</summary>
public readonly struct Board
{
    public readonly uint White, Black, WhiteKings, BlackKings;

    public Board(uint white, uint black, uint whiteKings, uint blackKings)
    {
        White = white; Black = black; WhiteKings = whiteKings; BlackKings = blackKings;
    }

    public uint Pieces(Side p) => p == Side.White ? White : Black;
    public uint Kings(Side p) => p == Side.White ? WhiteKings : BlackKings;
    public uint Occupied => White | Black;
    public bool IsKing(Side p, int sq) => (Kings(p) & (1u << sq)) != 0;

    /// <summary>The standard opening: twelve men each, rows 0-2 (black) and 5-7 (white).</summary>
    public static Board Starting()
    {
        uint black = 0, white = 0;
        for (int i = 0; i < 12; i++) black |= 1u << i;
        for (int i = 20; i < 32; i++) white |= 1u << i;
        return new Board(white, black, 0, 0);
    }
}

public static class CheckersEngine
{
    public const int BoardSquares = 32;

    private const int ManValue = 100;
    private const int KingValue = 175;

    // Row a man must reach to be crowned, and the row direction a man may move
    // (kings ignore this), per side.
    private static int KingRow(Side p) => p == Side.White ? 0 : 7;
    private static int Forward(Side p) => p == Side.White ? -1 : +1;

    public static Side Opponent(Side p) => p == Side.White ? Side.Black : Side.White;

    // ── Geometry ────────────────────────────────────────────────────────────

    /// <summary>Playable-square index → (row, col) on the full 8x8 board.</summary>
    public static (int r, int c) IdxToRc(int i)
    {
        int r = i / 4;
        // Dark squares shift by one column on alternating rows.
        int c = 2 * (i % 4) + (r % 2 == 0 ? 1 : 0);
        return (r, c);
    }

    /// <summary>(row, col) → playable-square index, or -1 if off-board or a light square.</summary>
    public static int RcToIdx(int r, int c)
    {
        if (r < 0 || r >= 8 || c < 0 || c >= 8) return -1;
        if ((r + c) % 2 == 0) return -1;
        return r * 4 + c / 2;
    }

    // Precomputed neighbours: quiet steps, and (jumpedOver, landedOn) pairs.
    private static readonly int[][] Steps;
    private static readonly (int over, int land)[][] Jumps;

    static CheckersEngine()
    {
        Steps = new int[BoardSquares][];
        Jumps = new (int, int)[BoardSquares][];
        var dirs = new[] { (-1, -1), (-1, 1), (1, -1), (1, 1) };
        for (int i = 0; i < BoardSquares; i++)
        {
            var (r, c) = IdxToRc(i);
            var s = new List<int>();
            var j = new List<(int, int)>();
            foreach (var (dr, dc) in dirs)
            {
                int n = RcToIdx(r + dr, c + dc);
                if (n < 0) continue;
                s.Add(n);
                int land = RcToIdx(r + 2 * dr, c + 2 * dc);
                if (land >= 0) j.Add((n, land));
            }
            Steps[i] = s.ToArray();
            Jumps[i] = j.ToArray();
        }
    }

    private static IEnumerable<int> BitsOf(uint mask)
    {
        while (mask != 0)
        {
            int b = System.Numerics.BitOperations.TrailingZeroCount(mask);
            yield return b;
            mask &= mask - 1;
        }
    }

    public static int PopCount(uint mask) => System.Numerics.BitOperations.PopCount(mask);

    // ── Move generation ───────────────────────────────────────────────────────

    /// <summary>Every maximal jump sequence starting at <paramref name="sq"/>. Jumping is
    /// compulsory and a started sequence must be played out, so only sequences that
    /// cannot be extended are returned.</summary>
    private static void JumpSequences(Board board, Side player, int sq, bool isKing, List<Move> outMoves)
    {
        uint foe = board.Pieces(Opponent(player));
        uint occupiedSansOrigin = board.Occupied & ~(1u << sq);

        void Walk(int at, bool king, List<int> captured, List<int> path)
        {
            bool extended = false;
            foreach (var (over, land) in Jumps[at])
            {
                if (captured.Contains(over) || (foe & (1u << over)) == 0) continue;
                // The origin square is vacated the moment the piece lifts, so it does
                // not block a sequence that loops back over it.
                if ((occupiedSansOrigin & (1u << land)) != 0 || captured.Contains(land)) continue;
                if (!king && (land / 4) - (at / 4) != 2 * Forward(player)) continue;

                bool crowned = !king && (land / 4) == KingRow(player);
                extended = true;

                var nextCaptured = new List<int>(captured) { over };
                var nextPath = new List<int>(path) { land };
                if (crowned)
                    // American rules: being crowned ends the turn, even mid-sequence.
                    outMoves.Add(new Move(nextPath.ToArray(), nextCaptured.ToArray(), true));
                else
                    Walk(land, king, nextCaptured, nextPath);
            }

            if (!extended && path.Count > 1)
                outMoves.Add(new Move(path.ToArray(), captured.ToArray(), false));
        }

        Walk(sq, isKing, new List<int>(), new List<int> { sq });
    }

    /// <summary>All legal moves. Captures are compulsory in American checkers, so if any
    /// jump exists the quiet moves are not returned at all.</summary>
    public static List<Move> LegalMoves(Board board, Side player)
    {
        var jumps = new List<Move>();
        foreach (int sq in BitsOf(board.Pieces(player)))
            JumpSequences(board, player, sq, board.IsKing(player, sq), jumps);
        if (jumps.Count > 0) return jumps;

        uint occupied = board.Occupied;
        var quiet = new List<Move>();
        foreach (int sq in BitsOf(board.Pieces(player)))
        {
            bool king = board.IsKing(player, sq);
            foreach (int n in Steps[sq])
            {
                if ((occupied & (1u << n)) != 0) continue;
                if (!king && (n / 4) - (sq / 4) != Forward(player)) continue;
                bool crowned = !king && (n / 4) == KingRow(player);
                quiet.Add(new Move(new[] { sq, n }, Array.Empty<int>(), crowned));
            }
        }
        return quiet;
    }

    /// <summary>The board after <paramref name="move"/>. Assumes the move is legal.</summary>
    public static Board ApplyMove(Board board, Side player, Move move)
    {
        Side foe = Opponent(player);
        uint start = 1u << move.Start, end = 1u << move.End;

        uint mine = (board.Pieces(player) & ~start) | end;
        uint myKings = board.Kings(player);
        if ((myKings & start) != 0) myKings = (myKings & ~start) | end;
        if (move.Promoted) myKings |= end;

        uint capMask = 0;
        foreach (int c in move.Captures) capMask |= 1u << c;
        uint theirs = board.Pieces(foe) & ~capMask;
        uint theirKings = board.Kings(foe) & ~capMask;

        return player == Side.White
            ? new Board(mine, theirs, myKings, theirKings)
            : new Board(theirs, mine, theirKings, myKings);
    }

    /// <summary>Which move (if any) takes <paramref name="from"/> to <paramref name="to"/>
    /// for <paramref name="player"/>. Used to validate/apply a human move by endpoints.</summary>
    public static Move? FindMove(Board board, Side player, int from, int to)
    {
        foreach (var mv in LegalMoves(board, player))
            if (mv.Start == from && mv.End == to) return mv;
        return null;
    }

    // ── Search ──────────────────────────────────────────────────────────────

    /// <summary>Material plus a small positional nudge, from <paramref name="player"/>'s
    /// point of view. Advancement breaks ties so the engine makes progress, but is
    /// never worth trading a man for.</summary>
    public static int Evaluate(Board board, Side player)
    {
        int score = 0;
        foreach (var (who, sign) in new[] { (player, 1), (Opponent(player), -1) })
        {
            foreach (int sq in BitsOf(board.Pieces(who)))
            {
                if (board.IsKing(who, sq)) score += sign * KingValue;
                else
                {
                    int row = sq / 4;
                    int advance = who == Side.White ? (7 - row) : row;
                    score += sign * (ManValue + advance * 2);
                }
            }
        }
        return score;
    }

    private sealed class Timeout : Exception { }

    private sealed class Search
    {
        private readonly long _deadlineTicks;
        private readonly Stopwatch _clock;
        public long Nodes;

        public Search(Stopwatch clock, long deadlineTicks) { _clock = clock; _deadlineTicks = deadlineTicks; }

        public int Negamax(Board board, Side player, int depth, int alpha, int beta)
        {
            Nodes++;
            // Checked on a node count rather than every node so the clock call does not
            // dominate the search at shallow depths.
            if (Nodes % 512 == 0 && _clock.ElapsedTicks > _deadlineTicks) throw new Timeout();

            var moves = LegalMoves(board, player);
            if (moves.Count == 0)
                // No move means this side has lost — weighted by depth so the engine
                // prefers a loss that is further away and a win that is nearer.
                return -100_000 - depth;

            if (depth <= 0)
            {
                // Stopping mid-exchange misreads the position, so a forced-capture node
                // is always extended by one.
                if (moves[0].Captures.Length == 0) return Evaluate(board, player);
                depth = 1;
            }

            int best = -1_000_000;
            foreach (var mv in moves)
            {
                var child = ApplyMove(board, player, mv);
                int score = -Negamax(child, Opponent(player), depth - 1, -beta, -alpha);
                if (score > best) best = score;
                if (score > alpha) alpha = score;
                if (alpha >= beta) break;
            }
            return best;
        }
    }

    public sealed record Choice(Move? Move, int Depth, int Score, long Nodes, int MovesAvailable, string Reason);

    /// <summary>Pick a move for <paramref name="player"/> by iterative deepening, keeping the
    /// best result from the last depth that finished inside the budget. A partial depth is
    /// discarded (its move ordering is arbitrary).</summary>
    public static Choice ChooseMove(Board board, Side player, int maxDepth = 8,
        int timeBudgetMs = 1000, Random? rng = null)
    {
        rng ??= Random.Shared;
        var moves = LegalMoves(board, player);
        if (moves.Count == 0) return new Choice(null, 0, 0, 0, 0, "no legal moves");
        if (moves.Count == 1) return new Choice(moves[0], 0, 0, 0, 1, "forced");

        var clock = Stopwatch.StartNew();
        long deadlineTicks = (long)(timeBudgetMs / 1000.0 * Stopwatch.Frequency);
        var search = new Search(clock, deadlineTicks);
        var best = new Choice(moves[rng.Next(moves.Count)], 0, 0, 0, moves.Count, "random fallback");

        for (int depth = 1; depth <= maxDepth; depth++)
        {
            var scored = new List<(int score, Move mv)>();
            try
            {
                foreach (var mv in moves)
                {
                    var child = ApplyMove(board, player, mv);
                    int score = -search.Negamax(child, Opponent(player), depth - 1, -1_000_000, 1_000_000);
                    scored.Add((score, mv));
                }
            }
            catch (Timeout) { break; }

            int top = scored.Max(s => s.score);
            // Ties broken at random so repeated positions don't play identically.
            var topMoves = scored.Where(s => s.score == top).Select(s => s.mv).ToList();
            best = new Choice(topMoves[rng.Next(topMoves.Count)], depth, top, search.Nodes, moves.Count, "search");
            // Best-first ordering for the next iteration makes alpha-beta cut far more,
            // which is what buys the extra depth.
            moves = scored.OrderByDescending(s => s.score).Select(s => s.mv).ToList();
            if (Math.Abs(top) > 50_000) break; // forced win/loss found; deeper cannot improve
        }

        return best;
    }

    // ── Wire format (the four 32-element boolean lists the API speaks) ──────────

    public static uint MaskFromList(IReadOnlyList<bool>? list)
    {
        uint m = 0;
        if (list != null)
            for (int i = 0; i < list.Count && i < BoardSquares; i++)
                if (list[i]) m |= 1u << i;
        return m;
    }

    public static bool[] MaskToList(uint mask)
    {
        var a = new bool[BoardSquares];
        for (int i = 0; i < BoardSquares; i++) a[i] = (mask & (1u << i)) != 0;
        return a;
    }

    /// <summary>Kings are intersected with the matching men mask: a king flag on an empty
    /// square is a client bug, and carrying it would produce an unreachable board.</summary>
    public static Board BoardFromLists(IReadOnlyList<bool>? white, IReadOnlyList<bool>? black,
        IReadOnlyList<bool>? whiteKings, IReadOnlyList<bool>? blackKings)
    {
        uint w = MaskFromList(white), b = MaskFromList(black);
        return new Board(w, b, MaskFromList(whiteKings) & w, MaskFromList(blackKings) & b);
    }
}
