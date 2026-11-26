using System.Text.Json;
using Controller.RobotControl.Cells;

namespace Controller.RobotControl.Commands;

/// <summary>
/// 3D cell packages (docs/cell-package.md §4): list, set the active one, delete. Upload and the
/// resolved cell.json / mesh files are HTTP (<see cref="Hosting.CellEndpoints"/>).
/// </summary>
internal sealed class CellCommands
{
    private readonly Func<CellRepository> _cells;

    public CellCommands(RobotController robot) : this(() => robot.Cells) { }

    /// <summary>For tests: commands over a standalone repository.</summary>
    internal CellCommands(CellRepository cells) : this(() => cells) { }

    private CellCommands(Func<CellRepository> cells) => _cells = cells;

    public void Register(CommandDispatcher d)
    {
        d.Add("GetCells",      GetCells);
        d.Add("SetActiveCell", SetActiveCell);
        d.Add("DeleteCell",    DeleteCell);
    }

    private object? GetCells(CommandMessage msg)
    {
        var cells = _cells();
        return new { cells = cells.List().Select(ToJson).ToList(), activeCellId = cells.ActiveId };
    }

    private object? SetActiveCell(CommandMessage msg)
    {
        var p = Params(msg);
        string? id = null;
        if (p.TryGetProperty("id", out var el))
        {
            if (el.ValueKind == JsonValueKind.String) id = el.GetString();
            else if (el.ValueKind != JsonValueKind.Null) return Fail("badParams", "id must be a string or null");
        }
        if (!_cells().SetActive(id)) return Fail("unknownCell", $"No cell package '{id}'");
        return new { activeCellId = _cells().ActiveId };
    }

    private object? DeleteCell(CommandMessage msg)
    {
        var p = Params(msg);
        string? id = p.TryGetProperty("id", out var el) && el.ValueKind == JsonValueKind.String ? el.GetString() : null;
        if (string.IsNullOrEmpty(id)) return Fail("badParams", "id is required");
        return _cells().Delete(id) ? null : Fail("unknownCell", $"No cell package '{id}'");
    }

    /// <summary>The summary shape shared by GetCells and <c>GET /cells</c>.</summary>
    internal static object ToJson(CellSummary s) => new
    {
        id            = s.Id,
        name          = s.Name,
        updatedUnixMs = s.UpdatedUnixMs,
        active        = s.Active,
        tools         = s.Tools,
        fixtures      = s.Fixtures,
    };

    private static object Fail(string code, string message) => new { ok = false, error = code, message };

    private static readonly JsonElement EmptyObject = JsonDocument.Parse("{}").RootElement.Clone();

    private static JsonElement Params(CommandMessage msg) =>
        msg.Params is { ValueKind: JsonValueKind.Object } p ? p : EmptyObject;
}
