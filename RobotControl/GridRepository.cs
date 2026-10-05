
namespace Controller.RobotControl.Persistence;

/// <summary>Persists named grids to grids.json. All behaviour lives in <see cref="JsonListRepository{T}"/>.</summary>
public class GridRepository : JsonListRepository<Grid>
{
    public GridRepository(string file = "grids.json") : base(file) { }
}
