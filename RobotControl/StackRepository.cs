
namespace Controller.RobotControl.Persistence;

/// <summary>Persists named stacks to stacks.json. All behaviour lives in <see cref="JsonListRepository{T}"/>.</summary>
public class StackRepository : JsonListRepository<RobotStack>
{
    public StackRepository(string file = "stacks.json") : base(file) { }
}
