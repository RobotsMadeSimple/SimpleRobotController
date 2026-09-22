using System.Text.Json;
using System.Text.Json.Serialization;

namespace Controller.RobotControl.Commands;

/// <summary>
/// JSON plumbing shared by every command handler: parameter deserialisation and
/// the serializer options for payloads the app receives as JSON strings.
/// </summary>
internal static class CommandJson
{
    // Deserialisation options — handles string enums and camelCase from the client.
    private static readonly JsonSerializerOptions ParamOptions = new()
    {
        Converters = { new JsonStringEnumConverter() },
        PropertyNameCaseInsensitive = true
    };

    // Serialisation options for payloads the app receives as JSON strings.
    public static readonly JsonSerializerOptions CamelCase = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
    };

    public static readonly JsonSerializerOptions CamelCaseWithEnums = new()
    {
        Converters           = { new JsonStringEnumConverter() },
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
    };

    // No naming policy (PascalCase output) — the grid and stack lists go out this
    // way. PropertyNameCaseInsensitive only affects reading; it is kept so the
    // options are exactly what those handlers have always used.
    public static readonly JsonSerializerOptions PascalCase = new()
    {
        PropertyNameCaseInsensitive = true,
    };

    public static T LoadParams<T>(CommandMessage msg)
    {
        if (msg.Params == null)
            throw new InvalidOperationException("Command has no params");

        return msg.Params.Value.Deserialize<T>(ParamOptions)
            ?? throw new InvalidOperationException("Command params were null");
    }
}
