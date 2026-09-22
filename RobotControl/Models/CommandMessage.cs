using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

public class CommandMessage
{
    public string Type { get; set; } = default!;
    public string Id { get; set; } = default!;
    public string Command { get; set; } = default!;
    public JsonElement? Params { get; set; }
}


