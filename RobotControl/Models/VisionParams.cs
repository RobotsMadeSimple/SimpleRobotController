using System;
using System.Collections.Generic;
using System.Linq;
using System.Text.Json;
using System.Text.Json.Serialization;

// ── Vision command params ─────────────────────────────────────────────────────

public class DeleteVisionProgramParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}

public class StartStopVisionParams
{
    [JsonPropertyName("id")] public string Id { get; set; } = "";
}


