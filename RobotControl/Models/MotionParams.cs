using System.Text.Json.Serialization;

namespace Controller.RobotControl;

// ── Motion command params ─────────────────────────────────────────────────────

/// <summary>
/// Declares one logical joint to be at a known value right now, with no motion — manual
/// homing of a single joint. Joint index: 0=J1/X, 1=Horizontal/Y, 2=Vertical/Z, 3=J4/RZ.
/// Zeroing is just <see cref="Value"/> = 0.
/// </summary>
public class SetJointPositionParams
{
    [JsonPropertyName("joint")] public int    Joint { get; set; }
    [JsonPropertyName("value")] public double Value { get; set; }
}
