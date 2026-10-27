namespace Controller.RobotControl.Serial
{
    /// <summary>
    /// What an Arduino board told us about itself in its "ID?" handshake reply.
    ///
    /// Two reply forms are accepted:
    ///   legacy:  ID:ROBOT_NANO_001
    ///   current: ID:ROBOT_NANO_001;fw=1.1.0;caps=io,neopixel,loadcell
    ///
    /// Legacy firmware (everything flashed before 1.1.0) only sends the id, so its version and
    /// capability set are <b>unknown</b>. Unknown is deliberately permissive — <see cref="Has"/>
    /// answers true for every capability — because the boards in the field today are legacy and
    /// must keep working untouched. When the board does report its capabilities, <see cref="Has"/>
    /// is the real answer and callers refuse features the board lacks (see <see cref="FirmwareGate"/>).
    /// </summary>
    public sealed class FirmwareInfo
    {
        /// <summary>Capability names a board may report (the exact strings the firmware sends).</summary>
        public const string CapIo       = "io";        // digital IO pins
        public const string CapNeopixel = "neopixel";
        public const string CapLoadCell = "loadcell";
        public const string CapStepper  = "stepper";

        /// <summary>First firmware version that answers the extended handshake.</summary>
        public const string FirstReportingVersion = "1.1.0";

        /// <summary>The state before any handshake, and of any legacy board.</summary>
        public static readonly FirmwareInfo Unknown = new("", null);

        /// <summary>Firmware version as reported ("major.minor.patch"); "" when unknown.</summary>
        public string Version { get; }

        /// <summary>Capability names (lower-case), or null when the board did not report them.</summary>
        public IReadOnlySet<string>? Capabilities { get; }

        /// <summary>True when the board reported its capability set (fw 1.1.0+); false for legacy boards.</summary>
        public bool IsKnown => Capabilities != null;

        public FirmwareInfo(string version, IEnumerable<string>? capabilities)
        {
            Version = version ?? "";
            Capabilities = capabilities == null
                ? null
                : new HashSet<string>(capabilities.Select(c => c.Trim().ToLowerInvariant()).Where(c => c.Length > 0),
                                      StringComparer.OrdinalIgnoreCase);
        }

        /// <summary>
        /// True when the board supports <paramref name="capability"/>. Permissive when the capability
        /// set is unknown (legacy firmware): nothing is blocked for a board that can't tell us what it has.
        /// </summary>
        public bool Has(string capability) =>
            Capabilities == null || Capabilities.Contains(capability.Trim());

        /// <summary>Capabilities as a plain array for JSON payloads; null when unknown.</summary>
        public string[]? CapabilityArray() => Capabilities?.OrderBy(c => c, StringComparer.Ordinal).ToArray();

        /// <summary>Human-readable summary for logs: "firmware 1.1.0 (io, neopixel)" / "firmware unknown (legacy, pre-1.1.0)".</summary>
        public string Describe()
        {
            if (!IsKnown && Version.Length == 0)
                return $"firmware unknown (legacy, pre-{FirstReportingVersion})";
            string version = Version.Length > 0 ? Version : "unknown";
            string caps    = Capabilities == null ? "capabilities unknown"
                           : Capabilities.Count == 0 ? "no capabilities"
                           : string.Join(", ", CapabilityArray()!);
            return $"firmware {version} ({caps})";
        }

        /// <summary>
        /// Parses an "ID?" reply. Accepts the legacy <c>ID:&lt;id&gt;</c> and the extended
        /// <c>ID:&lt;id&gt;;fw=&lt;ver&gt;;caps=&lt;a,b,c&gt;</c> forms; unknown <c>key=value</c> fields and
        /// surrounding whitespace are tolerated. Returns false (and <see cref="Unknown"/>) when the line
        /// is not an ID reply or its id is not <paramref name="expectedId"/>.
        /// </summary>
        public static bool TryParseHandshake(string? line, string expectedId, out FirmwareInfo info)
        {
            info = Unknown;
            if (line == null) return false;

            line = line.Trim();
            if (!line.StartsWith("ID:", StringComparison.Ordinal)) return false;

            var parts = line.Substring(3).Split(';');
            if (parts[0].Trim() != expectedId) return false;

            string version = "";
            List<string>? caps = null;
            for (int i = 1; i < parts.Length; i++)
            {
                var field = parts[i].Trim();
                if (field.Length == 0) continue;
                int eq = field.IndexOf('=');
                if (eq < 0) continue;                       // not key=value — ignore
                var key   = field.Substring(0, eq).Trim();
                var value = field.Substring(eq + 1).Trim();
                if (key.Equals("fw", StringComparison.OrdinalIgnoreCase))
                    version = value;
                else if (key.Equals("caps", StringComparison.OrdinalIgnoreCase))
                    caps = value.Split(',').Select(c => c.Trim()).Where(c => c.Length > 0).ToList();
                // anything else: a field a newer firmware added — tolerated
            }

            info = new FirmwareInfo(version, caps);
            return true;
        }

        public override string ToString() => Describe();
    }

    /// <summary>
    /// Capability gating. Refuses a feature a board is known not to have, with an error that names the
    /// device, its firmware version and which sketch/version to flash. Legacy boards (capabilities
    /// unknown) are never refused.
    /// </summary>
    public static class FirmwareGate
    {
        /// <summary>
        /// Throws <see cref="InvalidOperationException"/> when <paramref name="firmware"/> is known to lack
        /// <paramref name="capability"/>. The message reads e.g.
        /// "Load cell not supported by ROBOT_NANO_001 firmware 1.0.0 — flash NanoEdgeDevice 1.1.0 or newer".
        /// </summary>
        public static void Require(FirmwareInfo firmware, string deviceId, string capability, string feature,
                                   string sketch, string minVersion = FirmwareInfo.FirstReportingVersion)
        {
            if (firmware.Has(capability)) return;
            throw new InvalidOperationException(Message(firmware, deviceId, feature, sketch, minVersion));
        }

        /// <summary>The refusal text, exposed so tests and callers can build it without throwing.</summary>
        public static string Message(FirmwareInfo firmware, string deviceId, string feature, string sketch,
                                     string minVersion = FirmwareInfo.FirstReportingVersion)
        {
            string version = firmware.Version.Length > 0 ? firmware.Version : "unknown";
            return $"{feature} not supported by {deviceId} firmware {version} — flash {sketch} {minVersion} or newer";
        }
    }
}
