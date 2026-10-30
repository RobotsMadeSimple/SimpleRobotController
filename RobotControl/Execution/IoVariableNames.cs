using Controller.RobotControl.Io;

namespace Controller.RobotControl.Execution
{
    /// <summary>
    /// How a Nano pin is named and valued when expressions read it — the one place both
    /// <c>ProgramExecutor.AddIoVariables</c> (the values) and <c>GetExpressionSymbols</c>
    /// (the list the app offers) get it from, so they cannot drift apart.
    /// </summary>
    /// <remarks>
    /// <para>The key is <c>io.&lt;nano name&gt;.&lt;pin name&gt;</c>. Expression identifiers are
    /// <c>[A-Za-z0-9_]</c> only, so a nano named "Arduino Nano" with a pin "Load Cell" is
    /// unreachable as typed. Such a pin is ALSO published under an identifier-safe alias with
    /// every run of other characters replaced by one underscore (<c>io.Arduino_Nano.Load_Cell</c>);
    /// the app composes the same alias when it offers a load-cell chip.</para>
    /// <para>A load cell reads as its calibrated grams (the master unit); every other pin as 1 / 0.</para>
    /// </remarks>
    internal static class IoVariableNames
    {
        /// <summary>Whether <paramref name="c"/> may appear in an expression identifier (ASCII only,
        /// matching the app-side rule).</summary>
        public static bool IsIdentChar(char c) =>
            c is (>= 'A' and <= 'Z') or (>= 'a' and <= 'z') or (>= '0' and <= '9') or '_';

        /// <summary>
        /// <paramref name="name"/> with every run of characters outside <c>[A-Za-z0-9_]</c>
        /// replaced by a single underscore. Returns the same instance when nothing changes,
        /// so the common case allocates nothing.
        /// </summary>
        public static string Sanitize(string name)
        {
            int i = 0;
            while (i < name.Length && IsIdentChar(name[i])) i++;
            if (i == name.Length) return name;

            var sb = new System.Text.StringBuilder(name.Length);
            sb.Append(name, 0, i);
            bool inRun = false;
            for (; i < name.Length; i++)
            {
                if (IsIdentChar(name[i])) { sb.Append(name[i]); inRun = false; }
                else if (!inRun)          { sb.Append('_');     inRun = true;  }
            }
            return sb.ToString();
        }

        /// <summary>The key as the names are typed: <c>io.&lt;nano&gt;.&lt;pin&gt;</c>.</summary>
        public static string Key(string nanoName, string pinName) => $"io.{nanoName}.{pinName}";

        /// <summary>The identifier-safe alias of <see cref="Key"/>, or null when the key already is one.</summary>
        public static string? Alias(string nanoName, string pinName)
        {
            var n = Sanitize(nanoName);
            var p = Sanitize(pinName);
            return ReferenceEquals(n, nanoName) && ReferenceEquals(p, pinName) ? null : Key(n, p);
        }

        /// <summary>What an expression reads for a pin: grams for a load cell, else 1 / 0.</summary>
        public static double Value(IoPinState pin) =>
            pin.Type == PinType.LoadCell ? pin.Grams : (pin.Value ? 1.0 : 0.0);

        /// <summary>Writes every named pin of every named device into <paramref name="io"/>, under
        /// its typed key and, when that is not an identifier, its alias too.</summary>
        public static void AddPins(IEnumerable<IoDeviceState> devices, Dictionary<string, double> io)
        {
            foreach (var nano in devices)
            {
                if (string.IsNullOrEmpty(nano.Name)) continue;
                foreach (var pin in nano.Pins)
                {
                    if (string.IsNullOrEmpty(pin.Name)) continue;
                    double v = Value(pin);
                    io[Key(nano.Name, pin.Name)] = v;
                    if (Alias(nano.Name, pin.Name) is { } alias) io[alias] = v;
                }
            }
        }

        /// <summary>The names the app can offer — the alias when there is one (the typed form
        /// cannot be written in an expression), else the key — with a description.</summary>
        public static IEnumerable<(string Name, string Description)> Symbols(IEnumerable<IoDeviceState> devices)
        {
            foreach (var nano in devices)
            {
                if (string.IsNullOrEmpty(nano.Name)) continue;
                foreach (var pin in nano.Pins)
                {
                    if (string.IsNullOrEmpty(pin.Name)) continue;
                    var name = Alias(nano.Name, pin.Name) ?? Key(nano.Name, pin.Name);
                    yield return (name, pin.Type == PinType.LoadCell
                        ? $"Load cell weight in grams — '{nano.Name}' pin {pin.Pin}"
                        : $"Nano '{nano.Name}' pin {pin.Pin} ({pin.Type})");
                }
            }
        }
    }
}
