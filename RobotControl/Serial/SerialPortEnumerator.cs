using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Runtime.Versioning;
using Microsoft.Win32;

namespace Controller.RobotControl.Serial
{
    /// <summary>
    /// Lists the machine's serial ports with descriptive info (friendly name, manufacturer, VID/PID,
    /// serial number) so the user can pick an unused port to attach a generic serial device to.
    ///
    /// Windows: extends <c>SerialLineDevice.WindowsUsbComPorts</c>' registry walk
    /// (HKLM\SYSTEM\CurrentControlSet\Enum\USB) to also pull FriendlyName / VID / PID / Mfg.
    /// Linux: reads /dev/serial/by-id symlinks and sysfs. On any platform where the info is
    /// unavailable it degrades to portName-only — it never throws.
    /// </summary>
    public static class SerialPortEnumerator
    {
        /// <summary>Every serial port on the machine, enriched with descriptive info where available.
        /// <c>inUse</c>/<c>usedBy</c> are left for the caller to fill.</summary>
        public static List<SerialPortDescriptor> List()
        {
            var ports = new List<SerialPortDescriptor>();
            string[] names;
            try { names = SerialPort.GetPortNames(); }
            catch { names = Array.Empty<string>(); }

            IReadOnlyDictionary<string, SerialPortDescriptor> info;
            try
            {
                info = OperatingSystem.IsWindows() ? WindowsPortInfo()
                     : OperatingSystem.IsLinux()   ? LinuxPortInfo(names)
                     : new Dictionary<string, SerialPortDescriptor>();
            }
            catch { info = new Dictionary<string, SerialPortDescriptor>(); }

            // Union of the live port list and any ports the OS metadata named (a known port that
            // GetPortNames missed still shows), de-duplicated by name.
            var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            foreach (var name in names.Concat(info.Keys))
            {
                if (!seen.Add(name)) continue;
                if (info.TryGetValue(name, out var d))
                    ports.Add(d);
                else
                    ports.Add(new SerialPortDescriptor { PortName = name });
            }

            ports.Sort((a, b) => string.Compare(a.PortName, b.PortName, StringComparison.OrdinalIgnoreCase));
            return ports;
        }

        // ── Windows ───────────────────────────────────────────────────────────────

        [SupportedOSPlatform("windows")]
        private static Dictionary<string, SerialPortDescriptor> WindowsPortInfo()
        {
            var map = new Dictionary<string, SerialPortDescriptor>(StringComparer.OrdinalIgnoreCase);
            using var usb = Registry.LocalMachine.OpenSubKey(@"SYSTEM\CurrentControlSet\Enum\USB");
            if (usb == null) return map;

            foreach (var vidPid in usb.GetSubKeyNames())
            {
                using var dev = usb.OpenSubKey(vidPid);
                if (dev == null) continue;

                var (vid, pid) = ParseVidPid(vidPid);
                foreach (var instance in dev.GetSubKeyNames())
                {
                    using var inst = dev.OpenSubKey(instance);
                    if (inst == null) continue;

                    using var parms = inst.OpenSubKey("Device Parameters");
                    if (parms?.GetValue("PortName") is not string com || string.IsNullOrEmpty(com)) continue;

                    map[com] = new SerialPortDescriptor
                    {
                        PortName     = com,
                        Description  = CleanFriendly(inst.GetValue("FriendlyName") as string ?? inst.GetValue("DeviceDesc") as string),
                        Manufacturer = CleanRegString(inst.GetValue("Mfg") as string),
                        Vid          = vid,
                        Pid          = pid,
                        // The instance id is the device serial number when it is not a bus-location id
                        // (bus-location ids contain '&', real serials usually don't).
                        SerialNumber = instance.Contains('&') ? "" : instance,
                    };
                }
            }
            return map;
        }

        private static (string vid, string pid) ParseVidPid(string key)
        {
            string vid = "", pid = "";
            foreach (var part in key.Split('&'))
            {
                if (part.StartsWith("VID_", StringComparison.OrdinalIgnoreCase)) vid = part[4..];
                else if (part.StartsWith("PID_", StringComparison.OrdinalIgnoreCase)) pid = part[4..];
            }
            return (vid, pid);
        }

        /// <summary>Registry display strings can be a localized indirection
        /// ("@oem.inf,%desc%;Human Text") — keep the human-readable tail.</summary>
        private static string CleanRegString(string? s)
        {
            if (string.IsNullOrWhiteSpace(s)) return "";
            int semi = s.LastIndexOf(';');
            if (semi >= 0 && semi < s.Length - 1) s = s[(semi + 1)..];
            return s.Trim();
        }

        private static string CleanFriendly(string? s) => CleanRegString(s);

        // ── Linux ─────────────────────────────────────────────────────────────────

        private static Dictionary<string, SerialPortDescriptor> LinuxPortInfo(string[] names)
        {
            var map = new Dictionary<string, SerialPortDescriptor>(StringComparer.Ordinal);

            // /dev/serial/by-id/usb-<Manufacturer>_<Product>_<Serial>-ifXX-portY -> /dev/ttyUSBx
            const string byId = "/dev/serial/by-id";
            try
            {
                if (Directory.Exists(byId))
                {
                    foreach (var link in Directory.EnumerateFileSystemEntries(byId))
                    {
                        string target;
                        try { target = Path.GetFullPath(Path.Combine(byId, ReadLinkTarget(link) ?? "")); }
                        catch { continue; }
                        var name = Path.GetFileName(link);
                        map[target] = DescribeByIdName(target, name);
                    }
                }
            }
            catch { /* best effort */ }

            // Enrich / fill from sysfs for the live ports.
            foreach (var dev in names)
            {
                var baseName = Path.GetFileName(dev);
                var d = map.TryGetValue(dev, out var existing) ? existing : new SerialPortDescriptor { PortName = dev };
                try
                {
                    string sys = $"/sys/class/tty/{baseName}/device";
                    // Walk up to the USB device node that holds idVendor/idProduct/manufacturer/serial.
                    for (int up = 0; up < 4; up++)
                    {
                        string vidPath = Path.Combine(sys, "idVendor");
                        if (File.Exists(vidPath))
                        {
                            if (string.IsNullOrEmpty(d.Vid))          d.Vid          = ReadTrim(vidPath);
                            if (string.IsNullOrEmpty(d.Pid))          d.Pid          = ReadTrim(Path.Combine(sys, "idProduct"));
                            if (string.IsNullOrEmpty(d.Manufacturer)) d.Manufacturer = ReadTrim(Path.Combine(sys, "manufacturer"));
                            if (string.IsNullOrEmpty(d.SerialNumber)) d.SerialNumber = ReadTrim(Path.Combine(sys, "serial"));
                            if (string.IsNullOrEmpty(d.Description))   d.Description  = ReadTrim(Path.Combine(sys, "product"));
                            break;
                        }
                        sys = Path.Combine(sys, "..");
                    }
                }
                catch { /* best effort */ }
                map[dev] = d;
            }
            return map;
        }

        private static SerialPortDescriptor DescribeByIdName(string portName, string byIdName)
        {
            // usb-Manufacturer_Product_Serial-ifXX-portY
            string s = byIdName;
            if (s.StartsWith("usb-", StringComparison.Ordinal)) s = s[4..];
            int dash = s.IndexOf("-if", StringComparison.Ordinal);
            if (dash >= 0) s = s[..dash];
            var parts = s.Split('_');
            string serial = parts.Length > 0 ? parts[^1] : "";
            string desc   = s.Replace('_', ' ').Trim();
            return new SerialPortDescriptor { PortName = portName, Description = desc, SerialNumber = serial };
        }

        private static string? ReadLinkTarget(string path)
        {
            try
            {
                var info = new FileInfo(path);
                return info.LinkTarget;
            }
            catch { return null; }
        }

        private static string ReadTrim(string path)
        {
            try { return File.Exists(path) ? File.ReadAllText(path).Trim() : ""; }
            catch { return ""; }
        }
    }
}
