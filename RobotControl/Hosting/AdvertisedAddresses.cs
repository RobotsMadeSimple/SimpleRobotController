using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;

namespace Controller.RobotControl.Hosting
{
    /// <summary>
    /// The controller's IPv4 addresses, best first, for the mDNS <c>Addresses</c> TXT record.
    ///
    /// mDNS publishes an A record for every adapter (Wi-Fi, Ethernet, VPNs such as Tailscale, Hyper-V /
    /// WSL / VirtualBox switches) and Android's resolver hands the app just one of them, often one the
    /// phone can't reach. The TXT list gives the app all of them in order of how likely a client on the
    /// local network can use them, so it can try them and connect on the first that answers.
    /// </summary>
    public static class AdvertisedAddresses
    {
        public sealed record Candidate(string Address, string Name, string Description,
            NetworkInterfaceType Type, bool HasGateway);

        // Adapters that are virtual switches or overlay networks: still listed (a VPN address is the
        // way in from off-site) but after the physical LAN ones.
        private static readonly string[] VirtualHints =
        [
            "virtual", "vethernet", "hyper-v", "wsl", "vmware", "virtualbox", "docker",
            "tailscale", "zerotier", "wireguard", "vpn", "tap-", "tun", "loopback", "bluetooth",
        ];

        /// <summary>Every usable IPv4 address on an up interface.</summary>
        public static List<Candidate> Collect()
        {
            var list = new List<Candidate>();
            try
            {
                foreach (var nic in NetworkInterface.GetAllNetworkInterfaces())
                {
                    if (nic.OperationalStatus != OperationalStatus.Up) continue;
                    if (nic.NetworkInterfaceType == NetworkInterfaceType.Loopback) continue;
                    IPInterfaceProperties props;
                    try { props = nic.GetIPProperties(); } catch { continue; }
                    bool gateway = props.GatewayAddresses.Any(g =>
                        g.Address.AddressFamily == AddressFamily.InterNetwork && !g.Address.Equals(IPAddress.Any));
                    foreach (var ua in props.UnicastAddresses)
                    {
                        var a = ua.Address;
                        if (a.AddressFamily != AddressFamily.InterNetwork) continue;
                        if (IPAddress.IsLoopback(a)) continue;
                        var b = a.GetAddressBytes();
                        if (b[0] == 169 && b[1] == 254) continue;   // link-local: no DHCP, unreachable
                        list.Add(new Candidate(a.ToString(), nic.Name, nic.Description, nic.NetworkInterfaceType, gateway));
                    }
                }
            }
            catch { /* interface enumeration unavailable — advertise nothing extra */ }
            return list;
        }

        /// <summary>Lower is better: physical adapters with a default gateway first, overlays/virtual last.</summary>
        public static int Rank(Candidate c)
        {
            var label = (c.Name + " " + c.Description).ToLowerInvariant();
            bool isVirtual = VirtualHints.Any(label.Contains) || IsCgnat(c.Address);
            bool physical = c.Type is NetworkInterfaceType.Ethernet or NetworkInterfaceType.Wireless80211
                or NetworkInterfaceType.GigabitEthernet or NetworkInterfaceType.FastEthernetT;
            int rank = 0;
            if (isVirtual) rank += 4;
            if (!c.HasGateway) rank += 2;
            if (!physical) rank += 1;
            return rank;
        }

        /// <summary>100.64.0.0/10 — carrier-grade NAT space, used by Tailscale and similar overlays.</summary>
        public static bool IsCgnat(string address) =>
            IPAddress.TryParse(address, out var a) && a.AddressFamily == AddressFamily.InterNetwork
            && a.GetAddressBytes() is var b && b[0] == 100 && (b[1] & 0xC0) == 64;

        /// <summary>Comma-separated, best first, kept inside one TXT string (255 bytes with the key).</summary>
        public static string Txt(IEnumerable<Candidate> candidates)
        {
            var ordered = candidates.Select((c, i) => (c, i))
                .OrderBy(x => Rank(x.c)).ThenBy(x => x.i)
                .Select(x => x.c.Address).Distinct();
            var parts = new List<string>();
            int len = 0;
            foreach (var a in ordered)
            {
                if (len + a.Length + 1 > 240) break;
                parts.Add(a);
                len += a.Length + 1;
            }
            return string.Join(",", parts);
        }
    }
}
