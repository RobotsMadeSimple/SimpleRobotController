using Makaretu.Dns;
using Microsoft.Extensions.Hosting;
using System.Net.NetworkInformation;

namespace Controller.RobotControl.Hosting
{
    /// <summary>
    /// Advertises the robot as <c>&lt;serial&gt;._robot._tcp</c> over mDNS so the app can
    /// find it on the LAN without knowing its address. Re-advertises when the identity
    /// changes or the network address changes, re-announces periodically, and
    /// unadvertises on shutdown so peers stop seeing the robot immediately.
    /// </summary>
    public sealed class MdnsAdvertiser : IDisposable
    {
        private readonly ServiceDiscovery _sd = new();
        private readonly ushort _port;
        private readonly IHostApplicationLifetime _lifetime;
        private readonly object _lock = new();

        private RobotIdentity _identity;
        private ServiceProfile _service;

        public MdnsAdvertiser(RobotIdentity identity, int port, IHostApplicationLifetime lifetime)
        {
            _identity = identity;
            _port     = (ushort)port;
            _lifetime = lifetime;
            _service  = BuildProfile(identity);
        }

        public void Start()
        {
            _sd.Advertise(_service);
            Console.WriteLine($"[mDNS] Advertising as '{_identity.SerialNumber}._robot._tcp' " +
                              $"(Type: '{_identity.RobotType}', Name: '{_identity.RobotName}')");

            NetworkChange.NetworkAddressChanged += OnNetworkAddressChanged;
            _lifetime.ApplicationStopping.Register(Dispose);

            // Periodic re-announce — exits cleanly when ApplicationStopping fires.
            _ = Task.Run(async () =>
            {
                try
                {
                    while (true)
                    {
                        await Task.Delay(3000, _lifetime.ApplicationStopping);
                        try { _sd.Announce(_service); } catch { }
                    }
                }
                catch (OperationCanceledException) { }
            });
        }

        /// <summary>Call when the robot's identity (name/type/serial) changes at runtime.</summary>
        public void IdentityChanged(RobotIdentity updated)
        {
            _ = Task.Run(async () =>
            {
                lock (_lock) _identity = updated;
                await ReadvertiseAsync();
                Console.WriteLine($"[mDNS] Re-advertising with Type: '{updated.RobotType}', Name: '{updated.RobotName}'");
            });
        }

        private void OnNetworkAddressChanged(object? sender, EventArgs e)
        {
            _ = Task.Run(async () =>
            {
                await Task.Delay(2000); // wait for DHCP to assign the new IP
                await ReadvertiseAsync();
                Console.WriteLine("[mDNS] Network address changed — re-advertising");
            });
        }

        private async Task ReadvertiseAsync()
        {
            ServiceProfile old, fresh;
            lock (_lock)
            {
                old   = _service;
                fresh = BuildProfile(_identity);
            }
            try { _sd.Unadvertise(old); } catch { }
            await Task.Delay(250);
            lock (_lock) _service = fresh;
            _sd.Advertise(fresh);
        }

        private ServiceProfile BuildProfile(RobotIdentity id)
        {
            var p = new ServiceProfile(id.SerialNumber, "_robot._tcp", _port);
            p.AddProperty("ControlEndpoint", "/control");
            p.AddProperty("SerialNumber",    id.SerialNumber);
            p.AddProperty("RobotType",       id.RobotType);
            p.AddProperty("RobotName",       id.RobotName);
            return p;
        }

        public void Dispose()
        {
            Console.WriteLine("[mDNS] Unadvertising…");
            NetworkChange.NetworkAddressChanged -= OnNetworkAddressChanged;
            try { _sd.Unadvertise(_service); } catch { }
            try { _sd.Dispose(); } catch { }
        }
    }
}
