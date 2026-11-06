using System.Net.NetworkInformation;
using Controller.RobotControl.Hosting;
using Xunit;

namespace RobotControl.Tests;

public class AdvertisedAddressesTests
{
    private static AdvertisedAddresses.Candidate C(string ip, string name, NetworkInterfaceType t = NetworkInterfaceType.Ethernet, bool gw = true, string desc = "") =>
        new(ip, name, desc, t, gw);

    [Fact]
    public void LanComesBeforeOverlayAndVirtualSwitches()
    {
        var txt = AdvertisedAddresses.Txt([
            C("100.103.123.116", "Tailscale", NetworkInterfaceType.Unknown, gw: false, desc: "Tailscale Tunnel"),
            C("172.25.48.1", "vEthernet (WSL)", gw: false, desc: "Hyper-V Virtual Ethernet Adapter"),
            C("192.168.4.83", "Wi-Fi", NetworkInterfaceType.Wireless80211),
        ]);
        Assert.Equal("192.168.4.83,172.25.48.1,100.103.123.116", txt);
    }

    [Fact]
    public void CgnatRangeIsDetected()
    {
        Assert.True(AdvertisedAddresses.IsCgnat("100.64.0.1"));
        Assert.True(AdvertisedAddresses.IsCgnat("100.127.255.254"));
        Assert.False(AdvertisedAddresses.IsCgnat("100.128.0.1"));
        Assert.False(AdvertisedAddresses.IsCgnat("192.168.1.2"));
    }

    [Fact]
    public void TxtStaysWithinOneRecordString()
    {
        var many = Enumerable.Range(0, 40).Select(i => C($"10.0.{i}.123", $"eth{i}")).ToList();
        Assert.True(AdvertisedAddresses.Txt(many).Length <= 240);
    }

    [Fact]
    public void CollectReturnsNoLoopbackOrLinkLocal()
    {
        foreach (var c in AdvertisedAddresses.Collect())
        {
            Assert.False(c.Address.StartsWith("127."));
            Assert.False(c.Address.StartsWith("169.254."));
        }
    }
}
