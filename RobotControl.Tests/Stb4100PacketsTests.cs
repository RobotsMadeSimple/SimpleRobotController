using Controller.RobotControl.Controllers.STB4100;

public class Stb4100PacketsTests
{
    // ── Status packets ──────────────────────────────────────────────────────
    // Mirrors STB4100.SendStatus's original byte layout:
    //   [2, command, 0, commandCount LE×2, outputsByte, 0, tail×2]
    //   tail = [5, 0] when command == 14, else [0, 0].

    [Fact]
    public void StatusPacket_Heartbeat_MatchesOriginalLayout()
    {
        var bytes = Stb4100Packets.BuildStatusPacket(Stb4100Packets.StatusHeartbeat, commandCount: 5, outputsByte: 10);
        Assert.Equal(new byte[] { 2, 3, 0, 5, 0, 10, 0, 0, 0 }, bytes);
    }

    [Fact]
    public void StatusPacket_AwaitReady_UsesFiveZeroTail()
    {
        // commandCount=300 exercises the little-endian split across both count bytes.
        var bytes = Stb4100Packets.BuildStatusPacket(Stb4100Packets.StatusAwaitReady, commandCount: 300, outputsByte: 0);
        Assert.Equal(new byte[] { 2, 14, 0, 44, 1, 0, 0, 5, 0 }, bytes);
    }

    [Theory]
    [InlineData(Stb4100Packets.StatusJogStart)]
    [InlineData(Stb4100Packets.StatusJogPoll)]
    [InlineData(Stb4100Packets.StatusResetPhase1)]
    [InlineData(Stb4100Packets.StatusResetPhase2)]
    public void StatusPacket_NonAwaitReadyCodes_UseZeroTail(byte statusCode)
    {
        var bytes = Stb4100Packets.BuildStatusPacket(statusCode, commandCount: 0, outputsByte: 0);
        Assert.Equal(new byte[] { 2, statusCode, 0, 0, 0, 0, 0, 0, 0 }, bytes);
    }

    // ── Move / Jog command packets ──────────────────────────────────────────
    // Mirrors STB4100.SendCommand's "Jog"/"Move" branch:
    //   [3, commandId, 0, commandCount LE×2, 0×4, step×4 (int32 LE each), 0×2]

    [Fact]
    public void MoveCommandPacket_JogWithMixedSteps_MatchesOriginalLayout()
    {
        var bytes = Stb4100Packets.BuildMoveCommandPacket(
            Stb4100Packets.CommandJog, commandCount: 2, stepsByPin: new[] { 100, -50, 0, 32767 });

        Assert.Equal(new byte[]
        {
            3, 5, 0,          // header + command id
            2, 0,             // commandCount = 2 (LE)
            0, 0, 0, 0,       // reserved 4 zero bytes
            100, 0, 0, 0,     // pin 1: +100
            206, 255, 255, 255, // pin 2: -50 (int32 LE two's complement)
            0, 0, 0, 0,       // pin 3: 0
            255, 127, 0, 0,   // pin 4: 32767
            0, 0,             // trailing zero bytes
        }, bytes);
        Assert.Equal(27, bytes.Length);
    }

    [Fact]
    public void MoveCommandPacket_UsesRequestedCommandId()
    {
        var jog  = Stb4100Packets.BuildMoveCommandPacket(Stb4100Packets.CommandJog, 0, new[] { 0, 0, 0, 0 });
        var move = Stb4100Packets.BuildMoveCommandPacket(Stb4100Packets.CommandMove, 0, new[] { 0, 0, 0, 0 });

        Assert.Equal(Stb4100Packets.CommandJog, jog[1]);
        Assert.Equal(Stb4100Packets.CommandMove, move[1]);
    }

    // ── ClearSteps command packet ───────────────────────────────────────────
    // [3, 9, 0, commandCount LE×2, 0×16, 15, 0×5]

    [Fact]
    public void ClearStepsPacket_MatchesOriginalLayout()
    {
        var bytes = Stb4100Packets.BuildClearStepsPacket(commandCount: 1000);

        var expected = new List<byte> { 3, 9, 0, 232, 3 }; // 1000 = 0x03E8 -> LE [232, 3]
        expected.AddRange(new byte[16]);
        expected.Add(15);
        expected.AddRange(new byte[5]);

        Assert.Equal(expected, bytes);
        Assert.Equal(27, bytes.Length);
    }

    // ── Reset command packet ────────────────────────────────────────────────
    // [3, 15, 0, commandCount LE×2, 0×8, 0,0,0,100,5, 0×9]

    [Fact]
    public void ResetPacket_MatchesOriginalLayout()
    {
        var bytes = Stb4100Packets.BuildResetPacket(commandCount: 65535);

        var expected = new List<byte> { 3, 15, 0, 255, 255 }; // 65535 -> LE [255, 255]
        expected.AddRange(new byte[8]);
        expected.AddRange(new byte[] { 0, 0, 0, 100, 5 });
        expected.AddRange(new byte[9]);

        Assert.Equal(expected, bytes);
        Assert.Equal(27, bytes.Length);
    }
}
