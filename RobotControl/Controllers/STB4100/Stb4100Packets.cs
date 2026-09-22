using System.Collections.Generic;

namespace Controller.RobotControl.Controllers.STB4100;

/// <summary>
/// Byte-level payload construction for the STB4100's two outbound HID report
/// shapes — a "status" report (heartbeat / jog state machine) and a "command"
/// report (Jog/Move step deltas, ClearSteps, Reset). Pulled out of STB4100 so
/// the exact byte layout can be unit-tested in isolation from the HID stream
/// and the stateful motor-step bookkeeping (STB4100.SendCommand still owns
/// mutating StepperMotor.CurrentSteps — this class only turns already-computed
/// values into bytes).
///
/// LOAD-BEARING: the STB4100 board parses these reports by fixed byte offset.
/// Do not reorder or resize a packet — see docs/stb-loop-timing.md for why the
/// STB4100 code in general is sensitive to this kind of change.
/// </summary>
public static class Stb4100Packets
{
    // ── Command ids — second byte of a "command" report (STB4100.SendCommand) ──
    public const byte CommandMove       = 4;
    public const byte CommandJog        = 5;
    public const byte CommandClearSteps = 9;
    public const byte CommandReset      = 15;

    // ── Status codes — second byte of a "status" report (STB4100.SendStatus) ──
    // Named after where STB4100.Loop()'s jog/heartbeat state machine sends them.
    public const byte StatusResetPhase2 = 2;  // ResetInternal() — second status sent during reset
    public const byte StatusHeartbeat   = 3;  // Loop() steady-state, not jogging
    public const byte StatusJogPoll     = 6;  // Loop() case 2 — polling for IDLE while stopping a jog
    public const byte StatusResetPhase1 = 7;  // ResetInternal() — first status sent during reset
    public const byte StatusJogStart    = 10; // Loop() case 0 — entering the jog state
    public const byte StatusAwaitReady  = 14; // Loop() "not ready yet" branch

    /// <summary>
    /// Builds a 9-byte "status" report: [2, statusCode, 0, commandCount(2 bytes LE),
    /// outputsByte, 0, tail(2 bytes)]. The tail is [5, 0] for
    /// <see cref="StatusAwaitReady"/> and [0, 0] for every other status code —
    /// this mirrors STB4100.SendStatus's original `if (command == 14)` branch.
    /// </summary>
    public static byte[] BuildStatusPacket(byte statusCode, int commandCount, byte outputsByte)
    {
        var send = new List<byte> { 2, statusCode, 0 };
        send.AddRange(BitTools.NumberToBytes(commandCount, 2));
        send.Add(outputsByte);
        send.Add(0); // padding

        if (statusCode == StatusAwaitReady)
        {
            send.Add(5);
            send.Add(0);
        }
        else
        {
            send.Add(0);
            send.Add(0);
        }

        return send.ToArray();
    }

    /// <summary>
    /// Builds a 27-byte "Jog"/"Move" command report: [3, commandId, 0,
    /// commandCount(2 bytes LE), 0×4, stepsByPin[0..3] each as 4-byte signed LE, 0×2].
    /// <paramref name="stepsByPin"/> must have exactly 4 entries (pins 1–4), already
    /// direction-inverted by the caller — this method only encodes them.
    /// </summary>
    public static byte[] BuildMoveCommandPacket(byte commandId, int commandCount, IReadOnlyList<int> stepsByPin)
    {
        var send = new List<byte> { 3, commandId, 0 };
        send.AddRange(BitTools.NumberToBytes(commandCount, 2));
        send.AddRange(BitTools.NumberToBytes(0, 4));

        foreach (var step in stepsByPin)
            send.AddRange(BitTools.NumberToSignedBytes(step));

        send.AddRange(BitTools.NumberToBytes(0, 2));
        return send.ToArray();
    }

    /// <summary>
    /// Builds the 27-byte "ClearSteps" command report: [3, <see cref="CommandClearSteps"/>, 0,
    /// commandCount(2 bytes LE), 0×16, 15, 0×5].
    /// </summary>
    public static byte[] BuildClearStepsPacket(int commandCount)
    {
        var send = new List<byte> { 3, CommandClearSteps, 0 };
        send.AddRange(BitTools.NumberToBytes(commandCount, 2));
        send.AddRange(BitTools.NumberToBytes(0, 16));
        send.AddRange(BitTools.NumberToBytes(15, 1));
        send.AddRange(BitTools.NumberToBytes(0, 5));
        return send.ToArray();
    }

    /// <summary>
    /// Builds the 27-byte "Reset" command report: [3, <see cref="CommandReset"/>, 0,
    /// commandCount(2 bytes LE), 0×8, 0,0,0,100,5, 0×9].
    /// </summary>
    public static byte[] BuildResetPacket(int commandCount)
    {
        var send = new List<byte> { 3, CommandReset, 0 };
        send.AddRange(BitTools.NumberToBytes(commandCount, 2));
        send.AddRange(new byte[] { 0, 0, 0, 0, 0, 0, 0, 0 });   // Bytes 5-12
        send.AddRange(new byte[] { 0, 0, 0, 100, 5 });          // Bytes 13-17
        send.AddRange(BitTools.NumberToBytes(0, 9));            // Bytes 18-26
        return send.ToArray();
    }
}
