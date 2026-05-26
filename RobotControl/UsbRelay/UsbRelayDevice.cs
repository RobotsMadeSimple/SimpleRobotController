using HidSharp;
using System;
using System.Linq;

namespace Controller.RobotControl.UsbRelay
{
    /// <summary>
    /// Low-level driver for the DCTTECH 4-channel USB HID relay board.
    /// VID 0x16C0 / PID 0x05DF
    ///
    /// Protocol (feature reports via the HID control pipe):
    ///   Write: SetFeature([0x00, CMD, relay, 0x00 × 6])
    ///          CMD: 0xFF = ON, 0xFD = OFF
    ///   Read:  GetFeature([0x01, …×9])
    ///          bytes[1..5] = ASCII serial, byte[8] = relay bitmask (bit 0 = relay 1)
    /// </summary>
    public sealed class UsbRelayDevice : IDisposable
    {
        public const int VendorId   = 0x16C0;
        public const int ProductId  = 0x05DF;
        public const int RelayCount = 4;

        private const byte CmdOn  = 0xFF;
        private const byte CmdOff = 0xFD;

        private readonly HidStream _stream;

        public string Serial { get; private set; }

        private UsbRelayDevice(HidStream stream, string serial)
        {
            _stream = stream;
            Serial  = serial;
        }

        // ── Factory ────────────────────────────────────────────────────────────

        /// <summary>
        /// Opens the first matching relay board. Returns null if none is connected.
        /// </summary>
        public static UsbRelayDevice? TryOpen()
        {
            var hidDevice = DeviceList.Local.GetHidDeviceOrNull(VendorId, ProductId);
            if (hidDevice == null) return null;

            HidStream stream;
            try
            {
                stream = hidDevice.Open();
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[UsbRelay] Could not open device: {ex.Message}");
                return null;
            }

            // Read serial from feature report (best-effort)
            string serial = "";
            try
            {
                var buf = new byte[10];
                buf[0] = 0x01;
                stream.GetFeature(buf);
                serial = System.Text.Encoding.ASCII.GetString(
                    buf.Skip(1).Take(5).Where(b => b >= 0x20 && b <= 0x7E).ToArray());
            }
            catch { }

            Console.WriteLine($"[UsbRelay] Opened relay board (serial: '{serial}')");
            return new UsbRelayDevice(stream, serial);
        }

        // ── Public API ─────────────────────────────────────────────────────────

        /// <summary>Set relay on or off. relay: 1–4.</summary>
        public void Set(int relay, bool on)
        {
            if (relay < 1 || relay > RelayCount)
                throw new ArgumentOutOfRangeException(nameof(relay), $"Relay must be 1–{RelayCount}");

            byte cmd = on ? CmdOn : CmdOff;
            var buf = new byte[] { 0x00, cmd, (byte)relay, 0, 0, 0, 0, 0, 0 };
            _stream.SetFeature(buf);
        }

        /// <summary>Read the current relay bitmask. Bit 0 = relay 1, bit 3 = relay 4.</summary>
        public int ReadBitmask()
        {
            var buf = new byte[10];
            buf[0] = 0x01;
            _stream.GetFeature(buf);
            return buf[8];
        }

        /// <summary>Returns true if the given relay is on in the bitmask.</summary>
        public static bool IsOn(int bitmask, int relay) =>
            ((bitmask >> (relay - 1)) & 1) == 1;

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Dispose()
        {
            try { _stream.Dispose(); } catch { }
        }
    }
}
