using HidSharp;
using Microsoft.Win32.SafeHandles;
using System.Runtime.InteropServices;

namespace Controller.RobotControl.UsbRelay
{
    /// <summary>
    /// Low-level driver for the DCTTECH 4-channel USB HID relay board.
    /// VID 0x16C0 / PID 0x05DF
    ///
    /// Protocol (Windows blocks the interrupt OUT pipe for this device;
    /// feature reports via the control pipe are the only path that works):
    ///   Write: HidD_SetFeature([0x00, CMD, relay, 0x00 × 6])
    ///          CMD: 0xFF = ON, 0xFD = OFF
    ///   Read:  HidD_GetFeature([0x01, …×9])
    ///          bytes[1..5] = ASCII serial, byte[8] = relay bitmask (bit 0 = relay 1)
    /// </summary>
    public sealed class UsbRelayDevice : IDisposable
    {
        public const int VendorId   = 0x16C0;
        public const int ProductId  = 0x05DF;
        public const int RelayCount = 4;

        private const byte CmdOn  = 0xFF;
        private const byte CmdOff = 0xFD;

        private readonly SafeFileHandle _handle;

        public string Serial { get; private set; }

        private UsbRelayDevice(SafeFileHandle handle, string serial)
        {
            _handle = handle;
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

            string path;
            try { path = hidDevice.GetFileSystemName(); }
            catch (Exception ex)
            {
                Console.WriteLine($"[UsbRelay] Could not get device path: {ex.Message}");
                return null;
            }

            // Open with both read and write share so other processes can still enumerate
            var handle = NativeMethods.CreateFile(
                path,
                NativeMethods.GenericRead | NativeMethods.GenericWrite,
                NativeMethods.FileShareRead | NativeMethods.FileShareWrite,
                IntPtr.Zero,
                NativeMethods.OpenExisting,
                0,
                IntPtr.Zero);

            if (handle.IsInvalid)
            {
                Console.WriteLine($"[UsbRelay] CreateFile failed (error {Marshal.GetLastWin32Error()}).");
                handle.Dispose();
                return null;
            }

            // Read serial from feature report (best-effort)
            string serial = "";
            try
            {
                var buf = new byte[10];
                buf[0] = 0x01;
                if (NativeMethods.HidD_GetFeature(handle, buf, buf.Length))
                    serial = System.Text.Encoding.ASCII.GetString(
                        buf.Skip(1).Take(5).Where(b => b >= 0x20 && b <= 0x7E).ToArray());
            }
            catch { }

            Console.WriteLine($"[UsbRelay] Opened relay board (serial: '{serial}')");
            return new UsbRelayDevice(handle, serial);
        }

        // ── Public API ─────────────────────────────────────────────────────────

        /// <summary>Set relay on or off. relay: 1–4.</summary>
        public void Set(int relay, bool on)
        {
            if (relay < 1 || relay > RelayCount)
                throw new ArgumentOutOfRangeException(nameof(relay), $"Relay must be 1–{RelayCount}");

            byte cmd = on ? CmdOn : CmdOff;
            var buf = new byte[] { 0x00, cmd, (byte)relay, 0, 0, 0, 0, 0, 0 };
            if (!NativeMethods.HidD_SetFeature(_handle, buf, buf.Length))
                throw new IOException($"HidD_SetFeature failed (error {Marshal.GetLastWin32Error()})");
        }

        /// <summary>Read the current relay bitmask. Bit 0 = relay 1, bit 3 = relay 4.</summary>
        public int ReadBitmask()
        {
            var buf = new byte[10];
            buf[0] = 0x01;
            if (!NativeMethods.HidD_GetFeature(_handle, buf, buf.Length))
                throw new IOException($"HidD_GetFeature failed (error {Marshal.GetLastWin32Error()})");
            return buf[8];
        }

        /// <summary>Returns true if the given relay is on in the bitmask.</summary>
        public static bool IsOn(int bitmask, int relay) =>
            ((bitmask >> (relay - 1)) & 1) == 1;

        // ── Lifecycle ──────────────────────────────────────────────────────────

        public void Dispose()
        {
            try { _handle.Dispose(); } catch { }
        }

        // ── P/Invoke ───────────────────────────────────────────────────────────

        private static class NativeMethods
        {
            internal const uint GenericRead    = 0x80000000;
            internal const uint GenericWrite   = 0x40000000;
            internal const uint FileShareRead  = 0x00000001;
            internal const uint FileShareWrite = 0x00000002;
            internal const uint OpenExisting   = 3;

            [DllImport("kernel32.dll", SetLastError = true, CharSet = CharSet.Unicode)]
            internal static extern SafeFileHandle CreateFile(
                string lpFileName,
                uint dwDesiredAccess,
                uint dwShareMode,
                IntPtr lpSecurityAttributes,
                uint dwCreationDisposition,
                uint dwFlagsAndAttributes,
                IntPtr hTemplateFile);

            [DllImport("hid.dll", SetLastError = true)]
            internal static extern bool HidD_SetFeature(
                SafeFileHandle handle,
                byte[] lpReportBuffer,
                int reportBufferLength);

            [DllImport("hid.dll", SetLastError = true)]
            internal static extern bool HidD_GetFeature(
                SafeFileHandle handle,
                byte[] lpReportBuffer,
                int reportBufferLength);
        }
    }
}
