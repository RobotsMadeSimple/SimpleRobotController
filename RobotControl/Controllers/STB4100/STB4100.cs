using HidSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Net.NetworkInformation;
using System.Threading;

public class STB4100
{
    private readonly int VendorId = 42784;
    private readonly int ProductId = 63491;

    private HidDevice? _device;
    private HidStream? _stream;

    public readonly List<StepperMotor> _motors = new();
    private readonly Dictionary<string, int> _commands = new()
    {
        { "Jog", 5 },
        { "ClearSteps", 9 },
        { "Reset", 15 },
        { "Move", 4 }
    };

    private readonly Dictionary<int, string> _statusEnum = new()
    {
        { 0, "Error" },
        { 1, "IDLE" },
        { 2, "Moving" },
        { 6, "Moving" },
        { 7, "Stopping" }
    };

    public bool connected;
    public int status;
    private int _commandCount;
    private bool _ready;
    private bool _jogging;
    private bool _stopJog;
    public int _jogState;
    private bool _jog;
    private bool _flip;
    private bool _resetBit;

    public bool moving;

    public bool Input1  { get; private set; }
    public bool Input2  { get; private set; }
    public bool Input3  { get; private set; }
    public bool Input4  { get; private set; }

    public bool Output1 { get; private set; }
    public bool Output2 { get; private set; }
    public bool Output3 { get; private set; }
    public bool Output4 { get; private set; }

    private byte _outputsByte = 0;

    public void SetOutput(int output, bool value)
    {
        int bit = output - 1; // output 1 = bit 0, etc.
        if (bit < 0 || bit > 3) return;

        if (value) _outputsByte |=  (byte)(1 << bit);
        else       _outputsByte &= (byte)~(1 << bit);

        Output1 = (_outputsByte & 1) != 0;
        Output2 = (_outputsByte & 2) != 0;
        Output3 = (_outputsByte & 4) != 0;
        Output4 = (_outputsByte & 8) != 0;
    }

    private readonly Stopwatch _statusTimer = Stopwatch.StartNew();

    public StepperMotor Motor1 { get; }
    public StepperMotor Motor2 { get; }
    public StepperMotor Motor3 { get; }
    public StepperMotor Motor4 { get; }

    public STB4100() {
        Motor1 = AddMotor(1, 1600, 1, 0);
        Motor2 = AddMotor(2, 1600, 1, 0);
        Motor3 = AddMotor(3, 1600, 1, 0);
        Motor4 = AddMotor(4, 400, 1, 0);
    }

    private bool Connect()
    {
        // Check if already connected
        if (_device != null && _stream != null && _stream.CanRead && _stream.CanWrite)
            return true;

        foreach (var d in DeviceList.Local.GetHidDevices())
        {
            Console.WriteLine($"{d.ProductID} {d.VendorID:X4}:{d.ProductID:X4}");
        }

        // Get the first available device that matches vendor and product identifications
        _device = DeviceList.Local.GetHidDeviceOrNull(VendorId, ProductId);
        if (_device == null)
        {
            Console.WriteLine("STB4100 could not be found");
            return false;
        }

        // Get that stream open!
        try
        {
            Console.WriteLine("STB4100 found! Opening Connection");
            _stream = _device.Open();
            connected = true;
            return _stream != null && _stream.CanRead && _stream.CanWrite;
        }
        catch (Exception ex)
        {
            Console.WriteLine("Error trying to connect to STB4100: " + ex);
            _stream = null;
            _device = null;
            connected = false;
            return false;
        }
    }

    private StepperMotor AddMotor(int pin, int stepsPerRev, double gearRatio, double startAngle)
    {
        var newMotor = new StepperMotor(pin, stepsPerRev, gearRatio, startAngle);
        _motors.Add(newMotor);
        return newMotor;
    }

    public void SetMotorTargets(double? m1Deg = null, double? m2Deg = null, double? m3Deg = null, double? m4Deg=null)
    {
        if (m1Deg.HasValue) Motor1.SetTarget(m1Deg.Value);
        if (m2Deg.HasValue) Motor2.SetTarget(m2Deg.Value);
        if (m3Deg.HasValue) Motor3.SetTarget(m3Deg.Value);
        if (m4Deg.HasValue) Motor4.SetTarget(m4Deg.Value);
    }

    public void OverwriteMotorTargets(double? m1Deg = null, double? m2Deg = null, double? m3Deg = null, double? m4Deg = null)
    {
        if (m1Deg.HasValue) Motor1.OverwriteTarget(m1Deg.Value);
        if (m2Deg.HasValue) Motor2.OverwriteTarget(m2Deg.Value);
        if (m3Deg.HasValue) Motor3.OverwriteTarget(m3Deg.Value);
        if (m4Deg.HasValue) Motor4.OverwriteTarget(m4Deg.Value);
    }

    public void Start()
    {
        Connect();

        new Thread(StatusLoop) { IsBackground = true }.Start();
        new Thread(ControlLoop) { IsBackground = true }.Start();
    }

    private void ControlLoop()
    {
        var sw = Stopwatch.StartNew();
        long nextTick = 0;

        double periodSec = 0.004; // 4ms
        long periodTicks = (long)(periodSec * Stopwatch.Frequency);

        while (true)
        {
            long now = sw.ElapsedTicks;

            if (now >= nextTick)
            {
                Loop();

                if (moving)
                {
                    periodSec = 0.005; // 5ms
                    periodTicks = (long)(periodSec * Stopwatch.Frequency);
                }
                else
                {
                    periodSec = 0.02; // 20ms
                    periodTicks = (long)(periodSec * Stopwatch.Frequency);
                }

                nextTick += periodTicks;
                
            }

            // Small spin wait to reduce CPU burn but keep timing tight
            Thread.SpinWait(50);
        }
    }


    private void StatusLoop()
    {
        while (true)
        {
            if (_statusTimer.ElapsedMilliseconds >= 20)
            {
                GetStatus();
                _statusTimer.Restart();
            }
            Thread.Sleep(1);
        }
    }

    public void Loop()
    {
        bool move = moving || _motors.Any(m => m.StepError != 0);

        if (move && !_jogging) _jog = true;
        if (!move && _jogging) _stopJog = true;

        if (_jog)
        {
            switch (_jogState)
            {
                case 0:
                    SendStatus(10);
                    _jogging = true;
                    _jogState++;
                    break;

                case 1:
                    SendCommand("Jog");
                    if (_stopJog) _jogState++;
                    break;

                case 2:
                    SendStatus(6);
                    if (status == 1)
                    {
                        _jogState = 0;
                        _jog = false;
                        _stopJog = false;
                        _jogging = false;
                    }
                    break;
            }
        }
        else
        {
            if (!_ready && _flip)
            {
                SendStatus(14);
                _flip = false;
            }
            else
            {
                SendStatus(3);
                _flip = true;
            }
        }

        if (_resetBit)
        {
            ResetInternal();
            _ready = true;
            _resetBit = false;
        }
    }

    public void Reset() => _resetBit = true;

    private void ResetInternal()
    {
        SendStatus(7);
        SendStatus(2);
        SendCommand("ClearSteps");
        SendCommand("Reset");
    }

    private void GetStatus()
    {
        var buffer = new byte[49];
        if (_stream is null)
            return;

        int bytesRead = _stream.Read(buffer);
        if (bytesRead != buffer.Length) return;

        status = buffer[1];

        var steps = new[]
        {
            BitTools.BytesToNumber(buffer[9..17]),
            BitTools.BytesToNumber(buffer[17..25]),
            BitTools.BytesToNumber(buffer[25..33]),
            BitTools.BytesToNumber(buffer[33..41])
        };

        for (int i = 0; i < 4; i++)
        {
            foreach (var m in _motors)
                if (m.Pin == i + 1)
                    m.Steps = steps[i];
        }

        var bits = BitTools.ByteToBits(buffer[42]);
        Input1 = bits[0];
        Input2 = bits[1];
        Input3 = bits[2];
        Input4 = bits[3];
    }

    private void SendCommand(string command)
    {
        var send = new List<byte>
        {
            3,
            (byte)_commands[command],
            0
        };

        send.AddRange(BitTools.NumberToBytes(_commandCount++, 2));

        if (command == "Jog" || command == "Move")
        {
            send.AddRange(BitTools.NumberToBytes(0, 4));

            for (int pin = 0; pin < 4; pin++)
            {
                int step = 0;
                foreach (var m in _motors)
                {
                    if (m.Pin == pin + 1)
                    {
                        step = m.StepError;
                        m.CurrentSteps += step;
                        if (m.InvertDirection)
                        {
                            step *= -1;
                        }
                    }
                }
                send.AddRange(BitTools.NumberToSignedBytes(step));
            }

            send.AddRange(BitTools.NumberToBytes(0, 2));
        }
        else if (command == "ClearSteps")
        {
            send.AddRange(BitTools.NumberToBytes(0, 16));
            send.AddRange(BitTools.NumberToBytes(15, 1));
            send.AddRange(BitTools.NumberToBytes(0, 5));
        }
        else if (command == "Reset")
        {
            send.AddRange([0, 0, 0, 0, 0, 0, 0, 0]);   // Bytes 5�12
            send.AddRange([0, 0, 0, 100, 5]);         // Bytes 13�17
            send.AddRange(BitTools.NumberToBytes(0, 9));              // Bytes 18�26
        }
        if (_stream is null)
            return;

        //Console.WriteLine("Command: " + string.Join(", ", send.Select(b => (int)b)));
        _stream.Write([.. send]);
    }

    private void SendStatus(int command)
    {
        var send = new List<byte>
        {
            2,
            (byte)command,
            0
        };

        send.AddRange(BitTools.NumberToBytes(_commandCount++, 2));
        send.Add(_outputsByte); // Outputs Byte (bits 0-3 = Output1-4)
        send.Add(0); // Padding?
        if (command == 14)
        {
            send.Add(5);
            send.Add(0);
        }
        else
        {
            send.Add(0);
            send.Add(0);
        }
            

        if (_stream is null)
            return;

        //Console.WriteLine("Status: " + string.Join(", ", send.Select(b => (int)b)));
        _stream.Write([.. send]);
    }
}
