using HidSharp;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
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

    public int status;
    private int _commandCount;
    private bool _ready;
    private bool _jogging;
    private bool _stopJog;
    public int _jogState;
    private bool _jog;
    private bool _flip;
    private bool _resetBit;

    private readonly Stopwatch _statusTimer = Stopwatch.StartNew();

    public STB4100() {}

    private void Connect()
    {
        _device = DeviceList.Local.GetHidDeviceOrNull(VendorId, ProductId)
            ?? throw new Exception("STB4100 not found");

        _stream = _device.Open();
    }

    public StepperMotor AddMotor(string name, int stepsPerRev, double gearRatio, int pin, double startAngle)
    {
        var newMotor = new StepperMotor(name, stepsPerRev, gearRatio, pin, startAngle);
        _motors.Add(newMotor);
        return newMotor;
    }

    public void Start()
    {
        Connect();
        new Thread(StatusLoop) { IsBackground = true }.Start();
    }

    private void StatusLoop()
    {
        while (true)
        {
            if (_statusTimer.ElapsedMilliseconds >= 40)
            {
                GetStatus();
                _statusTimer.Restart();
            }
            Thread.Sleep(1);
        }
    }

    public void Loop(bool moving = false)
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
            send.AddRange([34, 0, 34, 0, 34, 0, 34, 0]);   // Bytes 5–12
            send.AddRange([68, 0, 255, 100, 5]);         // Bytes 13–17
            send.AddRange(BitTools.NumberToBytes(0, 9));              // Bytes 18–26
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
        send.Add(0);
        send.Add(0);
        send.Add(0);
        send.Add(0);

        if (_stream is null)
            return;

        //Console.WriteLine("Status: " + string.Join(", ", send.Select(b => (int)b)));
        _stream.Write([.. send]);
    }
}
