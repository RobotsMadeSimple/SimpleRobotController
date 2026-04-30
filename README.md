# SimpleRobotController

The core control platform for **ASTRO** — an open-source, workbench-sized 4-axis robot arm built for pick and place, machine tending, and small business automation. Part of the [Robots Made Simple](https://github.com/RobotsMadeSimple) ecosystem.

---

## What It Does

SimpleRobotController is a C# ASP.NET Core WebSocket server that runs on the robot's onboard computer. It handles:

- **Motion control** — jogging, homing, teaching points, running programs
- **IO management** — Arduino Nano edge devices, STB4100 IO board
- **Status broadcasting** — real-time robot state pushed to connected clients
- **Program execution** — create, store, and run automation sequences
- **Device discovery** — mDNS/Zeroconf so the app finds the robot automatically on the network
- **Status light** — NeoPixel strip reflects robot state at a glance

All robots, workstations, and tooling in the Robots Made Simple ecosystem share this platform and communicate over the same WebSocket protocol.

---

## Requirements

- [.NET 10 SDK](https://dotnet.microsoft.com/download/dotnet/10.0)
- Linux, Windows, or macOS
- A compatible robot (ASTRO) or a supported IO device

---

## Installation

### Linux

Run the install script to download the latest release, install it, and set it up as a systemd service that starts automatically on boot:

**1. Download and run the install script:**
```bash
curl -sSL https://raw.githubusercontent.com/RobotsMadeSimple/SimpleRobotController/main/install.sh | sudo bash
```

**2. Verify it's running:**
```bash
sudo systemctl status robot-controller
```

**3. Check the logs:**
```bash
sudo journalctl -u robot-controller -f
```

Once installed the controller will start automatically on every boot. Useful commands:

| Command | Description |
|---|---|
| `sudo systemctl status robot-controller` | Check if running |
| `sudo journalctl -u robot-controller -f` | View live logs |
| `sudo systemctl stop robot-controller` | Stop the controller |
| `sudo systemctl restart robot-controller` | Restart the controller |

### Windows

Download the latest `SimpleRobotController.exe` from the [Releases](https://github.com/RobotsMadeSimple/SimpleRobotController/releases/latest) page and run it directly — no installation needed.

### From Source

```bash
git clone https://github.com/RobotsMadeSimple/SimpleRobotController.git
cd SimpleRobotController/RobotControl
dotnet run
```

The server starts on port `9000`. Connect using [SimpleRobotApp](https://github.com/RobotsMadeSimple/SimpleRobotApp) or any WebSocket client.

---

## Configuration

**`nano_config.json`** — Configure connected Arduino Nano edge devices:

```json
[
  {
    "Id": "ROBOT_NANO_001",
    "Name": "Main IO",
    "Pins": [
      { "Pin": 2, "Type": "Input", "Name": "Part Sensor" },
      { "Pin": 7, "Type": "Output", "Name": "Clamp" },
      { "Pin": 6, "Type": "Neopixel", "Name": "Status Light", "PixelCount": 12 }
    ]
  }
]
```

---

## WebSocket Protocol

All communication is line-based JSON over WebSocket on port `9000`.

| Command | Description |
|---|---|
| `GetStatus` | Get current robot state |
| `RunProgram` | Execute a saved program |
| `StopProgram` | Stop current program |
| `JogAxis` | Manually move an axis |
| `Home` | Home all axes |
| `GetIO` | Get all IO device states |
| `SetNanoOutput` | Set an Arduino Nano output pin |
| `SetNeoPixel` | Set NeoPixel strip colors |
| `ConfigureNanoPin` | Configure a Nano pin type |
| `RenameNanoPin` | Rename a Nano pin |
| `SetSTBOutput` | Set an STB4100 output |

---

## Hardware Support

| Device | Description |
|---|---|
| ASTRO Robot Arm | 4-axis, 400mm cylindrical envelope |
| Arduino Nano | Edge IO device — configurable inputs, outputs, NeoPixel |
| STB4100 | Robot IO board — 4 inputs, 4 outputs |

---

## Ecosystem

Robots Made Simple is a fully open-source automation ecosystem. SimpleRobotController is the shared platform across all devices:

- **[SimpleRobotApp](https://github.com/RobotsMadeSimple/SimpleRobotApp)** — Mobile and web app for controlling the robot
- **[ArduinoNano](https://github.com/RobotsMadeSimple/ArduinoNano)** — Edge device firmware
- Workstations, tooling, and more — coming soon

---

## License

MIT License — see [LICENSE](LICENSE) for details.

Copyright (c) 2026 RobotsMadeSimple
