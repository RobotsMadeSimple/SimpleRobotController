# Linux Installation Guide

This guide walks you through installing SimpleRobotController on a Linux computer.

---

## Requirements

- Ubuntu 18.04 or newer (or any modern Linux distro with systemd)
- Internet connection for the initial download
- `sudo` access

---

## Install

Run the following command in a terminal:

```bash
curl -sSL https://raw.githubusercontent.com/RobotsMadeSimple/SimpleRobotController/main/install.sh | sudo bash
```

This will:

1. Download the latest `SimpleRobotController` binary from GitHub Releases
2. Install it to `/usr/local/bin`
3. Create a systemd service so it starts automatically on every boot
4. Start the controller immediately

---

## Verify It's Running

Once the install completes, check that the controller is running:

```bash
sudo systemctl status robot-controller
```

You should see `active (running)` in the output. The controller is now listening on port `9000`.

---

## Useful Commands

| Command | Description |
|---|---|
| `sudo systemctl status robot-controller` | Check if running |
| `sudo journalctl -u robot-controller -f` | View live logs |
| `sudo systemctl stop robot-controller` | Stop the controller |
| `sudo systemctl start robot-controller` | Start the controller |
| `sudo systemctl restart robot-controller` | Restart the controller |
| `sudo systemctl disable robot-controller` | Disable autostart on boot |

---

## Updating

To update to the latest version, simply re-run the install command:

```bash
curl -sSL https://raw.githubusercontent.com/RobotsMadeSimple/SimpleRobotController/main/install.sh | sudo bash
```

It will download the latest release and restart the service automatically.

---

## Uninstall

To remove the controller and service:

```bash
sudo systemctl stop robot-controller
sudo systemctl disable robot-controller
sudo rm /etc/systemd/system/robot-controller.service
sudo rm /usr/local/bin/SimpleRobotController
sudo systemctl daemon-reload
```

---

## Connecting

Once running, open the [SimpleRobotApp](https://github.com/RobotsMadeSimple/SimpleRobotApp) on your phone or computer. It will automatically discover the controller on your local network. If discovery fails, enter the robot's IP address manually and connect on port `9000`.
