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

To update, simply re-run the install command:

```bash
curl -sSL https://raw.githubusercontent.com/RobotsMadeSimple/SimpleRobotController/main/install.sh | sudo bash
```

A bare re-run updates **within the channel the box is already on** (see below).
A fresh box, or one on the stable channel, gets the latest production release.

---

## Experimental / PR Test Builds

Every push to an open pull request publishes a self-contained Linux binary as a
GitHub **prerelease** tagged `exp-pr<N>` (where `<N>` is the PR number). Because
`releases/latest` never resolves to a prerelease, **production boxes are never
affected** — only a box you explicitly switch will run experimental code.

### Switch a box to a specific PR

```bash
curl -sSL https://raw.githubusercontent.com/RobotsMadeSimple/SimpleRobotController/main/install.sh | sudo bash -s -- --tag exp-pr123
```

The box remembers this channel, so a later bare re-run pulls the newest build of
that same PR (handy while you iterate on it).

### Switch a box to the newest experimental build (any PR)

```bash
curl -sSL https://raw.githubusercontent.com/RobotsMadeSimple/SimpleRobotController/main/install.sh | sudo bash -s -- --channel experimental
```

### Return a box to production

```bash
curl -sSL https://raw.githubusercontent.com/RobotsMadeSimple/SimpleRobotController/main/install.sh | sudo bash -s -- --channel stable
```

| Flag | Effect |
|---|---|
| *(none)* | Update within the box's current channel (stable by default) |
| `--channel stable` | Latest production release; rejoins normal users |
| `--channel experimental` | Newest PR prerelease across all open PRs |
| `--tag exp-pr<N>` | A specific PR's build |

The current channel is recorded in `/etc/robot-controller/channel`. Experimental
prereleases are deleted automatically when their PR closes or merges, so a box
left on a merged PR's tag will fail to update until you point it back at
`--channel stable` (or another live tag).

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
