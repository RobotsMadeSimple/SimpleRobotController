#!/bin/bash
set -e

REPO="RobotsMadeSimple/SimpleRobotController"
INSTALL_DIR="/usr/local/bin"
SERVICE_NAME="robot-controller"
BINARY_NAME="SimpleRobotController"
STATE_DIR="/etc/robot-controller"
CHANNEL_FILE="$STATE_DIR/channel"

# ---------------------------------------------------------------------------
# Args (all optional):
#   (none)                    reuse the channel this box is already on — sticky,
#                             defaulting to stable on a fresh install. A bare
#                             re-run updates within the current channel.
#   --channel stable          production; tracks releases/latest (never a prerelease)
#   --channel experimental    newest PR prerelease, resolved via the GitHub API
#   --tag exp-prN             a specific experimental prerelease
#
# Piped form:
#   curl -sSL .../install.sh | sudo bash -s -- --tag exp-pr123
# ---------------------------------------------------------------------------
CHANNEL=""
TAG=""
while [ $# -gt 0 ]; do
    case "$1" in
        --channel) CHANNEL="$2"; shift 2 ;;
        --tag)     TAG="$2"; CHANNEL="experimental"; shift 2 ;;
        *) echo "Unknown option: $1"
           echo "Usage: install.sh [--channel stable|experimental] [--tag exp-prN]"
           exit 1 ;;
    esac
done

need_cmd() {
    if ! command -v "$1" &> /dev/null; then
        echo "$1 not found, installing..."
        apt-get install -y "$1"
    fi
}

need_cmd curl

# ---- Resolve channel + download URL ---------------------------------------
# No explicit channel/tag → fall back to the saved (sticky) channel.
if [ -z "$CHANNEL" ]; then
    if [ -f "$CHANNEL_FILE" ]; then
        CHANNEL=$(sed -n '1p' "$CHANNEL_FILE")
        TAG=$(sed -n '2p' "$CHANNEL_FILE")
    else
        CHANNEL="stable"
    fi
fi

if [ "$CHANNEL" = "experimental" ]; then
    need_cmd jq
    if [ -z "$TAG" ]; then
        echo "Resolving newest experimental prerelease..."
        TAG=$(curl -sSL "https://api.github.com/repos/$REPO/releases" \
              | jq -r 'map(select(.prerelease)) | .[0].tag_name // empty')
        if [ -z "$TAG" ]; then
            echo "No experimental prereleases are currently published. Aborting."
            exit 1
        fi
    fi
    DOWNLOAD_URL="https://github.com/$REPO/releases/download/$TAG/$BINARY_NAME"
    echo "Channel: EXPERIMENTAL ($TAG)"
elif [ "$CHANNEL" = "stable" ]; then
    TAG=""
    DOWNLOAD_URL="https://github.com/$REPO/releases/latest/download/$BINARY_NAME"
    echo "Channel: stable (latest release)"
else
    echo "Unknown channel: '$CHANNEL' (expected 'stable' or 'experimental')"
    exit 1
fi

echo "Installing Robot Controller..."

# Stop the service if it is already running (update scenario)
systemctl stop "$SERVICE_NAME" 2>/dev/null || true

# Download the selected binary (-f: fail on 404 instead of saving an error page)
curl -fSL "$DOWNLOAD_URL" -o "$INSTALL_DIR/$BINARY_NAME"
chmod +x "$INSTALL_DIR/$BINARY_NAME"

# Remember the channel so a bare re-run updates within the same channel
mkdir -p "$STATE_DIR"
printf '%s\n%s\n' "$CHANNEL" "$TAG" > "$CHANNEL_FILE"

echo "Creating systemd service..."

cat > /etc/systemd/system/$SERVICE_NAME.service << EOF
[Unit]
Description=Robot Controller
After=network.target

[Service]
ExecStart=$INSTALL_DIR/$BINARY_NAME
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"

echo ""
echo "Robot Controller installed and running!"
echo "  Channel: $CHANNEL${TAG:+ ($TAG)}"
echo "  Status:  sudo systemctl status $SERVICE_NAME"
echo "  Logs:    sudo journalctl -u $SERVICE_NAME -f"
echo "  Stop:    sudo systemctl stop $SERVICE_NAME"
if [ "$CHANNEL" = "experimental" ]; then
    echo ""
    echo "  !! This box is running EXPERIMENTAL PR code — not for production."
    echo "     Return it to production with:"
    echo "       curl -sSL https://raw.githubusercontent.com/$REPO/main/install.sh | sudo bash -s -- --channel stable"
fi
