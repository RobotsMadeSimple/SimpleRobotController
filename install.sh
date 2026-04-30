#!/bin/bash
set -e

INSTALL_DIR="/usr/local/bin"
SERVICE_NAME="robot-controller"
BINARY_NAME="SimpleRobotController"
DOWNLOAD_URL="https://github.com/RobotsMadeSimple/SimpleRobotController/releases/latest/download/SimpleRobotController"

echo "Installing Robot Controller..."

# Ensure curl is available
if ! command -v curl &> /dev/null; then
    echo "curl not found, installing..."
    apt-get install -y curl
fi

# Download the latest binary
curl -sSL "$DOWNLOAD_URL" -o "$INSTALL_DIR/$BINARY_NAME"
chmod +x "$INSTALL_DIR/$BINARY_NAME"

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
systemctl enable $SERVICE_NAME
systemctl start $SERVICE_NAME

echo ""
echo "Robot Controller installed and running!"
echo "  Status:  sudo systemctl status $SERVICE_NAME"
echo "  Logs:    sudo journalctl -u $SERVICE_NAME -f"
echo "  Stop:    sudo systemctl stop $SERVICE_NAME"
