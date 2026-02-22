#!/bin/bash
# Setup for Tub Lifter on Raspberry Pi with Waveshare 2-CH CAN HAT (WS-17912).
#
# The HAT uses two MCP2515 CAN controllers with SN65HVD230 transceivers,
# exposed as SocketCAN interfaces (can0/can1) via kernel dtoverlays.
#
# Run this script once after installing the HAT, then reboot.

set -e

# --- 1. /boot/config.txt dtoverlays ---

# Newer Raspberry Pi OS uses /boot/firmware/config.txt
if [ -f /boot/firmware/config.txt ]; then
    CONFIG="/boot/firmware/config.txt"
else
    CONFIG="/boot/config.txt"
fi
echo "Checking $CONFIG for MCP2515 dtoverlays..."

OVERLAYS_NEEDED=false

if ! grep -q "^dtoverlay=mcp2515-can0" "$CONFIG" 2>/dev/null; then
    OVERLAYS_NEEDED=true
fi

if [ "$OVERLAYS_NEEDED" = true ]; then
    echo "Adding MCP2515 dtoverlays to $CONFIG..."
    sudo tee -a "$CONFIG" > /dev/null <<'EOF'

# Waveshare 2-CH CAN HAT
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23,spimaxfrequency=2000000
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=25,spimaxfrequency=2000000
EOF
    echo "Overlays added. A reboot is required before CAN interfaces appear."
else
    echo "MCP2515 overlays already present."
fi

# --- 2. Install can-utils for diagnostics ---

if ! command -v candump &> /dev/null; then
    echo "Installing can-utils..."
    sudo apt-get install -y can-utils
else
    echo "can-utils already installed."
fi

# --- 3. Bring up can0 at 500 kbit/s ---

echo ""
echo "Bringing up can0 at 500 kbit/s..."
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0

echo ""
echo "CAN bus ready. Test with:  candump can0"
echo "Start the web app with:    uv run tub-lifter"
