#!/bin/bash
# Setup for Tub Lifter on Raspberry Pi.
#
# The MEGA-IND V2 hat communicates with the RPi via I2C. Our custom
# firmware on the STM32F072V8T6 bridges I2C <-> CAN at 500 kbit/s.
#
# Prerequisites:
#   1. Flash the custom firmware (firmware/ dir) via ST-Link SWD (J5 header)
#   2. I2C must be enabled on the RPi (sudo raspi-config -> Interface Options)
#
# Flashing firmware (requires ST-Link V2 connected to J5):
#   cd firmware && pio run --target upload

set -e

echo "Checking I2C bus..."
if ! command -v i2cdetect &> /dev/null; then
    echo "Installing i2c-tools..."
    sudo apt-get install -y i2c-tools
fi

echo "Scanning I2C bus 1 for CAN bridge (expect 0x48)..."
i2cdetect -y 1

echo ""
echo "If you see 0x48 on the bus, the CAN bridge firmware is running."
echo "Start the web app with: uv run tub-lifter"
