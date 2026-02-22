# Tub Lifter

Web app to control a Thomson Electrak HD linear actuator via CAN bus on a Raspberry Pi with a Waveshare 2-CH CAN HAT.

## Hardware

- **Actuator**: Thomson Electrak HD (HD24B045-1000CN02MPS) — 24V, 4.5kN, 1000mm stroke, CAN bus
- **HAT**: Waveshare 2-CH CAN HAT (WS-17912) — MCP2515 + SN65HVD230, isolated, SPI interface
- **SBC**: Raspberry Pi (any model with SPI + 40-pin header)

## How it works

The Waveshare HAT provides two isolated CAN channels via MCP2515 controllers. Linux kernel dtoverlays expose them as standard SocketCAN interfaces (`can0`/`can1`). The Python app uses `python-can` to talk directly over SocketCAN — no custom firmware needed.

```
Browser  →  Flask (RPi)  →  SocketCAN (python-can)  →  MCP2515 (kernel)  →  CAN bus  →  Actuator
```

The actuator uses the **J1939** protocol at 500 kbit/s. The web UI has hold-to-move buttons (extend/retract while pressed) and go-to-position buttons (full extend/retract).

## Setup

### 1. Install the HAT and configure overlays

Mount the Waveshare 2-CH CAN HAT on the Pi's 40-pin header, then run the setup script:

```bash
chmod +x setup_can.sh
sudo ./setup_can.sh
sudo reboot
```

This adds the MCP2515 dtoverlays to `/boot/config.txt` (oscillator=16MHz, INT on GPIO 23/25) and installs `can-utils`.

After reboot, bring up the CAN interface:

```bash
sudo ip link set can0 up type can bitrate 500000
```

Verify:

```bash
ip link show can0
candump can0   # should show frames once the actuator is powered
```

### 2. Wire the actuator

Connect the actuator's CAN bus wires to the HAT's **CH0** terminal block:

| Terminal | Signal |
|----------|--------|
| CAN_H    | CAN High |
| CAN_L    | CAN Low  |
| GND      | Ground (optional, for shielded cable) |

The actuator also needs 24V DC power on its power leads.

### 3. Run the web app

```bash
cd tub-lifter
uv run tub-lifter
```

Open `http://<rpi-ip>:5052` in a browser.

## Configuration

| Environment Variable | Default | Description |
|---------------------|---------|-------------|
| `CAN_CHANNEL`       | `can0`  | SocketCAN interface name |

## CAN Protocol

The Thomson Electrak HD uses J1939 with these messages:

- **ACM** (Actuator Control Message) — PGN 61184, CAN ID `0x18EF1300`
  - Position: 14-bit (0.1mm/bit), Speed: 5-bit (5%/bit), Motion enable: 1-bit
- **AFM** (Actuator Feedback Message) — PGN 126720
  - Position, current, speed, error flags

Default actuator node ID: 19 (0x13), baud rate: 500 kbit/s.

## Project structure

```
tub-lifter/
├── src/tub_lifter/
│   ├── actuator.py             # Electrak HD driver (SocketCAN + J1939)
│   ├── app.py                  # Flask web server
│   └── templates/index.html    # Web UI
├── pyproject.toml
├── setup_can.sh                # RPi + Waveshare HAT setup
└── README.md
```
