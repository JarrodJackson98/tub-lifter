# Tub Lifter

Web app to control a Thomson Electrak HD linear actuator via CAN bus on a Raspberry Pi with a Sequent Microsystems MEGA-IND V2 hat.

## Hardware

- **Actuator**: Thomson Electrak HD (HD24B045-1000CN02MPS) — 24V, 4.5kN, 1000mm stroke, CAN bus
- **Hat**: Sequent Microsystems Industrial Automation V2 (STM32F072V8T6 + MCP2551 CAN transceiver)
- **SBC**: Raspberry Pi (any model with I2C)
- **Programmer**: ST-Link V2 or RPi I2C (for flashing custom firmware)

## How it works

The stock MEGA-IND firmware does not support CAN. This project includes custom STM32 firmware that turns the hat into an **I2C-to-CAN bridge**:

```
Browser  →  Flask (RPi)  →  I2C  →  STM32 firmware  →  CAN bus  →  Actuator
```

The actuator uses the **J1939** protocol at 500 kbit/s. The web UI has hold-to-move buttons (extend/retract while pressed) and go-to-position buttons (full extend/retract).

## Setup

### 1. Flash the custom firmware

Build the firmware first:

```bash
cd firmware
pip install platformio
pio run
```

#### Option A: Flash via I2C (no extra hardware needed)

The STM32F072 has a built-in ROM bootloader that supports I2C — the same bus already connected to the RPi. No ST-Link required.

**Updating** (custom firmware already installed):

```bash
cd firmware
python3 flash_i2c.py flash .pio/build/megaind_can/firmware.bin
```

The script automatically tells the running firmware to jump to the ROM bootloader, flashes, verifies, and boots the new firmware.

**First-time flash** (replacing stock Sequent firmware):

1. **Enter bootloader mode**: bridge BOOT0 to 3.3V. On the PCB, R75 (15K pull-down) connects BOOT0 (STM32 pin 91) to ground. Solder or hold a wire from the BOOT0 side of R75 to any 3.3V point (e.g. J5 pin 6).
2. **Power cycle** the hat (unplug/replug the RPi).
3. **Verify** the bootloader is running:
   ```bash
   i2cdetect -y 1
   # Should show 0x56 (bootloader) instead of 0x48 (firmware)
   ```
4. **Back up and flash**:
   ```bash
   cd firmware
   pip install smbus2
   python3 flash_i2c.py --no-enter-bootloader backup backup.bin
   python3 flash_i2c.py --no-enter-bootloader flash .pio/build/megaind_can/firmware.bin
   ```
5. **Remove** the BOOT0 bridge and **power cycle** to boot the new firmware.

#### Option B: Flash via ST-Link SWD

Connect an ST-Link V2 to the **J5 SWD header** on the hat:

| J5 Pin | Signal | ST-Link |
|--------|--------|---------|
| 1      | SCL (SWCLK) | SWCLK |
| 2      | SDA (SWDIO) | SWDIO |
| 3      | GND    | GND     |
| 6      | 3.3V   | 3.3V    |

```bash
# Back up (so you can restore later)
st-flash read backup.bin 0x08000000 0x10000

# Flash custom firmware
pio run --target upload
```

### 2. Verify the bridge

Enable I2C on the RPi (`sudo raspi-config` → Interface Options → I2C), then:

```bash
sudo apt install i2c-tools
i2cdetect -y 1
```

You should see `0x48` on the bus. The I2C address is `0x48 + jumper offset` (jumpers J101/J102 on the hat).

### 3. Run the web app

```bash
cd tub-lifter
uv run tub-lifter
```

Open `http://<rpi-ip>:5052` in a browser.

### 4. Wire the actuator

Connect the actuator's CAN bus wires to the **J10** connector on the hat:

| J10 Pin | Signal |
|---------|--------|
| 1       | CANH   |
| 2       | CANL   |

The actuator also needs 24V DC power on its power leads.

## CAN Protocol

The Thomson Electrak HD uses J1939 with these messages:

- **ACM** (Actuator Control Message) — PGN 61184, CAN ID `0x18EF1300`
  - Position: 14-bit (0.1mm/bit), Speed: 5-bit (5%/bit), Motion enable: 1-bit
- **AFM** (Actuator Feedback Message) — PGN 126720
  - Position, current, speed, error flags

Default actuator node ID: 19 (0x13), baud rate: 500 kbit/s.

## I2C Register Map (firmware)

| Register | R/W | Size | Description |
|----------|-----|------|-------------|
| `0x00` | W | 14 | Send CAN frame: `[ID3 ID2 ID1 ID0] [FLAGS] [DLC] [D0..D7]` |
| `0x10` | R | 15 | Last RX frame: `[STATUS] [ID3..ID0] [FLAGS] [DLC] [D0..D7]` |
| `0x20` | R | 2 | Info: `[FW_VERSION] [CAN_STATUS]` |
| `0xF0` | W | 1 | Enter bootloader: write `0xBE` to jump to ROM bootloader |

FLAGS bit 0 = extended (29-bit) ID. STATUS bit 0 = frame valid, bit 1 = overrun.

## Restoring original firmware

```bash
st-flash write backup.bin 0x08000000
```

Or use the Sequent Microsystems update tool:

```bash
git clone https://github.com/SequentMicrosystems/megaind-rpi.git
cd megaind-rpi/update && ./update 0
```

## Project structure

```
tub-lifter/
├── firmware/
│   ├── platformio.ini          # PlatformIO build config
│   ├── flash_i2c.py            # I2C bootloader flasher (no ST-Link needed)
│   └── src/main.c              # STM32 I2C-to-CAN bridge firmware
├── src/tub_lifter/
│   ├── actuator.py             # Electrak HD driver (I2C + J1939)
│   ├── app.py                  # Flask web server
│   └── templates/index.html    # Web UI
├── pyproject.toml
└── setup_can.sh                # RPi setup helper
```
