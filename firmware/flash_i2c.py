#!/usr/bin/env python3
"""
Flash STM32F072 firmware via I2C bootloader on Raspberry Pi.

The STM32F072V8T6 on the MEGA-IND V2 hat has a built-in ROM bootloader
that supports I2C1 (PB6/PB7 — the same bus connected to the RPi).

If the custom CAN bridge firmware (v2+) is already running, the script
can trigger a software jump to the bootloader — no BOOT0 bridge or
power cycling needed. Just run:

  python3 flash_i2c.py flash firmware.bin

For the very first flash (replacing stock Sequent firmware), you need
to enter bootloader mode manually:
  1. Bridge BOOT0 to 3.3V (R75 pad near STM32 pin 91 to e.g. J5 pin 6)
  2. Power cycle the hat
  3. Verify with: i2cdetect -y 1  (should show 0x56 instead of 0x48)
  4. Run this script with --no-enter-bootloader
  5. Remove the BOOT0 bridge and power cycle

Bootloader I2C address: 0x56 (7-bit), per AN2606 for STM32F072
Protocol: ST AN4221 (I2C bootloader protocol)

Usage:
  python3 flash_i2c.py flash firmware.bin        Flash (auto-enters bootloader)
  python3 flash_i2c.py backup backup.bin         Read flash to file
  python3 flash_i2c.py verify firmware.bin       Verify flash matches file
  python3 flash_i2c.py info                      Show bootloader info
  python3 flash_i2c.py erase                     Mass erase flash
  python3 flash_i2c.py go                        Jump to firmware (0x08000000)

After building with PlatformIO, the firmware binary is at:
  .pio/build/megaind_can/firmware.bin
"""

import argparse
import sys
import time

from smbus2 import SMBus, i2c_msg

# I2C addresses
FIRMWARE_ADDR = 0x48     # CAN bridge firmware I2C address
BOOTLOADER_ADDR = 0x56   # STM32F072 ROM bootloader address (AN2606)

# Firmware register to trigger bootloader entry
REG_ENTER_BOOTLOADER = 0xF0
BOOTLOADER_MAGIC = 0xBE

# Flash constants
FLASH_BASE = 0x08000000
FLASH_SIZE = 64 * 1024   # 64KB for STM32F072V8
CHUNK_SIZE = 256          # Max bytes per Write Memory command

ACK = 0x79
NACK = 0x1F


def enter_bootloader_via_firmware(bus_num=1, fw_addr=FIRMWARE_ADDR):
    """Send the 'enter bootloader' command to the running CAN bridge firmware.

    The firmware writes 0xBE to register 0xF0, which causes it to deinit
    all peripherals and jump to the ROM bootloader at 0x1FFFC800.
    The I2C address changes from 0x48 (firmware) to 0x56 (bootloader).
    """
    bus = SMBus(bus_num)
    try:
        bus.write_byte_data(fw_addr, REG_ENTER_BOOTLOADER, BOOTLOADER_MAGIC)
    except OSError:
        return False
    finally:
        bus.close()
    return True


def wait_for_bootloader(bus_num=1, addr=BOOTLOADER_ADDR, timeout=3.0):
    """Poll until the bootloader responds at its I2C address."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            bus = SMBus(bus_num)
            msg = i2c_msg.read(addr, 1)
            bus.i2c_rdwr(msg)
            bus.close()
            return True
        except OSError:
            pass
        time.sleep(0.05)
    return False


class STM32I2CBootloader:
    """STM32 I2C bootloader client implementing the AN4221 protocol.

    Each bootloader command follows the pattern:
      1. Write [CMD, ~CMD]           -> wait for ACK
      2. Write parameters + checksum -> wait for ACK
      3. Read response data (if any)
    """

    def __init__(self, bus_num=1, addr=BOOTLOADER_ADDR):
        self.bus = SMBus(bus_num)
        self.addr = addr

    def close(self):
        self.bus.close()

    def _write(self, data):
        """Send bytes to the bootloader in a single I2C write transaction."""
        msg = i2c_msg.write(self.addr, data)
        self.bus.i2c_rdwr(msg)

    def _read(self, count):
        """Read bytes from the bootloader in a single I2C read transaction."""
        msg = i2c_msg.read(self.addr, count)
        self.bus.i2c_rdwr(msg)
        return list(msg)

    def _wait_ack(self, timeout=5.0):
        """Read a single ACK/NACK byte, retrying on I2C bus errors.

        The bootloader may stretch the clock or NACK its address while busy.
        We retry reads until we get a response or the timeout expires.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                data = self._read(1)
                if data[0] == ACK:
                    return True
                if data[0] == NACK:
                    return False
            except OSError:
                pass
            time.sleep(0.01)
        raise TimeoutError("Bootloader did not respond")

    def _send_cmd(self, cmd):
        """Send a command byte and its complement, then wait for ACK."""
        self._write([cmd, cmd ^ 0xFF])
        if not self._wait_ack():
            raise RuntimeError(f"Command 0x{cmd:02X} NACKed by bootloader")

    def _send_addr(self, addr):
        """Send a 4-byte address with XOR checksum, then wait for ACK."""
        b = [(addr >> 24) & 0xFF, (addr >> 16) & 0xFF,
             (addr >> 8) & 0xFF, addr & 0xFF]
        self._write(b + [b[0] ^ b[1] ^ b[2] ^ b[3]])
        if not self._wait_ack():
            raise RuntimeError(f"Address 0x{addr:08X} NACKed")

    # -- Bootloader commands --

    def get(self):
        """Get command (0x00): returns (version, [supported_commands])."""
        self._send_cmd(0x00)
        n = self._read(1)[0]          # number of following bytes - 1
        resp = self._read(n + 1)      # version byte + n command bytes
        self._read(1)                 # final ACK
        return resp[0], resp[1:]

    def get_id(self):
        """Get ID command (0x02): returns chip product ID."""
        self._send_cmd(0x02)
        n = self._read(1)[0]          # number of PID bytes - 1
        pid_bytes = self._read(n + 1)
        self._read(1)                 # final ACK
        pid = 0
        for b in pid_bytes:
            pid = (pid << 8) | b
        return pid

    def read_memory(self, addr, length):
        """Read Memory command (0x11): read up to 256 bytes."""
        self._send_cmd(0x11)
        self._send_addr(addr)
        n = length - 1  # bootloader expects N-1 (0 = 1 byte, 255 = 256 bytes)
        self._write([n, n ^ 0xFF])
        if not self._wait_ack():
            raise RuntimeError("Read Memory: length NACKed")
        return self._read(length)

    def write_memory(self, addr, data):
        """Write Memory command (0x31): write up to 256 bytes."""
        if len(data) > 256:
            raise ValueError("Maximum 256 bytes per Write Memory command")
        self._send_cmd(0x31)
        self._send_addr(addr)
        n = len(data) - 1
        xor = n
        for b in data:
            xor ^= b
        self._write([n] + list(data) + [xor & 0xFF])
        if not self._wait_ack(timeout=10):
            raise RuntimeError(f"Write Memory at 0x{addr:08X} failed")

    def extended_erase_mass(self):
        """Extended Erase command (0x44): mass erase all flash pages."""
        self._send_cmd(0x44)
        # Mass erase: [0xFF, 0xFF] + checksum (0xFF ^ 0xFF = 0x00)
        self._write([0xFF, 0xFF, 0x00])
        if not self._wait_ack(timeout=30):
            raise RuntimeError("Mass erase failed or timed out")

    def go(self, addr=FLASH_BASE):
        """Go command (0x21): jump to address and begin execution."""
        self._send_cmd(0x21)
        self._send_addr(addr)


# -- CLI commands --

def _progress(label, current, total):
    pct = current * 100 // total
    print(f"\r{label}: {pct}% ({current}/{total} bytes)", end="", flush=True)


def cmd_info(bl, _args):
    """Display bootloader version, supported commands, and chip ID."""
    print("Connecting to STM32 bootloader at 0x{:02X}...".format(bl.addr))
    version, cmds = bl.get()
    print(f"Bootloader version: {version >> 4}.{version & 0xF}")
    print(f"Supported commands: {' '.join(f'0x{c:02X}' for c in cmds)}")
    pid = bl.get_id()
    print(f"Chip ID: 0x{pid:04X}", end="")
    if pid == 0x0448:
        print("  (STM32F072xx)")
    else:
        print()


def cmd_erase(bl, _args):
    """Mass erase all flash."""
    print("Mass erasing flash...", end="", flush=True)
    bl.extended_erase_mass()
    print(" done.")


def cmd_flash(bl, args):
    """Erase, write, and verify a firmware binary."""
    with open(args.file, "rb") as f:
        fw = f.read()
    if len(fw) > FLASH_SIZE:
        sys.exit(f"Error: firmware ({len(fw)} bytes) exceeds flash ({FLASH_SIZE} bytes)")

    print(f"Flashing {len(fw)} bytes from {args.file}")

    # Erase
    print("Erasing...", end="", flush=True)
    bl.extended_erase_mass()
    print(" done.")

    # Write
    total = len(fw)
    for off in range(0, total, CHUNK_SIZE):
        chunk = fw[off:off + CHUNK_SIZE]
        bl.write_memory(FLASH_BASE + off, chunk)
        _progress("Writing", off + len(chunk), total)
    print()

    # Verify
    for off in range(0, total, CHUNK_SIZE):
        size = min(CHUNK_SIZE, total - off)
        got = bytes(bl.read_memory(FLASH_BASE + off, size))
        expected = fw[off:off + size]
        if got != expected:
            sys.exit(f"\nVerification FAILED at 0x{FLASH_BASE + off:08X}")
        _progress("Verifying", off + size, total)
    print()

    print("Flash complete and verified.")

    # Boot the new firmware
    if not args.no_go:
        print("Starting firmware...", end="", flush=True)
        try:
            bl.go(FLASH_BASE)
            print(" done.")
        except Exception:
            print(" (issue Go command manually or power cycle)")


def cmd_backup(bl, args):
    """Read entire flash contents to a binary file."""
    print(f"Reading {FLASH_SIZE} bytes from flash...")
    data = bytearray()
    for off in range(0, FLASH_SIZE, CHUNK_SIZE):
        size = min(CHUNK_SIZE, FLASH_SIZE - off)
        data.extend(bl.read_memory(FLASH_BASE + off, size))
        _progress("Reading", off + size, FLASH_SIZE)
    print()
    with open(args.file, "wb") as f:
        f.write(data)
    print(f"Saved to {args.file}")


def cmd_verify(bl, args):
    """Verify flash contents against a binary file."""
    with open(args.file, "rb") as f:
        fw = f.read()
    total = len(fw)
    print(f"Verifying {total} bytes...")
    for off in range(0, total, CHUNK_SIZE):
        size = min(CHUNK_SIZE, total - off)
        got = bytes(bl.read_memory(FLASH_BASE + off, size))
        expected = fw[off:off + size]
        if got != expected:
            for i in range(size):
                if got[i] != expected[i]:
                    sys.exit(
                        f"Mismatch at 0x{FLASH_BASE + off + i:08X}: "
                        f"flash=0x{got[i]:02X} file=0x{expected[i]:02X}"
                    )
        _progress("Verifying", off + size, total)
    print("\nVerification passed.")


def cmd_go(bl, _args):
    """Jump to firmware entry point."""
    print(f"Jumping to 0x{FLASH_BASE:08X}...")
    bl.go(FLASH_BASE)
    print("Done. The bootloader has handed off to firmware.")


def try_enter_bootloader(args):
    """Try to switch from running firmware to bootloader mode.

    Returns True if bootloader is ready (either already was, or we
    successfully triggered the jump from firmware).
    """
    # First check if bootloader is already responding
    try:
        bus = SMBus(args.bus)
        msg = i2c_msg.read(BOOTLOADER_ADDR, 1)
        bus.i2c_rdwr(msg)
        bus.close()
        print("Bootloader already active at 0x{:02X}.".format(BOOTLOADER_ADDR))
        return True
    except OSError:
        pass

    # Try to tell the running firmware to jump to bootloader
    fw_addr = int(args.fw_addr, 0) if isinstance(args.fw_addr, str) else args.fw_addr
    print("Sending enter-bootloader command to firmware at "
          "0x{:02X}...".format(fw_addr), end="", flush=True)
    if not enter_bootloader_via_firmware(args.bus, fw_addr):
        print(" failed (firmware not responding).")
        return False

    # Wait for bootloader to come up
    time.sleep(0.1)
    if wait_for_bootloader(args.bus, BOOTLOADER_ADDR, timeout=3.0):
        print(" ok.")
        return True
    else:
        print(" bootloader did not appear.")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Flash STM32F072 firmware via I2C bootloader (AN4221)",
        epilog=(
            "The script automatically tells the running firmware to jump\n"
            "to the ROM bootloader. No BOOT0 bridge needed (firmware v2+).\n"
            "\n"
            "For first-time flash (replacing stock Sequent firmware):\n"
            "  1. Bridge BOOT0 (R75) to 3.3V and power cycle\n"
            "  2. Run with --no-enter-bootloader"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "command",
        choices=["flash", "backup", "verify", "info", "erase", "go"],
        help="Operation to perform",
    )
    parser.add_argument(
        "file", nargs="?",
        help="Binary file path (required for flash/backup/verify)",
    )
    parser.add_argument(
        "--bus", type=int, default=1,
        help="I2C bus number (default: 1)",
    )
    parser.add_argument(
        "--addr", type=lambda x: int(x, 0), default=BOOTLOADER_ADDR,
        help=f"Bootloader I2C address (default: 0x{BOOTLOADER_ADDR:02X})",
    )
    parser.add_argument(
        "--fw-addr", default="0x48",
        help="Firmware I2C address for enter-bootloader command (default: 0x48)",
    )
    parser.add_argument(
        "--no-enter-bootloader", action="store_true",
        help="Skip the automatic enter-bootloader step (use when BOOT0 is "
             "already bridged or bootloader is already active)",
    )
    parser.add_argument(
        "--no-go", action="store_true",
        help="Don't issue Go command after flashing (requires manual power cycle)",
    )
    args = parser.parse_args()

    if args.command in ("flash", "backup", "verify") and not args.file:
        parser.error(f"'{args.command}' requires a file argument")

    # Enter bootloader mode (unless skipped or just issuing Go)
    if args.command != "go" and not args.no_enter_bootloader:
        if not try_enter_bootloader(args):
            sys.exit(
                "Error: Could not reach bootloader.\n"
                "If this is a first-time flash, bridge BOOT0 to 3.3V,\n"
                "power cycle, and run with --no-enter-bootloader."
            )

    handlers = {
        "info": cmd_info,
        "erase": cmd_erase,
        "flash": cmd_flash,
        "backup": cmd_backup,
        "verify": cmd_verify,
        "go": cmd_go,
    }

    bl = STM32I2CBootloader(bus_num=args.bus, addr=args.addr)
    try:
        handlers[args.command](bl, args)
    except TimeoutError as e:
        sys.exit(f"Error: {e}")
    except RuntimeError as e:
        sys.exit(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nAborted.")
        sys.exit(1)
    finally:
        bl.close()


if __name__ == "__main__":
    main()
