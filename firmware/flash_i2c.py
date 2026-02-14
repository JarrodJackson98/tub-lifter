#!/usr/bin/env python3
"""
Flash STM32F072 firmware via I2C bootloader on Raspberry Pi.

The STM32F072V8T6 on the MEGA-IND V2 hat has a built-in ROM bootloader
that supports I2C1 (PB6/PB7 — the same bus connected to the RPi).

To enter bootloader mode:
  1. Bridge BOOT0 to 3.3V (solder a wire from the R75 pad closest to
     pin 91 on the STM32 to any 3.3V point, e.g. J5 pin 6)
  2. Power cycle the hat (unplug and replug the RPi, or toggle the hat's
     power if it has a separate supply)
  3. Verify with: i2cdetect -y 1  (should show 0x56 instead of 0x48)
  4. Run this script
  5. Remove the BOOT0 bridge and power cycle to boot the new firmware

Bootloader I2C address: 0x56 (7-bit), per AN2606 for STM32F072
Protocol: ST AN4221 (I2C bootloader protocol)

Usage:
  python3 flash_i2c.py info                     Show bootloader info
  python3 flash_i2c.py flash firmware.bin        Flash a binary file
  python3 flash_i2c.py backup backup.bin         Read flash to file
  python3 flash_i2c.py verify firmware.bin       Verify flash matches file
  python3 flash_i2c.py erase                     Mass erase flash
  python3 flash_i2c.py go                        Jump to firmware (0x08000000)

After building with PlatformIO, the firmware binary is at:
  .pio/build/megaind_can/firmware.bin
"""

import argparse
import sys
import time

from smbus2 import SMBus, i2c_msg

# STM32F072 bootloader constants
BOOTLOADER_ADDR = 0x56   # 7-bit I2C address (AN2606)
FLASH_BASE = 0x08000000
FLASH_SIZE = 64 * 1024   # 64KB for STM32F072V8
CHUNK_SIZE = 256          # Max bytes per Write Memory command

ACK = 0x79
NACK = 0x1F


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
        raise TimeoutError("Bootloader did not respond (is BOOT0 bridged to 3.3V?)")

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
    print("\nRemove the BOOT0 bridge and power cycle to boot the new firmware.")


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


def main():
    parser = argparse.ArgumentParser(
        description="Flash STM32F072 firmware via I2C bootloader (AN4221)",
        epilog=(
            "Before running, enter bootloader mode:\n"
            "  1. Bridge BOOT0 (R75 pad near STM32 pin 91) to 3.3V\n"
            "  2. Power cycle the hat\n"
            "  3. Verify: i2cdetect -y 1  (should show 0x56)\n"
            "\n"
            "After flashing, remove the bridge and power cycle."
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
    args = parser.parse_args()

    if args.command in ("flash", "backup", "verify") and not args.file:
        parser.error(f"'{args.command}' requires a file argument")

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
