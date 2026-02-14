"""
Thomson Electrak HD (HD24B045-1000CN02MPS) CAN bus driver.

Communicates via J1939 protocol through the MEGA-IND hat's I2C-to-CAN bridge
firmware (STM32F072V8T6).

- PGN 61184 (0xEF00): Actuator Control Message (ACM)
- PGN 126720: Actuator Feedback Message (AFM)
- Default node ID: 19 (0x13)
- Baud rate: 500 kbit/s (configured in STM32 firmware)
- Stroke: 1000mm

Reference: https://github.com/collinbrake/electrakHDCAN
"""

import math
import threading
import time

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None


def _reverse_bits(num, bit_count):
    """Reverse bit order within a field of bit_count width."""
    if num == 0:
        return 0
    result = 0
    temp = num
    while temp:
        result = (result << 1) + (temp & 1)
        temp >>= 1
    sig_bits = math.ceil(math.log2(num + 1))
    return result << (bit_count - sig_bits)


class ActuatorControlMessage:
    """Encodes a J1939 Actuator Control Message (PGN 61184) for the Electrak HD."""

    PRIORITY = 6
    PGN = 61184       # 0xEF00
    DEST_ADDR = 19    # 0x13 - default Electrak HD node ID
    SRC_ADDR = 0

    def __init__(self, position_mm, speed_pct, current_limit_a=10.0, motion_enable=True):
        self.position_mm = position_mm
        self.speed_pct = speed_pct
        self.current_limit_a = current_limit_a
        self.motion_enable = motion_enable

    @property
    def arbitration_id(self):
        """Build 29-bit J1939 CAN ID."""
        return (
            (self.PRIORITY << 26)
            | (self.PGN << 8)
            | (self.DEST_ADDR << 8)
            | self.SRC_ADDR
        )

    def encode(self):
        """Encode data bytes per Electrak HD bit layout (LSBit-first fields)."""
        pos = int(self.position_mm / 0.1)       # 14-bit, 0.1mm/bit
        cur = int(self.current_limit_a / 0.1)   # 9-bit, 0.1A/bit
        spd = int(round(self.speed_pct / 5))    # 5-bit, 5%/bit
        mot = int(self.motion_enable)            # 1-bit

        # Bit-pack with reversed bit order per Electrak HD manual
        rb = _reverse_bits
        d0 = rb((rb(pos, 14) >> 6) & 0xFF, 8)
        d1 = rb((rb(pos, 14) << 2 | rb(cur, 9) >> 7) & 0xFF, 8)
        d2 = rb((rb(cur, 9) << 1 | rb(spd, 5) >> 4) & 0xFF, 8)
        d3 = rb((rb(spd, 5) << 4 | mot << 3) & 0xFF, 8)

        data = bytearray(8)
        data[0] = d0 & 0xFF
        data[1] = d1 & 0xFF
        data[2] = d2 & 0xFF
        data[3] = d3 & 0xFF
        data[4] = 0xFF
        data[5] = 0xFF
        data[6] = 0xFF
        data[7] = 0xFF
        return data

    def to_i2c_payload(self):
        """Pack into 14-byte I2C write payload for the CAN bridge firmware."""
        can_id = self.arbitration_id
        data = self.encode()
        return [
            (can_id >> 24) & 0xFF,
            (can_id >> 16) & 0xFF,
            (can_id >> 8) & 0xFF,
            can_id & 0xFF,
            0x01,  # flags: extended 29-bit ID
            8,     # DLC
        ] + list(data)


class ActuatorFeedbackMessage:
    """Decodes J1939 Actuator Feedback Message (PGN 126720) from the Electrak HD."""

    TARGET_PGN = 126720

    def __init__(self):
        self.position_mm = 0.0
        self.current_a = 0.0
        self.speed_pct = 0
        self.in_motion = False
        self.overload = False
        self.fatal_error = False
        self.valid = False

    @staticmethod
    def _extract_pgn(can_id):
        return (can_id >> 8) & 0x3FF00

    def decode_i2c(self, rx_data):
        """Decode from I2C RX_FRAME register read (15 bytes).

        Format: [STATUS] [ID3 ID2 ID1 ID0] [FLAGS] [DLC] [D0..D7]
        """
        status = rx_data[0]
        if not (status & 0x01):
            self.valid = False
            return

        can_id = (
            (rx_data[1] << 24) | (rx_data[2] << 16) |
            (rx_data[3] << 8) | rx_data[4]
        )

        pgn = self._extract_pgn(can_id)
        if pgn != self.TARGET_PGN:
            self.valid = False
            return

        d = rx_data[7:15]  # 8 data bytes
        rb = _reverse_bits

        def get_bits(d1, d2, shift, mask, length):
            val = rb(d1, 8)
            if d2 is not None:
                val = (val << 8) | rb(d2, 8)
            return rb((val >> shift) & mask, length)

        self.position_mm = get_bits(d[0], d[1], 2, 0x3FFF, 14) * 0.1
        self.current_a = get_bits(d[1], d[2], 1, 0x1FF, 9) * 0.1
        self.speed_pct = get_bits(d[2], d[3], 4, 0x1F, 5) * 5
        self.in_motion = bool(get_bits(d[4], None, 7, 1, 1))
        self.overload = bool(get_bits(d[4], None, 6, 1, 1))
        self.fatal_error = bool(get_bits(d[4], None, 2, 1, 1))
        self.valid = True


STROKE_MM = 1000.0  # HD24B045-1000CN02MPS has 1000mm stroke

# I2C bridge firmware registers
REG_TX_SEND = 0x00
REG_RX_FRAME = 0x10
REG_INFO = 0x20

# I2C address: 0x50 + jumper offset (matches Sequent MEGA-IND convention)
I2C_BASE_ADDR = 0x50


class ElectrakHD:
    """High-level interface for controlling the Thomson Electrak HD actuator
    via the MEGA-IND hat I2C-to-CAN bridge firmware."""

    def __init__(self, i2c_bus=1, i2c_addr=I2C_BASE_ADDR):
        self.i2c_bus_num = i2c_bus
        self.i2c_addr = i2c_addr
        self.bus = None
        self._running = False
        self._tx_thread = None
        self._lock = threading.Lock()
        self._target_position = None
        self._target_speed = 50
        self._motion_enable = False
        self.feedback = ActuatorFeedbackMessage()

    def connect(self):
        if SMBus is None:
            raise RuntimeError(
                "smbus2 not available - install it or run on Raspberry Pi"
            )
        self.bus = SMBus(self.i2c_bus_num)
        self._running = True
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()

    def disconnect(self):
        self._running = False
        if self._tx_thread:
            self._tx_thread.join(timeout=2)
        self.stop()
        if self.bus:
            self.bus.close()
            self.bus = None

    def extend(self, speed_pct=50):
        """Command actuator to fully extend (position = max stroke)."""
        with self._lock:
            self._target_position = STROKE_MM
            self._target_speed = speed_pct
            self._motion_enable = True

    def retract(self, speed_pct=50):
        """Command actuator to fully retract (position = 0)."""
        with self._lock:
            self._target_position = 0.0
            self._target_speed = speed_pct
            self._motion_enable = True

    def stop(self):
        """Stop actuator motion."""
        with self._lock:
            self._motion_enable = False

    def get_status(self):
        """Return current feedback status."""
        with self._lock:
            return {
                "position_mm": self.feedback.position_mm,
                "current_a": self.feedback.current_a,
                "speed_pct": self.feedback.speed_pct,
                "in_motion": self.feedback.in_motion,
                "overload": self.feedback.overload,
                "fatal_error": self.feedback.fatal_error,
                "valid": self.feedback.valid,
                "target_position": self._target_position,
                "motion_enabled": self._motion_enable,
            }

    def _send_can_frame(self, acm):
        """Send a CAN frame via the I2C-to-CAN bridge."""
        payload = acm.to_i2c_payload()
        self.bus.write_i2c_block_data(self.i2c_addr, REG_TX_SEND, payload)

    def _read_can_frame(self):
        """Read last received CAN frame from the bridge."""
        data = self.bus.read_i2c_block_data(self.i2c_addr, REG_RX_FRAME, 15)
        self.feedback.decode_i2c(data)

    def _tx_loop(self):
        """Send ACM at ~100ms intervals (J1939 recommended rate), read feedback."""
        while self._running:
            # Read any available feedback from CAN bus
            try:
                self._read_can_frame()
            except Exception:
                pass

            # Send control message
            with self._lock:
                pos = self._target_position if self._target_position is not None else 0
                spd = self._target_speed
                mot = self._motion_enable

            acm = ActuatorControlMessage(pos, spd, motion_enable=mot)
            try:
                self._send_can_frame(acm)
            except Exception:
                pass

            time.sleep(0.1)
