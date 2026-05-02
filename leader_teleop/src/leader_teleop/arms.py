"""Minimal STS3215 leader arm reader using scservo_sdk directly.

This is a much smaller, single-host alternative to lerobot's FeetechMotorsBus
for the read-only leader use case: we don't drive motors, we just read joint
positions for teleop.

Each LeaderArm represents one SO-101 leader (5 DoF + gripper) on its own
USB-serial bus, with motor IDs 1..6.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import scservo_sdk as scs

from .calibration import Calibration, decode_signed_ticks

logger = logging.getLogger(__name__)


# STS3215 control table — values transcribed from
# src/lerobot/motors/feetech/tables.py.
ADDR_HOMING_OFFSET = 31    # EEPROM, 2 bytes, sign-magnitude (sign bit 11)
ADDR_TORQUE_ENABLE = 40
ADDR_LOCK = 55             # set to 0 to allow EEPROM writes
ADDR_PRESENT_POSITION = 56
LEN_PRESENT_POSITION = 2

DEFAULT_BAUDRATE = 1_000_000
DEFAULT_PROTOCOL = 0
RESOLUTION = 4096
HALF_TURN = RESOLUTION // 2  # 2048; what Homing_Offset anchors home to
SIGN_BIT_HOMING_OFFSET = 11

# Per-write retry policy. SRAM writes occasionally drop a status packet on a
# busy bus; EEPROM writes (Homing_Offset) need ~10 ms to settle, so we sleep
# longer between attempts.
_TX_RETRIES = 4
_TX_RETRY_SLEEP = 0.05
_EEPROM_SETTLE_S = 0.03


def _encode_sign_magnitude(value: int, sign_bit: int) -> int:
    max_mag = (1 << sign_bit) - 1
    mag = abs(value)
    if mag > max_mag:
        raise ValueError(f"Magnitude {mag} exceeds {max_mag} (sign_bit={sign_bit})")
    return ((1 if value < 0 else 0) << sign_bit) | mag


# Motor names mirror the so-arm-5dof profile in lerobot's so_leader.py.
MOTOR_NAMES: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
MOTOR_ID_BY_NAME: dict[str, int] = {name: i + 1 for i, name in enumerate(MOTOR_NAMES)}


@dataclass
class LeaderArm:
    """Read-only access to one SO-101 leader arm via scservo_sdk."""

    port: str
    name: str  # "left" or "right" — used as calibration id key
    baudrate: int = DEFAULT_BAUDRATE
    protocol: int = DEFAULT_PROTOCOL

    def __post_init__(self) -> None:
        self._port_handler: scs.PortHandler | None = None
        self._packet_handler: scs.PacketHandler | None = None
        self._sync_reader: scs.GroupSyncRead | None = None
        self.calibration: Calibration | None = None

    # ----- connection lifecycle -----

    @property
    def is_connected(self) -> bool:
        return self._port_handler is not None and self._port_handler.is_open

    def connect(self) -> None:
        if self.is_connected:
            return
        ph = scs.PortHandler(self.port)
        if not ph.openPort():
            raise OSError(f"Failed to open serial port {self.port}")
        if not ph.setBaudRate(self.baudrate):
            ph.closePort()
            raise OSError(f"Failed to set baudrate {self.baudrate} on {self.port}")
        self._port_handler = ph
        self._packet_handler = scs.PacketHandler(self.protocol)
        self._sync_reader = scs.GroupSyncRead(
            ph, self._packet_handler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
        )
        for motor_id in MOTOR_ID_BY_NAME.values():
            if not self._sync_reader.addParam(motor_id):
                raise RuntimeError(f"GroupSyncRead.addParam failed for id={motor_id}")
        logger.info("Connected: %s (%s)", self.name, self.port)

    def disconnect(self) -> None:
        if self._port_handler is not None and self._port_handler.is_open:
            self._port_handler.closePort()
        self._port_handler = None
        self._packet_handler = None
        self._sync_reader = None

    # ----- low-level motor io -----

    def ping(self, motor_id: int) -> int:
        """Returns the model number reported by the motor (raises on failure)."""
        assert self._packet_handler and self._port_handler
        model, comm, err = self._packet_handler.ping(self._port_handler, motor_id)
        if comm != scs.COMM_SUCCESS:
            raise RuntimeError(
                f"ping id={motor_id} failed: "
                f"{self._packet_handler.getTxRxResult(comm)} (err={err})"
            )
        return model

    def ping_all(self) -> dict[str, int]:
        return {name: self.ping(mid) for name, mid in MOTOR_ID_BY_NAME.items()}

    def _write1(self, motor_id: int, addr: int, value: int) -> None:
        assert self._packet_handler and self._port_handler
        last_err: str = ""
        for _ in range(_TX_RETRIES):
            comm, err = self._packet_handler.write1ByteTxRx(
                self._port_handler, motor_id, addr, value & 0xFF
            )
            if comm == scs.COMM_SUCCESS and err == 0:
                return
            last_err = f"{self._packet_handler.getTxRxResult(comm)} err={err}"
            time.sleep(_TX_RETRY_SLEEP)
        raise RuntimeError(f"write1 id={motor_id} addr={addr}: {last_err}")

    def _write2(self, motor_id: int, addr: int, value: int) -> None:
        assert self._packet_handler and self._port_handler
        last_err: str = ""
        for _ in range(_TX_RETRIES):
            comm, err = self._packet_handler.write2ByteTxRx(
                self._port_handler, motor_id, addr, value & 0xFFFF
            )
            if comm == scs.COMM_SUCCESS and err == 0:
                return
            last_err = f"{self._packet_handler.getTxRxResult(comm)} err={err}"
            time.sleep(_TX_RETRY_SLEEP)
        raise RuntimeError(f"write2 id={motor_id} addr={addr}: {last_err}")

    def _read2(self, motor_id: int, addr: int) -> int:
        """Single-motor 2-byte read with retries. Used to verify EEPROM writes.

        scservo_sdk's read2ByteTxRx has a bug: on a timeout it raises
        IndexError instead of returning a clean error code. We catch that
        and treat it the same as a comm failure.
        """
        assert self._packet_handler and self._port_handler
        last_err = ""
        for _ in range(_TX_RETRIES):
            try:
                value, comm, err = self._packet_handler.read2ByteTxRx(
                    self._port_handler, motor_id, addr
                )
            except (IndexError, ValueError) as e:
                last_err = f"SDK exception: {e!r}"
                time.sleep(_TX_RETRY_SLEEP)
                continue
            if comm == scs.COMM_SUCCESS and err == 0:
                return value
            last_err = f"{self._packet_handler.getTxRxResult(comm)} err={err}"
            time.sleep(_TX_RETRY_SLEEP)
        raise RuntimeError(f"read2 id={motor_id} addr={addr}: {last_err}")

    # ----- high-level helpers -----

    def disable_torque(self) -> None:
        """Free the arm so the user can hand-move it. SRAM writes only."""
        for motor_id in MOTOR_ID_BY_NAME.values():
            self._write1(motor_id, ADDR_TORQUE_ENABLE, 0)
            self._write1(motor_id, ADDR_LOCK, 0)  # also unlocks EEPROM writes

    def write_homing_offsets(self, offsets: dict[str, int], max_attempts: int = 5) -> None:
        """Write Homing_Offset (EEPROM) for each named motor so future
        Present_Position reads are shifted: Present_Position = actual - offset.

        Used at calibration time to anchor the user's home pose at tick 2048
        (HALF_TURN), which moves the encoder wraparound boundary 2048 ticks
        away from any reachable pose.

        We **read back** after each write and retry on mismatch, because the
        STS3215 occasionally returns COMM_SUCCESS for a write that didn't
        actually persist (observed on a busy bus right after a burst of
        writes to other motors).
        """
        for name, offset in offsets.items():
            motor_id = MOTOR_ID_BY_NAME[name]
            encoded = _encode_sign_magnitude(offset, SIGN_BIT_HOMING_OFFSET)
            for attempt in range(max_attempts):
                # Lock=0 right before each write so we know EEPROM is unlocked,
                # in case anything (e.g. another driver, motor power glitch)
                # re-armed it.
                self._write1(motor_id, ADDR_LOCK, 0)
                time.sleep(_EEPROM_SETTLE_S)
                self._write2(motor_id, ADDR_HOMING_OFFSET, encoded)
                time.sleep(_EEPROM_SETTLE_S)
                actual = self._read2(motor_id, ADDR_HOMING_OFFSET)
                if actual == encoded:
                    break
                logger.warning(
                    "Homing_Offset write didn't stick on %s (id=%d): "
                    "wrote 0x%04x, read back 0x%04x (attempt %d/%d)",
                    name, motor_id, encoded, actual, attempt + 1, max_attempts,
                )
            else:
                raise RuntimeError(
                    f"Homing_Offset write to {name} (id={motor_id}) did not persist "
                    f"after {max_attempts} attempts; wrote 0x{encoded:04x}, "
                    f"read back 0x{actual:04x}"
                )

    def read_positions(self) -> dict[str, int]:
        """Return raw Present_Position (0..4095) per motor name."""
        assert self._sync_reader
        comm = self._sync_reader.txRxPacket()
        if comm != scs.COMM_SUCCESS:
            raise RuntimeError(f"sync_read txRxPacket: {self._packet_handler.getTxRxResult(comm)}")
        out: dict[str, int] = {}
        for name, motor_id in MOTOR_ID_BY_NAME.items():
            if not self._sync_reader.isAvailable(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                raise RuntimeError(f"sync_read missing data for {name} (id={motor_id})")
            raw = self._sync_reader.getData(motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            out[name] = decode_signed_ticks(raw)
        return out

    def read_angles_deg(self) -> dict[str, float]:
        """Return calibrated joint angles in degrees (and gripper as 0..100 percent)."""
        if self.calibration is None:
            raise RuntimeError(f"{self.name} has no calibration loaded")
        ticks = self.read_positions()
        return self.calibration.ticks_to_units(ticks)

    def apply_calibration(self, cal: Calibration) -> None:
        """Software-side calibration: just disable torque (so the user can move
        the arm) and stash the calibration. Tick→degrees conversion happens in
        Calibration.ticks_to_units. No EEPROM writes."""
        self.disable_torque()
        self.calibration = cal

    # ----- calibration-time helpers -----

    def stream_positions(self, period_s: float = 0.05):
        """Generator that yields fresh positions until the caller breaks out."""
        while True:
            yield self.read_positions()
            time.sleep(period_s)
