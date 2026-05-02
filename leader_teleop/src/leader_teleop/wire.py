"""UDP wire format for leader -> follower teleop.

Single source of truth for the packet schema. Both the skynet sender
(leader_teleop.main) and the fppe receiver (examples/debug/viser_control.py)
import from here.

Forward/backward compatibility rules:
  - New fields MUST have a default. Receivers ignore unknown fields.
  - Existing field types and meanings never change. Renames go through a
    deprecation cycle: add new field, fall back to old, remove old later.
  - Bump `version` only on incompatible changes; receivers may warn or refuse
    on mismatch.

Transport is UDP datagrams of JSON-encoded TeleopPacket. JSON keeps packets
human-readable in tcpdump and avoids extra deps. At 100 Hz / ~200 B per packet
the bandwidth is ~20 KB/s; if that ever matters, swap encode/decode for
msgpack while leaving the schema unchanged.
"""

from pydantic import BaseModel, ConfigDict


DEFAULT_PORT = 9999

WIRE_VERSION = 1


class ArmAngles(BaseModel):
    """Joint values for one SO-101 leader arm (so-arm-5dof profile).

    Units follow each joint's natural normalization on the leader:
      - 5 body joints (shoulder_pan ... wrist_roll): **degrees**, centred at 0
        (calibration midpoint), span ~360° per full revolution.
      - gripper: **percent (0..100)** over the calibrated range.

    The receiver applies the matching formula on the follower side, so each
    side owns its own calibration.
    """

    model_config = ConfigDict(extra="ignore")

    shoulder_pan: float
    shoulder_lift: float
    elbow_flex: float
    wrist_flex: float
    wrist_roll: float
    gripper: float


class TeleopPacket(BaseModel):
    model_config = ConfigDict(extra="ignore")

    version: int = WIRE_VERSION
    seq: int
    sent_ts: float

    left: ArmAngles | None = None
    right: ArmAngles | None = None

    # Future fields go here with defaults so old senders/receivers keep working.
    #   gamepad: GamepadState | None = None
    #   lift_velocity: float | None = None


# Joint name groups the receiver uses to apply the right unit conversion.
BODY_JOINT_NAMES: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)
GRIPPER_NAME: str = "gripper"


def encode(p: TeleopPacket) -> bytes:
    return p.model_dump_json(exclude_none=True).encode("utf-8")


def decode(data: bytes) -> TeleopPacket:
    return TeleopPacket.model_validate_json(data)


def seq_lt(a: int, b: int, window: int = 1 << 31) -> bool:
    """Wrap-aware 'a is older than b' for a uint32-style monotonic sequence."""
    return ((b - a) % (1 << 32)) < window and a != b
