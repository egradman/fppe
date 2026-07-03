"""UDP wire format for pedal -> follower base teleop.

Single source of truth for the packet schema. Both the skynet sender
(pedal_teleop.main) and the fppe receiver (examples/debug/viser_control.py)
import from here.

Forward/backward compatibility rules mirror leader_teleop/wire.py:
  - New fields MUST have a default. Receivers ignore unknown fields.
  - Bump `version` only on incompatible changes.

Transport is UDP datagrams of JSON-encoded PedalPacket on port 9998
(leader is on 9999). At 100 Hz / ~150 B per packet the bandwidth is
~15 KB/s.
"""

from typing import Literal

from pydantic import BaseModel, ConfigDict


DEFAULT_PORT = 9998

WIRE_VERSION = 1


LiftCmd = Literal["up", "down", "stop"]


class PedalPacket(BaseModel):
    """High-level base command derived from two foot-pedal IMUs.

    Kinematics live on fppe co-located with the wheel geometry; skynet
    just sends the body-frame command in physical units. Sign conventions
    match `viser_control.body_to_wheel_raw` so that the same numbers
    behave identically to the existing web-gamepad path.
    """

    model_config = ConfigDict(extra="ignore")

    version: int = WIRE_VERSION
    seq: int
    sent_ts: float

    x_vel: float        # m/s, +forward (matches viser_control gamepad path)
    y_vel: float        # m/s, +strafe-left (matches viser_control gamepad path)
    theta_vel: float    # deg/s, +rotate-CCW / +rotate-left
    lift: LiftCmd = "stop"


def encode(p: PedalPacket) -> bytes:
    return p.model_dump_json().encode("utf-8")


def decode(data: bytes) -> PedalPacket:
    return PedalPacket.model_validate_json(data)


def seq_lt(a: int, b: int, window: int = 1 << 31) -> bool:
    """Wrap-aware 'a is older than b' for a uint32-style monotonic sequence."""
    return ((b - a) % (1 << 32)) < window and a != b
