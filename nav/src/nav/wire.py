"""UDP wire format for the nav executor -> fppe base.

Byte-for-byte a `PedalPacket` (see pedal_teleop/src/pedal_teleop/wire.py): the
fppe receiver decodes both streams with the same `pedal_wire.decode`, so this
must stay schema-compatible. We keep an independent copy here rather than
importing across uv projects — same convention leader_teleop / pedal_teleop
already follow. The JSON schema is the contract, not the Python class.

Transport: UDP datagrams of JSON on port 9997 (pedals are on 9998). The fppe
receiver only acts on packets while `base_input_source == "nav"`, and a 300 ms
wheel watchdog stops the base if the stream stalls — so a crashed executor
coasts to a halt on its own.
"""

import json

DEFAULT_PORT = 9997
WIRE_VERSION = 1


def encode(seq: int, x_vel: float, y_vel: float, theta_vel: float,
           lift: str = "stop", sent_ts: float = 0.0) -> bytes:
    """Serialize one base command.

    x_vel:     m/s, +forward
    y_vel:     m/s, +strafe-left
    theta_vel: deg/s, +rotate-CCW / +left
    lift:      "up" | "down" | "stop"
    Sign conventions match `viser_control.body_to_wheel_raw`.
    """
    return json.dumps({
        "version": WIRE_VERSION,
        "seq": seq & 0xFFFFFFFF,
        "sent_ts": sent_ts,
        "x_vel": x_vel,
        "y_vel": y_vel,
        "theta_vel": theta_vel,
        "lift": lift,
    }).encode("utf-8")
