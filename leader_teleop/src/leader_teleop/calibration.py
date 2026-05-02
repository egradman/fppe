"""Per-arm calibration: persisted homing offsets and ROM ranges.

Format mirrors the lerobot MotorCalibration shape so the receiver can
interpret leader-side data identically (the math in viser_control.py uses
the same range_min/range_max convention).

Storage: ~/.config/leader_teleop/{arm_name}.json — one file per arm.
The 5 body joints carry their unit as `degrees`; the gripper as `percent`
(0..100), matching MotorNormMode.RANGE_0_100.
"""

from __future__ import annotations

import json
import os
import select
import sys
import termios
import time
import tty
from dataclasses import asdict, dataclass, field
from pathlib import Path

# Resolution of the STS3215 encoder.
RESOLUTION = 4096
HALF_TURN = RESOLUTION // 2

GRIPPER_NAME = "gripper"
BODY_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)


def _calibration_dir() -> Path:
    base = os.environ.get("LEADER_TELEOP_CALIBRATION_DIR")
    if base:
        return Path(base)
    return Path.home() / ".config" / "leader_teleop"


def calibration_path(arm_name: str) -> Path:
    return _calibration_dir() / f"{arm_name}.json"


def decode_signed_ticks(raw: int) -> int:
    """Decode a 16-bit sign-magnitude STS3215 Present_Position into a plain int.

    For STS3215, Present_Position uses sign bit 15 (so values 0..32767 are
    positive, 32768..65535 are negative). The motor still returns a value in
    its native one-rotation 0..4095 range when configured normally, but
    `Homing_Offset` can shift it negative.
    """
    if raw & (1 << 15):
        return -(raw & 0x7FFF)
    return raw


def unwrap_to_home(tick: int, home_tick: int, resolution: int = RESOLUTION) -> int:
    """Map `tick` to its closest equivalent within ±resolution/2 of home_tick.

    The STS3215 encoder is single-turn (12-bit, 0..4095). When the
    user-defined home pose is near a wraparound boundary, normal joint motion
    can cross 0/4095 and look like a sudden 4096-tick jump. This unwrap
    converts that jump into a small signed difference from home, so callers
    can treat the encoder as continuous around the home pose.

    Caveat: assumes the joint stays within ±180° of home. SO-101 joint ROMs
    are well under that, except wrist_roll which can spin freely — fine for
    teleop where you don't spin past ±180° from your starting pose.
    """
    half = resolution // 2
    diff = tick - home_tick
    if diff > half:
        diff -= resolution
    elif diff < -half:
        diff += resolution
    return home_tick + diff


@dataclass
class JointCalibration:
    home_tick: int  # raw tick at the user-defined home pose; this maps to 0° on the wire
    range_min: int  # observed min Present_Position during ROM sweep (raw 0..4095 tick)
    range_max: int  # observed max


@dataclass
class Calibration:
    arm_name: str
    joints: dict[str, JointCalibration] = field(default_factory=dict)

    def save(self) -> Path:
        path = calibration_path(self.arm_name)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "arm_name": self.arm_name,
            "joints": {name: asdict(j) for name, j in self.joints.items()},
        }
        path.write_text(json.dumps(payload, indent=2) + "\n")
        return path

    @classmethod
    def load(cls, arm_name: str) -> "Calibration | None":
        path = calibration_path(arm_name)
        if not path.exists():
            return None
        try:
            data = json.loads(path.read_text())
            joints = {n: JointCalibration(**v) for n, v in data["joints"].items()}
            return cls(arm_name=data["arm_name"], joints=joints)
        except (TypeError, KeyError, json.JSONDecodeError) as e:
            print(f"Ignoring incompatible calibration at {path}: {e}")
            return None

    # ----- tick <-> calibrated-unit conversion -----

    def ticks_to_units(self, ticks: dict[str, int]) -> dict[str, float]:
        out: dict[str, float] = {}
        for name, tick in ticks.items():
            j = self.joints[name]
            unwrapped = unwrap_to_home(tick, j.home_tick)
            if name == GRIPPER_NAME:
                # 0..100 percent over the calibrated range. Both leader and
                # follower configure the gripper as RANGE_0_100, so this
                # passes through cleanly.
                span = max(1, j.range_max - j.range_min)
                pct = (unwrapped - j.range_min) / span * 100.0
                out[name] = max(0.0, min(100.0, pct))
            else:
                # Degrees about the user-defined home pose. The follower
                # interprets 0° as its own MotorCalibration midpoint
                # (≈ tick 2047 after lerobot's Homing_Offset), so as long as
                # the user calibrated both arms to the SAME physical pose
                # for "home"/"middle", the two sides agree.
                out[name] = (unwrapped - j.home_tick) * 360.0 / (RESOLUTION - 1)
        return out


# ---------------------------------------------------------------------------
# Interactive calibration helpers
# ---------------------------------------------------------------------------

class _Cbreak:
    """Context manager: put stdin in cbreak mode so single keypresses are
    immediately readable via `select`. Only used while the live calibration
    table is on screen — restored on exit."""

    def __enter__(self) -> "_Cbreak":
        self._fd = sys.stdin.fileno() if sys.stdin.isatty() else None
        self._old = termios.tcgetattr(self._fd) if self._fd is not None else None
        if self._fd is not None:
            tty.setcbreak(self._fd)
        return self

    def __exit__(self, *_exc) -> None:
        if self._fd is not None and self._old is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def key_pressed(self) -> bool:
        if self._fd is None:
            return False
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.read(1)  # consume the byte
            return True
        return False


def calibrate_interactive(arm) -> Calibration:
    """Calibration flow (software-only, no EEPROM writes):
      1. Disable torque so the user can hand-move the arm.
      2. Prompt user to position arm at HOME pose; record raw ticks.
      3. Prompt user to sweep ROM; record per-joint min/max ticks in
         the unwrap-to-home space (handles encoder wraparound transparently).
      4. Save Calibration as JSON.

    We deliberately don't touch Homing_Offset on the motor: STS3215 EEPROM
    writes on this hardware silently fail to commit (the SDK reports success
    and even the verify-read returns the new value, but a later cold read
    shows stale EEPROM). Wraparound is instead handled by `_unwrap_to_home`
    at read time.
    """
    print(f"\n=== Calibrating {arm.name} arm ===")
    arm.disable_torque()

    input(
        "\nMove the arm to its HOME pose (the pose that should map to 0° on\n"
        "the follower) and press ENTER. Make sure the follower's calibrated\n"
        '"middle" pose is the same physical pose. '
    )
    home_ticks = arm.read_positions()

    print(
        "\nSweep every joint through its full range. Live tick values shown below\n"
        "(unwrapped to the home pose). Press any key when done.\n"
    )
    start = arm.read_positions()
    motor_names = list(start.keys())
    # Track everything in the unwrap-to-home space so encoder boundary
    # crossings during sweep don't blow up min/max.
    mins = {n: unwrap_to_home(start[n], home_ticks[n]) for n in motor_names}
    maxes = dict(mins)

    # Print header + N placeholder rows once; the loop redraws the rows in place.
    print(f"{'NAME':<14} {'MIN':>6} {'POS':>6} {'MAX':>6}")
    for _ in motor_names:
        print()

    with _Cbreak() as cb:
        while True:
            raw = arm.read_positions()
            unwrapped = {n: unwrap_to_home(raw[n], home_ticks[n]) for n in motor_names}
            for name, val in unwrapped.items():
                if val < mins[name]:
                    mins[name] = val
                if val > maxes[name]:
                    maxes[name] = val

            # Move cursor back to the first data row, redraw rows in place
            # (\033[2K = clear entire line). Header stays put.
            sys.stdout.write(f"\033[{len(motor_names)}A")
            for name in motor_names:
                sys.stdout.write(
                    f"\033[2K{name:<14} {mins[name]:>6} {unwrapped[name]:>6} {maxes[name]:>6}\n"
                )
            sys.stdout.flush()

            if cb.key_pressed():
                break
            time.sleep(0.05)

    cal = Calibration(arm_name=arm.name)
    for name in motor_names:
        if mins[name] == maxes[name]:
            raise RuntimeError(
                f"Joint '{name}' min == max ({mins[name]}); didn't move during sweep."
            )
        cal.joints[name] = JointCalibration(
            home_tick=home_ticks[name],
            range_min=mins[name],
            range_max=maxes[name],
        )
    path = cal.save()
    print(f"Calibration saved to {path}")
    return cal
