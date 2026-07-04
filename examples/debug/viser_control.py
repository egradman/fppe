#!/usr/bin/env python3
"""Viser-based web control panel for dual-arm LeKiwi robot.

Provides browser UI with:
  - Per-joint sliders for both arms (raw 0-4095)
  - Lock/unlock checkbox per arm (torque enable/disable)
  - Home and Middle position buttons
  - Elevator Z Up / Z Down / Z Stop buttons
  - Live 3D URDF visualization
  - Gamepad teleop (omni base + lift) via companion web UI

HTTP API (on --api-port, default 8091):
  GET  /state                — JSON snapshot of all motor positions and arm lock states
  POST /goal_position        — {"motor_name": raw_value, ...}
  POST /go_middle            — send all arms to midpoint (2048)
  POST /lock_arm             — {"side": "left"|"right", "lock": true|false}
  POST /estop                — emergency stop (release all torque)
  POST /lift                 — {"action": "up"|"down"|"stop"}
  POST /gamepad              — {"axes": [...], "buttons": [bool, ...], "enabled": bool}
  POST /teleop_mode          — {"mode": "local"|"remote"} — see TeleopMode
  POST /base_input_source    — {"value": "off"|"gamepad"|"pedals"|"auto"} — see BaseInputSource

The /state response includes "estop_engaged" (bool) read from a Raspberry Pi
GPIO pin (default GPIO19). The e-stop is inline with motor power, so when
engaged the Feetech bus is unresponsive — reading the GPIO lets us tell
"e-stop engaged" (expected; wait it out) apart from "motors broken"
(unexpected; loose cable, etc).

Usage:
    python viser_control.py                          # defaults
    python viser_control.py --port 8090              # custom viser port
    python viser_control.py --serial-port /dev/ttyACM1

Open http://<robot-ip>:<port> in your browser.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import signal
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
import mimetypes
from pathlib import Path

import cv2
import numpy as np
import viser

# Make `leader_teleop` (a sibling uv project under the repo root) importable
# without installing it: it has its own .venv, but its `wire.py` only depends
# on pydantic (which lerobot's env already provides), so we just put its src
# layout on sys.path.
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_REPO_ROOT / "leader_teleop" / "src"))
sys.path.insert(0, str(_REPO_ROOT / "pedal_teleop" / "src"))
# voice/scripts holds the speech pipeline; imported lazily in start_voice_thread
# (its top-level `openai`/`sounddevice` deps aren't installed unless --voice is used).
sys.path.insert(0, str(_REPO_ROOT / "voice" / "scripts"))

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.motors_bus import MotorCalibration
from lerobot.robots.alohamini.config_lekiwi import LeKiwiConfig
from lerobot.robots.alohamini.lekiwi import LeKiwi

from leader_teleop import wire as leader_wire
from pedal_teleop import wire as pedal_wire

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MIDPOINT = 2048
STEPS_PER_DEG = 4096.0 / 360.0
LIFT_SPEED_DEGPS = 180.0

# ---- Gamepad teleop (omni base + lift) ----
WHEEL_NAMES = ("base_left_wheel", "base_back_wheel", "base_right_wheel")
LIN_SPEED = 0.2             # m/s at full stick
ANG_SPEED = 80.0            # deg/s at full stick
WHEEL_RADIUS = 0.05
BASE_RADIUS = 0.125
WHEEL_MAX_RAW = 3000
GAMEPAD_DEADZONE = 0.1
GAMEPAD_WATCHDOG_S = 0.3    # stop wheels if no gamepad update for this long
AXIS_LIFT = 7               # D-pad vertical: -1 raises, +1 lowers

HOME_POSITION = {
    "arm_left_shoulder_pan": 2965,
    "arm_left_shoulder_lift": 1044,
    "arm_left_elbow_flex": 3071,
    "arm_left_wrist_flex": 953,
    "arm_left_wrist_roll": 2072,
    "arm_left_gripper": 1556,
    "arm_right_shoulder_pan": 992,
    "arm_right_shoulder_lift": 959,
    "arm_right_elbow_flex": 3033,
    "arm_right_wrist_flex": 1137,
    "arm_right_wrist_roll": 2046,
    "arm_right_gripper": 1611,
}

ARMS_UP_POSITION = {
    "arm_left_shoulder_pan": 1221,
    "arm_left_shoulder_lift": 1920,
    "arm_left_elbow_flex": 1472,
    "arm_left_wrist_flex": 1837,
    "arm_left_wrist_roll": 1840,
    "arm_left_gripper": 2283,
    "arm_right_shoulder_pan": 2875,   # mirrored: 4096 - 1221
    "arm_right_shoulder_lift": 1920,
    "arm_right_elbow_flex": 1472,
    "arm_right_wrist_flex": 1837,
    "arm_right_wrist_roll": 2256,     # mirrored: 4096 - 1840
    "arm_right_gripper": 2283,
}

URDF_PATH = Path(__file__).parent / "assets" / "alohamini" / "Aloha.urdf"

# Motor name -> URDF joint name mapping
MOTOR_TO_URDF_JOINT = {
    "arm_left_shoulder_pan": "left_joint1",
    "arm_left_shoulder_lift": "left_joint2",
    "arm_left_elbow_flex": "left_joint3",
    "arm_left_wrist_flex": "left_joint4",
    "arm_left_wrist_roll": "left_joint5",
    "arm_left_gripper": "left_joint6",
    "arm_right_shoulder_pan": "right_joint1",
    "arm_right_shoulder_lift": "right_joint2",
    "arm_right_elbow_flex": "right_joint3",
    "arm_right_wrist_flex": "right_joint4",
    "arm_right_wrist_roll": "right_joint5",
    "arm_right_gripper": "right_joint6",
}

# Radian offsets to align URDF joint zeros with servo midpoint (2048)
URDF_JOINT_OFFSETS = {
    "left_joint2": math.pi / 2,        # shoulder_lift
    "left_joint3": -math.pi / 2,       # elbow_flex
    "left_joint6": math.radians(-60),   # gripper
    "right_joint2": math.pi / 2,
    "right_joint3": -math.pi / 2,
    "right_joint6": math.radians(-60),
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def degps_to_raw(degps: float) -> int:
    mag = int(round(abs(degps) * STEPS_PER_DEG))
    if mag > 0x7FFF:
        mag = 0x7FFF
    return -mag if degps < 0 else mag


def raw_to_radians(raw_pos: float) -> float:
    return (raw_pos - MIDPOINT) * (2.0 * math.pi / 4096.0)


def body_to_wheel_raw(x_cmd: float, y_cmd: float, theta_cmd_degps: float) -> dict[str, int]:
    """Map body-frame velocity commands to raw per-wheel velocities for a 3-omni base."""
    theta_rad = theta_cmd_degps * (math.pi / 180.0)
    vel = np.array([-x_cmd, -y_cmd, theta_rad])
    angles = np.radians(np.array([240, 0, 120]) - 90)
    M = np.array([[np.cos(a), np.sin(a), BASE_RADIUS] for a in angles])
    v_lin = M.dot(vel)
    w_rad = v_lin / WHEEL_RADIUS
    w_degps = w_rad * (180.0 / math.pi)
    raw_abs = np.abs(w_degps) * STEPS_PER_DEG
    peak = float(np.max(raw_abs)) if raw_abs.size else 0.0
    if peak > WHEEL_MAX_RAW and peak > 1e-6:
        w_degps = w_degps * (WHEEL_MAX_RAW / peak)
    raw = [degps_to_raw(v) for v in w_degps]
    return {WHEEL_NAMES[0]: raw[0], WHEEL_NAMES[1]: raw[1], WHEEL_NAMES[2]: raw[2]}


# Forward kinematics for dead-reckoning: the inverse of body_to_wheel_raw. Given
# per-wheel raw *position* deltas since the last sample (ticks, 4096/rev), recover
# the incremental body-frame displacement. Used by the tour recorder to integrate
# an odometry estimate from the wheel encoders — fppe has no other odometry, but
# the LeKiwi omni wheels slip little enough that dead-reckoning is good locally.
_DR_ANGLES = np.radians(np.array([240, 0, 120]) - 90)
_DR_M = np.array([[np.cos(a), np.sin(a), BASE_RADIUS] for a in _DR_ANGLES])


def wheel_raw_delta_to_body(deltas: dict[str, float]) -> tuple[float, float, float]:
    """Per-wheel raw tick deltas -> incremental body displacement (dx_m, dy_m, dtheta_rad).

    Inverse of body_to_wheel_raw: ticks -> wheel angular displacement (rad) ->
    linear displacement at each wheel contact (m) -> solve M @ [-dx, -dy, dtheta].
    """
    w_rad = np.array([deltas[n] for n in WHEEL_NAMES]) * (2.0 * math.pi / 4096.0)
    v_lin = w_rad * WHEEL_RADIUS
    body = np.linalg.solve(_DR_M, v_lin)  # [-dx, -dy, dtheta]
    return -float(body[0]), -float(body[1]), float(body[2])



# Module-level lock around the Feetech bus. Acquired by _bus_write internally
# and explicitly wrapped around any direct bus.sync_read/sync_write call so the
# leader-UDP listener thread can safely write goals without racing the main
# loop. Re-entrant so a `with _BUS_LOCK` block can call helpers that also lock.
_BUS_LOCK = threading.RLock()


def _bus_write(bus: FeetechMotorsBus, item: str, name: str, value: int, retries: int = 3):
    """Write with retry to handle transient serial errors on a busy bus."""
    for attempt in range(retries):
        try:
            with _BUS_LOCK:
                bus.write(item, name, value, normalize=False)
            return
        except Exception:
            if attempt == retries - 1:
                raise
            time.sleep(0.05)


def _deg_to_tick(bus: FeetechMotorsBus, motor_name: str, deg: float) -> int:
    """Convert degrees-about-home to a raw tick.

    The follower's calibration writes Homing_Offset such that Present_Position
    reads (max_res // 2) at the user-defined home pose (lerobot's
    set_half_turn_homings). So tick 2047 IS the follower's home — anchor
    deg→tick on that, not on (range_min+range_max)/2 (which is only home if the
    joint's ROM is symmetric, and frequently isn't).
    """
    motor = bus.motors[motor_name]
    max_res = bus.model_resolution_table[motor.model] - 1   # 4095 for sts3215
    home = max_res // 2                                     # 2047
    return int(deg * max_res / 360 + home)


def _pct_to_tick(bus: FeetechMotorsBus, motor_name: str, pct: float) -> int:
    """Convert a 0..100 percent value to a raw tick (gripper). Mirrors
    FeetechMotorsBus._unnormalize for MotorNormMode.RANGE_0_100."""
    cal = bus.calibration[motor_name]
    pct = max(0.0, min(100.0, pct))
    return int((pct / 100.0) * (cal.range_max - cal.range_min) + cal.range_min)


def setup_arm_position_mode(bus: FeetechMotorsBus, names: list[str], accel: int = 20):
    for name in names:
        _bus_write(bus, "Torque_Enable", name, 0)
        time.sleep(0.02)
        _bus_write(bus, "Operating_Mode", name, OperatingMode.POSITION.value)
        time.sleep(0.02)
        _bus_write(bus, "Maximum_Acceleration", name, accel)
        time.sleep(0.02)
        _bus_write(bus, "Torque_Enable", name, 1)
        time.sleep(0.02)


def setup_lift_velocity_mode(bus: FeetechMotorsBus, name: str = "lift_axis"):
    _bus_write(bus, "Torque_Enable", name, 0)
    time.sleep(0.02)
    _bus_write(bus, "Operating_Mode", name, OperatingMode.VELOCITY.value)
    time.sleep(0.02)
    _bus_write(bus, "Torque_Enable", name, 1)
    time.sleep(0.02)


def setup_wheels_velocity_mode(bus: FeetechMotorsBus, names=WHEEL_NAMES):
    for name in names:
        _bus_write(bus, "Torque_Enable", name, 0)
        time.sleep(0.02)
        _bus_write(bus, "Operating_Mode", name, OperatingMode.VELOCITY.value)
        time.sleep(0.02)
        _bus_write(bus, "Torque_Enable", name, 1)
        time.sleep(0.02)


# ---------------------------------------------------------------------------
# E-stop GPIO monitor
# ---------------------------------------------------------------------------
class EstopMonitor:
    """Reads an e-stop button wired to a Raspberry Pi GPIO pin.

    The e-stop is inline with motor power. When engaged, the Feetech bus is
    unresponsive — so the GPIO is the source of truth.

    Wiring on this robot: switch is normally-closed to GND. With internal
    pull-up enabled, line reads LOW when released and HIGH when engaged
    (or the wire is broken — fail-safe).
    """

    def __init__(self, pin: int = 19):
        self._available = False
        self._device = None
        if pin <= 0:
            print("E-stop monitor disabled (pin <= 0).")
            return
        try:
            from gpiozero import DigitalInputDevice

            self._device = DigitalInputDevice(pin, pull_up=True)
            self._available = True
            state = "ENGAGED" if self.engaged else "released"
            print(f"E-stop monitor: GPIO{pin} (HIGH=engaged); current state: {state}")
        except Exception as e:
            print(f"E-stop GPIO unavailable ({e}); proceeding without monitoring.")

    @property
    def available(self) -> bool:
        return self._available

    @property
    def engaged(self) -> bool:
        if not self._available:
            return False
        return bool(self._device.value)

    def wait_for_release(self, log_interval: float = 5.0):
        """Block until e-stop is released, logging periodically."""
        if not self.engaged:
            return
        last_log = 0.0
        while self.engaged:
            now = time.monotonic()
            if now - last_log > log_interval:
                print("E-stop ENGAGED. Waiting for release...")
                last_log = now
            time.sleep(0.1)
        print("E-stop released.")


# ---------------------------------------------------------------------------
# HTTP API
# ---------------------------------------------------------------------------
class RobotState:
    """Thread-safe container for the latest robot state, read by the API."""

    def __init__(self):
        self._lock = threading.Lock()
        self._positions: dict[str, int] = {}
        self._arm_locked: dict[str, bool] = {"left": False, "right": False}
        self._estop_engaged: bool = False

    def update(
        self,
        positions: dict[str, int],
        arm_locked: dict[str, bool],
        estop_engaged: bool = False,
    ):
        with self._lock:
            self._positions = dict(positions)
            self._arm_locked = dict(arm_locked)
            self._estop_engaged = bool(estop_engaged)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "positions": dict(self._positions),
                "arm_locked": dict(self._arm_locked),
                "estop_engaged": self._estop_engaged,
            }


class TeleopMode:
    """Selects which input sources are allowed to drive the robot.

    'local'  — gamepad controls wheels + lift; remote UDP packets are dropped.
    'remote' — UDP packets from skynet drive the arm joints AND the gamepad
               continues to drive wheels + lift (the two paths target disjoint
               actuators). Useful for one operator driving with leader arms in
               their hands while a second operator drives the base, or just to
               keep the base usable during arm teleop.

    'off' would be the only state where no input source has authority; we
    don't currently model that explicitly — leaving the tab stops the gamepad
    pump on the client side, and the wheel watchdog coasts the base to a halt.
    """

    def __init__(self, initial: str = "local"):
        self._lock = threading.Lock()
        self._mode = initial

    @property
    def mode(self) -> str:
        with self._lock:
            return self._mode

    def set(self, mode: str) -> bool:
        if mode not in ("local", "remote"):
            return False
        with self._lock:
            self._mode = mode
        return True


class BaseInputSource:
    """Which input source is allowed to drive the wheels + lift.

    'off'     — no input authority; wheels/lift coast to stop and stay there.
    'gamepad' — the web gamepad's axes/buttons drive wheels + lift.
    'pedals'  — UDP packets from the skynet-side pedal_teleop drive wheels +
                lift. The web gamepad is silently dropped for the base.
    'auto'    — UDP packets from a skynet-side autonomous driver (the conductor:
                visual nav, chase, etc.) drive the wheels. Same PedalPacket wire
                format as 'pedals', on its own port. The web gamepad is dropped.

    Disjoint from TeleopMode, which gates the arms. E-stop overrides both.
    Default is 'off' — explicit-enable for safety.
    """

    _VALUES = ("off", "gamepad", "pedals", "auto")

    def __init__(self, initial: str = "off"):
        self._lock = threading.Lock()
        self._value = initial if initial in self._VALUES else "off"

    @property
    def value(self) -> str:
        with self._lock:
            return self._value

    def set(self, value: str) -> bool:
        if value not in self._VALUES:
            return False
        with self._lock:
            self._value = value
        return True


class GamepadState:
    """Latest gamepad state posted by the browser; consumed by the main loop."""

    def __init__(self):
        self._lock = threading.Lock()
        self._axes: list[float] = []
        self._buttons: list[bool] = []
        self._last_update: float = 0.0
        self._enabled: bool = False

    def update(self, axes: list[float], buttons: list[bool], enabled: bool):
        with self._lock:
            self._axes = list(axes)
            self._buttons = list(buttons)
            self._enabled = bool(enabled)
            self._last_update = time.monotonic()

    def snapshot(self) -> tuple[list[float], list[bool], bool, float]:
        with self._lock:
            age = time.monotonic() - self._last_update if self._last_update else 1e9
            return list(self._axes), list(self._buttons), self._enabled, age


WEB_DIST = Path(__file__).parent / "web-ui" / "dist"


class APIHandler(BaseHTTPRequestHandler):
    robot_state: RobotState
    cmd_queue: queue.Queue
    camera_readers: dict  # label -> CameraReader
    gamepad_state: "GamepadState"
    teleop_mode: "TeleopMode"
    base_input_source: "BaseInputSource"
    tour_recorder: "TourRecorder | None"

    def _json_response(self, code: int, data: dict):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict | None:
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        try:
            return json.loads(self.rfile.read(length))
        except Exception:
            return None

    def _stream_mjpeg(self, label: str):
        reader = self.camera_readers.get(label)
        if reader is None:
            self.send_error(404, f"No camera '{label}'")
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            while True:
                frame = reader.latest
                if frame is not None:
                    # Capped at ~15 fps per stream below; keep the encode lean
                    # so the UDP teleop thread isn't starved of CPU on the Pi.
                    small = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
                    _, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                time.sleep(1 / 15)  # ~15 fps cap per stream
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _serve_static(self):
        """Serve files from web-ui/dist, falling back to index.html for SPA routes."""
        url_path = self.path.split("?")[0]
        if url_path == "/":
            url_path = "/index.html"
        fpath = WEB_DIST / url_path.lstrip("/")
        if not fpath.is_file():
            fpath = WEB_DIST / "index.html"
        if not fpath.is_file():
            self.send_error(404)
            return
        mime, _ = mimetypes.guess_type(str(fpath))
        self.send_response(200)
        self.send_header("Content-Type", mime or "application/octet-stream")
        self.send_header("Content-Length", str(fpath.stat().st_size))
        self.end_headers()
        self.wfile.write(fpath.read_bytes())

    def do_GET(self):
        if self.path == "/state":
            snap = self.robot_state.snapshot()
            snap["teleop_mode"] = self.teleop_mode.mode
            snap["base_input_source"] = self.base_input_source.value
            if self.tour_recorder is not None:
                snap["record"] = self.tour_recorder.status()
            self._json_response(200, snap)
        elif self.path.startswith("/mjpeg/"):
            label = self.path.split("/mjpeg/", 1)[1]
            self._stream_mjpeg(label)
        elif self.path.startswith("/api/"):
            self.send_error(404)
        else:
            self._serve_static()

    def do_POST(self):
        body = self._read_json()
        if body is None:
            self._json_response(400, {"error": "invalid JSON"})
            return

        if self.path == "/goal_position":
            # {"motor_name": raw_value, ...}
            for name, value in body.items():
                self.cmd_queue.put(("goal_pos", name, int(value)))
            self._json_response(200, {"ok": True})

        elif self.path == "/go_home":
            self.cmd_queue.put(("go_home",))
            self._json_response(200, {"ok": True})

        elif self.path == "/go_arms_up":
            self.cmd_queue.put(("go_arms_up",))
            self._json_response(200, {"ok": True})

        elif self.path == "/go_middle":
            self.cmd_queue.put(("go_preset", MIDPOINT))
            self._json_response(200, {"ok": True})

        elif self.path == "/lock_arm":
            # {"side": "left"|"right", "lock": true|false}
            side = body.get("side")
            lock = body.get("lock", True)
            if side not in ("left", "right"):
                self._json_response(400, {"error": "side must be 'left' or 'right'"})
                return
            self.cmd_queue.put(("lock_arm" if lock else "unlock_arm", side))
            self._json_response(200, {"ok": True})

        elif self.path == "/estop":
            self.cmd_queue.put(("estop",))
            self._json_response(200, {"ok": True})

        elif self.path == "/lift":
            # {"action": "up"|"down"|"stop"}
            action = body.get("action", "stop")
            cmd_map = {"up": "lift_up", "down": "lift_down", "stop": "lift_stop"}
            cmd = cmd_map.get(action)
            if cmd is None:
                self._json_response(400, {"error": "action must be 'up', 'down', or 'stop'"})
                return
            self.cmd_queue.put((cmd,))
            self._json_response(200, {"ok": True})

        elif self.path == "/gamepad":
            # {"axes": [...], "buttons": [bool, ...], "enabled": bool}
            axes = [float(v) for v in body.get("axes", [])]
            buttons = [bool(v) for v in body.get("buttons", [])]
            enabled = bool(body.get("enabled", False))
            self.gamepad_state.update(axes, buttons, enabled)
            self._json_response(200, {"ok": True})

        elif self.path == "/gripper_calibration":
            # {"side": "left"|"right", "bound": "min"|"max"|"loosen"}
            # min/max captures the gripper's current Present_Position as the
            # new range_min/range_max; loosen widens motor limits so the
            # operator can drive past the current range during calibration.
            # All three persist to motor EEPROM and the on-disk calibration JSON.
            side = body.get("side")
            bound = body.get("bound")
            if side not in ("left", "right") or bound not in ("min", "max", "loosen"):
                self._json_response(
                    400,
                    {"error": "side must be 'left' or 'right', bound must be 'min', 'max', or 'loosen'"},
                )
                return
            if bound == "loosen":
                self.cmd_queue.put(("loosen_gripper_range", side))
            else:
                self.cmd_queue.put(("set_gripper_range", side, bound))
            self._json_response(200, {"ok": True})

        elif self.path == "/teleop_mode":
            # {"mode": "local"|"remote"}
            mode = body.get("mode")
            if not self.teleop_mode.set(mode):
                self._json_response(400, {"error": "mode must be 'local' or 'remote'"})
                return
            self._json_response(200, {"ok": True, "mode": self.teleop_mode.mode})

        elif self.path == "/base_input_source":
            # {"value": "off"|"gamepad"|"pedals"|"auto"}
            value = body.get("value")
            if not self.base_input_source.set(value):
                self._json_response(
                    400,
                    {"error": "value must be 'off', 'gamepad', 'pedals', or 'auto'"},
                )
                return
            # Any source change immediately halts the base + lift so the next
            # source starts from a known stopped state.
            self.cmd_queue.put(("wheels_stop",))
            self.cmd_queue.put(("lift_stop",))
            self._json_response(
                200, {"ok": True, "value": self.base_input_source.value}
            )

        elif self.path in ("/record/start", "/record/stop", "/record/label"):
            if self.tour_recorder is None:
                self._json_response(503, {"error": "recorder unavailable (no nav cam)"})
                return
            if self.path == "/record/start":
                result = self.tour_recorder.start(body.get("name"))
            elif self.path == "/record/stop":
                result = self.tour_recorder.stop()
            else:  # /record/label
                result = self.tour_recorder.add_label(body.get("label", ""))
            self._json_response(200 if result.get("ok") else 400, result)

        else:
            self.send_error(404)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def log_message(self, format, *a):
        pass  # silence per-request logs


def start_api_server(
    port: int,
    robot_state: RobotState,
    cmd_queue: queue.Queue,
    camera_readers: dict,
    gamepad_state: GamepadState,
    teleop_mode: TeleopMode,
    base_input_source: BaseInputSource,
    tour_recorder: "TourRecorder | None" = None,
):
    APIHandler.robot_state = robot_state
    APIHandler.cmd_queue = cmd_queue
    APIHandler.camera_readers = camera_readers
    APIHandler.gamepad_state = gamepad_state
    APIHandler.teleop_mode = teleop_mode
    APIHandler.base_input_source = base_input_source
    APIHandler.tour_recorder = tour_recorder
    httpd = ThreadingHTTPServer(("0.0.0.0", port), APIHandler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    print(f"HTTP API: http://0.0.0.0:{port}/state")
    return httpd


# ---------------------------------------------------------------------------
# Leader UDP listener: receives joint angles streamed from skynet at ~100 Hz
# and writes them straight to the bus from this thread (under bus_lock) so
# packets don't sit on the cmd_queue waiting for the main loop's next tick.
# Out-of-order/stale packets are dropped.
# ---------------------------------------------------------------------------
def start_leader_udp_listener(
    port: int,
    bus: FeetechMotorsBus,
    arm_locked: dict,
    available_arm_motors: set[str],
    teleop_mode: TeleopMode,
    estop: EstopMonitor,
) -> tuple[socket.socket, threading.Thread]:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.5)

    state = {"last_seq": None, "last_warn": 0.0, "pkt_count": 0, "drop_count": 0}

    def _run():
        while True:
            try:
                data, _addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return  # socket closed

            try:
                pkt = leader_wire.decode(data)
            except Exception as e:
                now = time.monotonic()
                if now - state["last_warn"] > 1.0:
                    print(f"Leader UDP: decode error: {e}")
                    state["last_warn"] = now
                continue

            if state["last_seq"] is not None and leader_wire.seq_lt(pkt.seq, state["last_seq"]):
                state["drop_count"] += 1
                continue
            state["last_seq"] = pkt.seq
            state["pkt_count"] += 1

            if teleop_mode.mode != "remote" or estop.engaged:
                continue

            goals: dict[str, int] = {}
            for side, arm in (("left", pkt.left), ("right", pkt.right)):
                if arm is None or not arm_locked.get(side):
                    continue
                for body_joint in leader_wire.BODY_JOINT_NAMES:
                    name = f"arm_{side}_{body_joint}"
                    if name in available_arm_motors:
                        deg = getattr(arm, body_joint)
                        goals[name] = _deg_to_tick(bus, name, deg)
                gripper = f"arm_{side}_{leader_wire.GRIPPER_NAME}"
                if gripper in available_arm_motors:
                    goals[gripper] = _pct_to_tick(bus, gripper, arm.gripper)

            if not goals:
                continue

            # Direct write under the shared bus lock — typical wait is <2 ms
            # (waiting for the main loop's current sync_read or sync_write to
            # finish), vs. up to ~25 ms if we'd queued instead.
            try:
                with _BUS_LOCK:
                    bus.sync_write("Goal_Position", goals, normalize=False)
            except Exception as e:
                now = time.monotonic()
                if now - state["last_warn"] > 1.0:
                    print(f"Leader UDP: sync_write failed: {e}")
                    state["last_warn"] = now

    t = threading.Thread(target=_run, daemon=True, name="leader-udp")
    t.start()
    print(f"Leader UDP listener: 0.0.0.0:{port}")
    return sock, t


# ---------------------------------------------------------------------------
# Pedal UDP listener: receives pedal-derived body commands from skynet at
# ~100 Hz and writes wheel velocities + lift commands directly to the bus.
# Gated by `base_input_source == "pedals"`; ignored otherwise. Owns a 300 ms
# watchdog: if no fresh packet arrives, wheels + lift are stopped exactly
# once on the stale edge.
# ---------------------------------------------------------------------------
def start_base_udp_listener(
    port: int,
    bus: FeetechMotorsBus,
    base_input_source: "BaseInputSource",
    estop: "EstopMonitor",
    wheels_available: bool,
    cmd_queue: queue.Queue,
    source_name: str = "pedals",
    thread_name: str = "pedal-udp",
) -> tuple[socket.socket, threading.Thread]:
    """Receive PedalPacket-shaped base commands on `port` and drive the wheels
    (+ lift) whenever `base_input_source` currently equals `source_name`.

    Used for both the pedal teleop stream ('pedals', :9998) and the autonomous
    conductor stream ('auto', :9997) — identical wire format, disjoint ports, each
    gated on its own source value so exactly one owns the base at a time.
    """
    log = f"{source_name.capitalize()} UDP"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.1)

    state = {
        "last_seq": None,
        "last_warn": 0.0,
        "last_fresh": 0.0,
        "active": False,
        "last_lift": "stop",
        # Wheels are only put in velocity mode at boot / e-stop-release; a servo
        # power-blip or mode change can leave them in position mode, where
        # Goal_Velocity writes are silently inert (the wheels just hold stiff).
        # So we (re)assert velocity mode on the edge where we START commanding,
        # instead of assuming boot-time setup still holds.
        "vmode_set": False,
    }

    def _stop_wheels():
        if not wheels_available:
            return
        for name in WHEEL_NAMES:
            try:
                _bus_write(bus, "Goal_Velocity", name, 0)
            except Exception as e:
                print(f"{log}: wheel-stop write failed ({name}): {e}")

    def _run():
        while True:
            try:
                data, _addr = sock.recvfrom(4096)
                got_pkt = True
            except socket.timeout:
                data = None
                got_pkt = False
            except OSError:
                return  # socket closed

            now = time.monotonic()

            if got_pkt:
                try:
                    pkt = pedal_wire.decode(data)
                except Exception as e:
                    if now - state["last_warn"] > 1.0:
                        print(f"{log}: decode error: {e}")
                        state["last_warn"] = now
                    continue

                # Drop duplicate / minor-reordered datagrams, but tolerate a
                # SENDER RESTART. When the conductor (or pedal sender) restarts,
                # its seq counter resets to ~0, which looks "older" than our
                # last_seq — the naive seq_lt check would then drop every packet
                # forever and the wheels would silently never move. So only skip a
                # *small* backward gap (genuine reordering); a large backward jump
                # means a fresh sender, so resync to it.
                if state["last_seq"] is not None:
                    behind = (state["last_seq"] - pkt.seq) & 0xFFFFFFFF
                    if 0 < behind < 256:
                        continue  # stale / reordered within the live stream
                state["last_seq"] = pkt.seq
                state["last_fresh"] = now

                # Drop if not the selected source or e-stopped — but keep
                # tracking last_fresh so the watchdog doesn't fire spuriously
                # when the source is intentionally off.
                if base_input_source.value != source_name or estop.engaged:
                    state["active"] = False
                    state["last_lift"] = "stop"
                    state["vmode_set"] = False  # re-assert velocity mode on next activation
                    continue

                if wheels_available:
                    # Explicitly ensure the wheels are in velocity mode before we
                    # command velocity — don't assume boot-time setup still holds.
                    # Once per activation (cheap: setup toggles torque + has
                    # sleeps, so we gate it on vmode_set, not per-packet).
                    if not state["vmode_set"]:
                        try:
                            setup_wheels_velocity_mode(bus)
                            state["vmode_set"] = True
                        except Exception as e:
                            if now - state["last_warn"] > 1.0:
                                print(f"{log}: wheel velocity-mode setup failed: {e}")
                                state["last_warn"] = now
                    # Per-wheel bus.write, matching the proven web-gamepad path
                    # (process_gamepad). sync_write("Goal_Velocity", ...) silently
                    # produced no motion here — the sign-magnitude encoding of
                    # negative wheel velocities didn't round-trip the same way it
                    # does through the individual write path.
                    cmds = body_to_wheel_raw(pkt.x_vel, pkt.y_vel, pkt.theta_vel)
                    for name, raw in cmds.items():
                        try:
                            _bus_write(bus, "Goal_Velocity", name, raw)
                        except Exception as e:
                            if now - state["last_warn"] > 1.0:
                                print(f"{log}: wheel write failed ({name}): {e}")
                                state["last_warn"] = now

                # Only enqueue lift commands on transition — process_commands
                # writes Goal_Velocity for lift each time, and at 100 Hz we'd
                # spam the bus otherwise.
                if pkt.lift != state["last_lift"]:
                    cmd_queue.put((f"lift_{pkt.lift}",))
                    state["last_lift"] = pkt.lift

                state["active"] = True
            else:
                # No packet this iteration — check watchdog.
                if state["active"] and (now - state["last_fresh"]) > GAMEPAD_WATCHDOG_S:
                    if base_input_source.value == source_name and not estop.engaged:
                        _stop_wheels()
                        if state["last_lift"] != "stop":
                            cmd_queue.put(("lift_stop",))
                            state["last_lift"] = "stop"
                    state["active"] = False
                    state["vmode_set"] = False  # re-assert velocity mode on next activation

    t = threading.Thread(target=_run, daemon=True, name=thread_name)
    t.start()
    print(f"{log} listener: 0.0.0.0:{port}  (source='{source_name}')")
    return sock, t


class TourRecorder:
    """Records a teleop 'tour' to disk for the topological nav map (Phase 3).

    While active, a background thread samples the wheel encoders, integrates a
    dead-reckoned pose (inverse of body_to_wheel_raw), grabs the latest full-res
    nav-cam frame, and writes both to a session directory at ~`rate_hz`. Label
    buttons in the web UI POST /record/label at a *stop*, tagging the current
    frame as a named goal node ('inward' / 'outward' for the garage test, the 14
    rooms for the house). Storage lives on the Pi; the session is rsync'd to
    skynet afterward where the offline builder subsamples it into a route.

    Layout:  <root>/<name>/
      meta.json      session info (name, cam, resolution, counts)
      frames.jsonl   one row per captured frame: {i, t, x, y, heading, file}
      labels.jsonl   label events: {t, label, frame_i}
      frames/000001.jpg ...
    """

    def __init__(self, bus, frame_getter, root: Path, cam_label: str, rate_hz: float = 5.0):
        self._bus = bus
        self._frame_getter = frame_getter  # () -> np.ndarray | None (full-res latest)
        self._root = root
        self._cam_label = cam_label
        self._period = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._active = False
        self._name: str | None = None
        self._dir: Path | None = None
        self._frames_dir: Path | None = None
        self._frames_fp = None
        self._labels_fp = None
        self._n_frames = 0
        self._labels: list[dict] = []
        # dead-reckoned pose (world frame, meters / radians) and last encoder read
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._last_raw: dict[str, float] | None = None

    def _read_wheels(self) -> dict[str, float] | None:
        try:
            with _BUS_LOCK:
                return self._bus.sync_read("Present_Position", list(WHEEL_NAMES), normalize=False)
        except Exception:
            return None

    def start(self, name: str | None = None) -> dict:
        with self._lock:
            if self._active:
                return {"ok": False, "error": "already recording", **self._status_locked()}
            name = (name or time.strftime("tour_%Y%m%d_%H%M%S")).strip().replace("/", "_")
            sdir = self._root / name
            frames_dir = sdir / "frames"
            frames_dir.mkdir(parents=True, exist_ok=True)
            self._name = name
            self._dir = sdir
            self._frames_dir = frames_dir
            self._frames_fp = (sdir / "frames.jsonl").open("w")
            self._labels_fp = (sdir / "labels.jsonl").open("w")
            self._n_frames = 0
            self._labels = []
            self._x = self._y = self._heading = 0.0
            self._last_raw = self._read_wheels()  # seed so first delta ~0
            (sdir / "meta.json").write_text(json.dumps({
                "name": name,
                "cam": self._cam_label,
                "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }, indent=2))
            self._active = True
            self._thread = threading.Thread(target=self._run, daemon=True, name="tour-rec")
            self._thread.start()
            print(f"Tour recorder: started '{name}' -> {sdir}")
            return {"ok": True, **self._status_locked()}

    def stop(self) -> dict:
        with self._lock:
            if not self._active:
                return {"ok": False, "error": "not recording", **self._status_locked()}
            self._active = False
            thread = self._thread
        if thread is not None:
            thread.join(timeout=2)
        with self._lock:
            summary = self._status_locked()
            # finalize meta with counts
            if self._dir is not None:
                (self._dir / "meta.json").write_text(json.dumps({
                    "name": self._name,
                    "cam": self._cam_label,
                    "n_frames": self._n_frames,
                    "labels": self._labels,
                    "ended_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                }, indent=2))
            for fp in (self._frames_fp, self._labels_fp):
                try:
                    if fp is not None:
                        fp.close()
                except Exception:
                    pass
            self._frames_fp = self._labels_fp = None
            print(f"Tour recorder: stopped '{self._name}' ({self._n_frames} frames, "
                  f"{len(self._labels)} labels)")
            return {"ok": True, **summary}

    def add_label(self, label: str) -> dict:
        label = (label or "").strip()
        with self._lock:
            if not self._active:
                return {"ok": False, "error": "not recording"}
            if not label:
                return {"ok": False, "error": "empty label"}
            # Attach to the frame most recently written (the operator is stopped).
            evt = {"t": time.time(), "label": label, "frame_i": max(self._n_frames - 1, 0)}
            self._labels.append(evt)
            if self._labels_fp is not None:
                self._labels_fp.write(json.dumps(evt) + "\n")
                self._labels_fp.flush()
            print(f"Tour recorder: label '{label}' @ frame {evt['frame_i']}")
            return {"ok": True, "label": label, "frame_i": evt["frame_i"]}

    def _run(self):
        next_t = time.monotonic()
        while True:
            with self._lock:
                if not self._active:
                    return
            self._tick()
            next_t += self._period
            slack = next_t - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.monotonic()

    def _tick(self):
        # 1) integrate dead-reckoned pose from wheel encoder deltas
        cur = self._read_wheels()
        if cur is not None and self._last_raw is not None:
            deltas = {}
            for n in WHEEL_NAMES:
                d = (cur[n] - self._last_raw[n] + 2048) % 4096 - 2048  # unwrap, signed
                deltas[n] = d
            dx, dy, dth = wheel_raw_delta_to_body(deltas)
            c, s = math.cos(self._heading), math.sin(self._heading)
            self._x += dx * c - dy * s
            self._y += dx * s + dy * c
            self._heading += dth
        if cur is not None:
            self._last_raw = cur

        # 2) capture the latest full-res nav-cam frame
        frame = self._frame_getter()
        if frame is None:
            return
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return
        with self._lock:
            if not self._active or self._frames_dir is None:
                return
            i = self._n_frames
            fname = f"{i:06d}.jpg"
            (self._frames_dir / fname).write_bytes(jpg.tobytes())
            row = {"i": i, "t": time.time(), "x": round(self._x, 4),
                   "y": round(self._y, 4), "heading": round(self._heading, 4),
                   "file": f"frames/{fname}"}
            if self._frames_fp is not None:
                self._frames_fp.write(json.dumps(row) + "\n")
            self._n_frames += 1

    def _status_locked(self) -> dict:
        return {
            "active": self._active,
            "name": self._name,
            "n_frames": self._n_frames,
            "x": round(self._x, 3),
            "y": round(self._y, 3),
            "heading_deg": round(math.degrees(self._heading), 1),
            "labels": list(self._labels),
        }

    def status(self) -> dict:
        with self._lock:
            return self._status_locked()


def start_voice_thread(
    host: str,
    port: int,
    stop_event: threading.Event,
    face_port: int = 8766,
) -> threading.Thread | None:
    """Fork the speech pipeline (voice/scripts/listen_and_play_realtime.py) on a
    daemon thread. It captures the local mic, plays TTS to the local speaker via
    a realtime connection to the speech server on `host:port` (skynet), and
    serves the face-state WebSocket on `face_port` for the "Face" tab.

    Imports are deferred to here: the pipeline pulls in `openai`/`sounddevice`,
    which aren't installed unless voice is in use, and the Pi has no mic until a
    USB audio device is added. A failure is logged, not fatal — robot control
    continues even if voice can't start.
    """
    import asyncio
    import traceback

    try:
        import listen_and_play_realtime as voice
    except Exception:
        print("== VOICE disabled: could not import speech pipeline ==", flush=True)
        print("   (install `openai sounddevice` + libportaudio2 in the env)", flush=True)
        traceback.print_exc()
        return None

    prompt_path = _REPO_ROOT / "voice" / "prompt.md"
    try:
        instructions = prompt_path.read_text().strip() or None
    except OSError:
        instructions = None

    # Pin to the ALSA "default" PCM by its exact PortAudio index. That PCM is the
    # ~/.asoundrc plug that resamples for the Jabra (16k mono -> 48k stereo).
    # PortAudio's *own* default-device pick is unreliable — it falls back to raw
    # HDMI, which rejects our 16 kHz mono format. None => let PortAudio choose.
    dev_idx = None
    try:
        import sounddevice as sd
        dev_idx = next(
            (i for i, d in enumerate(sd.query_devices()) if d["name"] == "default"),
            None,
        )
    except Exception:
        pass
    print(f"Voice audio device: {'default'!r} -> index {dev_idx}", flush=True)

    vargs = voice.ListenAndPlayRealtimeArguments(
        host=host,
        port=port,
        block_mic_during_playback=True,
        face_host="0.0.0.0",
        face_port=face_port,
        instructions=instructions,
        input_device=dev_idx,
        output_device=dev_idx,
    )

    def _run():
        # Keep voice alive across transient errors (device briefly busy at boot,
        # speech server restart, network blip). Never fatal to robot control.
        backoff = 2.0
        while not stop_event.is_set():
            try:
                asyncio.run(voice.listen_and_play_realtime(vargs, stop_event))
            except Exception:
                print("== VOICE error (robot control unaffected); will retry ==", flush=True)
                traceback.print_exc()
            if stop_event.is_set():
                break
            stop_event.wait(backoff)
            backoff = min(backoff * 1.5, 15.0)

    t = threading.Thread(target=_run, daemon=True, name="voice")
    t.start()
    print(f"Voice pipeline: speech server {host}:{port}, face WS 0.0.0.0:{face_port}")
    return t


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Viser web control panel for LeKiwi")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--api-port", type=int, default=8091)
    # Stable by-id path for fppe's motor-bus adapter (CH340). The bare
    # /dev/ttyACM* numbering shuffles when other USB devices (Jabra, webcams)
    # re-enumerate, so pin to the adapter's serial number instead.
    parser.add_argument(
        "--serial-port",
        default="/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6081221-if00",
    )
    parser.add_argument("--cam0", default="/dev/video0", help="First camera device")
    parser.add_argument("--cam1", default="/dev/video2", help="Second camera device")
    parser.add_argument(
        "--cam2",
        default="/dev/video4",
        help="Third camera device (fisheye nav cam; empty string disables)",
    )
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--cam-height", type=int, default=480)
    parser.add_argument(
        "--nav-cam",
        default="cam2",
        help="Camera label the tour recorder captures (nav/fisheye cam)",
    )
    parser.add_argument(
        "--tours-dir",
        default=str(Path.home() / "tours"),
        help="Where the tour recorder writes session dirs (on the Pi)",
    )
    parser.add_argument(
        "--estop-gpio",
        type=int,
        default=19,
        help="Raspberry Pi GPIO pin for e-stop input (HIGH=engaged, with pull-up). 0 disables.",
    )
    parser.add_argument(
        "--leader-udp-port",
        type=int,
        default=leader_wire.DEFAULT_PORT,
        help="UDP port to receive leader-arm joint-angle packets from skynet. 0 disables.",
    )
    parser.add_argument(
        "--pedal-udp-port",
        type=int,
        default=pedal_wire.DEFAULT_PORT,
        help="UDP port to receive pedal-derived base commands from skynet. 0 disables.",
    )
    parser.add_argument(
        "--auto-udp-port",
        type=int,
        default=9997,
        help="UDP port to receive autonomous base commands (PedalPacket wire) "
        "from the skynet conductor (visual nav, chase, etc.). 0 disables.",
    )
    parser.add_argument(
        "--voice",
        action="store_true",
        help="Fork the speech pipeline (mic in / speaker out via the skynet speech "
        "server) and serve the Face tab's state WebSocket. Off by default: the Pi "
        "needs a USB mic/speaker plus `openai sounddevice` + libportaudio2 first.",
    )
    parser.add_argument(
        "--voice-host",
        default="192.168.68.76",
        help="Host of the realtime speech server (skynet). Default 192.168.68.76.",
    )
    parser.add_argument(
        "--voice-port",
        type=int,
        default=8765,
        help="Port of the realtime speech server. Default 8765.",
    )
    args = parser.parse_args()

    # Ensure `pkill` (SIGTERM) runs the finally: block so cameras and motors
    # release cleanly. Without this, SIGTERM bypasses cleanup and leaves the
    # UVC driver in a state that makes the next process hang on open.
    def _on_sigterm(signum, _frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _on_sigterm)

    # ---- Connect motors via LeKiwi config ----
    config = LeKiwiConfig()
    config.left_port = args.serial_port
    r = LeKiwi(config)
    bus = r.left_bus
    bus.connect(handshake=False)

    left_motors: list[str] = r.left_arm_motors
    right_motors: list[str] = r.right_arm_motors
    all_arm_motors = left_motors + right_motors

    print(f"Connected. Left arm:  {left_motors}")
    print(f"           Right arm: {right_motors}")

    # ---- E-stop monitor ----
    estop = EstopMonitor(pin=args.estop_gpio)

    # Read initial positions & configure servos. If the e-stop is engaged the
    # bus will be unresponsive; wait it out instead of crashing the autostart.
    def _setup_motors_once():
        with _BUS_LOCK:
            ipos = bus.sync_read("Present_Position", all_arm_motors, normalize=False)
        setup_arm_position_mode(bus, all_arm_motors)
        setup_lift_velocity_mode(bus)
        wr = True
        wr_err: Exception | None = None
        try:
            setup_wheels_velocity_mode(bus)
        except Exception as e:
            wr = False
            wr_err = e
        with _BUS_LOCK:
            bus.sync_write("Goal_Position", ipos, normalize=False)
        return ipos, wr, wr_err

    last_log = 0.0
    while True:
        estop.wait_for_release()
        if estop.engaged:
            continue  # re-engaged before we got a chance; loop back
        try:
            initial_pos, wheels_ready, wheels_err = _setup_motors_once()
            break
        except Exception as e:
            now = time.monotonic()
            if now - last_log > 5.0:
                print(f"Motor setup failed ({e}). Retrying — check bus cable / motor power.")
                last_log = now
            time.sleep(1.0)

    print(f"Initial positions: {initial_pos}")
    if wheels_ready:
        print(f"Wheels ready: {WHEEL_NAMES}")
    else:
        print(f"Wheels not available ({wheels_err}); gamepad teleop will control lift only.")

    # ---- Cameras (threaded readers to avoid buffering lag) ----
    class CameraReader:
        """Continuously grabs frames in a background thread so the main loop always gets the latest."""
        def __init__(self, dev: str, width: int, height: int):
            self.cap = cv2.VideoCapture(dev)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._cond = threading.Condition()  # also serves as the lock
            self._frame = None
            self._version = 0  # bumped on every new frame
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

        def _loop(self):
            # Cap the read rate well below the camera's native fps. The
            # streamer only sends 15 fps anyway, and burning CPU + GIL on
            # 30+ fps reads we'll never use was starving the UDP teleop
            # listener thread.
            target_period = 1.0 / 20.0
            next_t = time.monotonic()
            while self._running:
                ret, frame = self.cap.read()
                if ret:
                    with self._cond:
                        self._frame = frame
                        self._version += 1
                        self._cond.notify_all()
                next_t += target_period
                slack = next_t - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
                else:
                    next_t = time.monotonic()  # we fell behind; resync

        @property
        def latest(self) -> np.ndarray | None:
            with self._cond:
                return self._frame

        def wait_for_new(
            self, last_version: int, timeout: float
        ) -> tuple[np.ndarray | None, int]:
            """Block until a frame newer than `last_version` arrives, or timeout."""
            with self._cond:
                if self._version == last_version:
                    self._cond.wait(timeout=timeout)
                return self._frame, self._version

        def release(self):
            # Order matters: releasing the capture first cancels any in-flight
            # V4L2 dqbuf so the reader thread's cap.read() returns promptly.
            # Otherwise the thread stays blocked in the kernel driver and
            # leaves UVC streaming state half-configured, which makes the
            # next process hang opening the device.
            self._running = False
            try:
                self.cap.release()
            except Exception:
                pass
            self._thread.join(timeout=2)

    cameras: dict[str, CameraReader] = {}
    cam_devs = [("cam0", args.cam0), ("cam1", args.cam1)]
    if args.cam2:  # fisheye nav cam; optional so a missing device is silent
        cam_devs.append(("cam2", args.cam2))
    for label, dev in cam_devs:
        reader = CameraReader(dev, args.cam_width, args.cam_height)
        if reader.cap.isOpened():
            cameras[label] = reader
            print(f"Camera '{label}' opened: {dev}")
        else:
            reader.release()
            print(f"Camera '{label}' failed to open: {dev}")

    # ---- URDF (optional 3D viz) ----
    urdf_viz = None

    # ---- Viser server ----
    server = viser.ViserServer(host="0.0.0.0", port=args.port)
    print(f"\nViser UI: http://0.0.0.0:{args.port}")

    if URDF_PATH.exists():
        try:
            from viser.extras import ViserUrdf

            urdf_viz = ViserUrdf(server, urdf_or_path=URDF_PATH)
            urdf_joint_names = urdf_viz.get_actuated_joint_names()
            print(f"URDF joints: {urdf_joint_names}")
        except Exception as e:
            print(f"URDF load failed ({e}); 3D view disabled.")

    # ---- Shared state ----
    arm_locked = {"left": True, "right": True}
    guard = {"updating": False}  # mutable flag shared across threads
    cmd_q: queue.Queue = queue.Queue()
    motor_sliders: dict[str, viser.GuiInputHandle] = {}
    # side -> (range_min display handle, range_max display handle) for the gripper
    gripper_displays: dict[str, tuple] = {}

    # ---- HTTP API state (server started after GUI is built so the kiosk
    # autostart's wait on port 8091 also implies the viser GUI is ready) ----
    robot_state = RobotState()
    robot_state.update(initial_pos, arm_locked)
    gamepad_state = GamepadState()
    teleop_mode = TeleopMode(initial="local")
    base_input_source = BaseInputSource(initial="off")

    # ---- Build GUI (tabbed sidebar) ----
    tabs = server.gui.add_tab_group()

    # ---- Pose tab ----
    with tabs.add_tab("Pose"):
        btn_home = server.gui.add_button("Home Position", color="blue")
        btn_arms_up = server.gui.add_button("Arms Up", color="blue")
        btn_middle = server.gui.add_button("Middle Position", color="blue")
        btn_estop = server.gui.add_button("E-Stop (Release All)", color="red")

    @btn_home.on_click
    def _(_):
        cmd_q.put(("go_home",))

    @btn_arms_up.on_click
    def _(_):
        cmd_q.put(("go_arms_up",))

    @btn_middle.on_click
    def _(_):
        cmd_q.put(("go_preset", MIDPOINT))

    @btn_estop.on_click
    def _(_):
        cmd_q.put(("estop",))

    # ---- Arm tab builder ----
    def build_arm_tab(side: str, motors: list[str], positions: dict):
        label = "Left Arm" if side == "left" else "Right Arm"
        prefix = "L" if side == "left" else "R"
        with tabs.add_tab(label):
            cb_lock = server.gui.add_checkbox(f"Lock {label}", initial_value=True)

            for motor_name in motors:
                short = motor_name.replace(f"arm_{side}_", "")
                nice = short.replace("_", " ").title()
                slider = server.gui.add_slider(
                    label=f"{prefix} {nice}",
                    min=0,
                    max=4095,
                    step=1,
                    initial_value=int(positions.get(motor_name, MIDPOINT)),
                )
                motor_sliders[motor_name] = slider

                def _make_cb(name: str, s):
                    def cb(_):
                        if not guard["updating"] and arm_locked[side]:
                            cmd_q.put(("goal_pos", name, int(s.value)))
                    return cb

                slider.on_update(_make_cb(motor_name, slider))

            @cb_lock.on_update
            def _(__, _side=side):
                if cb_lock.value:
                    cmd_q.put(("lock_arm", _side))
                else:
                    cmd_q.put(("unlock_arm", _side))

            # ---- Gripper range calibration ----
            # Lets the operator drive the gripper to a physical extreme (jaws
            # closed / fully open) and capture the current raw tick as the new
            # range_min / range_max. _pct_to_tick uses these to map the 0..100
            # leader-stream values back to raw ticks, so updating them here
            # immediately changes the teleop mapping. Persisted to the motor
            # EEPROM and to the on-disk calibration JSON.
            gripper_name = f"arm_{side}_gripper"
            gcal = bus.calibration.get(gripper_name)
            with server.gui.add_folder("Gripper calibration"):
                gmin = server.gui.add_number(
                    f"{prefix} gripper range_min",
                    initial_value=int(gcal.range_min) if gcal else 0,
                    disabled=True,
                )
                gmax = server.gui.add_number(
                    f"{prefix} gripper range_max",
                    initial_value=int(gcal.range_max) if gcal else 4095,
                    disabled=True,
                )
                btn_set_min = server.gui.add_button("Set min from current pos")
                btn_set_max = server.gui.add_button("Set max from current pos")
                # The motor enforces Min/Max_Position_Limit on Goal_Position,
                # so you can't drive past the *current* range_min/max with the
                # slider. "Loosen" temporarily widens those EEPROM limits to
                # the slider bounds so calibration can re-tighten them.
                btn_loosen = server.gui.add_button("Loosen limits (full ROM)")
            gripper_displays[side] = (gmin, gmax)

            # Use explicit method-call form (not decorator) and a closure
            # factory so each button captures its own side cleanly. The other
            # decorator-based handlers in this function rebind `_` repeatedly,
            # which mostly works but is fragile.
            def _make_cal_cb(_side: str, _kind: str, _bound: str | None):
                def cb(_event):
                    print(f"[gui] gripper-cal click: side={_side} kind={_kind} bound={_bound}", flush=True)
                    if _kind == "loosen":
                        cmd_q.put(("loosen_gripper_range", _side))
                    else:
                        cmd_q.put(("set_gripper_range", _side, _bound))
                return cb

            btn_set_min.on_click(_make_cal_cb(side, "set", "min"))
            btn_set_max.on_click(_make_cal_cb(side, "set", "max"))
            btn_loosen.on_click(_make_cal_cb(side, "loosen", None))

    build_arm_tab("left", left_motors, initial_pos)
    build_arm_tab("right", right_motors, initial_pos)

    # ---- Z tab ----
    with tabs.add_tab("Z"):
        btn_z_up = server.gui.add_button("Z Up", color="green")
        btn_z_down = server.gui.add_button("Z Down", color="orange")
        btn_z_stop = server.gui.add_button("Z Stop", color="gray")

    # ---- System tab ----
    # Restart button: exits this process so the autostart supervisor loop
    # relaunches a fresh one — the way to reload freshly-synced code (and pick up
    # env like ANTHROPIC_API_KEY) without a reboot. An "Arm restart" checkbox
    # gates it so a stray tap on the kiosk touchscreen can't kill robot control.
    with tabs.add_tab("System"):
        cb_restart_arm = server.gui.add_checkbox("Arm restart", initial_value=False)
        btn_restart = server.gui.add_button("Restart viser", color="red")

    @btn_restart.on_click
    def _(_):
        if not cb_restart_arm.value:
            print("[gui] restart clicked but not armed; check 'Arm restart' first", flush=True)
            return
        print("[gui] restart requested — exiting for supervisor relaunch", flush=True)
        # os._exit (not pkill -f viser_control.py): a hard exit of THIS process
        # tears down every daemon thread (voice, UDP listeners, HTTP API) and the
        # kernel reclaims the serial/audio fds, so the relaunched process can grab
        # them. Name-based pkill would also match the bash supervisor's own
        # command line (it contains "viser_control.py") and kill the loop meant
        # to relaunch us. Motors hold their last goal position across the gap.
        os._exit(0)

    # ---- HTTP API (started after the GUI build so port 8091 only opens
    # once viser's tab tree is ready; the kiosk autostart waits on 8091) ----
    # ---- Tour recorder (teleop -> topological-nav map; Phase 3) ----
    # Captures the nav cam + dead-reckoned pose while driving; label buttons in
    # the web UI tag stops. Only available when the nav cam opened and the wheels
    # are present (encoders back the dead-reckoning).
    tour_recorder = None
    nav_cam = cameras.get(args.nav_cam)
    if nav_cam is not None and wheels_ready:
        tour_recorder = TourRecorder(
            bus,
            frame_getter=lambda r=nav_cam: r.latest,
            root=Path(args.tours_dir),
            cam_label=args.nav_cam,
        )
        print(f"Tour recorder: ready (cam '{args.nav_cam}', -> {args.tours_dir})")
    else:
        print(f"Tour recorder: disabled (nav cam '{args.nav_cam}' or wheels unavailable)")

    start_api_server(
        args.api_port,
        robot_state,
        cmd_q,
        cameras,
        gamepad_state,
        teleop_mode,
        base_input_source,
        tour_recorder=tour_recorder,
    )

    # ---- Leader UDP listener (skynet -> fppe joint-angle stream) ----
    if args.leader_udp_port > 0:
        start_leader_udp_listener(
            args.leader_udp_port,
            bus,
            arm_locked=arm_locked,
            available_arm_motors=set(all_arm_motors),
            teleop_mode=teleop_mode,
            estop=estop,
        )

    # ---- Pedal UDP listener (skynet -> fppe pedal-derived base stream) ----
    if args.pedal_udp_port > 0:
        start_base_udp_listener(
            args.pedal_udp_port,
            bus,
            base_input_source=base_input_source,
            estop=estop,
            wheels_available=wheels_ready,
            cmd_queue=cmd_q,
            source_name="pedals",
            thread_name="pedal-udp",
        )

    # ---- Auto UDP listener (skynet conductor -> fppe autonomous base stream) ----
    if args.auto_udp_port > 0:
        start_base_udp_listener(
            args.auto_udp_port,
            bus,
            base_input_source=base_input_source,
            estop=estop,
            wheels_available=wheels_ready,
            cmd_queue=cmd_q,
            source_name="auto",
            thread_name="auto-udp",
        )

    # ---- Voice pipeline (mic/speaker <-> skynet speech server + Face tab) ----
    voice_stop = threading.Event()
    if args.voice:
        start_voice_thread(args.voice_host, args.voice_port, voice_stop)

    @btn_z_up.on_click
    def _(_):
        cmd_q.put(("lift_up",))

    @btn_z_down.on_click
    def _(_):
        cmd_q.put(("lift_down",))

    @btn_z_stop.on_click
    def _(_):
        cmd_q.put(("lift_stop",))

    # ---- URDF update ----
    def update_urdf(positions: dict[str, float]):
        if urdf_viz is None:
            return
        cfg = np.zeros(len(urdf_joint_names))
        for motor_name, urdf_joint in MOTOR_TO_URDF_JOINT.items():
            if urdf_joint in urdf_joint_names:
                idx = urdf_joint_names.index(urdf_joint)
                raw = positions.get(motor_name, MIDPOINT)
                cfg[idx] = raw_to_radians(raw) + URDF_JOINT_OFFSETS.get(urdf_joint, 0.0)
        urdf_viz.update_cfg(cfg)

    # ---- Process command queue (runs in main thread only) ----
    def process_commands():
        if estop.engaged:
            # Bus is unresponsive while e-stop is engaged. Drop pending commands
            # rather than letting them queue up to fire on release.
            while not cmd_q.empty():
                try:
                    cmd_q.get_nowait()
                except queue.Empty:
                    break
            return
        while not cmd_q.empty():
            try:
                cmd = cmd_q.get_nowait()
            except queue.Empty:
                break

            kind = cmd[0]

            if kind == "goal_pos":
                _, name, value = cmd
                _bus_write(bus, "Goal_Position", name, value)
                time.sleep(0.005)

            elif kind in ("go_preset", "go_home", "go_arms_up"):
                if kind == "go_home":
                    goals = {name: HOME_POSITION.get(name, MIDPOINT) for name in all_arm_motors}
                elif kind == "go_arms_up":
                    goals = {name: ARMS_UP_POSITION.get(name, MIDPOINT) for name in all_arm_motors}
                else:
                    _, target = cmd
                    goals = {name: target for name in all_arm_motors}
                # Lock both arms first, then move
                for side, motors in [("left", left_motors), ("right", right_motors)]:
                    if not arm_locked[side]:
                        setup_arm_position_mode(bus, motors)
                        arm_locked[side] = True
                with _BUS_LOCK:
                    bus.sync_write("Goal_Position", goals, normalize=False)
                guard["updating"] = True
                for name, slider in motor_sliders.items():
                    slider.value = goals[name]
                guard["updating"] = False

            elif kind == "estop":
                _bus_write(bus, "Goal_Velocity", "lift_axis", 0)
                time.sleep(0.02)
                for name in all_arm_motors:
                    _bus_write(bus, "Torque_Enable", name, 0)
                    time.sleep(0.02)
                arm_locked["left"] = False
                arm_locked["right"] = False

            elif kind == "lock_arm":
                _, side = cmd
                motors = left_motors if side == "left" else right_motors
                with _BUS_LOCK:
                    cur = bus.sync_read("Present_Position", motors, normalize=False)
                setup_arm_position_mode(bus, motors)
                with _BUS_LOCK:
                    bus.sync_write("Goal_Position", cur, normalize=False)
                arm_locked[side] = True
                guard["updating"] = True
                for name in motors:
                    if name in motor_sliders:
                        motor_sliders[name].value = int(cur[name])
                guard["updating"] = False

            elif kind == "unlock_arm":
                _, side = cmd
                motors = left_motors if side == "left" else right_motors
                for name in motors:
                    _bus_write(bus, "Torque_Enable", name, 0)
                    time.sleep(0.02)
                arm_locked[side] = False

            elif kind == "lift_up":
                _bus_write(bus, "Goal_Velocity", "lift_axis", degps_to_raw(-LIFT_SPEED_DEGPS))

            elif kind == "lift_down":
                _bus_write(bus, "Goal_Velocity", "lift_axis", degps_to_raw(LIFT_SPEED_DEGPS))

            elif kind == "lift_stop":
                _bus_write(bus, "Goal_Velocity", "lift_axis", 0)

            elif kind == "wheels_stop":
                if wheels_ready:
                    for name in WHEEL_NAMES:
                        try:
                            _bus_write(bus, "Goal_Velocity", name, 0)
                        except Exception:
                            pass

            elif kind == "loosen_gripper_range":
                _, _side = cmd
                gname = f"arm_{_side}_gripper"
                existing = bus.calibration.get(gname)
                if existing is None:
                    print(f"loosen_gripper_range: no existing calibration for {gname}")
                    continue
                # Min/Max_Position_Limit on Feetech servos are uint16; negative
                # values would be encoded as garbage by scservo_sdk and crash
                # the bus. Stick to the full single-turn range [0, 4095].
                new_cal = MotorCalibration(
                    id=existing.id,
                    drive_mode=existing.drive_mode,
                    homing_offset=existing.homing_offset,
                    range_min=0,
                    range_max=4095,
                )
                with _BUS_LOCK:
                    bus.write_calibration({gname: new_cal}, cache=False)
                bus.calibration[gname] = new_cal
                try:
                    r._save_calibration()
                except Exception as e:
                    print(f"loosen_gripper_range: failed to persist calibration JSON: {e}")
                disp = gripper_displays.get(_side)
                if disp is not None:
                    guard["updating"] = True
                    disp[0].value = int(new_cal.range_min)
                    disp[1].value = int(new_cal.range_max)
                    guard["updating"] = False
                print(
                    f"gripper {_side}: limits loosened to {new_cal.range_min}..{new_cal.range_max} "
                    "— drive to extremes and click Set min / Set max to re-tighten"
                )

            elif kind == "set_gripper_range":
                _, _side, bound = cmd
                gname = f"arm_{_side}_gripper"
                existing = bus.calibration.get(gname)
                if existing is None:
                    print(f"set_gripper_range: no existing calibration for {gname}")
                    continue
                with _BUS_LOCK:
                    cur = bus.sync_read("Present_Position", [gname], normalize=False)
                raw = int(cur[gname])
                new_cal = MotorCalibration(
                    id=existing.id,
                    drive_mode=existing.drive_mode,
                    homing_offset=existing.homing_offset,
                    range_min=raw if bound == "min" else existing.range_min,
                    range_max=raw if bound == "max" else existing.range_max,
                )
                # cache=False so we don't replace the bus's whole dict (which is
                # the same object as r.calibration); mutate the shared dict in
                # place instead so both views stay consistent.
                with _BUS_LOCK:
                    bus.write_calibration({gname: new_cal}, cache=False)
                bus.calibration[gname] = new_cal
                try:
                    r._save_calibration()
                except Exception as e:
                    print(f"set_gripper_range: failed to persist calibration JSON: {e}")
                disp = gripper_displays.get(_side)
                if disp is not None:
                    guard["updating"] = True
                    disp[0].value = int(new_cal.range_min)
                    disp[1].value = int(new_cal.range_max)
                    guard["updating"] = False
                print(
                    f"gripper {_side}: {bound}={raw} "
                    f"(range now {new_cal.range_min}..{new_cal.range_max})"
                )

    # ---- Gamepad → wheels + lift ----
    teleop_active = {"on": False}  # tracks whether wheels are currently driven by gamepad

    def apply_deadzone(vals: list[float]) -> list[float]:
        return [0.0 if abs(v) < GAMEPAD_DEADZONE else v for v in vals]

    def _stop_wheels_and_lift():
        if wheels_ready:
            for name in WHEEL_NAMES:
                try:
                    _bus_write(bus, "Goal_Velocity", name, 0)
                except Exception:
                    pass
        try:
            _bus_write(bus, "Goal_Velocity", "lift_axis", 0)
        except Exception:
            pass

    def process_gamepad():
        if estop.engaged:
            teleop_active["on"] = False
            return
        if base_input_source.value != "gamepad":
            # Pedals or "off" owns wheels+lift; silently drop. Coast once on
            # the edge so we don't leave the base spinning when the operator
            # switches sources mid-motion.
            if teleop_active["on"]:
                _stop_wheels_and_lift()
                teleop_active["on"] = False
            return
        if teleop_mode.mode not in ("local", "remote"):
            # No input authority at all — coast the base/lift down once.
            if teleop_active["on"]:
                _stop_wheels_and_lift()
                teleop_active["on"] = False
            return
        axes, buttons, enabled, age = gamepad_state.snapshot()
        fresh = enabled and age < GAMEPAD_WATCHDOG_S

        if fresh:
            # Ensure velocity mode on the edge into gamepad driving — don't
            # assume boot-time setup still holds (a servo blip can drop the
            # wheels into position mode, where Goal_Velocity is silently inert).
            if wheels_ready and not teleop_active["on"]:
                try:
                    setup_wheels_velocity_mode(bus)
                except Exception as e:
                    print(f"Gamepad: wheel velocity-mode setup failed: {e}")

            axes = apply_deadzone(axes)
            if len(axes) >= 3:
                x_cmd = -axes[1] * LIN_SPEED
                y_cmd = -axes[0] * LIN_SPEED
                th_cmd = -axes[2] * ANG_SPEED
            else:
                x_cmd = y_cmd = th_cmd = 0.0

            if wheels_ready:
                cmds = body_to_wheel_raw(x_cmd, y_cmd, th_cmd)
                for name, raw in cmds.items():
                    try:
                        _bus_write(bus, "Goal_Velocity", name, raw)
                    except Exception as e:
                        print(f"Wheel write failed ({name}): {e}")

            lift_axis_val = axes[AXIS_LIFT] if len(axes) > AXIS_LIFT else 0.0
            lift_raw = degps_to_raw(lift_axis_val * LIFT_SPEED_DEGPS)
            try:
                _bus_write(bus, "Goal_Velocity", "lift_axis", lift_raw)
            except Exception as e:
                print(f"Lift write failed: {e}")

            teleop_active["on"] = True
        elif teleop_active["on"]:
            # Edge: just went stale → stop wheels + lift once.
            _stop_wheels_and_lift()
            teleop_active["on"] = False

    # ---- Main loop ----
    # Catch C-level crashes (segfault, abort, sigbus) and dump a Python stack
    # before exiting. Combined with the broad try/except inside the loop, this
    # leaves us a fighting chance at debugging silent deaths.
    import faulthandler, traceback as _tb
    faulthandler.enable()
    print("Control loop running. Press Ctrl-C to quit.")
    estop_was_engaged = estop.engaged
    try:
        while True:
            try:
                process_commands()
                process_gamepad()
            except KeyboardInterrupt:
                raise
            except Exception:
                print("== EXCEPTION in main loop body ==", flush=True)
                _tb.print_exc()
                # Don't swallow forever — but a one-off serial error shouldn't
                # take the whole control plane down.
                time.sleep(0.05)
                continue

            if estop.engaged:
                if not estop_was_engaged:
                    print("E-stop ENGAGED. Servos unpowered; arms marked unlocked.")
                    arm_locked["left"] = False
                    arm_locked["right"] = False
                    estop_was_engaged = True
                robot_state.update({}, arm_locked, estop_engaged=True)
                time.sleep(0.1)
                continue

            if estop_was_engaged:
                # Just released — let servos finish powering up, then restore
                # velocity-mode setup for lift/wheels. Arms stay unlocked
                # (user must explicitly re-lock from the UI).
                print("E-stop RELEASED. Restoring lift/wheels (arms remain unlocked).")
                time.sleep(0.5)
                try:
                    setup_lift_velocity_mode(bus)
                    if wheels_ready:
                        try:
                            setup_wheels_velocity_mode(bus)
                        except Exception as e:
                            print(f"Wheel restore failed ({e})")
                    estop_was_engaged = False
                except Exception as e:
                    print(f"Lift restore failed ({e}); will retry.")
                    time.sleep(0.5)
                    continue

            # Read arm positions
            try:
                with _BUS_LOCK:
                    positions = bus.sync_read("Present_Position", all_arm_motors, normalize=False)
            except Exception as e:
                print(f"Read error: {e}")
                time.sleep(0.1)
                continue

            # Update sliders for unlocked arms (read-only feedback)
            guard["updating"] = True
            for name, slider in motor_sliders.items():
                side = "left" if "left" in name else "right"
                if not arm_locked[side]:
                    slider.value = int(positions[name])
            guard["updating"] = False

            # Update 3D visualization
            update_urdf(positions)

            # Update HTTP API state
            robot_state.update(positions, arm_locked, estop_engaged=False)

            # 5 ms target; the loop's actual floor is sync_read + sync_write
            # (~10–15 ms), landing the real tick around 50–70 Hz. The shorter
            # sleep cuts worst-case queue-dwell latency for leader UDP packets.
            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        # Signal the voice thread to close its audio streams / WS server. It's a
        # daemon thread, so this is best-effort graceful cleanup, not required.
        voice_stop.set()
        try:
            _bus_write(bus, "Goal_Velocity", "lift_axis", 0)
        except Exception:
            pass
        if wheels_ready:
            for name in WHEEL_NAMES:
                try:
                    _bus_write(bus, "Goal_Velocity", name, 0)
                except Exception:
                    pass
        for name in all_arm_motors:
            try:
                _bus_write(bus, "Torque_Enable", name, 0)
                time.sleep(0.02)
            except Exception:
                pass
        bus.disconnect(disable_torque=False)
        for reader in cameras.values():
            reader.release()
        print("Done.")


if __name__ == "__main__":
    main()
