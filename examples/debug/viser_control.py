#!/usr/bin/env python3
"""Viser-based web control panel for dual-arm LeKiwi robot.

Provides browser UI with:
  - Per-joint sliders for both arms (raw 0-4095)
  - Lock/unlock checkbox per arm (torque enable/disable)
  - Home and Middle position buttons
  - Elevator Z Up / Z Down / Z Stop buttons
  - Live 3D URDF visualization

HTTP API (on --api-port, default 8091):
  GET  /state                — JSON snapshot of all motor positions and arm lock states
  POST /goal_position        — {"motor_name": raw_value, ...}
  POST /go_middle            — send all arms to midpoint (2048)
  POST /lock_arm             — {"side": "left"|"right", "lock": true|false}
  POST /estop                — emergency stop (release all torque)
  POST /lift                 — {"action": "up"|"down"|"stop"}

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
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
import mimetypes
from pathlib import Path

import cv2
import numpy as np
import viser

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.robots.alohamini.config_lekiwi import LeKiwiConfig
from lerobot.robots.alohamini.lekiwi import LeKiwi

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MIDPOINT = 2048
STEPS_PER_DEG = 4096.0 / 360.0
LIFT_SPEED_DEGPS = 180.0

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



def _bus_write(bus: FeetechMotorsBus, item: str, name: str, value: int, retries: int = 3):
    """Write with retry to handle transient serial errors on a busy bus."""
    for attempt in range(retries):
        try:
            bus.write(item, name, value, normalize=False)
            return
        except Exception:
            if attempt == retries - 1:
                raise
            time.sleep(0.05)


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


# ---------------------------------------------------------------------------
# HTTP API
# ---------------------------------------------------------------------------
class RobotState:
    """Thread-safe container for the latest robot state, read by the API."""

    def __init__(self):
        self._lock = threading.Lock()
        self._positions: dict[str, int] = {}
        self._arm_locked: dict[str, bool] = {"left": False, "right": False}

    def update(self, positions: dict[str, int], arm_locked: dict[str, bool]):
        with self._lock:
            self._positions = dict(positions)
            self._arm_locked = dict(arm_locked)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "positions": dict(self._positions),
                "arm_locked": dict(self._arm_locked),
            }


WEB_DIST = Path(__file__).parent / "web-ui" / "dist"


class APIHandler(BaseHTTPRequestHandler):
    robot_state: RobotState
    cmd_queue: queue.Queue
    camera_readers: dict  # label -> CameraReader

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
                    small = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_NEAREST)
                    _, jpg = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg.tobytes())
                    self.wfile.write(b"\r\n")
                time.sleep(0.05)  # ~20 fps
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
            self._json_response(200, self.robot_state.snapshot())
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


def start_api_server(port: int, robot_state: RobotState, cmd_queue: queue.Queue, camera_readers: dict):
    APIHandler.robot_state = robot_state
    APIHandler.cmd_queue = cmd_queue
    APIHandler.camera_readers = camera_readers
    httpd = ThreadingHTTPServer(("0.0.0.0", port), APIHandler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    print(f"HTTP API: http://0.0.0.0:{port}/state")
    return httpd


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Viser web control panel for LeKiwi")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--api-port", type=int, default=8091)
    parser.add_argument("--serial-port", default="/dev/ttyACM0")
    parser.add_argument("--cam0", default="/dev/video0", help="First camera device")
    parser.add_argument("--cam1", default="/dev/video2", help="Second camera device")
    parser.add_argument("--cam-width", type=int, default=640)
    parser.add_argument("--cam-height", type=int, default=480)
    args = parser.parse_args()

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

    # Read initial positions & configure servos
    initial_pos = bus.sync_read("Present_Position", all_arm_motors, normalize=False)
    print(f"Initial positions: {initial_pos}")

    setup_arm_position_mode(bus, all_arm_motors)
    setup_lift_velocity_mode(bus)
    bus.sync_write("Goal_Position", initial_pos, normalize=False)

    # ---- Cameras (threaded readers to avoid buffering lag) ----
    class CameraReader:
        """Continuously grabs frames in a background thread so the main loop always gets the latest."""
        def __init__(self, dev: str, width: int, height: int):
            self.cap = cv2.VideoCapture(dev)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._lock = threading.Lock()
            self._frame = None
            self._running = True
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

        def _loop(self):
            while self._running:
                ret, frame = self.cap.read()
                if ret:
                    with self._lock:
                        self._frame = frame

        @property
        def latest(self) -> np.ndarray | None:
            with self._lock:
                return self._frame

        def release(self):
            self._running = False
            self._thread.join(timeout=2)
            self.cap.release()

    cameras: dict[str, CameraReader] = {}
    for label, dev in [("cam0", args.cam0), ("cam1", args.cam1)]:
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

    # ---- HTTP API ----
    robot_state = RobotState()
    robot_state.update(initial_pos, arm_locked)
    start_api_server(args.api_port, robot_state, cmd_q, cameras)

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

    build_arm_tab("left", left_motors, initial_pos)
    build_arm_tab("right", right_motors, initial_pos)

    # ---- Z tab ----
    with tabs.add_tab("Z"):
        btn_z_up = server.gui.add_button("Z Up", color="green")
        btn_z_down = server.gui.add_button("Z Down", color="orange")
        btn_z_stop = server.gui.add_button("Z Stop", color="gray")

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
                cur = bus.sync_read("Present_Position", motors, normalize=False)
                setup_arm_position_mode(bus, motors)
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

    # ---- Main loop ----
    print("Control loop running. Press Ctrl-C to quit.")
    try:
        while True:
            process_commands()

            # Read arm positions
            try:
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
            robot_state.update(positions, arm_locked)

            time.sleep(0.05)  # ~20 Hz

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        try:
            _bus_write(bus, "Goal_Velocity", "lift_axis", 0)
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
