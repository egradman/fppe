"""nav-serve — the visual-nav service the conductor reads in-tick.

Holds ViNT + the fisheye reader loaded once, runs a background loop that turns
the live view + the current goal photo into a body-velocity command, and serves
it over HTTP. It sends **no UDP to the robot** — the conductor's BaseCmdChannel
(ctx.base) is the single writer to the Pi's `auto` source on :9997. This mirrors
the perception service: a GPU worker that a thin client polls each tick.

Endpoints:
  POST   /goal      body = raw image bytes (jpeg/png)  -> set/replace the goal, start driving
  DELETE /goal                                         -> clear the goal, command goes inactive
  GET    /command   -> {active, reached, vx, vy, theta, dist, reason}
  GET    /health    -> {ok, device, frames, has_goal}

    uv run nav-serve --robot-host fppe --serve-port 8107
"""

import argparse
import io
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import torch
from PIL import Image as PILImage

from .controller import ApproachParams, ControlGains, NavPlanner
from .harness import run_vint
from .mjpeg import MjpegReader
from .model import load_config, load_model


class NavWorker:
    """Owns the model + fisheye reader + planner; runs the inference loop."""

    def __init__(self, args, device: str):
        self.args = args
        self.device = device
        self.config = load_config(args.config)
        self.model = load_model(self.config, device)
        self.reader = MjpegReader(
            f"http://{args.robot_host}:{args.cam_port}/mjpeg/{args.cam}",
            context_len=self.config["context_size"] + 1,
        )
        self.gains = ControlGains(
            cruise_speed=args.speed, max_omega=args.max_omega,
            heading_gain=args.heading_gain,
            invert_y=args.invert_y, invert_theta=args.invert_theta,
        )
        self.approach = ApproachParams(
            goal_dist=args.goal_dist, decel_range=args.decel_range,
            min_speed_frac=args.min_speed_frac, overshoot_margin=args.overshoot_margin,
        )
        self.planner = NavPlanner(args.speed, self.gains, self.approach)

        self._lock = threading.Lock()
        self._goal: PILImage.Image | None = None
        self._cmd = {"active": False, "reached": False, "vx": 0.0, "vy": 0.0,
                     "theta": 0.0, "dist": None, "reason": "no goal"}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="nav-worker")

    def start(self):
        self.reader.start()
        self._thread.start()

    def set_goal(self, img: PILImage.Image):
        with self._lock:
            self._goal = img.convert("RGB")
            self.planner.reset()
            self._cmd = {"active": True, "reached": False, "vx": 0.0, "vy": 0.0,
                         "theta": 0.0, "dist": None, "reason": "goal set; warming up"}

    def clear_goal(self):
        with self._lock:
            self._goal = None
            self._cmd = {"active": False, "reached": False, "vx": 0.0, "vy": 0.0,
                         "theta": 0.0, "dist": None, "reason": "goal cleared"}

    def command(self) -> dict:
        with self._lock:
            return dict(self._cmd)

    def health(self) -> dict:
        with self._lock:
            has_goal = self._goal is not None
        return {"ok": True, "device": self.device,
                "frames": self.reader.frame_count, "has_goal": has_goal}

    def _loop(self):
        period = 1.0 / self.args.rate
        while not self._stop.is_set():
            t0 = time.monotonic()
            with self._lock:
                goal = self._goal
                reached = self._cmd["reached"]
            # No goal, or already reached: publish a safe zero and idle.
            if goal is None or reached:
                time.sleep(period)
                continue
            snap = self.reader.snapshot()
            if snap is None:
                time.sleep(0.05)
                continue
            context, _ = snap
            dist, action, _, _ = run_vint(self.model, self.config, context, goal, self.device)
            (vx, vy, th), done, reason = self.planner.step(dist, action)
            with self._lock:
                # Only publish if the goal wasn't swapped/cleared mid-inference.
                if self._goal is goal:
                    self._cmd = {"active": True, "reached": done, "vx": vx, "vy": vy,
                                 "theta": th, "dist": round(dist, 3), "reason": reason}
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)


class _Handler(BaseHTTPRequestHandler):
    worker: NavWorker

    def log_message(self, *a):  # quiet
        pass

    def _json(self, code: int, data: dict):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/command":
            self._json(200, self.worker.command())
        elif self.path == "/health":
            self._json(200, self.worker.health())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/goal":
            self._json(404, {"error": "not found"})
            return
        n = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(n) if n else b""
        try:
            img = PILImage.open(io.BytesIO(raw))
            img.load()
        except Exception as e:
            self._json(400, {"error": f"bad image: {e}"})
            return
        self.worker.set_goal(img)
        self._json(200, {"ok": True, "size": list(img.size)})

    def do_DELETE(self):
        if self.path != "/goal":
            self._json(404, {"error": "not found"})
            return
        self.worker.clear_goal()
        self._json(200, {"ok": True})


def main():
    p = argparse.ArgumentParser(description="Visual-nav service (ViNT) for the conductor")
    p.add_argument("--robot-host", default="fppe", help="fppe hostname/IP")
    p.add_argument("--cam-port", type=int, default=8091, help="viser_control HTTP port")
    p.add_argument("--cam", default="cam2", help="MJPEG camera label (fisheye = cam2)")
    p.add_argument("--serve-port", type=int, default=8107, help="this service's HTTP port")
    p.add_argument("--config", default="vint.yaml")
    p.add_argument("--device", default="auto", help="cuda | cpu | auto")
    p.add_argument("--rate", type=float, default=4.0, help="inference loop Hz")
    p.add_argument("--speed", type=float, default=0.12, help="cruise speed m/s")
    p.add_argument("--max-omega", type=float, default=20.0, help="yaw cap deg/s")
    p.add_argument("--heading-gain", type=float, default=0.30, help="deg/s per deg heading err")
    p.add_argument("--goal-dist", type=float, default=2.0, help="hard-stop distance")
    p.add_argument("--decel-range", type=float, default=3.0, help="approach deceleration band")
    p.add_argument("--min-speed-frac", type=float, default=0.3, help="creep-speed floor")
    p.add_argument("--overshoot-margin", type=float, default=0.6, help="closest-approach stop margin")
    p.add_argument("--invert-y", action="store_true")
    p.add_argument("--invert-theta", action="store_true")
    args = p.parse_args()

    device = ("cuda" if torch.cuda.is_available() else "cpu") if args.device == "auto" else args.device
    print(f"[nav-serve] loading ViNT on {device} ...")
    worker = NavWorker(args, device)
    worker.start()
    _Handler.worker = worker
    srv = ThreadingHTTPServer(("0.0.0.0", args.serve_port), _Handler)
    print(f"[nav-serve] ready on :{args.serve_port}  (fisheye {args.robot_host}:{args.cam_port}/{args.cam})")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n[nav-serve] shutting down")
    finally:
        worker._stop.set()
        worker.reader.stop()


if __name__ == "__main__":
    main()
