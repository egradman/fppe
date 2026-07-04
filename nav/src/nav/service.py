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

from .controller import ApproachParams, ControlGains, NavPlanner, waypoints_to_velocity
from .harness import run_vint
from .mjpeg import MjpegReader
from .model import load_config, load_model
from .route import RouteMap


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
        # --- route (breadcrumb) following state ---
        self.route: RouteMap | None = None
        if getattr(args, "route", None):
            self.load_route(args.route)
        self._seq: list[int] | None = None   # node ids still to traverse, in order
        self._ptr = 0                         # index into _seq (current target breadcrumb)
        self._bmin = float("inf")             # closest-approach tracker for current breadcrumb
        self._bengaged = False
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

    # ---- route (breadcrumb) following ----

    def load_route(self, path: str):
        rm = RouteMap(path)
        rm.load_images()
        self.route = rm
        print(f"[nav-serve] route '{rm.data.get('name')}' loaded: "
              f"{len(rm.nodes)} nodes, labels {list(rm.labels)}")

    def _localize(self) -> int:
        """Which node is the robot nearest right now? ViNT distance from the live
        view to each node image; argmin. One-time at route start (~N inferences)."""
        snap = self.reader.snapshot()
        if snap is None:
            raise RuntimeError("no camera frames to localize against")
        context, _ = snap
        best_i, best_d = 0, float("inf")
        for n in self.route.nodes:
            d, _, _, _ = run_vint(self.model, self.config, context,
                                  self.route.image(n["id"]), self.device)
            if d < best_d:
                best_i, best_d = n["id"], d
        return best_i

    def set_route(self, label: str | None = None, node: int | None = None,
                  start: int | None = None) -> dict:
        """Begin following the loaded route toward `label`/`node`. Localizes the
        start node (unless `start` is given) and builds the breadcrumb sequence."""
        if self.route is None:
            return {"ok": False, "error": "no route loaded"}
        try:
            cur = start if start is not None else self._localize()
            goal = self.route.resolve_goal(label=label, node=node, cur=cur)
        except (KeyError, RuntimeError) as e:
            return {"ok": False, "error": str(e)}
        seq = self.route.sequence(cur, goal)
        with self._lock:
            self._goal = None  # route mode takes over from any single-goal
            self._seq = seq
            self._ptr = 0
            self._bmin = float("inf")
            self._bengaged = False
            self.planner.reset()
            reached = len(seq) == 0
            self._cmd = {"active": not reached, "reached": reached,
                         "vx": 0.0, "vy": 0.0, "theta": 0.0, "dist": None,
                         "reason": f"route from {cur} to {goal}" + (" (already there)" if reached else "")}
        return {"ok": True, "start": cur, "goal": goal, "sequence": seq}

    def clear_route(self):
        with self._lock:
            self._seq = None
            self._cmd = {"active": False, "reached": False, "vx": 0.0, "vy": 0.0,
                         "theta": 0.0, "dist": None, "reason": "route cleared"}

    def _advance_target(self, dist: float, action, is_final: bool):
        """One breadcrumb's stepping. Final breadcrumb: full NavPlanner (decel +
        precise closest-approach stop). Intermediate: cruise through, report
        reached on closest-approach or once within advance_dist — so the base
        flows past waypoints instead of stopping at each one."""
        if is_final:
            return self.planner.step(dist, action)
        a = self.approach
        self._bmin = min(self._bmin, dist)
        if dist <= a.goal_dist + a.decel_range:
            self._bengaged = True
        if dist < self.args.advance_dist:
            return (0.0, 0.0, 0.0), True, f"breadcrumb reached (dist {dist:.2f})"
        if self._bengaged and dist > self._bmin + a.overshoot_margin:
            return (0.0, 0.0, 0.0), True, f"breadcrumb passed (min {self._bmin:.2f})"
        self.gains.cruise_speed = self.args.speed
        return waypoints_to_velocity(action, self.gains), False, "driving"

    def command(self) -> dict:
        with self._lock:
            return dict(self._cmd)

    def gains_status(self) -> dict:
        g, a = self.gains, self.approach
        return {"speed": self.args.speed, "heading_gain": g.heading_gain,
                "max_omega": g.max_omega, "lookahead_idx": g.lookahead_idx,
                "advance_dist": self.args.advance_dist,
                "goal_dist": a.goal_dist, "decel_range": a.decel_range,
                "overshoot_margin": a.overshoot_margin}

    def set_gains(self, **kw) -> dict:
        """Live controller tuning — no ViNT reload. Any subset of the fields in
        gains_status(). 'lookahead_idx' nearer than -1 (the horizon end) makes the
        holonomic base hug the path curve instead of cutting toward the far end."""
        g, a = self.gains, self.approach
        with self._lock:
            if "heading_gain" in kw: g.heading_gain = float(kw["heading_gain"])
            if "max_omega" in kw: g.max_omega = float(kw["max_omega"])
            if "lookahead_idx" in kw: g.lookahead_idx = int(kw["lookahead_idx"])
            if "speed" in kw:
                self.args.speed = float(kw["speed"])
                self.planner.base_speed = float(kw["speed"])
            if "advance_dist" in kw: self.args.advance_dist = float(kw["advance_dist"])
            if "goal_dist" in kw: a.goal_dist = float(kw["goal_dist"])
            if "decel_range" in kw: a.decel_range = float(kw["decel_range"])
            if "overshoot_margin" in kw: a.overshoot_margin = float(kw["overshoot_margin"])
        return self.gains_status()

    def route_status(self) -> dict:
        with self._lock:
            seq, ptr = self._seq, self._ptr
            cmd = dict(self._cmd)
        if self.route is None:
            return {"loaded": False}
        return {"loaded": True, "name": self.route.data.get("name"),
                "n_nodes": len(self.route.nodes), "labels": self.route.labels,
                "following": seq is not None, "sequence": seq,
                "leg": (ptr + 1 if seq else 0), "command": cmd}

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
                seq = self._seq
                reached = self._cmd["reached"]

            if seq is not None:
                # Route (breadcrumb) mode.
                if reached:
                    time.sleep(period)
                    continue
                snap = self.reader.snapshot()
                if snap is None:
                    time.sleep(0.05)
                    continue
                self._tick_route(snap[0])
            elif goal is not None and not reached:
                # Single-goal mode (Phase 2 / conductor NavToGoal).
                snap = self.reader.snapshot()
                if snap is None:
                    time.sleep(0.05)
                    continue
                context, _ = snap
                dist, action, _, _ = run_vint(self.model, self.config, context, goal, self.device)
                (vx, vy, th), done, reason = self.planner.step(dist, action)
                with self._lock:
                    if self._goal is goal:  # not swapped/cleared mid-inference
                        self._cmd = {"active": True, "reached": done, "vx": vx, "vy": vy,
                                     "theta": th, "dist": round(dist, 3), "reason": reason}
            else:
                time.sleep(period)
                continue

            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)

    def _tick_route(self, context):
        """Drive toward the current breadcrumb; advance in-tick when reached so we
        publish a driving velocity toward the next one (no stop between crumbs)."""
        with self._lock:
            seq, ptr = self._seq, self._ptr
        for _hop in range(len(seq)):  # cap: advance at most once per remaining crumb
            target = seq[ptr]
            is_final = ptr == len(seq) - 1
            dist, action, _, _ = run_vint(self.model, self.config, context,
                                          self.route.image(target), self.device)
            (vx, vy, th), done, reason = self._advance_target(dist, action, is_final)
            if done and not is_final:
                ptr += 1
                self._bmin = float("inf")
                self._bengaged = False
                continue  # re-target the next breadcrumb with the same context
            with self._lock:
                if self._seq is seq:  # route not swapped/cleared mid-inference
                    self._ptr = ptr
                    self._cmd = {"active": not (done and is_final), "reached": done and is_final,
                                 "vx": vx, "vy": vy, "theta": th, "dist": round(dist, 3),
                                 "node": target, "leg": f"{ptr + 1}/{len(seq)}", "reason": reason}
            return


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

    def _read_body(self) -> bytes:
        n = int(self.headers.get("Content-Length", 0))
        return self.rfile.read(n) if n else b""

    def do_GET(self):
        if self.path == "/command":
            self._json(200, self.worker.command())
        elif self.path == "/health":
            self._json(200, self.worker.health())
        elif self.path == "/route":
            self._json(200, self.worker.route_status())
        elif self.path == "/gains":
            self._json(200, self.worker.gains_status())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        if self.path == "/goal":
            try:
                img = PILImage.open(io.BytesIO(self._read_body()))
                img.load()
            except Exception as e:
                self._json(400, {"error": f"bad image: {e}"})
                return
            self.worker.set_goal(img)
            self._json(200, {"ok": True, "size": list(img.size)})
        elif self.path == "/route":
            try:
                body = json.loads(self._read_body() or b"{}")
            except Exception as e:
                self._json(400, {"error": f"bad json: {e}"})
                return
            res = self.worker.set_route(label=body.get("label"), node=body.get("node"),
                                        start=body.get("start"))
            self._json(200 if res.get("ok") else 400, res)
        elif self.path == "/gains":
            try:
                body = json.loads(self._read_body() or b"{}")
            except Exception as e:
                self._json(400, {"error": f"bad json: {e}"})
                return
            self._json(200, self.worker.set_gains(**body))
        else:
            self._json(404, {"error": "not found"})

    def do_DELETE(self):
        if self.path == "/goal":
            self.worker.clear_goal()
            self._json(200, {"ok": True})
        elif self.path == "/route":
            self.worker.clear_route()
            self._json(200, {"ok": True})
        else:
            self._json(404, {"error": "not found"})


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
    p.add_argument("--route", default=None, help="route.json to load for breadcrumb following")
    p.add_argument("--advance-dist", type=float, default=1.5,
                   help="ViNT distance below which an intermediate breadcrumb counts as reached")
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
