"""Phase 2 executor: drive the fppe base toward a single goal photo.

The smallest honest milestone on real hardware — no graph, no labels, no
language. One hop:

  1. read the live fisheye MJPEG feed (context window of the last few frames),
  2. run ViNT (context, goal_photo) -> distance + waypoints,
  3. a proportional controller turns the waypoints into a body velocity,
  4. stream that velocity as a PedalPacket-shaped UDP datagram to viser_control,
  5. stop when ViNT's distance-to-goal drops below a threshold.

Runs on skynet (GPU). The fppe Pi must have `viser_control.py` up with a nav UDP
listener (`--nav-udp-port 9997`, on by default) and the operator must set
`base_input_source` to "nav" (web UI) to actually let the wheels move. Keep a
hand on the physical e-stop for the first base-moving run.

    uv run nav-drive --goal goal.jpg --robot-host fppe
    uv run nav-snap  --out goal.jpg           # grab a goal photo from the fisheye
"""

import argparse
import socket
import threading
import time

import torch
from PIL import Image as PILImage

from .controller import ControlGains, waypoints_to_velocity
from .harness import run_vint
from .mjpeg import MjpegReader
from .model import load_config, load_model
from . import wire


class VelocitySender:
    """Streams the latest commanded velocity at a fixed rate over UDP.

    Decoupled from the (slower, variable-latency) inference loop so the fppe
    300 ms wheel watchdog is fed continuously: the control loop just updates the
    target, this thread keeps the packets flowing. Setting the target to zero and
    letting it stream is the graceful stop; the watchdog is the backstop.
    """

    def __init__(self, host: str, port: int, rate_hz: float = 20.0):
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._period = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._vel = (0.0, 0.0, 0.0)
        self._seq = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="nav-send")

    def start(self) -> "VelocitySender":
        self._thread.start()
        return self

    def set_velocity(self, x_vel: float, y_vel: float, theta_vel: float):
        with self._lock:
            self._vel = (x_vel, y_vel, theta_vel)

    def stop(self):
        self.set_velocity(0.0, 0.0, 0.0)
        # Let a few zero packets go out before tearing the socket down.
        time.sleep(3 * self._period)
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._sock.close()

    def _run(self):
        while not self._stop.is_set():
            with self._lock:
                x, y, th = self._vel
                self._seq += 1
                seq = self._seq
            try:
                self._sock.sendto(
                    wire.encode(seq, x, y, th, lift="stop", sent_ts=time.time()),
                    self._addr,
                )
            except Exception as e:
                print(f"[nav] send error: {e}")
            time.sleep(self._period)


def _resolve_device(name: str) -> str:
    if name == "auto":
        return "cuda" if torch.cuda.is_available() else "cpu"
    return name


def drive(args):
    device = _resolve_device(args.device)
    config = load_config(args.config)
    goal = PILImage.open(args.goal).convert("RGB")

    print(f"[nav] model_type={config['model_type']}  device={device}")
    print(f"[nav] loading model + weights...")
    model = load_model(config, device)
    print(f"[nav] model ready ({sum(p.numel() for p in model.parameters())/1e6:.1f}M params)")

    gains = ControlGains(
        cruise_speed=args.speed,
        max_omega=args.max_omega,
        heading_gain=args.heading_gain,
        invert_y=args.invert_y,
        invert_theta=args.invert_theta,
    )

    feed = f"http://{args.robot_host}:{args.cam_port}/mjpeg/{args.cam}"
    reader = MjpegReader(feed, context_len=config["context_size"] + 1).start()
    print(f"[nav] reading fisheye feed: {feed}")

    sender = VelocitySender(args.robot_host, args.port, args.send_rate)
    if args.arm:
        _set_base_source(args.robot_host, args.cam_port, "nav")
        print("[nav] armed base_input_source=nav via HTTP")
    else:
        print("[nav] NOT armed — set base_input_source='nav' in the web UI to move.")
    sender.start()

    print("[nav] waiting for camera warmup...")
    period = 1.0 / args.rate
    reached = False
    base_speed = args.speed
    min_dist = float("inf")     # closest approach seen so far
    engaged = False             # True once we've entered the approach band
    engage_at = args.goal_dist + args.decel_range
    try:
        while True:
            t0 = time.monotonic()
            snap = reader.snapshot()
            if snap is None:
                time.sleep(0.05)
                continue
            context, _latest = snap

            dist, action, _, _ = run_vint(model, config, context, goal, device)
            min_dist = min(min_dist, dist)
            if dist <= engage_at:
                engaged = True

            # Hard stop: distance head crossed the goal threshold.
            if dist < args.goal_dist:
                print(f"[nav] dist {dist:.2f} < goal-dist {args.goal_dist} — reached, stopping.")
                reached = True
                break
            # Closest-approach stop: once in the approach band, if distance has
            # risen clearly above the minimum, we've passed the nearest point —
            # stop instead of orbiting. Robust to the distance head plateauing
            # above goal-dist (as it did on the first run: floored ~3.5).
            if engaged and dist > min_dist + args.overshoot_margin:
                print(f"[nav] passed closest approach (min {min_dist:.2f}, now {dist:.2f}) — stopping.")
                reached = True
                break

            # Decelerate on approach: full cruise far out, easing to min_speed_frac
            # as distance falls toward goal-dist, so it creeps in rather than
            # overshooting.
            span = max(args.decel_range, 1e-6)
            frac = (dist - args.goal_dist) / span
            frac = max(args.min_speed_frac, min(1.0, frac))
            gains.cruise_speed = base_speed * frac

            x_vel, y_vel, theta_vel = waypoints_to_velocity(action, gains)
            sender.set_velocity(x_vel, y_vel, theta_vel)
            print(
                f"[nav] dist={dist:5.2f} min={min_dist:5.2f}  vx={x_vel:+.3f} "
                f"vy={y_vel:+.3f} w={theta_vel:+5.1f}  frames={reader.frame_count}"
            )

            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        print("\n[nav] interrupted — stopping base.")
    finally:
        sender.set_velocity(0.0, 0.0, 0.0)
        sender.stop()
        reader.stop()
        if args.arm:
            _set_base_source(args.robot_host, args.cam_port, "off")
            print("[nav] disarmed base_input_source=off")
    return 0 if reached else 1


def _set_base_source(host: str, port: int, value: str):
    """POST base_input_source to viser_control. Best-effort convenience for --arm."""
    import json
    from urllib.request import Request, urlopen

    url = f"http://{host}:{port}/base_input_source"
    body = json.dumps({"value": value}).encode()
    try:
        req = Request(url, data=body, headers={"Content-Type": "application/json"})
        urlopen(req, timeout=2.0).read()
    except Exception as e:
        print(f"[nav] WARNING: could not set base_input_source={value}: {e}")


def snap(args):
    """Grab one frame off the fisheye feed and save it — for capturing a goal photo."""
    feed = f"http://{args.robot_host}:{args.cam_port}/mjpeg/{args.cam}"
    reader = MjpegReader(feed, context_len=1).start()
    print(f"[nav] grabbing a frame from {feed} ...")
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        with reader._lock:
            img = reader._latest
        if img is not None:
            img.save(args.out)
            print(f"[nav] saved {args.out} ({img.size[0]}x{img.size[1]})")
            reader.stop()
            return 0
        time.sleep(0.1)
    reader.stop()
    print("[nav] timed out waiting for a frame")
    return 1


def _add_feed_args(p):
    p.add_argument("--robot-host", default="fppe", help="fppe hostname/IP")
    p.add_argument("--cam-port", type=int, default=8091, help="viser_control HTTP port")
    p.add_argument("--cam", default="cam2", help="MJPEG camera label (fisheye = cam2)")


def drive_main():
    p = argparse.ArgumentParser(description="Phase 2 nav executor (drive to a goal photo)")
    _add_feed_args(p)
    p.add_argument("--goal", required=True, help="goal photo (jpg/png)")
    p.add_argument("--config", default="vint.yaml")
    p.add_argument("--device", default="auto", help="cuda | cpu | auto")
    p.add_argument("--port", type=int, default=wire.DEFAULT_PORT, help="fppe nav UDP port")
    p.add_argument("--rate", type=float, default=4.0, help="control (inference) loop Hz")
    p.add_argument("--send-rate", type=float, default=20.0, help="UDP packet stream Hz")
    p.add_argument("--speed", type=float, default=0.12, help="cruise speed m/s")
    p.add_argument("--max-omega", type=float, default=20.0, help="yaw cap deg/s")
    p.add_argument("--heading-gain", type=float, default=0.30, help="deg/s per deg heading err")
    p.add_argument("--goal-dist", type=float, default=2.0, help="hard-stop when ViNT distance < this")
    p.add_argument("--decel-range", type=float, default=3.0,
                   help="distance above goal-dist over which speed eases from full "
                        "cruise down to min-speed-frac (also sets the approach band)")
    p.add_argument("--min-speed-frac", type=float, default=0.3,
                   help="floor on cruise-speed fraction near the goal (creep speed)")
    p.add_argument("--overshoot-margin", type=float, default=0.6,
                   help="once in the approach band, stop if distance rises this far "
                        "above its minimum (closest-approach / passed-the-goal detector)")
    p.add_argument("--invert-y", action="store_true", help="flip strafe sign")
    p.add_argument("--invert-theta", action="store_true", help="flip yaw sign")
    p.add_argument("--arm", action="store_true",
                   help="auto-set base_input_source=nav on start / off on exit "
                        "(default off: arm manually from the web UI, hand on e-stop)")
    return drive(p.parse_args())


def snap_main():
    p = argparse.ArgumentParser(description="Save one fisheye frame as a goal photo")
    _add_feed_args(p)
    p.add_argument("--out", default="goal.jpg", help="output image path")
    return snap(p.parse_args())


if __name__ == "__main__":
    raise SystemExit(drive_main())
