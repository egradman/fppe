"""Stream leader-arm joint angles from skynet to fppe over UDP."""

from __future__ import annotations

import argparse
import logging
import socket
import time
from concurrent.futures import ThreadPoolExecutor

from . import wire
from .arms import LeaderArm
from .calibrate import LEFT_PORT, RIGHT_PORT
from .calibration import Calibration

logger = logging.getLogger("leader_teleop")


def _make_arm(side: str) -> LeaderArm:
    port = LEFT_PORT if side == "left" else RIGHT_PORT
    return LeaderArm(port=port, name=side)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fppe-host", default="fppe", help="Hostname/IP of the fppe Pi.")
    parser.add_argument("--port", type=int, default=wire.DEFAULT_PORT)
    parser.add_argument("--rate", type=float, default=100.0, help="Send rate in Hz.")
    parser.add_argument(
        "--arms",
        choices=("both", "left", "right"),
        default="both",
        help="Which arm(s) to stream.",
    )
    parser.add_argument(
        "--echo-only",
        action="store_true",
        help="Print packets locally instead of sending; useful for debugging.",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    sides = ("left", "right") if args.arms == "both" else (args.arms,)
    arms: dict[str, LeaderArm] = {}
    for side in sides:
        arm = _make_arm(side)
        cal = Calibration.load(side)
        if cal is None:
            raise SystemExit(
                f"No calibration for {side} arm. Run `uv run leader-calibrate --arms {side}` first."
            )
        logger.info("Connecting %s leader on %s ...", side, arm.port)
        arm.connect()
        arm.apply_calibration(cal)
        arms[side] = arm

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect((args.fppe_host, args.port))
    logger.info("Streaming -> %s:%d at %.1f Hz", args.fppe_host, args.port, args.rate)

    period = 1.0 / args.rate
    seq = 0
    next_t = time.perf_counter() + period

    stats_t0 = time.perf_counter()
    stats_frames = 0
    stats_max_read_ms = 0.0

    try:
        with ThreadPoolExecutor(max_workers=max(1, len(arms))) as pool:
            while True:
                read_start = time.perf_counter()
                futures = {side: pool.submit(arm.read_angles_deg) for side, arm in arms.items()}
                actions = {side: f.result() for side, f in futures.items()}
                read_ms = (time.perf_counter() - read_start) * 1e3

                pkt = wire.TeleopPacket(
                    seq=seq & 0xFFFFFFFF,
                    sent_ts=time.perf_counter(),
                    left=wire.ArmAngles(**actions["left"]) if "left" in actions else None,
                    right=wire.ArmAngles(**actions["right"]) if "right" in actions else None,
                )
                data = wire.encode(pkt)

                if args.echo_only:
                    print(data.decode())
                else:
                    sock.send(data)

                seq += 1
                stats_frames += 1
                stats_max_read_ms = max(stats_max_read_ms, read_ms)

                now = time.perf_counter()
                if now - stats_t0 >= 1.0:
                    hz = stats_frames / (now - stats_t0)
                    logger.info(
                        "%.1f Hz | max read %.1f ms | seq %d | %d B/pkt",
                        hz,
                        stats_max_read_ms,
                        seq,
                        len(data),
                    )
                    stats_t0 = now
                    stats_frames = 0
                    stats_max_read_ms = 0.0

                slack = next_t - time.perf_counter()
                if slack > 0:
                    time.sleep(slack)
                next_t += period
                if time.perf_counter() - next_t > 5 * period:
                    next_t = time.perf_counter() + period
    except KeyboardInterrupt:
        logger.info("Interrupted, shutting down.")
    finally:
        sock.close()
        for side, arm in arms.items():
            try:
                arm.disconnect()
            except Exception as exc:
                logger.warning("Error disconnecting %s arm: %s", side, exc)


if __name__ == "__main__":
    main()
