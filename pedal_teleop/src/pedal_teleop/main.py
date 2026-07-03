"""Stream foot-pedal body commands from skynet to fppe over UDP."""

from __future__ import annotations

import argparse
import curses
import logging
import socket
import sys
import time

import numpy as np

from . import wire
from .game import GameBroadcaster, start_game_server
from .mapping import PedalCal, compute_base_cmd, project_tilt
from .ports import (
    AXIS_SIGN_FORWARD_LEFT,
    AXIS_SIGN_FORWARD_RIGHT,
    AXIS_SIGN_RIGHT_LEFT,
    AXIS_SIGN_RIGHT_RIGHT,
    DEADZONE,
    FULL_SCALE_TILT,
    LEFT_PORT,
    LIFT_DEADZONE,
    REST_MAG_MAX,
    REST_MAG_MIN,
    RIGHT_PORT,
)
from .reader import PedalReader
from .viz import CursesVisualizer, VizFrame

logger = logging.getLogger("pedal_teleop")


STALE_S = 0.1   # treat a pedal sample older than this as missing


def _capture_rest(reader: PedalReader, duration_s: float) -> np.ndarray:
    g = reader.collect_rest(duration_s=duration_s)
    mag = float(np.linalg.norm(g))
    if not (REST_MAG_MIN <= mag <= REST_MAG_MAX):
        raise SystemExit(
            f"Pedal '{reader.name}' rest magnitude {mag:.3f}g is outside "
            f"[{REST_MAG_MIN}, {REST_MAG_MAX}]g. Is something resting on it? "
            f"Remove load and restart."
        )
    logger.info("%s pedal rest gravity: [%.3f %.3f %.3f] g (|g|=%.3f)",
                reader.name, g[0], g[1], g[2], mag)
    return g


def _run_loop(
    args,
    left: PedalReader,
    right: PedalReader,
    cal_L: PedalCal,
    cal_R: PedalCal,
    sock: socket.socket | None,
    viz: CursesVisualizer | None,
    broadcaster: GameBroadcaster | None,
) -> None:
    period = 1.0 / args.rate
    seq = 0
    next_t = time.perf_counter() + period

    stats_t0 = time.perf_counter()
    stats_frames = 0
    last_hz = 0.0

    while True:
        now_t = time.monotonic()
        aL, tsL = left.latest()
        aR, tsR = right.latest()

        stale = (
            aL is None or aR is None
            or (now_t - tsL) > STALE_S
            or (now_t - tsR) > STALE_S
        )

        if stale or aL is None or aR is None:
            x_vel = y_vel = theta_vel = 0.0
            lift: wire.LiftCmd = "stop"
            tilt_L = (0.0, 0.0)
            tilt_R = (0.0, 0.0)
        else:
            tilt_L = project_tilt(aL, cal_L)
            tilt_R = project_tilt(aR, cal_R)
            x_vel, y_vel, theta_vel, lift = compute_base_cmd(
                aL, aR, cal_L, cal_R,
                deadzone=DEADZONE,
                full_scale_tilt=FULL_SCALE_TILT,
                lift_deadzone=LIFT_DEADZONE,
            )

        pkt = wire.PedalPacket(
            seq=seq & 0xFFFFFFFF,
            sent_ts=time.perf_counter(),
            x_vel=x_vel,
            y_vel=y_vel,
            theta_vel=theta_vel,
            lift=lift,
        )
        data = wire.encode(pkt)

        if args.echo_only:
            sys.stdout.write(data.decode() + "\n")
            sys.stdout.flush()
        elif sock is not None:
            sock.send(data)

        if broadcaster is not None:
            broadcaster.publish(x_vel, y_vel, theta_vel, lift, stale=stale)

        seq += 1
        stats_frames += 1

        now = time.perf_counter()
        if now - stats_t0 >= 1.0:
            last_hz = stats_frames / (now - stats_t0)
            if viz is None:
                logger.info(
                    "%.1f Hz | seq %d | %d B/pkt | last x=%.2f y=%.2f th=%.1f lift=%s",
                    last_hz, seq, len(data), x_vel, y_vel, theta_vel, lift,
                )
            stats_t0 = now
            stats_frames = 0

        if viz is not None:
            viz.draw(VizFrame(
                raw_L=aL,
                raw_R=aR,
                tilt_L=tilt_L,
                tilt_R=tilt_R,
                x_vel=x_vel,
                y_vel=y_vel,
                theta_vel=theta_vel,
                lift=lift,
                seq=seq,
                hz=last_hz,
                stale=stale,
            ))

        slack = next_t - time.perf_counter()
        if slack > 0:
            time.sleep(slack)
        next_t += period
        if time.perf_counter() - next_t > 5 * period:
            next_t = time.perf_counter() + period


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fppe-host", default="fppe", help="Hostname/IP of the fppe Pi.")
    parser.add_argument("--port", type=int, default=wire.DEFAULT_PORT)
    parser.add_argument("--rate", type=float, default=100.0, help="Send rate in Hz.")
    parser.add_argument(
        "--rest-duration",
        type=float,
        default=1.0,
        help="Seconds to average for the startup rest-gravity capture.",
    )
    parser.add_argument(
        "--echo-only",
        action="store_true",
        help="Print packets locally instead of sending; useful for debugging.",
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Open an ncurses visualizer showing raw axes and output velocities.",
    )
    parser.add_argument(
        "--game-port",
        type=int,
        default=0,
        help="If > 0, serve the browser test-track game on this TCP port, "
             "bound to 0.0.0.0 so other machines on the LAN can connect. "
             "0 disables.",
    )
    args = parser.parse_args()

    if args.visualize and args.echo_only:
        raise SystemExit("--visualize and --echo-only both take over stdout; pick one.")

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    logger.info("Opening pedals: left=%s right=%s", LEFT_PORT, RIGHT_PORT)
    left = PedalReader(port=LEFT_PORT, name="left")
    right = PedalReader(port=RIGHT_PORT, name="right")
    left.start()
    right.start()

    # Give the firmware a moment to stream a few samples through CDC.
    time.sleep(0.2)

    logger.info("Capturing rest pose (%.1fs). Keep feet OFF the pedals.", args.rest_duration)
    g_rest_L = _capture_rest(left, args.rest_duration)
    g_rest_R = _capture_rest(right, args.rest_duration)

    cal_L = PedalCal(
        g_rest=g_rest_L,
        axis_sign_forward=AXIS_SIGN_FORWARD_LEFT,
        axis_sign_right=AXIS_SIGN_RIGHT_LEFT,
    )
    cal_R = PedalCal(
        g_rest=g_rest_R,
        axis_sign_forward=AXIS_SIGN_FORWARD_RIGHT,
        axis_sign_right=AXIS_SIGN_RIGHT_RIGHT,
    )

    sock: socket.socket | None = None
    if not args.echo_only:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((args.fppe_host, args.port))
        logger.info("Streaming -> %s:%d at %.1f Hz", args.fppe_host, args.port, args.rate)
    else:
        logger.info("Echo-only mode: printing packets to stdout, not sending.")

    broadcaster: GameBroadcaster | None = None
    if args.game_port > 0:
        broadcaster = GameBroadcaster()
        start_game_server(args.game_port, broadcaster)
        logger.info("Game server: http://0.0.0.0:%d/ (any LAN host)", args.game_port)

    try:
        if args.visualize:
            # Silence the logger so it doesn't punch through curses; viz shows Hz itself.
            logging.disable(logging.CRITICAL)
            curses.wrapper(lambda scr: _run_loop(args, left, right, cal_L, cal_R, sock, CursesVisualizer(scr), broadcaster))
        else:
            _run_loop(args, left, right, cal_L, cal_R, sock, None, broadcaster)
    except KeyboardInterrupt:
        pass
    finally:
        logging.disable(logging.NOTSET)
        if sock is not None:
            sock.close()
        left.stop()
        right.stop()
        logger.info("Shutdown complete.")


if __name__ == "__main__":
    main()
