"""BaseCmdChannel — the conductor's autonomous base-velocity output.

When the tree itself drives the base (chase, nav), it needs a command channel
analogous to the leader/pedal senders — but the velocity is *computed inside the
tick loop from live perception*, so the conductor streams it directly rather than
spawning a dumb sender process.

The channel holds a single ``(x_vel, y_vel, theta_vel)`` setpoint and a ~100 Hz
background thread that repeats it to the Pi over UDP as a ``PedalPacket``-shaped
datagram (schema mirrors ``pedal_teleop/wire.py``). Repeating at 100 Hz keeps the
Pi-side 300 ms watchdog fed even though the tree only recomputes at ~10 Hz;
behaviors just call ``set(...)``.

Phase-3 seam: this targets a future ``base_input_source == "auto"`` + a
``start_auto_udp_listener`` in ``viser_control.py`` (mirroring the pedal
listener). Until that lands, ``SetBaseSource("auto")`` is rejected by the Pi and
the chase mission fails safe to idle — nothing moves.

``FakeBaseCmdChannel`` records the setpoint with no socket, for tests.
"""

from __future__ import annotations

import json
import logging
import socket
import threading
import time

log = logging.getLogger(__name__)

DEFAULT_AUTO_PORT = 9997  # leader=9999, pedal=9998, (future) auto=9997

Setpoint = tuple[float, float, float]  # (x_vel m/s +fwd, y_vel m/s +left, theta_vel deg/s +CCW)


class BaseCmdChannel:
    def __init__(self, host: str = "fppe", port: int = DEFAULT_AUTO_PORT, rate_hz: float = 100.0):
        self._addr = (host, port)
        self._period = 1.0 / rate_hz
        self._sp: Setpoint = (0.0, 0.0, 0.0)
        self._seq = 0
        self._lock = threading.Lock()
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._loop, name="base-cmd", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def set(self, x_vel: float, y_vel: float, theta_vel: float) -> None:
        with self._lock:
            self._sp = (float(x_vel), float(y_vel), float(theta_vel))

    def stop(self) -> None:
        """Zero the setpoint. The channel keeps streaming zeros (safe)."""
        self.set(0.0, 0.0, 0.0)

    def setpoint(self) -> Setpoint:
        with self._lock:
            return self._sp

    def _loop(self) -> None:
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            with self._lock:
                x, y, th = self._sp
                self._seq = (self._seq + 1) & 0xFFFFFFFF
                seq = self._seq
            pkt = {
                "version": 1,
                "seq": seq,
                "sent_ts": time.time(),
                "x_vel": x,
                "y_vel": y,
                "theta_vel": th,
                "lift": "stop",
            }
            try:
                assert self._sock is not None
                self._sock.sendto(json.dumps(pkt).encode("utf-8"), self._addr)
            except Exception as exc:  # keep the loop alive on transient socket errors
                log.debug("base cmd send failed: %s", exc)
            self._stop_evt.wait(max(0.0, self._period - (time.monotonic() - t0)))


class FakeBaseCmdChannel:
    """In-memory stand-in; records the latest setpoint. Tests only."""

    def __init__(self) -> None:
        self._sp: Setpoint = (0.0, 0.0, 0.0)
        self.started = False

    def start(self) -> None:
        self.started = True

    def close(self) -> None:
        self.started = False

    def set(self, x_vel: float, y_vel: float, theta_vel: float) -> None:
        self._sp = (float(x_vel), float(y_vel), float(theta_vel))

    def stop(self) -> None:
        self._sp = (0.0, 0.0, 0.0)

    def setpoint(self) -> Setpoint:
        return self._sp
