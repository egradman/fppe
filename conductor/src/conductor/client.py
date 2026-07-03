"""RobotClient — the single integration seam to the fppe follower hub.

Wraps viser_control.py's HTTP API on ``:8091`` (see its module docstring):

  GET  /state              -> snapshot (positions, arm locks, estop, modes)
  POST /goal_position      {"motor_name": raw_int, ...}
  POST /go_home | /go_middle | /go_arms_up
  POST /lock_arm           {"side": "left"|"right", "lock": bool}
  POST /estop
  POST /lift               {"action": "up"|"down"|"stop"}
  POST /teleop_mode        {"mode": "local"|"remote"}
  POST /base_input_source  {"value": "off"|"gamepad"|"pedals"}

A background thread polls ``GET /state`` so behavior ticks read a cached
snapshot instead of blocking on HTTP. POSTs are fast (the Pi enqueues the
command and returns immediately), so issuing them synchronously from a
behaviour's ``initialise()`` is fine.

``FakeRobotClient`` mirrors the same interface with in-memory state and no
network, backing the offline engine tests.
"""

from __future__ import annotations

import copy
import logging
import threading
import time
from dataclasses import dataclass, field, replace
from typing import Protocol, runtime_checkable

import httpx

log = logging.getLogger(__name__)

# Presets exposed by viser_control -> the POST path that triggers each.
_PRESET_PATHS = {
    "home": "/go_home",
    "middle": "/go_middle",
    "arms_up": "/go_arms_up",
}


@dataclass
class RobotSnapshot:
    """A point-in-time view of the follower, mirroring ``GET /state``."""

    positions: dict[str, int] = field(default_factory=dict)
    arm_locked: dict[str, bool] = field(default_factory=dict)
    estop_engaged: bool = False
    teleop_mode: str = "local"
    base_input_source: str = "off"
    #: monotonic timestamp this snapshot was captured; 0.0 == never polled.
    ts: float = 0.0
    #: whether the most recent poll reached the Pi.
    online: bool = False

    def copy(self) -> "RobotSnapshot":
        return replace(self, positions=dict(self.positions), arm_locked=dict(self.arm_locked))


@runtime_checkable
class RobotClientBase(Protocol):
    """The surface behaviors depend on; both real and fake satisfy it."""

    def start(self) -> None: ...
    def stop(self) -> None: ...
    def snapshot(self) -> RobotSnapshot: ...
    def set_base_source(self, value: str) -> None: ...
    def set_teleop_mode(self, mode: str) -> None: ...
    def lock_arm(self, side: str, lock: bool) -> None: ...
    def lift(self, action: str) -> None: ...
    def go_preset(self, preset: str) -> None: ...
    def goal_position(self, goals: dict[str, int]) -> None: ...
    def estop(self) -> None: ...


class RobotClient:
    """HTTP client for a live viser_control instance."""

    def __init__(
        self,
        host: str = "fppe",
        api_port: int = 8091,
        poll_hz: float = 20.0,
        timeout: float = 2.0,
    ) -> None:
        self.base_url = f"http://{host}:{api_port}"
        self._poll_period = 1.0 / poll_hz
        self._http = httpx.Client(base_url=self.base_url, timeout=timeout)
        self._lock = threading.Lock()
        self._snap = RobotSnapshot()
        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    # -- lifecycle -------------------------------------------------------
    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._poll_loop, name="state-poller", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._http.close()

    def _poll_loop(self) -> None:
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            try:
                r = self._http.get("/state")
                r.raise_for_status()
                d = r.json()
                snap = RobotSnapshot(
                    positions={k: int(v) for k, v in d.get("positions", {}).items()},
                    arm_locked=dict(d.get("arm_locked", {})),
                    estop_engaged=bool(d.get("estop_engaged", False)),
                    teleop_mode=str(d.get("teleop_mode", "local")),
                    base_input_source=str(d.get("base_input_source", "off")),
                    ts=time.monotonic(),
                    online=True,
                )
            except Exception as exc:  # network hiccup: keep last snapshot, mark offline
                with self._lock:
                    self._snap = replace(self._snap, online=False)
                log.debug("state poll failed: %s", exc)
            else:
                with self._lock:
                    self._snap = snap
            dt = time.monotonic() - t0
            self._stop_evt.wait(max(0.0, self._poll_period - dt))

    def snapshot(self) -> RobotSnapshot:
        with self._lock:
            return self._snap.copy()

    # -- commands --------------------------------------------------------
    def _post(self, path: str, json: dict | None = None) -> None:
        try:
            r = self._http.post(path, json=json or {})
            r.raise_for_status()
        except Exception as exc:
            log.warning("POST %s failed: %s", path, exc)
            raise

    def set_base_source(self, value: str) -> None:
        self._post("/base_input_source", {"value": value})

    def set_teleop_mode(self, mode: str) -> None:
        self._post("/teleop_mode", {"mode": mode})

    def lock_arm(self, side: str, lock: bool) -> None:
        # The Pi only accepts "left"/"right"; expand "both" to two POSTs.
        for s in (["left", "right"] if side == "both" else [side]):
            self._post("/lock_arm", {"side": s, "lock": lock})

    def lift(self, action: str) -> None:
        self._post("/lift", {"action": action})

    def go_preset(self, preset: str) -> None:
        path = _PRESET_PATHS.get(preset)
        if path is None:
            raise ValueError(f"unknown preset {preset!r}; expected one of {list(_PRESET_PATHS)}")
        self._post(path)

    def goal_position(self, goals: dict[str, int]) -> None:
        self._post("/goal_position", {k: int(v) for k, v in goals.items()})

    def estop(self) -> None:
        self._post("/estop")


class FakeRobotClient:
    """In-memory stand-in for the offline engine tests. No network."""

    def __init__(self) -> None:
        self._snap = RobotSnapshot(
            arm_locked={"left": False, "right": False},
            online=True,
            ts=time.monotonic(),
        )
        #: recorded (method, args) calls, for assertions.
        self.calls: list[tuple] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def snapshot(self) -> RobotSnapshot:
        return self._snap.copy()

    def _touch(self) -> None:
        self._snap = replace(self._snap, ts=time.monotonic())

    def set_base_source(self, value: str) -> None:
        self.calls.append(("set_base_source", value))
        self._snap = replace(self._snap, base_input_source=value)
        self._touch()

    def set_teleop_mode(self, mode: str) -> None:
        self.calls.append(("set_teleop_mode", mode))
        self._snap = replace(self._snap, teleop_mode=mode)
        self._touch()

    def lock_arm(self, side: str, lock: bool) -> None:
        self.calls.append(("lock_arm", side, lock))
        locked = dict(self._snap.arm_locked)
        for s in (["left", "right"] if side == "both" else [side]):
            locked[s] = lock
        self._snap = replace(self._snap, arm_locked=locked)
        self._touch()

    def lift(self, action: str) -> None:
        self.calls.append(("lift", action))

    def go_preset(self, preset: str) -> None:
        self.calls.append(("go_preset", preset))

    def goal_position(self, goals: dict[str, int]) -> None:
        self.calls.append(("goal_position", dict(goals)))

    def estop(self) -> None:
        self.calls.append(("estop",))
        self._snap = replace(self._snap, estop_engaged=True)

    # -- test controls (not part of the Protocol) ------------------------
    def set_estop(self, engaged: bool) -> None:
        """Simulate the GPIO e-stop line changing (tests only)."""
        self._snap = replace(self._snap, estop_engaged=engaged)
        self._touch()

    def set_positions(self, positions: dict[str, int]) -> None:
        """Overwrite the reported motor positions (tests only)."""
        self._snap = replace(self._snap, positions=dict(positions))
        self._touch()
