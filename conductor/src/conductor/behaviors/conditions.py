"""Condition + flow leaves — read-only guards and simple timing/hold nodes."""

from __future__ import annotations

import time

import py_trees
from py_trees.common import Status

from conductor.behaviors.base import RobotBehaviour
from conductor.context import Context
from conductor.registry import register


@register(
    "estop_clear",
    "SUCCESS when the e-stop is released, FAILURE while engaged.",
    {},
)
class EstopClear(RobotBehaviour):
    def __init__(self, ctx: Context, name: str | None = None):
        super().__init__(ctx, name or "estop_clear", {})

    def update(self) -> Status:
        return Status.FAILURE if self.snapshot().estop_engaged else Status.SUCCESS


@register(
    "source_is",
    "SUCCESS when base_input_source equals the given value.",
    {"value": {"type": "string", "enum": ["off", "gamepad", "pedals"], "description": "expected source"}},
)
class SourceIs(RobotBehaviour):
    def __init__(self, ctx: Context, value: str, name: str | None = None):
        super().__init__(ctx, name or f"source_is={value}", {"value": value})
        self.value = value

    def update(self) -> Status:
        return Status.SUCCESS if self.snapshot().base_input_source == self.value else Status.FAILURE


@register(
    "arm_locked",
    "SUCCESS when the given arm(s) are locked (torque-enabled).",
    {"side": {"type": "string", "enum": ["left", "right", "both"], "description": "which arm(s)"}},
)
class ArmLocked(RobotBehaviour):
    def __init__(self, ctx: Context, side: str = "both", name: str | None = None):
        super().__init__(ctx, name or f"locked_{side}", {"side": side})
        self.side = side

    def update(self) -> Status:
        snap = self.snapshot()
        sides = ["left", "right"] if self.side == "both" else [self.side]
        return Status.SUCCESS if all(snap.arm_locked.get(s) for s in sides) else Status.FAILURE


@register(
    "sleep",
    "Return RUNNING for the given duration, then SUCCESS. A dwell/settle timer.",
    {"seconds": {"type": "number", "description": "how long to wait"}},
)
class Sleep(RobotBehaviour):
    def __init__(self, ctx: Context, seconds: float, name: str | None = None):
        super().__init__(ctx, name or f"sleep_{seconds}s", {"seconds": seconds})
        self.seconds = float(seconds)
        self._until = 0.0

    def initialise(self) -> None:
        self._until = time.monotonic() + self.seconds

    def update(self) -> Status:
        return Status.SUCCESS if time.monotonic() >= self._until else Status.RUNNING


@register(
    "wait_settled",
    "RUNNING until the arm motors stop moving (each joint stable within 'tol' ticks "
    "over a 'window'-second lookback), then SUCCESS. Succeeds anyway after 'timeout' "
    "seconds (logs a warning). Target-agnostic: waits on motion, not a goal pose.",
    {"tol": {"type": "integer", "default": 15, "description": "max tick spread to call a joint still"},
     "window": {"type": "number", "default": 0.4, "description": "lookback seconds that must be quiet"},
     "timeout": {"type": "number", "default": 6.0, "description": "give up waiting after this long"}},
)
class WaitSettled(RobotBehaviour):
    def __init__(self, ctx: Context, tol: int = 15, window: float = 0.4, timeout: float = 6.0,
                 name: str | None = None):
        super().__init__(ctx, name or "wait_settled",
                         {"tol": tol, "window": window, "timeout": timeout})
        self.tol = int(tol)
        self.window = float(window)
        self.timeout = float(timeout)
        self._hist: list[tuple[float, dict[str, int]]] = []
        self._t0 = 0.0

    def initialise(self) -> None:
        self._hist = []
        self._t0 = time.monotonic()

    def update(self) -> Status:
        now = time.monotonic()
        pos = self.snapshot().positions
        keys = [k for k in pos if k.startswith("arm_")]
        self._hist.append((now, {k: pos[k] for k in keys}))
        self._hist = [(t, p) for (t, p) in self._hist if now - t <= self.window]

        if now - self._t0 < self.window:
            return Status.RUNNING  # need a full window of history before judging

        spread = 0
        for k in keys:
            vals = [p[k] for (_, p) in self._hist if k in p]
            if vals:
                spread = max(spread, max(vals) - min(vals))
        if spread <= self.tol:
            self.feedback_message = f"settled (spread={spread})"
            return Status.SUCCESS
        if now - self._t0 > self.timeout:
            self.feedback_message = f"did not settle (spread={spread}) in {self.timeout}s; proceeding"
            self.logger.warning(self.feedback_message)
            return Status.SUCCESS
        self.feedback_message = f"moving (spread={spread})"
        return Status.RUNNING


@register(
    "hold",
    "Return RUNNING forever. Used as a mode's tail so the mode persists until "
    "it is preempted or swapped (e.g. teleop stays active).",
    {},
)
class Hold(RobotBehaviour):
    def __init__(self, ctx: Context, name: str | None = None):
        super().__init__(ctx, name or "hold", {})

    def update(self) -> Status:
        return Status.RUNNING


@register(
    "navigate_to",
    "Autonomously drive the base to a named location. STUB: the nav base source "
    "and UDP sender are not wired yet (Phase-3), so this reports FAILURE for now.",
    {"location": {"type": "string", "description": "named waypoint, e.g. 'kitchen'"}},
)
class NavigateTo(RobotBehaviour):
    """Phase-3 seam. When nav lands this becomes: start_sender(nav) +
    set_base_source('nav') + hold-until AtLocation(location)."""

    def __init__(self, ctx: Context, location: str, name: str | None = None):
        super().__init__(ctx, name or f"navigate_to={location}", {"location": location})
        self.location = location

    def update(self) -> Status:
        self.feedback_message = (
            f"nav to {self.location!r} not available: nav base_input_source + sender "
            "not yet wired (see viser_control BaseInputSource._VALUES and nav/)."
        )
        self.logger.warning(self.feedback_message)
        return Status.FAILURE
