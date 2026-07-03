"""Search-and-chase leaves — the reactive visual-pursuit building blocks.

These are parameterised by ``target`` and back onto the open-vocabulary detector,
so the same leaves chase a dog, a cat, or a person by changing one string — which
is exactly the DSL an LLM would emit from "go find the dog".

Composed as a reactive ``Selector(memory=false)``::

    selector                    # re-decides every tick
      ├── sequence [ object_visible, chase ]     # pursue if seen
      └── search_pattern                          # otherwise look

``search_pattern`` is the always-RUNNING fallback, so *something* always writes a
fresh base setpoint every tick — no stale-velocity hazard. Leaves deliberately do
NOT zero the base in ``terminate``: within the reactive selector the incoming leaf
already owns the setpoint, and zeroing there would clobber it on the switch. The
base is zeroed at the mission boundary instead (``Engine`` zeroes on every swap),
with the safety guard's ``SafeStop`` and the Pi's 300 ms watchdog as backstops.
"""

from __future__ import annotations

from py_trees.common import Status

from conductor.behaviors.base import RobotBehaviour
from conductor.context import Context
from conductor.registry import register


@register(
    "object_visible",
    "SUCCESS when a fresh sighting of <target> is above min_conf (reads perception).",
    {"target": {"type": "string", "description": "open-vocab label, e.g. 'dog'"},
     "min_conf": {"type": "number", "default": 0.4, "description": "confidence gate"}},
)
class ObjectVisible(RobotBehaviour):
    def __init__(self, ctx: Context, target: str, min_conf: float = 0.4, name: str | None = None):
        super().__init__(ctx, name or f"see={target}", {"target": target, "min_conf": min_conf})
        self.target = target
        self.min_conf = float(min_conf)

    def update(self) -> Status:
        s = self.ctx.perception.get(self.target)
        return Status.SUCCESS if (s.seen and s.conf >= self.min_conf) else Status.FAILURE


@register(
    "chase",
    "Proportional visual pursuit of <target>: turn toward its bearing and drive "
    "forward until within 'standoff' (bbox area fraction). RUNNING while pursuing, "
    "FAILURE if the target is lost.",
    {"target": {"type": "string", "description": "open-vocab label to chase"},
     "standoff": {"type": "number", "default": 0.15,
                  "description": "stop closing once bbox area fraction reaches this"}},
)
class Chase(RobotBehaviour):
    def __init__(self, ctx: Context, target: str, standoff: float = 0.15,
                 turn_gain: float = 60.0, fwd_gain: float = 2.0, fwd_max: float = 0.4,
                 aligned_rad: float = 0.20, name: str | None = None):
        super().__init__(ctx, name or f"chase={target}",
                         {"target": target, "standoff": standoff})
        self.target = target
        self.standoff = float(standoff)
        self.turn_gain = float(turn_gain)     # deg/s per rad of bearing
        self.fwd_gain = float(fwd_gain)       # m/s per unit area error
        self.fwd_max = float(fwd_max)
        self.aligned_rad = float(aligned_rad)

    def update(self) -> Status:
        s = self.ctx.perception.get(self.target)
        if not s.seen:
            self.feedback_message = "target lost"
            return Status.FAILURE
        theta = self.turn_gain * s.bearing_rad          # +left bearing -> +CCW turn
        if abs(s.bearing_rad) < self.aligned_rad:       # only drive forward when facing it
            fwd = max(0.0, self.fwd_gain * (self.standoff - s.area_frac))
            fwd = min(fwd, self.fwd_max)
        else:
            fwd = 0.0
        self.ctx.base.set(fwd, 0.0, theta)
        self.feedback_message = f"chasing {self.target}: fwd={fwd:.2f} theta={theta:.0f}"
        return Status.RUNNING


@register(
    "search_pattern",
    "Rotate in place to sweep the camera and bring a target into view. Always "
    "RUNNING (the reactive fallback under a chase selector).",
    {"spin_degps": {"type": "number", "default": 30.0, "description": "search yaw rate, deg/s"}},
)
class SearchPattern(RobotBehaviour):
    def __init__(self, ctx: Context, spin_degps: float = 30.0, name: str | None = None):
        super().__init__(ctx, name or "search", {"spin_degps": spin_degps})
        self.spin = float(spin_degps)

    def update(self) -> Status:
        self.ctx.base.set(0.0, 0.0, self.spin)
        return Status.RUNNING


@register(
    "start_perception",
    "Ask the perception client to track <target> for the duration of the mode; "
    "released when the mode is torn down.",
    {"target": {"type": "string", "description": "open-vocab label to detect"}},
)
class StartPerception(RobotBehaviour):
    def __init__(self, ctx: Context, target: str, name: str | None = None):
        super().__init__(ctx, name or f"perceive={target}", {"target": target})
        self.target = target
        self._requested = False

    def initialise(self) -> None:
        self.ctx.perception.request(self.target)
        self._requested = True

    def update(self) -> Status:
        return Status.SUCCESS

    def terminate(self, new_status: Status) -> None:
        # Same discipline as StartSender: release only on preemption/swap.
        if new_status == Status.INVALID and self._requested:
            self.ctx.perception.release(self.target)
            self._requested = False
