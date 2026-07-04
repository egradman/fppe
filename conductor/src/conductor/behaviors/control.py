"""Follower-control leaves — thin py_trees wrappers over the :8091 endpoints."""

from __future__ import annotations

import py_trees
from py_trees.common import Status

from conductor.behaviors.base import ConfirmBehaviour, RobotBehaviour
from conductor.client import RobotSnapshot
from conductor.context import Context
from conductor.registry import register


@register(
    "set_base_source",
    "Select which source drives the base (wheels + lift). Confirms via /state.",
    {"value": {"type": "string", "enum": ["off", "gamepad", "pedals", "auto"],
               "description": "base_input_source to switch to ('auto' = conductor-driven)"}},
)
class SetBaseSource(ConfirmBehaviour):
    def __init__(self, ctx: Context, value: str, name: str | None = None):
        super().__init__(ctx, name or f"base_source={value}", {"value": value})
        self.value = value

    def act(self) -> None:
        self.ctx.client.set_base_source(self.value)

    def confirmed(self, snap: RobotSnapshot) -> bool:
        return snap.base_input_source == self.value


@register(
    "set_teleop_mode",
    "Gate the leader-arm teleop path: 'remote' accepts UDP arm packets, 'local' drops them.",
    {"mode": {"type": "string", "enum": ["local", "remote"],
              "description": "teleop_mode to switch to"}},
)
class SetTeleopMode(ConfirmBehaviour):
    def __init__(self, ctx: Context, mode: str, name: str | None = None):
        super().__init__(ctx, name or f"teleop_mode={mode}", {"mode": mode})
        self.mode = mode

    def act(self) -> None:
        self.ctx.client.set_teleop_mode(self.mode)

    def confirmed(self, snap: RobotSnapshot) -> bool:
        return snap.teleop_mode == self.mode


@register(
    "lock_arm",
    "Torque-enable ('lock') or torque-disable ('unlock') an arm at its current pose.",
    {"side": {"type": "string", "enum": ["left", "right", "both"], "description": "which arm(s)"},
     "lock": {"type": "boolean", "default": True, "description": "lock (true) or unlock (false)"}},
)
class LockArm(ConfirmBehaviour):
    def __init__(self, ctx: Context, side: str = "both", lock: bool = True, name: str | None = None):
        verb = "lock" if lock else "unlock"
        super().__init__(ctx, name or f"{verb}_{side}", {"side": side, "lock": lock})
        self.side = side
        self.lock = lock

    def _sides(self) -> list[str]:
        return ["left", "right"] if self.side == "both" else [self.side]

    def act(self) -> None:
        self.ctx.client.lock_arm(self.side, self.lock)

    def confirmed(self, snap: RobotSnapshot) -> bool:
        return all(snap.arm_locked.get(s) == self.lock for s in self._sides())


class _FireAndForget(RobotBehaviour):
    """Issue one command in initialise, succeed immediately (no state to confirm)."""

    def _fire(self) -> None:  # pragma: no cover - abstract
        raise NotImplementedError

    def update(self) -> Status:
        try:
            self._fire()
            return Status.SUCCESS
        except Exception as exc:
            self.feedback_message = f"command failed: {exc}"
            self.logger.warning(self.feedback_message)
            return Status.FAILURE


@register(
    "lift",
    "Drive the elevator lift: 'up'/'down' start motion, 'stop' halts it.",
    {"action": {"type": "string", "enum": ["up", "down", "stop"], "description": "lift action"}},
)
class Lift(_FireAndForget):
    def __init__(self, ctx: Context, action: str, name: str | None = None):
        super().__init__(ctx, name or f"lift_{action}", {"action": action})
        self.action = action

    def _fire(self) -> None:
        self.ctx.client.lift(self.action)


@register(
    "go_preset",
    "Send both arms to a named posture: 'home', 'middle' (2048), or 'arms_up'.",
    {"preset": {"type": "string", "enum": ["home", "middle", "arms_up"], "description": "posture"}},
)
class GoPreset(_FireAndForget):
    def __init__(self, ctx: Context, preset: str, name: str | None = None):
        super().__init__(ctx, name or f"preset_{preset}", {"preset": preset})
        self.preset = preset

    def _fire(self) -> None:
        self.ctx.client.go_preset(self.preset)


@register(
    "goal_position",
    "Command raw goal positions (ticks 0-4095) for named motors.",
    {"goals": {"type": "object", "description": "{motor_name: raw_int}"}},
)
class GoalPosition(_FireAndForget):
    def __init__(self, ctx: Context, goals: dict, name: str | None = None):
        super().__init__(ctx, name or "goal_position", {"goals": dict(goals)})
        self.goals = goals

    def _fire(self) -> None:
        self.ctx.client.goal_position(self.goals)


@register(
    "estop",
    "Software e-stop: release all arm torque immediately.",
    {},
)
class Estop(_FireAndForget):
    def __init__(self, ctx: Context, name: str | None = None):
        super().__init__(ctx, name or "estop", {})

    def _fire(self) -> None:
        self.ctx.client.estop()
