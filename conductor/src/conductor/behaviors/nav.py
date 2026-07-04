"""Visual-navigation leaves — drive the base to a goal photo via nav-serve.

``nav_to_goal`` is the conductor half of the Phase-2 nav loop: it hands the goal
photo to the nav service (``ctx.nav``), then each tick reads the service's latest
body-velocity command and writes it to ``ctx.base`` — the single autonomous-base
writer (BaseCmdChannel -> the Pi's 'auto' source on :9997). It returns SUCCESS
when the service reports the goal reached, so the enclosing mode completes and the
engine falls back to idle (base off).

The heavy ViNT inference runs in nav-serve, not here, so the tick stays light —
same split as chase (perception service) uses.

Named-location nav (``navigate_to("kitchen")``) is Phase 3: it needs the
topological map to turn a name into a goal frame. This drives to one photo.
"""

from __future__ import annotations

from pathlib import Path

from py_trees.common import Status

from conductor.behaviors.base import RobotBehaviour
from conductor.context import Context
from conductor.registry import register
from conductor.senders import repo_root


def _read_goal(path: str) -> bytes:
    p = Path(path)
    if not p.is_absolute():
        p = repo_root() / p
    return p.read_bytes()


@register(
    "nav_to_goal",
    "Autonomously drive the base to a goal photo using the nav service. Streams "
    "the service's velocity to ctx.base each tick; SUCCESS when the goal is "
    "reached, FAILURE if the goal can't be loaded or the service is offline.",
    {"goal": {"type": "string",
              "description": "goal image path (repo-root-relative or absolute)"}},
)
class NavToGoal(RobotBehaviour):
    def __init__(self, ctx: Context, goal: str, name: str | None = None):
        super().__init__(ctx, name or f"nav_to_goal={goal}", {"goal": goal})
        self.goal = goal
        self._set = False

    def initialise(self) -> None:
        try:
            self.ctx.nav.set_goal(_read_goal(self.goal))
            self._set = True
        except Exception as exc:
            self._set = False
            self.feedback_message = f"nav goal load failed: {exc}"
            self.logger.warning(self.feedback_message)

    def update(self) -> Status:
        if not self._set:
            return Status.FAILURE
        c = self.ctx.nav.get()
        if not c.online:
            self.feedback_message = "nav service offline"
            self.logger.warning(self.feedback_message)
            self.ctx.base.stop()
            return Status.FAILURE
        if c.reached:
            self.ctx.base.stop()
            self.feedback_message = f"reached ({c.reason})"
            return Status.SUCCESS
        if not c.active:
            self.feedback_message = "waiting for nav command (warming up)"
            return Status.RUNNING
        self.ctx.base.set(c.vx, c.vy, c.theta)
        self.feedback_message = (
            f"dist={c.dist} v=({c.vx:.2f},{c.vy:.2f},{c.theta:.0f})"
        )
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # Release the nav goal on preemption/swap, same discipline as StartSender.
        if new_status == Status.INVALID and self._set:
            self.ctx.nav.clear_goal()
            self._set = False


@register(
    "navigate_to",
    "Autonomously drive the base to a named location on the topological route "
    "(Phase 3). Asks nav-serve to follow the breadcrumb sequence from the current "
    "position to the labeled goal node, streaming its velocity to ctx.base each "
    "tick; SUCCESS when the final breadcrumb is reached, FAILURE if the label is "
    "unknown or the service is offline.",
    {"label": {"type": "string",
               "description": "goal location label, e.g. 'inward' / 'kitchen' (must exist in the loaded route)"}},
)
class NavigateTo(RobotBehaviour):
    def __init__(self, ctx: Context, label: str, name: str | None = None):
        super().__init__(ctx, name or f"navigate_to={label}", {"label": label})
        self.label = label
        self._started = False

    def initialise(self) -> None:
        # Blocks while nav-serve localizes the start node (several ViNT inferences).
        res = self.ctx.nav.set_route(label=self.label)
        self._started = bool(res.get("ok"))
        if not self._started:
            self.feedback_message = f"route start failed: {res.get('error')}"
            self.logger.warning(self.feedback_message)
        else:
            self.feedback_message = f"route {res.get('start')}->{res.get('goal')}"

    def update(self) -> Status:
        if not self._started:
            return Status.FAILURE
        c = self.ctx.nav.get()
        if not c.online:
            self.feedback_message = "nav service offline"
            self.logger.warning(self.feedback_message)
            self.ctx.base.stop()
            return Status.FAILURE
        if c.reached:
            self.ctx.base.stop()
            self.feedback_message = f"arrived at {self.label} ({c.reason})"
            return Status.SUCCESS
        if not c.active:
            self.feedback_message = "localizing / warming up"
            return Status.RUNNING
        self.ctx.base.set(c.vx, c.vy, c.theta)
        self.feedback_message = (
            f"leg {c.leg} node={c.node} dist={c.dist} "
            f"v=({c.vx:.2f},{c.vy:.2f},{c.theta:.0f})"
        )
        return Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        if new_status == Status.INVALID and self._started:
            self.ctx.nav.clear_route()
            self._started = False
