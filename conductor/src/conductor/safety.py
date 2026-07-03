"""Safety guard — the top-level preemption layer and clean mode-handoff mechanism.

Root structure::

    Selector(memory=False)            # re-evaluated every tick (reactive)
      ├── Sequence "safety" (memory=False)
      │     ├── EstopEngaged           SUCCESS while e-stop is engaged/fault
      │     └── SafeStop               idempotent: base off + kill all senders
      └── <MissionSlot>                the current mode subtree

While the e-stop is engaged the safety branch wins; a memory=False Selector then
invalidates the lower-priority MissionSlot, and py_trees propagates
``stop(INVALID)`` through the whole mission subtree — so ``StartSender`` leaves
tear their processes down automatically. When the e-stop clears, the safety
branch fails and the Selector falls back to the MissionSlot, which re-initialises
and re-establishes the mode. This same invalidation path is what gives every
mode switch its clean teardown.
"""

from __future__ import annotations

import py_trees
from py_trees.common import Status

from conductor.behaviors.base import RobotBehaviour
from conductor.context import Context
from conductor.registry import register


@register(
    "estop_engaged",
    "SUCCESS while the GPIO e-stop is engaged (used inside the safety guard).",
    {},
)
class EstopEngaged(RobotBehaviour):
    def __init__(self, ctx: Context, name: str | None = None):
        super().__init__(ctx, name or "estop_engaged", {})

    def update(self) -> Status:
        return Status.SUCCESS if self.snapshot().estop_engaged else Status.FAILURE


@register(
    "safe_stop",
    "Idempotently bring the base to a safe state: base_input_source=off and all "
    "senders killed. Only touches the robot when something is not already safe.",
    {},
)
class SafeStop(RobotBehaviour):
    def __init__(self, ctx: Context, name: str | None = None):
        super().__init__(ctx, name or "safe_stop", {})

    def update(self) -> Status:
        # Zero any autonomous base command immediately (cheap, idempotent).
        self.ctx.base.stop()
        # Guard on current state so we don't spam POSTs every tick.
        if self.snapshot().base_input_source != "off":
            try:
                self.ctx.client.set_base_source("off")
            except Exception as exc:
                self.logger.warning("safe_stop set_base_source failed: %s", exc)
        if self.ctx.senders.running():
            self.ctx.senders.stop_all()
        return Status.SUCCESS


def build_safety_branch(ctx: Context) -> py_trees.composites.Sequence:
    return py_trees.composites.Sequence(
        "safety",
        memory=False,
        children=[EstopEngaged(ctx=ctx), SafeStop(ctx=ctx)],
    )


def build_root(ctx: Context, mission_slot: py_trees.behaviour.Behaviour) -> py_trees.composites.Selector:
    return py_trees.composites.Selector(
        "root",
        memory=False,
        children=[build_safety_branch(ctx), mission_slot],
    )
