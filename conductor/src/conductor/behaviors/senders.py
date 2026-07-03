"""Sender-lifecycle leaves — start/stop the skynet UDP source processes.

``StartSender`` is where the teardown-on-preemption discipline matters most: it
spawns a sender in ``initialise`` but only kills it in ``terminate`` when the new
status is ``INVALID`` (i.e. the enclosing mode was preempted or swapped). When
the leaf merely returns SUCCESS and its Sequence advances, the process keeps
running — which is what "start it and hold it for the duration of the mode"
means.
"""

from __future__ import annotations

import py_trees
from py_trees.common import Status

from conductor.behaviors.base import RobotBehaviour
from conductor.context import Context
from conductor.registry import register


@register(
    "start_sender",
    "Launch a skynet-side UDP sender ('leader', 'pedal', or future 'nav') and hold it "
    "for the duration of the enclosing mode; it is killed when the mode is torn down.",
    {"sender": {"type": "string", "enum": ["leader", "pedal", "nav"],
                "description": "which sender process to launch"}},
)
class StartSender(RobotBehaviour):
    def __init__(self, ctx: Context, sender: str, name: str | None = None):
        super().__init__(ctx, name or f"start_{sender}", {"sender": sender})
        self.sender = sender
        self._started = False

    def initialise(self) -> None:
        try:
            self.ctx.senders.start(self.sender)
            self._started = True
        except Exception as exc:
            self._started = False
            self.feedback_message = f"start failed: {exc}"
            self.logger.warning(self.feedback_message)

    def update(self) -> Status:
        if not self._started:
            return Status.FAILURE
        return Status.SUCCESS if self.ctx.senders.is_running(self.sender) else Status.RUNNING

    def terminate(self, new_status: Status) -> None:
        # Tear down ONLY on preemption/swap, never when the Sequence advances.
        if new_status == Status.INVALID and self._started:
            self.ctx.senders.stop(self.sender)
            self._started = False


@register(
    "stop_sender",
    "Kill a skynet-side UDP sender if running. Idempotent.",
    {"sender": {"type": "string", "enum": ["leader", "pedal", "nav"], "description": "which sender"}},
)
class StopSender(RobotBehaviour):
    def __init__(self, ctx: Context, sender: str, name: str | None = None):
        super().__init__(ctx, name or f"stop_{sender}", {"sender": sender})
        self.sender = sender

    def update(self) -> Status:
        self.ctx.senders.stop(self.sender)
        return Status.SUCCESS
