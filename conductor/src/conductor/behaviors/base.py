"""RobotBehaviour — base class for every leaf.

Contract (the non-blocking tick discipline):

  * ``initialise()`` fires the side-effecting command once (a fast POST or a
    subprocess spawn).
  * ``update()`` returns ``RUNNING`` until the effect is confirmed via the
    polled snapshot, then ``SUCCESS`` (or ``FAILURE`` on timeout).
  * ``terminate(new_status)`` does idempotent cleanup. Crucially, resource
    teardown keys on ``new_status == INVALID`` (preemption), *not* ``SUCCESS`` —
    a leaf that merely completes and lets its Sequence move on must NOT release
    the resource. py_trees propagates ``stop(INVALID)`` to every descendant when
    a subtree is preempted, so keying on INVALID yields correct mode teardown.

Leaves read robot state through ``self.snapshot()`` (the client's cached view).
The ``dsl_params`` dict each leaf records lets ``tree_to_dsl`` round-trip it.
"""

from __future__ import annotations

import time
from typing import Any

import py_trees
from py_trees.common import Status

from conductor.client import RobotSnapshot
from conductor.context import Context


class RobotBehaviour(py_trees.behaviour.Behaviour):
    #: default confirmation timeout (seconds) for state-confirming leaves.
    default_timeout: float = 4.0

    def __init__(self, ctx: Context, name: str, dsl_params: dict[str, Any] | None = None):
        super().__init__(name)
        self.ctx = ctx
        self.dsl_params: dict[str, Any] = dsl_params or {}
        self._deadline: float = 0.0

    # -- helpers ---------------------------------------------------------
    def snapshot(self) -> RobotSnapshot:
        return self.ctx.client.snapshot()

    def arm_deadline(self, timeout: float | None = None) -> None:
        self._deadline = time.monotonic() + (self.default_timeout if timeout is None else timeout)

    def expired(self) -> bool:
        return time.monotonic() > self._deadline


class ConfirmBehaviour(RobotBehaviour):
    """Fire a command in ``initialise``; hold RUNNING until ``confirmed()``.

    Subclasses implement ``act()`` (the command) and ``confirmed(snap)`` (the
    success predicate over the latest snapshot). Times out to FAILURE.
    """

    def act(self) -> None:  # pragma: no cover - abstract
        raise NotImplementedError

    def confirmed(self, snap: RobotSnapshot) -> bool:  # pragma: no cover - abstract
        raise NotImplementedError

    def initialise(self) -> None:
        self.arm_deadline()
        try:
            self.act()
        except Exception as exc:
            self.feedback_message = f"act failed: {exc}"
            self.logger.warning(self.feedback_message)

    def update(self) -> Status:
        if self.confirmed(self.snapshot()):
            return Status.SUCCESS
        if self.expired():
            self.feedback_message = "timed out waiting for confirmation"
            return Status.FAILURE
        return Status.RUNNING
