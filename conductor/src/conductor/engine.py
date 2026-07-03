"""Engine — the ticking runtime and the human/LLM control surface.

Wraps a ``py_trees.trees.BehaviourTree`` ticking at a fixed rate. The tree is::

    root = Selector[ safety , MissionSlot ]

where ``MissionSlot`` holds exactly one child: the current mode subtree. The
public methods are the same surface a future LLM loop will drive:

  * ``load_mission(name, dsl)`` — hard switch: replace the mission now.
  * ``append(name, dsl)``       — queue a mission to run after the current one ends.
  * ``interrupt(name, dsl)``    — run ``dsl`` now, resume the current mission when it ends.
  * ``abort()``                 — drop everything and return to idle.

Swaps are staged and applied between ticks (pre-tick handler) so we never mutate
the tree mid-traversal. Replacing the mission explicitly stops the old subtree
with INVALID, which propagates teardown (senders killed, base source off) down
every leaf regardless of its status.
"""

from __future__ import annotations

import collections
import logging
import threading
import time
from typing import Any, Callable, Optional

import py_trees
from py_trees.common import Status

from conductor.context import Context
from conductor.dsl import build_tree, tree_to_dsl
from conductor.safety import build_root

log = logging.getLogger(__name__)

Mission = tuple[str, dict]  # (name, dsl)

# The safe idle used for abort() and the auto-fallback. Must actively disengage
# the base source and drop arms to local — NOT a bare hold, or aborting from a
# base-driving mode would leave the source engaged. Mirrors modes/idle.yaml but
# lives here so the fallback is guaranteed self-contained.
_SAFE_IDLE: Mission = ("idle", {
    "type": "sequence", "name": "idle", "memory": True,
    "children": [
        {"type": "set_base_source", "params": {"value": "off"}},
        {"type": "set_teleop_mode", "params": {"mode": "local"}},
    ],
})


class Engine:
    def __init__(self, ctx: Context, tick_hz: float = 10.0, idle: Optional[Mission] = None):
        self.ctx = ctx
        self.tick_period = 1.0 / tick_hz
        self._idle: Mission = idle or _SAFE_IDLE

        self.slot = py_trees.composites.Selector("mission", memory=False, children=[])
        self.root = build_root(ctx, self.slot)
        self.tree = py_trees.trees.BehaviourTree(self.root)

        self._bb = py_trees.blackboard.Client(name="engine")
        self._bb.register_key("robot", access=py_trees.common.Access.WRITE)
        self._bb.register_key("mission", access=py_trees.common.Access.WRITE)

        self._lock = threading.Lock()
        self._pending: Optional[Mission] = None          # staged hard-swap
        self._queue: collections.deque[Mission] = collections.deque()
        self._resume_stack: list[Mission] = []           # for interrupt()
        self._current: Mission = self._idle
        self._stop_evt = threading.Event()

        self.tree.add_pre_tick_handler(self._pre_tick)
        self.tree.add_post_tick_handler(self._post_tick)

    # -- lifecycle -------------------------------------------------------
    def setup(self) -> None:
        self.ctx.client.start()
        self.ctx.base.start()
        self.ctx.perception.start()
        self._pending = self._current  # install idle on first tick
        self.tree.setup()

    def shutdown(self) -> None:
        self._stop_evt.set()
        # Tear the live mission down cleanly (INVALID -> senders stop).
        for child in list(self.slot.children):
            child.stop(Status.INVALID)
        self.slot.remove_all_children()
        try:
            self.ctx.senders.stop_all()
        except Exception:
            pass
        self.ctx.base.stop()
        self.ctx.base.close()
        self.ctx.perception.stop()
        self.ctx.client.stop()

    # -- control surface (thread-safe) -----------------------------------
    def load_mission(self, name: str, dsl: dict) -> None:
        with self._lock:
            self._queue.clear()
            self._resume_stack.clear()
            self._pending = (name, dsl)

    def append(self, name: str, dsl: dict) -> None:
        with self._lock:
            self._queue.append((name, dsl))

    def interrupt(self, name: str, dsl: dict) -> None:
        with self._lock:
            self._resume_stack.append(self._current)
            self._pending = (name, dsl)

    def abort(self) -> None:
        with self._lock:
            self._queue.clear()
            self._resume_stack.clear()
            self._pending = self._idle

    def current_mission(self) -> str:
        return self._current[0]

    def current_dsl(self) -> dict:
        if self.slot.children:
            return tree_to_dsl(self.slot.children[0])
        return {"type": "hold", "name": "empty"}

    # -- tick handlers ---------------------------------------------------
    def _pre_tick(self, tree: py_trees.trees.BehaviourTree) -> None:
        self._apply_pending()
        snap = self.ctx.client.snapshot()
        self._bb.robot = snap
        self._bb.mission = self._current[0]

    def _apply_pending(self) -> None:
        with self._lock:
            pending = self._pending
            self._pending = None
        if pending is None:
            return
        name, dsl = pending
        try:
            new_child = build_tree(dsl, self.ctx)
        except Exception as exc:
            log.error("failed to build mission %r: %s", name, exc)
            return
        # Explicit INVALID stop guarantees teardown even if the old subtree had
        # already completed with SUCCESS (remove_all_children only stops RUNNING).
        for child in list(self.slot.children):
            child.stop(Status.INVALID)
        self.slot.remove_all_children()
        # Zero any autonomous base command at the mission boundary. A base-driving
        # mode (chase/nav) re-sets it on its first tick; a non-driver (idle) leaves
        # it stopped. Leaves therefore never zero the base themselves.
        self.ctx.base.stop()
        self.slot.add_child(new_child)
        self._current = (name, dsl)
        log.info("mission -> %s", name)

    def _post_tick(self, tree: py_trees.trees.BehaviourTree) -> None:
        # Auto-advance when the current mission ends (SUCCESS or FAILURE).
        if not self.slot.children:
            return
        status = self.slot.children[0].status
        if status not in (Status.SUCCESS, Status.FAILURE):
            return
        with self._lock:
            if self._pending is not None:
                return  # a swap is already staged
            if self._queue:
                self._pending = self._queue.popleft()
            elif self._resume_stack:
                self._pending = self._resume_stack.pop()
            elif self._current[0] != self._idle[0]:
                self._pending = self._idle

    # -- run loop --------------------------------------------------------
    def tick_once(self) -> None:
        self.tree.tick()

    def spin(self, on_tick: Optional[Callable[["Engine"], None]] = None) -> None:
        """Block, ticking at the configured rate until ``shutdown``/KeyboardInterrupt."""
        while not self._stop_evt.is_set():
            t0 = time.monotonic()
            self.tick_once()
            if on_tick is not None:
                on_tick(self)
            self._stop_evt.wait(max(0.0, self.tick_period - (time.monotonic() - t0)))
