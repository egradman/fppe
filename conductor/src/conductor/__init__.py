"""Behavior-tree conductor for the fppe robot.

A py_trees "conductor" that runs on skynet, acts as a client of the fppe Pi's
viser_control HTTP API (:8091), and owns the lifecycle of the skynet-side UDP
sender processes. Robot "modes" (idle, teleop, nav, clean-room) are subtrees;
switching a mode swaps a subtree; the GPIO e-stop is a top-level preemption
guard that gives clean teardown for free.
"""

from conductor.context import Context
from conductor.client import RobotClient, FakeRobotClient, RobotSnapshot
from conductor.senders import SenderManager, FakeSenderManager
from conductor.engine import Engine

__all__ = [
    "Context",
    "RobotClient",
    "FakeRobotClient",
    "RobotSnapshot",
    "SenderManager",
    "FakeSenderManager",
    "Engine",
]
