"""wait_settled leaf — settles when positions go quiet, holds while they move."""

import time

from py_trees.common import Status

from conductor.behaviors.conditions import WaitSettled
from conductor.client import FakeRobotClient
from conductor.context import Context
from conductor.senders import FakeSenderManager


def _ctx(client):
    return Context(client=client, senders=FakeSenderManager(fail_unavailable=False))


def _tick(node):
    # Behaviour.tick() is a generator; consume it the way BehaviourTree does.
    for _ in node.tick():
        pass
    return node.status


def test_settles_when_still():
    c = FakeRobotClient()
    c.set_positions({"arm_left_elbow_flex": 2048, "arm_right_elbow_flex": 2048})
    node = WaitSettled(_ctx(c), tol=15, window=0.1, timeout=1.0)
    t0 = time.monotonic()
    status = _tick(node)
    while status == Status.RUNNING and time.monotonic() - t0 < 2.0:
        time.sleep(0.02)
        status = _tick(node)
    assert status == Status.SUCCESS


def test_holds_while_moving_then_times_out_to_success():
    c = FakeRobotClient()
    node = WaitSettled(_ctx(c), tol=15, window=0.1, timeout=0.4)
    _tick(node)
    t0 = time.monotonic()
    saw_running = False
    p = 1000
    while time.monotonic() - t0 < 0.3:
        p += 100
        c.set_positions({"arm_left_elbow_flex": p})
        if _tick(node) == Status.RUNNING:
            saw_running = True
        time.sleep(0.02)
    assert saw_running  # it did not falsely settle while moving
    time.sleep(0.2)     # past the timeout -> proceed rather than hang
    assert _tick(node) == Status.SUCCESS
