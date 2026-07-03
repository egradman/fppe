"""Engine behaviour against FakeRobotClient — no hardware.

Covers the three properties the whole design hinges on:
  1. a mode establishes its Pi-side switches and senders,
  2. swapping modes tears the old one down (senders killed, base off),
  3. the e-stop preempts the active mission and the mode restores when it clears.
"""

import pytest

from conductor.client import FakeRobotClient
from conductor.context import Context
from conductor.engine import Engine
from conductor.modes import load_mode
from conductor.senders import FakeSenderManager


@pytest.fixture
def engine():
    ctx = Context(client=FakeRobotClient(), senders=FakeSenderManager(fail_unavailable=False))
    eng = Engine(ctx, tick_hz=100.0)
    eng.setup()
    yield eng
    eng.shutdown()


def tick(eng: Engine, n: int = 1) -> None:
    for _ in range(n):
        eng.tick_once()


def test_teleop_establishes(engine):
    engine.load_mission("teleop", load_mode("teleop"))
    tick(engine, 2)
    snap = engine.ctx.client.snapshot()
    assert snap.teleop_mode == "remote"
    assert snap.base_input_source == "pedals"
    assert snap.arm_locked == {"left": True, "right": True}
    assert set(engine.ctx.senders.running()) == {"leader", "pedal"}
    assert engine.current_mission() == "teleop"


def test_switch_to_idle_tears_down(engine):
    engine.load_mission("teleop", load_mode("teleop"))
    tick(engine, 2)
    assert engine.ctx.senders.running()  # precondition

    engine.load_mission("idle", load_mode("idle"))
    tick(engine, 2)
    snap = engine.ctx.client.snapshot()
    assert snap.base_input_source == "off"
    assert snap.teleop_mode == "local"
    # senders killed via INVALID teardown on the preempted teleop subtree
    assert engine.ctx.senders.running() == []


def test_estop_preempts_and_restores(engine):
    engine.load_mission("teleop", load_mode("teleop"))
    tick(engine, 2)
    assert set(engine.ctx.senders.running()) == {"leader", "pedal"}

    # e-stop trips
    engine.ctx.client.set_estop(True)
    tick(engine, 2)
    snap = engine.ctx.client.snapshot()
    assert snap.estop_engaged
    assert snap.base_input_source == "off"     # SafeStop ran
    assert engine.ctx.senders.running() == []  # mission preempted -> senders down

    # e-stop clears -> the mode re-establishes itself
    engine.ctx.client.set_estop(False)
    tick(engine, 2)
    snap = engine.ctx.client.snapshot()
    assert snap.base_input_source == "pedals"
    assert set(engine.ctx.senders.running()) == {"leader", "pedal"}


def test_append_then_idle_fallback(engine):
    a = {"type": "set_base_source", "name": "a", "params": {"value": "gamepad"}}
    b = {"type": "set_base_source", "name": "b", "params": {"value": "off"}}
    engine.load_mission("a", a)
    engine.append("b", b)
    tick(engine, 6)
    # a -> b -> (queue empty) -> idle fallback
    assert engine.current_mission() == "idle"
    assert engine.ctx.client.snapshot().base_input_source == "off"


def test_abort_disengages_base_source(engine):
    # a base-driving mode leaves the source engaged; abort must turn it off
    driving = {"type": "sequence", "name": "drive", "memory": True, "children": [
        {"type": "set_base_source", "params": {"value": "gamepad"}},
        {"type": "hold"},
    ]}
    engine.load_mission("drive", driving)
    tick(engine, 2)
    assert engine.ctx.client.snapshot().base_input_source == "gamepad"

    engine.abort()
    tick(engine, 2)
    assert engine.current_mission() == "idle"
    assert engine.ctx.client.snapshot().base_input_source == "off"  # not left engaged


def test_nav_stub_fails_gracefully(engine):
    engine.load_mission("nav", load_mode("nav"))
    tick(engine, 3)
    # navigate_to is a stub that FAILUREs; engine falls back to idle.
    assert engine.current_mission() == "idle"
