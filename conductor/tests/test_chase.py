"""chase_dogs against fakes — the reactive search/pursue loop, no hardware.

Drives the FakePerceptionClient to script what the "camera" sees and asserts the
FakeBaseCmdChannel setpoint the tree produces: spin-to-search with no target,
turn+advance when a target appears, halt when close/aligned, back to search when
lost, and a full stop on e-stop.
"""

import pytest

from conductor.base_cmd import FakeBaseCmdChannel
from conductor.client import FakeRobotClient
from conductor.context import Context
from conductor.engine import Engine
from conductor.modes import load_mode
from conductor.perception import FakePerceptionClient
from conductor.senders import FakeSenderManager


@pytest.fixture
def engine():
    perception = FakePerceptionClient()
    base = FakeBaseCmdChannel()
    ctx = Context(
        client=FakeRobotClient(),
        senders=FakeSenderManager(fail_unavailable=False),
        base=base,
        perception=perception,
    )
    eng = Engine(ctx, tick_hz=100.0)
    eng.setup()
    yield eng
    eng.shutdown()


def tick(eng: Engine, n: int = 1) -> None:
    for _ in range(n):
        eng.tick_once()


def test_setup_and_search_when_nothing_seen(engine):
    engine.load_mission("chase_dogs", load_mode("chase_dogs"))
    tick(engine, 2)
    # perception was asked to track the dog, base handed to the conductor
    assert "dog" in engine.ctx.perception.requested
    assert engine.ctx.client.snapshot().base_input_source == "auto"
    # no sighting -> search_pattern spins in place
    assert engine.ctx.base.setpoint() == (0.0, 0.0, 30.0)


def test_pursues_when_target_appears(engine):
    engine.load_mission("chase_dogs", load_mode("chase_dogs"))
    tick(engine, 2)
    # dog to the left (positive bearing) and far (small area)
    engine.ctx.perception.set_sighting("dog", bearing_rad=0.05, area_frac=0.02, conf=0.9)
    tick(engine, 1)
    x, y, theta = engine.ctx.base.setpoint()
    assert x > 0.0          # driving forward toward it
    assert theta > 0.0      # turning left toward it
    assert y == 0.0


def test_halts_when_close_and_aligned(engine):
    engine.load_mission("chase_dogs", load_mode("chase_dogs"))
    tick(engine, 2)
    # centred and closer than standoff -> stop advancing, no turn needed
    engine.ctx.perception.set_sighting("dog", bearing_rad=0.0, area_frac=0.30, conf=0.9)
    tick(engine, 1)
    assert engine.ctx.base.setpoint() == (0.0, 0.0, 0.0)
    assert engine.current_mission() == "chase_dogs"  # still pursuing, not ended


def test_reverts_to_search_when_target_lost(engine):
    engine.load_mission("chase_dogs", load_mode("chase_dogs"))
    tick(engine, 2)
    engine.ctx.perception.set_sighting("dog", bearing_rad=0.05, area_frac=0.02, conf=0.9)
    tick(engine, 1)
    assert engine.ctx.base.setpoint()[0] > 0.0  # pursuing

    engine.ctx.perception.clear("dog")
    tick(engine, 1)
    # reactive selector falls back to search
    assert engine.ctx.base.setpoint() == (0.0, 0.0, 30.0)


def test_estop_stops_everything(engine):
    engine.load_mission("chase_dogs", load_mode("chase_dogs"))
    tick(engine, 2)
    engine.ctx.perception.set_sighting("dog", bearing_rad=0.05, area_frac=0.02, conf=0.9)
    tick(engine, 1)

    engine.ctx.client.set_estop(True)
    tick(engine, 2)
    snap = engine.ctx.client.snapshot()
    assert snap.base_input_source == "off"          # SafeStop released the base
    assert engine.ctx.base.setpoint() == (0.0, 0.0, 0.0)
    assert "dog" in engine.ctx.perception.released   # perception torn down on preempt
