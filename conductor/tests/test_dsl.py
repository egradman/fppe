"""DSL round-trip + registry catalog sanity."""

import json

import pytest

from conductor.client import FakeRobotClient
from conductor.context import Context
from conductor.dsl import build_tree, tree_to_dsl
from conductor.modes import list_modes, load_mode
from conductor.registry import catalog
from conductor.senders import FakeSenderManager


@pytest.fixture
def ctx():
    return Context(client=FakeRobotClient(), senders=FakeSenderManager(fail_unavailable=False))


def test_catalog_is_json_and_has_core_behaviors():
    cat = catalog()
    json.dumps(cat)  # must be serialisable
    ids = {b["type"] for b in cat["behaviors"]}
    for expected in {"set_base_source", "set_teleop_mode", "lock_arm", "start_sender",
                     "stop_sender", "hold", "navigate_to", "safe_stop", "estop_engaged"}:
        assert expected in ids
    assert {c["type"] for c in cat["composites"]} == {"sequence", "selector", "parallel"}


def test_build_and_roundtrip_teleop(ctx):
    dsl = load_mode("teleop")
    tree = build_tree(dsl, ctx)
    back = tree_to_dsl(tree)
    assert back["type"] == "sequence"
    assert back["name"] == "teleop"
    child_types = [c["type"] for c in back["children"]]
    assert child_types == [
        "set_teleop_mode", "lock_arm", "start_sender", "set_base_source", "start_sender", "hold",
    ]
    # params survive the round trip
    assert back["children"][0]["params"] == {"mode": "remote"}
    assert back["children"][2]["params"] == {"sender": "leader"}


def test_all_modes_build(ctx):
    for name in list_modes():
        tree = build_tree(load_mode(name), ctx)
        assert tree is not None


def test_parallel_and_selector_build(ctx):
    dsl = {
        "type": "selector",
        "name": "fallback",
        "children": [
            {"type": "source_is", "params": {"value": "pedals"}},
            {"type": "parallel", "policy": "success_on_one", "children": [
                {"type": "hold"},
            ]},
        ],
    }
    tree = build_tree(dsl, ctx)
    back = tree_to_dsl(tree)
    assert back["type"] == "selector"
    assert back["children"][1]["type"] == "parallel"


def test_unknown_behavior_raises(ctx):
    with pytest.raises(KeyError):
        build_tree({"type": "does_not_exist"}, ctx)
