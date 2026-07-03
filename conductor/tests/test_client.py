"""RobotClient wire-level checks — guards the fake/real divergence that let the
lock_arm('both') 400 slip through."""

import json

import httpx

from conductor.client import RobotClient


def _mock_client():
    calls: list[tuple[str, dict]] = []

    def handler(request: httpx.Request) -> httpx.Response:
        body = json.loads(request.content) if request.content else {}
        calls.append((request.url.path, body))
        return httpx.Response(200, json={"ok": True})

    c = RobotClient(host="x")
    c._http = httpx.Client(transport=httpx.MockTransport(handler), base_url=c.base_url)
    return c, calls


def test_lock_arm_both_expands_to_two_per_side_posts():
    c, calls = _mock_client()
    c.lock_arm("both", True)
    assert [p for p, _ in calls] == ["/lock_arm", "/lock_arm"]
    assert {b["side"] for _, b in calls} == {"left", "right"}
    assert all(b["lock"] is True for _, b in calls)


def test_lock_arm_single_side_is_one_post():
    c, calls = _mock_client()
    c.lock_arm("left", False)
    assert calls == [("/lock_arm", {"side": "left", "lock": False})]


def test_go_preset_maps_to_endpoint():
    c, calls = _mock_client()
    c.go_preset("middle")
    assert calls == [("/go_middle", {})]
