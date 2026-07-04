"""NavClient — the conductor's window into the nav-serve visual-nav service.

nav-serve (in the `nav/` uv project, on the GPU) runs ViNT against the fisheye
and publishes a body-velocity command for the current goal photo. This is the
thin HTTP client a behavior polls each tick; it holds no model. Analogous to
PerceptionClient, but nav lives in its own process (torch stays out of the
conductor venv), so this talks to it over HTTP rather than calling in-process.

  set_goal(bytes) -> POST /goal      start driving toward that photo
  clear_goal()    -> DELETE /goal    stop driving (command goes inactive)
  get()           -> GET /command    latest NavCommand (online=False if unreachable)

``FakeNavClient`` scripts commands directly for tests — no socket, no service.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass

import httpx

log = logging.getLogger(__name__)


@dataclass
class NavCommand:
    online: bool = False       # was the service reachable this read?
    active: bool = False       # is a goal set and being driven toward?
    reached: bool = False      # has the goal been reached (closest approach / final crumb)?
    vx: float = 0.0            # m/s +forward
    vy: float = 0.0            # m/s +strafe-left
    theta: float = 0.0         # deg/s +CCW
    dist: float | None = None  # ViNT temporal distance to current goal/breadcrumb
    reason: str = ""
    node: int | None = None    # route mode: current target breadcrumb node id
    leg: str = ""              # route mode: "k/N" progress through the breadcrumb sequence


class NavClient:
    def __init__(self, base_url: str = "http://localhost:8107", timeout: float = 1.0):
        self.base_url = base_url.rstrip("/")
        self._http = httpx.Client(base_url=self.base_url, timeout=timeout)

    def set_goal(self, image_bytes: bytes) -> None:
        self._http.post("/goal", content=image_bytes,
                        headers={"Content-Type": "application/octet-stream"})

    def clear_goal(self) -> None:
        try:
            self._http.delete("/goal")
        except Exception as exc:
            log.debug("nav clear_goal failed: %s", exc)

    def set_route(self, label: str | None = None, node: int | None = None,
                  start: int | None = None) -> dict:
        """Begin following the loaded route toward a label/node. Blocks while
        nav-serve localizes the start node (several ViNT inferences), so this
        gets a generous timeout, not the per-tick one."""
        body: dict = {}
        if label is not None:
            body["label"] = label
        if node is not None:
            body["node"] = node
        if start is not None:
            body["start"] = start
        try:
            r = self._http.post("/route", json=body, timeout=20.0)
            return r.json()
        except Exception as exc:
            log.debug("nav set_route failed: %s", exc)
            return {"ok": False, "error": str(exc)}

    def clear_route(self) -> None:
        try:
            self._http.delete("/route")
        except Exception as exc:
            log.debug("nav clear_route failed: %s", exc)

    def get(self) -> NavCommand:
        try:
            r = self._http.get("/command")
            d = r.json()
        except Exception as exc:
            log.debug("nav get failed: %s", exc)
            return NavCommand(online=False)
        return NavCommand(
            online=True,
            active=bool(d.get("active")),
            reached=bool(d.get("reached")),
            vx=float(d.get("vx", 0.0)),
            vy=float(d.get("vy", 0.0)),
            theta=float(d.get("theta", 0.0)),
            dist=d.get("dist"),
            reason=str(d.get("reason", "")),
            node=d.get("node"),
            leg=str(d.get("leg", "")),
        )

    def close(self) -> None:
        self._http.close()


class FakeNavClient:
    """Scriptable nav for tests. No service."""

    def __init__(self) -> None:
        self._cmd = NavCommand(online=True)
        self.goals: list[bytes] = []
        self.routes: list[dict] = []
        self.cleared = 0
        self.routes_cleared = 0

    def set_goal(self, image_bytes: bytes) -> None:
        self.goals.append(image_bytes)
        self._cmd = NavCommand(online=True, active=True, reason="goal set")

    def clear_goal(self) -> None:
        self.cleared += 1
        self._cmd = NavCommand(online=True, active=False, reason="cleared")

    def set_route(self, label: str | None = None, node: int | None = None,
                  start: int | None = None) -> dict:
        self.routes.append({"label": label, "node": node, "start": start})
        self._cmd = NavCommand(online=True, active=True, reason="route set")
        return {"ok": True, "start": start or 0, "goal": node or 0, "sequence": []}

    def clear_route(self) -> None:
        self.routes_cleared += 1
        self._cmd = NavCommand(online=True, active=False, reason="route cleared")

    def get(self) -> NavCommand:
        return self._cmd

    def close(self) -> None:
        pass

    # -- test controls ---------------------------------------------------
    def set_command(self, **kw) -> None:
        self._cmd = NavCommand(online=True, active=True, **kw)
