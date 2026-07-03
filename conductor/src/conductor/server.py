"""Conductor HTTP API — the browser/LLM control surface for mode switching.

The behavior tree owns mode switching; clients (the web UI, an LLM, curl) no
longer poke viser_control's /base_input_source directly. They ask the conductor
for a mode and it effects the underlying switches + sender lifecycle.

Runs alongside the engine tick loop (engine spins in a background thread; these
handlers mutate it through its thread-safe load_mission/abort). Dependency-free
(stdlib http.server) and CORS-open, since the browser served from the Pi
(fppe:8091) calls this cross-origin on skynet.

  GET  /status         -> {mission, modes, estop, online, robot{...}, tree}
  GET  /modes          -> {modes: [...]}
  GET  /catalog        -> the behavior catalog (leaves + composites)
  POST /mission        {name}          -> load a named mode
  POST /mission/dsl    {name?, dsl}    -> load an arbitrary tree (LLM/custom)
  POST /abort                          -> drop to idle
"""

from __future__ import annotations

import json
import logging
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import py_trees

from conductor.modes import list_modes, load_mode
from conductor.registry import catalog

log = logging.getLogger(__name__)


class ConductorHandler(BaseHTTPRequestHandler):
    engine = None  # injected by serve()

    # -- helpers ---------------------------------------------------------
    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, code: int, data: dict) -> None:
        body = json.dumps(data).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self._cors()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self) -> dict | None:
        try:
            n = int(self.headers.get("Content-Length", 0) or 0)
            return json.loads(self.rfile.read(n)) if n else {}
        except Exception:
            return None

    def _status(self) -> dict:
        eng = self.engine
        snap = eng.ctx.client.snapshot()
        return {
            "mission": eng.current_mission(),
            "modes": list_modes(),
            "estop": snap.estop_engaged,
            "online": snap.online,
            "robot": {
                "base_input_source": snap.base_input_source,
                "teleop_mode": snap.teleop_mode,
                "arm_locked": snap.arm_locked,
            },
            "senders": eng.ctx.senders.running(),
            "tree": py_trees.display.unicode_tree(eng.root, show_status=True),
        }

    # -- verbs -----------------------------------------------------------
    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self) -> None:
        if self.path == "/status":
            self._json(200, self._status())
        elif self.path == "/modes":
            self._json(200, {"modes": list_modes()})
        elif self.path == "/catalog":
            self._json(200, catalog())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self) -> None:
        eng = self.engine
        body = self._read_json()
        if body is None:
            self._json(400, {"error": "invalid JSON"})
            return

        if self.path == "/mission":
            name = body.get("name")
            if name not in list_modes():
                self._json(400, {"error": f"unknown mode {name!r}", "modes": list_modes()})
                return
            eng.load_mission(name, load_mode(name))
            log.info("HTTP -> load_mission(%s)", name)
            self._json(200, {"ok": True, "mission": name})

        elif self.path == "/mission/dsl":
            name = body.get("name", "custom")
            dsl = body.get("dsl")
            if not isinstance(dsl, dict):
                self._json(400, {"error": "'dsl' must be a tree object"})
                return
            eng.load_mission(name, dsl)
            log.info("HTTP -> load_mission(%s, <dsl>)", name)
            self._json(200, {"ok": True, "mission": name})

        elif self.path == "/abort":
            eng.abort()
            log.info("HTTP -> abort")
            self._json(200, {"ok": True, "mission": "idle"})

        else:
            self._json(404, {"error": "not found"})

    def log_message(self, *args) -> None:
        pass  # silence per-request logging


def make_server(engine, host: str = "0.0.0.0", port: int = 8100) -> ThreadingHTTPServer:
    ConductorHandler.engine = engine
    httpd = ThreadingHTTPServer((host, port), ConductorHandler)
    log.info("conductor HTTP API listening on %s:%d", host, port)
    return httpd
