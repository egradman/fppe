"""Mini HTTP server + browser game for testing pedal commands locally.

Exposes:
  GET /        — static game.html (car on infinite grid, lift = pole on top)
  GET /events  — server-sent events with the latest body command at ~30 Hz

Server is intentionally tiny (stdlib only, threaded). Each connected SSE
client gets its own writer thread that polls the latest snapshot — there's
no fan-out queue, so a slow client can't backpressure others.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


SSE_RATE_HZ = 30.0


class GameBroadcaster:
    """Thread-safe holder for the latest body command, polled by SSE clients."""

    def __init__(self):
        self._lock = threading.Lock()
        self._snapshot: dict | None = None

    def publish(
        self,
        x_vel: float,
        y_vel: float,
        theta_vel: float,
        lift: str,
        stale: bool,
    ) -> None:
        with self._lock:
            self._snapshot = {
                "ts": time.time(),
                "x_vel": x_vel,
                "y_vel": y_vel,
                "theta_vel": theta_vel,
                "lift": lift,
                "stale": stale,
            }

    def snapshot(self) -> dict | None:
        with self._lock:
            return dict(self._snapshot) if self._snapshot else None


_GAME_HTML = Path(__file__).parent / "game.html"


class _Handler(BaseHTTPRequestHandler):
    broadcaster: GameBroadcaster

    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            try:
                content = _GAME_HTML.read_bytes()
            except OSError:
                self.send_error(500, "game.html missing")
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(content)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            period = 1.0 / SSE_RATE_HZ
            try:
                while True:
                    snap = self.broadcaster.snapshot()
                    if snap is not None:
                        payload = f"data: {json.dumps(snap)}\n\n"
                        self.wfile.write(payload.encode())
                        self.wfile.flush()
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
        else:
            self.send_error(404)


def start_game_server(port: int, broadcaster: GameBroadcaster) -> ThreadingHTTPServer:
    _Handler.broadcaster = broadcaster
    httpd = ThreadingHTTPServer(("0.0.0.0", port), _Handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True, name="pedal-game-http")
    t.start()
    return httpd
