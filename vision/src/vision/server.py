"""Vision HTTP API — the robot's on-demand sense of sight.

A dependency-free (stdlib http.server), CORS-open service that, on request,
grabs a frame from the Pi's camera and runs it through Moondream. The voice/LLM
`look` tool calls this directly (it's a read-only *query*, not an action, so it
does NOT go through the conductor, which owns actuation/mode switching).

  GET  /health                 -> {ok, model, device, loaded}
  GET  /look?q=<question>&cam=  -> {ok, text, cam, question}
       q   optional free-text question (VQA); omitted -> a short caption
       cam optional camera label (default cam0); the other eye is cam1
"""
from __future__ import annotations

import json
import logging
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

log = logging.getLogger(__name__)


class VisionHandler(BaseHTTPRequestHandler):
    model = None  # injected by make_server()
    pi_base = "http://fppe:8091"
    default_cam = "cam0"

    # -- helpers ---------------------------------------------------------
    def _cors(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _json(self, code: int, data: dict) -> None:
        body = json.dumps(data).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self._cors()
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    # -- verbs -----------------------------------------------------------
    def do_OPTIONS(self) -> None:
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self) -> None:
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/health":
            self._json(200, {
                "ok": True,
                "model": self.model.model_id,
                "revision": self.model.revision,
                "device": self.model.device,
                "loaded": self.model.loaded,
            })
            return
        if parsed.path == "/look":
            qs = urllib.parse.parse_qs(parsed.query)
            question = (qs.get("q") or qs.get("question") or [""])[0].strip()
            cam = (qs.get("cam") or [self.default_cam])[0]
            url = f"{self.pi_base}/mjpeg/{cam}"
            try:
                from vision.frame import grab_frame
                image = grab_frame(url)
            except Exception as exc:
                log.warning("frame grab failed: %s", exc)
                self._json(502, {"ok": False, "error": f"camera unavailable: {exc}"})
                return
            try:
                text = self.model.describe(image, question)
            except Exception as exc:
                log.exception("describe failed")
                self._json(500, {"ok": False, "error": f"vision model error: {exc}"})
                return
            log.info("look cam=%s q=%r -> %r", cam, question or None, text)
            self._json(200, {
                "ok": True, "text": text, "cam": cam,
                "question": question or None,
            })
            return
        self._json(404, {"ok": False, "error": "not found"})

    def log_message(self, *args) -> None:
        pass  # silence per-request logging (we log meaningfully above)


def make_server(model, host: str = "0.0.0.0", port: int = 8102,
                pi_base: str = "http://fppe:8091", cam: str = "cam0") -> ThreadingHTTPServer:
    VisionHandler.model = model
    VisionHandler.pi_base = pi_base.rstrip("/")
    VisionHandler.default_cam = cam
    httpd = ThreadingHTTPServer((host, port), VisionHandler)
    log.info("vision HTTP API listening on %s:%d (cameras via %s)", host, port, pi_base)
    return httpd
