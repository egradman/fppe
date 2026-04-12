import asyncio
import json
import logging
import threading
import time
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
from typing import Any

import websockets

from lerobot.processor import RobotAction
from lerobot.utils.decorators import check_if_not_connected

from ..teleoperator import Teleoperator
from .config_web_gamepad import WebGamepadTeleopConfig

logger = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"


class WebGamepadTeleop(Teleoperator):
    """Teleoperator that receives gamepad input from a web browser via WebSocket.

    On connect(), starts:
      1. A WebSocket server that receives JSON gamepad state from the browser
      2. An HTTP server that serves the gamepad HTML page

    The browser reads the Gamepad API and streams axis/button state at ~60Hz.
    get_action() returns the latest state received.
    """

    config_class = WebGamepadTeleopConfig
    name = "web_gamepad"

    def __init__(self, config: WebGamepadTeleopConfig):
        super().__init__(config)
        self.config = config

        self._connected = False
        self._ws_thread: threading.Thread | None = None
        self._http_thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ws_server = None
        self._http_server: HTTPServer | None = None

        # Latest gamepad state from browser
        self._lock = threading.Lock()
        self._axes: list[float] = []
        self._buttons: list[bool] = []
        self._last_update: float = 0.0

    @property
    def action_features(self) -> dict:
        return {
            "axes": list[float],
            "buttons": list[bool],
        }

    @property
    def feedback_features(self) -> dict:
        return {}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return

        # Start WebSocket server in a background thread
        self._ws_thread = threading.Thread(target=self._run_ws_server, daemon=True)
        self._ws_thread.start()

        # Start HTTP server for the gamepad page
        self._http_thread = threading.Thread(target=self._run_http_server, daemon=True)
        self._http_thread.start()

        self._connected = True
        logger.info(
            f"WebGamepad teleop started. Open http://localhost:{self.config.http_port} "
            f"in a browser and connect a gamepad."
        )
        print(
            f"[WebGamepad] Open http://localhost:{self.config.http_port} in a browser. "
            f"WebSocket on ws://localhost:{self.config.port}"
        )

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    async def _ws_handler(self, websocket):
        logger.info(f"WebSocket client connected: {websocket.remote_address}")
        print(f"[WebGamepad] Browser connected from {websocket.remote_address}")
        try:
            async for message in websocket:
                data = json.loads(message)
                axes = data.get("axes", [])
                buttons = data.get("buttons", [])

                # Apply deadzone
                dz = self.config.deadzone
                axes = [0.0 if abs(v) < dz else v for v in axes]

                with self._lock:
                    self._axes = axes
                    self._buttons = buttons
                    self._last_update = time.monotonic()
        except websockets.ConnectionClosed:
            print("[WebGamepad] Browser disconnected")
        except Exception as e:
            print(f"[WebGamepad] Handler error: {e}")
            import traceback
            traceback.print_exc()

    def _run_ws_server(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        async def serve():
            self._ws_server = await websockets.serve(
                self._ws_handler, self.config.host, self.config.port
            )
            print(f"[WebGamepad] WebSocket server listening on {self.config.host}:{self.config.port}")
            await self._ws_server.wait_closed()

        try:
            self._loop.run_until_complete(serve())
        except Exception as e:
            print(f"[WebGamepad] WebSocket server error: {e}")

    def _run_http_server(self):
        handler = partial(SimpleHTTPRequestHandler, directory=str(STATIC_DIR))
        self._http_server = HTTPServer(("", self.config.http_port), handler)
        self._http_server.serve_forever()

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        with self._lock:
            return {
                "axes": list(self._axes),
                "buttons": list(self._buttons),
            }

    def get_axes(self) -> list[float]:
        """Get raw axis values (with deadzone applied)."""
        with self._lock:
            return list(self._axes)

    def get_buttons(self) -> list[bool]:
        """Get raw button states."""
        with self._lock:
            return list(self._buttons)

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        pass

    def disconnect(self) -> None:
        if not self._connected:
            return

        if self._ws_server and self._loop:
            self._loop.call_soon_threadsafe(self._ws_server.close)

        if self._http_server:
            self._http_server.shutdown()

        self._connected = False
        logger.info("WebGamepad teleop stopped.")
