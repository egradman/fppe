from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("web_gamepad")
@dataclass
class WebGamepadTeleopConfig(TeleoperatorConfig):
    """Configuration for web gamepad teleoperator.

    Runs a websocket server that receives gamepad state from a browser page
    using the Web Gamepad API.

    Attributes:
        host: Host to bind the websocket server to
        port: Port for the websocket server
        http_port: Port for the HTTP server serving the gamepad page
        deadzone: Stick deadzone (0.0-1.0)
    """

    host: str = "0.0.0.0"
    port: int = 8765
    http_port: int = 8766
    deadzone: float = 0.1
