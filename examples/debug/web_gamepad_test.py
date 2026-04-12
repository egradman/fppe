#!/usr/bin/env python3
"""Quick test: print web gamepad state to terminal."""

import argparse
import time

from lerobot.teleoperators.web_gamepad import WebGamepadTeleop, WebGamepadTeleopConfig


def main():
    parser = argparse.ArgumentParser(description="Test web gamepad teleop")
    parser.add_argument("--port", type=int, default=8765, help="WebSocket port")
    parser.add_argument("--http-port", type=int, default=8766, help="HTTP port for gamepad page")
    args = parser.parse_args()

    config = WebGamepadTeleopConfig(port=args.port, http_port=args.http_port)
    teleop = WebGamepadTeleop(config)
    teleop.connect()

    try:
        while True:
            action = teleop.get_action()
            axes = action["axes"]
            buttons = action["buttons"]
            if axes:
                axes_str = " ".join(f"{v:+.2f}" for v in axes)
                pressed = [i for i, b in enumerate(buttons) if b]
                print(f"axes: [{axes_str}]  buttons: {pressed}", end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        teleop.disconnect()


if __name__ == "__main__":
    main()
