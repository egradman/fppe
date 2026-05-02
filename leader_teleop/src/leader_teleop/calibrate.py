"""CLI: interactively calibrate one or both leader arms."""

from __future__ import annotations

import argparse
import logging

from .arms import LeaderArm
from .calibration import Calibration, calibrate_interactive

# Stable serial-by-id paths. Confirmed left/right with the user 2026-05-02.
LEFT_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A056746-if00"
RIGHT_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A7A056664-if00"


def calibrate_one(side: str) -> None:
    port = LEFT_PORT if side == "left" else RIGHT_PORT
    arm = LeaderArm(port=port, name=side)
    arm.connect()
    try:
        existing = Calibration.load(side)
        if existing is not None:
            ans = input(
                f"Calibration already exists for {side} ({len(existing.joints)} joints). "
                f"Recalibrate? [y/N] "
            )
            if ans.strip().lower() not in ("y", "yes"):
                print(f"Keeping existing calibration for {side}.")
                return
        calibrate_interactive(arm)
    finally:
        arm.disconnect()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arms",
        choices=("both", "left", "right"),
        default="both",
        help="Which arm(s) to calibrate.",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    if args.arms in ("both", "left"):
        calibrate_one("left")
    if args.arms in ("both", "right"):
        calibrate_one("right")
    print("\nDone.")


if __name__ == "__main__":
    main()
