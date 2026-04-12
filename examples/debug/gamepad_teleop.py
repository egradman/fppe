#!/usr/bin/env python3
"""Gamepad teleop: drives omni wheels + lift axis via web gamepad.

Left stick: x/y movement (strafe + forward/back)
Right stick LR (axis 2): rotation
Button 0 (A/Cross): lift down
Button 1 (B/Circle): lift up
"""

from __future__ import annotations
import argparse
import time
from typing import Dict, List

import numpy as np

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.teleoperators.web_gamepad import WebGamepadTeleop, WebGamepadTeleopConfig

# ---- Wheel constants ---- #
MODEL = "sts3215"
LEFT_ID = 8
BACK_ID = 9
RIGHT_ID = 10
LIN_SPEED = 0.2       # m/s at full stick
ANG_SPEED = 80.0      # deg/s at full stick
WHEEL_RADIUS = 0.05
BASE_RADIUS = 0.125
MAX_RAW = 3000

# ---- Lift constants ---- #
LIFT_ID = 11
LIFT_SPEED_DEGPS = 180.0
LIFT_CURRENT_CUTOFF = 1000.0
LIFT_SAMPLES_TO_TRIGGER = 2

STEPS_PER_DEG = 4096.0 / 360.0


def degps_to_raw(degps: float) -> int:
    mag = int(round(abs(degps) * STEPS_PER_DEG))
    if mag > 0x7FFF:
        mag = 0x7FFF
    return -mag if degps < 0 else mag


def body_to_wheel_raw(
    x_cmd: float, y_cmd: float, theta_cmd_degps: float,
) -> Dict[str, int]:
    theta_rad = theta_cmd_degps * (np.pi / 180.0)
    vel = np.array([-x_cmd, -y_cmd, theta_rad])

    angles = np.radians(np.array([240, 0, 120]) - 90)
    M = np.array([[np.cos(a), np.sin(a), BASE_RADIUS] for a in angles])

    v_lin = M.dot(vel)
    w_rad = v_lin / WHEEL_RADIUS
    w_degps = w_rad * (180.0 / np.pi)

    steps_per_deg = 4096.0 / 360.0
    raw_abs = np.abs(w_degps) * steps_per_deg
    peak = float(np.max(raw_abs)) if raw_abs.size else 0.0
    if peak > MAX_RAW and peak > 1e-6:
        w_degps = w_degps * (MAX_RAW / peak)

    raw = [degps_to_raw(v) for v in w_degps]
    return {"left_wheel": raw[0], "back_wheel": raw[1], "right_wheel": raw[2]}


def main():
    parser = argparse.ArgumentParser(description="Gamepad teleop: wheels + lift")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--ws-port", type=int, default=8765)
    parser.add_argument("--http-port", type=int, default=8766)
    args = parser.parse_args()

    # -- Motors -- #
    wheel_motors = {
        "left_wheel":  Motor(id=LEFT_ID,  model=MODEL, norm_mode=MotorNormMode.RANGE_0_100),
        "back_wheel":  Motor(id=BACK_ID,  model=MODEL, norm_mode=MotorNormMode.RANGE_0_100),
        "right_wheel": Motor(id=RIGHT_ID, model=MODEL, norm_mode=MotorNormMode.RANGE_0_100),
        "lift_axis":   Motor(id=LIFT_ID,  model=MODEL, norm_mode=MotorNormMode.RANGE_0_100),
    }
    bus = FeetechMotorsBus(port=args.port, motors=wheel_motors)
    bus.connect(handshake=False)
    print(f"[INFO] Connected on {args.port}")

    for name in wheel_motors:
        try:
            bus.write("Lock", name, 0, normalize=False)
        except Exception:
            pass
        try:
            bus.disable_torque(name)
        except Exception:
            pass
        bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value, normalize=False)
        bus.enable_torque(name)
    print("[INFO] All motors set to VELOCITY mode.")

    # -- Gamepad -- #
    gamepad = WebGamepadTeleop(WebGamepadTeleopConfig(port=args.ws_port, http_port=args.http_port))
    gamepad.connect()

    lift_over_cnt = 0

    try:
        while True:
            axes = gamepad.get_axes()
            buttons = gamepad.get_buttons()

            # -- Wheels: left stick x/y + right stick rotation -- #
            if len(axes) >= 4:
                x_cmd = axes[1] * LIN_SPEED    # left stick UD -> strafe
                y_cmd = -axes[0] * LIN_SPEED   # left stick LR -> forward/back
                th_cmd = -axes[2] * ANG_SPEED  # right stick LR -> rotate
            else:
                x_cmd = y_cmd = th_cmd = 0.0

            wheel_cmds = body_to_wheel_raw(x_cmd, y_cmd, th_cmd)
            for name in ("left_wheel", "back_wheel", "right_wheel"):
                bus.write("Goal_Velocity", name, wheel_cmds[name], normalize=False)

            # -- Lift: button 0 = down, button 1 = up -- #
            btn_down = buttons[12] if len(buttons) > 12 else False
            btn_up = buttons[13] if len(buttons) > 13 else False

            # Overcurrent protection
            try:
                raw_current = bus.read("Present_Current", "lift_axis", normalize=False)
                if isinstance(raw_current, tuple):
                    raw_current = raw_current[0]
                current_ma = (raw_current or 0) * 6.5
            except Exception:
                current_ma = 0.0

            if current_ma > LIFT_CURRENT_CUTOFF:
                lift_over_cnt += 1
            else:
                lift_over_cnt = 0

            if lift_over_cnt >= LIFT_SAMPLES_TO_TRIGGER:
                print(f"\n[SAFETY] Lift overcurrent: {current_ma:.0f} mA — stopping lift")
                bus.write("Goal_Velocity", "lift_axis", 0, normalize=False)
                try:
                    bus.disable_torque("lift_axis")
                except Exception:
                    pass
                break

            if btn_up and not btn_down:
                lift_raw = degps_to_raw(-LIFT_SPEED_DEGPS)
            elif btn_down and not btn_up:
                lift_raw = degps_to_raw(LIFT_SPEED_DEGPS)
            else:
                lift_raw = 0
            bus.write("Goal_Velocity", "lift_axis", lift_raw, normalize=False)

            # -- Status -- #
            if axes:
                print(
                    f"x:{x_cmd:+.2f} y:{y_cmd:+.2f} th:{th_cmd:+.1f} "
                    f"lift:{'UP' if btn_up else 'DN' if btn_down else '--'} "
                    f"I:{current_ma:.0f}mA",
                    end="\r",
                )

            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping...")
    finally:
        # Stop all motors
        for name in wheel_motors:
            try:
                bus.write("Goal_Velocity", name, 0, normalize=False)
            except Exception:
                pass
        gamepad.disconnect()
        bus.disconnect(disable_torque=False)
        print("Done.")


if __name__ == "__main__":
    main()
