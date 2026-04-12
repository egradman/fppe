#!/usr/bin/env python3
# axis.py - Overcurrent-protected axis demo (PORT parameterized)

import time
import argparse
import select
import sys
import termios
import tty
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

# ==================== Constants ==================== #
SERVO_ID = 11               # Servo ID
MODEL = "sts3215"           # Servo model
SPEED_DEGPS = 180.0         # Speed (deg/s)
CURRENT_CUTOFF = 1000.0      # Overcurrent threshold (mA)
SAMPLES_TO_TRIGGER = 2      # Consecutive samples before triggering
# ================================================= #

STEPS_PER_DEG = 4096.0 / 360.0  # ≈11.377 ticks/deg
TELEOP_KEYS = {"up": "r", "down": "f"}
pressed = {"up": False, "down": False}


def degps_to_raw(degps: float) -> int:
    mag = int(round(abs(degps) * STEPS_PER_DEG))
    if mag > 0x7FFF:
        mag = 0x7FFF
    return -mag if degps < 0 else mag


def read_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    parser = argparse.ArgumentParser(description="Lift-axis teleop with overcurrent cutoff")
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB-TTL port, e.g. /dev/ttyACM0")
    args = parser.parse_args()

    name = "lift_axis"
    motors = {name: Motor(id=SERVO_ID, model=MODEL, norm_mode=MotorNormMode.RANGE_0_100)}
    bus = FeetechMotorsBus(port=args.port, motors=motors)

    bus.connect(handshake=False)
    print(f"[INFO] Connected on {args.port}")
    try:
        bus.disable_torque(name)
    except Exception:
        pass
    bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value, normalize=False)
    try:
        bus.enable_torque(name)
    except Exception:
        pass

    print(f"[INFO] Ready. R ↑ / F ↓ / Q to quit.")
    print(f"[INFO] Overcurrent cutoff: {CURRENT_CUTOFF} mA, trigger after {SAMPLES_TO_TRIGGER} samples")

    old_settings = termios.tcgetattr(sys.stdin)
    over_cnt = 0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            # Reset pressed state each tick
            pressed["up"] = False
            pressed["down"] = False

            # Read all pending keys this tick
            ch = read_key()
            while ch is not None:
                if ch == "q":
                    return
                if ch == "\x1b":
                    # Consume rest of escape sequence (e.g. arrow keys)
                    while read_key() is not None:
                        pass
                    break
                if ch == TELEOP_KEYS["up"]:
                    pressed["up"] = True
                elif ch == TELEOP_KEYS["down"]:
                    pressed["down"] = True
                ch = read_key()

            # 1) Read current
            try:
                raw_current_ma = bus.read("Present_Current", name, normalize=False)
                if isinstance(raw_current_ma, tuple):
                    raw_current_ma = raw_current_ma[0]
                if raw_current_ma is None:
                    raw_current_ma = 0
                else:
                    current_ma = raw_current_ma * 6.5
                    print(f"[DEBUG] Present_Current={current_ma} mA")  # debug
            except Exception:
                raw_current_ma = 0

            # 2) Check for overcurrent
            if current_ma > CURRENT_CUTOFF:
                over_cnt += 1
            else:
                over_cnt = 0

            if over_cnt >= SAMPLES_TO_TRIGGER:
                print(f"\n[SAFETY] Overcurrent detected: {current_ma} mA ≥ {CURRENT_CUTOFF} mA")
                try:
                    bus.write("Goal_Velocity", name, 0, normalize=False)
                except Exception:
                    pass
                try:
                    bus.disable_torque(name)
                except Exception:
                    pass
                try:
                    bus.disconnect(disable_torque=True)
                except Exception:
                    pass
                return

            # 3) Velocity control
            if pressed["up"] and not pressed["down"]:
                raw = degps_to_raw(-SPEED_DEGPS)
            elif pressed["down"] and not pressed["up"]:
                raw = degps_to_raw(SPEED_DEGPS)
            else:
                raw = 0
            bus.write("Goal_Velocity", name, raw, normalize=False)

            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\n[INFO] Stopping motor...")
        try:
            bus.write("Goal_Velocity", name, 0, normalize=False)
        except Exception:
            pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass


if __name__ == "__main__":
    main()
