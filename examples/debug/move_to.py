"""Move one or more motors to target positions (slowly).

Usage:
    python move_to.py 3:2048                     # motor ID 3 to 2048
    python move_to.py 1:2048 2:1500 3:2048       # multiple motors
    python move_to.py all:2048                    # all arm motors to 2048
    python move_to.py --port /dev/ttyACM1 5:1000  # custom port

Options:
    --port PORT       serial port (default /dev/ttyACM0)
    --accel N         Maximum_Acceleration value (default 20, lower=slower)
    --step N          ticks per interpolation step (default 20)
"""

import argparse
import time
import sys

from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors import Motor, MotorNormMode


def parse_targets(args, bus):
    """Parse 'id:pos' or 'all:pos' arguments into {motor_name: target_pos}."""
    targets = {}
    for arg in args:
        if ":" not in arg:
            print(f"Invalid target '{arg}', expected id:position or all:position")
            sys.exit(1)
        key, pos = arg.split(":", 1)
        pos = int(pos)
        if key == "all":
            for name in bus.motors:
                targets[name] = pos
        else:
            motor_id = int(key)
            name = f"motor_{motor_id}"
            if name not in bus.motors:
                print(f"Motor ID {motor_id} not in bus")
                sys.exit(1)
            targets[name] = pos
    return targets


def main():
    parser = argparse.ArgumentParser(description="Move motors to target positions")
    parser.add_argument("targets", nargs="+", help="id:position or all:position")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--accel", type=int, default=20)
    parser.add_argument("--step", type=int, default=20)
    parser.add_argument("--scan-start", type=int, default=1)
    parser.add_argument("--scan-end", type=int, default=30)
    args = parser.parse_args()

    # Build motors dict by scanning the bus
    import os, importlib.util
    _spec = importlib.util.spec_from_file_location("motors", os.path.join(os.path.dirname(__file__), "motors.py"))
    _mod = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(_mod)
    found = _mod.probe_scan_ids(args.port)
    if not found:
        print(f"No motors found on {args.port}")
        sys.exit(1)

    motors = {
        f"motor_{mid}": Motor(id=mid, model=model, norm_mode=MotorNormMode.RANGE_M100_100)
        for mid, model in found.items()
    }
    print(f"Found motors: {[f'{n}(id={m.id})' for n, m in motors.items()]}")

    bus = FeetechMotorsBus(port=args.port, motors=motors)
    bus.connect(handshake=False)

    targets = parse_targets(args.targets, bus)
    motor_names = list(targets.keys())

    print(f"Targets: {targets}")

    # Set position mode + acceleration
    for name in motor_names:
        bus.write("Torque_Enable", name, 0, normalize=False)
        time.sleep(0.01)
        bus.write("Operating_Mode", name, OperatingMode.POSITION.value, normalize=False)
        time.sleep(0.01)
        bus.write("Maximum_Acceleration", name, args.accel, normalize=False)
        time.sleep(0.01)
        bus.write("Torque_Enable", name, 1, normalize=False)
        time.sleep(0.01)

    # Interpolate
    step = args.step
    while True:
        cur = bus.sync_read("Present_Position", motor_names, normalize=False)
        goal = {}
        done = True
        for name in motor_names:
            diff = targets[name] - cur[name]
            if abs(diff) <= step:
                goal[name] = targets[name]
            else:
                goal[name] = cur[name] + (step if diff > 0 else -step)
                done = False
        bus.sync_write("Goal_Position", goal, normalize=False)
        if done:
            break
        time.sleep(0.02)

    # Verify
    time.sleep(0.5)
    final = bus.sync_read("Present_Position", motor_names, normalize=False)
    for name in motor_names:
        print(f"  {name}: {final[name]} (target {targets[name]})")

    # Disable torque
    for name in motor_names:
        bus.write("Torque_Enable", name, 0, normalize=False)
        time.sleep(0.01)

    print("Done.")
    bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    main()
