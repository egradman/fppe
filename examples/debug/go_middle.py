"""Send all arm motors to midpoint (2048) slowly, then disable torque."""

import time
from lerobot.robots.alohamini.lekiwi import LeKiwi
from lerobot.robots.alohamini.config_lekiwi import LeKiwiConfig
from lerobot.motors.feetech import OperatingMode

MIDPOINT = 2048
STEP = 20        # ticks per step
INTERVAL = 0.02  # seconds between steps

r = LeKiwi(LeKiwiConfig())
bus = r.left_bus
bus.connect(handshake=False)

arm_motors = r.left_arm_motors + r.right_arm_motors

# Set position mode + low acceleration for smooth motion
for name in arm_motors:
    bus.write("Torque_Enable", name, 0, normalize=False)
    time.sleep(0.01)
    bus.write("Operating_Mode", name, OperatingMode.POSITION.value, normalize=False)
    time.sleep(0.01)
    bus.write("Maximum_Acceleration", name, 20, normalize=False)
    time.sleep(0.01)
    bus.write("Torque_Enable", name, 1, normalize=False)
    time.sleep(0.01)

cur = bus.sync_read("Present_Position", arm_motors, normalize=False)
print("Current positions:")
for name, val in cur.items():
    print(f"  {name}: {val}")

print(f"\nMoving to midpoint ({MIDPOINT})...")

# Interpolate toward midpoint
done = False
while not done:
    cur = bus.sync_read("Present_Position", arm_motors, normalize=False)
    goal = {}
    done = True
    for name in arm_motors:
        pos = cur[name]
        diff = MIDPOINT - pos
        if abs(diff) <= STEP:
            goal[name] = MIDPOINT
        else:
            goal[name] = pos + (STEP if diff > 0 else -STEP)
            done = False
    bus.sync_write("Goal_Position", goal, normalize=False)
    time.sleep(INTERVAL)

print("At midpoint.")

# Read final positions
final = bus.sync_read("Present_Position", arm_motors, normalize=False)
for name, val in final.items():
    print(f"  {name}: {val}")

# Disable torque
for name in arm_motors:
    bus.write("Torque_Enable", name, 0, normalize=False)
    time.sleep(0.01)

print("Torque disabled.")
bus.disconnect(disable_torque=False)
