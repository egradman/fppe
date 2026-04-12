"""Smoke test: connect, read positions, wiggle each arm motor +100/-100 ticks, return."""

import time
from lerobot.robots.alohamini.lekiwi import LeKiwi
from lerobot.robots.alohamini.config_lekiwi import LeKiwiConfig
from lerobot.motors.feetech import OperatingMode

WIGGLE = 100  # ticks (~8.8 degrees)

r = LeKiwi(LeKiwiConfig())
bus = r.left_bus
bus.connect(handshake=False)

arm_motors = r.left_arm_motors + r.right_arm_motors
print(f"Arm motors: {arm_motors}")

# Set position mode + enable torque
for name in arm_motors:
    bus.write("Torque_Enable", name, 0, normalize=False)
    time.sleep(0.01)
    bus.write("Operating_Mode", name, OperatingMode.POSITION.value, normalize=False)
    time.sleep(0.01)
    bus.write("Torque_Enable", name, 1, normalize=False)
    time.sleep(0.01)

# Read starting positions
start_pos = bus.sync_read("Present_Position", arm_motors, normalize=False)
print("\nStarting positions:")
for name, val in start_pos.items():
    print(f"  {name}: {val}")

# Wiggle +
print(f"\nWiggling +{WIGGLE} ticks...")
bus.sync_write("Goal_Position", {name: start_pos[name] + WIGGLE for name in arm_motors}, normalize=False)
time.sleep(1.0)

cur = bus.sync_read("Present_Position", arm_motors, normalize=False)
for name in arm_motors:
    print(f"  {name}: {start_pos[name]} -> {cur[name]}")

# Wiggle -
print(f"\nWiggling -{WIGGLE} ticks...")
bus.sync_write("Goal_Position", {name: start_pos[name] - WIGGLE for name in arm_motors}, normalize=False)
time.sleep(1.0)

cur = bus.sync_read("Present_Position", arm_motors, normalize=False)
for name in arm_motors:
    print(f"  {name}: {start_pos[name]} -> {cur[name]}")

# Return to start
print("\nReturning to start...")
bus.sync_write("Goal_Position", {name: start_pos[name] for name in arm_motors}, normalize=False)
time.sleep(1.0)

# Disable torque
for name in arm_motors:
    bus.write("Torque_Enable", name, 0, normalize=False)
    time.sleep(0.01)

print("Done.")
bus.disconnect(disable_torque=False)
