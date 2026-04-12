"""Interactive calibration for single-bus dual-arm setup. Run on the robot host."""

import logging
from lerobot.robots.alohamini.lekiwi import LeKiwi
from lerobot.robots.alohamini.config_lekiwi import LeKiwiConfig

logging.basicConfig(level=logging.INFO)

r = LeKiwi(LeKiwiConfig())

# Connect with handshake=False to avoid firmware check issues on crowded bus
r.left_bus.connect(handshake=False)
print(f"Connected to {r.left_bus.port}")
print(f"Left arm motors:  {r.left_arm_motors}")
print(f"Right arm motors: {r.right_arm_motors}")
print(f"Base motors:      {r.base_motors}")

# Run calibration (interactive — will prompt for arm positioning)
r.calibrate()

r.left_bus.disconnect(disable_torque=False)
print("Calibration complete, disconnected.")
