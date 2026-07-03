"""Hardware identity and per-pedal axis convention.

Update LEFT_PORT / RIGHT_PORT after physically labelling the pedals.
Identify which serial belongs to which side by running pedal-teleop in
--echo-only mode with one pedal at rest and the other being tapped, and
seeing which side's accel values change.

Axis convention assumes the XIAO is mounted on the pedal with:
  - USB connector pointing AWAY from the user (forward)
  - Top face (silkscreen up) facing the sky

If a pedal is mounted differently, flip the matching AXIS_SIGN_* below;
no firmware reflash needed.
"""

# Stable USB by-id paths. The vendor string is "Seeed_XIAO_nRF52840_Sense"
# once our firmware is flashed (factory-state XIAOs show "Arduino_...");
# the suffix is the bootloader-burned serial number.
#
# The serial visible to Linux changes after a firmware flash, because the
# Adafruit-derived bootloader and our sketch present a different USB serial
# than the factory descriptor. Re-capture the by-id path with
#   ls -l /dev/serial/by-id/
# after flashing each pedal, and update the two lines below.
LEFT_PORT = "/dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_1F87D235E3130B78-if00"
RIGHT_PORT = "/dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_C8712E2269A97C91-if00"


# Mapping from IMU body axes to user-frame (forward, right) tilt.
# At rest, gravity reads ~+1 g along the body axis pointing "up out of
# the top face." With the mounting convention above, tilting the pedal
# toe-down (forward) rotates gravity toward body +X; rolling the pedal
# to the user's right rotates gravity toward body +Y.
#
# The signs below let you correct a pedal that's been mounted in a
# rotated or mirrored orientation. Empirically validate at first run
# (see pedal-teleop --echo-only).
AXIS_SIGN_FORWARD_LEFT = +1     # +1 if body +X = forward (toe-down)
AXIS_SIGN_RIGHT_LEFT = +1       # +1 if body +Y = roll-right

AXIS_SIGN_FORWARD_RIGHT = +1
AXIS_SIGN_RIGHT_RIGHT = +1


# Deadzone applied per-pedal in user-frame (g units) for wheel commands.
# A 0.10 g tilt corresponds to ~5.7° from level — small enough to be
# unintentional, large enough to feel like a deliberate press.
DEADZONE = 0.10

# Lift uses a much larger deadzone because the gate is "both pedals rolled
# in matching directions" and any normal driving roll would otherwise nudge
# the lift up or down. ~0.35 g ≈ 20° of deliberate outward/inward tilt.
LIFT_DEADZONE = 0.35


# Sanity bounds on captured rest gravity magnitude (in g). Outside these,
# pedal-teleop refuses to start so we don't fly with a foot-on-pedal zero.
REST_MAG_MIN = 0.95
REST_MAG_MAX = 1.05


# Full-scale tilt that maps to LIN_SPEED / ANG_SPEED in viser_control.
# A 0.5 g tilt (~30°) commands full speed. Anything beyond is clamped.
FULL_SCALE_TILT = 0.50
