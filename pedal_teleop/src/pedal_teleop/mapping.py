"""Pedal IMU -> base body-velocity + lift command.

Each pedal is treated as a 2-axis analog stick whose value is the
direction gravity has rotated relative to the captured rest pose. The
two pedals combine tank-style:

    x_vel  (forward) = mean(pitch_L, pitch_R)
    y_vel  (strafe)  = mean(roll_L,  roll_R)
    theta  (rotate)  = (pitch_R - pitch_L) / 2
    lift             = both-outward => up, both-inward => down, else stop

Sign conventions on x_vel / y_vel / theta_vel match
`viser_control.body_to_wheel_raw` so the wheels behave identically to
the existing gamepad path of the same magnitude.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .wire import LiftCmd


# Mirror viser_control.py constants exactly.
LIN_SPEED = 0.2     # m/s at full tilt
ANG_SPEED = 80.0    # deg/s at full tilt


@dataclass
class PedalCal:
    """Per-pedal calibration: rest gravity + axis-sign convention."""

    g_rest: np.ndarray              # 3-vector, body frame
    axis_sign_forward: int = +1
    axis_sign_right: int = +1


def project_tilt(accel: np.ndarray, cal: PedalCal) -> tuple[float, float]:
    """Project a raw accel reading onto user-frame (right, forward) tilt in g."""
    delta = accel - cal.g_rest
    # body +X -> forward, body +Y -> right (mounting convention).
    forward = cal.axis_sign_forward * delta[0]
    right = cal.axis_sign_right * delta[1]
    return right, forward


def _deadband(v: float, dz: float) -> float:
    if v > dz:
        return v - dz
    if v < -dz:
        return v + dz
    return 0.0


def compute_base_cmd(
    accel_L: np.ndarray,
    accel_R: np.ndarray,
    cal_L: PedalCal,
    cal_R: PedalCal,
    deadzone: float,
    full_scale_tilt: float,
    lift_deadzone: float,
) -> tuple[float, float, float, LiftCmd]:
    """Return (x_vel, y_vel, theta_vel, lift) ready for the wire packet."""
    rL_raw, fL_raw = project_tilt(accel_L, cal_L)
    rR_raw, fR_raw = project_tilt(accel_R, cal_R)

    # Apply deadzone per-axis per-pedal before combining, so a still pedal
    # doesn't contaminate the other pedal's signal. Keep the raw values too;
    # the lift AND-gate uses a larger, separate deadzone applied to raw tilt.
    rL = _deadband(rL_raw, deadzone)
    fL = _deadband(fL_raw, deadzone)
    rR = _deadband(rR_raw, deadzone)
    fR = _deadband(fR_raw, deadzone)

    # Normalize tilt-in-g to a unit-ish [-1, +1] command. Clip to avoid
    # commanding past full scale if someone really stomps a pedal.
    def norm(v: float) -> float:
        n = v / full_scale_tilt
        return max(-1.0, min(1.0, n))

    nfL, nfR = norm(fL), norm(fR)
    nrL, nrR = norm(rL), norm(rR)

    # Combined body command (signs chosen to match viser_control's gamepad path:
    # in body_to_wheel_raw the vector is [-x, -y, theta], and gamepad maps
    # axes[1] (UD, +down) -> x_cmd and axes[0] (LR, +right) -> y_cmd. We want:
    #   tilt-forward -> drive forward (axes[1] = -1 = stick-up). So x_vel
    #   positive = forward means we negate the pitch sum to align with the
    #   existing convention used downstream.
    #
    # In practice it's easier to keep x/y/theta in *physical* units and let
    # body_to_wheel_raw consume them: it's the same function the gamepad
    # path feeds, so as long as we say "+x = forward in m/s" we're good.
    # Sign of y_vel matches viser_control's gamepad path: there, axes[0]
    # stick-left (negative) becomes y_cmd positive, so y_cmd > 0 means
    # "strafe left." Negate the pedal sum so "both pedals rolled left"
    # produces y_vel > 0 too.
    x_vel = LIN_SPEED * (nfL + nfR) / 2.0      # +forward
    y_vel = -LIN_SPEED * (nrL + nrR) / 2.0     # +strafe-left
    theta_vel = ANG_SPEED * (nfR - nfL) / 2.0  # +CCW (rotate-left)

    # Lift: literal AND, with a larger dedicated deadband applied to the
    # *raw* projected tilt. Lift would otherwise fire from any incidental
    # outward/inward roll during normal driving, so require both pedals to
    # be tilted decisively in matching directions before committing.
    if rL_raw < -lift_deadzone and rR_raw > lift_deadzone:
        lift: LiftCmd = "up"      # both outward
    elif rL_raw > lift_deadzone and rR_raw < -lift_deadzone:
        lift = "down"             # both inward
    else:
        lift = "stop"

    return x_vel, y_vel, theta_vel, lift
