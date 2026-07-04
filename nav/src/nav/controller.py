"""Turn ViNT waypoints into a body-frame velocity command.

ViNT emits `len_traj_pred` relative waypoints in the robot's local frame
(x = forward, y = left), as (dx, dy[, cos, sin]) deltas. We integrate them to a
cumulative path, pick a lookahead point, and command the holonomic base to move
toward it.

Because the LeKiwi base is holonomic we can **strafe straight** at the waypoint
(no need to rotate first), which keeps the controller a couple of proportional
terms. We add a gentle yaw term anyway so the base slowly faces the goal — that
keeps the target near the fisheye's center, where the lens distortion (and thus
ViNT's error) is smallest.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class ControlGains:
    cruise_speed: float = 0.12    # m/s along the waypoint direction (start slow)
    max_omega: float = 20.0       # deg/s cap on the yaw term
    heading_gain: float = 0.30    # deg/s per deg of heading error
    lookahead_idx: int = -1       # which cumulative waypoint to steer toward (-1 = horizon end)
    deadband: float = 1e-3        # ignore targets closer than this (normalized units)
    invert_y: bool = False        # flip strafe sign if the base strafes the wrong way
    invert_theta: bool = False    # flip yaw sign if the base rotates the wrong way


def waypoints_to_velocity(
    action: np.ndarray, gains: ControlGains
) -> tuple[float, float, float]:
    """(dx, dy, ...) waypoint deltas -> (x_vel, y_vel, theta_vel_deg).

    Returns body-frame velocities in the units the PedalPacket wire expects:
    x_vel/y_vel in m/s (+forward / +left), theta_vel in deg/s (+CCW/left).
    """
    xy = np.cumsum(action[:, :2], axis=0)          # cumulative local path
    tx, ty = float(xy[gains.lookahead_idx, 0]), float(xy[gains.lookahead_idx, 1])

    dist = math.hypot(tx, ty)
    if dist < gains.deadband:
        return 0.0, 0.0, 0.0

    # Strafe straight at the target: unit direction * cruise speed.
    ux, uy = tx / dist, ty / dist
    x_vel = gains.cruise_speed * ux
    y_vel = gains.cruise_speed * uy

    # Gentle yaw to keep the goal centered in the fisheye.
    heading_err_deg = math.degrees(math.atan2(ty, tx))
    theta_vel = max(
        -gains.max_omega,
        min(gains.max_omega, gains.heading_gain * heading_err_deg),
    )

    if gains.invert_y:
        y_vel = -y_vel
    if gains.invert_theta:
        theta_vel = -theta_vel

    return x_vel, y_vel, theta_vel
