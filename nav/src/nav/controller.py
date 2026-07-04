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


@dataclass
class ApproachParams:
    goal_dist: float = 2.0        # hard-stop distance
    decel_range: float = 3.0      # ease speed over this band above goal_dist
    min_speed_frac: float = 0.3   # creep-speed floor near the goal
    overshoot_margin: float = 0.6 # stop when dist rises this far above its min


class NavPlanner:
    """Stateful per-goal planner shared by the standalone executor and the
    service. Turns each (distance, waypoints) inference into a body velocity with
    approach deceleration, and decides when the goal is reached.

    Reached = distance crossed `goal_dist`, OR (once inside the approach band) the
    distance has risen `overshoot_margin` above its running minimum — the
    closest-approach detector, robust to ViNT's distance head plateauing above
    `goal_dist`.
    """

    def __init__(self, base_speed: float, gains: ControlGains, approach: ApproachParams):
        self.base_speed = base_speed
        self.gains = gains
        self.approach = approach
        self.min_dist = float("inf")
        self.engaged = False

    def reset(self):
        self.min_dist = float("inf")
        self.engaged = False

    def step(self, dist: float, action) -> tuple[tuple[float, float, float], bool, str]:
        """Return ((x_vel, y_vel, theta_vel), reached, reason)."""
        a = self.approach
        self.min_dist = min(self.min_dist, dist)
        if dist <= a.goal_dist + a.decel_range:
            self.engaged = True

        if dist < a.goal_dist:
            return (0.0, 0.0, 0.0), True, f"dist {dist:.2f} < goal_dist {a.goal_dist}"
        if self.engaged and dist > self.min_dist + a.overshoot_margin:
            return (0.0, 0.0, 0.0), True, (
                f"passed closest approach (min {self.min_dist:.2f}, now {dist:.2f})"
            )

        span = max(a.decel_range, 1e-6)
        frac = max(a.min_speed_frac, min(1.0, (dist - a.goal_dist) / span))
        self.gains.cruise_speed = self.base_speed * frac
        return waypoints_to_velocity(action, self.gains), False, "driving"
