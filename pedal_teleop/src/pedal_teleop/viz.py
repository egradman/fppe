"""ncurses visualizer for pedal_teleop.

Shows per-pedal raw accel, user-frame tilt, and the resulting body-velocity
command, refreshing in place. Useful for sanity-checking axis signs and
deadband without staring at JSON.
"""

from __future__ import annotations

import curses
from dataclasses import dataclass

import numpy as np

from .mapping import ANG_SPEED, LIN_SPEED


@dataclass
class VizFrame:
    raw_L: np.ndarray | None
    raw_R: np.ndarray | None
    tilt_L: tuple[float, float]   # (right_g, forward_g)
    tilt_R: tuple[float, float]
    x_vel: float                  # m/s
    y_vel: float                  # m/s
    theta_vel: float              # deg/s
    lift: str
    seq: int
    hz: float
    stale: bool


def _bar(value: float, full_scale: float, half_width: int = 14) -> str:
    """Two-sided horizontal bar, centred at zero."""
    if full_scale <= 0:
        return "[" + " " * (2 * half_width + 1) + "]"
    n = int(round(half_width * max(-1.0, min(1.0, value / full_scale))))
    if n >= 0:
        left = "." * half_width
        right = "#" * n + "." * (half_width - n)
    else:
        left = "." * (half_width + n) + "#" * (-n)
        right = "." * half_width
    return f"[{left}|{right}]"


class CursesVisualizer:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        stdscr.nodelay(True)
        try:
            curses.curs_set(0)
        except curses.error:
            pass
        stdscr.clear()

    def _safe_addstr(self, row: int, col: int, s: str) -> None:
        try:
            self.stdscr.addstr(row, col, s)
        except curses.error:
            pass  # off the edge of a tiny terminal — skip

    def draw(self, f: VizFrame) -> None:
        s = self.stdscr
        s.erase()
        row = 0
        self._safe_addstr(row, 0, "pedal_teleop visualizer   (Ctrl-C to quit)")
        row += 2

        self._safe_addstr(row, 0, f"          {'LEFT pedal':<36}{'RIGHT pedal'}")
        row += 1

        def fmt3(v: np.ndarray | None) -> str:
            if v is None:
                return "(no data)"
            return f"ax={v[0]:+.3f} ay={v[1]:+.3f} az={v[2]:+.3f}"

        self._safe_addstr(row, 0, f"  raw     {fmt3(f.raw_L):<36}{fmt3(f.raw_R)}")
        row += 1
        self._safe_addstr(
            row, 0,
            f"  tilt    right={f.tilt_L[0]:+.3f} fwd={f.tilt_L[1]:+.3f}"
            f"        right={f.tilt_R[0]:+.3f} fwd={f.tilt_R[1]:+.3f}",
        )
        row += 2

        self._safe_addstr(row, 0, "Output body command")
        row += 1
        self._safe_addstr(
            row, 2,
            f"x_vel   {f.x_vel:+6.3f} m/s     {_bar(f.x_vel, LIN_SPEED)}",
        )
        row += 1
        self._safe_addstr(
            row, 2,
            f"y_vel   {f.y_vel:+6.3f} m/s     {_bar(f.y_vel, LIN_SPEED)}",
        )
        row += 1
        self._safe_addstr(
            row, 2,
            f"theta   {f.theta_vel:+6.1f} deg/s   {_bar(f.theta_vel, ANG_SPEED)}",
        )
        row += 1
        lift_marker = {"up": "^^", "down": "vv", "stop": "  "}.get(f.lift, "??")
        self._safe_addstr(row, 2, f"lift    {f.lift:<10} {lift_marker}")
        row += 2

        status = "STALE" if f.stale else "OK   "
        self._safe_addstr(
            row, 0,
            f"seq {f.seq}   |   {f.hz:5.1f} Hz   |   pedals: {status}",
        )
        s.refresh()
