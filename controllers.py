
"""
Basic controllers for hovercraft:
- PID speed and heading control → left/right thrust commands.
- Pure pursuit waypoint tracker with speed scheduling.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    u_min: float
    u_max: float
    dt: float
    integ: float = 0.0
    prev_e: float = 0.0

    def reset(self):
        self.integ = 0.0
        self.prev_e = 0.0

    def __call__(self, e: float):
        self.integ += e * self.dt
        d = (e - self.prev_e) / self.dt
        self.prev_e = e
        u = self.kp*e + self.ki*self.integ + self.kd*d
        return np.clip(u, self.u_min, self.u_max)

@dataclass
class SpeedHeadingController:
    pid_speed: PID
    pid_heading: PID
    F_max: float
    base_bias: float = 0.0  # to counter steady-state drag if needed

    def compute(self, v_ref: float, v_now: float, psi_ref: float, psi_now: float):
        # Normalize heading error to [-pi, pi]
        e_psi = (psi_ref - psi_now + np.pi) % (2*np.pi) - np.pi
        u_speed = self.pid_speed(v_ref - v_now)
        u_yaw = self.pid_heading(e_psi)

        # Map to left/right forces
        F_total = np.clip(self.base_bias + u_speed, -self.F_max*2, self.F_max*2)
        F_diff  = np.clip(u_yaw, -self.F_max, self.F_max)

        F_L = np.clip(0.5*(F_total - F_diff), -self.F_max, self.F_max)
        F_R = np.clip(0.5*(F_total + F_diff), -self.F_max, self.F_max)
        return float(F_L), float(F_R)

class PurePursuit:
    def __init__(self, waypoints, lookahead=0.8):
        self.waypoints = np.array(waypoints, dtype=float)
        self.lookahead = lookahead
        self.i = 0

    def target(self, pos):
        # advance until target is at least lookahead away
        while self.i < len(self.waypoints)-1:
            d = np.linalg.norm(self.waypoints[self.i] - pos)
            if d > self.lookahead:
                break
            self.i += 1
        return self.waypoints[self.i]
