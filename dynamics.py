
"""
Simple 2D hovercraft dynamics for simulation and control prototyping.

State vector:
    x = [x, y, theta, vx, vy, omega]^T
Inputs:
    u = [F_L, F_R]  (left/right thruster forces in Newtons)

Assumptions:
- Flat plane, cushion provides near-constant lift; vertical dynamics neglected.
- Linear aerodynamic/hydrodynamic drag on body axes.
- Thrusters mounted symmetrically at +/- b/2 lateral offset from centerline, aligned with x-body.
- No wheel contact; low Coulomb friction approximated as linear damping.
- Small-angle leakage model folded into effective mass/drag parameters.

Author: Generated for Cuervo's hovercraft project.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np

@dataclass
class HovercraftParams:
    m: float = 25.0          # mass [kg]
    J: float = 2.2           # yaw inertia [kg m^2]
    b: float = 0.6           # thruster lateral separation [m]
    Cd_u: float = 6.0        # linear drag coefficient in surge [N/(m/s)]
    Cd_v: float = 8.0        # linear drag coefficient in sway [N/(m/s)]
    Cd_r: float = 1.2        # yaw damping [N m /(rad/s)]
    F_max: float = 60.0      # max force per thruster [N]
    dt: float = 0.01         # integration step [s]

def clip(u, lo, hi):
    return np.minimum(np.maximum(u, lo), hi)

class HovercraftSim:
    def __init__(self, p: HovercraftParams):
        self.p = p
        self.x = np.zeros(6)  # [x, y, theta, vx, vy, omega]

    def reset(self, x0=None):
        self.x = np.zeros(6) if x0 is None else np.array(x0, dtype=float)

    def step(self, u):
        p = self.p
        # Saturate forces
        F_L = float(clip(u[0], -p.F_max, p.F_max))
        F_R = float(clip(u[1], -p.F_max, p.F_max))

        # Unpack state
        x, y, th, vx, vy, r = self.x

        # Body-frame velocities (already vx, vy in world? keep state in world)
        # Convert world vel to body vel for drag
        c, s = np.cos(th), np.sin(th)
        # rotation matrix world->body
        Rwb = np.array([[ c, s],
                        [-s, c]])
        vb = Rwb @ np.array([vx, vy])
        u_b = vb[0]; v_b = vb[1]

        # Drag in body frame
        X_drag = -p.Cd_u * u_b
        Y_drag = -p.Cd_v * v_b
        N_drag = -p.Cd_r * r

        # Thrust (body frame)
        X_thrust = F_L + F_R
        N_thrust = (F_R - F_L) * (p.b/2.0)

        # Sum forces/moments in body, then convert to world accel
        Fx_b = X_drag + X_thrust
        Fy_b = Y_drag  # no lateral thruster

        # Accel in body frame
        ax_b = Fx_b / p.m
        ay_b = Fy_b / p.m
        ar = (N_drag + N_thrust) / p.J

        # Convert body accels to world
        Rbw = np.array([[ c, -s],
                        [ s,  c]])
        aw = Rbw @ np.array([ax_b, ay_b])

        # Integrate (semi-implicit Euler)
        r_new = r + p.dt * ar
        vx_new = vx + p.dt * aw[0]
        vy_new = vy + p.dt * aw[1]
        th_new = th + p.dt * r_new
        x_new = x + p.dt * vx_new
        y_new = y + p.dt * vy_new

        self.x = np.array([x_new, y_new, th_new, vx_new, vy_new, r_new])
        return self.x.copy()
