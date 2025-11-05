
from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt

from dynamics import HovercraftParams, HovercraftSim
from controllers import PID, SpeedHeadingController, PurePursuit

def run_demo():
    p = HovercraftParams()
    sim = HovercraftSim(p)
    sim.reset([0,0,0, 0,0,0])

    # Controllers
    pid_speed = PID(kp=18, ki=4, kd=2.5, u_min=-2*p.F_max, u_max=2*p.F_max, dt=p.dt)
    pid_heading = PID(kp=60, ki=0.0, kd=8.0, u_min=-p.F_max, u_max=p.F_max, dt=p.dt)
    ctrl = SpeedHeadingController(pid_speed, pid_heading, F_max=p.F_max)

    # Path: rectangle
    waypoints = np.array([[0,0],[6,0],[6,4],[0,4],[0,0]])
    tracker = PurePursuit(waypoints, lookahead=0.7)

    T = 60.0
    N = int(T/p.dt)
    log = {'x':[], 'y':[], 'th':[], 'vx':[], 'vy':[], 'r':[], 'FL':[], 'FR':[], 'wx':[], 'wy':[]}

    for k in range(N):
        state = sim.x
        pos = state[0:2]
        th = state[2]
        v = np.hypot(state[3], state[4])

        tgt = tracker.target(pos)
        vec = tgt - pos
        psi_ref = np.arctan2(vec[1], vec[0])
        v_ref = np.clip(np.linalg.norm(vec), 0.0, 4.0)  # slow near corners

        FL, FR = ctrl.compute(v_ref, v, psi_ref, th)
        sim.step([FL, FR])

        log['x'].append(sim.x[0]); log['y'].append(sim.x[1]); log['th'].append(sim.x[2])
        log['vx'].append(sim.x[3]); log['vy'].append(sim.x[4]); log['r'].append(sim.x[5])
        log['FL'].append(FL); log['FR'].append(FR)
        log['wx'].append(tgt[0]); log['wy'].append(tgt[1])

    for k in log: log[k] = np.array(log[k])

    # Trajectory plot
    plt.figure()
    plt.plot(log['x'], log['y'], linewidth=2)
    plt.plot(waypoints[:,0], waypoints[:,1], linestyle='--', marker='o')
    plt.axis('equal')
    plt.title('Trayectoria vs Waypoints')
    plt.xlabel('x [m]'); plt.ylabel('y [m]')
    plt.tight_layout()
    plt.savefig('trajectory.png', dpi=180)

    # Speed plot
    plt.figure()
    speed = np.hypot(log['vx'], log['vy'])
    plt.plot(np.arange(len(speed))*0.01, speed, linewidth=2)
    plt.title('Velocidad')
    plt.xlabel('t [s]'); plt.ylabel('v [m/s]')
    plt.tight_layout()
    plt.savefig('speed.png', dpi=180)

    # Inputs plot
    plt.figure()
    plt.plot(np.arange(len(log['FL']))*0.01, log['FL'], linewidth=1.5)
    plt.plot(np.arange(len(log['FR']))*0.01, log['FR'], linewidth=1.5)
    plt.title('Fuerzas de los propulsores')
    plt.xlabel('t [s]'); plt.ylabel('F [N]')
    plt.tight_layout()
    plt.savefig('inputs.png', dpi=180)

    return log

if __name__ == '__main__':
    run_demo()
