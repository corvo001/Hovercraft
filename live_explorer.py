
"""
Hovercraft Live Explorer
------------------------
Explora cómo cambian las dinámicas de un aerodeslizador al variar parámetros físicos
en tiempo real. Incluye un modelo de "lift" (sustentación) simplificado para ver la
relación empuje vertical vs peso y su efecto en la fricción horizontal.

Requisitos: numpy, matplotlib
Ejecución:  python live_explorer.py

Controles:
- Sliders: masa m, inercia J, separación b, F_max, Cd_u, Cd_v, Cd_r, Lift_max, mu_ground
- "Wind [N]": fuerza lateral constante (viento) en el eje y-mundo
- "Controller": on/off. Si OFF, puedes inyectar fuerzas manuales con los sliders F_L/R bias.
- Botones: Reset, Pause
- Click en el plano para mover el waypoint objetivo.
"""
from __future__ import annotations
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons
from matplotlib.animation import FuncAnimation

# ---------------- Dynamics ----------------
class HoverParams:
    def __init__(self):
        self.m = 25.0
        self.J = 2.2
        self.b = 0.6
        self.Cd_u = 6.0
        self.Cd_v = 8.0
        self.Cd_r = 1.2
        self.F_max = 60.0
        self.dt = 0.02

        # Lift model
        self.Lift_max = 400.0  # N of total vertical lift capacity (fan)
        self.mu_ground = 10.0  # extra horizontal damping when no lift (ground contact)

        # Disturbance
        self.wind = 0.0  # N, +y direction in world

class HoverSim:
    def __init__(self, p: HoverParams):
        self.p = p
        self.reset()

    def reset(self):
        self.x = np.array([0.,0., 0., 0.,0., 0.])  # x,y,theta,vx,vy,omega
        self.lift_cmd = 0.8  # 0..1 fraction of Lift_max

    def step(self, u):
        p = self.p
        # Inputs
        FL = np.clip(u[0], -p.F_max, p.F_max)
        FR = np.clip(u[1], -p.F_max, p.F_max)

        x,y,th,vx,vy,r = self.x
        c,s = np.cos(th), np.sin(th)

        # world->body
        vb = np.array([[ c, s],[-s, c]]) @ np.array([vx,vy])
        u_b, v_b = vb[0], vb[1]

        # Drag
        Xd = -p.Cd_u*u_b
        Yd = -p.Cd_v*v_b
        Nd = -p.Cd_r*r

        # Thrusters
        Xt = FL + FR
        Nt = (FR - FL)*(p.b/2.0)

        # Lift model → modulates extra ground damping
        L = self.lift_cmd * p.Lift_max
        W = p.m * 9.81
        lift_margin = (L - W)/max(W,1e-6)  # >0 cushion strong
        extra_damp = p.mu_ground * max(0.0, -lift_margin)  # only when lift < weight
        # Extra damping acts in body frame proportional to velocity
        Xg = -extra_damp*u_b
        Yg = -extra_damp*v_b

        # Sum body forces
        Fx_b = Xd + Xg + Xt
        Fy_b = Yd + Yg  # no lateral thruster
        Nr   = Nd + Nt

        # Body accels
        ax_b = Fx_b/p.m
        ay_b = Fy_b/p.m
        ar   = Nr/p.J

        # Back to world
        aw = np.array([[ c,-s],[s,c]]) @ np.array([ax_b, ay_b])
        aw[1] += p.wind/p.m  # y-world disturbance

        # Integrate
        r += p.dt*ar
        th += p.dt*r
        vx += p.dt*aw[0]
        vy += p.dt*aw[1]
        x  += p.dt*vx
        y  += p.dt*vy
        self.x = np.array([x,y,th,vx,vy,r])
        return self.x

# ---------------- Controller ----------------
class PID:
    def __init__(self,kp,ki,kd,dt,umin,umax):
        self.kp=kp;self.ki=ki;self.kd=kd;self.dt=dt;self.umin=umin;self.umax=umax
        self.i=0; self.e_prev=0
    def reset(self): self.i=0; self.e_prev=0
    def step(self,e):
        self.i += e*self.dt
        d = (e - self.e_prev)/self.dt
        self.e_prev = e
        u = self.kp*e + self.ki*self.i + self.kd*d
        return np.clip(u, self.umin, self.umax)

class Controller:
    def __init__(self,p:HoverParams):
        self.speed = PID(18,4,2.5,p.dt,-2*p.F_max,2*p.F_max)
        self.heading= PID(60,0,8,p.dt,-p.F_max,p.F_max)
        self.F_max = p.F_max
    def set_dt(self,dt):
        self.speed.dt = dt; self.heading.dt = dt
    def compute(self, v_ref, v_now, psi_ref, psi_now, biasL=0, biasR=0):
        epsi = (psi_ref - psi_now + np.pi)%(2*np.pi)-np.pi
        u_spd = self.speed.step(v_ref - v_now)
        u_yaw = self.heading.step(epsi)
        F_total = np.clip(u_spd, -2*self.F_max, 2*self.F_max)
        F_diff  = np.clip(u_yaw, -self.F_max, self.F_max)
        FL = np.clip(0.5*(F_total - F_diff)+biasL, -self.F_max, self.F_max)
        FR = np.clip(0.5*(F_total + F_diff)+biasR, -self.F_max, self.F_max)
        return FL,FR

# ---------------- UI / Animation ----------------
def main():
    p = HoverParams()
    sim = HoverSim(p)
    ctrl = Controller(p)
    running = True
    use_ctrl = True
    waypoint = np.array([6.0,4.0])
    biasL = 0.0; biasR = 0.0

    # Figure and axes
    fig = plt.figure(figsize=(10,7))
    ax = fig.add_axes([0.05,0.35,0.6,0.6])
    ax.set_title("Plano XY (clic para mover waypoint)")
    ax.set_aspect('equal','box')
    ax.set_xlim(-2,10); ax.set_ylim(-2,6)
    traj_line, = ax.plot([],[],'-',lw=2)
    craft_pt,  = ax.plot([],[],'o',ms=6)
    wp_pt,     = ax.plot([waypoint[0]],[waypoint[1]],'o--',ms=6)

    # Live readouts
    txt = ax.text(0.02,0.98,"", transform=ax.transAxes, va='top', fontsize=10)

    # Sliders panel
    ax_m   = fig.add_axes([0.72,0.84,0.25,0.03])
    ax_J   = fig.add_axes([0.72,0.80,0.25,0.03])
    ax_b   = fig.add_axes([0.72,0.76,0.25,0.03])
    ax_F   = fig.add_axes([0.72,0.72,0.25,0.03])
    ax_Cu  = fig.add_axes([0.72,0.68,0.25,0.03])
    ax_Cv  = fig.add_axes([0.72,0.64,0.25,0.03])
    ax_Cr  = fig.add_axes([0.72,0.60,0.25,0.03])
    ax_L   = fig.add_axes([0.72,0.56,0.25,0.03])
    ax_mu  = fig.add_axes([0.72,0.52,0.25,0.03])
    ax_wind= fig.add_axes([0.72,0.48,0.25,0.03])
    ax_biasL=fig.add_axes([0.72,0.44,0.25,0.03])
    ax_biasR=fig.add_axes([0.72,0.40,0.25,0.03])

    s_m   = Slider(ax_m,   "m [kg]",  2, 80, valinit=p.m)
    s_J   = Slider(ax_J,   "J [kgm²]",0.2,20, valinit=p.J)
    s_b   = Slider(ax_b,   "b [m]",   0.2, 1.2, valinit=p.b)
    s_F   = Slider(ax_F,   "F_max [N]",10,200, valinit=p.F_max)
    s_Cu  = Slider(ax_Cu,  "Cd_u", 0.1, 30, valinit=p.Cd_u)
    s_Cv  = Slider(ax_Cv,  "Cd_v", 0.1, 30, valinit=p.Cd_v)
    s_Cr  = Slider(ax_Cr,  "Cd_r", 0.05, 8, valinit=p.Cd_r)
    s_L   = Slider(ax_L,   "Lift_max [N]", 50, 1200, valinit=p.Lift_max)
    s_mu  = Slider(ax_mu,  "mu_ground", 0.0, 40, valinit=p.mu_ground)
    s_wind= Slider(ax_wind,"Wind [N]", -60, 60, valinit=p.wind)
    s_biasL=Slider(ax_biasL,"Bias FL [N]", -100,100, valinit=0.0)
    s_biasR=Slider(ax_biasR,"Bias FR [N]", -100,100, valinit=0.0)

    # Buttons
    bax_reset = fig.add_axes([0.72,0.32,0.12,0.05])
    bax_pause = fig.add_axes([0.85,0.32,0.12,0.05])
    btn_reset = Button(bax_reset, "Reset")
    btn_pause = Button(bax_pause, "Pause")

    # Checkbox controller on/off
    cax = fig.add_axes([0.72,0.23,0.25,0.07])
    cb = CheckButtons(cax, ["Controller ON"], [True])

    # Secondary plot: speed and forces (compact)
    ax2 = fig.add_axes([0.05,0.08,0.6,0.2])
    ax2.set_title("Velocidad (m/s) y fuerzas (N)")
    v_line, = ax2.plot([],[],label="v")
    FL_line,= ax2.plot([],[],label="FL")
    FR_line,= ax2.plot([],[],label="FR")
    ax2.legend(loc="upper right")

    # Event handlers
    traj = []
    t_hist = []
    v_hist = []
    FL_hist = []
    FR_hist = []

    def on_click(event):
        nonlocal waypoint
        if event.inaxes is ax:
            waypoint = np.array([event.xdata, event.ydata])
            wp_pt.set_data([waypoint[0]],[waypoint[1]])
            fig.canvas.draw_idle()

    def on_reset(event):
        sim.reset()
        traj.clear(); t_hist.clear(); v_hist.clear(); FL_hist.clear(); FR_hist.clear()

    def on_pause(event):
        nonlocal running
        running = not running
        btn_pause.label.set_text("Run" if not running else "Pause")

    def on_check(label):
        nonlocal use_ctrl
        use_ctrl = not use_ctrl

    cid = fig.canvas.mpl_connect("button_press_event", on_click)
    btn_reset.on_clicked(on_reset)
    btn_pause.on_clicked(on_pause)
    cb.on_clicked(on_check)

    # Animation loop
    t = 0.0
    def update(frame):
        nonlocal t, biasL, biasR
        # Update params from sliders
        p.m = s_m.val; p.J = s_J.val; p.b = s_b.val; p.F_max = s_F.val
        p.Cd_u = s_Cu.val; p.Cd_v = s_Cv.val; p.Cd_r = s_Cr.val
        p.Lift_max = s_L.val; p.mu_ground = s_mu.val; p.wind = s_wind.val
        biasL = s_biasL.val; biasR = s_biasR.val

        # Reference
        pos = sim.x[:2]; th = sim.x[2]; v = np.hypot(sim.x[3], sim.x[4])
        vec = waypoint - pos
        psi_ref = np.arctan2(vec[1],vec[0])
        v_ref = np.clip(np.linalg.norm(vec), 0.0, 4.0)

        if use_ctrl:
            FL, FR = ctrl.compute(v_ref, v, psi_ref, th, biasL, biasR)
        else:
            # Manual forces from bias only
            FL = np.clip(biasL, -p.F_max, p.F_max)
            FR = np.clip(biasR, -p.F_max, p.F_max)

        if running:
            sim.step([FL,FR])
            t += p.dt

        # Logs
        traj.append(sim.x[:2].copy())
        t_hist.append(t); v_hist.append(v); FL_hist.append(FL); FR_hist.append(FR)

        # Draw
        if len(traj)>1:
            data = np.array(traj)
            traj_line.set_data(data[:,0], data[:,1])
        craft_pt.set_data([sim.x[0]],[sim.x[1]])
        wp_pt.set_data([waypoint[0]],[waypoint[1]])

        txt.set_text(f"x={sim.x[0]:.2f}  y={sim.x[1]:.2f}  v={v:.2f} m/s\n"
                     f"theta={np.degrees(sim.x[2])%360:5.1f}°  r={np.degrees(sim.x[5]):.1f}°/s\n"
                     f"Lift: {sim.lift_cmd*p.Lift_max:.0f} N  Weight: {p.m*9.81:.0f} N")

        # Secondary plot
        ax2.set_xlim(max(0,t-20), t+0.1)
        ax2.set_ylim(-1.1*max(1, p.F_max), 1.1*max(1, p.F_max))
        v_line.set_data(t_hist, v_hist)
        FL_line.set_data(t_hist, FL_hist)
        FR_line.set_data(t_hist, FR_hist)

        return traj_line, craft_pt, wp_pt, v_line, FL_line, FR_line, txt

    ani = FuncAnimation(fig, update, interval=int(p.dt*1000), blit=False)
    plt.show()

if __name__ == "__main__":
    main()
