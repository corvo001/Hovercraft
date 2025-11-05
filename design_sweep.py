
"""
Design Sweep for Hovercraft Parameters
-------------------------------------
Explora una rejilla de parámetros (masa m y empuje máximo F_max) y calcula métricas de seguimiento.
Genera CSV de resultados y mapas de calor (PNG).

Uso:
    python design_sweep.py --out out_sweep --m 5 40 8 --f 20 160 8
"""
from __future__ import annotations
import argparse, math, csv
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt

# ---- Modelo y control (idénticos al live) ----
@dataclass
class HoverParams:
    m: float = 25.0
    J: float = 2.2
    b: float = 0.6
    Cd_u: float = 6.0
    Cd_v: float = 8.0
    Cd_r: float = 1.2
    F_max: float = 60.0
    dt: float = 0.02
    Lift_max: float = 400.0
    mu_ground: float = 10.0
    wind: float = 0.0

class HoverSim:
    def __init__(self, p: HoverParams):
        self.p = p
        self.x = np.zeros(6)

    def reset(self):
        self.x[:] = 0.0

    def step(self, u):
        p = self.p
        FL = float(np.clip(u[0], -p.F_max, p.F_max))
        FR = float(np.clip(u[1], -p.F_max, p.F_max))
        x,y,th,vx,vy,r = self.x
        c,s = np.cos(th), np.sin(th)
        vb = np.array([[ c, s],[-s, c]]) @ np.array([vx,vy])
        u_b, v_b = vb

        Xd = -p.Cd_u*u_b; Yd=-p.Cd_v*v_b; Nd=-p.Cd_r*r
        Xt = FL+FR; Nt=(FR-FL)*(p.b/2.0)
        # Lift/friction coupling
        L = p.Lift_max; W = p.m*9.81
        extra = p.mu_ground * max(0.0, (W-L)/max(W,1e-6))
        Xg = -extra*u_b; Yg=-extra*v_b

        Fx_b = Xd+Xg+Xt; Fy_b = Yd+Yg; Nr = Nd+Nt
        ax_b = Fx_b/p.m; ay_b = Fy_b/p.m; ar = Nr/p.J
        aw = np.array([[ c,-s],[s,c]]) @ np.array([ax_b,ay_b])
        aw[1] += p.wind/p.m

        r += p.dt*ar; th += p.dt*r
        vx += p.dt*aw[0]; vy += p.dt*aw[1]
        x  += p.dt*vx;    y  += p.dt*vy
        self.x = np.array([x,y,th,vx,vy,r])
        return self.x

class PID:
    def __init__(self,kp,ki,kd,dt,umin,umax):
        self.kp=kp; self.ki=ki; self.kd=kd; self.dt=dt; self.umin=umin; self.umax=umax
        self.i=0; self.e_prev=0
    def reset(self): self.i=0; self.e_prev=0
    def step(self,e):
        self.i += e*self.dt
        d = (e - self.e_prev)/self.dt
        self.e_prev = e
        u = self.kp*e + self.ki*self.i + self.kd*d
        return float(np.clip(u, self.umin, self.umax))

class Controller:
    def __init__(self,p:HoverParams):
        self.speed = PID(18,4,2.5,p.dt,-2*p.F_max,2*p.F_max)
        self.heading= PID(60,0,8,p.dt,-p.F_max,p.F_max)
        self.F_max = p.F_max
    def compute(self, v_ref, v_now, psi_ref, psi_now):
        epsi = (psi_ref - psi_now + np.pi)%(2*np.pi)-np.pi
        u_spd = self.speed.step(v_ref - v_now)
        u_yaw = self.heading.step(epsi)
        F_total = np.clip(u_spd, -2*self.F_max, 2*self.F_max)
        F_diff  = np.clip(u_yaw, -self.F_max, self.F_max)
        FL = np.clip(0.5*(F_total - F_diff), -self.F_max, self.F_max)
        FR = np.clip(0.5*(F_total + F_diff), -self.F_max, self.F_max)
        return float(FL), float(FR)

# ---- Métrica de desempeño ----
def simulate_score(p: HoverParams, T=25.0):
    sim = HoverSim(p); sim.reset()
    ctrl = Controller(p)
    # Trayectoria: rectángulo
    WPs = np.array([[0,0],[6,0],[6,4],[0,4],[0,0]])
    iwp=1
    dt=p.dt; N=int(T/dt)
    err_pos=0.0; sat_time=0; sat_count=0
    vmax=0.0
    for k in range(N):
        pos = sim.x[:2]; th = sim.x[2]; v = float(np.hypot(sim.x[3], sim.x[4]))
        vmax = max(vmax, v)
        vec = WPs[iwp]-pos
        if np.linalg.norm(vec) < 0.5 and iwp < len(WPs)-1:
            iwp += 1; vec = WPs[iwp]-pos
        psi_ref = math.atan2(vec[1],vec[0])
        v_ref = min(np.linalg.norm(vec), 3.5)
        FL,FR = Controller(p).compute(v_ref, v, psi_ref, th)  # stateless to avoid bias
        if abs(FL)>=p.F_max-1e-6 or abs(FR)>=p.F_max-1e-6: sat_count += 1
        sim.step([FL,FR])
        err_pos += float(np.linalg.norm(WPs[iwp]-sim.x[:2]))*dt
        # Divergencia simple
        if abs(sim.x[0])>50 or abs(sim.x[1])>50 or not np.isfinite(sim.x).all():
            return np.inf, 1.0, vmax  # error enorme + saturación 100%
    sat_ratio = sat_count/N
    return err_pos, sat_ratio, vmax

def sweep(m_range, f_range, outdir):
    outdir.mkdir(parents=True, exist_ok=True)
    Ms = np.linspace(*m_range)
    Fs = np.linspace(*f_range)
    E = np.zeros((len(Ms), len(Fs)))  # error integral de posición
    S = np.zeros_like(E)              # % tiempo saturado
    V = np.zeros_like(E)              # v_max
    for i,m in enumerate(Ms):
        for j,F in enumerate(Fs):
            p = HoverParams(m=m, F_max=F)
            err, sat, vmax = simulate_score(p)
            E[i,j]=err; S[i,j]=sat; V[i,j]=vmax

    # Guardar CSV
    import csv
    with open(outdir/'results.csv','w',newline='') as f:
        w=csv.writer(f); w.writerow(["m_values"]+list(Ms))
        w.writerow(["F_values"]+list(Fs))
        w.writerow(["metric","row","col","value"])
        for i in range(len(Ms)):
            for j in range(len(Fs)):
                w.writerow(["E_err",i,j,E[i,j]])
        for i in range(len(Ms)):
            for j in range(len(Fs)):
                w.writerow(["S_sat",i,j,S[i,j]])
        for i in range(len(Ms)):
            for j in range(len(Fs)):
                w.writerow(["V_vmax",i,j,V[i,j]])

    # Mapas de calor
    def heat(data, title, fname):
        plt.figure()
        plt.imshow(data, origin='lower', aspect='auto',
                   extent=[Fs[0],Fs[-1],Ms[0],Ms[-1]])
        plt.colorbar(label=title)
        plt.xlabel("F_max [N]")
        plt.ylabel("m [kg]")
        plt.title(title)
        plt.tight_layout()
        plt.savefig(outdir/fname, dpi=160)
        plt.close()

    heat(E, "Error de seguimiento (menor es mejor)", "heat_error.png")
    heat(S, "% de saturación (menor es mejor)", "heat_saturation.png")
    heat(V, "Velocidad máxima [m/s]", "heat_vmax.png")

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=str, default="sweep_out")
    ap.add_argument("--m", type=float, nargs=3, default=[5, 40, 8], help="inicio fin puntos")
    ap.add_argument("--f", type=float, nargs=3, default=[20, 160, 8], help="inicio fin puntos")
    return ap.parse_args()

if __name__=="__main__":
    args = parse_args()
    outdir = Path(args.out)
    m0,m1,mp = args.m; f0,f1,fp = args.f
    sweep((m0,m1,int(mp)), (f0,f1,int(fp)), outdir)
    print(f"Listo. Resultados en: {outdir}")
