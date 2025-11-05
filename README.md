# 🚀 Hovercraft Simulator & Control Lab

Laboratorio completo para el **diseño, simulación y control** de aerodeslizadores (hovercraft), incluyendo:

✔ Modelo dinámico 2D  
✔ Control de velocidad y rumbo con PID  
✔ Simulación en tiempo real con interacción  
✔ Barrido de parámetros para diseño preliminar  
✔ Firmware base para pruebas en hardware real  

Este proyecto permite **iterar y validar** un diseño antes de fabricar el prototipo físico.

---

## 🎯 Objetivos del proyecto

- Investigar y desarrollar un aerodeslizador con **control autónomo**.
- Validar decisiones de diseño **sin gastar en materiales** todavía.
- Disponer de una herramienta experimental para tuning de control.
- Preparar integración futura con electrónica y sensores reales.

---

## 📦 Estructura del repositorio

hovercraft/
 ├─ dynamics.py                       # Modelo físico 2D
 ├─ controllers.py                    # PID velocidad / heading + waypoint tracker
 ├─ run_sim.py                        # Demo rápida de simulación + gráficos
 ├─ live_explorer.py                  # Simulador en tiempo real con sliders
 ├─ design_sweep.py                   # Barrido automático de parámetros (m, F_max)
 ├─ firmware/
 │   └─ arduino_hovercraft_ctrl.ino   # Control básico para ESC y sensores
 └─ README.md                         # Este archivo

---

## 🧠 Requisitos

Python 3.10+  
Dependencias:

pip install numpy matplotlib

---

## 🚀 Uso rápido

Simulación offline clásica:

python run_sim.py

Genera:
- trajectory.png
- speed.png
- inputs.png

---

## 🕹️ Simulación en tiempo real

python live_explorer.py

### Controles

Mover objetivo: Clic en el plano XY  
Pausar / Continuar: Botón Pause  
Reiniciar: Botón Reset  
Control automático ON/OFF: Check Controller ON  
Ajustar parámetros: Sliders  

### Parámetros configurables

- Masa m, inercia J, separación b
- Empuje máximo F_max
- Arrastres Cd_u, Cd_v, Cd_r
- Lift_max vs Peso → contacto o deslizamiento con suelo
- Amortiguamiento extra mu_ground
- Viento lateral Wind
- Control manual con Bias FL/FR

---

## 📊 Barrido de parámetros (Design Sweep)

python design_sweep.py --out sweep_out --m 5 40 8 --f 20 160 8

Genera en sweep_out/:
- results.csv
- heat_error.png (error de trayectoria — menor = mejor)
- heat_saturation.png (% saturación — menor = mejor)
- heat_vmax.png (velocidad máxima)

Ayuda a decidir:
- masa del prototipo
- motor/ESC adecuados
- equilibrio estabilidad ↔ rendimiento

---

## 🧪 Notas del modelo

- Dinámica 2D en plano (X/Y)
- La gravedad influye via cushion: si Lift < Peso → aumenta fricción
- Viento como fuerza constante en eje Y
- Modelo preliminar → ideal para descartar configuraciones malas rápido

No sustituye validación experimental final.

---

## 🔧 Integración futura (roadmap)

- Anti-windup + feedforward; control LQR / MPC
- Faltones segmentados y pérdidas de sustentación realistas
- Telemetría y exportación de datos
- Integración con IMU (yaw/ω) y estimador de velocidad (óptica/UWB)
- Joystick/Gamepad + ESC y microcontrolador

---

## 📜 Licencia

MIT License

---

## ✨ Créditos

Proyecto original, diseño y experimentación: Cuervo

