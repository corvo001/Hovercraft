
# Hovercraft Sim + Control (Cuervo)

Contenido:
- `dynamics.py`: modelo 2D simple del aerodeslizador.
- `controllers.py`: PID de velocidad y rumbo + seguidor de waypoints.
- `run_sim.py`: demo de simulación y generación de gráficos.
- `firmware/arduino_hovercraft_ctrl.ino`: esqueleto de firmware para dos ESC.

## Requisitos
Python 3.10+ con `numpy` y `matplotlib`.

## Uso rápido
```bash
python run_sim.py
```
Genera:
- `trajectory.png`
- `speed.png`
- `inputs.png` en `/mnt/data/hovercraft_sim/`.

## Próximos pasos sugeridos
1. Ajustar parámetros físicos (masa, inercia, drag, F_max) a tu prototipo.
2. Sustituir seguidor de waypoints por control manual (joystick) o ROS2.
3. Añadir saturación/anti-windup explícitos y feedforward.
4. Integrar IMU (MPU6050/ICM-20948) y estimador de velocidad (odometría óptica o UWB).
5. Migrar a control robusto/LQR/MPC cuando el modelo esté validado.


# Hovercraft Live Explorer

Simulador "en tiempo real" con **sliders** para variar parámetros físicos
y observar la respuesta del aerodeslizador (trayectoria, velocidad y fuerzas).

## Requisitos
```
pip install numpy matplotlib
```

## Ejecutar
```
python live_explorer.py
```

## Qué puedes variar
- Masa `m`, inercia `J`, separación de propulsores `b`
- Empuje máximo por propulsor `F_max`
- Arrastres `Cd_u`, `Cd_v`, `Cd_r`
- **Lift_max** (capacidad de sustentación vertical) y **mu_ground** (amortiguamiento extra cuando el lift < peso)
- **Wind** (fuerza lateral constante en eje Y mundo)
- **Bias FL/FR** para aplicar fuerzas manuales si apagas el controlador

## Cómo usar
- Clic en el **plano XY** para poner el waypoint objetivo.
- Marca/Desmarca **Controller ON** para ver la diferencia entre control automático y fuerzas manuales.
- **Reset** reinicia la simulación. **Pause** congela/continúa.
- Observa en el recuadro: posición, velocidad, ángulo, giro, **lift vs peso**.

## Notas de modelo
- La gravedad no afecta directamente a X/Y; su efecto se modela como aumento de fricción cuando `Lift < Peso`.
- El viento entra como fuerza constante en Y-mundo.
- Es un modelo 2D de primer orden, útil para diseño preliminar de control y trade-offs. No sustituye pruebas reales.

---

## Barrido de parámetros (design_sweep.py)

Ejemplo rápido:
```
python design_sweep.py --out sweep_out --m 5 40 8 --f 20 160 8
```
Genera:
- `results.csv`
- `heat_error.png`  (error integral de seguimiento, menor es mejor)
- `heat_saturation.png`  (% tiempo al límite de F_max, menor es mejor)
- `heat_vmax.png`  (velocidad máxima alcanzada)

Consejo: busca una zona con **bajo error** y **baja saturación**.
