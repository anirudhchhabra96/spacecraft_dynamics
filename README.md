# 🛰️ Spacecraft Attitude Simulation in MATLAB

This repository simulates the free rotational motion of a rigid-body spacecraft using Euler's equations and quaternions, visualizes the 3D attitude of a satellite-shaped cube, and plots angular velocity, Euler angles, and Euler angle rates over time.

<p align="center">
  <img src="preview.gif" width="500"/>
</p>

---

## 📌 Features

- Integration of 3D rotational dynamics using `ode45`
- Quaternion-based attitude propagation
- Real-time 3D animation of a **shiny gold cube satellite**
- Live display of:
  - Angular velocity (ω)
  - Euler angles (roll, pitch, yaw)
  - Axis of rotation (visualized as a red line)
- Subplot visualization of:
  - Body angular velocity
  - Euler angle rates
  - Euler angles (ZYX/3-2-1)

---

## 🧠 Dynamics Model

The simulation uses:

- **Euler’s equations** (torque-free rotation):
  \[
  \dot{\omega} = J^{-1} \left( -\omega \times (J \omega) \right)
  \]
- **Quaternion kinematics**:
  \[
  \dot{q} = \frac{1}{2} \Omega(\omega) q
  \]
- **Euler angle extraction** from quaternion using ZYX convention (3-2-1)
- **Euler angle rates** computed from body angular velocity

---

## 🛠️ How to Run

1. Clone the repository or download the `.m` files
2. Open the main script (e.g., `simulate_spacecraft.m`)
3. Run in MATLAB

```matlab
simulate_spacecraft
