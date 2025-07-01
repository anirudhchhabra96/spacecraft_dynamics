# Spacecraft Attitude Dynamics Simulation and Hexapod Validation

This MATLAB project simulates the 3D rotational motion of a rigid spacecraft and validates it using a physical hexapod robot with a mounted cube. The system integrates dynamic simulation, real-time animation, and hardware-in-the-loop (HIL) execution.

---

## 🛰️ Overview

- Simulates torque-free rotational dynamics using Euler’s rigid body equations
- Uses quaternion integration and Euler angle extraction (ZYX: 3-2-1)
- Controls a PI hexapod to physically reproduce the cube’s attitude
- Compares simulated vs real trajectories
- Provides smooth, shiny cube animation with overlaid metrics and axis of rotation
- Exports both `.gif` and `.mp4` animations

---

## ⚙️ Equations Used

- **Euler’s Equation (body frame):**  
  d(ω)/dt = J⁻¹ × [ -ω × (Jω) ]

- **Quaternion Kinematics:**  
  d(q)/dt = ½ × Ω(ω) × q

- **Euler Angle Extraction (ZYX):**  
  ϕ = atan2(R₃₂, R₃₃),  
  θ = –asin(R₃₁),  
  ψ = atan2(R₂₁, R₁₁)

- **Euler Angle Rates:**  
  [ϕ̇, θ̇, ψ̇] computed from ω using inverse kinematic mapping

---

## 🧪 Hardware Setup

- **Platform:** PI 6-DOF Stewart hexapod (connected via TCP/IP)
- **Cube:** Mounted 99mm above the moving platform in Z-axis
- **Driver:** `PI_GCS_Controller` from `PI_MATLAB_Driver_GCS2`
- **Feedback:** Actual pose retrieved using `qPOS` during streaming

---

## 📽️ Output Visuals

- Simulated vs tracked cube animation (`sim_vs_hexapod.mp4`)
- Red bidirectional axis showing rotation axis
- Euler angles, angle rates, and angular velocity plots
- Exportable GIF: `sim_vs_hexapod.gif`

---

## ▶️ How to Run

```matlab
sim_vs_hil();
```

---

## 📂 File Structure

```
📁 .
 ┣ .git/
 ┣ animate_sim_vs_hexapod.m
 ┣ create_cube.m
 ┣ data.mat
 ┣ eul321_to_rotm.m
 ┣ hexapod_MOV_test.m
 ┣ LICENSE                     ← GNU GPLv3
 ┣ quat2eul321.m
 ┣ quat2rotm.m
 ┣ quat_to_rotm.m
 ┣ README.md
 ┣ rotm2eul_custom.m
 ┣ rotm_to_axis_angle.m
 ┣ sim_vs_hexapod.gif
 ┣ sim_vs_hexapod.mp4
 ┣ sim_vs_hil.m                ← Main script
 ┣ spacecraft_dynamics.m
 ┗ spacecraft_sim.m
```

---

## 📄 License

This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**.  
You **must open-source** any derivative work if you modify and distribute this code.

For details, see [`LICENSE`](LICENSE).
