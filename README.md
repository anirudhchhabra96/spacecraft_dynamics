# Spacecraft Attitude Dynamics and Hardware-in-the-Loop Validation

This project simulates the free rotational motion of a spacecraft in 3D using rigid body dynamics (Euler’s equations) and visualizes the evolution of its orientation using a 3D animated cube. It also supports hardware-in-the-loop (HIL) testing using a real hexapod positioning platform, allowing validation of the simulation on physical hardware.

---

## 🚀 Features

- Simulates spacecraft rotational dynamics under torque-free motion.
- Visualizes the cube’s attitude evolution in real-time (animated 3D rendering).
- Plots:
  - Angular velocity (ω)
  - Euler angles (ZYX convention)
  - Euler angle rates (𝜙̇, 𝜃̇, ψ̇)
- Compares simulated vs. real attitude when using hardware.
- Option to save animation as GIF or MP4.
- Includes trajectory playback on a **PI hexapod** with a cube mounted 99 mm above the platform.
- Records and plots actual hexapod pose (if available) against the simulated trajectory.

---

## 🧠 Physics Modeled

- Uses Euler's equations for rotational motion:
  
dω/dt = J⁻¹ ( -ω × Jω )


- Quaternion-based integration for stable rotation tracking.
- Converts quaternion to ZYX Euler angles for interpretability.

---

## 🛠️ Hardware-in-the-Loop Setup

- **Platform:** PI Hexapod (controlled over TCP/IP via PI GCS MATLAB Library)
- **Setup:** Cube mounted at 99 mm height on hexapod platform
- **Control Mode:** Position + orientation commands (X, Y, Z, U, V, W axes)
- **Software:**
- Uses MATLAB `PI_GCS_Controller` for trajectory execution
- Streaming of simulated attitude to hexapod
- Optionally records feedback for closed-loop comparison

---

## 🖥️ Requirements

### MATLAB Toolboxes:
- No special toolboxes required for simulation only.
- For `quat2rotm` / `rotm2eul` equivalents, custom versions are provided if toolboxes are unavailable.

### Hexapod Control:
- PI MATLAB GCS2 driver (`PI_MATLAB_Driver_GCS2`)
- IP and port settings of hexapod controller configured correctly

---

## 📂 File Structure

```bash
.
├── sim_vs_hil.m                  # Main script (run this)
├── spacecraft_dynamics.m        # ODE function
├── animate_sim_vs_hexapod.m     # 3D visualization for sim vs hardware
├── create_cube.m                # Cube geometry
├── quat2eul321.m                # Custom ZYX Euler from quaternion
├── eul321_to_rotm.m             # Rotation matrix from Euler angles
├── utils/
│   └── rotm_to_axis_angle.m     # Used for red line visualization
├── output/
│   └── sim_vs_hexapod.mp4       # Example output video (if saved)

## ▶️ How to Run
1. Simulation Only

sim_vs_hil()

2. With Hardware (Hexapod)

    Ensure your PI hexapod is connected and reachable via TCP/IP.

    Set your hexapod's IP address in the script:

ip = '192.168.20.3';  % or as detected

Then run:

    sim_vs_hil()

📈 Output

    3-panel plot comparing:

        Angular velocity (ω)

        Euler angles

        Euler angle rates

    Dual 3D animation:

        Simulated cube vs. hexapod-tracked cube

        Rotation axis visualized as a red line

        Live overlay of key metrics

📜 License

MIT License — see LICENSE file for details.


---

Let me know if you want:
- Vicon integration instructions included (future extension)
- Screenshot or animation preview embedded
- Installation steps for PI GCS MATLAB driver
