# Spacecraft Attitude Simulation and Hardware-in-the-Loop Validation

This project simulates the free rigid-body rotational dynamics of a spacecraft and compares it to a real-time hardware emulation using a 6-DOF hexapod with a mounted cube. The simulation results and hexapod motion are visualized and recorded side-by-side.

## Features

- Euler's equations of motion integrated using `ode45`
- Quaternion-based attitude propagation
- Conversion to ZYX Euler angles and angle rates
- Real-time animation of a rotating gold cube
- Live metrics overlay: angular velocity, Euler angles, etc.
- Red axis showing instantaneous axis of rotation
- Comparison with physical hexapod motion (HIL)
- MATLAB control of PI hexapod via TCP/IP
- Visualization: Simulated vs. Tracked cube rotation
- Exports video (`.mp4`) of side-by-side animation
- Optional comparison plots for:
  - Angular velocity (ω)
  - Euler angles (ϕ, θ, ψ)
  - Euler angle rates (𝜙̇, 𝜃̇, 𝜓̇)

## Requirements

- MATLAB (R2020+)
- PI GCS MATLAB Driver (`PI_MATLAB_Driver_GCS2`)
- PI Hexapod controller connected over TCP/IP
- Cube attached to hexapod, centered at 99 mm height
- (Optional) `VideoWriter` support for `.mp4` export

## Run Simulation and Stream to Hexapod

```matlab
sim_vs_hil();
```

This will:
1. Simulate the desired spacecraft attitude dynamics.
2. Stream the corresponding rotation profile (XYZ + RPY) to the real hexapod.
3. Record the actual hexapod response using `qPOS()` over time.
4. Animate both cubes side-by-side for visual validation.
5. Save the animation as a video.

## Notes

- If you don’t have a hexapod, set `simulate_only = true` to just run the simulation and animation.
- Euler angles are computed using 3-2-1 (ZYX) convention.
- The cube is visualized with shiny gold shading and a red rotation axis.

## License

This project is licensed under the MIT License. See `LICENSE` for details.
