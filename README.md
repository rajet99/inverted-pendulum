# Linear Inverted Pendulum Simulation & Control

The goal of this project is to model the physical dynamics of a single inverted pendulum in 3D space and implement control algorithms to achieve upright balance.

## Overview

This repository contains the Simulink model, C++ control algorithm implementations, and MATLAB scripts used to simulate and control a single inverted pendulum system. This project served as the simulation and prototyping phase prior to building the physical embedded system.

### Key Features & Control Strategies
* **Åström-Furuta Swing-Up Control:** Energy-based control strategy that pumps kinetic energy into the pendulum from its downward resting state to swing it up toward vertical equilibrium.
  <p align="center">
    <img src="https://latex.codecogs.com/svg.image?%5Cdisplaystyle%20V%20%3D%20%5Cfrac%7B1%7D%7B2%7DmL%5E2%5Cdot%7B%5Ctheta%7D%5E2%20%2B%20mgL(1-%5Ccos%5Ctheta)" alt="Swing-up energy equation" />
  </p>
  <p align="center">
    <img src="https://latex.codecogs.com/svg.image?%5Cdisplaystyle%20%5Cdot%7BV%7D%20%3D%20mL%5Cdot%7B%5Ctheta%7D(%5Ccos%5Ctheta)%5Cddot%7Bx%7D" alt="Swing-up energy rate equation" />
  </p>
* **LQR Control:** Optimal state-feedback control for maintaining balance in the linear regime near vertical while maintaining desired position.
  <p align="center">
    <img src="https://latex.codecogs.com/svg.image?%5Cdisplaystyle%20%5Cdot%7Bx%7D%20%3D%20Ax%20%2B%20Bu%2C%20%5Cquad%20u%20%3D%20-Kx" alt="LQR state equation" />
  </p>
  <p align="center">
    <img src="https://latex.codecogs.com/svg.image?%5Cdisplaystyle%20J%20%3D%20%5Cint_0%5Cinfty%20(x%5ETQx%20%2B%20u%5ETRu)%2Cdt" alt="LQR cost equation" />
  </p>
* **Cascaded PID Control:** An alternative balancing architecture utilizing an outer loop (position/angle reference) and inner loop (angle/actuator control) to stabilize the pendulum.
  <p align="center">
    <img src="https://latex.codecogs.com/svg.image?%5Cdisplaystyle%20e_x%20%3D%20x-x_0%2C%20%5Cquad%20e_%5Ctheta%20%3D%20%5Ctheta-%5Ctheta_%7B%5Ctext%7Bref%7D%7D" alt="PID error definitions" />
  </p>
  <p align="center">
    <img src="https://latex.codecogs.com/svg.image?%5Cdisplaystyle%20%5Ctheta_%7B%5Ctext%7Bref%7D%7D%20%3D%20K_%7Bp%2Cx%7De_x%20%2B%20K_%7Bi%2Cx%7D%5Cint%20e_x%20dt%20%2B%20K_%7Bd%2Cx%7D%5Cdot%7Be%7D_x" alt="Outer PID equation" />
  </p>
  <p align="center">
    <img src="https://latex.codecogs.com/svg.image?%5Cdisplaystyle%20u%20%3D%20K_%7Bp%2C%5Ctheta%7De_%5Ctheta%20%2B%20K_%7Bi%2C%5Ctheta%7D%5Cint%20e_%5Ctheta%20dt%20%2B%20K_%7Bd%2C%5Ctheta%7D%5Cdot%7Be%7D_%5Ctheta" alt="Inner PID equation" />
  </p>
* **Swing-Up Control & Deceleration Safeguard:** Energy-based swing-up logic to swing the pendulum up from rest. Includes safety deceleration logic to prevent excessive angular velocity if the swing-up gains too much speed, ensuring the LQR or PID can smoothly catch and stabilize it.

* **Built-in 3D Visualization:** Utilizes Simscape Multibody to render and visualize physical pendulum motion and joint dynamics during simulation runs.
* **Support for Sensor Noise and Motor Limitations:** Can simulate imperfections in the system by adding quantization and noise to encoders, acceleration, velocity (or current) limits for the motor, bridging the Sim2Real gap.


## Demo Videos

Here are some videos demonstrating the functionality and outcomes of this project.

### Simscape Visualizations

#### 1. PID Control

A cascaded PID-based simulation showing the pendulum attempting to stabilize while maintaining its center position, inducing oscillations.

#### 2. Swing-up + LQR Control

A combined energy-based swing-up and LQR control example showcasing the sharp handoff transition once the pendulum reaches near the desired equilibrium point.

#### 3. Swing-up -> LQR Handoff Failure

A demonstration of an unsuccessful swing-up to LQR handoff, highlighting the instability that can occur during the transition, especially using less powerful motors with lower acceleration limits. This is representative of the stepper motor used in the physical project.

#### 4. Swing-up -> LQR Handoff Resolved

A resolved swing-up to LQR handoff with angular velocity damping, showing a smoother transition into balanced control in spite of motor restraints.

### Embedded Hardware Test Demo

<p align="center">
  <a href="https://youtu.be/FhDWkm6h2VQ" target="_blank">
    <img src="https://img.youtube.com/vi/FhDWkm6h2VQ/hqdefault.jpg" alt="Embedded hardware demo 1" width="45%" style="margin-right: 5%;" />
  </a>
  <a href="https://youtu.be/eeGyaRxlmUQ" target="_blank">
    <img src="https://img.youtube.com/vi/eeGyaRxlmUQ/hqdefault.jpg" alt="Embedded hardware demo 2" width="45%" />
  </a>
</p>

Utilizing the control logic from this project, I led the implementation of the swing-up + LQR control logic into a real embedded system based on an ESP32 microcontroller, a NEMA17-style stepper motor, and an A4988 driver board. The initial plan was to use a Teensy 4.0 and a BLDC motor with field-oriented control (FOC) via a module like [SimpleFOC](https://github.com/simplefoc); however, due to board and motor controller malfunctions we were forced to switch to iterate to a new architecture.
Although the swing-up damping logic was not well tuned, the project was limited by time constraints and, even so, the work demonstrated the core functionality of this project and established how the Simulink workflow could be used to prototype control systems for physical linkages. The system was robust to adding disturbances and even new weights at the end of the pendulum, highlighting the effectiveness of LQR control for a system like this despite not having a perfect physics model to calculate gains.

## System Requirements

Before running the simulation, ensure you have the following installed:

* **MATLAB & Simulink** (R2024b - not forward compatible)
* **Simscape** & **Simscape Multibody** Toolboxes (required for 3D physics modeling and built-in visualization)
* **C++ Compiler** (GCC, Clang, or MSVC) if compiling/testing control C++ modules independently


## Setup & Running

1. Open **MATLAB R2024b** and set the repository folder as your working directory.
2. Ensure the control logic discrete time step is defined in your MATLAB workspace. 
   > **Note:** The control logic requires a discrete 'timestep' variable (e.g., `timestep = 0.001`). If running standalone scripts or loading the model directly, ensure `timestep = 0.001` (1 ms) is defined in the workspace prior to simulation.
3. Run `lqr.mlx` to compute linear system dynamics and calculate the optimal LQR gain matrix ($K$).
4. Open `single_pendulum.slx` in **Simulink**.
5. Run the simulation:
   * Select your control strategy (LQR or Cascaded PID) within the model configuration.
   * View the interactive **Simscape Multibody Mechanics Explorer** window to watch the real-time 3D motion and control dynamics.
   * Use the custom MATLAB script to use angular velocity damping in the swingup -> balance transition if desired.

## Repository Structure

```text
.
├── single_pendulum.slx  # Main Simulink simulation model (Simscape Multibody)
├── lqr.mlx              # MATLAB Live Script for LQR gain calculation & linear dynamics
├── swing_up.cpp / .h    # Swing-up energy controller & deceleration logic
├── lqr.cpp / .h         # LQR control law implementation
├── run_lqr.cpp / .h     # Execution logic and state transition manager for LQR
├── inner_pid.cpp / .h   # Inner loop PID implementation (velocity/rate control)
├── outer_pid.cpp / .h   # Outer loop PID implementation (position/angle control)
├── pid.cpp / .h         # Base PID controller module
├── LICENSE              # Repository license
└── README.md            # Project documentation