# Linear Inverted Pendulum Simulation & Control

This repository contains the Simulink model, C++ control algorithm implementations, and MATLAB scripts used to simulate and control a single inverted pendulum system. This project served as the simulation and prototyping phase prior to building the physical embedded system.

---

## Overview

The goal of this project is to model the physical dynamics of a single inverted pendulum in 3D space and implement control algorithms to achieve upright balance.

### Key Features & Control Strategies
* **LQR Control:** Optimal state-feedback control for maintaining balance in the linear regime near vertical.
* **Cascaded PID Control:** An alternative balancing architecture utilizing an outer loop (position/angle reference) and inner loop (rate control) to stabilize the pendulum.
* **Swing-Up Control & Deceleration Safeguard:** Energy-based swing-up logic to swing the pendulum up from rest. Includes safety deceleration logic to prevent excessive angular velocity if the swing-up gains too much speed, ensuring the LQR or PID can smoothly catch and stabilize it.
* **Built-in 3D Visualization:** Utilizes Simscape Multibody to render and visualize physical pendulum motion and joint dynamics during simulation runs.

---

## System Requirements

Before running the simulation, ensure you have the following installed:

* **MATLAB & Simulink** (R2024b - not forward compatible)
* **Simscape** & **Simscape Multibody** Toolboxes (required for 3D physics modeling and built-in visualization)
* **C++ Compiler** (GCC, Clang, or MSVC) if compiling/testing control C++ modules independently

---

## Setup & Running

1. Open **MATLAB R2024b** and set the repository folder as your working directory.
2. Ensure the control logic discrete time step is defined in your MATLAB workspace. 
   > **Note:** The control logic requires a discrete 'timestep' variable (e.g., `timestep = 0.001`). If running standalone scripts or loading the model directly, ensure `timestep = 0.001` (1 ms) is defined in the workspace prior to simulation.
3. Run `lqr.mlx` to compute linear system dynamics and calculate the optimal LQR gain matrix ($K$).
4. Open `single_pendulum.slx` in **Simulink**.
5. Run the simulation:
   * Select your control strategy (LQR or Cascaded PID) within the model configuration.
   * View the interactive **Simscape Multibody Mechanics Explorer** window to watch the real-time 3D motion and control dynamics.

---

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