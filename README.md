# Quadrotor Trajectory Tracking: Safe RL-NMPC & Feedback Linearization

This repository contains the simulation framework developed for my graduation thesis at the Czech Technical University in Prague (CTU). It focuses on highly dynamic quadrotor trajectory tracking by combining the safety bounds of Nonlinear Model Predictive Control (NMPC) with the zero-shot adaptability of a Twin Delayed DDPG (TD3) Reinforcement Learning agent.

## 1. System Architecture

The framework employs a flattened, fully synchronous control loop operating at a unified sampling time ($T_s = 0.1s$) to eliminate temporal delays between position and attitude tracking.

- **Path Planner (RRT):** Generates collision-free waypoints.
- **Outer Loop (Feedback Linearization):** Computes total thrust ($U_1$) and reference Euler angles ($\phi_{ref}, \theta_{ref}$).
- **Inner Loop (NMPC + RL):** The NMPC solves the online optimization for the nominal 6-DOF nonlinear model. Concurrently, the RL agent observes the state and computes a residual torque compensation ($\Delta U$).
- **Final Control Law:** $U_i = U_{i,NMPC} + \Delta U_i$ for $i \in \{2, 3, 4\}$.

## 2. Deep Reinforcement Learning (TD3) Setup

Instead of end-to-end control, the RL agent acts as a residual compensator to mitigate nonlinear aerodynamic effects and unmodeled dynamics.

### Observation Space (19 Dimensions)

- **Normalized State Errors (10):** Position error (3), Yaw error (1), Body velocities (3), Angular rates (3).
- **Future Lookahead (9):** Reference trajectory coordinates at $t+0.3s$, $t+0.6s$, and $t+1.0s$ to prevent latency during sharp turns.

### Network Architecture

- **Actor Network:** 19 Inputs $\rightarrow$ Dense(128) $\rightarrow$ Dense(128) $\rightarrow$ Dense(64) $\rightarrow$ 3 Outputs (Tanh activated, scaled to physical torque limits).
- **Critic Network:** 22 Inputs (19 States + 3 Actions) $\rightarrow$ Dense(128) $\rightarrow$ Dense(64) $\rightarrow$ 1 Q-Value.

## 3. Zero-Shot Generalization Performance

The TD3 agent was trained exclusively on a standard expanding **Spiral Trajectory** utilizing Domain Randomization (varying initial coordinates). During inference, the learned policy demonstrates robust zero-shot transfer capabilities, successfully tracking complex, unseen paths without retraining:

- Figure-Eight (Lemniscate)
- 3D Lissajous (Space Butterfly)
- Wavy Circle (Dynamic Throttle Modulation)

## 4. Repository Structure

```text
├── docs/
│   ├── 01_system_architecture.md
│   ├── 02_nonlinear_quadrotor_model.md
│   ├── 03_rl_nmpc_integration.md
│   └── 04_zero_shot_simulation_results.md
├── src/
│   ├── DroneRLEnv.m          # Custom MATLAB RL Environment (Normalization & Lookahead)
│   ├── train_RL.m            # TD3 Agent Training Script
│   ├── main_RL_NMPC.m        # Synchronous Closed-loop Simulation
│   ├── pos_controller.m      # Feedback Linearization
│   └── attitude_cost_nmpc.m  # NMPC Cost Function
└── weights/
    └── RL_NMPC_Policy.mat    # Pre-trained TD3 Actor-Critic weights
```
