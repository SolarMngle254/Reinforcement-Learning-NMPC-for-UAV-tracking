# Quadrotor Trajectory Tracking: Safe RL-NMPC & Feedback Linearization

![MATLAB](https://img.shields.io/badge/MATLAB-R2025a-blue)
![Control](https://img.shields.io/badge/Control-RL--NMPC-green)
![AI](https://img.shields.io/badge/Deep_RL-TD3-orange)
![Status](https://img.shields.io/badge/Status-Graduation_Thesis-red)

> This repository was developed as part of my graduation thesis at the Ho Chi Minh city University of Technology - Vietnam National University (HCMUT - VNU).
>
> The project focuses on highly dynamic quadrotor trajectory tracking by combining the safety bounds of Nonlinear Model Predictive Control (NMPC) with the zero-shot adaptability of a Twin Delayed DDPG (TD3) Reinforcement Learning agent.
>
> Moving beyond traditional piecewise linear approximations (like LPV-MPC), this framework utilizes a fully nonlinear 6-DOF model where the RL agent acts as a residual compensator, correcting unmodeled dynamics and aerodynamic disturbances in real-time.
>
> **Note:** This repository contains detailed mathematical modeling, controller derivations, and the complete training/simulation environment used to validate the zero-shot generalization of the RL-NMPC framework.

A hierarchical and synchronous trajectory-tracking framework for a quadrotor UAV:

**Traj Gen** → **Feedback Linearization** → **Safe RL-NMPC (NMPC + TD3)** → **Quadrotor**

<div align="center">
  <img src="pics/control_architecture.jpg" alt="Quadrotor RL-NMPC Control Architecture" width="850">
  <br><em>Overall control architecture.</em>
</div>

---

## 1. System Overview

Unlike traditional cascaded architectures with multi-rate loops, this framework employs a flattened, fully synchronous control loop operating at a unified sampling time ($T_s = 0.1s$) to eliminate temporal delays between position and attitude tracking.

**Outer Loop (Feedback Linearization):**

$$
\mathbf{p}_{ref} \rightarrow \mathrm{FL} \rightarrow
\begin{bmatrix}
U_1 & \phi_{ref} & \theta_{ref}
\end{bmatrix}^{T}
$$

**Inner Loop (Safe RL-NMPC):**

The total torque command is a superposition of the nominal NMPC output and the RL residual compensation:

$$
U_i = U_{i, NMPC} + \Delta U_i
\quad \text{for } i \in \{2, 3, 4\}
$$

$$
\mathbf{U} =
\begin{bmatrix}
U_1 & U_2 & U_3 & U_4
\end{bmatrix}^{T}
$$

---

## 2. Quadrotor Nonlinear Model

The nonlinear plant is a 6-DOF Newton–Euler model. The state vector is defined as:

$$
\mathbf{x} =
\begin{bmatrix}
X & Y & Z & u & v & w & \phi & \theta & \psi & p & q & r
\end{bmatrix}^{T}
$$

The control input vector combines the total thrust and the three body torques:

$$
\mathbf{U} =
\begin{bmatrix}
U_1 & U_2 & U_3 & U_4
\end{bmatrix}^{T}
$$

<div align="center">
  <img src="pics/reference_frames.jpg" alt="Reference Frames" width="650">
  <br><em>Earth-fixed and body-fixed reference frames.</em>
</div>

---

## 3. Deep Reinforcement Learning (TD3) Setup

Instead of relying on end-to-end black-box control, the RL agent acts as a residual compensator to mitigate nonlinear aerodynamic effects while preserving the Lyapunov stability guarantees provided by the NMPC.

### Observation Space (19 Dimensions)

- **Normalized State Errors (10):** Position error (3), Yaw error (1), Body velocities (3), Angular rates (3).
- **Future Lookahead (9):** Reference trajectory coordinates at $t+0.3s$, $t+0.6s$, and $t+1.0s$ to prevent latency during sharp maneuvers.

### Network Architecture

- **Actor Network:** 19 Inputs → Dense(128) → Dense(128) → Dense(64) → 3 Outputs (Tanh activated, scaled to physical torque limits).
- **Critic Network:** 22 Inputs (19 States + 3 Actions) → Dense(128) → Dense(64) → 1 Q-Value.

---

## 4. Zero-Shot Generalization

The TD3 agent was trained exclusively on a standard expanding **Spiral Trajectory** utilizing Domain Randomization. During inference, the learned policy demonstrates robust **zero-shot transfer capabilities**, successfully tracking complex, unseen paths without retraining:

1. **Circle Trajectory:** Baseline performance verification.
2. **Figure-Eight (Lemniscate):** Continuous acceleration reversal and sharp crossovers.
3. **3D Lissajous (Space Butterfly):** Aggressive multi-axis coupled maneuvers.
4. **Wavy Circle:** Dynamic throttle modulation testing.

<div align="center">
  <img src="pics/sim/sim_tracking_zero_shot.gif" alt="Zero Shot Tracking" width="850">
  <br><em>Zero-shot trajectory-tracking simulation (Figure-Eight).</em>
</div>

---

## 5. Repository Structure

```text
.
├── docs/
│   ├── 01_system_architecture.md
│   ├── 02_nonlinear_quadrotor_model.md
│   ├── 03_feedback_linearization.md
│   ├── 04_rl_nmpc_integration.md
│   ├── 05_td3_agent_design.md
│   └── 06_zero_shot_simulation_results.md
│
├── pics/
│   ├── sim/
│   │   ├── train_convergence.jpg
│   │   ├── sim_tracking_spiral.gif
│   │   └── sim_tracking_zero_shot.gif
│   ├── control_architecture.jpg
│   ├── euler_angle_formation.jpg
│   ├── reference_frames.jpg
│   ├── actor_critic_network.jpg
│   └── mdp_diagram.jpg
│
├── src/
│   ├── DroneRLEnv.m            # Custom MATLAB RL Environment
│   ├── trajectory_generator.m  # Generates 5 distinct trajectories
│   ├── train_RL.m              # TD3 Agent Training Script
│   ├── main_RL_NMPC.m          # Synchronous Closed-loop Simulation
│   ├── pos_controller.m        # Feedback Linearization (Outer Loop)
│   ├── attitude_cost_nmpc.m    # NMPC Cost Function
│   └── nonlinear_drone_model.m # 6-DOF Plant Dynamics (ODE45)
│
├── weights/
│   └── RL_NMPC_Policy.mat      # Pre-trained TD3 Actor-Critic weights
│
└── README.md
```

---

## 6. Documentation

| Topic | Documentation |
|---|---|
| System Architecture & Synchronous Loop | `01_system_architecture.md` |
| 6-DOF Nonlinear Dynamic Model | `02_nonlinear_quadrotor_model.md` |
| Position Controller (Feedback Linearization) | `03_feedback_linearization.md` |
| Safe RL-NMPC Integration | `04_rl_nmpc_integration.md` |
| TD3 Agent & MDP Design | `05_td3_agent_design.md` |
| Zero-Shot Simulation Results | `06_zero_shot_simulation_results.md` |

---

## 7. Key Idea

$$
\boxed{
\text{Plan Path}
\rightarrow
\text{Compute Virtual Acceleration (FL)}
\rightarrow
\text{NMPC}
\rightarrow
\text{TD3 Residual Compensation}
\rightarrow
\text{Quadrotor}
}
$$

The **NMPC** provides theoretical safety guarantees and constraint satisfaction, while the **TD3 RL Agent** acts as an intelligent residual compensator to eliminate tracking errors caused by complex nonlinearities that the online optimizer cannot resolve in real-time.

---

## 8. References

[1] Dat Vu Tien et al., *"Safe Dual-Actor Reinforcement Learning-Based Nonlinear Model Predictive Control Architecture and Imitation learning for Autonomous Vehicle Longitudinal Control,"* 2024.

[2] N. Hansen et al., *"TD-MPC2: Scalable, Robust World Models,"* 2023.

[3] J. Choi et al., *"Reinforcement Learning for Safety-Critical Control under Model Uncertainty,"* 2020.

[4] S. A. and A. Bemporad, *"Learning Lyapunov Terminal Costs from Data for Complexity Reduction in NMPC,"* International Journal of Robust and Nonlinear Control.

[5] P. Wang, Z. Man, Z. Cao, and J. Zheng, *"Dynamics Modelling and Linear Control of Quadcopter,"* in 2016 International Conference on Advanced Mechatronic Systems (ICAMechS), Melbourne, Australia, 2016.
