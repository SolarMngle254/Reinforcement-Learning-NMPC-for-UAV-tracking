# 06. Zero-Shot Generalization and Simulation Results

To validate the effectiveness, robustness, and safety of the proposed Residual RL-NMPC architecture, comprehensive simulations are conducted using MATLAB/Simulink. The primary objective is to evaluate the system's "Zero-Shot" generalization capability—its ability to track highly complex, aggressive trajectories that the Reinforcement Learning agent has never encountered during training.

## 1. Simulation Setup & Zero-Shot Conditions

The TD3 agent is trained strictly on a simple 3D Spiral trajectory to learn fundamental aerodynamic compensations. During the evaluation phase, the trained policy is locked (inference mode) and the quadrotor is tasked with tracking a highly dynamic **3D Lissajous curve (Butterfly trajectory)**. 

To simulate real-world conditions (the sim-to-real gap), unmodeled aerodynamic disturbances and parameter mismatches (e.g., variations in mass and inertia) are injected into the nonlinear plant model.

<p align="center">
  <img src="../pics/zero_shot_lissajous.gif" alt="3D Zero-Shot Trajectory Tracking Animation" width="700">
</p>
<p align="center"><i>Figure 1: Quadrotor tracking the unseen 3D Lissajous trajectory in real-time, demonstrating zero-shot generalization.</i></p>

## 2. 3D Trajectory Tracking Performance

The performance of the proposed dual-architecture is compared against a baseline NMPC-only controller. When navigating the aggressive curves of the Lissajous trajectory, the unmodeled dynamics cause significant tracking degradation for the baseline NMPC.

<p align="center">
  <img src="../pics/3d_trajectory_comparison.png" alt="3D Trajectory Comparison: NMPC vs RL-NMPC" width="700">
</p>
<p align="center"><i>Figure 2: 3D spatial comparison showing the reference trajectory (Black), baseline NMPC drifting (Red), and tight RL-NMPC tracking (Blue).</i></p>

As observed in Figure 2, the baseline NMPC (red) struggles at sharp corners due to the mismatch between its nominal prediction model and the actual nonlinear plant. In contrast, the RL-NMPC architecture (blue) utilizes the high-frequency residual compensation ($\Delta\mathbf{U}_{RL}$) to eliminate these model mismatches, resulting in near-perfect spatial alignment with the reference curve.

## 3. Quantitative Error Analysis

The superiority of the residual control scheme is further demonstrated through the transient response of the position tracking errors.

<p align="center">
  <img src="../pics/position_tracking_error.png" alt="Position Tracking (X, Y, Z) vs Time" width="800">
</p>
<p align="center"><i>Figure 3: Time-series evaluation of Position Tracking ($X, Y, Z$) over the simulation horizon.</i></p>

<p align="center">
  <img src="../pics/attitude_tracking.png" alt="Euler Angles Tracking" width="800">
</p>
<p align="center"><i>Figure 4: Real-time attitude regulation (Roll, Pitch, Yaw) dynamically commanded by the outer-loop Feedback Linearization.</i></p>

The predictive lookahead feature (9-D predictive states) in the TD3 observation space allows the quadrotor to anticipate sharp turns. Figure 4 highlights the aggressive yet stable roll ($\phi$) and pitch ($\theta$) maneuvers executed to decelerate and pivot the vehicle precisely at the trajectory's inflection points.

## 4. Control Effort and Safety Verification

A critical concern in model-free Reinforcement Learning is the generation of erratic or unbounded control commands that could damage physical actuators. The proposed architecture guarantees input-to-state safety by bounding the RL action output.

<p align="center">
  <img src="../pics/control_inputs.png" alt="Control Inputs U1, U2, U3, U4" width="800">
</p>
<p align="center"><i>Figure 5: Control inputs demonstrating smooth total thrust ($U_1$) and strictly bounded rotational torques ($U_2, U_3, U_4$).</i></p>

Figure 5 verifies that the total thrust $U_1$ (calculated algebraically via Feedback Linearization) handles the massive translational lifting force smoothly. Meanwhile, the rotational torques ($U_2, U_3, U_4$) provided by the fused NMPC and RL outputs remain strictly within the physical limits of the BLDC motors ($\Omega_{min}, \Omega_{max}$). The residual torque $\Delta\mathbf{U}_{RL}$ successfully operates within its clipped safety bounds $[-\epsilon, \epsilon]$, proving that the RL agent "fine-tunes" the mathematical baseline without ever jeopardizing the flight safety.
