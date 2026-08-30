# 06. Zero-Shot Generalization and Simulation Results

To validate the effectiveness, robustness, and safety of the proposed Residual RL-NMPC architecture, comprehensive simulations are conducted using the custom MATLAB environment. The primary objective is to evaluate the system's "Zero-Shot" generalization capability—its ability to strictly track highly complex, aggressive trajectories that the Reinforcement Learning agent has never encountered during its training phase.

## 1. Simulation Setup & 3D Spatial Tracking

During the training phase, the TD3 agent was exposed exclusively to a simplified 3D Spiral trajectory. For the evaluation phase, the trained policy is locked (inference mode), and the quadrotor is tasked with navigating unseen, aggressive nonlinear curves (Figure-8 and Lissajous trajectories). 

*(Note: Insert your MATLAB animation GIF here to visualize the real-time flight dynamics).*
<p align="center">
  <img src="../pics/sim/tracking_simulation.gif" alt="3D Zero-Shot Trajectory Tracking Animation" width="700">
</p>
<p align="center"><i>Figure 1: Animated simulation of the quadrotor tracking an unseen 3D trajectory in real-time.</i></p>

### 1.1. Figure-Eight Trajectory
The first evaluation subjects the quadrotor to a 3D Figure-8 trajectory, requiring simultaneous state variations across all three spatial axes.

<p align="center">
  <img src="../pics/sim/figure-8_tracking.gif" alt="3D Figure-8 Trajectory Tracking" width="800">
</p>
<p align="center"><i>Figure 2: 3D spatial tracking of the Figure-8 trajectory. The reference path is represented by the blue dashed line, and the actual RL-NMPC flight path is the solid red line.</i></p>

### 1.2. Complex Lissajous (Butterfly) Trajectory
To push the dynamic limits further, the system navigates a 3D Lissajous curve featuring extreme variations in curvature. 

<p align="center">
  <img src="../pics/sim/lissajous_tracking.gif" alt="3D Lissajous Trajectory Tracking" width="800">
</p>
<p align="center"><i>Figure 3: 3D spatial tracking of the highly complex Lissajous (Butterfly) curve.</i></p>

**Analysis:** As observed in Figures 2 and 3, the actual flight path (red) perfectly traces the complex reference curve (blue). Traditional MPC controllers typically suffer from spatial drift at these sharp corners due to unmodeled aerodynamic drag. However, the RL-NMPC architecture entirely eliminates these model mismatches. The residual control signal ($\Delta\mathbf{U}_{RL}$) successfully neutralizes the nonlinearities, preventing any divergent oscillations or corner-cutting.

## 2. Quantitative State Error Analysis

To dissect the performance, the time-series transient responses of both translational and rotational dynamics are analyzed.

<p align="center">
  <img src="../pics/image_5b4e8d.png" alt="Position Tracking (X, Y, Z) vs Time" width="800">
</p>
<p align="center"><i>Figure 4: Time-series evaluation of Position Tracking ($X, Y, Z$) amplitudes.</i></p>

**Position Analysis:** Figure 4 demonstrates a flawless amplitude match across the $X, Y,$ and $Z$ axes. Crucially, there is zero phase lag or overshoot at the trajectory's peaks and troughs. This confirms that the 9-dimensional predictive lookahead within the TD3 observation space successfully enables the agent to anticipate sharp turns before the quadrotor reaches them.

<p align="center">
  <img src="../pics/image_5b4e4d.png" alt="Euler Angles Tracking" width="800">
</p>
<p align="center"><i>Figure 5: Real-time attitude regulation (Roll, Pitch, Yaw) dynamically commanded to execute the trajectory.</i></p>

**Attitude Analysis:** Figure 5 highlights the aggressive maneuvering required for zero-shot tracking. The Roll ($\phi$) and Pitch ($\theta$) angles exhibit high-frequency oscillations—a physical necessity for continuous banking and acceleration. Meanwhile, the Yaw angle ($\psi$) smoothly tracks the tangential heading of the reference path, handling the $\pm\pi$ trigonometric wrap-arounds without causing erratic mechanical jerks.

## 3. Control Effort and Actuator Safety Verification

A primary concern in model-free Reinforcement Learning is the generation of unbounded control commands that could saturate or damage physical BLDC motors.

<p align="center">
  <img src="../pics/image_5b4e92.png" alt="Control Inputs U1, U2, U3, U4" width="800">
</p>
<p align="center"><i>Figure 6: Generated Control Inputs. $U_1$ represents the total lifting thrust, while $U_2, U_3, U_4$ represent the bounded rotational torques.</i></p>

**Control Analysis:** Figure 6 mathematically verifies the safety and efficiency of the proposed architecture. 
* The total thrust ($U_1$) remains highly stable around $7$ N to counteract gravity, with only minor adjustments to regulate the $Z$-axis altitude.
* The rotational torques ($U_2, U_3, U_4$), which contain the fused NMPC and RL signals, operate at a high frequency but are strictly confined within a narrow amplitude envelope (predominantly between $-0.05$ and $0.05$ Nm). 

This proves that the RL agent strictly operates as a "residual compensator." It fine-tunes the baseline mathematical model to achieve perfect tracking without ever generating extreme values that would jeopardize flight safety or saturate the physical actuators.
