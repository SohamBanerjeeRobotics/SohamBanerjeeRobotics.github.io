---
layout: post
title: Virtual-to-Real Mapless Navigation via Deep Reinforcement Learning
description: >
    Implemented an asynchronous actor-critic deep reinforcement learning (ADDPG) 
    framework for continuous control of differential-drive mobile robots. 
    Achieved mapless navigation using 10-dimensional sparse range findings and target 
    coordinates, bridging the sim-to-real gap without fine-tuning.
skills:
  - Deep Reinforcement Learning
  - Asynchronous DDPG (ADDPG)
  - Continuous Control
  - Sim-to-Real Transfer
  - Mobile Robotics
  - ROS /  Pybullet Simulation
  - Python
  - PyTorch / TensorFlow

main-image: /mapless_navigation_front.jpg
---

## Overview

Traditional mobile robot navigation systems, such as SLAM-based architectures, rely on building dense obstacle maps from expensive high-resolution sensors . These methods are computationally intensive and slow to update dynamically . 

This project implements an end-to-end mapless motion planner for differential-drive mobile robots using continuous Deep Reinforcement Learning . Based on **Asynchronous Deterministic Policy Gradient (ADDPG)**, the model takes a highly compressed 14-dimensional state representation—comprising 10 sparse laser range samples, 2D relative target positions in polar coordinates, and previous velocity commands—and outputs smooth continuous linear and angular velocity commands . Trained entirely within a simulated environment ( Pybullet), the learned policy directly transfers to physical robotic hardware (Kobuki-based TurtleBot) without any real-world fine-tuning .
This whole idea is taken from the paper : Virtual-to-Real Deep Reinforcement Learning: Continuous Control of Mobile Robots for Mapless Navigation by *Tai et al*.
---

## State & Action Space Formulation

The mapless navigation task is modeled as a continuous Markov Decision Process (MDP) . The input state vector $s_t \in \mathbb{R}^{14}$ is constructed at each control step by concatenating three distinct observations :

$$s_t = \begin{bmatrix} x_t & v_{t-1} & p_t \end{bmatrix}$$ 

- **Sparse Laser Readings ($x_t \in \mathbb{R}^{10}$):** 10-dimensional sparse laser range findings sampled at fixed angles between $-90^\circ$ and $+90^\circ$, normalized to $[0, 1]$ .
- **Previous Control Inputs ($v_{t-1} \in \mathbb{R}^2$):** Continuous linear and angular velocities from the preceding step $(v_{t-1}, \omega_{t-1})$ .
- **Relative Target Position ($p_t \in \mathbb{R}^2$):** Target destination represented in 2D polar coordinates (distance $r_t$ and relative heading angle $\phi_t$) with respect to the robot's local frame .

The action vector $a_t = [v_t, \omega_t]^T \in \mathbb{R}^2$ commands continuous velocities bounded by physical vehicle constraints :
- Linear Velocity: $v_t \in [0, 0.5]\text{ m/s}$ 
- Angular Velocity: $\omega_t \in [-1.0, 1.0]\text{ rad/s}$ 

---

## Continuous Control via Asynchronous DDPG (ADDPG)

To accelerate sample generation without requiring multiple parallel simulation environments, sample collection is decoupled into dedicated asynchronous background threads while maintaining a centralized off-policy replay buffer $\mathcal{D}$ .

<div style="text-align: center; margin: 20px 0;">
  <img src="/addpg_pseudocode.png" alt="Asynchronous DDPG Algorithm Pseudocode" style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);" />
  <p style="font-size: 12px; color: #666; margin-top: 8px;"><em>Figure 1: Asynchronous DDPG (ADDPG) training algorithm pseudocode.</em></p>
</div>

### Loss Formulations

The **Critic Network** $Q(s, a | \theta^Q)$ is optimized by minimizing the Mean Squared Bellman Error (MSBE) across mini-batches sampled from $\mathcal{D}$ :

$$L(\theta^Q) = \frac{1}{M} \sum_{i=1}^{M} \left( Q(s_i, a_i | \theta^Q) - y_i \right)^2$$ 

$$y_i = r_i + \gamma Q'\left(s'_{i}, \mu'(s'_{i} | \theta^{\mu'}) \middle| \theta^{Q'}\right)$$ 

The **Actor Network** $\mu(s | \theta^\mu)$ is updated using the sampled Deterministic Policy Gradient :

$$\nabla_{\theta^\mu} J \approx \frac{1}{M} \sum_{i=1}^{M} \left. \nabla_a Q(s, a | \theta^Q) \right|_{a=\mu(s_i)} \nabla_{\theta^\mu} \mu(s_i | \theta^\mu)$$ 

---

## Reward Function Design

The step reward function $r(s_t, a_t)$ guides target-reaching behavior while penalizing obstacle proximity :

$$r(s_t, a_t) = \begin{cases} r_{\text{arrive}} & \text{if } d_t < c_d \text{ (Target Reached)} \\ r_{\text{collision}} & \text{if } \min(x_t) < c_o \text{ (Collision Hazard)} \\ c_r (d_{t-1} - d_t) & \text{otherwise (Progress Reward)} \end{cases}$$ 

| Hyperparameter | Description | Value |
|---|---|---|
| $r_{\text{arrive}}$ | Terminal success reward  | $+15.0$ |
| $r_{\text{collision}}$ | Terminal failure penalty  | $-15.0$ |
| $c_r$ | Distance reduction scaling multiplier  | $2.5$ |
| $c_d$ | Terminal arrival tolerance radius  | $0.25\text{ m}$ |
| $c_o$ | Minimum clearance distance threshold  | $0.15\text{ m}$ |

---

## Network Architecture & Pseudocode

The policy network processes the 14-dimensional state vector through parallel dense layers before generating bounded velocity signals .

> **Source Code:** View full repository details on GitHub at [SohamBanerjeeRobotics](https://github.com/SohamBanerjeeRobotics).

<div style="text-align: center; margin: 20px 0;">
  <img src="/network_architecture_code.png" alt="Actor-Critic Architecture Pseudocode Image" style="max-width: 100%; height: auto; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);" />
  <p style="font-size: 12px; color: #666; margin-top: 8px;"><em>Figure 2: Network structure and layer configuration breakdown.</em></p>
</div>

---

## Visualizations & Demonstration Video

The trained ADDPG policy was evaluated across complex simulated arenas in  Pybullet .

{% include youtube-video.html id="sNinZlbigUI" autoplay="false" width="900px" %}
<span style="font-size: 10px">Simulation Demonstration: Continuous control mapless navigation in Pybullet indoor environments.</span>

---

## Experimental & Benchmark Results

The performance of the continuous Deep-RL agent was benchmarked against traditional map-based navigation stack components (`Move Base`) in simulated test layouts and real-world deployment evaluation :

| Planner Model | Map Required | Max Control Frequency | Execution Stability | Sim-to-Real Transfer |
|---|---|---|---|---|
| **Move Base (Full Laser)**  | Yes  | $\approx 14\text{ Hz}$  | Optimal Path  | N/A (Requires Full Map)  |
| **Move Base (10-D Sparse)**  | Yes  | $\approx 14\text{ Hz}$  | Failed at narrow corridors  | Failed (Cost-map errors)  |
| **Deep-RL (Env-1 Policy)**  | No  | $\approx 100\text{ Hz}$  | Collision-Free  | Poor Generalization  |
| **Deep-RL (Env-2 Policy)**  | **No**  | $\mathbf{\approx 100\text{ Hz}}$  | **Smooth & Highly Robust**  | **100% Zero-Shot Success**  |

### Critical Performance Metrics

- **Control Latency:** Inference time per action query is $\approx 1\text{ ms}$, achieving a **$7\times$ speedup** over traditional local trajectory planners ($\approx 14\text{ Hz}$) .
- **Sensor Reduction:** Functionally replaces dense 2D LiDAR scans with 10 discrete angular distance beams without sacrificing obstacle avoidance capabilities .
- **Emergent Behaviors:** Autonomous rotational recovery maneuvers naturally emerged from the trained policy when navigating tightly constrained passages .

---

## Key Takeaways

- Successfully developed an end-to-end mapless navigation pipeline leveraging continuous Deep-RL via Asynchronous DDPG (ADDPG) .
- Proved that an abstracted 14-dimensional state representation effectively bridges the simulation-to-reality gap without fine-tuning on real hardware .
- High inference rates ($\approx 100\text{ Hz}$) enable reactive collision avoidance under real-time execution constraints .
- Future work involves integrating Recurrent Neural Network layers (LSTM/GRU) to mitigate trajectory tortuosity caused by partial observability .
