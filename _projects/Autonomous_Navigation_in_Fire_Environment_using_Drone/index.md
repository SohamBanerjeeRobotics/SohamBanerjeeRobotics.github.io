---
layout: post
title: Autonomous Navigation in Fire Environment using Drone
description: >
  Developed an autonomous navigation framework for a Crazyflie 2.1 micro-UAV operating in
  fire-affected environments using temperature feedback. Simulated realistic
  thermal environments via Pyrosim/FDS and analytical models, and evaluated 
  reinforcement-learning strategies (Greedy, Epsilon-Greedy, UCB) for fire source 
  localization across single-source, multi-source, and sink-distorted thermal landscapes.
skills:
  - Autonomous Navigation
  - Reinforcement Learning
  - Multi-Armed Bandits (Epsilon-Greedy, UCB)
  - ROS / ROS2
  - Webots Simulation
  - PyroSim / FDS
  - Thermal Field Modeling
  - Python
  - Proportional Closed-Loop Control

main-image: /fire_drone_main.jpeg
---

## Overview

Autonomous navigation in indoor fire-affected environments presents severe challenges due to extreme temperatures, dense smoke, and non-linear thermal dynamics. Traditional optical sensors like LiDAR or visual cameras often fail or become unreliable in low-visibility, smoke-filled scenarios. Lightweight micro-drones—such as the **Crazyflie 2.1**—offer a safe, non-intrusive alternative for indoor exploration, provided they can interpret local environmental cues for decision-making.

This project presents a closed-loop **reinforcement-learning-based navigation framework** integrated within the **Webots simulation platform and ROS middleware**. The micro-UAV autonomously locates high-temperature regions using only **point-wise local temperature feedback**, operating without prior map knowledge of the thermal field. Beyond fire localization, this framework extends naturally to tracking other spatial-temporal continuous fields, such as chemical leakages or hazardous gas sources.

---

## Architecture & System Pipeline

The system runs in a continuous feedback loop linking physical fire dynamics with real-time drone control:

1. **Fire Simulation Node:** Computes the temperature $T(x,y)$ at the drone's current GPS coordinates and publishes it over ROS topics.
2. **Reinforcement Learning Node:** Receives local temperature readings, evaluates directional gradients, and selects target waypoints based on the active exploration policy.
3. **Control & Execution:** A proportional position controller converts target position coordinates into velocity commands, moving the drone inside Webots until position error falls below threshold.

{% include image-gallery.html images="/fire_architecture_block_diagram.jpeg" height="400" %}
<span style="font-size: 10px">System flow: ROS-Webots closed-loop integration with Pyrosim and RL decision nodes.</span>

---

## Thermal Environment Modeling

To evaluate algorithm robustness across diverse thermal conditions, a dual-generation pipeline was constructed:

### 1. Physics-Based Fire Dynamics (PyroSim / FDS)
Realistic indoor fire scenarios were modeled using **PyroSim with the Fire Dynamics Simulator (FDS)** backend. Mesh-resolved node data is exported as spatial CSV files and parsed by a custom ROS node to stream real-time temperature feedback during drone exploration.

### 2. Analytical Thermal Fields (Superposition Models)
To complement Pyrosim with high-speed, dynamic test cases, analytical thermal fields were built using attenuation-based exponential propagation equations inspired by indoor fire fluid dynamics:

$$\frac{\Delta T_{x}}{\Delta T_{\text{ref}}} = f_{1}(N) e^{f_{2}(N) K_{1} x}$$

Using the **principle of superposition**, secondary fire sources and thermal cold sinks were modeled to generate challenging, non-symmetric thermal landscapes.

{% include image-gallery.html images="/fire_environment.jpeg" height="400" %}
<span style="font-size: 10px">Simulated indoor fire environment and temperature distribution.</span>

---

## Decision-Making & Navigation Algorithms

Three spatial decision-making strategies were implemented and evaluated:

### 1. Greedy Hill Climbing
The drone evaluates local temperature gradients by scanning candidate points on a perimeter circle of radius $r$ around its current coordinates. It strictly moves step-by-step along the vector of maximum thermal ascent.

{% include image-gallery.html images="/algorithm_hill_climbing_pseudocode.jpeg" height="350" %}
<span style="font-size: 10px">Algorithm 1: Hill Climbing Algorithm (Greedy Approach).</span>

### 2. Multi-Armed Bandit (*epsilon*-Greedy Approach)
To avoid getting trapped in zero-gradient or local maxima regions, the agent selects a random direction with probability $\epsilon$ and exploits the highest empirical temperature gradient direction with probability $1 - \epsilon$.

{% include image-gallery.html images="/algorithm_egreedy_pseudocode.jpeg" height="400" %}
<span style="font-size: 10px">Algorithm 2: Multi-Armed Bandit Algorithm (*epsilon*-Greedy Approach).</span>

### 3. Multi-Armed Bandit (Upper Confidence Bound - UCB)
Unlike *epsilon*-greedy, UCB balances exploration and exploitation deterministically by computing an upper confidence bound for each directional arm:

$$\text{UCB}_i = Q(i) + c \sqrt{\frac{2 \ln T_s}{N_i}}$$

Here $Q(i)$ is the mean temperature gradient along arm $i$, $c$ is the exploration constant, $T_s$ is total steps, and $N_i$ is arm visit count. An **adaptive update rule** dynamically scales $c$ based on the variance of arm visits to avoid premature convergence.

### Convergence & Adaptive Update Mechanics

{% include image-gallery.html images="/algorithm_convergence_and_adaptive_update.jpeg" height="400" %}
<span style="font-size: 10px">Algorithm 3 & 4: Convergence Criteria and Adaptive Update Rule of Exploration Constant.</span>

---

## Drone Navigation & Visualizations

The navigation algorithms were integrated into a **ROS2-based control architecture** and evaluated in Webots simulation.

{% include youtube-video.html id="dBY5fK-LCBU" autoplay="false" width="900px" %}
<span style="font-size: 10px">Fire propagation and thermal environment generated for the navigation study.</span>

The drone was then simulated while autonomously navigating through the generated thermal environment toward the fire source.

{% include youtube-video.html id="iQTsDOnC4lY" autoplay="false" width="900px" %}
<span style="font-size: 10px">Autonomous drone navigation using temperature-based decision making.</span>

---

## Simulation & Benchmark Results

The algorithms were benchmarked in a simulated 25m × 25m indoor warehouse environment (ambient baseline $20^\circ\text{C}$):

| Scenario | Algorithm | Distance Traveled (m) | Mean Squared Error | Final Position Error (m) | Convergence Status |
|---|---|---|---|---|---|
| **Single Source (Near)** | Greedy Hill Climbing | **39.84** | **0.0065** | **0.153** | Converged |
| | *epsilon*-Greedy Bandit | 68.29 | 0.0227 | 0.707 | Converged |
| | UCB Bandit | 23.02 | 0.0170 | 2.915 | Partial/Exploratory |
| **Far Source (Flat Gradient)** | Greedy Hill Climbing | — | — | — | **Failed (No Conv.)** |
| | *epsilon*-Greedy Bandit | **55.05** | **0.0318** | **0.860** | Converged |
| | UCB Bandit | 27.87 | 0.0867 | 1.803 | Converged |
| **Thermal Sinks Distortions** | Greedy Hill Climbing | **61.52** | **0.1783** | **0.262** | Converged |
| | *epsilon*-Greedy Bandit | 79.97 | 0.2741 | 0.823 | Converged |
| | UCB Bandit | 87.64 | 0.2331 | 1.231 | Converged |

---

## Key Takeaways

- Developed indoor fire environments using **PyroSim / FDS**.
- Built a **ROS/ROS2 & Webots closed-loop navigation stack**.
- Implemented **Greedy Hill Climbing, *epsilon*-Greedy, and Adaptive UCB** bandit algorithms.
- **Unimodal / Strong Gradients:** Greedy hill climbing achieves optimal efficiency and minimal distance when steep local gradients exist.
- **Flat Gradients & Local Traps:** Multi-armed bandit strategies (especially *epsilon*-greedy) prevent total navigation failure in flat thermal fields or regions containing secondary hot spots.
- Validated that lightweight micro-quadrotors (Crazyflie 2.1) can localize thermal targets using only point temperature measurements without vision/LiDAR payloads.
