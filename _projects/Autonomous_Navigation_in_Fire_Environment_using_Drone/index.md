---
layout: post
title: Autonomous Navigation in Fire Environment using Drone
description: >
    Developed an autonomous navigation framework for a micro-UAV operating in
    fire-affected environments using temperature feedback. Simulated realistic
    thermal environments and investigated reinforcement-learning-based navigation
    strategies for locating fire sources under different thermal conditions.
skills:
  - Autonomous Navigation
  - Reinforcement Learning
  - Multi-Armed Bandits
  - ROS2
  - PyroSim / FDS
  - Drone Simulation
  - Thermal Field Modeling
  - Python

main-image: /fire_drone_main.jpg
---

## Overview

This project focuses on **autonomous fire-source localization using a
micro-UAV** in an unknown indoor environment. The drone uses temperature
measurements as its primary source of information and must navigate toward the
fire source without relying on a pre-defined map of the thermal field.

The project combines **fire simulation, autonomous navigation, reinforcement
learning, and robotic control** into a single framework.

## Fire Environment Simulation

The fire scenarios were developed using **PyroSim with the Fire Dynamics
Simulator (FDS)**. Different indoor configurations were considered, including
multiple rooms, corridors, primary and secondary fire sources, and thermal
sinks.

The resulting thermal environments were used to study how different
temperature distributions affect the navigation behavior of the drone.

{% include image-gallery.html images="/fire_environment.jpg" height="400" %}

<span style="font-size: 10px">
Simulated indoor fire environment and temperature distribution.
</span>

## Autonomous Navigation

The drone estimates the direction toward the fire source from the local
**temperature gradient** and selects its next movement direction accordingly.

Three navigation strategies were implemented and compared:

- **Greedy Hill Climbing** for directly following the temperature gradient.
- **$\epsilon$-Greedy Multi-Armed Bandit** for balancing exploration and
  exploitation.
- **Upper Confidence Bound (UCB)** for uncertainty-aware directional selection.

The algorithms were evaluated under different thermal conditions, including
weak temperature gradients and environments containing misleading secondary
fire sources. The results showed that greedy hill climbing works well when a
strong temperature gradient is available, whereas exploration-based bandit
methods become more useful when the gradient is weak or contains misleading
local maxima. :contentReference[oaicite:1]{index=1} :contentReference[oaicite:2]{index=2}

## Drone Navigation

The navigation algorithms were integrated into a **ROS2-based control
architecture** and evaluated in simulation.

{% include youtube-video.html id="YOUR_FIRE_SPREAD_VIDEO_ID" autoplay="false" width="900px" %}

<span style="font-size: 10px">
Fire propagation and thermal environment generated for the navigation study.
</span>

The drone was then simulated while autonomously navigating through the
generated thermal environment toward the fire source.

{% include youtube-video.html id="YOUR_DRONE_NAVIGATION_VIDEO_ID" autoplay="false" width="900px" %}

<span style="font-size: 10px">
Autonomous drone navigation using temperature-based decision making.
</span>

## Results

The experiments demonstrated that the choice of navigation strategy depends
strongly on the structure of the thermal field. Greedy hill climbing provided
efficient localization when the temperature gradient was strong, while
$\epsilon$-greedy exploration was more effective when the drone started far
from the fire source or encountered misleading thermal maxima. :contentReference[oaicite:3]{index=3}

The project provided an end-to-end workflow combining **thermal environment
simulation, robotic decision making, reinforcement learning, and autonomous
drone navigation**.

## Key Takeaways

- Developed fire environments using **PyroSim/FDS**.
- Built a **ROS2-based autonomous navigation framework**.
- Implemented **Hill Climbing, $\epsilon$-Greedy, and UCB** navigation strategies.
- Evaluated navigation performance under different thermal configurations.
- Studied the effect of **exploration and local temperature gradients** on
  autonomous fire-source localization.
