---
layout: post
title: Motion Planning and Coordination for Autonomous Vehicles (TurtleBot)
description: >
    Implemented configuration space (C-space) modeling, grid-based search (A*), 
    and sampling-based motion planning (RRT) for a nonholonomic differential-drive 
    TurtleBot. Benchmarked planner efficiency, path length, and obstacle clearance, 
    and resolved sim-to-real deployment challenges using AMCL localization and 
    Pure Pursuit path tracking.
skills:
  - Motion Planning
  - Configuration Space (C-Space) Modeling
  - A* Search Algorithm
  - Rapidly-Exploring Random Trees (RRT)
  - Differential-Drive Kinematics
  - ROS / ROS2
  - AMCL Localization
  - Pure Pursuit Control
  - Sim-to-Real Gap Mitigation
  - Python

main-image: /turtlebot_main.jpeg
---

## Overview

In mobile robotics, motion planning is formulated in configuration space (C-space) rather than directly in the physical workspace[cite: 4]. The configuration space represents every possible pose of a robot as a point in a higher-dimensional space[cite: 4]. For the differential-drive TurtleBot, C-space modeling provides a rigorous mathematical framework for navigation, planning, and control[cite: 4].

This project implements and evaluates nonholonomic trajectory planning strategies for a differential-drive TurtleBot operating in obstacle-laden environments[cite: 4]. We contrast deterministic grid-based planning (**A\***) against probabilistic sampling-based planning (**RRT**), dynamic obstacle inflation via Minkowski sums, and robust trajectory execution using ROS-integrated localization and trajectory tracking controllers[cite: 4].

---

## Configuration Space & Kinematic Modeling

### Configuration Space of TurtleBot
The TurtleBot operates on a planar surface and can both translate and rotate[cite: 4]. Its configuration is defined by three variables[cite: 4]:

$$q = (x, y, \theta)$$

where $x$ and $y$ represent the robot's position in the plane, and $\theta$ represents the robot's orientation (heading angle)[cite: 4]. Thus, the configuration space is $\mathcal{C} = \mathbb{R}^2 \times S^1$[cite: 4]. The position lies in two-dimensional Euclidean space ($\mathbb{R}^2$), while orientation lies on a circle ($S^1$), making the configuration manifold mathematically equivalent to the Lie group $SE(2)$[cite: 4].

### Kinematic Model & Constraints
The TurtleBot is a differential-drive robot subject to nonholonomic constraints[cite: 4]:

$$\dot{x} = v \cos(\theta)$$

$$\dot{y} = v \sin(\theta)$$

$$\dot{\theta} = \omega$$

The system is subject to the continuous zero-lateral-velocity constraint:

$$\dot{y}\cos(\theta) - \dot{x}\sin(\theta) = 0$$

This constraint prevents instantaneous sideways motion; the robot must reorient itself before changing direction[cite: 4]. Furthermore, because $\mathcal{C} = \mathbb{R}^2 \times S^1$ is a smooth three-dimensional manifold with angular periodicity, special consideration is required during state sampling and interpolation[cite: 4]. Despite these nonholonomic constraints, the system is small-time locally controllable, allowing arbitrary poses in free space to be reached via sequences of forward motions and rotations[cite: 4].

---

## Configuration Space Obstacles & Free Space Construction

Let $\mathcal{O}$ represent obstacle regions in the physical workspace[cite: 4]. The corresponding C-space obstacle region $\mathcal{C}_{\text{obs}}$ is defined as[cite: 4]:

$$\mathcal{C}_{\text{obs}} = \{q \in \mathcal{C} \mid \text{the robot at configuration } q \text{ intersects } \mathcal{O}\}$$

Assuming a circular disk approximation for the TurtleBot:
- Workspace obstacles are inflated by the robot's radius using Minkowski sum operations[cite: 4].
- $\mathcal{C}_{\text{obs}} = (\text{inflated workspace obstacles}) \times S^1$[cite: 4].
- Free configuration space is defined as $\mathcal{C}_{\text{free}} = \mathcal{C} \setminus \mathcal{C}_{\text{obs}}$[cite: 4].

The trajectory generation goal is to establish a continuous trajectory $\gamma(t) \in \mathcal{C}_{\text{free}}$ for $0 \le t \le 1$ such that $\gamma(0) = q_{\text{start}}$ and $\gamma(1) = q_{\text{goal}}$, while respecting differential kinodynamic limits[cite: 4].

---

## Decision-Making & Path Planning Algorithms

Two core spatial path-planning strategies were implemented and benchmarked[cite: 4]:

### 1. A* Planner (Grid-Based Search)
A deterministic graph-search planner using cost function evaluation $f(n) = g(n) + h(n)$, where $g(n)$ is the exact path cost from start to node $n$, and $h(n)$ is an admissible heuristic[cite: 4]. Implemented in `astar_planner.py`, it guarantees resolution optimality, generating shortest-path grid trajectories[cite: 4].

{% include image-gallery.html images="/astar_planner_code.jpeg" height="400" %}
<span style="font-size: 10px">Source Code: Implementation of deterministic A* search planner (`astar_planner.py`)[cite: 4].</span>

### 2. Rapidly-Exploring Random Tree (RRT Planner)
A probabilistic, sampling-based planner implemented in `RRT_Planner.py`[cite: 4]. RRT rapidly explores high-dimensional C-spaces by incrementally growing a search tree toward randomly sampled states $q_{\text{rand}} \in \mathcal{C}_{\text{free}}$[cite: 4]. It is probabilistically complete and explores open spaces quickly, though raw paths tend to be non-optimal and jagged[cite: 4].

{% include image-gallery.html images="/rrt_planner_code.jpeg" height="400" %}
<span style="font-size: 10px">Source Code: Implementation of sampling-based RRT planner (`RRT_Planner.py`)[cite: 4].</span>

---

## Visualizations & Experimental Demos

The generated global paths were executed on the TurtleBot platform via ROS integration[cite: 4].

{% include youtube-video.html id="9YX_3wYgB8M" autoplay="false" width="900px" %}
<span style="font-size: 10px">Live Demonstration: A* Path Planner running on TurtleBot[cite: 4].</span>

{% include youtube-video.html id="dD21LSMiPJs" autoplay="false" width="900px" %}
<span style="font-size: 10px">Live Demonstration: RRT Path Planner running on TurtleBot[cite: 4].</span>

{% include youtube-video.html id="SIMULATION_VIDEO_ID" autoplay="false" width="900px" %}
<span style="font-size: 10px">Simulation Demonstration: Full C-Space Navigation Trajectory in Gazebo/Webots Environment.</span>

---

## Comparative Performance & Benchmark Results

Performance metrics were collected across 5 test runs per planner in the target environment[cite: 4]:

| Metric | A* Planner (Average) | RRT Planner (Average) |
|---|---|---|
| **Time to Goal** | **42.5 seconds** | 48.2 seconds |
| **Path Length** | **8.42 meters** | 10.15 meters |
| **Min. Distance to Obstacle** | 0.12 meters | **0.25 meters** |
| **Success Rate** | **100%** | 80% (Occasional failures) |

---

## Sim-to-Real Gap Analysis & Hardware Integration

Deploying planned paths to physical hardware introduced discrepancies between simulation assumptions and real-world execution[cite: 4]:

- **Wheel Slip & Kinematic Drift:** Real motor wheel slip degraded dead-reckoning accuracy[cite: 4]. This was corrected by integrating Adaptive Monte Carlo Localization (**AMCL**) over laser scan data[cite: 4].
- **Obstacle Safety Margins:** Imperfection in real-world wall alignment required dynamic tuning of the `inflation_radius` within the costmap generation node[cite: 4].
- **Processing Latency:** Processing overhead on the Raspberry Pi onboard unit created execution lag[cite: 4]. This was compensated by dynamically adjusting the lookahead distance within the **Pure Pursuit** path-tracking controller[cite: 4].

---

## Key Takeaways

- Developed analytical C-space models incorporating nonholonomic $SE(2)$ differential constraints for TurtleBot navigation[cite: 4].
- Implemented and benchmarked grid-search (**A\***) vs. sampling-based (**RRT**) motion planners[cite: 4].
- Demonstrated that **A\*** yields optimal path length and consistent convergence, whereas **RRT** offers faster exploration in open regions at the expense of path smoothness[cite: 4].
- Overcame sim-to-real transfer errors by combining C-space Minkowski obstacle inflation, AMCL pose tracking, and adaptive Pure Pursuit trajectory control[cite: 4].
