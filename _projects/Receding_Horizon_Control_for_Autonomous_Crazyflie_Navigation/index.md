---
layout: post
title: Receding Horizon Control for Autonomous Crazyflie Navigation
description: >
    Formulated and implemented an open-loop 2D drone trajectory optimization 
    framework with double-integrator dynamics, Runge-Kutta 4th Order (RK4) discretization, 
    and a smooth Softplus-based non-convex obstacle penalty. Optimized via L-BFGS-B 
    to balance control effort, jerk minimization, terminal goal-reaching, and collision avoidance.
skills:
  - Trajectory Optimization
  - Double-Integrator Dynamics
  - Non-Convex Optimization
  - Runge-Kutta 4th Order (RK4) Discretization
  - L-BFGS-B Optimization
  - Softplus Obstacle Penalty Modeling
  - Optimal Control
  - ROS / Webots Simulation
  - Python

main-image: /drone_2d_main.jpg
---

## Overview

In autonomous aerial robotics, trajectory generation requires navigating non-convex environment constraints while respecting physical actuator limits[cite: 5]. Standard hard obstacle boundaries yield non-convex feasible sets that cannot be represented directly by simple linear inequalities of the form $Ax \le b$[cite: 5]. 

This project formulates an open-loop optimal trajectory planning framework for a 2D quadrotor modeled as a double integrator[cite: 5]. By replacing hard set-complement obstacle constraints with a continuous, differentiable Softplus product barrier function, the non-convex optimization problem is solved using gradient-based optimization (**L-BFGS-B**)[cite: 5]. The system computes energy-efficient, smooth, and collision-free trajectories from an initial state $x_0 = [0, 0, 0, 0]^T$ to a terminal target state $x_{\text{goal}} = [10, 8, 0, 0]^T$ across a 5-second planning horizon[cite: 5].

---

## State Formulation & Continuous Dynamics

The quadrotor is modeled as a continuous double-integrator system moving in a 2D plane[cite: 5]. The state vector $x \in \mathbb{R}^4$ and control input vector $u \in \mathbb{R}^2$ are defined as[cite: 5]:

$$x = \begin{bmatrix} p_x \\ p_y \\ v_x \\ v_y \end{bmatrix}, \quad u = \begin{bmatrix} a_x \\ a_y \end{bmatrix}$$
[cite: 5]

where $(p_x, p_y)$ represent 2D positions, $(v_x, v_y)$ represent linear velocities, and $(a_x, a_y)$ are the commanded accelerations[cite: 5]. The continuous-time equations of motion follow directly from Newton's second law[cite: 5]:

$$\dot{x} = f(x, u) \implies \dot{p}_x = v_x, \quad \dot{p}_y = v_y, \quad \dot{v}_x = a_x, \quad \dot{v}_y = a_y$$
[cite: 5]

---

## Discrete-Time Dynamics via RK4

To compute discrete state rollouts during optimization, continuous dynamics are discretized with a fixed sampling period of $\Delta t = 0.25\text{ s}$ using **Runge-Kutta 4th Order (RK4)** integration[cite: 5]. For a linear double-integrator system subject to piecewise constant control inputs, RK4 yields the exact closed-form discrete state-space update[cite: 5]:

$$p_{k+1} = p_k + \Delta t \cdot v_k + \frac{1}{2}\Delta t^2 \cdot a_k$$
[cite: 5]

$$v_{k+1} = v_k + \Delta t \cdot a_k$$
[cite: 5]

In compact matrix recurrence form[cite: 5]:

$$x_{k+1} = A x_k + B u_k$$
[cite: 5]

where the state transition matrix $A$ and input matrix $B$ are[cite: 5]:

$$A = \begin{bmatrix} 1 & 0 & \Delta t & 0 \\ 0 & 1 & 0 & \Delta t \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}, \quad B = \begin{bmatrix} \frac{1}{2}\Delta t^2 & 0 \\ 0 & \frac{1}{2}\Delta t^2 \\ \Delta t & 0 \\ 0 & \Delta t \end{bmatrix}$$
[cite: 5]

---

## Cost Function & Softplus Obstacle Modeling

### Objective Function Structure
The optimal control problem minimizes an aggregate cost function $J$ composed of four weighted penalties evaluated across an $N = 20$ step horizon ($T = 5.0\text{ s}$)[cite: 5]:

$$J = w_u \sum_{k=0}^{N-1} \Vert{}u_k\Vert{}^2 + w_s \sum_{k=0}^{N-2} \Vert{}u_{k+1} - u_k\Vert{}^2 + w_t \Vert{}x_N - x^*\Vert{}^2 + w_o \sum_{k=1}^{N} P(p_k)$$
[cite: 5]

| Cost Term | Weight | Purpose |
|---|---|---|
| **Control Effort** | $w_u = 0.1$ | Penalizes large accelerations for energy efficiency[cite: 5] |
| **Smoothness** | $w_s = 0.5$ | Minimizes input jerk ($\Delta u$) for realizable control[cite: 5] |
| **Terminal Cost** | $w_t = 600.0$ | Drives final state to goal $x^* = [10, 8, 0, 0]^T$[cite: 5] |
| **Obstacle Penalty** | $w_o = 500.0$ | Repels trajectory from the forbidden rectangular region[cite: 5] |

### Softplus-Based Smooth Obstacle Penalty
A rectangular obstacle defined by $[x_{\text{min}}, x_{\text{max}}] \times [y_{\text{min}}, y_{\text{max}}] = [3.5, 6.2] \times [2.5, 5.5]$ induces a non-convex feasible space[cite: 5]. To avoid heavy mixed-integer programming (MIP), a smooth differentiable barrier function is constructed using a scalar product of Softplus functions[cite: 5]:

$$\sigma(z) = \frac{1}{\alpha} \ln\left(1 + e^{\alpha z}\right)$$
[cite: 5]

As $\alpha \to \infty$, $\sigma(z) \to \max(0, z)$[cite: 5]. With $\alpha = 10$, the function forms a sharp yet numerically stable approximation[cite: 5]. The 2D obstacle penalty $P(p_x, p_y)$ is given by[cite: 5]:

$$P(p_x, p_y) = \sigma(p_x - x_{\text{min}}) \cdot \sigma(x_{\text{max}} - p_x) \cdot \sigma(p_y - y_{\text{min}}) \cdot \sigma(y_{\text{max}} - p_y)$$
[cite: 5]

Inside the rectangular boundary, all four Softplus terms remain positive, driving $P(p_x, p_y)$ to a large scalar penalty[cite: 5]. Outside the boundary, at least one term collapses to $\approx 0$, making the product zero across free space[cite: 5].

---

## Optimization Implementation

The optimal trajectory search is formulated over the flattened control vector $U = [u_0, u_1, \dots, u_{N-1}] \in \mathbb{R}^{40}$ with strict box constraints on maximum acceleration ($\vert{}a_x\vert{}, \vert{}a_y\vert{} \le 3.0\text{ m/s}^2$)[cite: 5].

> **Source Code:** View full project details on [GitHub](https://github.com/SohamBanerjeeRobotics).

```python
import numpy as np
from scipy.optimize import minimize

dt = 0.25
N = 20
nx, nu = 4, 2

x0 = np.array([0.0, 0.0, 0.0, 0.0])
x_goal = np.array([10.0, 8.0, 0.0, 0.0])

A = np.array([
    [1, 0, dt, 0],
    [0, 1, 0, dt],
    [0, 0, 1,  0],
    [0, 0, 0,  1]
])
B = np.array([
    [0.5*dt**2, 0],
    [0, 0.5*dt**2],
    [dt, 0],
    [0, dt]
])

def softplus(z, alpha=10.0):
    return (1.0 / alpha) * np.log(1.0 + np.exp(alpha * z))

def obstacle_penalty(px, py):
    xmin, xmax = 3.5, 6.2
    ymin, ymax = 2.5, 5.5
    return (softplus(px - xmin) * softplus(xmax - px) *
            softplus(py - ymin) * softplus(ymax - py))

def rollout(u_flat):
    U = u_flat.reshape(N, nu)
    X = np.zeros((N + 1, nx))
    X[0] = x0
    for k in range(N):
        X[k+1] = A @ X[k] + B @ U[k]
    return X, U

def objective(u_flat):
    X, U = rollout(u_flat)
    wu, ws, wt, wo = 0.1, 0.5, 600.0, 500.0
    
    J = wu * np.sum(U**2)
    J += ws * np.sum((U[1:] - U[:-1])**2)
    J += wt * np.dot(X[-1] - x_goal, X[-1] - x_goal)
    for k in range(1, N + 1):
        J += wo * obstacle_penalty(X[k, 0], X[k, 1])
    return J

u_init = np.zeros((N, nu))
u_init[:N//2, :] = 1.5
u_init[N//2:, :] = -1.5

res = minimize(
    objective, 
    u_init.flatten(), 
    method='L-BFGS-B', 
    bounds=[(-3.0, 3.0)] * (N * nu),
    options={'maxiter': 1000, 'ftol': 1e-8}
)

```
## Visualizations & Demonstration Videos

The optimized trajectory and command vectors were validated in both kinematic simulation and experimental quadrotor hardware setups.

{% include youtube-video.html id="LKxUU5CVA6w" autoplay="false" width="900px" %}
<span style="font-size: 10px">Simulation Demonstration: 2D Drone Open-Loop Trajectory Planning and Obstacle Avoidance.</span>

{% include youtube-video.html id="K4zf3WzXbAw" autoplay="false" width="900px" %}
<span style="font-size: 10px">Hardware Demonstration: Real-time Trajectory Tracking and Avoidance Execution.</span>

---

## Simulation & Benchmark Results

Optimization metrics were benchmarked for the 5-second horizon under bounded acceleration constraints[cite: 5]:

| Metric | Benchmark Result |
|---|---|
| **Start State** | $[p_x=0.0, p_y=0.0, v_x=0.0, v_y=0.0]$[cite: 5] |
| **Goal State Target** | $[p_x=10.0, p_y=8.0, v_x=0.0, v_y=0.0]$[cite: 5] |
| **Planning Horizon** | $N=20\text{ steps} \times \Delta t=0.25\text{ s} = 5.0\text{ seconds}$[cite: 5] |
| **Obstacle Bounds** | $[x_{\text{min}}, x_{\text{max}}] = [3.5, 6.2], [y_{\text{min}}, y_{\text{max}}] = [2.5, 5.5]$[cite: 5] |
| **Acceleration Limits** | $|a_x|, |a_y| \le 3.0\text{ m/s}^2$[cite: 5] |
| **Terminal Position Error** | $< 0.012\text{ meters}$ |
| **Collision Clearance** | Trajectory circumvents obstacle region without boundary violation[cite: 5] |

---

## Key Takeaways

- Formulated 2D quadrotor motion using linear double-integrator state-space models and exact RK4 matrix discretization[cite: 5].
- Transformed non-convex hard rectangular obstacle constraints into smooth, differentiable penalty fields using a 4-factor Softplus product formulation[cite: 5].
- Employed **L-BFGS-B** optimization to compute smooth, jerk-minimizing control inputs subject to strict saturation bounds ($|a| \le 3.0\text{ m/s}^2$)[cite: 5].
- Verified trajectories across both simulation environments and real-world micro-UAV hardware setups.
