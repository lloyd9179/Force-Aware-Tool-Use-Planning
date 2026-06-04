---
name: force-aware-tool-use
description: Use this skill when implementing, reviewing, or extending this repository's force-aware robot tool-use planning demo, including planar arm kinematics, IK candidate generation, torque feasibility with J^T F, baseline vs force-aware planning, fixture-aware strategy planning, robustness analysis, and simple simulation/control extensions.
---

# Force-Aware Tool-Use Planning Skill

## Purpose

Use this skill to work on a minimal force-aware tool-use planning project for learning and research practice. The project is designed to help build understanding of robot planning, ROS2-ready software structure, force-and-motion constraints, simplified task-and-motion planning, and fixture-aware manipulation.

Do not describe the project as being made to join a lab or to impress a professor. Frame it as a learning-oriented robotics planning project.

## Core idea

A robot using a tool must do more than reach a path geometrically. It must also be able to transmit the required force/wrench through its joints, grasp, tool, and environment. A path that satisfies IK can still fail because the desired wrench creates joint torques beyond the robot's limits.

The first implementation should demonstrate this clearly with a 3-link planar arm.

## Phase 1 target

Build a complete force-aware path planning demo.

Inputs:
- 3-link planar arm model
- joint limits
- joint torque limits
- several grasp transforms G
- tool-tip path
- desired external force or planar wrench

Pipeline:
1. Generate or load a tool-tip path.
2. For each grasp transform G, convert the tool-tip path into an end-effector path.
3. For each waypoint, generate multiple IK candidates.
4. Filter IK candidates by joint limits.
5. Compute the planar Jacobian J(q).
6. Compute joint torque:
   tau = J(q).T @ F
7. Filter IK candidates by torque limits.
8. Build a layered graph of IK candidates across waypoints.
9. Use dynamic programming or graph search to select the smoothest IK sequence.
10. Compare:
    - baseline planner: IK + joint limits only
    - force-aware planner: IK + joint limits + torque feasibility
11. Save plots to media/figures/.

## Important implementation details

### Frames

Clearly document:
- world frame
- end-effector frame
- tool frame
- tool-tip frame

For the planar version, use x, y, theta poses. Be explicit about whether a transform maps from tool to end-effector or from end-effector to tool.

### Kinematics

Implement forward kinematics for a 3-link planar revolute arm.

Expected output:
- joint positions for visualization
- end-effector pose [x, y, theta]

### Jacobian

Implement the planar geometric Jacobian. For force-only checks in x/y, J may be 2 x n. For planar wrench checks [Fx, Fy, tau_z], J may be 3 x n.

Use consistent dimensions:
- If F is [Fx, Fy], use translational Jacobian J_xy and tau = J_xy.T @ F.
- If wrench is [Fx, Fy, Mz], use planar Jacobian J_planar and tau = J_planar.T @ wrench.

### IK

The IK solver should return multiple candidates for each waypoint. It can use:
- random restarts with scipy.optimize
- seeded local optimization from previous solutions
- analytic IK if implemented later

Do not require perfect IK in the first version. It is acceptable to use numerical IK with tolerances.

### Torque feasibility

A candidate is torque-feasible if:

abs(tau_i) <= tau_limit_i

for every joint i.

Keep rejected candidates for visualization. Do not silently discard all debugging information.

### Graph search / dynamic programming

Each waypoint is a layer. Each IK candidate is a node. Edge cost should measure smoothness, for example:

cost(q_i, q_j) = ||wrap_to_pi(q_j - q_i)||^2

The selected path is the minimum-cost sequence through the layers.

If any layer has no feasible candidates, return a structured failure result explaining which waypoint failed and why.

### Baseline vs force-aware comparison

The demo should intentionally include a desired force large enough that:
- the baseline planner finds a geometrically valid path
- some selected baseline configurations violate torque limits
- the force-aware planner either selects another grasp or another IK branch

This comparison is the main result of Phase 1.

## Phase 2 target: simple execution

Add simple trajectory execution after Phase 1 works:
- q_des(t) from the planner
- PD position control
- no contact simulation required initially

This can later be connected to ROS2, Gazebo, PyBullet, or RViz.

## Phase 3 target: simplified force control

Add a quasi-static force-aware control command:

tau_cmd = J(q).T @ F_des + Kp * (q_des - q) + Kd * (qdot_des - qdot)

Only add gravity compensation if the project includes a dynamics model.

## Phase 4 target: fixture-aware minimal TAMP

Add a small enumerative strategy planner. It does not need full PDDLStream at first.

Strategy choices:
- use_tool: true/false
- grasp option
- fixture option
- IK path
- force-control parameter

Planning levels:
- Level 1: geometry-only planning using IK and joint limits.
- Level 2: torque-aware planning using IK, joint limits, and tau = J.T @ F.
- Level 3: fixture-aware and robustness-aware planning.

Fixture options:
- table_friction
- clamp
- weight

Simplified fixture stability:
- table_friction succeeds when required tangential force <= mu * normal_force
- clamp succeeds when clamp force and friction margin are sufficient
- weight succeeds when weight normal force produces enough friction margin

Robustness:
Estimate robustness by perturbing parameters such as:
- friction coefficient
- applied force
- normal force
- contact location
- fixture placement

A robust strategy should avoid brittle solutions even if they are shorter.

## Visualization requirements

Generate clear figures for:
- tool-tip path
- end-effector path
- IK candidates
- rejected IK candidates due to torque limits
- selected baseline robot configurations
- selected force-aware robot configurations
- joint torque over path with torque-limit lines
- strategy comparison for Level 1/2/3
- fixture robustness if implemented

Plots should be saved under:

media/figures/

## Code organization

Prefer:

src/force_tool_planning/
  kinematics.py
  jacobian.py
  ik.py
  torque.py
  planner.py
  grasps.py
  fixtures.py
  visualization.py

scripts/
  run_phase1_planner.py
  run_baseline_vs_force_aware.py
  run_fixture_planner.py

tests/
  test_kinematics.py
  test_jacobian.py
  test_torque_filter.py
  test_planner.py

configs/
  demo_planar_arm.yaml

## Quality rules

- Keep the first version simple.
- Prefer reliable 2D implementation over incomplete 3D implementation.
- Use dataclasses for structured data.
- Use type hints.
- Write small functions.
- Add tests for math functions.
- Make scripts runnable from the repository root.
- Save generated figures.
- Do not add ROS2 dependencies in Phase 1.
- Do not add heavy dependencies unless explicitly requested.
- Do not implement full contact simulation unless explicitly requested.

## Done definition for Phase 1

Phase 1 is complete when:
- pytest passes
- run_phase1_planner.py runs from the repo root
- baseline and force-aware planners both run
- at least one scenario shows baseline torque violation
- force-aware planning avoids torque violation by changing grasp or IK branch
- figures are saved in media/figures/
- README explains how to reproduce the demo
