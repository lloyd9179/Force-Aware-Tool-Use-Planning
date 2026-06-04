# Repository Instructions for Codex

## Project overview

This repository implements a minimal force-aware tool-use planning project for learning and research practice. The goal is to build a clean, reproducible demo that connects robot kinematics, IK planning, torque feasibility, simplified force control, and eventually fixture-aware task-and-motion planning.

The project starts with a 3-link planar arm and a tool-use task. Given a tool-tip path and a desired external force/wrench, the planner should search over grasp transforms and IK branches to find a smooth joint path. A baseline planner considers only IK and joint limits. A force-aware planner also checks torque feasibility using:

tau = J(q).T @ F

The project may later extend toward simple simulation execution, force control, ROS2 integration, and fixture-aware strategy planning.

## Research motivation

The central idea is that geometric motion feasibility is not enough for forceful tool use. A robot may be able to reach every waypoint, but the required wrench may cause joint torque violations or unstable contacts. Planning should therefore reason about motion and force constraints together.

This repository is inspired by force-and-motion constrained tool-use planning and multi-stage forceful manipulation with fixturing. The implementation should remain intentionally minimal at first so that the core planning ideas are clear.

## Development priorities

1. Build a reliable 2D/2.5D force-aware planning demo before attempting full 3D.
2. Make the baseline vs force-aware comparison visually obvious.
3. Keep code modular, deterministic, and easy to test.
4. Prioritize clear plots and reproducible scripts over complex physics.
5. Add ROS2 or contact simulation only after the planning core works.

## Recommended milestones

### Milestone 1: Force-aware path planner

Implement:
- 3-link planar arm forward kinematics.
- Planar Jacobian.
- Multiple IK candidate generation for each end-effector waypoint.
- Several grasp transforms from tool frame to end-effector frame.
- Tool-tip path generation.
- Tool-tip path to end-effector path conversion.
- Torque computation using tau = J(q).T @ F.
- IK candidate filtering using joint limits and torque limits.
- Dynamic programming or graph search to select the smoothest IK sequence.
- Baseline planner using IK and joint limits only.
- Force-aware planner using IK, joint limits, and torque feasibility.
- Plots saved to media/figures/.

### Milestone 2: Simple execution

Implement:
- Simple position-controlled execution of q_des(t).
- A PD controller.
- No contact simulation required in the first version.

### Milestone 3: Simplified force control

Implement:
- Quasi-static force-aware control:
  tau_cmd = J(q).T @ F_des + Kp * (q_des - q) + Kd * (qdot_des - qdot)
- Optional gravity compensation only if a dynamics model is added.

### Milestone 4: Fixture-aware minimal TAMP

Implement:
- A small enumerative strategy planner over:
  - whether to use a tool
  - which grasp to use
  - which fixture to use
  - which IK path to use
  - which force-control parameter to use
- Three planning levels:
  - Level 1: geometry-only planning
  - Level 2: torque-aware planning
  - Level 3: fixture-aware and robustness-aware planning
- Fixture choices may include table_friction, clamp, and weight.
- Robustness may be estimated by perturbing friction coefficient, contact location, normal force, or applied force.

## Expected figures

The project should eventually generate:
- tool-tip path
- transformed end-effector path
- IK candidates per waypoint
- rejected IK candidates due to torque limits
- selected baseline path
- selected force-aware path
- robot configurations along selected path
- joint torques over the path with torque-limit lines
- planner comparison table or plot
- optional fixture robustness plot

## Repository structure

Prefer this structure:

src/
  force_tool_planning/
    __init__.py
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

configs/
  demo_planar_arm.yaml

tests/
  test_kinematics.py
  test_jacobian.py
  test_torque_filter.py
  test_planner.py

docs/
  project_plan.md
  research_notes.md

media/
  figures/

## Coding standards

- Use Python for the first version.
- Use NumPy, SciPy, Matplotlib, PyYAML, and pytest.
- Use type hints for public functions.
- Use dataclasses for structured objects.
- Keep mathematical functions deterministic and easy to test.
- Avoid hidden global state.
- Use clear names for frames and units.
- Document frame conventions in docstrings.
- Every script should be runnable from the repository root.
- Save generated figures under media/figures/.
- Do not add ROS2 dependencies during Phase 1.
- Do not add heavy dependencies unless explicitly requested.
- Do not implement full contact dynamics unless explicitly requested.

## Testing

After changing math or planning code, run:

pytest

After changing plotting scripts, run the relevant script and verify that figures are generated.

If dependencies change, update requirements.txt.

## Communication style for code changes

When making changes, summarize:
- what was implemented
- which files changed
- how to run it
- what tests were run
- any known limitations
