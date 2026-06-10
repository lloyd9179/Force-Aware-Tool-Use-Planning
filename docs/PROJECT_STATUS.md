# Project Status and Roadmap

This document tracks implementation status, repository organization, and
planned phases. The public-facing project summary and usage instructions remain
in the repository `README.md`.

## Current Status

### Phase 1: Force-Aware Planar Planning

Status: complete.

Implemented:

- planar 3-link arm kinematics and Jacobians;
- planar pose and grasp transforms;
- deterministic analytic inverse kinematics;
- structured candidate generation and rejection diagnostics;
- torque checks using `tau = J(q).T @ F`;
- layered dynamic programming;
- baseline and force-aware planners;
- validated YAML scenario configuration;
- deterministic scripts, tests, and Matplotlib figures.

Verified deterministic result:

```text
Baseline:
  selected grasp: angled_down
  torque feasible: no
  max torque ratio: 1.314285

Force-aware:
  selected grasp: short_inline
  torque feasible: yes
  max torque ratio: 0.875000
```

Completion checklist:

- [x] `python3 -m pytest -q` passes.
- [x] `python3 scripts/run_phase1_planner.py` runs.
- [x] `python3 scripts/run_baseline_vs_force_aware.py` runs.
- [x] Baseline finds a geometric path that violates torque limits.
- [x] Force-aware planner finds a torque-feasible alternative.
- [x] The selected force-aware result differs from the baseline.
- [x] Deterministic figures are saved under `media/figures/`.
- [x] Usage and limitations are documented.

Detailed source of truth:

```text
.agents/skills/force-aware-tool-use/PHASE1_INSTRUCTIONS.md
```

### Phase 2: ROS2, RViz, and ros2_control Demo

Status: in progress.

Completed:

- [x] Phase 2 instructions and repository scope defined.
- [x] Separate ROS2 workspace and package directories created.
- [x] Empty `ament_python` package metadata added.
- [x] Empty ROS2 package build verified.
- [x] Minimal 3-link planar arm Xacro added.
- [x] Xacro generation, URDF parsing, and robot_state_publisher loading
  verified.
- [x] Force-aware `short_inline` tool and tool-tip frames added.
- [x] Deterministic display joint-state node added and tested.
- [x] Display-only launch and initial RViz configuration added.
- [x] Moving arm and attached tool verified through RViz, joint states, and TF.
- [x] Phase 1-to-ROS result data contract and deterministic adapter added.
- [x] Result shapes, selected grasps, torque feasibility, and baseline violation
  waypoint derivation tested.
- [x] Existing Phase 1 Python package installed with the ROS package so the
  adapter works from the sourced workspace.
- [x] Baseline-versus-force-aware ROS summary node added.
- [x] Deterministic desired-path, baseline-path, violation, and force-aware-path
  marker construction helpers added.
- [x] Diagnostic marker IDs, frames, types, colors, and point counts tested.
- [x] Reliable transient-local diagnostic MarkerArray publisher added.
- [x] Display launch starts the marker publisher and RViz shows the desired
  tool path, baseline path, baseline violations, and force-aware path.
- [x] Position-only ros2_control mock-hardware Xacro block added for the three
  actuated arm joints.
- [x] Joint-state broadcaster and position JointTrajectoryController
  configuration added and checked against the mock-hardware interfaces.

Remaining major milestones:

- [ ] ros2_control mock hardware and trajectory execution.
- [ ] Full demo launch and captured RViz media.

Detailed small-step plan and live checklist:

```text
.agents/skills/force-aware-tool-use/PHASE2_INSTRUCTIONS.md
```

## Repository Structure

```text
Force-Aware-Tool-Use-Planning/
├── configs/                    # Deterministic planner scenario
├── docs/                       # Status and project documentation
├── media/figures/              # Generated Phase 1 figures
├── scripts/                    # Runnable Phase 1 demos
├── src/force_tool_planning/    # Pure Python planning package
├── tests/                      # Phase 1 tests
├── ros2_ws/src/
│   └── force_tool_planning_ros/ # Phase 2 ROS2 package
└── .agents/skills/
    └── force-aware-tool-use/   # Phase-specific implementation instructions
```

Generated ROS2 workspace outputs under `ros2_ws/build/`, `ros2_ws/install/`,
and `ros2_ws/log/` are ignored by Git.

## Planned Phases

### Phase 3: Simplified Force-Control-Inspired Execution

- Add a simple feedforward torque term.
- Compare planned torque demand with commanded torque.
- Preserve the lightweight educational model.

### Phase 4: Fixture-Aware Strategy Planning

- Add simplified fixture choices such as table friction, clamps, and weights.
- Compare geometric-only, torque-aware, and fixture-aware strategies.
- Evaluate robustness under parameter perturbations.

## Scope Boundaries

The project intentionally avoids claiming physical force execution from the
current planar model. Unless a later phase explicitly changes the scope, the
project excludes Gazebo physics, MoveIt, real hardware drivers, contact
simulation, full rigid-body dynamics, 3D planning, 6D wrench planning,
perception, and learned control.
