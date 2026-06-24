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

Status: complete.

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
- [x] Control-only launch added for robot_state_publisher, ros2_control mock
  hardware, and the two required controller spawners.
- [x] Joint-state broadcaster and force-aware trajectory controller activation
  verified; the broadcaster publishes all three arm joints on `/joint_states`.
- [x] Force-aware selected joint path converts to a deterministic position-only
  `JointTrajectory` with tested joint names, positions, point count, and
  strictly increasing timestamps.
- [x] One-shot `FollowJointTrajectory` sender added; it completes a controlled
  move to the first force-aware waypoint before sending the selected path.
- [x] Mock-controller execution verified through the final selected force-aware
  waypoint while both required controllers remained active.
- [x] Complete Phase 2 launch added for summary, diagnostics, RViz, mock
  control, controller activation, and ordered force-aware execution.
- [x] Separate baseline comparison launch added with the baseline grasp and
  explicit torque-infeasible visualization labeling.
- [x] Both complete execution demos repeat the entire selected motion twice,
  waiting for each full motion to stop before starting the next run.
- [x] Executable, support-module, and launch-file responsibilities documented.
- [x] Planning-diagnostic labels, inclined RViz recording camera, and execution
  timing polished and documented.
- [x] Final force-aware and baseline comparison GIFs saved under
  `media/figures/` and displayed together in the README.
- [x] Final verification completed: `58` Phase 1 tests and `34` ROS2 package
  tests passed with no failures; both complete demos were verified live.

Final Phase 2 media:

```text
media/figures/baseline.gif
media/figures/forcecontrol.gif
```

Detailed small-step plan and live checklist:

```text
.agents/skills/force-aware-tool-use/PHASE2_INSTRUCTIONS.md
```

### Phase 3: Contact-Constrained Tool-Use Execution

Status: complete.

Completed:

- [x] Phase 3 config shell and documentation shell;
- [x] simplified 2D contact surface model and focused tests;
- [x] simplified tool-tip contact model and focused tests;
- [x] contact force estimation and existing planar-wrench torque utility check;
- [x] position-only baseline execution controller and focused tests;
- [x] force-aware feedback execution controller and focused tests;
- [x] structured contact execution result dataclass and focused tests;
- [x] Python-only contact execution simulator and focused tests;
- [x] contact execution metrics and focused tests;
- [x] comparison pipeline, Phase 1-referenced torque estimation, and numeric
  comparison script;
- [x] comparison plots and figure generation script;
- [x] main Phase 3 demo script that prints metrics and saves figures;
- [x] ROS2/RViz live simulation wrapper for publishing contact execution
  results;
- [x] README, project status, executable docs, Phase 3 notes, and Phase 3
  checklist synchronized with the implemented behavior;
- [x] focused tests for contact models, controllers, metrics, scripts, plots,
  and ROS2 wrapper helpers.

Phase 3 must preserve completed Phase 1 planning behavior and completed Phase 2
mock-control visualization behavior. Core Phase 3 simulation must run without
ROS2; ROS2/RViz is only a live visualization and publication layer around the
pure-Python simulation.

The default Phase 3 contact strip is configured in the forward Phase 1
workspace region (`x=1.2 m` to `x=1.7 m`, planned surface height `y=0.6 m`).
The nominal desired path assumes this simple contact line, while the actual
surface is a deterministic sinusoidal height field with offset and amplitude
error. Position-only execution follows the nominal geometry and fails under
that contact uncertainty; force-aware feedback uses measured normal force to
reduce the contact failure modes and succeeds in the default comparison.

Final Phase 3 media:

```text
media/figures/phase3_tool_tip_trajectory.png
media/figures/phase3_force_tracking.png
media/figures/phase3_contact_state.png
media/figures/phase3_torque_ratio.png
```

Detailed source of truth:

```text
.agents/skills/force-aware-tool-use/PHASE3_INSTRUCTIONS.md
```

Supporting design notes:

```text
docs/PHASE3_CONTACT_EXECUTION.md
```

## Repository Structure

```text
Force-Aware-Tool-Use-Planning/
├── configs/                    # Phase 1 scenario and Phase 3 comparison config
├── docs/                       # Status and project documentation
├── media/figures/              # Phase 1/Phase 3 figures and Phase 2 GIFs
├── scripts/                    # Runnable Phase 1 demos and Phase 3 scripts
├── src/force_tool_planning/    # Pure Python planning, contact, and analysis package
├── tests/                      # Phase 1 tests and Phase 3 focused tests
├── ros2_ws/src/
│   └── force_tool_planning_ros/ # Phase 2 and Phase 3 ROS2/RViz package
└── .agents/skills/
    └── force-aware-tool-use/   # Phase-specific implementation instructions
```

Generated ROS2 workspace outputs under `ros2_ws/build/`, `ros2_ws/install/`,
and `ros2_ws/log/` are ignored by Git.

## Planned Phases

### Phase 4: Fixture-Aware Strategy Planning

- Add simplified fixture choices such as table friction, clamps, and weights.
- Compare geometric-only, torque-aware, and fixture-aware strategies.
- Evaluate robustness under parameter perturbations.

## Scope Boundaries

The project intentionally avoids claiming physical force execution from the
current planar model. Phase 3 may add a simplified deterministic 2D contact
execution model, but the project still excludes Gazebo physics, MoveIt, real
hardware drivers, full physical contact simulation, real force or impedance
control, full rigid-body dynamics, 3D planning, 6D wrench planning, perception,
and learned control.
