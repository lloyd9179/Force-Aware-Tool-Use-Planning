# Repository Instructions for Codex

## Authoritative Instructions

The repository contains a completed Phase 1 planner and an incremental Phase 2
ROS2/RViz/ros2_control integration.

For every Phase 1 implementation, review, or documentation task:

1. Use `.agents/skills/force-aware-tool-use/SKILL.md`.
2. Read `.agents/skills/force-aware-tool-use/PHASE1_INSTRUCTIONS.md` before making changes.
3. Treat `PHASE1_INSTRUCTIONS.md` as the detailed source of truth.
4. If this file, README, existing code comments, or older documentation conflict
   with the skill or Phase 1 instructions, follow the skill and Phase 1
   instructions.

For every Phase 2 implementation, review, or documentation task:

1. Use `.agents/skills/force-aware-tool-use/SKILL.md`.
2. Read `.agents/skills/force-aware-tool-use/PHASE2_INSTRUCTIONS.md` before
   making changes.
3. Treat `PHASE2_INSTRUCTIONS.md` as the detailed source of truth for ROS2,
   RViz, and ros2_control integration.
4. Preserve the completed Phase 1 behavior and follow
   `PHASE1_INSTRUCTIONS.md` for any Phase 1 code touched by Phase 2 work.

All repository commands must use `python3`, never `python`.

## Project Overview

This repository implements a minimal force-aware tool-use planning demo for
learning and research practice.

The central idea is that a geometrically feasible robot motion may still be
physically infeasible when the required planar wrench exceeds joint torque
limits:

```text
tau = J(q).T @ F
```

Phase 1 compares:

- a baseline planner using IK, joint limits, and smoothness;
- a force-aware planner that also filters candidates by torque limits.

The deterministic demo must show that the baseline finds a geometric path that
violates torque limits while the force-aware planner finds a torque-feasible
alternative.

## Phase Scope

Phase 1 is complete and remains the source of truth for planning. Phase 2 may
add ROS2, RViz, robot_state_publisher, and ros2_control mock-hardware trajectory
execution under `ros2_ws/`.

Do not add Gazebo, MoveIt, PyBullet, real hardware execution, physical contact
simulation, force control, impedance control, fixture-aware planning,
PDDLStream, full TAMP, 3D planning, 6D wrench planning, dynamics, gravity
compensation, perception, reinforcement learning, or learned grasping.

Phase 1 code must not depend on ROS2. Phase 2 must consume Phase 1 results
without duplicating planner mathematics.

## Existing Code

Preserve and extend the working mathematical foundation:

```text
src/force_tool_planning/kinematics.py
src/force_tool_planning/jacobian.py
src/force_tool_planning/torque.py
```

Do not rewrite working code unnecessarily. Preserve existing tests and add
focused tests for new behavior.

The existing `wrap_to_pi()` implementation is authoritative. Reuse or re-export
it instead of creating a divergent implementation.

## Frame Convention

Use planar poses `[x, y, theta]` in meters and radians.

Use the detailed frame convention from `PHASE1_INSTRUCTIONS.md`:

```text
world_T_ee = world_T_tool ⊕ tool_T_ee
```

For Phase 1, the tool-tip frame and tool frame may be treated as the same frame
only when this simplification is documented explicitly:

```text
world_T_tooltip ≈ world_T_tool
```

If a separate tool-tip offset is added later, represent it explicitly:

```text
world_T_ee = world_T_tooltip ⊕ tooltip_T_tool ⊕ tool_T_ee
```

Every transform function must document transform direction.

## Development Rules

- Use Python, NumPy, Matplotlib, PyYAML, and pytest. Use SciPy only if needed.
- Prefer deterministic analytic 3-link IK for the main demo.
- Use type hints for public functions.
- Use dataclasses for structured objects.
- Keep mathematical functions deterministic and easy to test.
- Avoid hidden global state.
- Use clear frame and unit names.
- Preserve rejected candidates and structured diagnostics.
- Make scripts runnable from the repository root.
- Save generated figures under `media/figures/`.
- Do not add heavy dependencies.
- Keep `README.md` concise and reader-facing.
- Record implementation status, checklists, repository structure, and roadmap
  changes in `docs/PROJECT_STATUS.md`.

## Required Verification

After changing math or planning code, run:

```bash
python3 -m pytest -q
```

After changing plotting or demo scripts, run the relevant scripts:

```bash
python3 scripts/run_phase1_planner.py
python3 scripts/run_baseline_vs_force_aware.py
```

Verify that generated figures are saved under `media/figures/`.

After changing Phase 2 package files, source ROS2 Humble and build the package:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select force_tool_planning_ros
```

If dependencies change, update `requirements.txt`.

## Communication Style for Code Changes

When making code changes, summarize:

- what was implemented;
- which files changed;
- how to run it using `python3`;
- what tests were run;
- where figures were saved;
- any known limitations.

Keep summaries concrete and concise.
