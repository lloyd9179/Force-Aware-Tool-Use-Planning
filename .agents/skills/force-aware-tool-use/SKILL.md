---
name: force-aware-tool-use
description: Use this skill when implementing, reviewing, debugging, or extending this repository's force-aware robot tool-use planning demo, including the Phase 1 planar planner and the Phase 2 ROS2/RViz/ros2_control integration.
---

# Force-Aware Tool-Use Planning Skill

## Sources Of Truth

Before implementing or reviewing work, read the instruction file for the
relevant phase:

- Phase 1 planning work: `PHASE1_INSTRUCTIONS.md`
- Phase 2 ROS2 integration work: `PHASE2_INSTRUCTIONS.md`

Treat the relevant phase instruction file as the detailed source of truth. If
Phase 2 touches Phase 1 code, both files apply and Phase 1 behavior must remain
unchanged.

All repository commands must use `python3`, never `python`.

## Purpose

Use this skill for the learning-oriented Phase 1 force-aware tool-use planning
demo.

The demo must show:

```text
same deterministic tool path and desired planar wrench
→ baseline planner finds a geometric path that violates torque limits
→ force-aware planner selects a torque-feasible alternative
```

The core feasibility equation is:

```text
tau = J(q).T @ F
```

Phase 2 presents and executes the existing result through ROS2, RViz, and
ros2_control mock hardware. It must not duplicate or replace Phase 1 planning.

## Required Workflow

1. Read the instruction file for the active phase.
2. Inspect the current repository before editing.
3. Preserve the existing kinematics, Jacobian, torque APIs, and tests unless a
   change is necessary.
4. Implement the smallest next part of the active phase pipeline.
5. Preserve structured diagnostics and rejected candidates.
6. Add or update focused tests.
7. Run the required `python3` verification commands.
8. Keep README concise and update it when public installation, usage, behavior,
   or selected result figures change.
9. Record implementation status, checklists, repository structure, and roadmap
   changes in `docs/PROJECT_STATUS.md`.

## Phase 1 Pipeline

Implement the pipeline specified in `PHASE1_INSTRUCTIONS.md`:

```text
tool path
→ grasp transform
→ end-effector path
→ multiple IK candidates per waypoint
→ joint-limit filtering
→ torque checks using tau = J(q).T @ F
→ torque-limit filtering
→ layered dynamic programming
→ baseline planner
→ force-aware planner
→ deterministic comparison and saved figures
```

Use the required modules, dataclasses, function names, frame conventions,
configuration behavior, script behavior, terminal outputs, and completion
criteria defined in `PHASE1_INSTRUCTIONS.md`. Do not substitute conflicting
APIs from older summaries.

## Strict Scope

Implement only the active phase and preserve the completed Phase 1 planner.

Do not add:

- Gazebo, MoveIt, or PyBullet;
- physical simulation or hardware execution;
- force or impedance control;
- fixture-aware planning, PDDLStream, or full TAMP;
- 3D kinematics or 6D spatial wrench planning;
- dynamics or gravity compensation;
- perception, reinforcement learning, or learned grasping.

Phase 1 grasps are simplified planar rigid transforms. The wrench is a
simplified planar task wrench. Do not claim that Phase 1 performs physical
contact simulation or real force control.

Phase 2 may use ROS2, RViz, robot_state_publisher, and ros2_control mock
hardware for position trajectory execution. Do not claim that this validates
the desired wrench physically or performs force control.

## Engineering Rules

- Keep the implementation deterministic and lightweight.
- Prefer analytic 3-link IK over a numerical optimizer.
- Reuse the existing `wrap_to_pi()` implementation.
- Use explicit frame and unit names.
- Document every transform direction.
- Use dataclasses and structured planning results.
- Preserve failed and rejected candidates for diagnostics.
- Return structured failures from top-level planner functions; internal dynamic
  programming helpers may return `None` as specified in
  `PHASE1_INSTRUCTIONS.md`.
- Save deterministic, non-interactive Matplotlib figures under
  `media/figures/`.
- Do not rewrite working mathematical modules unnecessarily.

## Verification

Run the full test suite with:

```bash
python3 -m pytest -q
```

When the relevant scripts exist, run:

```bash
python3 scripts/run_phase1_planner.py
python3 scripts/run_baseline_vs_force_aware.py
```

Confirm the intended deterministic condition:

```text
baseline geometric success == True
baseline torque feasible == False
force-aware success == True
force-aware torque feasible == True
```

Confirm that at least three figures are saved under `media/figures/`.

## Completion Report

When making code changes, report:

- what was implemented;
- which files changed;
- how to run it using `python3`;
- what tests and scripts were run;
- where figures were saved;
- known limitations.
