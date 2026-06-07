# Force-Aware Tool-Use Planning

A minimal research-style demo for **force-aware tool-use planning** with a planar robot arm.

The project demonstrates a simple but important idea:

> A robot may be able to geometrically reach a tool path, but still be unable to execute the task because the required force would exceed joint torque limits.

For a robot configuration `q`, applying a planar end-effector wrench

```text
F = [Fx, Fy, Mz]
```

requires joint torques

```text
tau = J(q).T @ F
```

A baseline planner that only checks inverse kinematics and joint limits can therefore choose a path that looks valid but is not physically feasible. This repository compares that baseline with a force-aware planner that also checks torque feasibility.

---

## Core comparison

This project compares two planners:

| Planner | Checks IK | Checks joint limits | Checks torque limits | Expected behavior |
|---|---:|---:|---:|---|
| Baseline planner | Yes | Yes | No | May find a geometrically valid but torque-infeasible path |
| Force-aware planner | Yes | Yes | Yes | Rejects torque-infeasible candidates and may choose another grasp or IK branch |

The intended Phase 1 demo is:

```text
tool-tip path
→ candidate grasp transforms
→ end-effector paths
→ multiple IK candidates per waypoint
→ baseline path selection
→ force-aware torque filtering
→ baseline vs force-aware comparison
```

---

## Project motivation

Many manipulation tasks require both motion and force. Tool-use examples include cutting, wrench turning, screw driving, and pulling with a hammer claw. In these tasks, a path that is reachable in configuration space may still fail because the robot cannot transmit the required wrench through its joints or grasp.

This repository focuses on a lightweight Phase 1 version of that problem:

- planar 3-link robot arm
- planar tool path
- simplified grasp transforms
- desired planar wrench
- inverse kinematics candidates
- torque computation with `tau = J(q).T @ F`
- dynamic programming over IK candidates
- baseline vs force-aware planning comparison

The project is intentionally small so that the planning idea is easy to inspect, test, and visualize.

---

## Current status

Implemented so far:

- Planar arm model
- Forward kinematics
- Joint position computation for visualization
- Joint-limit checking
- Planar Jacobian
- Translational Jacobian
- Joint torque computation from planar force or wrench
- Torque-limit checking with structured diagnostics
- Planar pose composition, inversion, relative transforms, and path transforms
- Simplified deterministic grasp transforms
- Deterministic horizontal tool-use task generation
- Analytic 3-link planar IK with elbow-up and elbow-down branches
- Structured IK candidates with preserved rejection diagnostics
- Layered dynamic programming for smooth joint-path selection
- Baseline planner using IK and joint limits
- Force-aware planner using IK, joint limits, and torque limits
- Validated YAML configuration for the deterministic Phase 1 scenario
- Deterministic scenario-search helper
- Non-interactive Matplotlib visualizations for paths, torques, and filtering
- Runnable force-aware-only and baseline-versus-force-aware demo scripts
- Unit tests for core mathematical modules

Current test status:

```bash
python3 -m pytest -q
```

Expected current result:

```text
58 passed
```

Phase 1 is complete. The repository provides the mathematical foundation,
planar path and grasp transforms, deterministic task generation, analytic
3-link IK, candidate generation, dynamic programming, both planners, a
validated deterministic YAML scenario, runnable scripts, and saved
visualizations.

---

## Phase 1 goal

Phase 1 is complete when the repository can run a deterministic demo where:

1. The baseline planner finds a geometrically valid joint path.
2. The baseline path violates joint torque limits under the desired wrench.
3. The force-aware planner rejects torque-infeasible candidates.
4. The force-aware planner finds a torque-feasible path by selecting a different grasp, IK branch, or joint trajectory.
5. The demo saves clear figures to `media/figures/`.

The main command is:

```bash
python3 scripts/run_baseline_vs_force_aware.py
```

Current deterministic result:

```text
Baseline planner:
  geometric path found: yes
  selected grasp: angled_down
  torque feasible after evaluation: no
  max torque ratio: 1.314285

Force-aware planner:
  path found: yes
  selected grasp: short_inline
  torque feasible: yes
  max torque ratio: 0.875000

Saved figures:
  media/figures/tool_and_ee_paths.png
  media/figures/baseline_vs_force_aware_paths.png
  media/figures/torque_profiles.png
  media/figures/candidate_filtering_summary.png
```

---

## Repository structure

```text
force-aware-tool-use-planning/
├── README.md
├── requirements.txt
├── .agents/
│   └── skills/
│       └── force-aware-tool-use/
│           ├── SKILL.md
│           └── PHASE1_INSTRUCTIONS.md
├── configs/
│   └── demo_planar_arm.yaml
├── media/
│   └── figures/
├── scripts/
│   ├── run_phase1_planner.py
│   ├── run_baseline_vs_force_aware.py
│   └── search_demo_scenario.py
├── src/
│   └── force_tool_planning/
│       ├── __init__.py
│       ├── kinematics.py
│       ├── jacobian.py
│       ├── torque.py
│       ├── transforms.py
│       ├── grasps.py
│       ├── ik.py
│       ├── tasks.py
│       ├── planner.py
│       ├── config.py
│       └── visualization.py
└── tests/
    ├── test_config.py
    ├── test_end_to_end.py
    ├── test_grasps_tasks.py
    ├── test_kinematics.py
    ├── test_jacobian.py
    ├── test_torque_filter.py
    ├── test_transforms.py
    ├── test_ik.py
    ├── test_planner.py
    └── test_visualization.py
```

---

## Quick start

Clone the repository:

```bash
git clone https://github.com/<your-username>/force-aware-tool-use-planning.git
cd force-aware-tool-use-planning
```

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Install dependencies:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
```

Run the tests and main demo:

```bash
python3 -m pytest -q
python3 scripts/run_baseline_vs_force_aware.py
```

---

## Running tests

Run all tests:

```bash
python3 -m pytest -q
```

The test suite should verify:

- forward kinematics
- Jacobian correctness
- torque computation
- pose transforms
- analytic IK
- candidate filtering
- dynamic programming path selection
- baseline vs force-aware behavior
- validated YAML loading
- a core end-to-end smoke test that does not import Matplotlib
- non-interactive figure generation and figure closing

Run only the no-plot core smoke test:

```bash
python3 -m pytest -q tests/test_end_to_end.py
```

---

## Running the Phase 1 demo

Run the main Phase 1 comparison:

```bash
python3 scripts/run_baseline_vs_force_aware.py
```

This command:

1. Loads `configs/demo_planar_arm.yaml`.
2. Builds a 3-link planar arm.
3. Generates a simple tool-use path and desired wrench.
4. Converts the tool path to end-effector paths for several grasps.
5. Generates multiple IK candidates at each waypoint.
6. Runs the baseline planner.
7. Runs the force-aware planner.
8. Prints a comparison summary.
9. Saves figures to `media/figures/`.

The deterministic scenario is stored in:

```text
configs/demo_planar_arm.yaml
```

It currently produces:

```text
Baseline:    grasp=angled_down, torque feasible=no,  max torque ratio=1.314285
Force-aware: grasp=short_inline, torque feasible=yes, max torque ratio=0.875000
```

To search a small deterministic parameter grid around the configured scenario:

```bash
python3 scripts/search_demo_scenario.py
```

The search helper reports the first matching scenario and does not modify the
YAML file.

Run the force-aware planner without the baseline comparison:

```bash
python3 scripts/run_phase1_planner.py
```

The main comparison saves:

```text
media/figures/tool_and_ee_paths.png
media/figures/baseline_vs_force_aware_paths.png
media/figures/torque_profiles.png
media/figures/candidate_filtering_summary.png
```

Matplotlib uses a non-interactive backend, so the scripts do not require a
display server.

---

## Using a custom scenario

Copy or edit `configs/demo_planar_arm.yaml`. The main fields are:

- `arm.link_lengths_m`: three positive planar link lengths.
- `arm.joint_limits_rad`: lower and upper limits for each joint.
- `arm.torque_limits_nm`: positive absolute joint torque limits.
- `task.start_pose`: first tool pose `[x, y, theta]`.
- `task.length_m`: horizontal path length.
- `task.desired_wrench`: simplified planar wrench `[Fx, Fy, Mz]`.
- `grasps[].tool_T_ee`: end-effector pose expressed in the tool frame.
- `output.figures_dir`: figure output directory.

Run either script with a custom configuration:

```bash
python3 scripts/run_baseline_vs_force_aware.py --config path/to/config.yaml
python3 scripts/run_phase1_planner.py --config path/to/config.yaml
```

The config loader rejects missing sections, invalid vector lengths, nonpositive
link or torque limits, invalid waypoint counts, and duplicate grasp names.

---

## Reading the output

- `geometric path found: yes` means the baseline found a complete IK path
  satisfying joint limits. It does not imply torque feasibility.
- `torque feasible: no` means at least one selected configuration requires
  torque beyond an absolute joint limit.
- `max torque ratio` is the largest value of
  `abs(required torque) / torque limit` along the selected path.
- A ratio greater than `1.0` is infeasible; a ratio at or below `1.0` is
  feasible.
- `candidate_filtering_summary.png` shows that the baseline only rejects
  joint-limit failures, while the force-aware planner also rejects
  torque-limit failures.

For the default scenario, the baseline selects the smoother `angled_down`
grasp but reaches a maximum ratio of `1.314285`. The force-aware planner selects
`short_inline` and stays within limits at `0.875000`.

---

## Programmatic use

The repository uses a `src/` layout without package-install metadata. From the
repository root, set `PYTHONPATH=src` when calling the library directly:

```bash
PYTHONPATH=src python3 -c "
from force_tool_planning.config import arm_from_config, grasps_from_config, load_demo_config, task_from_config
from force_tool_planning.planner import plan_baseline, plan_force_aware

config = load_demo_config('configs/demo_planar_arm.yaml')
arm = arm_from_config(config)
task = task_from_config(config)
grasps = grasps_from_config(config)

baseline = plan_baseline(arm, task, grasps)
force_aware = plan_force_aware(arm, task, grasps)
print(baseline.selected_grasp, baseline.max_torque_ratio)
print(force_aware.selected_grasp, force_aware.max_torque_ratio)
"
```

---

## Generated figures

The main Phase 1 demo generates:

```text
media/figures/tool_and_ee_paths.png
media/figures/baseline_vs_force_aware_paths.png
media/figures/torque_profiles.png
media/figures/candidate_filtering_summary.png
```

The torque profile plot shows that the baseline trajectory exceeds a joint
torque limit while the force-aware trajectory remains within all limits.

---

## Phase 1 implementation checklist

Phase 1 is considered complete only when all of the following are true:

- [x] `python3 -m pytest -q` passes.
- [x] `python3 scripts/run_phase1_planner.py` runs.
- [x] `python3 scripts/run_baseline_vs_force_aware.py` runs.
- [x] Baseline planner finds a geometric path.
- [x] Baseline selected path violates torque limits.
- [x] Force-aware planner finds a torque-feasible path.
- [x] Force-aware result differs from baseline by grasp, IK branch, or joint trajectory.
- [x] At least three figures are saved to `media/figures/`.
- [x] README explains how to reproduce the demo.
- [x] The code remains lightweight and does not require ROS2 or heavy robotics dependencies.

---

## Limitations

This repository currently focuses on planning, not control.

Phase 1 does **not** claim to implement:

- real force control
- contact-rich physical simulation
- grasp stability mechanics
- fixture-aware task and motion planning
- ROS2 execution
- real robot deployment
- 3D manipulation

The wrench is treated as a simplified planar task wrench, and the grasp is represented as a rigid planar transform. These simplifications are intentional for the first planning demo.

The wrench is applied directly in the simplified world/end-effector planar task
frame. Phase 1 does not implement wrench-frame transforms, gravity
compensation, dynamics, contact simulation, or execution control.

---

## Future phases

Possible future extensions:

### Phase 2: Simple execution layer

- Time-parameterized joint trajectories
- Simple joint-space tracking
- Optional ROS2 wrapper
- RViz or lightweight simulation visualization

### Phase 3: Simplified force-control-inspired execution

- Add feedforward torque term
- Compare planned torque demand and commanded torque
- Keep the model simple and educational

### Phase 4: Fixture-aware strategy planning

- Add simplified fixture models such as table friction, clamp, and weight
- Compare geometric-only, torque-aware, and fixture-aware strategy choices
- Add robustness analysis under parameter perturbations

---

## Development philosophy

The project should remain small, deterministic, and easy to inspect.

Guiding principles:

- Keep planning logic separate from visualization.
- Preserve diagnostic information instead of silently discarding failed candidates.
- Prefer deterministic examples over random demos.
- Use simple mathematical models before adding robotics middleware.
- Make the baseline failure and force-aware success visually obvious.

The key message is:

> The baseline planner succeeds geometrically but fails under the desired wrench. The force-aware planner reasons about `J(q).T @ F`, rejects torque-infeasible IK candidates, and selects a path that stays within joint torque limits.
