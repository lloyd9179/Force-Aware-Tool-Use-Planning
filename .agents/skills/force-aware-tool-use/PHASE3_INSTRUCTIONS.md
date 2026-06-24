# Phase 3 Instructions: Contact-Constrained Tool-Use Execution

This file is the detailed source of truth for Phase 3 work, but it must remain
compatible with `SKILL.md`, `AGENTS.md`, `README.md`, and
`docs/PROJECT_STATUS.md`.

If this file conflicts with the skill or repository instructions, update this
file first. If implementation changes public behavior, commands, status,
figures, or repository structure, update the relevant docs in the same change.

All repository Python commands must use `python3`, never `python`.

## 1. Mission

Phase 3 extends the completed planning and ROS2 visualization work into a
minimal contact-execution comparison:

```text
position-only execution
vs.
force-aware contact execution
```

The demo should show that a nominal tool path can be geometrically reachable
and torque-aware in planning, but still perform poorly during execution when
the real contact surface differs from the assumed geometry and contact forces
are ignored.

The final Phase 3 story is:

```text
same nominal tool-tip path and contact task
-> actual surface geometry differs from the assumed straight contact line
-> position-only execution follows nominal geometry and over-penetrates,
   tracks force poorly, or exceeds torque limits
-> force-aware feedback uses measured normal force to correct execution and
   reduce contact-related failure modes
```

## 2. Non-Negotiable Scope

Preserve completed Phase 1 and Phase 2 behavior.

Phase 3 may add:

- a deterministic 2D contact surface and tool-tip contact model;
- a simplified contact force estimate;
- position-only and force-aware feedback controllers;
- Python-only simulation, metrics, and plots;
- ROS2/RViz live visualization wrappers around the Python simulation.

Phase 3 must not add:

- Gazebo, MoveIt, PyBullet, or real robot execution;
- full physical contact simulation;
- real force control or impedance control;
- full rigid-body dynamics, gravity compensation, or 3D/6D planning;
- contact-implicit MPC, learning, perception, PDDLStream, or full TAMP;
- fixture-aware strategy planning.

Use careful wording in docs and README. Phase 3 is a simplified deterministic
model for learning and research practice, not hardware validation.

## 3. Architecture Rules

- Keep Phase 1 planning code independent of ROS2.
- Reuse Phase 1 kinematics, Jacobian, torque, configuration, and transform
  utilities where practical.
- Do not duplicate planner mathematics in Phase 3 or in the ROS2 package.
- Keep core Phase 3 simulation under `src/force_tool_planning/` and runnable
  without ROS2.
- Keep ROS2 wrappers under `ros2_ws/src/force_tool_planning_ros/`.
- ROS2 nodes may own simulator/controller objects and advance them from a ROS2
  timer, but contact, controller, force, and metric computations must remain in
  the pure-Python package.
- Keep scripts runnable from the repository root.
- Save deterministic Matplotlib figures under `media/figures/`.
- Use dataclasses and typed public functions.
- Preserve structured metrics and failure reasons.
- Add focused tests as behavior is added.
- Keep README concise and reader-facing; put detailed status in
  `docs/PROJECT_STATUS.md` and Phase 3 details in
  `docs/PHASE3_CONTACT_EXECUTION.md`.

## 4. Documentation Sync

After each Phase 3 edit, update the docs that changed behavior or expectations:

- `README.md`: public usage, results, limitations, and result figures.
- `docs/PROJECT_STATUS.md`: status, checklist, structure, and roadmap.
- `docs/PHASE3_CONTACT_EXECUTION.md`: Phase 3 design, metrics, outputs, and
  interpretation.
- `docs/EXECUTABLES_AND_LAUNCH_FILES.md`: scripts, ROS2 nodes, and launch
  files.
- `SKILL.md` or this file: authoritative workflow or scope changes.

Do not let commands, file paths, expected outputs, or completion status drift.
Every future Phase 3 action must end by updating all related docs and marking
the related completed steps in this file.

## 5. Target Package Layout

Extend the existing Python package instead of creating top-level import roots
such as `src/contact` or `src/control`.

Target structure:

```text
configs/
└── phase3_contact_execution.yaml

docs/
└── PHASE3_CONTACT_EXECUTION.md

src/force_tool_planning/
├── contact/
│   ├── __init__.py
│   ├── surface.py
│   ├── contact_model.py
│   ├── force_estimator.py
│   └── contact_metrics.py
├── control/
│   ├── __init__.py
│   ├── position_controller.py
│   ├── force_aware_controller.py
│   └── hybrid_position_force.py
├── simulation/
│   ├── __init__.py
│   ├── contact_execution_sim.py
│   ├── contact_execution_stepper.py
│   └── execution_result.py
├── analysis/
│   ├── __init__.py
│   ├── compare_execution.py
│   └── plot_contact_results.py
└── visualization/
    ├── __init__.py
    └── contact_markers.py

scripts/
├── run_phase3_contact_demo.py
├── compare_phase3_controllers.py
└── generate_phase3_figures.py

tests/
├── test_surface.py
├── test_contact_model.py
├── test_force_aware_controller.py
└── test_contact_metrics.py

ros2_ws/src/force_tool_planning_ros/
├── force_tool_planning_ros/
│   ├── contact_execution_demo_node.py
│   └── contact_markers.py
├── launch/
│   └── phase3_contact_execution.launch.py
└── rviz/
    └── phase3_contact_execution.rviz
```

Reuse existing directories if they already exist. Do not break Phase 1 scripts
or Phase 2 launches.

## 6. Configuration

Create or update:

```text
configs/phase3_contact_execution.yaml
```

The config should include these sections:

- `demo`: name and short description.
- `arm`: reference existing Phase 1/2 arm configuration; do not duplicate link
  lengths, joint limits, or torque limits unless an override is necessary.
- `tool`: reference existing tool and grasp configuration where practical.
- `task`: duration, timestep, tangential start/end, planned surface height,
  desired normal force, and tangential speed.
- `surface`: flat or sinusoidal uncertainty parameters.
- `contact_model`: normal stiffness, damping, friction coefficient, and max
  penetration.
- `controllers`: position-only gains and force-aware gains/deadband/clamp.
- `metrics`: contact loss, excessive force, torque warning, and torque failure
  thresholds.
- `output`: figure directory and optional CSV logging.

Phase 3 scripts must load this YAML by default and accept a custom config path.

## 7. Core Model Requirements

### Surface

Implement `Surface2D` in `src/force_tool_planning/contact/surface.py`.

Required behavior:

- `height(x)` supports `flat` and `sinusoidal` surfaces.
- `normal(x)` returns a unit normal pointing away from the surface into free
  space.
- `tangent(x)` returns a unit tangent.
- Normal and tangent are approximately orthogonal.

### Contact Model

Implement `PlanarContactModel` in
`src/force_tool_planning/contact/contact_model.py`.

Use the simplified convention:

```text
penetration = surface_height - tool_tip_y
```

If `penetration > 0`, contact exists. Compute:

```text
normal_force = max(0, normal_stiffness * penetration - normal_damping * normal_velocity)
friction_limit = friction_coefficient * normal_force
```

Return structured data containing surface height, penetration, normal force,
friction limit, contact state, and excessive penetration state.

### Force Estimator

Implement `estimate_contact_wrench_2d()` in
`src/force_tool_planning/contact/force_estimator.py`.

Use a planar wrench:

```text
wrench = [Fx, Fy, tau_z]
Fx = tangential_force
Fy = normal_force
tau_z = contact_point_x * Fy - contact_point_y * Fx
```

Map this wrench to joint torques using the existing Phase 1 torque utilities.
If an adapter is needed, keep it small and tested.

## 8. Controller Requirements

### Position-Only Controller

Implement `PositionOnlyController` in
`src/force_tool_planning/control/position_controller.py`.

It tracks desired tool-tip position and velocity only. It must not accept or
use measured normal force.

### Force-Aware Controller

Implement `ForceAwareController` in
`src/force_tool_planning/control/force_aware_controller.py`.

It should:

- track tangential motion along the surface tangent;
- use normal-force error for normal correction;
- apply a deadband;
- clamp the normal correction;
- document the sign convention.

Use this sign convention unless the implementation documents a better one:

```text
surface normal points away from the surface into free space
force_error = desired_normal_force - measured_normal_force
positive force_error means measured force is too low
positive normal correction moves opposite the surface normal, into contact
```

## 9. Simulation and Metrics

Create `ContactExecutionResult` in
`src/force_tool_planning/simulation/execution_result.py`.

Store at least:

- time;
- desired and actual tool-tip positions;
- normal force and desired normal force;
- penetration and contact state;
- joint torque and torque ratio;
- controller name;
- metrics and failure reasons.

Implement `ContactExecutionSimulator` in
`src/force_tool_planning/simulation/contact_execution_sim.py`.

Use a simple deterministic kinematic update, not full dynamics:

```text
actual_pos[t + 1] = actual_pos[t] + dt * commanded_velocity
```

or document an equivalent first-order update.

Implement metrics in `src/force_tool_planning/contact/contact_metrics.py`:

- force RMSE;
- contact loss fraction;
- max penetration;
- max torque ratio;
- number of torque violations;
- excessive force or excessive penetration counts;
- success boolean;
- failure reasons.

Success must require acceptable contact, torque ratio below the failure
threshold, bounded penetration, and no persistent excessive force.

## 10. Analysis, Plotting, and Scripts

Implement `compare_controllers(config_path: str)` in
`src/force_tool_planning/analysis/compare_execution.py`.

It should load config, build both controllers, run both simulations, compute
metrics, and return both results. Keep reusable config/controller/simulator
construction helpers in Python so the later ROS2 live node can share the same
setup instead of duplicating Phase 3 logic.

Implement plotting in `src/force_tool_planning/analysis/plot_contact_results.py`
with Matplotlib only. Save these figures:

```text
media/figures/phase3_tool_tip_trajectory.png
media/figures/phase3_force_tracking.png
media/figures/phase3_contact_state.png
media/figures/phase3_torque_ratio.png
```

Create scripts:

```text
scripts/compare_phase3_controllers.py
scripts/generate_phase3_figures.py
scripts/run_phase3_contact_demo.py
```

The main demo should print a concise summary for both controllers:

```text
Controller: position_only
  Force RMSE: ...
  Contact loss fraction: ...
  Max penetration: ...
  Max torque ratio: ...
  Success: ...
  Failure reasons: ...

Controller: force_aware
  Force RMSE: ...
  Contact loss fraction: ...
  Max penetration: ...
  Max torque ratio: ...
  Success: ...
  Failure reasons: ...
```

## 11. ROS2/RViz Requirements

Phase 3 ROS2 code is a live visualization wrapper around the pure-Python
simulation. The Python simulation must work without ROS2.

Add a launchable RViz demo under `ros2_ws/src/force_tool_planning_ros/` that can
run either controller mode from a ROS2 timer. The ROS2 node should instantiate
the pure-Python surface, contact model, controller, simulator/stepper, and
metrics code, advance one deterministic step per timer tick, and publish the
current result to ROS2 topics and RViz markers. It should show:

- robot and attached tool when existing Phase 2 visualization can be reused;
- desired and actual tool-tip path;
- contact surface;
- contact force or normal marker;
- controller mode and status text;
- visual distinction between contact loss, excessive penetration, warning, and
  success states.

The ROS2 wrapper must not reimplement contact, controller, force, or metric
math. It may publish live marker arrays, text/status messages, numeric topics,
and optional joint states if a tested Phase 1 IK/grasp adapter is used.

Required launch:

```bash
ros2 launch force_tool_planning_ros phase3_contact_execution.launch.py controller_mode:=force_aware
```

Support `controller_mode:=position_only` as well.

## 12. Incremental Implementation Checklist

Implement Phase 3 in small verified steps:

- [x] Step 1: Add or update Phase 3 config and docs shell.
- [x] Step 2: Implement surface model and tests.
  - Add `Surface2D`, flat and sinusoidal height behavior, unit tangent/normal
    vectors, and focused tests.
- [x] Step 3: Implement contact model and tests.
  - Add `PlanarContactModel`, contact state, penetration, normal force,
    friction limit, excessive penetration flag, and focused tests.
- [x] Step 4: Implement force estimator and torque adapter tests if needed.
  - Convert normal/tangential contact force into `[Fx, Fy, tau_z]` and connect
    it to the existing Phase 1 torque utilities without duplicating math.
- [x] Step 5: Implement position-only controller.
  - Track desired tool-tip position/velocity only and keep force feedback out
    of the baseline controller API.
- [x] Step 6: Implement force-aware controller and tests.
  - Add tangential tracking, normal-force correction, deadband, clamp, sign
    convention comments, and focused tests.
- [x] Step 7: Implement execution result dataclass.
  - Store time series for desired/actual tool-tip motion, force, penetration,
    contact state, torque, torque ratio, controller name, and metrics.
- [x] Step 8: Implement contact execution simulator.
  - Run both controller modes with a deterministic kinematic update and produce
    `ContactExecutionResult` without ROS2.
- [x] Step 9: Implement metrics and tests.
  - Compute force RMSE, contact loss, max penetration, max torque ratio,
    violation counts, success, and failure reasons.
- [x] Step 10: Implement comparison pipeline and numeric script.
  - Load config, run both controllers, compute metrics, and print a concise
    controller comparison.
  - Estimate execution-time torque after contact force is known by mapping the
    current tool-tip state through the Phase 1 force-aware selected grasp,
    selecting the IK branch closest to the previous or nearest Phase 1
    reference configuration, and reusing the existing Phase 1 Jacobian and
    torque utilities.
- [x] Step 11: Implement plotting and figure script.
  - Generate the four required Matplotlib figures under `media/figures/`
    without ROS2.
- [x] Step 12: Implement main demo script.
  - Run the full Phase 3 comparison, print metrics, save figures, and support
    a custom config path.
- [x] Step 13: Add ROS2/RViz live simulation wrapper.
  - Run position-only or force-aware contact execution from a ROS2 timer and
    publish live topics/markers without moving core simulation logic into ROS2.
- [x] Step 14: Update README, project status, executable docs, and Phase 3
  notes.
  - Finalize public usage, limitations, command references, result
    interpretation, and checklist status after the implementation changes.

Do not jump to ROS2 before the Python simulation, metrics, and tests work.
After each step is completed and verified, update this checklist in the same
change that updates the related docs.

## 13. Verification

For Phase 3 Python code changes, run the focused tests for the changed area and
then:

```bash
python3 -m pytest -q
```

For Phase 3 demo or plotting changes, run the relevant scripts when they exist:

```bash
python3 scripts/compare_phase3_controllers.py
python3 scripts/generate_phase3_figures.py
python3 scripts/run_phase3_contact_demo.py
```

For ROS2 package changes:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select force_tool_planning_ros
source install/setup.bash
ros2 launch force_tool_planning_ros phase3_contact_execution.launch.py controller_mode:=force_aware
```

Also verify Phase 1 and Phase 2 behavior is not regressed when shared code or
configuration changes:

```bash
python3 scripts/run_phase1_planner.py
python3 scripts/run_baseline_vs_force_aware.py
```

## 14. Definition of Done

Phase 3 is complete when:

- `python3 scripts/run_phase3_contact_demo.py` runs from the repository root;
- metrics are printed for both controllers;
- position-only execution performs worse or fails under the deterministic
  contact uncertainty;
- force-aware execution improves normal-force tracking;
- the four required figures are saved under `media/figures/`;
- focused Phase 3 tests and `python3 -m pytest -q` pass;
- ROS2/RViz live simulation launches when ROS2 Humble is available;
- README, project status, executable docs, and Phase 3 notes match the
  implemented behavior;
- Phase 1 planner scripts and Phase 2 demos remain valid.

The final project message should remain:

```text
A robot may be able to reach a tool path geometrically, but successful tool-use
execution also requires reasoning about wrench feasibility, joint torque
limits, grasp/tool transforms, and contact forces.
```
