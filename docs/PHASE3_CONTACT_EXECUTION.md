# Phase 3 Contact-Constrained Tool-Use Execution

Phase 3 is complete. It extends the completed force-aware planning demo into a
completed simplified deterministic execution comparison:

```text
position-only execution
vs.
force-aware contact execution
```

The demo shows that a nominal trajectory can be geometrically reachable and
torque-aware in planning, yet still perform poorly during execution when the
real contact surface differs from the assumed geometry and contact forces are
ignored.

## Scope

Phase 3 implements a deterministic 2D tool-surface contact model, a tool-tip
contact force estimate, position-only and force-aware feedback controllers,
execution metrics, comparison plots, and an RViz live visualization wrapper.

The core simulation remains pure Python and runs without ROS2. ROS2 and RViz
publish live simulation state from a timer-driven wrapper node.

Core Phase 3 modules extend the existing package under
`src/force_tool_planning/`, for example `contact/`, `control/`, `simulation/`,
and `analysis/` subpackages. Do not create parallel top-level import roots such
as `src/contact` or duplicate Phase 1 kinematics and torque code.

Do not add Gazebo, MoveIt, real robot execution, full rigid-body dynamics,
contact-implicit MPC, learning, or real force/impedance control.

## Expected Outputs

The Phase 3 figure script saves:

```text
media/figures/phase3_tool_tip_trajectory.png
media/figures/phase3_force_tracking.png
media/figures/phase3_contact_state.png
media/figures/phase3_torque_ratio.png
```

The main demo prints metrics for both controllers, including force RMSE,
contact loss fraction, max penetration, max torque ratio, success, and failure
reasons.

The Step 10 numeric comparison can be run from the repository root:

```bash
python3 scripts/compare_phase3_controllers.py
```

It loads `configs/phase3_contact_execution.yaml`, runs both controllers through
the pure-Python simulator, estimates contact-execution torques with the Phase 1
force-aware selected grasp and IK branch as the reference, attaches contact
metrics, and prints the concise controller summary. With the default
deterministic surface uncertainty, position-only execution fails from torque
limit exceedance, excessive penetration, and excessive force, while force-aware
execution improves normal-force tracking, stays below the torque failure
threshold, and succeeds.

The Step 11 figure generation script can be run from the repository root:

```bash
python3 scripts/generate_phase3_figures.py
```

It reuses the Step 10 comparison result and saves the four required Matplotlib
figures under `media/figures/`.

The Step 12 main demo script runs the complete Python-only Phase 3 demo:

```bash
python3 scripts/run_phase3_contact_demo.py
```

It supports `--config`, prints the same position-only and force-aware metrics,
and saves the required figures in one command.

The ROS2/RViz demo shows the same contact execution live. The ROS2 node owns
the Python simulator/controller objects and advances a pure-Python stepper from
a fixed timer, while controller, contact, force, torque, and metric
computations remain in `src/force_tool_planning/`.

After building and sourcing the ROS2 workspace, run:

```bash
ros2 launch force_tool_planning_ros phase3_contact_execution.launch.py controller_mode:=force_aware
```

Use `controller_mode:=position_only` to view the baseline contact-execution
controller.

## Configuration

The default Phase 3 config is:

```text
configs/phase3_contact_execution.yaml
```

It references the existing Phase 1 scenario in `configs/demo_planar_arm.yaml`
for arm, torque-limit, and grasp data. Phase 3 scripts load this config by
default and accept a custom config path.

The default contact task is placed in the same forward workspace region as the
Phase 1 tool-use path: tangential motion runs from `x=1.2 m` to `x=1.7 m`
around a planned surface height of `y=0.6 m`. The desired tool-tip trajectory
uses this nominal straight contact line. The actual surface is configured as a
sinusoidal height field with offset and amplitude error, representing imperfect
surface geometry knowledge rather than a perfectly known surface. This keeps
the Phase 3 contact surface away from the `base_link` visual while preserving
the simplified contact comparison. The project still does not perform
collision detection.

## Implemented Surface Model

`src/force_tool_planning/contact/surface.py` provides `Surface2D`, a
deterministic height-field surface model for Phase 3. It supports flat and
sinusoidal surfaces, returns height in meters, and provides unit tangent and
normal vectors. The normal points away from the surface into free space.

The default demo uses:

```text
y_surface(x) = planned_height + offset + amplitude * sin(2*pi*frequency*x)
```

This intentionally differs from the nominal straight desired path so the
controllers must handle surface-geometry error during contact execution.

Focused tests live in `tests/test_surface.py`.

## Implemented Contact Model

`src/force_tool_planning/contact/contact_model.py` provides
`PlanarContactModel`, a deterministic linear spring-damper contact model for a
2D tool tip and `Surface2D`. It uses the Phase 3 height-field convention:

```text
penetration = surface_height - tool_tip_y
```

Positive penetration creates contact, normal force, a Coulomb friction limit,
and an excessive-penetration flag. The contact result is returned as a typed
`ContactState` dataclass with an `as_dict()` helper for future scripts and
diagnostics.

Focused tests live in `tests/test_contact_model.py`.

## Implemented Force Estimator

`src/force_tool_planning/contact/force_estimator.py` provides
`estimate_contact_wrench_2d()`, which converts simplified normal and tangential
contact forces into a planar wrench:

```text
wrench = [Fx, Fy, tau_z]
tau_z = contact_point_x * Fy - contact_point_y * Fx
```

The existing Phase 1 `joint_torques_from_wrench()` helper already maps this
wrench to joint torques with `tau = J(q).T @ wrench`, so no separate torque
adapter is needed.

Focused tests live in `tests/test_force_estimator.py`.

## Implemented Position-Only Controller

`src/force_tool_planning/control/position_controller.py` provides
`PositionOnlyController`, the Phase 3 baseline execution controller. It returns
a 2D tool-tip velocity command:

```text
desired_vel + kp_task * position_error + kd_task * velocity_error
```

The controller intentionally has no force-feedback inputs. It tracks desired
tool-tip position and velocity only. In the default demo it follows the nominal
straight-line geometry, so contact quality can degrade when the actual surface
differs from that assumed geometry.

Focused tests live in `tests/test_position_controller.py`.

## Implemented Force-Aware Controller

`src/force_tool_planning/control/force_aware_controller.py` provides
`ForceAwareController`, the Phase 3 feedback execution controller. It returns a
2D tool-tip velocity command that tracks motion along the surface tangent and
uses normal-force error for a bounded normal velocity correction.

The sign convention is:

```text
surface normal points away from the surface into free space
force_error = desired_normal_force - measured_normal_force
positive force_error means measured force is too low
positive normal correction moves opposite the surface normal, into contact
```

The controller applies a normal-force deadband and clamps the normal correction
with `max_normal_correction_mps`. Focused tests live in
`tests/test_force_aware_controller.py`.

This lets the actual tool-tip motion deviate from the nominal geometric path
when the measured force is too low or too high, which is the intended contrast
with the position-only baseline.

## Implemented Execution Result Container

`src/force_tool_planning/simulation/execution_result.py` provides
`ContactExecutionResult`, a frozen dataclass for storing Phase 3 execution
time series. It stores controller name, time, desired and actual tool-tip
positions, normal-force traces, penetration, contact state, joint torques,
torque ratios, metrics, and failure reasons.

The container validates array shapes and finite numeric values so later
simulation, metrics, plotting, and ROS2 live visualization code can rely on a
consistent result structure. Focused tests live in
`tests/test_execution_result.py`.

## Implemented Contact Execution Simulator

`src/force_tool_planning/simulation/contact_execution_sim.py` provides
`ContactExecutionSimulator`, a deterministic kinematic rollout runner for
Phase 3. It accepts either `PositionOnlyController` or `ForceAwareController`,
computes contact state at each sample, and updates the actual tool-tip
position with:

```text
actual_pos[t + 1] = actual_pos[t] + dt * commanded_velocity
```

The simulator returns `ContactExecutionResult` and remains independent of ROS2.
Joint torque values come from an optional torque-estimator callback. Without
one, the simulator records zero torques, which keeps the simulator independent
from planner details. The Step 10 comparison supplies a Phase 1-referenced
torque estimator so the numeric comparison reports nonzero torque ratios.
Focused tests live in `tests/test_contact_execution_sim.py`.

## Implemented Contact Metrics

`src/force_tool_planning/contact/contact_metrics.py` provides
`ContactMetricThresholds`, `ContactExecutionMetrics`,
`compute_contact_metrics()`, and `result_with_contact_metrics()`. The metrics
cover force RMSE, contact loss fraction, max penetration, max torque ratio,
torque warning count, torque violation count, excessive force count, excessive
penetration count, success, and failure reasons.

Success requires acceptable contact loss, torque ratio below the failure
threshold, bounded penetration, and no excessive-force samples. Focused tests
live in `tests/test_contact_metrics.py`.

## Implemented Comparison Pipeline

`src/force_tool_planning/analysis/compare_execution.py` provides
`compare_controllers()`, reusable builders for the Phase 3 surface, contact
model, controllers, simulator, desired trajectory, Phase 1 force-aware
execution reference, torque estimator, and metric thresholds, plus
`format_comparison_summary()` for script output. The torque estimator maps the
current Phase 3 tool-tip state through the Phase 1 selected grasp, selects the
IK branch closest to the previous or nearest Phase 1 reference configuration,
and computes `tau = J(q).T @ wrench` with the existing Phase 1 Jacobian and
torque utilities after contact force is known.

`scripts/compare_phase3_controllers.py` is the Step 10 numeric entry point.
`scripts/generate_phase3_figures.py` is the Step 11 figure entry point.
`scripts/run_phase3_contact_demo.py` is the Step 12 combined demo entry point.
Focused tests live in `tests/test_compare_execution.py` and
`tests/test_phase3_contact_demo_script.py`.

`build_contact_execution_stepper()` builds the Step 13 live stepping interface
from the same config/controller/simulator helpers. It preserves the Step 10
Phase 1-referenced torque estimator, so the ROS2 live wrapper uses the same
nonzero torque-ratio path as the numeric scripts and figures.

## Implemented Phase 3 Plots

`src/force_tool_planning/analysis/plot_contact_results.py` provides
`save_phase3_figures()` and individual Matplotlib plot helpers for:

- desired and actual tool-tip trajectories;
- desired and measured normal-force tracking;
- contact state and penetration;
- torque-limit ratio over time.

The plotting helpers are non-interactive, close figures after saving, and write
the required PNGs under the configured figure directory. Focused tests live in
`tests/test_phase3_plot_contact_results.py`.

## Implemented ROS2/RViz Live Simulation

The Phase 3 ROS2 package adds a live simulation wrapper rather than replaying a
precomputed result. `contact_execution_demo_node.py` loads the Phase 3 config,
builds the pure-Python contact execution stepper, advances one sample per ROS2
timer tick, and publishes the current desired path, actual tool-tip state,
contact state, normal-force marker, status, numeric force and torque-ratio
topics, controller mode, and `/joint_states` for the current Phase 1 IK branch.

The launch file is:

```bash
ros2 launch force_tool_planning_ros phase3_contact_execution.launch.py controller_mode:=force_aware
```

It also supports:

```bash
ros2 launch force_tool_planning_ros phase3_contact_execution.launch.py controller_mode:=position_only
```

Live topics:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/force_tool_planning/contact_execution_markers` | `visualization_msgs/MarkerArray` | Contact surface, desired path, actual path, tool-tip status, contact-force normal, and status text. |
| `/force_tool_planning/contact_execution_status` | `std_msgs/String` | Controller mode, sample index, status, force, penetration, and torque ratio. |
| `/force_tool_planning/contact_normal_force_n` | `std_msgs/Float64` | Current measured normal force. |
| `/force_tool_planning/contact_torque_ratio` | `std_msgs/Float64` | Current max joint torque-limit ratio. |

ROS2 remains a wrapper: contact physics, force feedback, torque estimation, and
metrics stay in the pure-Python package. This keeps batch scripts, tests,
plots, and live RViz behavior aligned.

## Authoritative Instructions

Implementation details, required file names, development steps, and completion
criteria are maintained in:

```text
.agents/skills/force-aware-tool-use/PHASE3_INSTRUCTIONS.md
```

After each Phase 3 edit, update relevant docs in the same change so the README,
project status, executable docs, and Phase 3 notes stay aligned with the code.
Also update the Phase 3 checklist in `PHASE3_INSTRUCTIONS.md` whenever a
related implementation step is completed.
