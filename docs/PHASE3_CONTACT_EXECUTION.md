# Phase 3 Contact-Constrained Tool-Use Execution

Phase 3 is planned, not yet implemented.
Step 1 is complete: the Phase 3 config shell exists at
`configs/phase3_contact_execution.yaml`, this notes file exists, README mentions
the Phase 3 roadmap, and project status tracks Phase 3.
Step 2 is complete: the `Surface2D` model and focused surface tests exist.
Step 3 is complete: the simplified `PlanarContactModel` and focused contact
model tests exist.
Step 4 is complete: the contact force estimator exists and connects directly to
the existing Phase 1 planar-wrench torque utility.
Step 5 is complete: the position-only baseline controller and focused tests
exist.
Step 6 is complete: the force-aware feedback controller and focused tests
exist.

The goal is to extend the completed force-aware planning demo into a simplified
execution comparison:

```text
position-only execution
vs.
force-aware contact execution
```

The demo should show that a trajectory can be geometrically reachable and
torque-aware in planning, yet still perform poorly during execution when
contact forces are ignored.

## Scope

Phase 3 should add a deterministic 2D tool-surface contact model, a tool-tip
contact force estimate, position-only and force-aware feedback controllers,
execution metrics, comparison plots, and an RViz replay wrapper.

The core simulation must remain pure Python and must run without ROS2. ROS2 and
RViz should only visualize or replay generated results.

Core Phase 3 modules should extend the existing package under
`src/force_tool_planning/`, for example `contact/`, `control/`, `simulation/`,
and `analysis/` subpackages. Do not create parallel top-level import roots such
as `src/contact` or duplicate Phase 1 kinematics and torque code.

Do not add Gazebo, MoveIt, real robot execution, full rigid-body dynamics,
contact-implicit MPC, learning, or real force/impedance control.

## Expected Outputs

The Phase 3 scripts should eventually save:

```text
media/figures/phase3_tool_tip_trajectory.png
media/figures/phase3_force_tracking.png
media/figures/phase3_contact_state.png
media/figures/phase3_torque_ratio.png
```

The main demo should print metrics for both controllers, including force RMSE,
contact loss fraction, max penetration, max torque ratio, success, and failure
reasons.

## Configuration

The default Phase 3 config is:

```text
configs/phase3_contact_execution.yaml
```

It references the existing Phase 1 scenario in `configs/demo_planar_arm.yaml`
for arm, torque-limit, and grasp data. Phase 3 implementations should load this
config by default and accept a custom config path from scripts.

## Implemented Surface Model

`src/force_tool_planning/contact/surface.py` provides `Surface2D`, a
deterministic height-field surface model for Phase 3. It supports flat and
sinusoidal surfaces, returns height in meters, and provides unit tangent and
normal vectors. The normal points away from the surface into free space.

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
tool-tip position and velocity only, so future Phase 3 comparisons can isolate
the effect of force-aware feedback.

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
