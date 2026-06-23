# Executables and Launch Files

The repository keeps the public command surface small while separating planning,
ROS messages, nodes, and launch wiring into testable files. Most Python modules
are support code and are not intended to be run directly.

## Recommended Demo Commands

```bash
# Phase 1 planner comparison and saved figures
python3 scripts/run_baseline_vs_force_aware.py

# Phase 3 numeric contact execution comparison
python3 scripts/compare_phase3_controllers.py

# Phase 3 saved contact execution figures
python3 scripts/generate_phase3_figures.py

# Phase 3 full Python-only contact execution demo
python3 scripts/run_phase3_contact_demo.py

# Phase 3 live contact execution RViz wrapper
ros2 launch force_tool_planning_ros phase3_contact_execution.launch.py controller_mode:=force_aware

# Phase 2 torque-feasible force-aware execution demo
ros2 launch force_tool_planning_ros phase2.launch.py

# Phase 2 torque-infeasible baseline visual comparison
ros2 launch force_tool_planning_ros baseline_demo.launch.py
```

Both Phase 2 execution demos run their selected complete motion twice, starting
the second run only after the first path has stopped. The baseline launch is
explicitly a mock-hardware visualization of a path known to violate the Phase 1
torque limits. It is not the recommended execution result.

## Phase 1 Scripts

| File | Purpose |
| --- | --- |
| `scripts/run_baseline_vs_force_aware.py` | Main Phase 1 comparison. Runs both planners, prints feasibility results, and saves comparison figures. |
| `scripts/run_phase1_planner.py` | Runs only the force-aware planner and saves its focused figures. Useful for isolated planner checks. |
| `scripts/search_demo_scenario.py` | Development utility that searches a small deterministic parameter grid for a scenario showing the intended baseline-versus-force-aware result. |

## Phase 3 Scripts

| File | Purpose |
| --- | --- |
| `scripts/compare_phase3_controllers.py` | Runs the Python-only Phase 3 comparison, estimates execution torque using the Phase 1 force-aware reference branch, prints metrics for the position-only and force-aware controllers, and does not require ROS2. |
| `scripts/generate_phase3_figures.py` | Runs the same Python-only Phase 3 comparison and saves the required trajectory, force-tracking, contact-state, and torque-ratio figures under `media/figures/`. |
| `scripts/run_phase3_contact_demo.py` | Main Python-only Phase 3 demo. Prints the controller metrics and saves the required Phase 3 figures in one command, with optional `--config` support. |

## ROS2 Launch Files

| File | Purpose |
| --- | --- |
| `launch/phase2.launch.py` | Recommended complete demo. Uses the force-aware grasp and executes the torque-feasible force-aware path twice. |
| `launch/baseline_demo.launch.py` | Comparison demo. Uses the baseline grasp and executes the known torque-infeasible baseline path twice on mock hardware. |
| `launch/phase3_contact_execution.launch.py` | Phase 3 live contact execution demo. Starts robot_state_publisher, the timer-driven contact execution wrapper, and RViz. Supports `controller_mode:=force_aware` and `controller_mode:=position_only`. |
| `launch/display.launch.py` | Display-only diagnostic launch. Uses a synthetic joint-state animation to validate URDF, TF, markers, and RViz without ros2_control. |
| `launch/control.launch.py` | Control-only diagnostic launch. Starts mock hardware and controllers without RViz or automatic trajectory execution. |

The two complete launch files are intentionally thin. Shared complete-demo
wiring lives in `force_tool_planning_ros/demo_launch.py`.

The Phase 3 launch is different from the Phase 2 trajectory demos: it does not
send a ros2_control trajectory. Instead, `contact_execution_demo_node` advances
the pure-Python contact execution stepper from a ROS2 timer, publishes
`/joint_states` for the current Phase 1 IK branch, and visualizes contact state
in RViz. The contact, controller, force, torque, and metric computations remain
in `src/force_tool_planning/`.

The default Phase 3 contact strip is deliberately placed forward of the base,
from `x=1.2 m` to `x=1.7 m` around `y=0.6 m`, so the live surface marker and
tool-tip motion are not centered on the `base_link` visual.
The saved Phase 3 RViz view is focused near `(x=1.45 m, y=0.6 m)` to frame
that contact strip by default.

Phase 3 live topics:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/force_tool_planning/contact_execution_markers` | `visualization_msgs/MarkerArray` | Desired path, actual path, surface, tool-tip status, contact-force normal, and status text. |
| `/force_tool_planning/contact_execution_status` | `std_msgs/String` | Current controller mode, sample index, contact status, force, penetration, and torque ratio. |
| `/force_tool_planning/contact_normal_force_n` | `std_msgs/Float64` | Current measured normal force. |
| `/force_tool_planning/contact_torque_ratio` | `std_msgs/Float64` | Current max joint torque-limit ratio. |

## Understanding RViz Planning Diagnostics

The RViz display named `Planning Diagnostics` is a single static `MarkerArray`.
It is not a second robot, a second URDF, or a motion trail. The actual moving
arm appears under the separate `RobotModel` display and follows `/joint_states`.

The `MarkerArray` contains:

- a white line for the desired tool-tip (the small red sphere in RViz) path;
- an orange line for the baseline planner's selected end-effector (the yellow sphere in RViz) path;
- red spheres at baseline waypoints that violate torque limits;
- a green line for the force-aware planner's selected, torque-feasible
  end-effector path.

These markers intentionally remain fixed while either demo executes. The
tool-tip path and end-effector paths differ because the selected grasp defines
a rigid transform between those frames. Small Z offsets separate the markers
visually when their XY positions overlap.

## ROS2 Python Modules

| File | Purpose |
| --- | --- |
| `demo_launch.py` | Shared launch builder for the force-aware and baseline complete demos. Prevents duplicate launch wiring. |
| `result_adapter.py` | Runs Phase 1 and converts its results into a ROS-independent data contract used by Phase 2. |
| `trajectory_helpers.py` | Converts an explicitly selected planner path into deterministic ROS `JointTrajectory` messages. |
| `trajectory_sender_node.py` | Sends the selected path to ros2_control, waits for each full motion to stop, repeats it the requested number of times, then exits. |
| `marker_helpers.py` | Converts Phase 1 paths and violation indices into deterministic RViz markers. |
| `marker_publisher_node.py` | Publishes retained diagnostic markers for RViz. |
| `summary_node.py` | Prints the baseline-versus-force-aware result and clearly labels which path the current demo executes. |
| `joint_state_demo_node.py` | Publishes synthetic joint motion only for the display-only diagnostic launch. It is not used by either complete execution demo. |
| `contact_execution_demo_node.py` | Phase 3 live wrapper node. Loads Phase 3 config, advances the shared pure-Python contact stepper from a ROS2 timer, and publishes live markers, status, normal force, torque ratio, and joint states. |
| `contact_markers.py` | Phase 3 marker helpers for the surface, desired and actual paths, contact-force normal, tool-tip status, and status text. |

## Why Keep the Diagnostic Launches?

`display.launch.py` isolates visualization and TF problems. `control.launch.py`
isolates controller and mock-hardware problems. Keeping them avoids debugging
the full demo when only one subsystem is under test. Normal users can ignore
both and use one of the two complete demo launches.
