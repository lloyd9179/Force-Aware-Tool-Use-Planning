# Phase 2 Codex Instructions: ROS2, RViz, and ros2_control Demo

This document is the source of truth for implementing Phase 2 of the
Force-Aware Tool-Use Planning project.

Phase 1 is complete and remains the source of truth for planning. Phase 2 wraps
the deterministic Phase 1 result in a ROS2 Humble visualization and mock-control
execution demo.

All repository Python commands must use `python3`, never `python`.

## 1. Phase 2 Mission

The Phase 2 demo must communicate:

```text
completed Phase 1 planning result
-> ROS2 nodes and RViz markers
-> ros2_control mock hardware
-> force-aware joint trajectory execution
```

The final visual story is:

1. Show the desired tool path.
2. Show the geometrically valid but torque-infeasible baseline path.
3. Highlight baseline torque-violation waypoints.
4. Show the torque-feasible force-aware path.
5. Execute only the force-aware joint path through a
   `joint_trajectory_controller`.

## 2. Strict Scope

Implement:

- one ROS2 Humble `ament_python` package under
  `ros2_ws/src/force_tool_planning_ros/`;
- a 3-link planar arm and attached tool in URDF/Xacro;
- robot_state_publisher and RViz visualization;
- deterministic MarkerArray diagnostics;
- ros2_control mock hardware;
- joint_state_broadcaster and joint_trajectory_controller;
- force-aware position-trajectory execution;
- focused tests, launch files, and documentation.

Do not implement:

- Gazebo, MoveIt, PyBullet, or physical contact simulation;
- real robot drivers or hardware execution;
- force control, impedance control, or real wrench sensing;
- torque-command controllers;
- fixture-aware planning, PDDLStream, or full TAMP;
- 3D planning, 6D wrench planning, dynamics, or gravity compensation;
- perception, reinforcement learning, or learned grasping.

Phase 2 trajectory execution is position-only visualization through mock
hardware. It does not prove physical wrench feasibility beyond the Phase 1
model.

## 3. Architecture Rules

- Keep the pure Python planner under `src/force_tool_planning/` independent of
  ROS2.
- Keep all ROS2 code under `ros2_ws/src/force_tool_planning_ros/`.
- Keep `README.md` concise and reader-facing; record implementation status,
  checklists, repository structure, and roadmap changes in
  `docs/PROJECT_STATUS.md`.
- Consume Phase 1 results; do not copy planner mathematics into the ROS2
  package.
- Preserve Phase 1 APIs, tests, scripts, deterministic behavior, and figures.
- Add dependencies only when the step using them is implemented.
- Keep nodes small and move deterministic conversions into testable helpers.
- Use the Phase 1 configuration as the source of arm dimensions and scenario
  data where practical.

## 4. Frame and Data Contract

Phase 1 planar poses use `[x, y, theta]` in meters and radians:

```text
world_T_ee = world_T_tool compose tool_T_ee
```

For Phase 2:

- treat the Phase 1 world frame as ROS `base_link`;
- move the arm in the XY plane with revolute joint axes along Z;
- publish path markers in `base_link`;
- preserve `tool_T_ee` direction when reading Phase 1 data;
- use the inverse transform `ee_T_tool` for a URDF fixed joint from
  `ee_link` to `tool_link`;
- visibly attach the tool using the selected force-aware grasp;
- derive baseline violating waypoint indices from torque checks on the
  baseline selected candidates.

The main force-aware execution path is `PlanningResult.path_q` with shape
`(N, 3)`. Do not send the torque-infeasible baseline path to the controller in
the default final demo. A separate, clearly labeled baseline comparison launch
may execute it through mock hardware only to visualize the difference; it must
state that the path violates the Phase 1 torque model.

## 5. Target Package Structure

```text
ros2_ws/
└── src/
    └── force_tool_planning_ros/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── force_tool_planning_ros
        ├── force_tool_planning_ros/
        │   └── __init__.py
        ├── config/
        ├── launch/
        ├── urdf/
        └── test/
```

Generated `ros2_ws/build/`, `ros2_ws/install/`, and `ros2_ws/log/` directories
must remain untracked.

## 6. Small-Step Implementation Plan and Status

Update this checklist after each Phase 2 task. Keep each implementation step
small and verify it before starting the next one.

- [x] 1. Define Phase 2 instructions and authorize the scope in repository docs.
- [x] 2. Create the ROS2 workspace and package directories.
- [x] 3. Add empty `ament_python` package metadata.
- [x] 4. Verify the empty package builds.
- [x] 5. Add a minimal 3-link planar arm Xacro without the tool.
- [x] 6. Validate Xacro generation and robot_state_publisher loading.
- [x] 7. Add tool and tool-tip frames using the force-aware grasp transform.
- [x] 8. Add a deterministic joint-state demo node.
- [x] 9. Add display-only launch.
- [x] 10. Add the initial top-down RViz configuration.
- [x] 11. Verify display-only arm motion and tool attachment.
- [x] 12. Define the Phase 1-to-ROS result data contract.
- [x] 13. Implement the Phase 1 result adapter.
- [x] 14. Test result shapes, grasps, feasibility, and violation waypoints.
- [x] 15. Add the baseline-versus-force-aware summary node.
- [x] 16. Add deterministic marker-construction helpers.
- [x] 17. Test marker IDs, frames, types, colors, and point counts.
- [x] 18. Add the marker publisher node.
- [x] 19. Add markers to RViz and verify the visual comparison.
- [x] 20. Add the ros2_control mock-hardware Xacro block.
- [x] 21. Add controller configuration.
- [x] 22. Add control-only launch.
- [x] 23. Verify controllers activate and publish joint states.
- [x] 24. Add the force-aware JointTrajectory conversion helper.
- [x] 25. Test joint names, positions, points, and timestamps.
- [x] 26. Add the FollowJointTrajectory sender node.
- [x] 27. Add a controlled move to the first planned waypoint.
- [x] 28. Verify successful force-aware controller execution.
- [x] 29. Add the complete Phase 2 launch file.
- [x] 30. Polish markers, labels, camera, and timing.
- [x] 31. Complete Phase 2 README and usage documentation.
- [x] 32. Run final Phase 1 and Phase 2 verification.
- [x] 33. Save an RViz screenshot and optionally a short video.

Current status:

```text
Phase 1: complete and verified
Phase 2: complete; steps 1-33 verified
ROS2 package: buildable display-only, control-only, and complete demos
Robot model: planar arm, force-aware short_inline tool, and tool tip implemented
Display: deterministic joint motion and RViz launch verified
Planning bridge: deterministic Phase 1 result adapter implemented and tested
Summary node and deterministic diagnostic marker helpers: implemented
Retained diagnostic markers: published and displayed in RViz
Position-only ros2_control mock-hardware description: implemented and tested
Joint-state broadcaster and position trajectory controller config: implemented
Control-only launch: implemented
Controller activation and joint-state publication: verified
Force-aware position JointTrajectory conversion helper: implemented and tested
FollowJointTrajectory sender with controlled first-waypoint move: implemented
Force-aware mock-controller execution: verified through the final waypoint
Complete Phase 2 launch: implemented
Baseline mock-execution comparison launch: implemented
Force-aware and baseline complete motions: each repeats twice, then stops
Executable and launch-file guide: documented
Final inclined RViz camera and planning-diagnostic explanation: documented
Final verification: 58 Phase 1 tests and 34 ROS2 package tests passed
Captured comparison media: media/figures/baseline.gif and forcecontrol.gif
Repository and ROS package license: MIT
```

## 7. Verification

After every Phase 2 change, preserve Phase 1:

```bash
python3 -m pytest -q
```

Build the ROS2 package from `ros2_ws/`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select force_tool_planning_ros
```

When package tests exist:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon test --packages-select force_tool_planning_ros
colcon test-result --verbose
```

Run launch files only after their corresponding checklist steps are complete.

## 8. Phase 2 Completion Criteria

Phase 2 is complete only when:

1. Phase 1 tests and deterministic scripts still pass.
2. The ROS2 workspace builds and package tests pass.
3. Display-only launch shows the planar arm and attached tool in RViz.
4. RViz shows the desired path, baseline path, violation waypoints, and
   force-aware path.
5. ros2_control mock hardware and both required controllers become active.
6. The force-aware joint path executes successfully.
7. One launch command runs the complete demo.
8. Terminal output clearly explains the baseline and force-aware results.
9. README documents public setup, execution, expected output, and limitations;
   `docs/PROJECT_STATUS.md` records implementation status and roadmap.
10. At least one Phase 2 RViz screenshot, GIF, or video is saved under
    `media/images/` or `media/figures/`.
11. No out-of-scope physics, control, planning, or hardware dependencies are
    added.

## 9. Completion Reports

For each Phase 2 task, report:

- the checklist steps completed;
- files changed;
- exact build, test, or launch commands run;
- observed result;
- remaining limitations and next incomplete step.
