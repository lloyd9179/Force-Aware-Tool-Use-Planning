# Phase 1 Codex Instructions: Force-Aware Tool-Use Planning Demo

This document is the single source of truth for implementing **Phase 1** of the `Force-Aware Tool-Use Planning` project. Codex should follow this file strictly when modifying the repository.

Phase 1 is intentionally limited in scope. Do **not** implement ROS2 execution, physical simulation, force control, fixture-aware TAMP, PDDLStream, Gazebo, MoveIt, PyBullet contact dynamics, or real robot control in this phase. The goal is to create a small but complete planning demo that clearly shows why force-aware planning matters.

---

## 1. Phase 1 mission

The project demonstrates one core idea:

> A robot may be able to geometrically reach a tool-use path, but still be unable to execute the task because the required wrench would exceed joint torque limits.

For a planar manipulator configuration `q`, applying a planar end-effector wrench

```text
F = [Fx, Fy, Mz]
```

requires joint torques

```text
tau = J(q).T @ F
```

A geometric planner that only checks inverse kinematics and joint limits can select a trajectory that looks valid but is physically infeasible. A force-aware planner should additionally check torque feasibility and may select a different IK branch or grasp transform.

The final Phase 1 demo must compare:

1. **Baseline planner**: checks IK and joint limits only.
2. **Force-aware planner**: checks IK, joint limits, and torque limits.

The expected final story is:

```text
same tool-tip path
+ same desired wrench
+ several candidate grasps
→ baseline finds a smooth path but violates torque limits
→ force-aware planner finds a different path or grasp that satisfies torque limits
```

---

## 2. Strict scope boundaries

### 2.1 Must implement in Phase 1

Codex should implement only the following:

- 2D planar pose utilities.
- Grasp transforms for converting tool-tip/tool poses into end-effector poses.
- A deterministic 3-link planar IK solver.
- IK candidate generation for each waypoint and grasp.
- Joint-limit filtering.
- Torque-limit filtering using `tau = J(q).T @ wrench`.
- Layered graph / dynamic programming path selection.
- Baseline planner.
- Force-aware planner.
- Structured planning diagnostics.
- Deterministic demo script.
- Visualization scripts that save figures to `media/figures/`.
- YAML configuration for the demo.
- Unit tests and end-to-end tests.
- README instructions for reproducing Phase 1.

### 2.2 Must not implement in Phase 1

Do **not** implement:

- ROS2 nodes.
- Gazebo / MoveIt / RViz integration.
- PyBullet contact simulation.
- Real robot execution.
- Dynamic simulation.
- Cartesian impedance control.
- Joint-space PD tracking.
- Fixture-aware planning.
- Table friction / clamp / weight models.
- PDDLStream.
- Robustness under parameter perturbation.
- 3D kinematics.
- General n-link numerical IK unless specifically needed as a small helper.

The current `ArmModel` may support arbitrary planar revolute chains, but the Phase 1 demo should use a deterministic **3-link planar arm**.

---

## 3. Existing code assumptions

The repository already contains core mathematical modules. Do not rewrite them unnecessarily.

Expected existing modules:

```text
src/force_tool_planning/kinematics.py
src/force_tool_planning/jacobian.py
src/force_tool_planning/torque.py
```

Expected existing functionality:

- `ArmModel`
- `joint_positions()`
- `forward_kinematics()`
- `within_joint_limits()`
- `wrap_to_pi()`
- full planar Jacobian with output corresponding to `[x_dot, y_dot, theta_dot]`
- translational Jacobian with output corresponding to `[x_dot, y_dot]`
- `joint_torques_from_wrench()`
- `joint_torques_from_force()`
- `check_torque_limits()`
- `is_torque_feasible()`
- `TorqueCheckResult`

Preserve the existing style:

- small deterministic functions
- type hints
- dataclasses for structured data
- explicit units in variable names when useful, such as `_m`, `_rad`, `_nm`
- no silent failure
- tests for mathematical correctness

---

## 4. Repository structure required by Phase 1

Codex should create or update the following files.

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
    ├── test_kinematics.py
    ├── test_jacobian.py
    ├── test_torque.py
    ├── test_transforms.py
    ├── test_ik.py
    └── test_planner.py
```

If the repository currently has a slightly different structure, adapt minimally while preserving compatibility with existing tests.

---

## 5. Dependency policy

Phase 1 should stay lightweight.

Required dependencies:

```text
numpy
matplotlib
pyyaml
pytest
```

Optional dependency:

```text
scipy
```

Use `scipy` only if needed. Prefer an analytic 3-link planar IK solver over a numerical optimizer for the main demo because the analytic solver is deterministic and easier to test.

Do not add heavy robotics dependencies in Phase 1.

---

## 6. Coordinate and frame conventions

This section is critical. Codex must keep frame conventions explicit and consistent.

### 6.1 Planar pose representation

Use a planar pose vector:

```text
pose = [x, y, theta]
```

where:

- `x` is position in meters.
- `y` is position in meters.
- `theta` is orientation in radians.

### 6.2 Pose composition

Implement 2D pose composition as:

```text
world_T_b = world_T_a ⊕ a_T_b
```

For poses:

```text
a = [x_a, y_a, theta_a]
b = [x_b, y_b, theta_b]
```

composition should be:

```text
x = x_a + cos(theta_a) * x_b - sin(theta_a) * y_b
y = y_a + sin(theta_a) * x_b + cos(theta_a) * y_b
theta = wrap_to_pi(theta_a + theta_b)
```

### 6.3 Grasp transform convention

Use the following convention in Phase 1:

```text
world_T_tool = tool pose in the world
world_T_ee   = end-effector pose in the world
tool_T_ee    = end-effector pose expressed in the tool frame
```

Then:

```text
world_T_ee = world_T_tool ⊕ tool_T_ee
```

In Phase 1, the tool-tip frame and tool frame may be treated as the same frame for simplicity:

```text
world_T_tooltip ≈ world_T_tool
```

If a separate tool-tip offset is later added, implement it explicitly as:

```text
world_T_ee = world_T_tooltip ⊕ tooltip_T_tool ⊕ tool_T_ee
```

Do not silently change these conventions.

### 6.4 Wrench convention

Use planar wrench:

```text
wrench = [Fx, Fy, Mz]
```

where the wrench is expressed in the world/end-effector task frame used by the planar Jacobian. For Phase 1, keep this simple and clearly documented.

If a wrench transform is not implemented, do not pretend it exists. State in comments and README that the wrench is applied directly in the end-effector/world planar frame for the simplified demo.

---

## 7. Required module details

## 7.1 `transforms.py`

Implement small deterministic utilities for planar poses.

Required functions:

```python
def as_pose3(pose: ArrayLike) -> np.ndarray:
    """Validate and return pose as shape-(3,) float array."""


def compose_pose(a: ArrayLike, b: ArrayLike) -> np.ndarray:
    """Return a ⊕ b for planar poses [x, y, theta]."""


def invert_pose(pose: ArrayLike) -> np.ndarray:
    """Return inverse planar transform."""


def relative_pose(a: ArrayLike, b: ArrayLike) -> np.ndarray:
    """Return a_T_b = inverse(a) ⊕ b."""


def transform_path(path: ArrayLike, transform: ArrayLike) -> np.ndarray:
    """Apply pose composition to every pose in a path."""
```

Test requirements:

- Identity composition returns original pose.
- `compose_pose(p, invert_pose(p))` is approximately identity.
- `relative_pose(a, compose_pose(a, b))` returns `b`.
- `transform_path` preserves number of waypoints.

---

## 7.2 `grasps.py`

Implement grasp definitions and conversion from tool path to end-effector path.

Required dataclass:

```python
@dataclass(frozen=True)
class Grasp:
    name: str
    tool_T_ee: np.ndarray
```

Required methods or functions:

```python
def make_default_grasps() -> list[Grasp]:
    """Return a small deterministic set of candidate grasps."""


def tool_path_to_ee_path(tool_path: np.ndarray, grasp: Grasp) -> np.ndarray:
    """Convert each world_T_tool pose into world_T_ee using tool_T_ee."""
```

Recommended default grasps:

```text
short_inline:  tool_T_ee = [-0.20, 0.00, 0.00]
long_inline:   tool_T_ee = [-0.60, 0.00, 0.00]
angled_up:     tool_T_ee = [-0.40, 0.00, 0.60]
angled_down:   tool_T_ee = [-0.40, 0.00, -0.60]
```

These are not physical gripper contacts yet. They are simplified rigid transforms used to show that grasp choice changes the required end-effector path and torque feasibility.

Test requirements:

- `tool_path_to_ee_path` returns same number of waypoints.
- Different grasps produce different end-effector paths.
- A zero grasp transform returns the same path.

---

## 7.3 `ik.py`

Implement deterministic analytic IK for a 3-link planar arm.

### 7.3.1 Main function

```python
def solve_planar_3link_ik(
    arm: ArmModel,
    target_pose: ArrayLike,
    *,
    include_joint_limit_check: bool = False,
    atol: float = 1e-9,
) -> list[np.ndarray]:
    """
    Return analytic IK candidates for a 3-link planar arm.

    target_pose is [x, y, theta].
    The solver should return up to two branches: elbow-up and elbow-down.
    If the target is unreachable, return an empty list.
    If include_joint_limit_check is True, discard candidates outside joint limits.
    """
```

### 7.3.2 Analytic IK formula

For link lengths:

```text
l1, l2, l3
```

and target:

```text
x, y, theta
```

compute wrist position:

```text
xw = x - l3 * cos(theta)
yw = y - l3 * sin(theta)
```

Then solve 2-link IK:

```text
c2 = (xw^2 + yw^2 - l1^2 - l2^2) / (2*l1*l2)
```

If `c2 < -1 - atol` or `c2 > 1 + atol`, the target is unreachable.

Clamp `c2` to `[-1, 1]` for numerical stability after the reachability check.

For each branch:

```text
s2 = ±sqrt(1 - c2^2)
q2 = atan2(s2, c2)
q1 = atan2(yw, xw) - atan2(l2*s2, l1 + l2*c2)
q3 = theta - q1 - q2
```

Wrap all angles to `[-pi, pi]`.

### 7.3.3 Candidate uniqueness

If the two branches are numerically identical, return only one candidate.

### 7.3.4 Test requirements

- IK round-trip test: for several reachable target poses, each returned `q` should satisfy `forward_kinematics(arm, q) ≈ target_pose`.
- Unreachable target returns empty list.
- Joint-limit filtering works.
- Elbow-up and elbow-down branches are both returned for a generic reachable pose.

---

## 7.4 `tasks.py`

Implement deterministic task generation.

Required dataclass:

```python
@dataclass(frozen=True)
class ToolUseTask:
    name: str
    tool_path: np.ndarray      # shape (T, 3)
    desired_wrench: np.ndarray # shape (3,)
```

Required functions:

```python
def make_horizontal_cutting_task(
    *,
    num_waypoints: int = 20,
    start_pose: tuple[float, float, float] = (1.2, 0.6, 0.0),
    length_m: float = 0.5,
    desired_wrench: tuple[float, float, float] = (0.0, -10.0, 0.0),
) -> ToolUseTask:
    """Create a simple deterministic tool path and downward force."""
```

Recommended path:

- tool moves horizontally from left to right
- orientation remains constant
- downward force is applied throughout the path

This does not need to simulate cutting. It only provides a path and wrench for planning.

Test requirements:

- path has requested number of waypoints
- path shape is `(T, 3)`
- wrench shape is `(3,)`
- orientation is constant for the default task

---

## 7.5 `planner.py`

This is the core Phase 1 module.

### 7.5.1 Required dataclasses

Implement structured candidates and results.

```python
@dataclass(frozen=True)
class IKCandidate:
    waypoint_index: int
    grasp_name: str
    q: np.ndarray
    ee_pose: np.ndarray
    tool_pose: np.ndarray
    joint_limit_feasible: bool
    torque_check: TorqueCheckResult | None
```

```python
@dataclass(frozen=True)
class PlanningResult:
    planner_name: str
    success: bool
    selected_grasp: str | None
    path_q: np.ndarray | None
    tool_path: np.ndarray
    ee_path: np.ndarray | None
    desired_wrench: np.ndarray
    failure_reason: str | None
    total_candidates: int
    joint_limit_feasible_candidates: int
    torque_feasible_candidates: int
    rejected_by_joint_limits: list[IKCandidate]
    rejected_by_torque: list[IKCandidate]
    selected_candidates: list[IKCandidate]
    max_torque_ratio: float | None
    diagnostics: dict[str, object]
```

If Python version compatibility makes `dict[str, object]` difficult, use `Dict[str, object]` from `typing`.

### 7.5.2 Candidate generation

Required function:

```python
def generate_candidates_for_grasp(
    arm: ArmModel,
    task: ToolUseTask,
    grasp: Grasp,
    *,
    check_torque: bool,
) -> list[list[IKCandidate]]:
    """
    Return layered IK candidates.

    Outer list index: waypoint index.
    Inner list: candidates for that waypoint.
    """
```

Behavior:

1. Convert tool path to end-effector path using the grasp.
2. For each end-effector waypoint, solve 3-link IK.
3. For each IK solution:
   - check joint limits
   - if `check_torque=True`, compute torque check using the task wrench
   - store full candidate info
4. Do not silently discard candidates inside this function unless explicitly requested. Candidate objects should preserve whether they fail joint limits or torque limits.

### 7.5.3 Filtering rules

Baseline planner accepts candidates where:

```text
joint_limit_feasible == True
```

Force-aware planner accepts candidates where:

```text
joint_limit_feasible == True
and torque_check.feasible == True
```

### 7.5.4 Dynamic programming path selection

Implement layered graph / dynamic programming path selection.

Required function:

```python
def select_smoothest_path(
    layers: list[list[IKCandidate]],
) -> list[IKCandidate] | None:
    """
    Select one candidate per waypoint minimizing squared joint motion.
    Return None if any layer is empty.
    """
```

Transition cost:

```text
cost(q_prev, q_curr) = sum(wrap_to_pi(q_curr - q_prev)^2)
```

Total path cost is sum of transition costs.

Tie-breaking should be deterministic. If two paths have nearly equal cost, preserve the earlier candidate order.

### 7.5.5 Baseline planner

Required function:

```python
def plan_baseline(
    arm: ArmModel,
    task: ToolUseTask,
    grasps: list[Grasp],
) -> PlanningResult:
    """Plan using IK and joint limits only."""
```

Behavior:

1. For each grasp, generate candidates with torque checks enabled or disabled depending on implementation convenience.
2. Filter candidates by joint limits only.
3. Run DP for each grasp.
4. Select the feasible grasp/path with lowest smoothness cost.
5. After selecting a baseline path, evaluate torque along the selected path for diagnostics.
6. The baseline `success` means a geometric path was found, not necessarily torque-feasible.
7. Store whether the baseline path violates torque limits in diagnostics.

Important: baseline should **not** reject paths because of torque infeasibility.

### 7.5.6 Force-aware planner

Required function:

```python
def plan_force_aware(
    arm: ArmModel,
    task: ToolUseTask,
    grasps: list[Grasp],
) -> PlanningResult:
    """Plan using IK, joint limits, and torque limits."""
```

Behavior:

1. For each grasp, generate candidates with torque checks.
2. Filter candidates by joint limits and torque feasibility.
3. Run DP for each grasp.
4. Select the feasible grasp/path with lowest smoothness cost.
5. Return structured diagnostics.

### 7.5.7 Failure reasons

Use clear failure reasons, for example:

```text
no_ik_candidates
no_joint_limit_feasible_candidates
no_torque_feasible_candidates
no_complete_layered_path
```

Do not return a bare `None` from top-level planner functions.

### 7.5.8 Test requirements

- If all layers contain one candidate, DP returns that path.
- If any layer is empty, DP returns `None`.
- DP chooses smoother path in a small synthetic example.
- Baseline can return success even if selected path violates torque limits.
- Force-aware rejects torque-infeasible candidates.
- Force-aware returns failure reason when no torque-feasible complete path exists.

---

## 7.6 `config.py`

Implement YAML loading for the demo.

Required function:

```python
def load_demo_config(path: str | Path) -> dict:
    """Load YAML config and return a validated dictionary."""
```

Optional but recommended:

```python
def arm_from_config(config: dict) -> ArmModel:
    """Build ArmModel from YAML config."""


def grasps_from_config(config: dict) -> list[Grasp]:
    """Build Grasp list from YAML config."""


def task_from_config(config: dict) -> ToolUseTask:
    """Build ToolUseTask from YAML config."""
```

Validation should catch:

- missing required sections
- wrong vector lengths
- nonpositive link lengths
- invalid torque limits
- invalid waypoint count

Do not over-engineer the config system.

---

## 7.7 `visualization.py`

Implement simple Matplotlib visualizations. Use only Matplotlib, not Seaborn.

Required functions:

```python
def plot_tool_and_ee_paths(task, grasps, output_path: str | Path) -> None:
    """Plot tool path and end-effector paths induced by each grasp."""


def plot_baseline_vs_force_aware(arm, baseline_result, force_result, output_path: str | Path) -> None:
    """Plot selected arm paths for baseline and force-aware planners."""


def plot_torque_profiles(arm, baseline_result, force_result, output_path: str | Path) -> None:
    """Plot joint torque magnitudes over waypoints with torque limits."""


def plot_candidate_filtering_summary(baseline_result, force_result, output_path: str | Path) -> None:
    """Plot or save a visual summary of candidate counts and rejections."""
```

Visualization requirements:

- Save figures to `media/figures/`.
- Create parent directories if needed.
- Do not require a display server.
- Use `plt.close(fig)` after saving.
- Keep labels and titles clear.
- Figures should support the demo story, not just show raw data.

Do not set custom colors unless already present in the codebase or necessary for clarity. If using colors, keep the usage minimal and consistent.

---

## 7.8 Scripts

### 7.8.1 `scripts/run_phase1_planner.py`

Purpose: run the force-aware planner only.

Expected behavior:

```bash
python3 scripts/run_phase1_planner.py
```

should:

1. load default config from `configs/demo_planar_arm.yaml`
2. build arm, task, and grasps
3. run `plan_force_aware`
4. print a concise planning summary
5. save figures to `media/figures/`
6. exit with code 0 if planner succeeds

### 7.8.2 `scripts/run_baseline_vs_force_aware.py`

Purpose: main Phase 1 demo script.

Expected behavior:

```bash
python3 scripts/run_baseline_vs_force_aware.py
```

should:

1. load default config from `configs/demo_planar_arm.yaml`
2. build arm, task, and grasps
3. run baseline planner
4. run force-aware planner
5. print a comparison summary
6. explicitly report whether baseline violates torque limits
7. explicitly report whether force-aware satisfies torque limits
8. save all core figures to `media/figures/`
9. exit with code 0 if the intended demo condition is met:

```text
baseline geometric success == True
baseline torque feasible == False
force-aware success == True
force-aware torque feasible == True
```

If the intended condition is not met, the script should still print diagnostics clearly.

### 7.8.3 `scripts/search_demo_scenario.py`

Purpose: helper script to tune a deterministic scenario.

This script may enumerate:

- wrench magnitudes
- torque limits
- tool path positions
- grasp transforms

Goal: find a scenario satisfying:

```text
baseline finds path
baseline violates torque limits
force-aware finds torque-feasible path
```

This script is useful during development and does not need to be part of the main demo.

---

## 8. Demo configuration

Create:

```text
configs/demo_planar_arm.yaml
```

Recommended initial content:

```yaml
arm:
  link_lengths_m: [1.0, 1.0, 0.7]
  joint_limits_rad:
    - [-3.141592653589793, 3.141592653589793]
    - [-2.6179938779914944, 2.6179938779914944]
    - [-2.6179938779914944, 2.6179938779914944]
  torque_limits_nm: [18.0, 12.0, 8.0]

task:
  name: horizontal_cutting
  num_waypoints: 20
  start_pose: [1.2, 0.6, 0.0]
  length_m: 0.5
  desired_wrench: [0.0, -10.0, 0.0]

grasps:
  - name: short_inline
    tool_T_ee: [-0.20, 0.00, 0.00]
  - name: long_inline
    tool_T_ee: [-0.60, 0.00, 0.00]
  - name: angled_up
    tool_T_ee: [-0.40, 0.00, 0.60]
  - name: angled_down
    tool_T_ee: [-0.40, 0.00, -0.60]

output:
  figures_dir: media/figures
```

Codex may tune these values if necessary to produce the intended deterministic baseline failure and force-aware success. If tuned, document the reason in comments or README.

---

## 9. Required terminal outputs

The main demo should print something similar to the following:

```text
=== Force-Aware Tool-Use Planning: Phase 1 Demo ===

Task: horizontal_cutting
Waypoints: 20
Desired wrench [Fx, Fy, Mz]: [0.0, -10.0, 0.0]
Candidate grasps: short_inline, long_inline, angled_up, angled_down

Baseline planner:
  geometric path found: yes
  selected grasp: short_inline
  torque feasible after evaluation: no
  max torque ratio: 1.43
  violating joints: [0, 1]

Force-aware planner:
  path found: yes
  selected grasp: angled_down
  torque feasible: yes
  max torque ratio: 0.82

Saved figures:
  media/figures/tool_and_ee_paths.png
  media/figures/baseline_vs_force_aware_paths.png
  media/figures/torque_profiles.png
  media/figures/candidate_filtering_summary.png
```

The exact numbers may differ, but the output must make the demo story obvious.

---

## 10. Required testing policy

All tests must pass with:

```bash
python3 -m pytest -q
```

Add tests incrementally. Do not remove existing tests unless they are clearly obsolete and replaced by stronger tests.

### 10.1 Minimum required tests before Phase 1 is considered complete

```text
tests/test_transforms.py
tests/test_ik.py
tests/test_planner.py
```

Must test:

- pose composition and inverse
- transform path behavior
- IK round trip through FK
- unreachable IK target
- joint-limit filtering
- torque filtering
- DP smoothness selection
- baseline can produce torque-infeasible path
- force-aware can avoid torque-infeasible path in deterministic scenario

### 10.2 End-to-end smoke test

Add a test that runs the core planning pipeline without plotting:

```text
build arm
build task
build grasps
run baseline
run force-aware
assert intended demo condition
```

Do not make this test fragile by depending on exact floating-point values unless necessary.

---

## 11. Completion criteria for Phase 1

Phase 1 is complete only when all of the following are true:

1. `python3 -m pytest -q` passes.
2. `python3 scripts/run_phase1_planner.py` runs successfully.
3. `python3 scripts/run_baseline_vs_force_aware.py` runs successfully.
4. Baseline planner finds a geometric path.
5. Baseline selected path violates torque limits.
6. Force-aware planner finds a path that satisfies torque limits.
7. The force-aware result differs from baseline by at least one of:
   - selected grasp
   - selected IK branch
   - selected joint trajectory
8. At least three figures are saved to `media/figures/`.
9. README explains how to install, test, and run the demo.
10. The code avoids heavy robotics dependencies.
11. The code contains no claims that Phase 1 performs real force control or physical contact simulation.

---

## 12. README update requirements

When updating `README.md`, include:

- Project title.
- Core idea.
- Relation to force-aware tool-use planning.
- Current status.
- Phase 1 scope.
- Installation instructions.
- Test command.
- Demo command.
- Expected output.
- Generated figures.
- Repository structure.
- Limitations.
- Future phases.

Do not write that the project is for joining a lab. The README should describe the project as a learning-oriented research demo.

---

## 13. Code style requirements

- Prefer clear names over short names.
- Use NumPy arrays internally.
- Validate shapes at module boundaries.
- Keep functions deterministic.
- Avoid global mutable state.
- Avoid hidden randomness in the main demo.
- If randomness is necessary in `search_demo_scenario.py`, use a fixed seed.
- Use dataclasses for structured return values.
- Preserve diagnostic information.
- Do not silently discard failed candidates without recording why.
- Keep plotting separate from planning logic.
- Keep config loading separate from planning logic.
- Keep scripts thin; core logic belongs in `src/force_tool_planning/`.

---

## 14. Suggested implementation order for Codex

Codex should implement Phase 1 in this order:

1. Update `requirements.txt`.
2. Update `__init__.py` exports for existing modules.
3. Add `transforms.py` and tests.
4. Add `grasps.py` and tests.
5. Add `ik.py` and tests.
6. Add `tasks.py`.
7. Add `planner.py` with candidate generation and DP.
8. Add baseline and force-aware planner functions.
9. Add `config.py` and `configs/demo_planar_arm.yaml`.
10. Add `scripts/search_demo_scenario.py` to tune deterministic values.
11. Add `scripts/run_baseline_vs_force_aware.py`.
12. Add `scripts/run_phase1_planner.py`.
13. Add `visualization.py`.
14. Add end-to-end tests.
15. Update README.
16. Run tests and demo scripts.
17. If the intended demo condition is not met, tune config values carefully and document the changes.

---

## 15. Things to avoid

Avoid these common mistakes:

- Building a generic robotics framework instead of the Phase 1 demo.
- Making IK numerical and non-deterministic when analytic 3-link IK is enough.
- Returning only `None` when planning fails.
- Hiding torque-infeasible baseline results.
- Filtering out candidates without keeping diagnostic information.
- Mixing plotting logic into planner functions.
- Adding ROS2 before the Phase 1 comparison works.
- Claiming the demo proves real-world force execution.
- Overcomplicating grasp physics in Phase 1.
- Changing frame conventions without updating tests and README.

---

## 16. Final Phase 1 message to preserve

The final demo should communicate this clearly:

> The baseline planner succeeds geometrically but fails under the desired wrench. The force-aware planner reasons about `J(q).T @ wrench`, rejects torque-infeasible IK candidates, and selects a grasp or IK branch that remains within joint torque limits.

This is the main research idea Phase 1 must demonstrate.
