"""End-to-end Phase 1 smoke test without importing visualization code."""

import os
import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np

from force_tool_planning.config import (
    arm_from_config,
    grasps_from_config,
    load_demo_config,
    task_from_config,
)
from force_tool_planning.planner import plan_baseline, plan_force_aware

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "configs" / "demo_planar_arm.yaml"


def test_phase1_core_pipeline_smoke_without_plotting() -> None:
    config = load_demo_config(DEFAULT_CONFIG)
    arm = arm_from_config(config)
    task = task_from_config(config)
    grasps = grasps_from_config(config)

    baseline = plan_baseline(arm, task, grasps)
    force_aware = plan_force_aware(arm, task, grasps)

    assert baseline.success
    assert baseline.path_q is not None
    assert baseline.path_q.shape == (len(task.tool_path), arm.n_joints)
    assert baseline.diagnostics["torque_feasible"] is False
    assert baseline.max_torque_ratio is not None and baseline.max_torque_ratio > 1.0

    assert force_aware.success
    assert force_aware.path_q is not None
    assert force_aware.path_q.shape == (len(task.tool_path), arm.n_joints)
    assert force_aware.diagnostics["torque_feasible"] is True
    assert force_aware.max_torque_ratio is not None and force_aware.max_torque_ratio <= 1.0
    assert all(
        candidate.torque_check is not None and candidate.torque_check.feasible
        for candidate in force_aware.selected_candidates
    )

    assert baseline.selected_grasp != force_aware.selected_grasp
    assert not np.allclose(baseline.path_q, force_aware.path_q)


def test_core_pipeline_does_not_import_matplotlib() -> None:
    code = textwrap.dedent(
        """
        import sys
        from force_tool_planning.config import (
            arm_from_config,
            grasps_from_config,
            load_demo_config,
            task_from_config,
        )
        from force_tool_planning.planner import plan_baseline, plan_force_aware

        config = load_demo_config("configs/demo_planar_arm.yaml")
        arm = arm_from_config(config)
        task = task_from_config(config)
        grasps = grasps_from_config(config)
        baseline = plan_baseline(arm, task, grasps)
        force_aware = plan_force_aware(arm, task, grasps)

        assert baseline.success
        assert force_aware.success
        assert "matplotlib" not in sys.modules
        """
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(ROOT / "src")

    completed = subprocess.run(
        [sys.executable, "-c", code],
        cwd=ROOT,
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
