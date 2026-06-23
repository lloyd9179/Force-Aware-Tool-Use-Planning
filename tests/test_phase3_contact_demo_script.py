from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import yaml

from force_tool_planning.analysis.compare_execution import DEFAULT_PHASE3_CONFIG
from force_tool_planning.analysis.plot_contact_results import PHASE3_FIGURE_FILENAMES


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT / "scripts" / "run_phase3_contact_demo.py"


def _load_demo_script_module():
    spec = importlib.util.spec_from_file_location("run_phase3_contact_demo", SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_phase3_contact_demo_script_prints_summary_and_saves_figures(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    with DEFAULT_PHASE3_CONFIG.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    figure_dir = tmp_path / "figures"
    config["output"]["figure_dir"] = str(figure_dir)
    config_path = tmp_path / "phase3.yaml"
    with config_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream)

    module = _load_demo_script_module()
    monkeypatch.setattr(
        sys,
        "argv",
        ["run_phase3_contact_demo.py", "--config", str(config_path)],
    )

    assert module.main() == 0

    output = capsys.readouterr().out
    assert "Phase 3 Contact Execution Demo" in output
    assert "Controller: position_only" in output
    assert "Controller: force_aware" in output
    assert "Saved figures:" in output
    for filename in PHASE3_FIGURE_FILENAMES.values():
        path = figure_dir / filename
        assert path.is_file()
        assert path.stat().st_size > 0
