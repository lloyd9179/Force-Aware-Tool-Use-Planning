"""Generate Phase 3 contact execution comparison figures."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from force_tool_planning.analysis.compare_execution import (  # noqa: E402
    DEFAULT_PHASE3_CONFIG,
    compare_controllers,
    load_phase3_config,
)
from force_tool_planning.analysis.plot_contact_results import (  # noqa: E402
    save_phase3_figures,
)


def _display_path(path: Path) -> Path:
    try:
        return path.relative_to(ROOT)
    except ValueError:
        return path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_PHASE3_CONFIG,
        help="YAML configuration for the Phase 3 contact execution comparison.",
    )
    args = parser.parse_args()

    config = load_phase3_config(args.config)
    comparison = compare_controllers(args.config)
    figure_dir = ROOT / config["output"]["figure_dir"]
    output_paths = save_phase3_figures(comparison, figure_dir)

    print("=== Phase 3 Contact Execution Figures ===")
    print(f"Config: {args.config}")
    print("Saved figures:")
    for path in output_paths.values():
        print(f"  {_display_path(path)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
