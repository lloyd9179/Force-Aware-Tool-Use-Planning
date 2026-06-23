"""Run the Phase 3 numeric position-only versus force-aware comparison."""

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
    format_comparison_summary,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_PHASE3_CONFIG,
        help="YAML configuration for the Phase 3 contact execution comparison.",
    )
    args = parser.parse_args()

    comparison = compare_controllers(args.config)
    print("=== Phase 3 Contact Execution Comparison ===")
    print(f"Config: {comparison.config_path}")
    print()
    print(format_comparison_summary(comparison))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
