from pathlib import Path

from force_tool_planning.analysis.compare_execution import (
    DEFAULT_PHASE3_CONFIG,
    compare_controllers,
)
from force_tool_planning.analysis.plot_contact_results import (
    PHASE3_FIGURE_FILENAMES,
    save_phase3_figures,
)
import matplotlib.pyplot as plt


def test_phase3_contact_plots_save_required_figures_and_close(
    tmp_path: Path,
) -> None:
    comparison = compare_controllers(DEFAULT_PHASE3_CONFIG)

    output_paths = save_phase3_figures(comparison, tmp_path)

    assert set(output_paths) == set(PHASE3_FIGURE_FILENAMES)
    assert {path.name for path in output_paths.values()} == set(
        PHASE3_FIGURE_FILENAMES.values()
    )
    assert all(path.is_file() and path.stat().st_size > 0 for path in output_paths.values())
    assert plt.get_fignums() == []
