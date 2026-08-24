"""Junction centre estimation and cross-recording geometric stability.

The junction centre is the least-squares intersection of the fitted road axes.
Because the geometry is derived rather than drawn, its reproducibility across
independent recordings is a property worth measuring, and this module provides
the statistics that do so.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

from .axis_fitting import ApproachAxis

__all__ = ["estimate_junction_center", "axis_stability", "junction_stability", "StabilityReport"]


def estimate_junction_center(axes: dict[str, ApproachAxis | None]) -> np.ndarray:
    """Estimate the junction centre as the least-squares axis intersection.

    Each approach ``k`` defines a line through its centroid ``p̄ₖ`` in
    direction ``dₖ``. Minimising the sum of squared perpendicular distances
    from a point to all such lines gives

    .. math::
        \\left( \\sum_k A_k \\right) j = \\sum_k A_k \\bar{p}_k,
        \\qquad A_k = I - d_k d_k^{\\top}

    solved via pseudoinverse, which degrades gracefully when fewer than two
    independent axes are available.

    Args:
        axes: Fitted axes; ``None`` entries are skipped.

    Returns:
        Junction centre in the sensor frame, metres.

    Raises:
        ValueError: If no axes were fitted.
    """
    fitted = [a for a in axes.values() if a is not None]
    if not fitted:
        raise ValueError("Cannot estimate junction centre: no approach axes were fitted.")

    a_sum = np.zeros((2, 2))
    b_sum = np.zeros(2)

    for axis in fitted:
        d = axis.direction
        projection = np.eye(2) - np.outer(d, d)
        a_sum += projection
        b_sum += projection @ axis.centroid

    return np.linalg.pinv(a_sum) @ b_sum


@dataclass
class StabilityReport:
    """Cross-recording geometric stability statistics."""

    axis_stats: pd.DataFrame
    junction_stats: dict[str, float]
    sparse_notes: list[str]

    def passes(self, max_axis_std_deg: float) -> bool:
        """Whether every approach meets the axis stability gate."""
        return bool((self.axis_stats["std_deg"] <= max_axis_std_deg).all())


def axis_stability(
    per_recording: pd.DataFrame,
    approaches: list[str],
    *,
    min_fit_detections: int,
) -> tuple[pd.DataFrame, list[str]]:
    """Summarise axis angle spread across recordings.

    Recordings whose fit rests on few detections are excluded from the
    statistics and reported separately rather than silently averaged in — a
    sparse overnight fit should not widen the apparent spread of a geometry
    that is otherwise stable.

    Args:
        per_recording: One row per recording, indexed by recording identifier,
            with ``axis_<label>_deg`` and ``n_<label>`` columns.
        approaches: Approach labels.
        min_fit_detections: Density threshold for inclusion.

    Returns:
        A statistics frame indexed by approach, and a list of human-readable
        notes describing each excluded fit.
    """
    rows = []
    notes: list[str] = []

    for label in approaches:
        angle_col, count_col = f"axis_{label}_deg", f"n_{label}"
        sub = per_recording[[angle_col, count_col]].dropna()

        dense = sub.loc[sub[count_col] >= min_fit_detections, angle_col].astype(float)
        sparse = sub.loc[sub[count_col] < min_fit_detections]

        rows.append(
            {
                "approach": label,
                "n_recordings": int(len(dense)),
                "mean_deg": round(float(dense.mean()), 2) if len(dense) else np.nan,
                "std_deg": round(float(dense.std()), 3) if len(dense) > 1 else 0.0,
                "range_deg": (
                    round(float(dense.max() - dense.min()), 3) if len(dense) > 1 else 0.0
                ),
            }
        )

        for recording, row in sparse.iterrows():
            notes.append(
                f"{label} in {recording}: only {int(row[count_col])} fit detections "
                f"(< {min_fit_detections}) — excluded from stability statistics; "
                f"axis was {row[angle_col]}°"
            )

    return pd.DataFrame(rows).set_index("approach"), notes


def junction_stability(
    per_recording: pd.DataFrame,
    approaches: list[str],
    *,
    min_fit_detections: int,
) -> dict[str, float]:
    """Spread of the estimated junction centre across dense recordings."""
    count_cols = [f"n_{label}" for label in approaches]
    dense = per_recording[per_recording[count_cols].min(axis=1) >= min_fit_detections]

    return {
        "n_recordings": int(len(dense)),
        "jx_std_m": round(float(dense["jx"].std()), 3) if len(dense) > 1 else 0.0,
        "jy_std_m": round(float(dense["jy"].std()), 3) if len(dense) > 1 else 0.0,
    }