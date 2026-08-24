"""Approach classification and empirical road axis derivation.

Road axes are fitted to the spatial distribution of moving vehicle detections
by singular value decomposition. No map data or manual digitisation is used:
the geometry comes from where vehicles were actually observed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

__all__ = ["ApproachAxis", "classify", "select_fit_detections", "fit_axes"]


@dataclass(frozen=True)
class ApproachAxis:
    """A fitted road axis for one approach.

    Attributes:
        label: Approach label, e.g. ``"N3"``.
        direction: Unit vector along the road axis, sensor frame. Sign is
            canonicalised to point outward from the junction toward the
            approach centroid, so angles are comparable across recordings.
        centroid: Mean position of the fitting detections, metres.
        n_fit: Number of detections that contributed to the fit.
    """

    label: str
    direction: np.ndarray
    centroid: np.ndarray
    n_fit: int

    @property
    def angle_deg(self) -> float:
        """Axis bearing in the sensor frame, degrees."""
        return float(np.degrees(np.arctan2(self.direction[1], self.direction[0])))


def classify(x: np.ndarray, y: np.ndarray, labels: dict[str, str]) -> np.ndarray:
    """Assign detections to approach corridors by position quadrant.

    The sensor frame is divided into four quadrants by the lines ``|x| = |y|``.
    Which real-world approach falls in which quadrant depends on the sensor
    mounting, so the mapping is supplied by configuration rather than assumed.

    Args:
        x: X positions in the sensor frame, metres.
        y: Y positions in the sensor frame, metres.
        labels: Quadrant to approach-label mapping with keys
            ``x_pos``, ``x_neg``, ``y_pos``, ``y_neg``.

    Returns:
        Array of approach labels, one per detection.
    """
    return np.where(
        np.abs(x) >= np.abs(y),
        np.where(x > 0, labels["x_pos"], labels["x_neg"]),
        np.where(y > 0, labels["y_pos"], labels["y_neg"]),
    )


def select_fit_detections(
    detections: pd.DataFrame,
    labels: dict[str, str],
    *,
    min_speed_mps: float,
    min_range_m: float,
    max_range_m: float,
) -> pd.DataFrame:
    """Select the detections used for geometry fitting.

    Three constraints apply. Stationary vehicles cluster at the stop bar and
    would bias the axis toward it. Near-junction detections include turning
    movements that do not follow the road axis. Beyond sensor range, position
    error grows and corridors are sparsely sampled.

    Note that the speed floor is correct here, where moving detections define
    the axes. It is *not* correct as a per-track through-movement filter.

    Args:
        detections: Detection records with ``x``, ``y`` and ``speed``.
        labels: Quadrant mapping passed through to :func:`classify`.
        min_speed_mps: Speed floor for fitting detections.
        min_range_m: Inner range bound from the sensor origin.
        max_range_m: Outer range bound from the sensor origin.

    Returns:
        Filtered copy with an added ``approach`` column.
    """
    if "speed" not in detections.columns:
        detections = detections.assign(
            speed=np.hypot(detections["vx"], detections["vy"])
        )

    r = np.hypot(detections["x"], detections["y"])
    sel = detections[
        (detections["speed"] >= min_speed_mps) & (r >= min_range_m) & (r <= max_range_m)
    ].copy()
    sel["approach"] = classify(sel["x"].to_numpy(), sel["y"].to_numpy(), labels)
    return sel


def fit_axes(
    fit_detections: pd.DataFrame,
    approaches: list[str],
    *,
    min_detections: int = 100,
) -> dict[str, ApproachAxis | None]:
    """Fit one road axis per approach by singular value decomposition.

    For each approach, the dominant right singular vector of the centred
    detection scatter is the road axis. SVD leaves the sign ambiguous — ``+d``
    and ``-d`` describe the same axis — so the sign is canonicalised to point
    outward from the junction area toward the approach centroid.

    Args:
        fit_detections: Output of :func:`select_fit_detections`.
        approaches: Approach labels to fit.
        min_detections: Below this count an approach yields ``None`` rather
            than an unreliable fit.

    Returns:
        Mapping of approach label to :class:`ApproachAxis`, or ``None`` where
        too few detections were available.
    """
    axes: dict[str, ApproachAxis | None] = {}

    for label in approaches:
        points = fit_detections.loc[
            fit_detections["approach"] == label, ["x", "y"]
        ].to_numpy()

        if len(points) < min_detections:
            axes[label] = None
            continue

        centroid = points.mean(axis=0)
        _, _, vt = np.linalg.svd(points - centroid, full_matrices=False)
        direction = vt[0] / np.linalg.norm(vt[0])

        if np.dot(direction, centroid) < 0:
            direction = -direction

        axes[label] = ApproachAxis(
            label=label,
            direction=direction,
            centroid=centroid,
            n_fit=len(points),
        )

    return axes