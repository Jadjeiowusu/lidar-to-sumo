#!/usr/bin/env python3
"""Derive network geometry from detections and emit SUMO network files.

Fits road axes and the junction centre independently for each recording,
reports cross-recording stability, then refits on the pooled detections and
writes the network from that pooled geometry.

Usage:
    python scripts/build_network.py --config config/pipeline.yaml
    python scripts/build_network.py --config config/pipeline.yaml --report-offset
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from lidar_to_sumo.network import (
    axis_stability,
    compile_network,
    derive_netconvert_offset,
    estimate_junction_center,
    fit_axes,
    junction_stability,
    select_fit_detections,
    write_network_files,
)


def fit_recording(detections: pd.DataFrame, site: dict) -> tuple[dict, dict, np.ndarray]:
    """Fit axes and junction centre for one set of detections."""
    labels = site["sensor_frame"]["quadrant_labels"]
    approach_labels = [a["label"] for a in site["approaches"]]
    gf = site["detection_filters"]["geometry_fit"]

    fit_det = select_fit_detections(
        detections,
        labels,
        min_speed_mps=gf["min_speed_mps"],
        min_range_m=gf["min_range_m"],
        max_range_m=gf["max_range_m"],
    )
    axes = fit_axes(fit_det, approach_labels)
    junction = estimate_junction_center(axes)

    row: dict = {"jx": float(junction[0]), "jy": float(junction[1])}
    for label in approach_labels:
        axis = axes[label]
        row[f"axis_{label}_deg"] = None if axis is None else round(axis.angle_deg, 2)
        row[f"n_{label}"] = 0 if axis is None else axis.n_fit

    return row, axes, junction


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, required=True)
    ap.add_argument("--report-offset", action="store_true",
                    help="compile the network and print the netconvert offset")
    args = ap.parse_args()

    cfg = yaml.safe_load(args.config.read_text())
    site = yaml.safe_load(Path(cfg["site_config"]).read_text())
    outdir = Path(cfg["paths"]["extraction_dir"])
    netdir = Path(cfg["paths"]["network_dir"])

    approach_labels = [a["label"] for a in site["approaches"]]
    min_fit = site["network"]["min_fit_detections"]

    #files = sorted(outdir.glob("*_detections.parquet"))
    files = sorted(outdir.glob("detections_*.parquet"))
    if not files:
        raise SystemExit(f"No *_detections.parquet in {outdir} — run extraction first.")

    rows, pooled = [], []
    for path in files:
        det = pd.read_parquet(path)
        pooled.append(det)
        row, _, _ = fit_recording(det, site)
        #row["recording"] = path.stem.replace("_detections", "")
        row["recording"] = path.stem.replace("detections_", "")
        rows.append(row)

    per_recording = pd.DataFrame(rows).set_index("recording")

    stats, notes = axis_stability(per_recording, approach_labels, min_fit_detections=min_fit)
    junction_stats = junction_stability(per_recording, approach_labels, min_fit_detections=min_fit)

    pooled_row, pooled_axes, pooled_junction = fit_recording(pd.concat(pooled), site)

    netdir.mkdir(parents=True, exist_ok=True)
    per_recording.to_csv(netdir / "geometry_by_recording.csv")
    pd.DataFrame([pooled_row]).to_csv(netdir / "geometry_pooled.csv", index=False)

    nod, edg = write_network_files(
        pooled_axes,
        pooled_junction,
        site["approaches"],
        site["sensor_frame"]["quadrant_labels"],
        netdir,
        axis_length_m=site["network"]["axis_length_m"],
    )

    print("=== Geometry per recording ===")
    print(per_recording.round(2).to_string())
    print("\n=== Cross-recording stability ===")
    print(stats.to_string())
    print(f"\nJunction: {junction_stats}")
    for note in notes:
        print(f"  ! {note}")

    gate = site["network"]["max_axis_std_deg"]
    if not (stats["std_deg"] <= gate).all():
        print(f"\nWARNING: axis std exceeds gate of {gate}° — check the fit range window.")

    print(f"\nPooled junction centre: ({pooled_row['jx']:.2f}, {pooled_row['jy']:.2f}) m")
    print(f"Wrote {nod.name} and {edg.name}")

    if args.report_offset:
        net = compile_network(nod, edg, netdir / "lidar.net.xml")
        offset = derive_netconvert_offset(net, pooled_junction)
        print(f"\nnetconvert offset: [{offset[0]:.2f}, {offset[1]:.2f}]")
        print("Write this into config/intersection.yaml as network.netconvert_offset_m,")
        print("then verify visually in sumo-gui before trusting any replay.")


if __name__ == "__main__":
    main()