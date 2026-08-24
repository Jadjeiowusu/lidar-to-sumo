#!/usr/bin/env python3
"""
02_filter_diagnostics.py
Build per-track trajectories from extracted detections and compare the
paper's original through-movement filters against a dwell-tolerant
relaxed variant. Produces the disposition table and the per-approach
recovery table for the paper.

Usage:
    python 02_filter_diagnostics.py out/            # runs on all detections_*.parquet
    python 02_filter_diagnostics.py out/ --jx 5.62 --jy 3.26

Filter variants
---------------
BASELINE (paper):
    F1 class in {VEHICLE, LARGE_VEHICLE}   (already applied upstream)
    F2 npts > 0                            (already applied upstream)
    F3 min distance to junction < 15 m
    F4 duration >= 3.0 s AND mean speed >= 1.0 m/s

RELAXED (dwell-tolerant — replaces the mean-speed test, which penalizes
vehicles that queue through Dudley's long red):
    F3 min distance to junction < 15 m
    F4' duration >= 3.0 s
        AND net displacement >= 30 m       (it actually traversed)
        AND p95 speed >= 3.0 m/s           (it genuinely moved at some point)

A queued vehicle (long dwell, then discharge) passes F4' and fails F4.
A parked/static object fails both. Tune thresholds via CLI flags.
"""
import argparse
from pathlib import Path
import numpy as np
import pandas as pd


def classify_approach(x, y, vx, vy):
    """Paper Eq. (1)/(2): position-quadrant classifier with heading fallback near origin."""
    r = np.hypot(x, y)
    phi = np.degrees(np.arctan2(vy, vx))
    pos = np.where(np.abs(x) >= np.abs(y),
                   np.where(x > 0, "N3", "S3"),
                   np.where(y > 0, "W4", "E3"))
    near = np.select(
        [np.abs(phi) >= 135, (phi >= 45) & (phi < 135), np.abs(phi) < 45],
        ["S3", "W4", "N3"], default="E3")
    return np.where(r < 8.0, near, pos)


def track_features(df, jx, jy):
    """One row of features per track_id."""
    g = df.sort_values("t_s").groupby("track_id")
    f = pd.DataFrame({
        "duration_s": g["t_s"].agg(lambda s: s.max() - s.min()),
        "n_det": g.size(),
        "mean_speed": g["speed"].mean(),
        "p95_speed": g["speed"].quantile(0.95),
        "min_dist_junction": g.apply(
            lambda t: np.hypot(t["x"] - jx, t["y"] - jy).min(), include_groups=False),
        "net_disp": g.apply(
            lambda t: np.hypot(t["x"].iloc[-1] - t["x"].iloc[0],
                               t["y"].iloc[-1] - t["y"].iloc[0]), include_groups=False),
        "dwell_frac": g["speed"].agg(lambda s: (s < 0.5).mean()),  # share of frames near-stopped
    })
    # dominant approach per track (mode of per-detection classification)
    df = df.copy()
    df["approach"] = classify_approach(df["x"].values, df["y"].values,
                                       df.get("vx", pd.Series(0, index=df.index)).values,
                                       df.get("vy", pd.Series(0, index=df.index)).values)
    f["approach"] = df.groupby("track_id")["approach"].agg(lambda s: s.mode().iat[0])
    return f.reset_index()


def apply_filters(f, a):
    f = f.copy()
    # Empirical stop-bar thresholds per approach (from 10_stopbar_distance.py)
    JUNCTION_THRESHOLDS = {"E3": 35, "N3": 36, "S3": 44, "W4": 61}
    f["pass_junction"] = f.apply(
        lambda r: r["min_dist_junction"] < JUNCTION_THRESHOLDS.get(r["approach"], a.junction_m),
        axis=1)
    f["pass_baseline"] = f["pass_junction"] & (f["duration_s"] >= a.min_dur) & (f["mean_speed"] >= a.min_mean_speed)
    f["pass_relaxed"] = (f["pass_junction"] & (f["duration_s"] >= a.min_dur)
                         & (f["net_disp"] >= a.min_disp) & (f["p95_speed"] >= a.min_p95))
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir", type=Path)
    ap.add_argument("--jx", type=float, default=5.62)
    ap.add_argument("--jy", type=float, default=3.26)
    ap.add_argument("--junction-m", type=float, default=15.0)
    ap.add_argument("--min-dur", type=float, default=3.0)
    ap.add_argument("--min-mean-speed", type=float, default=1.0)
    ap.add_argument("--min-disp", type=float, default=30.0)
    ap.add_argument("--min-p95", type=float, default=3.0)
    args = ap.parse_args()

    files = sorted(args.outdir.glob("detections_*.parquet"))
    if not files:
        raise SystemExit(f"No detections_*.parquet in {args.outdir} — run 01 first.")

    all_disp, all_recov, all_feats = [], [], []
    for fp in files:
        rec = fp.stem.replace("detections_", "")
        det = pd.read_parquet(fp)
        f = apply_filters(track_features(det, args.jx, args.jy), args)
        f["recording"] = rec
        all_feats.append(f)

        n = len(f)
        nj = int(f["pass_junction"].sum())
        disp = {
            "recording": rec,
            "tracks_total": n,
            "fail_junction_passage": n - nj,
            "junction_tracks": nj,
            "baseline_kept": int(f["pass_baseline"].sum()),
            "relaxed_kept": int(f["pass_relaxed"].sum()),
            "recovered_by_relaxed": int((f["pass_relaxed"] & ~f["pass_baseline"]).sum()),
            "dropped_by_relaxed_only": int((f["pass_baseline"] & ~f["pass_relaxed"]).sum()),
        }
        all_disp.append(disp)

        rec_tab = (f[f["pass_relaxed"] & ~f["pass_baseline"]]
                   .groupby("approach").agg(
                       recovered=("track_id", "count"),
                       mean_dwell_frac=("dwell_frac", "mean"),
                       mean_duration_s=("duration_s", "mean"),
                       mean_speed=("mean_speed", "mean")).round(2).reset_index())
        rec_tab["recording"] = rec
        all_recov.append(rec_tab)

    feats = pd.concat(all_feats)
    disp = pd.DataFrame(all_disp)
    recov = pd.concat(all_recov)
    feats.to_csv(args.outdir / "track_features.csv", index=False)
    disp.to_csv(args.outdir / "filter_disposition.csv", index=False)
    recov.to_csv(args.outdir / "recovered_by_approach.csv", index=False)

    print("=== Filter disposition per recording ===")
    print(disp.to_string(index=False))
    print("\n=== Vehicles recovered by relaxed filter, per approach ===")
    print(recov.to_string(index=False))
    print("\nIf recoveries concentrate on the Dudley approaches with high dwell_frac,")
    print("that is your evidence the mean-speed filter was dropping queued vehicles.")


if __name__ == "__main__":
    main()