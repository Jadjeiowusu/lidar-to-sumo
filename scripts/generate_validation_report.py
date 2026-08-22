#!/usr/bin/env python3
"""Generate a small synthetic dataset in the canonical LiDAR-to-SUMO schema.

The output is *not* real field data. It is a physically plausible simulation
of what a fixed-infrastructure LiDAR deployment and a roadside unit would
publish at a four-approach signalised intersection, produced so that the
pipeline and the example notebooks are runnable by anyone, including people
with no access to a sensor deployment.

Vehicles arrive by a Poisson process on each approach, travel inbound along
an approach axis, decelerate and queue at the stop bar when their signal is
red, discharge on green, and exit downstream. The signal runs a fixed dual-ring
cycle. Detections carry position noise, a distance-dependent point count, and
an occasional zero-support ghost so that the ghost filter has something to do.

Usage:
    python scripts/generate_sample_data.py --out data/sample/
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

# --------------------------------------------------------------------------
# Site constants — mirror config/intersection.yaml for the reference site
# --------------------------------------------------------------------------

JUNCTION_CENTER = np.array([3.51, 2.91])   # metres, sensor frame
SAMPLE_RATE_HZ = 10.0
FREE_FLOW_MPS = 13.4
ACCEL_MPS2 = 2.2
DECEL_MPS2 = 2.8
JAM_SPACING_M = 7.0
DETECTION_RANGE_M = 80.0
LANE_WIDTH_M = 3.6

# Upstream unit vectors in the sensor frame. The sensor frame is rotated 90
# degrees clockwise from real-world cardinal directions, which is why the
# approach labels do not line up with the axis names.
APPROACHES: dict[str, dict] = {
    "N3": {"upstream": np.array([1.0, 0.0]),  "lanes": 3, "stopbar_m": 28.0, "groups": (7, 8)},
    "S3": {"upstream": np.array([-1.0, 0.0]), "lanes": 3, "stopbar_m": 30.0, "groups": (3, 4)},
    "E3": {"upstream": np.array([0.0, -1.0]), "lanes": 3, "stopbar_m": 26.0, "groups": (1, 2)},
    "W4": {"upstream": np.array([0.0, 1.0]),  "lanes": 4, "stopbar_m": 32.0, "groups": (5, 6)},
}

# Dual-ring fixed cycle. Ring A serves N3/S3, ring B serves E3/W4.
CYCLE_S = 90.0
RING_A_GREEN_S = 35.0
YELLOW_S = 4.0
ALL_RED_S = 2.0
RING_B_GREEN_S = CYCLE_S - RING_A_GREEN_S - 2 * (YELLOW_S + ALL_RED_S)

RING_A = {"N3", "S3"}


def signal_state(approach: str, t: float) -> str:
    """Return RED, GREEN or YELLOW for an approach at simulation time t."""
    phase = t % CYCLE_S
    a_green_end = RING_A_GREEN_S
    a_yellow_end = a_green_end + YELLOW_S
    b_start = a_yellow_end + ALL_RED_S
    b_green_end = b_start + RING_B_GREEN_S
    b_yellow_end = b_green_end + YELLOW_S

    in_ring_a = approach in RING_A
    if in_ring_a:
        if phase < a_green_end:
            return "GREEN"
        if phase < a_yellow_end:
            return "YELLOW"
        return "RED"
    if phase < b_start:
        return "RED"
    if phase < b_green_end:
        return "GREEN"
    if phase < b_yellow_end:
        return "YELLOW"
    return "RED"


def green_start_after(approach: str, t: float) -> float:
    """Earliest time >= t at which the approach shows GREEN."""
    step = 0.1
    probe = t
    for _ in range(int(2 * CYCLE_S / step) + 1):
        if signal_state(approach, probe) == "GREEN":
            return probe
        probe += step
    return t


@dataclass
class Vehicle:
    track_id: int
    approach: str
    lane: int
    arrival_t: float
    is_large: bool
    stop_release_t: float | None = None
    samples: list[tuple] = field(default_factory=list)


def simulate_approach(
    approach: str,
    spec: dict,
    duration_s: float,
    arrivals_per_min: float,
    rng: np.random.Generator,
    next_id: int,
) -> tuple[list[Vehicle], int]:
    """Simulate one approach and return its vehicles."""
    upstream = spec["upstream"]
    perp = np.array([-upstream[1], upstream[0]])
    stopbar = spec["stopbar_m"]
    n_lanes = spec["lanes"]

    # Poisson arrivals at the detection boundary
    mean_gap = 60.0 / arrivals_per_min
    entry_times: list[float] = []
    t = rng.exponential(mean_gap)
    while t < duration_s:
        entry_times.append(t)
        t += rng.exponential(mean_gap)

    vehicles: list[Vehicle] = []
    # Time each lane's stop bar is next free, and the tail of its queue
    lane_free_t = np.zeros(n_lanes)
    lane_queue_tail = np.full(n_lanes, stopbar)
    lane_queue_clear_t = np.zeros(n_lanes)

    for entry_t in entry_times:
        lane = int(rng.integers(0, n_lanes))
        is_large = bool(rng.random() < 0.10)
        v = Vehicle(next_id, approach, lane, entry_t, is_large)
        next_id += 1

        # Free-flow run from the detection boundary to the stop bar
        cruise_dist = DETECTION_RANGE_M - stopbar
        arrive_stopbar_t = entry_t + cruise_dist / FREE_FLOW_MPS

        state = signal_state(approach, arrive_stopbar_t)
        queued = state in ("RED", "YELLOW") or arrive_stopbar_t < lane_queue_clear_t[lane]

        if queued:
            # Position in the standing queue for this lane. The tail resets
            # whenever the previous queue has discharged.
            if arrive_stopbar_t < lane_queue_clear_t[lane]:
                lane_queue_tail[lane] += JAM_SPACING_M + (2.0 if is_large else 0.0)
            else:
                lane_queue_tail[lane] = stopbar
            hold_pos = lane_queue_tail[lane]
            release_t = green_start_after(approach, max(arrive_stopbar_t, lane_free_t[lane]))
            # Start-up lost time grows down the queue
            queue_index = int((hold_pos - stopbar) / JAM_SPACING_M)
            release_t += 1.5 + 1.1 * queue_index
            lane_free_t[lane] = release_t + 1.1
            lane_queue_clear_t[lane] = release_t + 2.0
            v.stop_release_t = release_t
        else:
            hold_pos = None
            lane_queue_tail[lane] = stopbar
            lane_free_t[lane] = max(lane_free_t[lane], arrive_stopbar_t + 1.0)

        # Sample the trajectory at the sensor rate
        dt = 1.0 / SAMPLE_RATE_HZ
        d = DETECTION_RANGE_M          # distance upstream of junction centre
        speed = FREE_FLOW_MPS
        t_now = entry_t
        guard = 0

        while d > -DETECTION_RANGE_M and guard < 6000:
            guard += 1

            if queued and hold_pos is not None and t_now < v.stop_release_t:
                gap = d - hold_pos
                if gap > 0.5:
                    # Decelerate smoothly into the stop position
                    target = max(0.0, np.sqrt(max(2 * DECEL_MPS2 * gap, 0.0)))
                    speed = min(speed, target)
                    speed = max(speed - DECEL_MPS2 * dt, 0.0) if target < speed else speed
                else:
                    d = hold_pos
                    speed = 0.0
            else:
                speed = min(speed + ACCEL_MPS2 * dt, FREE_FLOW_MPS)

            d -= speed * dt
            t_now += dt

            lane_offset = (v.lane - (n_lanes - 1) / 2.0) * LANE_WIDTH_M
            pos = JUNCTION_CENTER + upstream * d + perp * lane_offset
            pos = pos + rng.normal(0.0, 0.12, size=2)

            vel = -upstream * speed + rng.normal(0.0, 0.08, size=2)

            rng_dist = np.linalg.norm(pos - JUNCTION_CENTER)
            base_pts = 900.0 * np.exp(-rng_dist / 42.0) * (1.6 if is_large else 1.0)
            n_pts = max(int(rng.normal(base_pts, base_pts * 0.18)), 1)
            if rng.random() < 0.008:      # occasional ghost extrapolation
                n_pts = 0

            v.samples.append(
                (t_now, pos[0], pos[1], vel[0], vel[1], n_pts)
            )

        vehicles.append(v)

    return vehicles, next_id


def build_detections(vehicles: list[Vehicle], t0_ns: int) -> pd.DataFrame:
    rows = []
    for v in vehicles:
        cls = "LARGE_VEHICLE" if v.is_large else "VEHICLE"
        for (t, x, y, vx, vy, n_pts) in v.samples:
            rows.append(
                (
                    t0_ns + int(t * 1e9),
                    v.track_id,
                    round(float(x), 3),
                    round(float(y), 3),
                    round(float(vx), 3),
                    round(float(vy), 3),
                    cls,
                    int(n_pts),
                )
            )
    df = pd.DataFrame(
        rows,
        columns=["t", "track_id", "x", "y", "vx", "vy", "vehicle_class", "n_pts"],
    )
    return df.sort_values("t", kind="stable").reset_index(drop=True)


def build_spat(duration_s: float, t0_ns: int) -> pd.DataFrame:
    """Emit one row per phase-state change per signal group."""
    rows = []
    dt = 0.1
    last: dict[int, str] = {}
    steps = int(duration_s / dt)
    for i in range(steps):
        t = i * dt
        for approach, spec in APPROACHES.items():
            state = signal_state(approach, t)
            for g in spec["groups"]:
                if last.get(g) != state:
                    # Time until this state ends
                    remaining = 0.0
                    probe = t
                    while probe < t + CYCLE_S and signal_state(approach, probe) == state:
                        probe += dt
                        remaining += dt
                    rows.append((t0_ns + int(t * 1e9), g, state, round(remaining, 1)))
                    last[g] = state
    df = pd.DataFrame(rows, columns=["t", "group_id", "state", "countdown_s"])
    return df.sort_values(["t", "group_id"], kind="stable").reset_index(drop=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=Path("data/sample"))
    ap.add_argument("--duration", type=float, default=240.0, help="seconds")
    ap.add_argument("--arrivals-per-min", type=float, default=4.5, help="per approach")
    ap.add_argument("--seed", type=int, default=20260825)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    args.out.mkdir(parents=True, exist_ok=True)

    # Arbitrary but fixed epoch so timestamps are stable across regenerations
    t0_ns = 1_782_000_000_000_000_000

    all_vehicles: list[Vehicle] = []
    next_id = 1000
    for approach, spec in APPROACHES.items():
        vehicles, next_id = simulate_approach(
            approach, spec, args.duration, args.arrivals_per_min, rng, next_id
        )
        all_vehicles.extend(vehicles)

    detections = build_detections(all_vehicles, t0_ns)
    spat = build_spat(args.duration, t0_ns)

    det_path = args.out / "detections_sample.parquet"
    spat_path = args.out / "spat_sample.parquet"
    detections.to_parquet(det_path, index=False, compression="snappy")
    spat.to_parquet(spat_path, index=False, compression="snappy")

    per_approach = {a: sum(1 for v in all_vehicles if v.approach == a) for a in APPROACHES}
    queued = sum(1 for v in all_vehicles if v.stop_release_t is not None)

    print(f"Wrote {det_path}  ({len(detections):,} rows, {det_path.stat().st_size/1024:.0f} KB)")
    print(f"Wrote {spat_path}  ({len(spat):,} rows, {spat_path.stat().st_size/1024:.0f} KB)")
    print(f"Vehicles: {len(all_vehicles)}  {per_approach}")
    print(f"Queued at least once: {queued} ({100*queued/len(all_vehicles):.0f}%)")
    print(f"Ghost detections (n_pts == 0): {(detections.n_pts == 0).sum()}")
    print(f"Duration: {args.duration:.0f} s   Seed: {args.seed}")


if __name__ == "__main__":
    main()