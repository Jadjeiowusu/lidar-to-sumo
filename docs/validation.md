# Validation

This document describes how the toolkit was validated, what was measured, and where it falls short. Every figure here is reproducible from the pipeline outputs.

The short version: **93.7% ground-truth accuracy** against manual camera counts, and **exact count parity** between extracted trajectories and SUMO replay across all six recordings.

---

## 1. Dataset

Six 30-minute recordings collected on 25–26 June 2026 at a four-approach signalized intersection, spanning peak, off-peak, and overnight conditions. All sensor streams recorded simultaneously to rosbag2 SQLite3 archives.

| Recording | Local time (ET) | Duration | Tracks | Condition |
|---|---|---|---|---|
| `..._000` | 12:05–12:35 | 30 min | 602 | Peak (midday) |
| `..._001` | 17:35–18:05 | 30 min | 590 | Peak (evening) |
| `..._009` | 16:35–17:05 | 30 min | 619 | Peak (afternoon) |
| `..._018` | 02:05–02:35 | 30 min | 483 | Overnight |
| `..._027` | 01:35–02:05 | 30 min | 312 | Overnight |
| `..._039` | 08:35–09:05 | 30 min | 426 | Off-peak (morning) |

`Tracks` counts junction-passing ingress vehicles after filtering.

Totals: approximately **550,000 LiDAR object frames**, **108,000 SPaT records**, and **3,032 junction-passing ingress trajectories** across all four approaches. Observed throughput ranges from 13.6 vehicles/minute overnight to 24.2 vehicles/minute at peak.

No preprocessing, filtering, or manual annotation was applied outside the pipeline itself.

---

## 2. Geometric stability

The network geometry is derived from the spatial distribution of vehicle detections rather than drawn by hand, so the first question is whether two recordings of the same intersection produce the same geometry.

- **Axis angle standard deviation below 2°** across all dense recordings
- **Junction center**, single recording: `(5.62, 3.26) m` in the sensor frame
- **Junction center**, pooled across all six recordings: `(3.51, 2.91) m`, derived from over 548,000 vehicle detections

The pooled estimate is the one to use in production. Single-recording estimates drift most in overnight windows, where sparse detections give the least-squares fit less to work with.

**How to reproduce:** run `scripts/build_network.py` against each recording separately and compare the emitted `axis_angles` and `junction_center` fields. The pipeline fails loudly if axis standard deviation exceeds `network.max_axis_std_deg`.

---

## 3. Track identity stability

Trajectory extraction assumes the tracker assigns one stable identifier per vehicle for the duration of its passage. Three tests confirm this.

| Test | Method | Result |
|---|---|---|
| Spatial continuity | Consecutive frame pairs with gap < 2.0 s and position jump < 5.0 m | **97.3%** |
| Spatial uniqueness | Distinct track IDs within 3.0 m of each other at 1-second intervals | None observed |
| Through-movement continuity | Single ID maintained from approach entry to egress exit | Confirmed |

---

## 4. Count accuracy against ground truth

Three of the six recordings were manually counted from synchronized camera footage. The table compares manual counts, LiDAR-extracted counts, and SUMO replay counts on the two camera-covered approaches.

| Recording | Approach | Ground truth | LiDAR | SUMO | LiDAR acc. | SUMO acc. |
|---|---|---|---|---|---|---|
| `000` (12:05–12:35) | S3 | 83 | 81 | 81 | 97.6% | 97.6% |
| | E3 | 188 | 192 | 192 | 97.9% | 97.9% |
| `009` (16:35–17:05) | S3 | 104 | 90 | 90 | 86.5% | 86.5% |
| | E3 | 204 | 207 | 207 | 98.5% | 98.5% |
| `027` (01:35–02:05) | S3 | 11 | 13 | 13 | 81.8% | 81.8% |
| | E3 | 10 | 10 | 10 | 100.0% | 100.0% |
| **Overall** | | | | | **93.7%** | **93.7%** |

Accuracy is `1 − |LiDAR − ground truth| / ground truth × 100`.

This sits within the 85–98% per-direction range reported in published roadside LiDAR counting studies, and close to the 94.74% overall benchmark those studies report.

### Where it degrades

Two results deserve attention rather than rounding away:

**S3 in recording `009` (86.5%).** An undercount of 14 vehicles during the afternoon peak, attributable to tracker fragmentation during dense queuing on the S3 corridor — a queued vehicle occluded for long enough gets a new track ID on reacquisition and fails the junction-passage filter. This is the failure mode to expect at congested approaches, and it undercounts rather than overcounts.

**S3 in recording `027` (81.8%).** An overcount of 2 vehicles against a ground truth of 11. At overnight volumes a two-vehicle difference is 18% by construction. Percentage accuracy is not a meaningful statistic at these counts; the absolute error is what matters.

**Practical guidance:** treat approach-level accuracy as reliable above roughly 50 vehicles per window. Below that, report absolute counts.

---

## 5. Count parity between LiDAR and SUMO

Separate from ground-truth accuracy is whether the replay faithfully reproduces what was extracted. Across all six recordings, **every vehicle extracted from the LiDAR pipeline appears exactly once in the SUMO replay** — no losses to network snapping failures, no duplicates.

This is checked automatically on every run by `src/lidar_to_sumo/validation/count_parity.py`, which fails the pipeline on any mismatch. Parity is a property of the toolkit, not of the site, so it should hold at your intersection too. If it does not, that is a bug — please open an issue.

---

## 6. Replay fidelity

Position injection runs at the LiDAR acquisition rate of 10 Hz with a **mean frame residual of 47 ms**, well within one sensor period. A vehicle is removed when the minimum frame residual exceeds 2.0 s or when simulation time overruns the trajectory's last frame by more than 1.0 s.

---

## 7. Performance metric agreement

Metrics are computed twice by independent routes: directly from the LiDAR trajectory records, and from SUMO via TraCI during replay. Agreement between them tests whether the replay reproduces operational behavior, not just vehicle counts.

Mean accumulated waiting time per vehicle, correlation between the two routes by recording:

| Recording | Condition | Pearson *r* |
|---|---|---|
| `000` | Peak (midday) | 0.968 |
| `001` | Peak (evening) | 0.976 |
| `009` | Peak (afternoon) | 0.982 |
| `018` | Overnight | 0.990 |
| `027` | Overnight | 0.985 |
| `039` | Off-peak (morning) | 0.952 |

There is a **systematic offset in absolute values** between the two routes. SUMO's `getAccumulatedWaitingTime()` accumulates at each TraCI step under its own speed threshold (0.1 m/s), while the LiDAR computation accumulates against a different stopped-speed threshold (0.3 m/s) over the sensor record. The two are measuring closely related but not identical quantities.

**Read the metrics as directional and comparative, not as calibrated absolutes.** Ranking approaches by delay, comparing a before-and-after signal timing change, or identifying which approach carries the queue — all supported. Reporting a single delay figure as ground truth — not supported without further calibration.

Across all six recordings the S3 and N3 approaches consistently record the highest waiting time, consistent with the longer red-phase duration those approaches receive under the site's dual-ring plan. That the data recovers a known property of the signal plan is itself a check on the pipeline.

---

## 8. Known limitations

**No interaction between replayed vehicles.** A replayed vehicle follows its recorded trajectory regardless of simulated conflicts. Car-following and gap-acceptance behavior are not modeled, because the trajectories are observations rather than simulated decisions. This makes the replay a faithful record and an unsuitable base for counterfactuals that change vehicle behavior. Evaluating a signal timing change that would alter how vehicles behave requires reactive agents alongside the kinematic replay — planned, not implemented.

**Single-modality validation.** Radar data is available at the reference site but this validation uses LiDAR alone, deliberately, to establish what a single modality is sufficient for. Fusion should improve occlusion handling at congested approaches.

**Two approaches ground-truthed.** Camera coverage supported manual counting on S3 and E3 only. N3 and W4 are validated by parity and geometric stability, not against manual counts.

**Three recordings ground-truthed.** Recordings `001`, `018`, and `039` have no manual counts.

**Free-flow speed is an assumption.** Control delay uses a free-flow speed derived from the posted limit (13.4 m/s at the reference site), not measured. Sites where prevailing speeds diverge from the posted limit should measure it and set `metrics.free_flow_speed_mps` accordingly.

---

## 9. Reproducing this report

```bash
python scripts/generate_validation_report.py \
    --config config/pipeline.yaml \
    --recordings all \
    --out outputs/validation/
```

Emits the count parity check, the ground-truth comparison for any recording with manual counts configured, geometric stability statistics across recordings, and the metric agreement correlations.

---

## 10. Reporting validation from other sites

Validation results from other intersections are the most useful contribution this project can receive. If you run the toolkit at your site, please open an issue with your sensor configuration, approach geometry, count comparison if you have ground truth, and any failure modes you hit. Results that disagree with these are as valuable as results that agree.