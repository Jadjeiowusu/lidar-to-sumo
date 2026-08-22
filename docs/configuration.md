# Configuration

Everything site-specific lives in `config/intersection.yaml`. Nothing about a particular intersection is hard-coded elsewhere in the toolkit, so adapting to a new site means editing one file.

This document explains how to derive each value rather than guess it. Parameters are ordered by how much damage a wrong value does.

---

## Setting up a new site — the short path

1. Copy `config/intersection.yaml` and edit `site` and `approaches`
2. Run stage 1 on one recording
3. Derive the `netconvert` offset (§1) — **do this before anything else**
4. Derive stop-bar thresholds from the deceleration profile (§2)
5. Derive the signal group mapping empirically (§3)
6. Run the track stability check
7. Run the full pipeline and inspect the validation report

Budget an afternoon for a site you have recordings for.

---

## 1. `network.netconvert_offset_m` — get this right first

**Wrong value: every vehicle lands in the wrong place, silently.**

`netconvert` normalises the network origin during compilation, introducing a constant translation between sensor coordinates and simulation coordinates. It is constant for a given network but changes whenever the network is regenerated.

To determine it:

```bash
python scripts/build_network.py --config config/pipeline.yaml --report-offset
```

The script compares the junction centre in the pre-compilation description against its position in the compiled network and prints the difference. Write that into the config.

**Verify visually before trusting a replay.** Run one recording, open `sumo-gui`, and watch whether vehicles travel along lanes or through the median. There is no automated check that catches this — a wrong offset produces a replay that runs cleanly and means nothing.

Regenerate the network, re-derive the offset. Every time.

---

## 2. `approaches[].stopbar_distance_m`

**Wrong value: trajectories are wrongly included or excluded, and counts drift.**

This is the junction-passage threshold — how close to the junction centre a track must come to count as having passed through. It reflects real stop-bar geometry, which is rarely symmetric across approaches.

Derive it empirically rather than measuring off an aerial image:

```bash
python scripts/build_network.py --config config/pipeline.yaml --profile-stopbars
```

For each approach the script plots speed against distance from the junction centre across all recordings. Vehicles decelerate to near-zero at a characteristic distance — the stop bar. Read that distance off the profile.

Approaches with dedicated turn bays, wide medians, or setback stop lines will differ from each other by several metres. The reference site ranges from 26 m to 32 m across four approaches.

**Symptoms of a wrong value:** too small and through vehicles are rejected, undercounting; too large and vehicles on the cross street get captured, overcounting.

---

## 3. `spat.group_map`

**Wrong value: signal-referenced metrics attribute vehicles to the wrong phase.**

Controller group numbering does not reliably correspond to approach geometry. Derive the mapping from observed behaviour, not from the plan sheet:

```bash
python scripts/build_network.py --config config/pipeline.yaml --infer-spat-mapping
```

For each signal group, the script computes the fraction of moving vehicles near each stop bar during that group's GREEN intervals. The approach with the highest fraction is the one that group serves.

Cross-check the result against the controller plan. If they disagree, trust the data and find out why — a mismatch usually means either a mapping error or a phase you have misidentified.

The reference site runs a dual-ring plan with eight groups mapping in pairs to four approaches.

---

## 4. `metrics.free_flow_speed_mps`

**Wrong value: control delay is biased uniformly.**

Control delay is `actual travel time − (distance / free-flow speed)`, so this parameter sets the baseline the delay is measured against.

The reference site uses 13.4 m/s, derived from the posted speed limit. That is defensible where prevailing speeds track the posted limit and wrong where they do not.

To measure it instead, take the 85th percentile speed of vehicles crossing the intersection during green with no queue ahead — overnight recordings are ideal. Where measured and posted diverge by more than about 15%, use measured and say so when you report.

---

## 5. `sensor_frame.rotation_deg`

Rotation of the sensor frame relative to real-world cardinal directions, positive clockwise. Affects only which approach gets which label.

To determine it: run stage 1, plot the detection scatter, and compare the corridor bearings against an aerial image. The reference site is 90° clockwise.

**Symptom of a wrong value:** everything works, but N3 is actually the eastbound approach. Check the labels against the aerial once, at setup.

---

## 6. `detection_filters.geometry_fit`

Constrains which detections contribute to axis fitting.

| Parameter | Reference value | Rationale |
|---|---|---|
| `min_speed_mps` | 1.0 | Stationary vehicles cluster at the stop bar and bias the fit toward it |
| `min_range_m` | 15.0 | Near-junction detections include turning movements that do not follow the road axis |
| `max_range_m` | 80.0 | Beyond sensor range, position error grows and corridors are sparsely sampled |

Adjust `max_range_m` to your sensor's reliable range. If axis standard deviation exceeds `network.max_axis_std_deg`, tighten the range window before touching anything else.

---

## 7. `detection_filters.near_junction_radius_m`

Inside this radius, position alone cannot separate approaches, so velocity heading is used instead. Reference value 8.0 m.

Scale it to intersection size. A wide multi-lane intersection needs a larger radius; a narrow one needs less. Too large and vehicles are classified by heading where position would be more reliable.

---

## 8. `trajectory` thresholds

| Parameter | Reference value | Notes |
|---|---|---|
| `max_frame_gap_s` | 2.0 | Larger gaps suggest the tracker dropped and reacquired |
| `max_position_jump_m` | 5.0 | Jumps beyond this suggest an identity swap |
| `heading_unreliable_below_mps` | 0.5 | Below this, velocity direction is noise; ingress filter is bypassed |

These are diagnostic as much as functional. If a large share of frame pairs violate the first two, your tracker is not maintaining identity and the counts will undercount regardless of tuning.

---

## 9. `metrics` thresholds

| Parameter | Reference value | Notes |
|---|---|---|
| `stopped_speed_threshold_mps` | 0.3 | LiDAR-derived stopped delay and queue |
| `sumo_halting_threshold_mps` | 0.1 | SUMO's own default; changing it breaks comparability with SUMO outputs |
| `queue_radius_m` | 80.0 | Distance from junction centre within which queued vehicles are counted |

The two speed thresholds differ deliberately, and that difference is the main source of the systematic offset between LiDAR-derived and SUMO-derived delay documented in [`validation.md`](validation.md) §7. Aligning them reduces the offset but makes the LiDAR metric more sensitive to sensor noise at low speeds. Leave them as they are unless you have a reason.

Set `queue_radius_m` to cover the longest queue you expect. Too short and long queues are truncated.

---

## 10. `replay`

| Parameter | Reference value | Notes |
|---|---|---|
| `step_length_s` | 0.1 | Match your sensor sample rate |
| `disable_signal_compliance` | true | **Leave true.** False lets SUMO's driver model override observed kinematics — you are then simulating, not replaying |
| `disable_lane_changing` | true | Same reasoning |
| `max_frame_residual_s` | 2.0 | Vehicle removed when no frame is available within this window |
| `removal_grace_s` | 1.0 | Grace period past a trajectory's final frame |

---

## Common failure modes

| Symptom | Likely cause |
|---|---|
| Vehicles drive through the median in `sumo-gui` | `netconvert_offset_m` wrong (§1) |
| Counts far below manual counts | Stop-bar threshold too small (§2), or tracker instability |
| Counts above manual counts | Stop-bar threshold too large, or ghost filter disabled |
| Axis standard deviation exceeds gate | Geometry fit range window too wide (§6), or too few detections in that recording |
| Delay metrics implausibly large | `free_flow_speed_mps` too high (§4) |
| Throughput does not align with observed cycles | `spat.group_map` wrong (§3), or clock misalignment between streams |
| Approach labels do not match reality | `sensor_frame.rotation_deg` wrong (§5) |