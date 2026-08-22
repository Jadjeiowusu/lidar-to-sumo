# Pipeline

The toolkit runs in five stages. Each consumes the previous stage's output and writes to disk, so any stage can be inspected, re-run, or replaced without touching the others. Stages 1–4 are offline preprocessing; stage 5 drives SUMO.

```
rosbag2  →  [1] Extraction  →  [2] Network gen  →  [3] SPaT index
                                       ↓                  ↓
                              [4] Trajectory extraction ←──┘
                                       ↓
                              [5] SUMO replay + metrics
```

---

## Stage 1 — Extraction

**Module:** `src/lidar_to_sumo/extraction/`

Each rosbag2 archive is opened directly as an SQLite3 database. Every detected object yields one record:

| Field | Type | Notes |
|---|---|---|
| `t` | int64 | ROS timestamp, nanoseconds |
| `track_id` | int | Tracker-assigned identifier |
| `x`, `y`, `z` | float | Position in sensor frame, metres |
| `vx`, `vy` | float | Velocity components, m/s |
| `speed` | float | Derived: `sqrt(vx² + vy²)` |
| `vehicle_class` | str | Sensor classification |
| `n_pts` | int | LiDAR point count for the detection |

Two filters apply in sequence:

1. **Class filter** — retain only classes listed in `detection_filters.classes` (default `VEHICLE`, `LARGE_VEHICLE`)
2. **Ghost removal** — discard detections with `n_pts == 0`, which are tracker extrapolations unsupported by LiDAR evidence

Output is written as Parquet. Reading a 30-minute archive is I/O-bound; expect a minute or two per recording.

Adapting a different sensor platform means implementing a reader that emits this schema. See [`data-format.md`](data-format.md).

---

## Stage 2 — Network generation

**Module:** `src/lidar_to_sumo/network/`

This is the stage that distinguishes the toolkit. The simulation network is derived from where vehicles were actually observed, rather than drawn by hand or imported from OpenStreetMap.

### Approach classification

Each detection is assigned to an approach corridor by a position quadrant classifier in the sensor frame. Within `detection_filters.near_junction_radius_m` of the origin, position alone cannot separate approaches, so the velocity heading angle `atan2(vy, vx)` is used instead.

### Axis fitting

Only detections satisfying `geometry_fit` constraints — moving above 1.0 m/s, between 15 m and 80 m from the origin — contribute to geometry. Slow and near-junction detections cluster around the stop bar and bias the fit.

For each approach, singular value decomposition on the detection scatter yields the dominant direction vector: the road axis.

### Junction estimation

The junction centre is the least-squares intersection of the four road axes. Each approach `k` defines a line through its centroid `p̄ₖ` in direction `dₖ`, with projection matrix `Aₖ = I − dₖdₖᵀ`. Minimising summed squared perpendicular distances gives

```
( Σₖ Aₖ ) j = Σₖ Aₖ p̄ₖ
```

solved via pseudoinverse.

### Compilation and coordinate transform

Lane geometry is assembled from the fitted axes and the configured lane counts, emitted as a SUMO network description, and compiled with `netconvert`.

`netconvert` normalises the network origin during compilation, introducing a constant offset. This is determined empirically once per network and stored as `network.netconvert_offset_m`:

```
p_SUMO = p_LiDAR + offset
```

**Determine this offset for your own network before replaying anything.** Getting it wrong places every vehicle in the wrong location, and the failure is silent. See [`configuration.md`](configuration.md).

---

## Stage 3 — SPaT indexing

**Module:** `src/lidar_to_sumo/spat/`

Roadside unit messages carry the phase state (`RED`, `GREEN`, `YELLOW`) and countdown timer for each signal group. The stage builds a time-indexed structure supporting fast lookup of signal state at any instant.

Signal groups are mapped to approaches through `spat.group_map`. Derive that mapping empirically from observed phase sequencing rather than assuming it from the controller plan sheet — group numbering does not reliably correspond to approach geometry across controllers. The method: for each group, compute the fraction of moving vehicles near each stop bar during that group's GREEN intervals. The approach with the highest fraction is the one that group serves.

This stage is optional. Without SPaT, trajectory extraction and replay still work; signal-referenced metrics and violation detection do not.

---

## Stage 4 — Trajectory extraction

**Module:** `src/lidar_to_sumo/trajectory/`

Detections are grouped by `track_id`, sorted by timestamp, and passed through four filters in order:

1. **Class filter** — `VEHICLE` and `LARGE_VEHICLE` only
2. **Ghost removal** — drop `n_pts == 0`
3. **Junction passage** — retain tracks whose minimum distance from the junction centre falls within the approach's configured threshold. Thresholds are derived empirically by locating where vehicles consistently decelerate to near-zero speed, which recovers the real stop-bar geometry of each approach rather than assuming symmetry
4. **Ingress filter** — retain tracks whose velocity at first detection points toward the junction centre (positive dot product between velocity and the vector from vehicle to junction). For near-stationary vehicles below `trajectory.heading_unreliable_below_mps`, heading is unreliable and this filter is bypassed; junction passage alone confirms ingress

Surviving tracks are through-movement ingress trajectories. Signal state at zone entry is attached from the stage 3 index.

---

## Stage 5 — SUMO replay and metrics

**Modules:** `src/lidar_to_sumo/replay/`, `src/lidar_to_sumo/metrics/`

### Coordinate and heading transform

Positions transform by the stage 2 offset. Heading converts from mathematical convention to SUMO geographic bearing:

```
ψ_SUMO = (90° − ψ_LiDAR) mod 360°
```

### Spawning and behavioural overrides

Vehicles are added via `traci.vehicle.add()`, then two overrides apply immediately:

- `setSpeedMode(0)` — disables signal compliance, safe-distance enforcement, and speed limits
- `setLaneChangeMode(0)` — disables autonomous lane changing

Both are essential. Without them SUMO's driver model overrides the observed kinematics and you are simulating rather than replaying.

Placement uses `moveToXY(keepRoute=0)` on approach and `keepRoute=2` after junction entry, which prevents erroneous snapping back to ingress lanes during egress.

### Frame synchronisation

At each simulation step the closest LiDAR frame in time is selected; the vehicle is relocated via `moveToXY` with speed set through `setSpeed`. Removal occurs when the minimum frame residual exceeds `replay.max_frame_residual_s` or simulation time overruns the trajectory's last frame by `replay.removal_grace_s`.

### Metrics

Computed twice — once from the LiDAR records directly, once from SUMO via TraCI — so the two can be compared.

| Metric | LiDAR route | SUMO route |
|---|---|---|
| Control delay | `t_actual − (distance / v_free_flow)` | — |
| Stopped delay | Cumulative time below `stopped_speed_threshold_mps` | `getAccumulatedWaitingTime()` at removal |
| Queue length | Distinct track IDs below threshold within `queue_radius_m` | `getLastStepHaltingNumber()` per edge |
| Throughput per cycle | Zone-entry timestamps cross-referenced against SPaT | — |

The two routes carry a systematic offset and should be read comparatively. See [`validation.md`](validation.md) §7.

---

## Running it

Full pipeline:

```bash
python scripts/run_pipeline.py --config config/pipeline.yaml
```

Individual stages:

```bash
python scripts/build_network.py       --config config/pipeline.yaml
python scripts/replay_recording.py    --config config/pipeline.yaml --recording 000
python scripts/generate_validation_report.py --config config/pipeline.yaml
```

Stages 1–4 are deterministic and cacheable. Stage 5 requires a working SUMO installation with `traci` importable.