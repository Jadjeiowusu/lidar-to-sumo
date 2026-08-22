# Data format

The toolkit reads two streams: **vehicle detections** and, optionally, **signal phase (SPaT) records**. Everything downstream of stage 1 operates on these schemas alone, so bringing your own sensor platform means writing one reader that emits them.

Nothing else in the pipeline knows what hardware produced the data.

---

## 1. Detection schema

One row per detected object per frame.

| Column | Type | Unit | Required | Description |
|---|---|---|---|---|
| `t` | int64 | nanoseconds | yes | Timestamp, monotonically increasing |
| `track_id` | int64 | — | yes | Stable identifier for one object across frames |
| `x` | float64 | metres | yes | Position, sensor frame |
| `y` | float64 | metres | yes | Position, sensor frame |
| `z` | float64 | metres | no | Position, sensor frame. Unused by the pipeline; retained if present |
| `vx` | float64 | m/s | yes | Velocity component along sensor `x` |
| `vy` | float64 | m/s | yes | Velocity component along sensor `y` |
| `vehicle_class` | string | — | yes | Sensor classification label |
| `n_pts` | int64 | — | no | Point count supporting the detection |

`speed` is derived as `sqrt(vx² + vy²)` and need not be supplied.

### Requirements that actually matter

**Track ID stability is the load-bearing assumption.** The pipeline assumes one identifier per vehicle for the duration of its passage through the intersection. If your tracker reassigns IDs on occlusion, trajectories fragment, the junction-passage filter rejects the pieces, and counts fall — the failure mode observed on the congested approach in [`validation.md`](validation.md) §4. Run the stability tests in §3 of that document against your own data before trusting the counts.

**Origin at or near the junction.** Approach classification is a quadrant test in the sensor frame, so the origin should sit close to the junction centre. Exact placement is not required — stage 2 estimates the true centre — but an origin far outside the intersection breaks the classifier.

**Coordinate frame consistency.** All detections must be in one frame. If you run multiple sensors, fuse them upstream. The reference deployment fuses two LiDAR units at the ROS 2 level and publishes a single unified object list.

**Sensor frame rotation** relative to real-world cardinal directions is declared in `sensor_frame.rotation_deg` and affects only the labelling of approaches, not the geometry.

**Sample rate** is declared in `sensor_frame.sample_rate_hz`. The reference deployment runs at 10 Hz. Lower rates work but degrade the frame residual during replay; below roughly 5 Hz, kinematics between frames become interpolation rather than observation.

### Class labels

`detection_filters.classes` lists the labels to retain. The reference deployment emits `VEHICLE` and `LARGE_VEHICLE`. Map your platform's labels to whatever strings you configure — the pipeline does no normalisation, it matches exactly.

### Ghost detections

`n_pts == 0` marks a tracker extrapolation not supported by sensor returns. If your platform exposes an equivalent confidence or support field, map it to `n_pts` and set `detection_filters.min_lidar_points` accordingly. If it exposes nothing equivalent, omit the column and the filter is skipped — expect a modest overcount, since extrapolated ghosts occasionally satisfy the junction-passage test.

---

## 2. SPaT schema

One row per phase state change per signal group.

| Column | Type | Unit | Required | Description |
|---|---|---|---|---|
| `t` | int64 | nanoseconds | yes | Timestamp, same clock as detections |
| `group_id` | int64 | — | yes | Signal group identifier from the controller |
| `state` | string | — | yes | One of `RED`, `GREEN`, `YELLOW` |
| `countdown_s` | float64 | seconds | no | Time remaining in current state |

**Clock alignment is non-negotiable.** Detections and SPaT records must share a clock. The reference deployment records both into the same rosbag2 archive, which guarantees it. If your streams come from separate systems, verify alignment before trusting any signal-referenced metric — an offset of even a second or two will misattribute vehicles to the wrong phase, and nothing in the pipeline can detect that for you.

SPaT is optional. Without it the pipeline runs, trajectories extract, and replay works; you lose signal-referenced throughput and violation detection.

---

## 3. Storage

Stage 1 writes Parquet to `outputs/extraction/`:

```
outputs/extraction/
├── <recording_id>_detections.parquet
└── <recording_id>_spat.parquet
```

Parquet because it is columnar, compresses well on this data, and preserves dtypes across the pipeline. A 30-minute recording at 10 Hz produces roughly 90,000 detection rows and 18,000 SPaT rows per approach-hour of traffic.

---

## 4. Writing an adapter

Adapters live in `src/lidar_to_sumo/extraction/`. The contract is one function returning a DataFrame conforming to the schema above.

```python
# src/lidar_to_sumo/extraction/my_platform_reader.py

import pandas as pd
from .schema import DETECTION_SCHEMA, validate_detections


def read_detections(source_path: str, config: dict) -> pd.DataFrame:
    """Read detections from <platform> and return the canonical schema.

    Args:
        source_path: Path to the recording.
        config: Parsed pipeline configuration.

    Returns:
        DataFrame conforming to DETECTION_SCHEMA.
    """
    raw = ...  # platform-specific reading

    df = pd.DataFrame({
        "t":             raw["timestamp_ns"],
        "track_id":      raw["object_id"],
        "x":             raw["pos_x"],
        "y":             raw["pos_y"],
        "vx":            raw["vel_x"],
        "vy":            raw["vel_y"],
        "vehicle_class": raw["classification"],
        "n_pts":         raw.get("support_count", 0),
    })

    validate_detections(df)
    return df
```

`validate_detections()` checks column presence, dtypes, monotonic timestamps, and that track IDs are not globally unique per frame — a common symptom of a tracker that is not actually tracking.

Register the adapter in `config/pipeline.yaml`:

```yaml
extraction:
  reader: my_platform_reader
  source: /path/to/recordings/
```

### Platforms worth adapting

Contributions welcome for any of these, and results from any of them are more valuable to this project than more results from the reference site:

- Roadside radar with track output
- Camera-based detection and tracking pipelines
- Multi-sensor fusion stacks publishing unified object lists
- Existing trajectory datasets in other schemas

---

## 5. Validating your data before you trust the pipeline

Run this before drawing any conclusion from a new deployment:

```bash
python scripts/run_pipeline.py --config config/pipeline.yaml --stage extract
python -m lidar_to_sumo.validation.ground_truth --check-track-stability
```

Reports the three track stability statistics from [`validation.md`](validation.md) §3. If spatial continuity falls much below the 97.3% observed at the reference site, trajectory extraction will undercount, and no amount of downstream tuning will fix a tracker that reassigns identities.