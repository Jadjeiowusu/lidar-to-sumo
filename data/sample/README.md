# Sample data

**This is synthetic data. It is not from a real deployment.**

These files exist so the pipeline and the example notebooks are runnable by anyone, including people with no access to a sensor deployment. They are physically plausible but they are simulated, and no conclusion about any real intersection should be drawn from them.

Raw field data from the reference installation is not distributed. It belongs to the institution and the municipality that hosted the deployment, and continuous vehicle trajectory data can be re-identifying in low-volume conditions.

---

## Files

| File | Rows | Description |
|---|---|---|
| `detections_sample.parquet` | ~23,500 | Vehicle detections, canonical schema |
| `spat_sample.parquet` | ~68 | Signal phase state changes per group |

Regenerate at any time:

```bash
python scripts/generate_sample_data.py --out data/sample/
```

The generator is deterministic under its default seed, so the committed files are reproducible.

## What it simulates

- Four-approach signalised intersection matching the geometry in `config/intersection.yaml`
- 240 seconds at 10 Hz, roughly 4.5 arrivals per minute per approach
- Fixed dual-ring signal cycle of 90 s — ring A serves N3/S3, ring B serves E3/W4
- Poisson arrivals, free-flow approach, deceleration and queueing at the stop bar on red, discharge with start-up lost time on green
- Approach-specific stop-bar distances, so the geometry is not symmetric
- Lane assignment with realistic lateral offsets
- Gaussian position and velocity noise
- Distance-dependent LiDAR point counts, with about 0.8% zero-support ghost detections so the ghost filter has something to remove
- Roughly 10% large vehicles

## What it does not simulate

- Turning movements — every vehicle travels through
- Pedestrians, cyclists, and non-vehicle objects
- Occlusion, tracker fragmentation, and identity reassignment, which are the dominant real-world error sources
- Actuated or adaptive signal control
- Weather, lighting, and sensor degradation effects

Because tracker fragmentation is absent, **count accuracy on this dataset will be near-perfect and is not representative.** Real deployments undercount at congested approaches for exactly that reason — see [`docs/validation.md`](../../docs/validation.md) §4.

## Schema

Conforms to [`docs/data-format.md`](../../docs/data-format.md). The optional `z` column is omitted, since the pipeline does not use it.

## Tuning

```bash
python scripts/generate_sample_data.py \
    --out data/sample/ \
    --duration 600 \
    --arrivals-per-min 9 \
    --seed 42
```

Higher arrival rates build longer queues and are useful for exercising the queue-length metrics. Longer durations produce proportionally larger files.

## Licence

Released under the same Apache License 2.0 as the rest of the repository. Being synthetic, it carries no privacy or data-sharing restrictions.