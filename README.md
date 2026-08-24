# LiDAR-to-SUMO

**Derive your intersection's geometry, vehicle trajectories, and signal phase data from the sensors you already have.**

If your agency has LiDAR, radar, or a roadside unit at a signalized intersection, this toolkit reconstructs the intersection from that deployment alone — no manual network drawing, no OpenStreetMap import, no field experiment — and replays real observed vehicle behavior in [SUMO](https://eclipse.dev/sumo/).

The result is a time-accurate digital record of what actually happened at your intersection, usable as a ground-truth foundation for evaluating signal timing changes in simulation before changing anything on the street.

> **Release status.** `v0.1.0` ships the extraction and network generation stages, validated against six field recordings. Trajectory extraction, SPaT indexing, SUMO replay, and the metrics suite are being ported from the research implementation and will follow in subsequent releases. See [Roadmap](#roadmap).

---

## Why this exists

Agencies identify dangerous intersections by counting crashes that already happened. That method is retrospective by construction: it needs injuries to accumulate before a location is flagged, it is statistically unstable at a single site because serious crashes are rare, it cannot see the near-misses that precede collisions, and it cannot tell you whether a countermeasure worked without waiting years for a fresh crash sample.

Continuous measurement of how vehicles actually behave fixes all four problems. The barrier has never been the concept — it has been that converting raw multi-sensor output into geometry and trajectories accurate enough to support a safety conclusion is difficult, and the procedures for doing it have not been publicly available.

This repository is those procedures.

## Who it's for

- **Municipal traffic engineering departments** with instrumented intersections and no in-house analytics capacity
- **Metropolitan planning organizations** evaluating corridor safety
- **State DOTs** measuring the effect of signal or geometric interventions
- **Transportation engineering firms** supporting agency contracts
- **Researchers** in traffic simulation, surrogate safety, and connected-vehicle evaluation

## What it does

| Stage | Input | Output | Status |
|---|---|---|---|
| 1. Extraction | rosbag2 SQLite archives | Filtered vehicle detections, SPaT records | **Released** |
| 2. Network generation | Detection point cloud | SUMO network from SVD axis fitting and least-squares junction estimation | **Released** |
| 3. SPaT indexing | Roadside unit messages | Time-indexed signal state per approach | In progress |
| 4. Trajectory extraction | Detections + geometry | Validated per-vehicle through-movement tracks | In progress |
| 5. Replay and measurement | Trajectories + network | SUMO replay at 10 Hz with delay, queue, throughput | In progress |

Each stage runs independently and writes to disk, so you can inspect or replace any of them.

## Validation

### Verified in this release

Network geometry was derived independently from six 30-minute recordings spanning peak, off-peak, and overnight conditions at a four-approach signalized intersection.

- **Pooled junction center reproduced exactly** at (3.51, 2.91) m in the sensor frame, from over 548,000 vehicle detections
- **Axis angle standard deviation below 2°** on the three-lane approaches — N3 1.85°, S3 0.60°, E3 1.21°
- **Greater spread on the four-lane W4 approach** (6.24°), driven by the two low-volume overnight recordings; wider approaches require more detections before the axis fit converges

Methodology, per-recording results, and limitations: [`docs/validation.md`](docs/validation.md).

### Reported in the accompanying paper

The full pipeline, including the stages not yet released here, achieves 93.7% ground-truth accuracy against manual camera counts, exact count parity between extracted trajectories and SUMO replay across all six recordings, and a mean replay frame residual of 47 ms.

## Quick start

```bash
git clone https://github.com/Jadjeiowusu/lidar-to-sumo.git
cd lidar-to-sumo
python -m venv .venv && source .venv/bin/activate   # Windows: .\.venv\Scripts\Activate.ps1
pip install -e .
```

Extract detections from a rosbag2 recording, then derive the network:

```bash
python scripts/build_network.py --config config/pipeline.yaml
```

To see it run without a sensor deployment, point the pipeline at the bundled synthetic sample:

```bash
cp data/sample/detections_sample.parquet outputs/extraction/
python scripts/build_network.py --config config/pipeline.yaml
```

Stage 2 requires SUMO 1.18+ with `netconvert` on your path only if you compile the emitted network. Python 3.10+ throughout.

## Using it at your intersection

Everything site-specific lives in `config/intersection.yaml`. Nothing about a particular intersection is hard-coded anywhere else — that constraint is what makes the toolkit reusable rather than a record of one deployment.

To adapt it you supply your approach labels and lane counts, your quadrant mapping, your fit range window, and your density thresholds. [`docs/configuration.md`](docs/configuration.md) gives the full parameter reference, a derivation procedure for each value, and a symptom-to-cause table for when something looks wrong.

## Data requirements

- Fixed-infrastructure LiDAR publishing an object list with position, velocity, and class
- Optional: roadside unit broadcasting SPaT messages, for signal-referenced analysis
- Recordings in rosbag2 SQLite3 format, or any source adapted to the schema in [`docs/data-format.md`](docs/data-format.md)

A small synthetic sample is included in `data/sample/` so the toolkit is runnable without access to a deployment. Raw field data from the reference installation is not distributed.

## Documentation

- [`docs/pipeline.md`](docs/pipeline.md) — stage-by-stage technical description
- [`docs/data-format.md`](docs/data-format.md) — input schema and writing adapters for other sensors
- [`docs/validation.md`](docs/validation.md) — methodology, results, and known limitations
- [`docs/configuration.md`](docs/configuration.md) — parameter reference and derivation procedures

## Roadmap

- `v0.2.0` — SPaT indexing and trajectory extraction
- `v0.3.0` — SUMO replay via TraCI, performance metrics
- `v0.4.0` — validation suite: count parity, ground-truth comparison, report generation
- Adapters for roadside radar and camera-based tracking platforms

Validation results from other intersections are the most useful contribution this project can receive — including results that disagree with ours. See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## Citing

If you use this toolkit in published work, please cite the accompanying paper:

> Adjei Owusu, J., Bhowmik, B., Al-dabbagh, A. H. A., Comert, G., and Karimoddini, A. *From Sensor to Simulation: An Empirical LiDAR-Driven Framework for Intersection Reconstruction and Trajectory Replay in SUMO.* Submitted to the Transportation Research Board 2027 Annual Meeting (paper TRBAM-27-03392).

Machine-readable citation metadata is in [`CITATION.cff`](CITATION.cff). Each release is archived with a DOI.

## Acknowledgments

Developed at North Carolina Agricultural and Technical State University. The authors acknowledge the support of the City of Greensboro, which facilitated the installation of equipment at the smart intersection and provided technical assistance throughout this research.

Supported by the U.S. Department of Transportation University Transportation Center Program through the Center for Rural and Regional Connected Communities (CR2C2) under Grant No. 69A3552348304; the National Science Foundation under Grant Nos. 2131080, 2242812, 2200457, and 2234920; the Yale ASCEND Program; and the U.S. Department of Transportation Federal Motor Carrier Safety Administration under Award No. 69A3602641756MHP0NC.

## License

Apache License 2.0 — see [`LICENSE`](LICENSE). You may use, modify, and redistribute this toolkit freely, including commercially.