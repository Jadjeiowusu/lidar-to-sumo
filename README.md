# LiDAR-to-SUMO

**Turn a roadside LiDAR deployment into a replayable, validated simulation of your intersection.**

If your agency has LiDAR, radar, or a roadside unit at a signalized intersection, this toolkit derives the intersection geometry, the vehicle trajectories, and the signal phase data from that deployment alone, replays them in [SUMO](https://eclipse.dev/sumo/), and computes delay, queue length, and throughput from real observed behavior. No manual network drawing. No OpenStreetMap import. No field experiment.

The result is a time-accurate digital record of what actually happened at your intersection, which you can use to test a signal timing change in simulation before changing anything on the street.

---

## Why this exists

Agencies identify dangerous intersections by counting crashes that already happened. That method is retrospective by construction: it needs injuries to accumulate before a location is flagged, it is statistically unstable at a single site because serious crashes are rare, it cannot see the near-misses that precede collisions, and it cannot tell you whether a countermeasure worked without waiting years for a fresh crash sample.

Continuous measurement of how vehicles actually behave fixes all four problems. The barrier has never been the concept — it has been that converting raw multi-sensor output into trajectories accurate enough to support a safety conclusion is difficult, and the procedures for doing it have not been publicly available.

This repository is those procedures.

## Who it's for

- **Municipal traffic engineering departments** with instrumented intersections and no in-house analytics capacity
- **Metropolitan planning organizations** evaluating corridor safety
- **State DOTs** measuring the effect of signal or geometric interventions
- **Transportation engineering firms** supporting agency contracts
- **Researchers** in traffic simulation, surrogate safety, and connected-vehicle evaluation

## What it does

| Stage | Input | Output |
|---|---|---|
| 1. Extraction | rosbag2 SQLite archives | Filtered vehicle detections, SPaT phase records |
| 2. Network generation | Detection point cloud | SUMO network derived by SVD axis fitting and least-squares junction estimation |
| 3. SPaT indexing | Roadside unit messages | Time-indexed signal state per approach |
| 4. Trajectory extraction | Detections + geometry | Validated per-vehicle through-movement tracks |
| 5. Replay and measurement | Trajectories + network | SUMO replay at 10 Hz with delay, queue, and throughput metrics |

Each stage runs independently and writes to disk, so you can inspect or replace any of them.

## Validation

Validated against six 30-minute recordings spanning peak, off-peak, and overnight conditions at a four-approach signalized intersection — approximately 550,000 LiDAR object frames and 108,000 signal phase records.

- **Ground-truth accuracy: 93.7%** overall against manual camera counts, consistent with published roadside LiDAR counting benchmarks
- **Exact count parity** between extracted trajectories and SUMO replay across all six recordings — every extracted vehicle appears exactly once
- **Sub-2° axis stability** across recordings, confirming the derived geometry is reproducible session to session
- **Mean frame residual of 47 ms**, within one sensor period

Full validation methodology and per-approach results: [`docs/validation.md`](docs/validation.md).

## Quick start

```bash
git clone https://github.com/<Jadjeiowusu>/lidar-to-sumo.git
cd lidar-to-sumo
pip install -e .

# Run the full pipeline on the bundled sample
python scripts/run_pipeline.py --config config/pipeline.yaml
```

Requires Python 3.10+ and SUMO 1.18+ with `netconvert` and `traci` available on your path.

New to the toolkit? Start with [`examples/01_extract_detections.ipynb`](examples/) — three notebooks walk the pipeline end to end on the sample data.

## Using it at your intersection

Everything site-specific lives in `config/intersection.yaml`. Nothing about a particular intersection is hard-coded anywhere else.

To adapt the toolkit you supply your approach labels and lane counts, your stop-bar thresholds, your free-flow speed, and your signal group mapping. See [`docs/configuration.md`](docs/configuration.md) for the full parameter reference and guidance on deriving stop-bar thresholds empirically from your own recordings.

## Data requirements

- Fixed-infrastructure LiDAR publishing an object list with position, velocity, heading, and class
- Optional: roadside unit broadcasting SPaT messages, for signal-referenced metrics
- Recordings in rosbag2 SQLite3 format, or any source adapted to the schema in [`docs/data-format.md`](docs/data-format.md)

A small synthetic sample is included in `data/sample/` so the pipeline is runnable without access to a deployment. Raw field data from the reference installation is not distributed.

## Documentation

- [`docs/pipeline.md`](docs/pipeline.md) — stage-by-stage technical description
- [`docs/data-format.md`](docs/data-format.md) — input schema and adapters
- [`docs/validation.md`](docs/validation.md) — methodology and results
- [`docs/configuration.md`](docs/configuration.md) — parameter reference

## Citing

If you use this toolkit in published work, please cite the accompanying paper:

> Adjei Owusu, J., Bhowmik, B., Al-dabbagh, A. H. A., Comert, G., and Karimoddini, A. *From Sensor to Simulation: An Empirical LiDAR-Driven Framework for Intersection Reconstruction and Trajectory Replay in SUMO.* Submitted to the Transportation Research Board 2027 Annual Meeting.

Machine-readable citation metadata is in [`CITATION.cff`](CITATION.cff). Each release is archived with a DOI.

## Contributing

Issues and pull requests are welcome, particularly adapters for other sensor platforms and validation results from other intersections. See [`CONTRIBUTING.md`](CONTRIBUTING.md).

## Acknowledgments

Developed at North Carolina Agricultural and Technical State University. The authors acknowledge the support of the City of Greensboro, which facilitated the installation of equipment at the smart intersection and provided technical assistance throughout this research.

Supported by the U.S. Department of Transportation University Transportation Center Program through the Center for Rural and Regional Connected Communities (CR2C2) under Grant No. 69A3552348304; the National Science Foundation under Grant Nos. 2131080, 2242812, 2200457, and 2234920; the Yale ASCEND Program; and the U.S. Department of Transportation Federal Motor Carrier Safety Administration under Award No. 69A3602641756MHP0NC.

## License

Apache License 2.0 — see [`LICENSE`](LICENSE). You may use, modify, and redistribute this toolkit freely, including commercially.