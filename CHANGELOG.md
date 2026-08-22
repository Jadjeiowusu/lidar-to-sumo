# Changelog

All notable changes to this project are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

Nothing yet.

---

## [0.1.0] — 2026-08-25

Initial public release, accompanying the manuscript *From Sensor to Simulation: An Empirical LiDAR-Driven Framework for Intersection Reconstruction and Trajectory Replay in SUMO*, submitted to the Transportation Research Board 2027 Annual Meeting.

### Added

**Extraction**
- Reader for rosbag2 SQLite3 archives emitting the canonical detection and SPaT schemas
- Class filtering and ghost-detection removal on LiDAR point support
- Parquet output with dtype preservation across pipeline stages
- Schema validation with track-stability diagnostics

**Network generation**
- Empirical road axis derivation from vehicle detection scatter via singular value decomposition
- Junction centre estimation as the least-squares intersection of fitted road axes
- Cross-recording geometric stability gate on axis angle standard deviation
- Lane geometry assembly and `netconvert` compilation
- Empirical determination of the sensor-to-simulation coordinate offset

**SPaT**
- Time-indexed signal phase structure supporting lookup at arbitrary instants
- Empirical inference of controller signal group to approach mapping from observed phase sequencing

**Trajectory extraction**
- Position quadrant approach classifier with velocity-heading fallback near the junction
- Track assembly by identifier with temporal ordering
- Junction passage filtering against approach-specific empirical stop-bar thresholds
- Ingress filtering by velocity direction, bypassed for near-stationary vehicles
- Signal state attachment at zone entry

**Replay**
- Coordinate and heading transformation to the SUMO frame
- Vehicle spawning with signal compliance and lane changing disabled, preserving observed kinematics
- Frame-synchronised position updates at sensor rate via `moveToXY`
- Residual-based vehicle removal

**Metrics**
- Control delay, stopped delay, queue length, and throughput per signal cycle
- Dual computation from LiDAR records and from SUMO via TraCI, for comparison

**Validation**
- Automatic count parity check between extracted trajectories and replay, failing the pipeline on mismatch
- Three-way comparison against manual camera ground truth
- Geometric stability reporting across recordings
- Validation report generation

**Documentation**
- Stage-by-stage pipeline description
- Input schema and adapter guide
- Validation methodology, results, and known limitations
- Configuration reference with derivation procedures and a failure-mode table

**Packaging**
- Installable package with console entry point
- Three example notebooks walking the pipeline end to end
- Synthetic sample dataset so the pipeline is runnable without a deployment

### Validation at initial release

Six 30-minute recordings spanning peak, off-peak, and overnight conditions at a four-approach signalised intersection; approximately 550,000 LiDAR object frames and 108,000 SPaT records.

- Ground-truth accuracy 93.7% overall against manual camera counts
- Exact count parity between extraction and replay across all six recordings
- Axis angle standard deviation below 2° across recordings
- Mean replay frame residual 47 ms

Full results and limitations: [`docs/validation.md`](docs/validation.md).

### Known limitations

- Replayed vehicles do not interact; the replay is a record, not a behavioural simulation
- Validation uses LiDAR alone; radar fusion is available at the reference site but unused
- Ground truth available for two of four approaches and three of six recordings
- Free-flow speed is derived from the posted limit rather than measured

---

[Unreleased]: https://github.com/<Jadjeiowusu>/lidar-to-sumo/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/<Jadjeiowusu>/lidar-to-sumo/releases/tag/v0.1.0