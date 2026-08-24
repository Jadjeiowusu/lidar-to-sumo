# Changelog

All notable changes to this project are documented here.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Planned

- SPaT indexing: time-indexed phase structure and empirical signal-group inference
- Trajectory extraction: approach classification, track assembly, junction passage and ingress filtering
- SUMO replay via TraCI with observed kinematics preserved
- Performance metrics: control delay, stopped delay, queue length, throughput per cycle
- Validation suite: count parity, ground-truth comparison, report generation
- Adapters for roadside radar and camera-based tracking platforms

---

## [0.1.0] — 2026-08-24

Initial public release. Ships the extraction and network generation stages of the framework described in *From Sensor to Simulation: An Empirical LiDAR-Driven Framework for Intersection Reconstruction and Trajectory Replay in SUMO*, submitted to the Transportation Research Board 2027 Annual Meeting (paper TRBAM-27-03392).

### Added

**Extraction**
- Reader for rosbag2 SQLite3 archives, emitting the canonical detection schema
- Class filtering and ghost-detection removal on LiDAR point support
- Parquet output preserving dtypes across pipeline stages

**Network generation**
- Approach classification by position quadrant, with the sensor-frame mapping supplied by configuration rather than assumed
- Empirical road axis derivation from vehicle detection scatter via singular value decomposition, with sign canonicalization so angles are comparable across recordings
- Junction center estimation as the least-squares intersection of the fitted axes
- Cross-recording geometric stability statistics, with sparse fits excluded from the summary and reported separately rather than silently averaged in
- SUMO node and edge emission from the fitted geometry, with `netconvert` compilation
- Derivation of the constant coordinate offset `netconvert` introduces when normalizing the network origin, cross-checked against the compiled junction position

**Configuration**
- Single site configuration file holding every intersection-dependent value; no site-specific constants elsewhere in the codebase

**Documentation**
- Stage-by-stage pipeline description covering all five stages
- Input schema reference and guide to writing adapters for other sensor platforms
- Validation methodology, results, and known limitations
- Configuration reference with a derivation procedure for each parameter and a symptom-to-cause table

**Sample data**
- Synthetic detection and SPaT datasets in the canonical schema, with a deterministic generator, so the toolkit is runnable without access to a sensor deployment

**Packaging**
- Installable Python package with console entry point

### Validation at this release

Network geometry derived independently from six 30-minute recordings spanning peak, off-peak, and overnight conditions at a four-approach signalized intersection.

- Pooled junction center reproduced at (3.51, 2.91) m in the sensor frame, from over 548,000 vehicle detections
- Axis angle standard deviation below 2° on the three-lane approaches: N3 1.85°, S3 0.60°, E3 1.21°
- Four-lane W4 approach shows greater spread at 6.24°, driven by the two low-volume overnight recordings; raising the density threshold to 3,000 fitting detections reduces this to 3.45°

Full results: [`docs/validation.md`](docs/validation.md).

### Known limitations

- Only stages 1 and 2 are released; the remaining stages are being ported from the research implementation
- Axis fits on wide approaches require more detections to converge than on narrow ones, and low-volume recordings should be treated with corresponding caution
- The `netconvert` coordinate offset must be re-derived whenever the network is regenerated, and verified visually — a wrong offset produces a replay that runs cleanly and means nothing
- Validation to date comes from a single intersection

---

[Unreleased]: https://github.com/Jadjeiowusu/lidar-to-sumo/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/Jadjeiowusu/lidar-to-sumo/releases/tag/v0.1.0