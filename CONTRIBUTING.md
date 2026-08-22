# Contributing

Contributions are welcome. Two kinds are especially valuable:

**Sensor adapters.** The pipeline reads one canonical schema ([`docs/data-format.md`](docs/data-format.md)). An adapter for radar, camera-based tracking, or a fusion stack makes the toolkit usable at deployments that look nothing like the reference site.

**Validation from other intersections.** Results from a second site are worth more to this project than any amount of additional work at the first one — including, and especially, results that disagree with ours. If the toolkit performs worse at your intersection, that is a finding and we would like to know about it.

---

## Reporting validation results

Open an issue with the `validation` label and include:

- Sensor configuration — modality, mounting, sample rate, tracker
- Approach geometry — number of approaches, lane counts, signal plan type
- Track stability statistics from [`docs/validation.md`](docs/validation.md) §3
- Count comparison against ground truth, if you have it
- Any failure mode you hit and how you diagnosed it

Anonymised or aggregate reporting is fine. Do not post raw trajectory data — see Data and privacy below.

## Reporting bugs

Include the pipeline stage, your `intersection.yaml` (redact site identifiers if needed), the command you ran, and the full traceback. If it is a replay problem, say whether you have verified the `netconvert` offset visually in `sumo-gui` — that accounts for a large share of replay issues and there is no automated check for it.

## Suggesting changes

Open an issue before writing a large pull request. Changes affecting the derivation methods in stages 2 and 4 need to be justified against the validation results, since those methods are what the accompanying paper reports.

---

## Development setup

```bash
git clone https://github.com/<your-handle>/lidar-to-sumo.git
cd lidar-to-sumo
python -m venv .venv && source .venv/bin/activate
pip install -e ".[dev,notebooks]"
pytest
```

Stage 5 additionally requires SUMO 1.18+ with `netconvert` on your path and `traci` importable. Tests touching SUMO are marked `@pytest.mark.sumo` and skip cleanly without it.

## Standards

- **Formatting:** `black` and `ruff`, both configured in `pyproject.toml`. Run `black . && ruff check --fix .` before committing.
- **Types:** annotate public functions. `mypy` is configured but not enforced in CI.
- **Tests:** new logic needs a test. Numerical routines — axis fitting, junction estimation, coordinate transforms — need a test with known input and expected output, not just a smoke test.
- **Docstrings:** Google style. Say what the function does and what the parameters mean physically, including units.

## Configuration discipline

**Nothing site-specific goes in code.** Every intersection-dependent value belongs in `config/intersection.yaml` with an entry in [`docs/configuration.md`](docs/configuration.md) explaining how to derive it. A pull request that hard-codes a threshold, a lane count, or an offset will be asked to move it.

This is the constraint that makes the toolkit reusable rather than a record of one deployment.

## Pull requests

One logical change per PR. Reference the issue it addresses. Update the relevant document under `docs/` in the same PR — documentation drift is harder to fix later than it is to prevent now. Add a `CHANGELOG.md` entry under Unreleased.

---

## Data and privacy

Do not commit raw sensor recordings, and do not attach them to issues. Continuous vehicle trajectory data can be re-identifying in low-volume conditions, and raw data from the reference deployment belongs to the institution and the municipality that hosted it.

Sample data in `data/sample/` is synthetic and exists so the pipeline is runnable without access to a deployment. Keep it that way.

## Licensing

Contributions are accepted under the Apache License 2.0, the project's licence. By submitting a pull request you confirm you have the right to contribute the code under those terms — worth checking if you are working under a sponsored research agreement, as institutional IP policies vary.

## Conduct

Be straightforward and be kind. Assume the person on the other end is trying to make an intersection safer.