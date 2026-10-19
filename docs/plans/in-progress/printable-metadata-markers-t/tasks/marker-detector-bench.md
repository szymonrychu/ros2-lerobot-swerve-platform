---
title: Marker detector bench
status: todo
repo: ros2-lerobot-sverve-platform
value: We know what decoding and pose cost on the Pi 5 before choosing the processing rate.
files: [nodes/marker_detector/marker_detector/bench.py, nodes/marker_detector/config/bench.yaml, nodes/marker_detector/tests/test_bench.py, scripts/marker_bench.sh, scripts/README.md, nodes/marker_detector/README.md]
verify: "cd nodes/marker_detector && uv run --frozen pytest tests/test_bench.py -q && uv run --frozen poe lint && cd ../.. && bash -n scripts/marker_bench.sh"
depends_on: [marker-detector-node]
owns: []
uses: [marker-detector-config, marker-synthetic-scenes, marker-decode, marker-pose]
model: sonnet
estimate: 45
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Marker detector bench

## Requirement

- `nodes/marker_detector/marker_detector/bench.py`: `python -m marker_detector.bench [config]` (config file `nodes/marker_detector/config/bench.yaml`, pydantic: `iterations` 200, `warmup` 20, `scenes` list of {`image_size` [1152, 648] or [576, 324] or [1024, 576] (zoom crop), `markers` 1-3 with sizes and distances}, `seed`): renders the scenes once with `synthetic.py`, then times per frame `decode_frame`, `estimate_pose` per candidate and the whole frame (`time.perf_counter_ns`), and prints JSON `{scene: {stage: {p50_ms, p95_ms, max_ms}}, "decoded_ratio": ..., "cv2": version, "zxingcpp": version, "machine": platform}`; `run_bench(cfg) -> dict` for tests.
- `scripts/marker_bench.sh`: runs the bench on the client in the deployed node venv over ssh (host and user from the ansible inventory, pattern `scripts/rtk_diag.sh`), copies the JSON to `.logs/marker_bench.json`, prints the summary; `--help` text.
- Tests first in `nodes/marker_detector/tests/test_bench.py`, seen failing: `run_bench` with 3 iterations on two scenes returns every stage with p50 <= p95 <= max, decoded_ratio 1.0 for the near scenes, and the versions.
- `scripts/README.md` gains a `marker_bench.sh` entry; the node README gains a Benchmark section (how to run, the budget of design D14, where results go).

## Acceptance conditions

- a1: the bench reports p50 and p95 per stage and decode success on synthetic frames [evidence: command: cd nodes/marker_detector && uv run --frozen pytest tests/test_bench.py -q]
- a2: the robot bench script is valid shell and documented [evidence: command: bash -n scripts/marker_bench.sh && grep -q 'marker_bench.sh' scripts/README.md]

## Files

- `nodes/marker_detector/marker_detector/bench.py`
- `nodes/marker_detector/config/bench.yaml`
- `nodes/marker_detector/tests/test_bench.py`
- `scripts/marker_bench.sh`
- `scripts/README.md`
- `nodes/marker_detector/README.md`

## Interfaces

- `python -m marker_detector.bench`, `nodes/marker_detector/marker_detector/bench.py:run_bench`, `scripts/marker_bench.sh`.

## Expected outcome

One script measures decode and pose cost on the Pi 5 and stores the JSON in `.logs/`.

## Steps

- Write `tests/test_bench.py`; run `cd nodes/marker_detector && uv run --frozen pytest tests/test_bench.py -q`; see it fail.
- Implement bench.py, bench.yaml and the script; rerun; run the verify command.
- Update both READMEs; commit `feat(marker_detector): decode and pose benchmark`.

## Notes

- Do not run the script against the robot in this task; the runbook does.
