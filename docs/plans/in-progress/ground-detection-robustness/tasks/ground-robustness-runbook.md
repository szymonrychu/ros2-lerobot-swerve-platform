---
title: Ground robustness runbook
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "The fixes are proven on the user's real box, ramp and board scene with the robot parked."
files: [docs/runbooks/ground-robustness.md, tests/test_ground_robustness_runbook.py, tests/README.md]
verify: "uv run pytest tests/test_ground_robustness_runbook.py -q && uv run poe lint"
depends_on: [ground-robustness-docs, ground-noise-report-script]
owns: []
uses: []
model: ""
estimate: 45
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/ground-detection-robustness--ground-robustness-runbook
branch: ground-detection-robustness--ground-robustness-runbook
verified_head: ""
run: ""
provides: []
---

# Ground robustness runbook

## Requirement

The static robot runbook that proves the fixes on the user's scene (design D12, brief fix 8). The orchestrator executes it after deploy and fills the Results table.

- `docs/runbooks/ground-robustness.md`, sections:
  - `## Safety`: the robot stays parked and charging; no drive commands; only head pan/tilt moves; no arm motion.
  - `## Scene`: the user's ground truth (ChArUco board flat at x ~0.40-0.55 m; box exactly 5 cm high at x 0.62-0.93, y +0.10..+0.35; black rubber ramp 25 cm long, 63 mm high, ~14 deg, at x 1.00-1.25, y -0.20..-0.45) and the expected classes (box obstacle, ramp warning, board warning only, empty floor nothing lethal).
  - `## Prerequisites`: head-chain-floor-plane-fit-a and this epic deployed with `scripts/deploy-nodes.sh client ground_estimator_cpp costmap_zones` (output teed to `.logs/`); topic_scraper_api enabled in `ansible/group_vars/client.yml` and deployed with `scripts/deploy-nodes.sh client topic_scraper_api`, then restored to `enabled: false` after the run.
  - `## Procedure`: per scene (box, ramp, board, empty floor) point the head at it, wait 10 s, then capture 2 minutes with `python scripts/topic_scraper_collect.py --source client=http://client.ros2.lan:18100 --select /ground/state:.data --interval 0.5` into `.logs/ground-<scene>.ndjson`, then run `python scripts/ground_noise_report.py scripts/ground_noise_report.yaml .logs/ground-<scene>.ndjson <scene>`.
  - `## Acceptance`: box obstacle >= 95% of states; ramp warning >= 90% with slope p50 within +-3 deg of 14; board no lethal state; empty floor 0 lethal points at p95; warning-area churn < 10% of states in every scene.
  - `## Results`: a table (date, scene, metric, value, threshold, pass) for the orchestrator to fill, and what to do on a failure (capture kept in `.logs/`, numbers recorded, the failing metric named in the epic record).
- `tests/test_ground_robustness_runbook.py`: `test_runbook_sections`, `test_runbook_thresholds_match_report_config` (the Acceptance numbers equal `scripts/ground_noise_report.yaml`), `test_runbook_commands_exist` (every script it names is tracked: assert on `git ls-files`, not the filesystem), `test_runbook_forbids_driving`.
- `tests/README.md`: the new test file.

Notes: deploy only through scripts/deploy-nodes.sh, never ansible-playbook; topic_scraper_api is disabled by default (port 18100 refused until enabled and deployed).

## Acceptance conditions

- a1: the runbook has the Safety, Scene, Prerequisites, Procedure, Acceptance and Results sections [evidence: test: tests/test_ground_robustness_runbook.py::test_runbook_sections]
- a2: the runbook thresholds equal the report config [evidence: test: tests/test_ground_robustness_runbook.py::test_runbook_thresholds_match_report_config]
- a3: every script the runbook names is tracked in git [evidence: test: tests/test_ground_robustness_runbook.py::test_runbook_commands_exist]
- a4: the runbook forbids driving and allows head moves only [evidence: test: tests/test_ground_robustness_runbook.py::test_runbook_forbids_driving]

## Files

- `docs/runbooks/ground-robustness.md`
- `tests/test_ground_robustness_runbook.py`
- `tests/README.md`

## Interfaces

Doc only; pinned by tests/test_ground_robustness_runbook.py; thresholds equal scripts/ground_noise_report.yaml.

## Expected outcome

`uv run pytest tests/test_ground_robustness_runbook.py -q` PASS; the orchestrator can run every scene from the doc after deploy and fill Results.

## Steps

- write tests/test_ground_robustness_runbook.py
- run `uv run pytest tests/test_ground_robustness_runbook.py -q`, see FAIL (no runbook)
- write docs/runbooks/ground-robustness.md and tests/README.md
- run the verify command, see PASS; commit `docs(runbooks): ground robustness static runbook`

## Notes

See the Notes paragraph of the Requirement; design: docs/plans/todo/ground-detection-robustness/design.md.
