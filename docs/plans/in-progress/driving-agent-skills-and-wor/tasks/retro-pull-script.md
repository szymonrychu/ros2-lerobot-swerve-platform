---
title: Retro pull script
status: in-progress
repo: ros2-lerobot-sverve-platform
value: "Developer agents on the Mac can pull the robot's retrospectives into the repo's logs in one command."
files: [scripts/agent_retrospectives.py, scripts/agent_retrospectives.yaml, tests/test_agent_retrospectives_script.py, tests/README.md]
verify: "uv run pytest tests/test_agent_retrospectives_script.py -q && uv run poe lint && uv run ruff check scripts/agent_retrospectives.py && uv run ruff format --check scripts/agent_retrospectives.py"
depends_on: [retro-api-endpoint, web-ui-retros-proxy]
owns: []
uses: [retro-api]
model: sonnet
estimate: 40
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/driving-agent-skills-and-wor--retro-pull-script
branch: driving-agent-skills-and-wor--retro-pull-script
verified_head: ""
run: ""
provides: []
---

# Retro pull script

## Requirement

- New `scripts/agent_retrospectives.yaml` (D10, file config): `base_url: http://client.ros2.lan/api/agent`, `limit: 50`, `output_dir: .logs/retrospectives`, `timeout_s: 10`.
- New `scripts/agent_retrospectives.py`: `uv run python scripts/agent_retrospectives.py [config.yaml]` (default the yaml next to it); pydantic config (extra forbid); fetches `GET <base_url>/retrospectives?limit=` with httpx; writes `<output_dir>/retrospectives-<YYYYmmdd-HHMMSS>.md` (one section per record: date, workflow, outcome, author, summary, went well/wrong, improvements, notes for fixers, skipped mandatory steps, phases) and the raw `.jsonl` beside it; prints both paths and the count; exit 0 on success, 1 on HTTP or connection error with the reason, 2 on a bad config. Functions typed, logging via the standard library.
- `tests/README.md` lists the new test file.
- Tests first in new `tests/test_agent_retrospectives_script.py` (httpx.MockTransport or an injected fetch), seen failing: config validation; Markdown rendering of a sample record including skipped steps and notes for fixers; files written with the timestamped names; exit codes 1 and 2.

## Acceptance conditions

- a1: the script renders retrospectives to Markdown and JSONL [evidence: command: uv run pytest tests/test_agent_retrospectives_script.py -q -k render]
- a2: config errors exit 2 and fetch errors exit 1 [evidence: command: uv run pytest tests/test_agent_retrospectives_script.py -q -k exit]
- a3: lint passes [evidence: command: uv run ruff check scripts/agent_retrospectives.py && uv run ruff format --check scripts/agent_retrospectives.py && uv run poe lint]

## Files

- `scripts/agent_retrospectives.py`
- `scripts/agent_retrospectives.yaml`
- `tests/test_agent_retrospectives_script.py`
- `tests/README.md`

## Interfaces

- `scripts/agent_retrospectives.py` reading `scripts/agent_retrospectives.yaml`; consumes `GET /api/agent/retrospectives`.

## Expected outcome

`uv run python scripts/agent_retrospectives.py` drops the robot's retrospectives into `.logs/retrospectives/`.

## Steps

- Write the test; run it; expect failures.
- Implement; run the verify command; commit `feat(scripts): pull agent retrospectives`.

## Notes

- Pattern: scripts/eval (run.yaml + pydantic config, base_url through the web_ui proxy).
