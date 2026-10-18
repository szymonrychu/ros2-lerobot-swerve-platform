---
title: Retro pull script
status: todo
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
worktree: ""
branch: ""
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

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
