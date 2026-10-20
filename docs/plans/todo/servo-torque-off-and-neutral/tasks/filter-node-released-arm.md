---
title: Filter node released arm
status: todo
repo: ros2-lerobot-sverve-platform
value: "Teleop, web UI and autonomy cannot drive a limp follower arm and re-enable never snaps it to the leader."
files: [nodes/filter_node/filter_node/arbitration.py, nodes/filter_node/filter_node/node.py, nodes/filter_node/filter_node/config.py, nodes/filter_node/tests/test_arbitration.py, nodes/filter_node/tests/test_config.py, nodes/filter_node/README.md]
verify: "cd nodes/filter_node && uv run --frozen pytest tests/test_arbitration.py tests/test_config.py -q && uv run --frozen poe lint"
depends_on: [bridge-torque-services-topic]
owns: []
uses: [torque-state-topic]
model: sonnet
estimate: 60
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Filter node released arm

## Requirement

- Config (`config.py`): `follower_torque_state_topic` (default `/follower/torque_state`; an explicit empty value disables, via `read_topic`). `tests/test_config.py` covers default, override and disable.
- `node.py` subscribes to it with reliable + transient_local QoS depth 1 (the bridge latches it), parses the JSON `released` field (payload from bridge-torque-services-topic); an unparsable payload is logged and ignored.
- `arbitration.SourceArbiter.on_follower_released(released: bool) -> bool` (returns True on a transition). While released (D6): leader, web_ui and autonomy input are not forwarded, `should_publish_filtered` is False, nothing is published on `/follower/joint_commands`, an autonomy lease is ended, and the active source reported on the active-source topic is the new `SOURCE_RELEASED = "released"`. On released -> enabled: active source `none`, `proximity_required = True` (the leader resumes only within `takeover_threshold_rad` of the follower), and the node resets its filter state as on `resumed_after_release`, so no stale estimate is published.
- Tests first, seen failing, in `tests/test_arbitration.py`: every source refused while released; autonomy lease ended by a release; after re-enable the leader is refused until within threshold, then accepted; web UI accepted after re-enable; transitions reported once.
- `README.md` `## Command source arbitration` documents the released state.

## Acceptance conditions

- a1: while the follower is released no source is forwarded and nothing is published [evidence: test: nodes/filter_node/tests/test_arbitration.py]
- a2: after re-enable the leader resumes only via the proximity rule [evidence: test: nodes/filter_node/tests/test_arbitration.py]
- a3: follower_torque_state_topic config default, override and disable [evidence: test: nodes/filter_node/tests/test_config.py]
- a4: on_follower_released exists [evidence: symbol: nodes/filter_node/filter_node/arbitration.py::SourceArbiter.on_follower_released]
- a5: filter_node suite and lint pass [evidence: command: cd nodes/filter_node && uv run --frozen pytest tests -q && uv run --frozen poe lint]

## Files

- `nodes/filter_node/filter_node/arbitration.py`
- `nodes/filter_node/filter_node/node.py`
- `nodes/filter_node/filter_node/config.py`
- `nodes/filter_node/tests/test_arbitration.py`
- `nodes/filter_node/tests/test_config.py`
- `nodes/filter_node/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
