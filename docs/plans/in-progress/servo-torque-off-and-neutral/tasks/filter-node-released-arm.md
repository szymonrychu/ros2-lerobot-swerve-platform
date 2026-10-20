---
title: Filter node released arm
status: done
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/servo-torque-off-and-neutral--filter-node-released-arm
branch: servo-torque-off-and-neutral--filter-node-released-arm
verified_head: 04325bea2dc0507adaf51ac0e5f18d3182fdea8d
run: servo-torque-off-and-neutral
provides: []
acs_for_audit: []
process_gaps: []
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

- Uses part `torque-state-topic` (`/follower/torque_state`, field `released`).
- `SourceArbiter.on_follower_released(released: bool) -> bool`; `SOURCE_RELEASED = "released"` on the active-source topic (mcp_server reads that topic for its lease, see mcp-torque-off-gate).
- Config key `follower_torque_state_topic`.

## Expected outcome

Teleop, web UI and autonomy cannot drive a limp follower arm, and re-enabling it never snaps the arm to the leader pose.

## Steps

- Write the arbitration and config tests; run `cd nodes/filter_node && uv run --frozen pytest tests/test_arbitration.py tests/test_config.py -q`; expect failures.
- Implement arbiter state, node subscription and filter reset; run the verify command; commit `feat(filter_node): quiet the follower while its torque is released`.

## Ruling

- 2026-10-20 accepted: False positives: README/test comment describe the retry-after-fault behaviour (no deferral); SourceArbiter.on_follower_released exists at arbitration.py:137 (nested-method symbol not resolved by the checker). Both verified green.

## Notes

- Existing proximity rule (arbitration.py `proximity_required`, default threshold 0.15 rad) is the re-enable guard; reuse it.
- After an arm re-zero the leader and follower zeros differ: the proximity rule then keeps the leader out until the user re-zeroes the leader (D-C2: the leader re-zero is outside this epic). Document that in the README.
