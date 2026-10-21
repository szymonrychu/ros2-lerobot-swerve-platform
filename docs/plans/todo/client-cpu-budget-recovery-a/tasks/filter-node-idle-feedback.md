---
title: Filter node idle feedback
status: todo
repo: ros2-lerobot-sverve-platform
value: "filter_node stops handling 89 Hz arm feedback while nobody drives the arm (about 0.08 core) without weakening the takeover rule."
files: [nodes/filter_node/filter_node/config.py, nodes/filter_node/filter_node/feedback_rate.py, nodes/filter_node/filter_node/node.py, nodes/filter_node/tests/test_feedback_rate.py, nodes/filter_node/README.md, ansible/group_vars/client.yml, tests/test_client_cpu_load.py, tests/README.md]
verify: "cd nodes/filter_node && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_client_cpu_load.py -q && uv run poe test-ansible"
depends_on: [filter-node-events-executor, client-executor-config]
owns: []
uses: []
model: ""
estimate: 75
actual: ""
review: full
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Filter node idle feedback

## Requirement

Item 5 of the brief (design D5; approval A5): filter_node reads the 10 Hz `/follower/joint_states/throttled` copy while no source is commanding, and the full-rate `/follower/joint_states` (~89 Hz) while one is.

- `filter_node/config.py`: new key `follower_feedback_idle_topic: str = ""` (empty = always full rate, today's behaviour) and `follower_feedback_idle_fallback_s: float = 2.0` (> 0).
- New module `filter_node/feedback_rate.py` with a public class `FeedbackRateSwitch` (typed, docstrings) that decides which feedback topic should be subscribed from (now, last input time of each source, idle_timeout_s, web_ui_timeout_s, autonomy lease held, last idle-feedback message time). Active = leader input within idle_timeout_s, web UI command within web_ui_timeout_s, or the autonomy lease held; otherwise idle. Idle selects the idle topic, except when no idle-topic message arrived for `follower_feedback_idle_fallback_s` after subscribing (throttle node down), which selects the full-rate topic and logs once until the idle topic delivers again.
- `filter_node/node.py`: the follower feedback subscription is created on the selected topic and swapped (destroy, then create on the other topic) from the main loop, never from inside a callback, when the selection changes. Both topics feed the same `on_follower_feedback` -> `arbiter.update_follower_positions`. The takeover/proximity rule is unchanged; the switch to full rate happens on the first active input, before the next proximity check.
- `ansible/group_vars/client.yml` filter_node `config: |`: `follower_feedback_idle_topic: /follower/joint_states/throttled` with a comment (joint_state_throttle_follower, 10 Hz).
- Tests first (red seen) in `tests/test_feedback_rate.py`: `test_idle_selects_the_throttled_topic`, `test_any_active_source_selects_full_rate`, `test_return_to_idle_selects_throttled_again`, `test_silent_idle_topic_falls_back_to_full_rate`, `test_empty_idle_topic_keeps_full_rate`; in root `tests/test_client_cpu_load.py`: `test_filter_node_idle_feedback_is_throttled`.
- `nodes/filter_node/README.md` (config table and the idle/active behaviour) and `tests/README.md` are updated.

## Acceptance conditions

- a1: idle selects the throttled feedback topic [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_idle_selects_the_throttled_topic]
- a2: leader, web UI or autonomy activity selects the full-rate topic [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_any_active_source_selects_full_rate]
- a3: returning to idle selects the throttled topic again [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_return_to_idle_selects_throttled_again]
- a4: a silent throttled topic falls back to full rate [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_silent_idle_topic_falls_back_to_full_rate]
- a5: an empty idle topic keeps today's full-rate behaviour [evidence: test:nodes/filter_node/tests/test_feedback_rate.py::test_empty_idle_topic_keeps_full_rate]
- a6: the client config sets the throttled idle topic [evidence: test:tests/test_client_cpu_load.py::test_filter_node_idle_feedback_is_throttled]
- a7: the subscription swap runs from the main loop, never inside a callback [evidence: judgement:]
- a8: filter_node tests and lint, root tests and ansible lint pass [evidence: command:cd nodes/filter_node && uv run --frozen pytest tests -q && uv run poe lint && cd ../.. && uv run pytest tests/test_client_cpu_load.py -q && uv run poe test-ansible]

## Files

- `nodes/filter_node/filter_node/config.py`
- `nodes/filter_node/filter_node/feedback_rate.py`
- `nodes/filter_node/filter_node/node.py`
- `nodes/filter_node/tests/test_feedback_rate.py`
- `nodes/filter_node/README.md`
- `ansible/group_vars/client.yml`
- `tests/test_client_cpu_load.py`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.
A retirement test asserts on `git ls-files <path>`, never on the filesystem.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
