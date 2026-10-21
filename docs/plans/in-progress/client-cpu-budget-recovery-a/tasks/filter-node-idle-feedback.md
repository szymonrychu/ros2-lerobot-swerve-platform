---
title: Filter node idle feedback
status: in-progress
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
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/client-cpu-budget-recovery-a--filter-node-idle-feedback
branch: client-cpu-budget-recovery-a--filter-node-idle-feedback
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

`filter_node/feedback_rate.py::FeedbackRateSwitch` (new, public): constructed from the config (full topic, idle topic, idle_timeout_s, web_ui_timeout_s, fallback_s); a method returning the topic that should be subscribed now from the last input times, the autonomy lease flag and the last idle-feedback time. Config keys `follower_feedback_idle_topic` and `follower_feedback_idle_fallback_s`. Keep `Arbiter.update_follower_positions`, the takeover threshold, the released-arm handling and every published topic.

## Expected outcome

With the client config, filter_node subscribes `/follower/joint_states/throttled` while nobody commands the arm and `/follower/joint_states` within one control period of the first command; all tests pass.

## Steps

- Write tests/test_feedback_rate.py and the root pin; `cd nodes/filter_node && uv run --frozen pytest tests/test_feedback_rate.py -q` FAILS (no module).
- Add the config keys, FeedbackRateSwitch, and the swap in node.py's main loop; rerun, PASS; run the package suite.
- Set the idle topic in client.yml; `uv run pytest tests/test_client_cpu_load.py -q` PASS; `uv run poe lint` in the node and at the root; `uv run poe test-ansible`.
- Update nodes/filter_node/README.md and tests/README.md.
- Commit `perf(filter_node): throttled arm feedback while idle`.

## Notes

Evidence (brief item 5): the follower feedback runs at 89 Hz although it is only used for the takeover proximity rule; expected -0.08 core. The throttled copy comes from joint_state_throttle_follower (topic_tools throttle, 10 Hz, background tier). Prior art: mcp_server's dual-rate joint states (nodes/mcp_server/mcp_server/config.py ~1591: throttled copy while idle, fallback to full rate when the copy is silent, logged once). Swapping a subscription inside a callback raced the executor wait set in mcp_server (InvalidHandle, 2026-10-08): swap only from the main loop between spin_once calls. Depends on filter-node-events-executor (same files) and client-executor-config (client.yml order). Servo safety: no arm motion in this task. Cross-epic order: ToF depth fix and ground noise robustness (tof-depth-fix-and-ground-noi) edits `ansible/group_vars/client.yml` (head-pan-inverted, tof-calibrated-k-deploy, ground-temporal-wiring) and merged into main first (a072aef8, 2026-10-21); base on main and touch only the keys named here.
