---
title: Alloy scrape interval 15 s
status: done
repo: ros2-lerobot-sverve-platform
value: "The client's Alloy and the ~18 Python /metrics handlers it scrapes cost a third (about 0.06 core) while dashboards stay correct."
files: [ansible/group_vars/client.yml, ansible/roles/monitoring/defaults/main.yml, tests/test_monitoring_agent_role.py, tests/test_monitoring_role.py, tests/test_monitoring_dashboards.py, ansible/README.md, shared/ros2_metrics/ros2_metrics/__init__.py, tests/README.md]
verify: "uv run pytest tests/test_monitoring_agent_role.py tests/test_monitoring_role.py tests/test_monitoring_dashboards.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [filter-node-idle-feedback]
owns: []
uses: []
model: ""
estimate: 60
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/client-cpu-budget-recovery-a--alloy-scrape-interval-15-s
branch: "current worktree branch (HEAD c8e4973a)"
verified_head: c8e4973abdfca78a66f745ed9275884b7269242a
run: client-cpu-budget-recovery-a
provides: []
acs_for_audit: []
process_gaps: []
---

# Alloy scrape interval 15 s

## Requirement

Item 8 of the brief (design D8; approval A8): the client's Alloy scrapes every 15 s instead of 5 s, and nothing in Grafana reads a rate window shorter than 4 scrape intervals (60 s).

- `ansible/group_vars/client.yml`: `monitoring_agent_interval: "15s"` with a comment (alloy 0.12 core, 40% sys from cAdvisor; also ~18 Python /metrics handlers scraped 3x less). `monitoring_agent_scrape_timeout` stays 4s (below the interval). The role default (`roles/monitoring_agent/defaults/main.yml`, 5s) and the server stay at 5 s.
- `ansible/roles/monitoring/defaults/main.yml`: `monitoring_interval: "15s"`, the Grafana Mimir datasource `timeInterval`, so `$__rate_interval` is at least 60 s for client series (my call in design D8: the datasource takes the slowest scrape interval).
- Dashboards (`ansible/roles/monitoring/files/*.json`) and any Mimir ruler/alert rule files: every PromQL range selector is `$__rate_interval`, `$__auto`, or a literal of at least 60 s; widen any literal below 60 s (today's scan found only `[1m]` and longer).
- Tests first (red seen): `tests/test_monitoring_agent_role.py::test_client_scrapes_every_15_s` (client group_vars interval 15s, timeout below it), `tests/test_monitoring_role.py::test_datasource_time_interval_is_the_slowest_scrape` (replaces the `monitoring_interval == "5s"` pin), `tests/test_monitoring_dashboards.py::test_no_rate_window_below_four_client_scrapes`.
- Docs that say every 5 s for the client: `ansible/README.md` (Alloy row, retention paragraph, node targets paragraph) and the `shared/ros2_metrics/ros2_metrics/__init__.py` module docstring; `tests/README.md` updated.

## Acceptance conditions

- a1: the client's Alloy scrapes every 15 s with the timeout below the interval [evidence: test:tests/test_monitoring_agent_role.py::test_client_scrapes_every_15_s]
- a2: the Grafana datasource time interval is 15 s [evidence: test:tests/test_monitoring_role.py::test_datasource_time_interval_is_the_slowest_scrape]
- a3: no dashboard or rule uses a rate window below 60 s [evidence: test:tests/test_monitoring_dashboards.py::test_no_rate_window_below_four_client_scrapes]
- a4: root tests, lint and ansible lint pass [evidence: command:uv run pytest tests/test_monitoring_agent_role.py tests/test_monitoring_role.py tests/test_monitoring_dashboards.py -q && uv run poe lint && uv run poe test-ansible]

## Files

- `ansible/group_vars/client.yml`
- `ansible/roles/monitoring/defaults/main.yml`
- `tests/test_monitoring_agent_role.py`
- `tests/test_monitoring_role.py`
- `tests/test_monitoring_dashboards.py`
- `ansible/README.md`
- `shared/ros2_metrics/ros2_metrics/__init__.py`
- `tests/README.md`

## Interfaces

`monitoring_agent_interval` (client group_vars override; role default `ansible/roles/monitoring_agent/defaults/main.yml` stays 5s), `monitoring_interval` (`ansible/roles/monitoring/defaults/main.yml`, rendered as the Mimir datasource `timeInterval` in templates/grafana-datasources.yaml.j2). Keep `monitoring_agent_scrape_timeout: 4s` and the server's 5 s.

## Expected outcome

The rendered client Alloy config scrapes every 15 s; Grafana's `$__rate_interval` is at least 60 s; no dashboard panel goes empty; the listed tests pass.

## Steps

- Add the three tests (and replace the 5s datasource pin); `uv run pytest tests/test_monitoring_agent_role.py tests/test_monitoring_role.py tests/test_monitoring_dashboards.py -q` FAILS.
- Set the client interval and monitoring_interval; widen any window below 60 s; rerun, PASS.
- Fix the 5 s statements in ansible/README.md and the ros2_metrics docstring; `uv run poe lint && uv run poe test-ansible`; update tests/README.md.
- Commit `perf(monitoring): client Alloy scrapes every 15 s`.

## Notes

Evidence (brief item 8): alloy 0.12 core on the client, 40% of it sys time from cAdvisor; expected -0.06 core. Today's dashboard scan: [1m] is the shortest literal window (robot-slam.json), the rest are $__rate_interval/$__auto or 5m and longer. Editing shared/ros2_metrics changes the shared/ hash, so every src_shared node restarts at the next deploy; the runbook deploys the whole client anyway. Deploy is `./scripts/deploy-nodes.sh client monitoring` and `./scripts/deploy-nodes.sh server monitoring` (in the runbook). Cross-epic order: ToF depth fix and ground noise robustness (tof-depth-fix-and-ground-noi) edits `ansible/group_vars/client.yml` (head-pan-inverted, tof-calibrated-k-deploy, ground-temporal-wiring) and merged into main first (a072aef8, 2026-10-21); base on main and touch only the keys named here.
