---
title: Client chrony role
status: todo
repo: ros2-lerobot-sverve-platform
value: "The client keeps chrony on plain internet pools from its own role, independent of the Jetson link."
files: [ansible/roles/chrony/, ansible/playbooks/client.yml, tests/test_chrony_role.py, ansible/README.md, tests/README.md]
verify: "uv run pytest tests/test_chrony_role.py -q && uv run poe lint && uv run poe test-ansible"
depends_on: [drop-monitoring-host-targets]
owns: [chrony-role]
uses: []
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# Client chrony role

## Requirement

- New role `ansible/roles/chrony`: `defaults/main.yml` with `chrony_ntp_pools` (the same pools as `jetson_link_ntp_pools` in `roles/jetson_link/defaults/main.yml`); `tasks/main.yml` that gathers service facts, stops and disables `systemd-timesyncd` only when that unit exists, installs `chrony`, templates `/etc/chrony/chrony.conf` from `templates/chrony.conf.j2` (registered), enables and starts `chrony`, and restarts it only when the template task changed.
- `chrony.conf.j2` renders one `pool <pool> iburst` line per pool, `driftfile /var/lib/chrony/chrony.drift`, `ntsdumpdir /var/lib/chrony`, `logdir /var/log/chrony`, `rtcsync` and `makestep 1.0 3`; it has no `allow`, no `local stratum`, no `server` line and no 10.77 address.
- `ansible/playbooks/client.yml` (client provisioning) applies the `chrony` role.
- `tests/test_chrony_role.py`, written first: renders the template with the defaults (jinja2) and asserts the lines above and the absent directives; asserts the timesyncd tasks are guarded on the unit's presence and the restart is gated on the template result; asserts `playbooks/client.yml` lists the role.
- `ansible/README.md` documents client time sync (chrony role, internet pools); `tests/README.md` lists the tests.

## Acceptance conditions

- a1: the rendered chrony.conf has the pools and base directives and no allow, local stratum, server or 10.77 line [evidence: test: tests/test_chrony_role.py::test_chrony_conf_is_plain_pools]
- a2: timesyncd is stopped only when present and chrony restarts only on a config change [evidence: test: tests/test_chrony_role.py::test_tasks_guard_timesyncd_and_restart_on_change]
- a3: client provisioning applies the chrony role [evidence: test: tests/test_chrony_role.py::test_client_provisioning_applies_chrony]
- a4: test-ansible passes with the new role [evidence: command: uv run poe test-ansible]
- a5: ansible/README.md documents the chrony role and tests/README.md lists its tests [evidence: judgement]

## Files

- `ansible/roles/chrony/`
- `ansible/playbooks/client.yml`
- `tests/test_chrony_role.py`
- `ansible/README.md`
- `tests/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes

Context the implementer needs: pitfalls, shared parts to reuse, decisions already made.
