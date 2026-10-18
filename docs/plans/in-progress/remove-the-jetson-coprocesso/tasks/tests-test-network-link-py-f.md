---
title: "tests/test_network_link.py fails 2 tests (test_all_yml_defines_the_link_vars_and_names_d8 and test_all_yml_keys_the_link_by_group_membership; they read jetson_link_* vars and the templated network_link_ethernets from all.yml)."
status: done
repo: ros2-lerobot-sverve-platform
value: "This task's requirement removes those vars from all.yml. The file belongs to drop-network-link-support, which deletes it. Integration must merge both tasks, or the full suite fails on this file."
files: ["tests/test_network_link.py:180-215"]
verify: "! test -e tests/test_network_link.py"
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: "b0046dacf288a7401f9613948f21ef54909d8aff"
run: ""
provides: []
---

# tests/test_network_link.py fails 2 tests (test_all_yml_defines_the_link_vars_and_names_d8 and test_all_yml_keys_the_link_by_group_membership; they read jetson_link_* vars and the templated network_link_ethernets from all.yml).

## Requirement

tests/test_network_link.py fails 2 tests (test_all_yml_defines_the_link_vars_and_names_d8 and test_all_yml_keys_the_link_by_group_membership; they read jetson_link_* vars and the templated network_link_ethernets from all.yml).

Where: tests/test_network_link.py:180-215

Why: This task's requirement removes those vars from all.yml. The file belongs to drop-network-link-support, which deletes it. Integration must merge both tasks, or the full suite fails on this file.

## Acceptance conditions

- a1: tests/test_network_link.py no longer exists on main (deleted by drop-network-link-support) and the root suite passes [evidence: command: ! test -e tests/test_network_link.py]


## Files

- `tests/test_network_link.py:180-215`

## Interfaces

- None.

## Expected outcome

- The root suite has no failing test_network_link tests.

## Steps

- `! test -e tests/test_network_link.py`: exit 0. Resolved by the merge of drop-network-link-support; no code change.

## Notes


Discovered in task `remove-jetson-link-on-client` during run `remove-the-jetson-coprocesso`.


## Resolution

No change needed (orchestrator ruling 2026-10-18): drop-network-link-support deleted tests/test_network_link.py and both tasks are merged; main runs 1117 passed.
