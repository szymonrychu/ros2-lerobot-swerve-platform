---
title: "tests/test_network_link.py fails 2 tests (test_all_yml_defines_the_link_vars_and_names_d8 and test_all_yml_keys_the_link_by_group_membership; they read jetson_link_* vars and the templated network_link_ethernets from all.yml)."
status: todo
repo: ros2-lerobot-sverve-platform
value: "This task's requirement removes those vars from all.yml. The file belongs to drop-network-link-support, which deletes it. Integration must merge both tasks, or the full suite fails on this file."
files: ["tests/test_network_link.py:180-215"]
verify: ""
depends_on: []
owns: []
uses: []
model: ""
estimate: ""
actual: ""
review: ""
worktree: ""
branch: ""
verified_head: ""
run: ""
provides: []
---

# tests/test_network_link.py fails 2 tests (test_all_yml_defines_the_link_vars_and_names_d8 and test_all_yml_keys_the_link_by_group_membership; they read jetson_link_* vars and the templated network_link_ethernets from all.yml).

## Requirement

tests/test_network_link.py fails 2 tests (test_all_yml_defines_the_link_vars_and_names_d8 and test_all_yml_keys_the_link_by_group_membership; they read jetson_link_* vars and the templated network_link_ethernets from all.yml).

Where: tests/test_network_link.py:180-215

Why: This task's requirement removes those vars from all.yml. The file belongs to drop-network-link-support, which deletes it. Integration must merge both tasks, or the full suite fails on this file.

## Acceptance conditions



## Files

- `tests/test_network_link.py:180-215`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

Test first. One step per bullet, with the exact command and the output to expect.

## Notes


Discovered in task `remove-jetson-link-on-client` during run `remove-the-jetson-coprocesso`.

