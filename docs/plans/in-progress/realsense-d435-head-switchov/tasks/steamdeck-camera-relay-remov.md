---
title: Steamdeck camera relay removed
status: in-progress
repo: ros2-lerobot-sverve-platform
value: The legacy Steam Deck UI stops waiting for a camera that is gone.
files: [nodes/steamdeck_ui/bridge/bridge_server.py, nodes/steamdeck_ui/config/default.yaml, ansible/roles/steamdeck_ui/templates/config.yaml.j2, nodes/steamdeck_ui/tests/test_config.test.ts, nodes/steamdeck_ui/README.md]
verify: "cd nodes/steamdeck_ui && npx jest tests/test_config.test.ts && cd ../.. && uv run poe test-ansible"
depends_on: []
model: sonnet
review: ""
verified_head: ""
points: 2
expected_minutes: 23
expected_tokens: 1500000
points_actual: ""
actual_minutes: ""
actual_tokens: ""
actual_roles: ""
actual_role_tokens: ""
---

# Steamdeck camera relay removed

## Requirement

Goal G9: "... relays removed or re-pointed". User decision 4: retire the "master2master steamdeck relay" of the gripper camera. Design D11.

- `nodes/steamdeck_ui/bridge/bridge_server.py`: the `/controller/camera_0/image_compressed` subscription and its forwarding are removed. `nodes/steamdeck_ui/config/default.yaml` and `ansible/roles/steamdeck_ui/templates/config.yaml.j2` drop the camera topic and any camera view config. The frontend camera view, if it only shows that topic, is removed from `src/` config (README says what changed).
- `nodes/steamdeck_ui/tests/test_config.test.ts`: the camera_0 assertions are no longer accurate and are removed; a new case asserts that no config names `camera_0`.
- `nodes/steamdeck_ui/README.md` updated.
The client.yml master2master relay entry is removed by retire-gripper-camera-in-ans.

## Acceptance conditions

- AC1: the steamdeck bridge and configs no longer name the gripper camera relay [evidence: command:! grep -rn "camera_0" nodes/steamdeck_ui/bridge nodes/steamdeck_ui/config ansible/roles/steamdeck_ui]
- AC2: steamdeck_ui tests pass [evidence: command:cd nodes/steamdeck_ui && npx jest tests/test_config.test.ts]
- AC3: Ansible checks pass [evidence: command:uv run poe test-ansible]

## Files

- `nodes/steamdeck_ui/bridge/bridge_server.py`
- `nodes/steamdeck_ui/config/default.yaml`
- `ansible/roles/steamdeck_ui/templates/config.yaml.j2`
- `nodes/steamdeck_ui/tests/test_config.test.ts`
- `nodes/steamdeck_ui/README.md`

## Interfaces

Signatures, schemas and contracts this task builds or must keep, with file:symbol.

## Expected outcome

What is true and observable when the task is done.

## Steps

- Update `tests/test_config.test.ts`; run `cd nodes/steamdeck_ui && npx jest tests/test_config.test.ts`; expect a failure.
- Remove the subscription and config entries; rerun; expect pass; run `./scripts/test_steamdeck_ui.sh` if it covers the bridge.
- Run the verify command; commit `refactor(steamdeck_ui): drop the retired gripper camera relay`.

## Notes

Context the implementer needs: pitfalls, existing code to reuse, decisions already made.
