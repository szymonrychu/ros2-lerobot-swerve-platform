---
title: Map guard cpp load hold
status: done
repo: ros2-lerobot-sverve-platform
value: "map_guard cannot unpause or reset SLAM while a loaded map is not yet trusted, so a failed relocalisation never destroys the loaded map."
files: [nodes/map_guard_cpp/include/map_guard_cpp/config.hpp, nodes/map_guard_cpp/src/config.cpp, nodes/map_guard_cpp/include/map_guard_cpp/guard.hpp, nodes/map_guard_cpp/src/guard.cpp, nodes/map_guard_cpp/include/map_guard_cpp/map_guard_node.hpp, nodes/map_guard_cpp/src/map_guard_node.cpp, nodes/map_guard_cpp/test/test_load_hold.cpp, nodes/map_guard_cpp/CMakeLists.txt]
verify: scripts/cpp-node-test.sh map_guard_cpp
depends_on: [map-manager-contract]
owns: []
uses: [map-manager-contract]
model: sonnet
estimate: 75
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/named-maps-with-pois-and-rel--map-guard-cpp-load-hold
branch: named-maps-with-pois-and-rel--map-guard-cpp-load-hold
verified_head: 65ccf2e89990cd3b22da0f9ff7f082fe22c46ce9
run: named-maps-with-pois-and-rel
provides: []
acs_for_audit: [a3]
process_gaps: []
---

# Map guard cpp load hold

## Requirement

map_guard_cpp follows map_manager's localisation verdict (design D9).

- Config key `localised_topic` (string, default `/map_manager/localised`, the `LOCALISED_TOPIC` of
  `shared/ros2_common/maps.py`), validated like the other topic keys; existing configs without it load unchanged.
- The node subscribes to it with reliable + transient_local depth 1 (`std_msgs/Bool`).
- `MapGuard::set_hold(bool held)`: while held the guard tick does nothing (no state change, no pause toggle, no
  serialize, no reset, no reset publish, no events) and the node drops agreement samples; on held -> released the
  mismatch monitor is reset (re-armed only once the new map is healthy, as after a map reset) and the state returns to
  OK. With no message ever received the guard behaves exactly as today.
- Log lines: `map_guard: holding while the loaded map is not localised` once on hold and
  `map_guard: loaded map localised, guard re-armed` once on release.
- `test/test_load_hold.cpp` (gtest, test first, registered in CMakeLists.txt): held guard ignores lifted / mismatch
  triggers and makes no slam calls; release re-arms (no mismatch trigger until healthy samples); no message = unchanged
  behaviour (one golden scenario re-run); the config default and an explicit value.
- Existing gtests and golden fixtures stay green; ament lint passes.

## Acceptance conditions

- a1: the C++ package builds, all gtests including test_load_hold pass and ament lint is clean [evidence: command: scripts/cpp-node-test.sh map_guard_cpp]
- a2: MapGuard has set_hold [evidence: symbol: set_hold]
- a3: held guard makes no slam_toolbox call on lifted or mismatch [evidence: judgement]

## Files

- `nodes/map_guard_cpp/include/map_guard_cpp/config.hpp`
- `nodes/map_guard_cpp/src/config.cpp`
- `nodes/map_guard_cpp/include/map_guard_cpp/guard.hpp`
- `nodes/map_guard_cpp/src/guard.cpp`
- `nodes/map_guard_cpp/include/map_guard_cpp/map_guard_node.hpp`
- `nodes/map_guard_cpp/src/map_guard_node.cpp`
- `nodes/map_guard_cpp/test/test_load_hold.cpp`
- `nodes/map_guard_cpp/CMakeLists.txt`

## Interfaces

- `MapGuardConfig::localised_topic`; `MapGuard::set_hold(bool)`; `MapGuardNode` Bool subscription.

## Expected outcome

With `/map_manager/localised` false, map_guard_cpp makes no slam_toolbox call and publishes nothing; true re-arms it; absent, nothing changes.

## Steps

- Write the test(s) named in the requirement first; run `scripts/cpp-node-test.sh map_guard_cpp` and see it fail for the missing behaviour.
- Implement the smallest change that makes it pass, then refactor with the test green.
- Run the verify command `scripts/cpp-node-test.sh map_guard_cpp`; expect every test passed and lint clean.
- Commit: `feat(map_guard_cpp): hold the guard while a loaded map is not localised`.

## Notes

The golden-fixture gtests compare with the Python node; the hold is C++-only and off unless a message arrives, so existing fixtures stay valid. The Python map_guard (disabled rollback) is not changed; the docs task records that.
- Servo safety (CLAUDE.md): this task moves no servo.
