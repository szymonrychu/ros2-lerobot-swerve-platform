---
title: Drop depth_wire and zstd
status: in-progress
repo: ros2-lerobot-sverve-platform
value: ros2_common_cpp loses the Jetson depth wire format and its zstd dependency.
files: [shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp, shared/ros2_common_cpp/src/depth_wire.cpp, shared/ros2_common_cpp/test/test_depth_wire.cpp, shared/ros2_common_cpp/CMakeLists.txt, shared/ros2_common_cpp/package.xml, ansible/group_vars/client.yml, shared/README.md, tests/test_jetson_retired.py]
verify: "uv run pytest tests/test_jetson_retired.py tests/test_cpp_node_deploy.py tests/test_cpp_scaffolding.py -q && uv run poe lint && ./scripts/cpp-node-test.sh ros2_common_cpp && ./scripts/cpp-node-test.sh map_guard_cpp"
depends_on: [delete-jetson-bridge-sources]
owns: []
uses: [jetson-retired-test]
model: sonnet
estimate: 50
actual: ""
review: ""
worktree: /Users/szymonri/Documents/ros2-lerobot-sverve-platform/.claude/worktrees/remove-the-jetson-coprocesso--drop-depth-wire-and-zstd
branch: remove-the-jetson-coprocesso--drop-depth-wire-and-zstd
verified_head: ""
run: ""
provides: []
---

# Drop depth_wire and zstd

## Requirement

- Extend `tests/test_jetson_retired.py` first (see it fail): `REMOVED_PATHS` += the three depth_wire files; `ABSENT_TOKENS` += (`shared/ros2_common_cpp/CMakeLists.txt`, `zstd`), (`shared/ros2_common_cpp/CMakeLists.txt`, `depth_wire`), (`shared/ros2_common_cpp/package.xml`, `zstd`), (`ansible/group_vars/client.yml`, `libzstd-dev`), (`shared/README.md`, `depth_wire`), (`shared/README.md`, `depthd`).
- Delete `depth_wire.hpp`, `src/depth_wire.cpp` and `test/test_depth_wire.cpp`. `CMakeLists.txt` drops the zstd find (lines ~15-16), the `depth_wire` library target and its install (~35-46) and its gtest (~64-65); `package.xml` drops `libzstd-dev`; the `map_guard_cpp` `apt_packages` list in `client.yml` drops `libzstd-dev`.
- `shared/README.md`: the depth wire section (~166-191) and its other mentions (~86, ~206) are removed.
- `ros2_common_cpp` builds and its remaining tests pass, and `map_guard_cpp` builds and its tests pass, both through `scripts/cpp-node-test.sh` in docker.
- `tests/test_cpp_node_deploy.py::test_cpp_node_apt_packages_cover_every_in_tree_package_xml` passes.

## Acceptance conditions

- a1: the depth_wire files are gone [evidence: test: tests/test_jetson_retired.py::test_removed_path_is_gone]
- a2: no zstd or depth_wire token remains in CMakeLists, package.xml, client.yml or shared/README.md [evidence: test: tests/test_jetson_retired.py::test_removed_token_is_absent]
- a3: ros2_common_cpp builds and tests green in docker [evidence: command: ./scripts/cpp-node-test.sh ros2_common_cpp]
- a4: map_guard_cpp builds and tests green in docker [evidence: command: ./scripts/cpp-node-test.sh map_guard_cpp]
- a5: apt lists still cover every in-tree package.xml [evidence: test: tests/test_cpp_node_deploy.py::test_cpp_node_apt_packages_cover_every_in_tree_package_xml]

## Files

- `shared/ros2_common_cpp/include/ros2_common_cpp/depth_wire.hpp`
- `shared/ros2_common_cpp/src/depth_wire.cpp`
- `shared/ros2_common_cpp/test/test_depth_wire.cpp`
- `shared/ros2_common_cpp/CMakeLists.txt`
- `shared/ros2_common_cpp/package.xml`
- `ansible/group_vars/client.yml`
- `shared/README.md`
- `tests/test_jetson_retired.py`

## Interfaces


- `ros2_common_cpp` CMake exports keep `config` and `metrics` only (map_guard_cpp links these).

## Expected outcome


ros2_common_cpp has no depth_wire target and no zstd dependency; map_guard_cpp still builds; the client stamp change makes map_guard_cpp rebuild on the next deploy.

## Steps


- Extend the lists; run `uv run pytest tests/test_jetson_retired.py -q` and see the new cases fail.
- Delete the files; edit CMakeLists.txt, package.xml, client.yml and shared/README.md.
- Run the verify command (docker builds); commit `refactor(ros2_common_cpp): drop depth_wire and zstd`.

## Notes


- Only `jetson/depthd` and `nodes/jetson_bridge_cpp` used depth_wire; both are deleted by earlier tasks.
- The `metrics.hpp` usage comment (stereo example names) is reworded by final-suite-and-grep.
