---
title: Zoom request JSON
status: todo
repo: ros2-lerobot-sverve-platform
value: "One authoritative zoom request/result contract shared by the node, mcp_server and the web UI."
files: [nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp, nodes/head_camera_cpp/src/core/zoom_json.cpp, nodes/head_camera_cpp/test/core/test_zoom_json.cpp]
verify: scripts/cpp-node-test.sh head_camera_cpp
depends_on: [head-camera-node-config]
owns: [zoom-json-contract]
uses: [head-camera-config]
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

# Zoom request JSON

## Requirement

- `zoom_json.hpp` / `src/core/zoom_json.cpp` with nlohmann-json. The header comment is the D4 contract, verbatim in substance: the three topics (`/head_camera/zoom/command`, `/zoom/image`, `/zoom/result`), every request field with range and default, every result field, the status list (`ok`, `af_failed`, `busy`, `timeout`, `camera_error`, `invalid`), which statuses carry an image (`ok`, `af_failed`), that the image is published before the result and that `stamp_ns` equals the image header stamp.
- `struct ZoomRequest {std::string request_id; double u; double v; int zoom; int max_px; bool af;}`; `struct ParseError {std::optional<std::string> request_id; std::string message;}`; `std::variant<ZoomRequest, ParseError> parse_request(const std::string & json, const ZoomConfig & limits)`: u, v numbers in [0, 1]; zoom an integer in `limits.factors`; max_px integer in [min_max_px, max_max_px], default `default_max_px`; af bool default true; unknown keys rejected; a missing, empty or non-string request_id gives a ParseError without id (the node logs and drops it).
- `enum class ZoomStatus`, `std::string to_string(ZoomStatus)`, `bool carries_image(ZoomStatus)`; `struct ZoomResult {request_id, status, message, af_state, std::optional<double> lens_position, std::optional<std::array<int,4>> crop, std::array<int,2> full, std::optional<std::array<int,2>> out, int64_t stamp_ns, PhaseMs phase_ms}`; `std::string to_json(const ZoomResult &)` writing exactly the D4 keys (`stamp_s` = stamp_ns / 1e9; absent optionals written as null); `ZoomResult invalid_result(const ParseError &)` for errors that carry an id.
- Tests first in `test/core/test_zoom_json.cpp`, seen failing: a full valid request; defaults (max_px 1024, af true); each invalid field (u -0.1 and 1.1, zoom 3, max_px 100 and 2000, af "yes", extra key, not JSON, not an object) gives a ParseError keeping the request_id when present; missing/empty/non-string request_id gives no id; `to_json` of ok and of busy results has exactly the D4 keys with the right types (stamp_ns an integer, crop null on busy); `carries_image` true only for ok and af_failed.

## Acceptance conditions

- a1: valid requests parse with the D4 defaults and every invalid field is rejected keeping the request_id [evidence: command: cd nodes/head_camera_cpp && uv run --frozen pytest test/core/test_zoom_json.cpp -q]
- a2: results serialise exactly the D4 keys and only ok/af_failed carry an image [evidence: command: cd nodes/head_camera_cpp && uv run --frozen pytest test/core/test_zoom_json.cpp -q]
- a3: the header comment states the full topic and JSON contract [evidence: command: grep -q 'zoom/command' nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp && grep -q 'stamp_ns' nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp && grep -q 'af_failed' nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp]
- a4: package gtests and linters pass [evidence: command: scripts/cpp-node-test.sh head_camera_cpp]

## Files

- `nodes/head_camera_cpp/include/head_camera_cpp/zoom_json.hpp`
- `nodes/head_camera_cpp/src/core/zoom_json.cpp`
- `nodes/head_camera_cpp/test/core/test_zoom_json.cpp`

## Interfaces

- `zoom-json-contract`: `include/head_camera_cpp/zoom_json.hpp` (`ZoomRequest`, `ParseError`, `parse_request`, `ZoomStatus`, `ZoomResult`, `PhaseMs`, `to_json`, `invalid_result`, `carries_image`). mcp_server and web_ui implement the same JSON from this header.

## Expected outcome

The zoom wire contract exists once, tested, and is readable by the Python consumers.

## Steps

- Write `test/core/test_zoom_json.cpp`; run `scripts/cpp-node-test.sh head_camera_cpp`, see it fail.
- Implement; rerun until PASS; commit `feat(head_camera_cpp): zoom request and result JSON contract`.

## Notes

- Field name `request_id` (D4, my call) matches `/poi/command` and `/grasp/command`.
- Crop and full are plain int arrays so this task does not depend on crop.hpp.
